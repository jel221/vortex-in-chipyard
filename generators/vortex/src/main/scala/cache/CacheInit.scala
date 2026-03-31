// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Chisel translation of VX_cache_init.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * CacheInit – cache flush/init sequencer that sits in front of the
 * core-request input.
 *
 * Mirrors VX_cache_init.sv.
 *
 * When a flush request arrives on any core bus input, this module:
 *   1. (optionally) waits for any in-flight requests to drain
 *   2. asserts flush_begin for one cycle to each bank
 *   3. waits for all banks to assert flush_end
 *   4. releases the originating flush request(s) to the output
 *
 * Parameters:
 *   numReqs          – number of core request lanes (NUM_REQS)
 *   numBanks         – number of cache banks
 *   tagWidth         – core request tag width (includes UUID field)
 *   uuidWidth        – width of the UUID sub-field (may be 0)
 *   bankSelLatency   – pipeline stages between core-bus output and bank
 *                      (BANK_SEL_LATENCY); when 0, in-flight tracking is skipped
 *
 * The MEM_REQ_FLAG_FLUSH bit is bit 0 of the flags field (matches the SV).
 */
class CacheInit(
  numReqs:        Int,
  numBanks:       Int,
  tagWidth:       Int,
  uuidWidth:      Int  = 0,
  bankSelLatency: Int  = 1,
  dataSize:       Int  = 4,   // word size in bytes (for MemBusBundle sizing)
  addrWidth:      Int  = 28,
  flagsWidth:     Int  = 3
) extends Module {

  val io = IO(new Bundle {
    // Core bus – slave side (from requester)
    val core_bus_in  = Vec(numReqs, Flipped(new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth)))
    // Core bus – master side (to cache)
    val core_bus_out = Vec(numReqs, new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
    // Bank interaction
    val bank_req_fire = Input(UInt(numBanks.W))
    val flush_begin   = Output(UInt(numBanks.W))
    val flush_uuid    = Output(UInt(math.max(1, uuidWidth).W))
    val flush_end     = Input(UInt(numBanks.W))
  })

  // ---- FSM states ----------------------------------------------------------
  val STATE_IDLE  = 0.U(3.W)
  val STATE_WAIT1 = 1.U(3.W)
  val STATE_FLUSH = 2.U(3.W)
  val STATE_WAIT2 = 3.U(3.W)
  val STATE_DONE  = 4.U(3.W)

  val state      = RegInit(STATE_IDLE)
  val flush_done = RegInit(0.U(numBanks.W))
  val lock_released = RegInit(0.U(numReqs.W))
  val flush_uuid_r  = RegInit(0.U(math.max(1, uuidWidth).W))

  // ---- detect flush request -----------------------------------------------
  // MEM_REQ_FLAG_FLUSH is flags bit 0
  val flush_req_mask = Wire(UInt(numReqs.W))
  flush_req_mask := VecInit((0 until numReqs).map { i =>
    io.core_bus_in(i).req.valid && io.core_bus_in(i).req.bits.flags(0)
  }).asUInt
  val flush_req_enable = flush_req_mask.orR

  // ---- in-flight request counter (only when bankSelLatency != 0) -----------
  val no_inflight_reqs = Wire(Bool())
  if (bankSelLatency != 0) {
    // Count requests that fire out vs those that arrive at the bank.
    val outCnt  = PopCount(VecInit((0 until numReqs).map { i =>
      io.core_bus_out(i).req.valid && io.core_bus_out(i).req.ready
    }).asUInt)
    val bankCnt = PopCount(io.bank_req_fire)

    // Pending size tracker: incremented by outCnt, decremented by bankCnt.
    // Max pending = bankSelLatency * numBanks
    val maxPending = bankSelLatency * numBanks
    val pending    = RegInit(0.U(log2Ceil(maxPending + 1).W))
    pending := pending + outCnt - bankCnt
    no_inflight_reqs := (pending === 0.U)
  } else {
    no_inflight_reqs := false.B
  }

  // ---- UUID capture --------------------------------------------------------
  val core_bus_out_uuid = Wire(Vec(numReqs, UInt(math.max(1, uuidWidth).W)))
  for (i <- 0 until numReqs) {
    if (uuidWidth != 0) {
      core_bus_out_uuid(i) := io.core_bus_in(i).req.bits.tag.uuid
    } else {
      core_bus_out_uuid(i) := 0.U
    }
  }

  // ---- core bus pass-through (with lock) -----------------------------------
  for (i <- 0 until numReqs) {
    val input_enable = !flush_req_enable || lock_released(i)
    io.core_bus_out(i).req.valid := io.core_bus_in(i).req.valid && input_enable
    io.core_bus_out(i).req.bits  := io.core_bus_in(i).req.bits
    io.core_bus_in(i).req.ready  := io.core_bus_out(i).req.ready && input_enable

    // response path pass-through
    io.core_bus_in(i).rsp.valid  := io.core_bus_out(i).rsp.valid
    io.core_bus_in(i).rsp.bits   := io.core_bus_out(i).rsp.bits
    io.core_bus_out(i).rsp.ready := io.core_bus_in(i).rsp.ready
  }

  // ---- FSM -----------------------------------------------------------------
  val state_n         = WireDefault(state)
  val flush_done_n    = WireDefault(flush_done)
  val lock_released_n = WireDefault(lock_released)
  val flush_uuid_n    = WireDefault(flush_uuid_r)

  val core_bus_out_ready = VecInit((0 until numReqs).map { i =>
    io.core_bus_out(i).req.ready
  }).asUInt

  switch (state) {
    is (STATE_IDLE) {
      when (flush_req_enable) {
        state_n := Mux((bankSelLatency != 0).B, STATE_WAIT1, STATE_FLUSH)
        // Capture UUID from highest-index flush requester
        for (i <- 0 until numReqs) {
          when (flush_req_mask(i)) {
            flush_uuid_n := core_bus_out_uuid(i)
          }
        }
      }
    }
    is (STATE_WAIT1) {
      when (no_inflight_reqs) {
        state_n := STATE_FLUSH
      }
    }
    is (STATE_FLUSH) {
      // One-cycle pulse; immediately move to WAIT2
      state_n := STATE_WAIT2
    }
    is (STATE_WAIT2) {
      val next_flush_status = flush_done | io.flush_end

      when (next_flush_status.andR) {
        state_n         := STATE_DONE
        flush_done_n    := 0.U
        lock_released_n := flush_req_mask
      }.otherwise {
        flush_done_n    := next_flush_status
      }
    }
    is (STATE_DONE) {
      lock_released_n := lock_released & ~core_bus_out_ready
      when (lock_released_n === 0.U) {
        state_n := STATE_IDLE
      }
    }
  }

  when (reset.asBool) {
    state         := STATE_IDLE
    flush_done    := 0.U
    lock_released := 0.U
  } .otherwise {
    state         := state_n
    flush_done    := flush_done_n
    lock_released := lock_released_n
  }
  flush_uuid_r := flush_uuid_n

  // ---- outputs -------------------------------------------------------------
  io.flush_begin := Fill(numBanks, (state === STATE_FLUSH).asUInt)
  io.flush_uuid  := flush_uuid_r
}
