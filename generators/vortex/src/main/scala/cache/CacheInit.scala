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
import VortexGPUPkg._

/**
 * CacheInit – cache flush/init sequencer that sits in front of the
 * core-request input.
 *
 * Mirrors VX_cache_init.sv exactly.
 *
 * When a flush request arrives on any core bus input, this module:
 *   1. (optionally) waits for in-flight requests to drain (STATE_WAIT1)
 *   2. asserts flush_begin for one cycle to all banks (STATE_FLUSH)
 *   3. waits for all banks to assert flush_end (STATE_WAIT2)
 *   4. releases the originating flush request(s) to the output (STATE_DONE)
 *
 * SV STATE_IDLE=0, STATE_WAIT1=1, STATE_FLUSH=2, STATE_WAIT2=3, STATE_DONE=4
 *
 * VX_pending_size tracks in-flight requests between core-bus output and
 * bank pipeline; it uses saturating arithmetic (clamped increment/decrement).
 *
 * MEM_REQ_FLAG_FLUSH is the flush flag bit position in the flags field.
 */
class CacheInit(
  numReqs:        Int,
  numBanks:       Int,
  tagWidth:       Int,
  uuidWidth:      Int = 1,
  bankSelLatency: Int = 1,
  dataSize:       Int = 4,
  addrWidth:      Int = 28,
  flagsWidth:     Int = 3
) extends Module {

  val io = IO(new Bundle {
    val core_bus_in  = Vec(numReqs, Flipped(new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth)))
    val core_bus_out = Vec(numReqs, new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
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

  val state         = RegInit(STATE_IDLE)
  val flush_done    = RegInit(0.U(numBanks.W))
  val lock_released = RegInit(0.U(numReqs.W))
  val flush_uuid_r  = RegInit(0.U(math.max(1, uuidWidth).W))

  // ---- detect flush request -----------------------------------------------
  // SV: assign flush_req_mask[i] = core_bus_in_if[i].req_valid &&
  //                                 core_bus_in_if[i].req_data.flags[MEM_REQ_FLAG_FLUSH];
  val flush_req_mask = VecInit((0 until numReqs).map { i =>
    io.core_bus_in(i).req.valid && io.core_bus_in(i).req.bits.flags(MEM_REQ_FLAG_FLUSH)
  }).asUInt
  val flush_req_enable = flush_req_mask.orR

  // ---- in-flight request counter -------------------------------------------
  // SV: if (BANK_SEL_LATENCY != 0) instantiate VX_pending_size
  //     else                        assign no_inflight_reqs = 0
  val no_inflight_reqs = Wire(Bool())

  if (bankSelLatency != 0) {
    // SV: incr = core_bus_out_cnt (pop-count of output fires)
    //     decr = bank_req_cnt    (pop-count of bank arrivals)
    // VX_pending_size uses saturating arithmetic; SIZE = BANK_SEL_LATENCY * NUM_BANKS.
    val maxPending = bankSelLatency * numBanks
    val cntWidth   = log2Ceil(maxPending + 1) + 1   // extra bit to detect overflow

    val core_bus_out_fire = VecInit((0 until numReqs).map { i =>
      io.core_bus_out(i).req.valid && io.core_bus_out(i).req.ready
    }).asUInt
    val outCnt  = PopCount(core_bus_out_fire)
    val bankCnt = PopCount(io.bank_req_fire)

    // Saturating counter: clamp to [0, maxPending].
    val pending = RegInit(0.U(cntWidth.W))
    val next_p  = pending +& outCnt   // widen to avoid overflow
    val sub_p   = next_p  - bankCnt
    // Saturate: if subtraction would underflow (bankCnt > next_p) → 0
    //           if addition would overflow (next_p > maxPending) → maxPending
    val saturated = Mux(next_p < bankCnt.asUInt,   0.U(cntWidth.W),
                    Mux(sub_p  > maxPending.U,      maxPending.U(cntWidth.W),
                        sub_p))
    pending := saturated
    no_inflight_reqs := (pending === 0.U)
  } else {
    // SV: assign no_inflight_reqs = 0 (note: the SV says 0, not 1!)
    // This means when bankSelLatency==0 we never wait in STATE_WAIT1,
    // but the FSM also never enters STATE_WAIT1 in that case (it goes
    // directly from IDLE→FLUSH when bankSelLatency==0).
    no_inflight_reqs := false.B
  }

  // ---- UUID capture --------------------------------------------------------
  // SV: for (i) core_bus_out_uuid[i] = UUID_WIDTH ? core_bus_in_if[i].req_data.tag.uuid : 0
  val core_bus_out_uuid = Wire(Vec(numReqs, UInt(math.max(1, uuidWidth).W)))
  for (i <- 0 until numReqs) {
    if (uuidWidth != 0) {
      core_bus_out_uuid(i) := io.core_bus_in(i).req.bits.tag.uuid
    } else {
      core_bus_out_uuid(i) := 0.U
    }
  }

  // ---- core bus pass-through (with lock) -----------------------------------
  // SV:
  //   input_enable = ~flush_req_enable || lock_released[i];
  //   core_bus_out.req_valid = core_bus_in.req_valid && input_enable;
  //   core_bus_out.req_data  = core_bus_in.req_data;
  //   core_bus_in.req_ready  = core_bus_out.req_ready && input_enable;
  //   core_bus_in.rsp_*      = core_bus_out.rsp_*;
  for (i <- 0 until numReqs) {
    val input_enable = !flush_req_enable || lock_released(i)
    io.core_bus_out(i).req.valid := io.core_bus_in(i).req.valid && input_enable
    io.core_bus_out(i).req.bits  := io.core_bus_in(i).req.bits
    io.core_bus_in(i).req.ready  := io.core_bus_out(i).req.ready && input_enable

    io.core_bus_in(i).rsp.valid  := io.core_bus_out(i).rsp.valid
    io.core_bus_in(i).rsp.bits   := io.core_bus_out(i).rsp.bits
    io.core_bus_out(i).rsp.ready := io.core_bus_in(i).rsp.ready
  }

  val core_bus_out_ready = VecInit((0 until numReqs).map { i =>
    io.core_bus_out(i).req.ready
  }).asUInt

  // ---- FSM -----------------------------------------------------------------
  val state_n         = WireDefault(state)
  val flush_done_n    = WireDefault(flush_done)
  val lock_released_n = WireDefault(lock_released)
  val flush_uuid_n    = WireDefault(flush_uuid_r)

  switch (state) {
    // STATE_IDLE (default)
    is (STATE_IDLE) {
      when (flush_req_enable) {
        // SV: state_n = (BANK_SEL_LATENCY != 0) ? STATE_WAIT1 : STATE_FLUSH;
        state_n := Mux((bankSelLatency != 0).B, STATE_WAIT1, STATE_FLUSH)
        // Capture UUID: SV iterates i from NUM_REQS-1 downto 0 with if(flush_req_mask[i]),
        // so the LOWEST-index match that fires wins (last assignment in loop wins for
        // decreasing i — the lowest index has the last write).
        // SV code: for (integer i = NUM_REQS-1; i >= 0; --i) if (flush_req_mask[i]) flush_uuid_n = ...
        // This means index 0 has priority (last write wins in a for loop going high→low).
        for (i <- (numReqs - 1) to 0 by -1) {
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
      // Generate a flush request pulse (one cycle), then immediately go to WAIT2
      state_n := STATE_WAIT2
    }
    is (STATE_WAIT2) {
      // SV: flush_done_n = flush_done | flush_end;
      //     if (flush_done_n == {NUM_BANKS{1'b1}}) { state_n=DONE; flush_done_n='0; lock_released_n=flush_req_mask; }
      val next_flush_status = flush_done | io.flush_end
      flush_done_n := next_flush_status
      when (next_flush_status === Fill(numBanks, 1.U)) {
        state_n         := STATE_DONE
        flush_done_n    := 0.U
        lock_released_n := flush_req_mask
      }
    }
    is (STATE_DONE) {
      // SV: lock_released_n = lock_released & ~core_bus_out_ready;
      //     if (lock_released_n == 0) state_n = STATE_IDLE;
      lock_released_n := lock_released & ~core_bus_out_ready
      when ((lock_released & ~core_bus_out_ready) === 0.U) {
        state_n := STATE_IDLE
      }
    }
  }

  // SV sequential block:
  //   if (reset) { state<=IDLE; flush_done<='0; lock_released<='0; }
  //   else       { state<=state_n; flush_done<=flush_done_n; lock_released<=lock_released_n; }
  //   flush_uuid_r <= flush_uuid_n;   // unconditional (no reset)
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
  // SV: assign flush_begin = {NUM_BANKS{state == STATE_FLUSH}};
  io.flush_begin := Fill(numBanks, (state === STATE_FLUSH).asUInt)
  io.flush_uuid  := flush_uuid_r
}
