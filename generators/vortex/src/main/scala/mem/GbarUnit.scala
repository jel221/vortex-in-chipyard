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

// Chisel translations of:
//   VX_gbar_bus_if.sv – already translated in MemBusBundle.scala (GbarBusBundle)
//   VX_gbar_arb.sv    – arbitrated N-input → 1-output global-barrier bus + broadcast
//   VX_gbar_unit.sv   – global barrier synchronisation unit

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// GbarArb
//
// Translation of VX_gbar_arb.sv.
//
// Arbitrates NUM_REQS global-barrier bus inputs down to a single output
// using a round-robin arbiter.  Responses from the single output are
// registered and broadcast back to ALL input ports (not just the one that
// won the arbitration).
//
// SV behaviour:
//   • Request path: VX_stream_arb (round-robin) selects one of NUM_REQS
//     inputs and forwards its req_data/req_valid to the single output;
//     req_ready is fed back to the winning input only.
//   • Response path: the output's rsp_valid / rsp_data are registered
//     (one pipeline stage) and broadcast to all inputs.
//
// Parameters
//   numReqs  – NUM_REQS
//   nbWidth  – NB_WIDTH = UP(CLOG2(NUM_BARRIERS))
//   ncWidth  – NC_WIDTH = UP(CLOG2(NUM_CORES))
// ---------------------------------------------------------------------------

class GbarArb(
    numReqs: Int,
    nbWidth: Int,
    ncWidth: Int
) extends Module {
  require(numReqs >= 1)

  val io = IO(new Bundle {
    val busIn  = Flipped(Vec(numReqs, new GbarBusBundle(nbWidth, ncWidth)))
    val busOut =         new GbarBusBundle(nbWidth, ncWidth)
  })

  // -----------------------------------------------------------------------
  // Request arbitration path
  // -----------------------------------------------------------------------
  val arb = Module(new RRArbiter(new GbarReqBundle(nbWidth, ncWidth), numReqs))

  for (i <- 0 until numReqs) {
    arb.io.in(i).valid := io.busIn(i).req.valid
    arb.io.in(i).bits  := io.busIn(i).req.bits
    io.busIn(i).req.ready := arb.io.in(i).ready
  }

  io.busOut.req.valid := arb.io.out.valid
  io.busOut.req.bits  := arb.io.out.bits
  arb.io.out.ready    := io.busOut.req.ready

  // -----------------------------------------------------------------------
  // Response broadcast path
  //
  // The SV module registers rsp_valid and rsp_data by one clock cycle before
  // broadcasting.  We replicate that exactly.
  // -----------------------------------------------------------------------
  val rspValidReg = RegNext(io.busOut.rsp.valid, init = false.B)
  val rspDataReg  = RegNext(io.busOut.rsp.bits)

  for (i <- 0 until numReqs) {
    io.busIn(i).rsp.valid := rspValidReg
    io.busIn(i).rsp.bits  := rspDataReg
  }
}

// ---------------------------------------------------------------------------
// GbarUnit
//
// Translation of VX_gbar_unit.sv.
//
// Hardware global barrier synchronisation unit.  Maintains per-barrier
// participation bitmasks.  When the number of cores that have checked in
// equals size_m1 (the threshold), the barrier is released and a response
// is generated.
//
// SV behaviour:
//   1. req_ready is always 1 (the unit is never back-pressured).
//   2. On each cycle:
//      a. If rsp_valid is set, clear it.
//      b. If req_valid is set:
//         • Read active_barrier_count (popcount of barrier_masks[req_id]).
//         • If count == size_m1 → clear the mask, emit response.
//         • Else → set barrier_masks[req_id][core_id].
//
// Note: the SV uses NC_WIDTH bits of `active_barrier_count` for the
// comparison.  We do the same by truncating the popcount to ncWidth bits.
//
// Parameters
//   numBarriers – NUM_BARRIERS (determines nbWidth = UP(CLOG2(NUM_BARRIERS)))
//   numCores    – NUM_CORES    (determines ncWidth = UP(CLOG2(NUM_CORES)))
// ---------------------------------------------------------------------------

class GbarUnit(
    numBarriers: Int,
    numCores:    Int
) extends Module {
  val nbWidth = math.max(1, log2Ceil(math.max(2, numBarriers)))
  val ncWidth = math.max(1, log2Ceil(math.max(2, numCores)))

  val io = IO(new Bundle {
    val gbarBus = Flipped(new GbarBusBundle(nbWidth, ncWidth))
  })

  // Per-barrier participation bitmask: barrier_masks[NUM_BARRIERS][NUM_CORES]
  // SV: reg [NB_WIDTH-1:0][NUM_CORES-1:0] barrier_masks
  // i.e. index 0..NUM_BARRIERS-1, each entry is NUM_CORES bits.
  val numBarriersActual = math.max(1, 1 << nbWidth)
  val barrierMasks = RegInit(VecInit(Seq.fill(numBarriersActual)(0.U(numCores.W))))

  val rspValid  = RegInit(false.B)
  val rspBarId  = Reg(UInt(nbWidth.W))

  // Current barrier mask for the incoming request id.
  val currMask = barrierMasks(io.gbarBus.req.bits.id)

  // Population count of currMask (number of cores already waiting).
  val activeCount = PopCount(currMask)

  // Sequential logic mirroring the SV always @(posedge clk).
  when (rspValid) {
    rspValid := false.B
  }
  when (io.gbarBus.req.valid) {
    // Compare the lower ncWidth bits of the popcount with size_m1.
    when (activeCount(ncWidth - 1, 0) === io.gbarBus.req.bits.size_m1) {
      // All expected cores have arrived: release barrier.
      barrierMasks(io.gbarBus.req.bits.id) := 0.U
      rspBarId  := io.gbarBus.req.bits.id
      rspValid  := true.B
    } .otherwise {
      // Mark this core as having arrived.
      val updated = WireDefault(currMask)
      updated := currMask | (1.U << io.gbarBus.req.bits.core_id)
      barrierMasks(io.gbarBus.req.bits.id) := updated
    }
  }

  // Outputs
  io.gbarBus.rsp.valid    := rspValid
  io.gbarBus.rsp.bits.id  := rspBarId
  // The global barrier unit is always ready (SV: assign req_ready = 1).
  io.gbarBus.req.ready    := true.B
}
