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
//
// Translated from VX_dispatch.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// Dispatch — fan-out decoded instructions to per-execution-unit streams.
//
// Mirrors VX_dispatch.sv:
//   - Receives operands_if (OperandsBundle) from the operand collector.
//   - For each execution unit type (EX_ALU, EX_LSU, EX_SFU, EX_FPU …),
//     keeps a 2-entry elastic output buffer.
//   - The ready signal back to operands_if is taken from the buffer
//     corresponding to the current instruction's ex_type.
//   - Optionally counts per-unit stall cycles for performance counters.
//
// OUT_DATAW = $bits(dispatch_t) = DispatchBundle size.
// ---------------------------------------------------------------------------
class Dispatch(
  instanceId: String  = "dispatch",
  issueId:    Int     = 0,
  perfEnable: Boolean = false
) extends Module {

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Operands input (slave)
    val op_valid  = Input(Bool())
    val op_ready  = Output(Bool())
    val op_data   = Input(new OperandsBundle)

    // Per-execution-unit dispatch outputs (master)
    val dispatch_valid  = Output(Vec(NUM_EX_UNITS, Bool()))
    val dispatch_ready  = Input(Vec(NUM_EX_UNITS, Bool()))
    val dispatch_data   = Output(Vec(NUM_EX_UNITS, new DispatchBundle))

    // Performance counter: per-unit stall cycles
    val perf_stalls = Output(Vec(NUM_EX_UNITS, UInt(PERF_CTR_BITS.W)))
  })

  // -------------------------------------------------------------------------
  // Per-execution-unit 2-entry elastic buffers — Queue(2, pipe=true) matches
  // VX_elastic_buffer(SIZE=2, OUT_REG=1) from VX_dispatch.sv.
  // -------------------------------------------------------------------------
  val bufReadyIn = Wire(Vec(NUM_EX_UNITS, Bool()))
  io.op_ready := bufReadyIn(io.op_data.ex_type)

  for (i <- 0 until NUM_EX_UNITS) {
    val enq = Wire(Decoupled(new DispatchBundle))
    enq.valid          := io.op_valid && (io.op_data.ex_type === i.U)
    enq.bits.uuid      := io.op_data.uuid
    enq.bits.wis       := io.op_data.wis
    enq.bits.sid       := io.op_data.sid
    enq.bits.tmask     := io.op_data.tmask
    enq.bits.PC        := io.op_data.PC
    enq.bits.op_type   := io.op_data.op_type
    enq.bits.op_args   := io.op_data.op_args
    enq.bits.wb        := io.op_data.wb
    enq.bits.rd        := io.op_data.rd
    enq.bits.rs1_data  := io.op_data.rs1_data
    enq.bits.rs2_data  := io.op_data.rs2_data
    enq.bits.rs3_data  := io.op_data.rs3_data
    enq.bits.sop       := io.op_data.sop
    enq.bits.eop       := io.op_data.eop

    bufReadyIn(i) := enq.ready

    val deq = Queue(enq, entries = 2, pipe = true)
    deq.ready := io.dispatch_ready(i)

    io.dispatch_valid(i) := deq.valid
    io.dispatch_data(i)  := deq.bits
  }

  // -------------------------------------------------------------------------
  // Performance counters: stall cycle counting per execution unit
  // -------------------------------------------------------------------------
  val perfRegs = Seq.tabulate(NUM_EX_UNITS)(i => RegInit(0.U(PERF_CTR_BITS.W)))

  for (i <- 0 until NUM_EX_UNITS) {
    val stall = io.op_valid && !io.op_ready && (io.op_data.ex_type === i.U)
    if (perfEnable) {
      perfRegs(i) := perfRegs(i) + stall.asUInt.pad(PERF_CTR_BITS)
    }
    io.perf_stalls(i) := perfRegs(i)
  }
}
