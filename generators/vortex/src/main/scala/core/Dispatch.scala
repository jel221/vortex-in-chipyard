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
  // Per-execution-unit 2-entry elastic buffers
  // -------------------------------------------------------------------------
  // Each buffer holds a DispatchBundle and accepts from operands_if when
  // ex_type matches the buffer index.

  val bufValid  = RegInit(VecInit(Seq.fill(NUM_EX_UNITS)(false.B)))
  val bufData   = Reg(Vec(NUM_EX_UNITS, new DispatchBundle))

  val bufReadyIn = Wire(Vec(NUM_EX_UNITS, Bool()))

  // Connect ready back to operands_if: ready when the matching buffer accepts
  io.op_ready := bufReadyIn(io.op_data.ex_type)

  for (i <- 0 until NUM_EX_UNITS) {
    val exMatch = io.op_valid && (io.op_data.ex_type === i.U)

    // Buffer accepts when not full or downstream is consuming
    bufReadyIn(i) := !bufValid(i) || io.dispatch_ready(i)

    when (exMatch && bufReadyIn(i)) {
      bufValid(i) := true.B
      // Copy operands fields into dispatch bundle
      bufData(i).uuid     := io.op_data.uuid
      bufData(i).wis      := io.op_data.wis
      bufData(i).sid      := io.op_data.sid
      bufData(i).tmask    := io.op_data.tmask
      bufData(i).PC       := io.op_data.PC
      bufData(i).op_type  := io.op_data.op_type
      bufData(i).op_args  := io.op_data.op_args
      bufData(i).wb       := io.op_data.wb
      bufData(i).rd       := io.op_data.rd
      bufData(i).rs1_data := io.op_data.rs1_data
      bufData(i).rs2_data := io.op_data.rs2_data
      bufData(i).rs3_data := io.op_data.rs3_data
      bufData(i).sop      := io.op_data.sop
      bufData(i).eop      := io.op_data.eop
    }.elsewhen (io.dispatch_ready(i) && bufValid(i)) {
      bufValid(i) := false.B
    }

    io.dispatch_valid(i) := bufValid(i)
    io.dispatch_data(i)  := bufData(i)
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
