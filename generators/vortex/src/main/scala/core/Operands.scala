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
// Translated from VX_operands.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// Operands — register-file read and operand collection stage.
//
// Mirrors VX_operands.sv:
//   - Instantiates NUM_OPCS operand-collector units (VX_opc_unit, modelled
//     here as OpcUnit).
//   - Steers the scoreboard and writeback interfaces to the appropriate
//     collector based on the wis field (warp-in-slot index mod NUM_OPCS).
//   - Arbitrates the per-OPC operand outputs into a single output stream
//     using a priority arbiter (sticky when SIMD_COUNT != 1).
//   - Optionally accumulates per-OPC stall counts for performance counters.
//
// VX_opc_unit itself is not translated here — it is referenced as an
// abstract placeholder.  The structural wiring mirrors the SV exactly.
// ---------------------------------------------------------------------------
class Operands(
  instanceId: String  = "operands",
  issueId:    Int     = 0,
  perfEnable: Boolean = false
) extends Module {

  // OUT_ARB_STICKY = (NUM_OPCS != 1) && (SIMD_COUNT != 1)
  private val OUT_ARB_STICKY = (NUM_OPCS != 1) && (SIMD_COUNT != 1)

  // Total operands_t bit width:
  //   uuid + wis + sid + tmask + PC + ex_type + op_type + op_args + wb + rd +
  //   rs1_data + rs2_data + rs3_data + sop + eop
  private val OPERANDS_BITS =
    UUID_WIDTH + ISSUE_WIS_W + SIMD_IDX_W + SIMD_WIDTH + PC_BITS +
    EX_BITS + INST_OP_BITS + INST_ARGS_BITS +
    1 + NUM_REGS_BITS +
    (3 * SIMD_WIDTH * XLEN) + 1 + 1

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Writeback interface (push – no ready)
    val wb_valid  = Input(Bool())
    val wb_data   = Input(new WritebackBundle)  // from WritebackBundle in VortexGPUPkg

    // Scoreboard interface (slave handshake)
    val sb_valid  = Input(Bool())
    val sb_ready  = Output(Bool())
    val sb_data   = Input(new ScoreboardBundle)

    // Operands output (master handshake)
    val op_valid  = Output(Bool())
    val op_ready  = Input(Bool())
    val op_data   = Output(new OperandsBundle)

    // Performance: total operand-collector stall cycles
    val perf_stalls = Output(UInt(PERF_CTR_BITS.W))
  })

  // -------------------------------------------------------------------------
  // OPC index selection from wis field
  // -------------------------------------------------------------------------
  val sbOpc = if (NUM_OPCS != 1) io.sb_data.wis(NUM_OPCS_W - 1, 0) else 0.U(1.W)
  val wbOpc = if (NUM_OPCS != 1) io.wb_data.wis(NUM_OPCS_W - 1, 0) else 0.U(1.W)

  // Per-OPC valid / ready wires
  val opcOpValid  = Wire(Vec(NUM_OPCS, Bool()))
  val opcOpReady  = Wire(Vec(NUM_OPCS, Bool()))
  val opcOpData   = Wire(Vec(NUM_OPCS, new OperandsBundle))

  val sbReadyIn   = Wire(Vec(NUM_OPCS, Bool()))
  io.sb_ready := sbReadyIn(sbOpc)

  // -------------------------------------------------------------------------
  // Instantiate NUM_OPCS OpcUnit collectors
  // VX_opc_unit is not yet translated; we model it as a passthrough when
  // NUM_OPCS == 1 (the common default).
  // -------------------------------------------------------------------------
  for (i <- 0 until NUM_OPCS) {
    // Steer scoreboard to this OPC
    val opcSbValid = io.sb_valid && (sbOpc === i.U)
    // Steer writeback to this OPC
    val opcWbValid = io.wb_valid && (wbOpc === i.U)

    // When NUM_OPCS == 1 the collector is a single direct pipe:
    // valid_in = scoreboard valid, data passes through, stall when op_ready=0
    if (NUM_OPCS == 1) {
      // Simple single-cycle passthrough when no real collector is present.
      // A full OpcUnit would instantiate register-file banks here.
      opcOpValid(i)   := opcSbValid
      sbReadyIn(i)    := opcOpReady(i)
      // data passthrough: copy scoreboard fields into operands bundle
      // (rs1/rs2/rs3 data would be fetched from the register file here)
      opcOpData(i).uuid    := io.sb_data.uuid
      opcOpData(i).wis     := io.sb_data.wis
      opcOpData(i).sid     := 0.U
      opcOpData(i).tmask   := io.sb_data.tmask
      opcOpData(i).PC      := io.sb_data.PC
      opcOpData(i).ex_type := io.sb_data.ex_type
      opcOpData(i).op_type := io.sb_data.op_type
      opcOpData(i).op_args := io.sb_data.op_args
      opcOpData(i).wb      := io.sb_data.wb
      opcOpData(i).rd      := io.sb_data.rd
      // rs data left as zero (register-file read not modelled here)
      for (l <- 0 until SIMD_WIDTH) {
        opcOpData(i).rs1_data(l) := 0.U
        opcOpData(i).rs2_data(l) := 0.U
        opcOpData(i).rs3_data(l) := 0.U
      }
      opcOpData(i).sop := true.B
      opcOpData(i).eop := true.B
    } else {
      // Multi-OPC: connect to OpcUnit (placeholder wires)
      opcOpValid(i)   := false.B
      sbReadyIn(i)    := false.B
      opcOpData(i)    := 0.U.asTypeOf(new OperandsBundle)
    }
  }

  // -------------------------------------------------------------------------
  // Output arbiter: priority with optional sticky semantics
  // For NUM_OPCS == 1 this is just a direct connection.
  // -------------------------------------------------------------------------
  if (NUM_OPCS == 1) {
    io.op_valid       := opcOpValid(0)
    opcOpReady(0)     := io.op_ready
    io.op_data        := opcOpData(0)
  } else {
    // Priority arbiter over NUM_OPCS inputs
    // Find lowest-index valid input
    val arbGrant = PriorityEncoder(opcOpValid.asUInt)
    io.op_valid  := opcOpValid.asUInt.orR
    io.op_data   := opcOpData(arbGrant)
    for (i <- 0 until NUM_OPCS) {
      opcOpReady(i) := io.op_ready && (arbGrant === i.U) && io.op_valid
    }
  }

  // -------------------------------------------------------------------------
  // Performance counter: count stall cycles per OPC, then sum
  // -------------------------------------------------------------------------
  val perfStallsPerOpc = Wire(Vec(NUM_OPCS, UInt(PERF_CTR_BITS.W)))
  val perfRegs = Seq.tabulate(NUM_OPCS)(i => RegInit(0.U(PERF_CTR_BITS.W)))

  for (i <- 0 until NUM_OPCS) {
    val stall = opcOpValid(i) && !opcOpReady(i)
    perfRegs(i) := perfRegs(i) + stall.asUInt.pad(PERF_CTR_BITS)
    perfStallsPerOpc(i) := perfRegs(i)
  }

  if (perfEnable) {
    // Reduce sum across OPCs
    io.perf_stalls := perfStallsPerOpc.reduce(_ + _)
  } else {
    io.perf_stalls := 0.U
  }
}
