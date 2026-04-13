// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_issue_slice.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_issue_slice: one issue pipeline lane.
 *
 *  Chains: ibuffer → scoreboard → operands → dispatch.
 *  Also notifies the scheduler when the first SIMD packet (sop) departs.
 *
 *  Sub-modules:
 *    VX_ibuffer    → VxIbuffer
 *    VX_scoreboard → VxScoreboard
 *    VX_operands   → VxOpcUnit
 *    VX_dispatch   → Dispatch
 *
 *  @param instanceId  debug string
 *  @param issueId     which issue slot this slice belongs to
 */
class VxIssueSlice(
    val instanceId: String = "",
    val issueId:    Int    = 0
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  val io = IO(new Bundle {
    // Decode input (slave, one per issue slot)
    val decode_valid   = Input(Bool())
    val decode_ready   = Output(Bool())
    val decode_uuid    = Input(UInt(UUID_WIDTH.W))
    val decode_wid     = Input(UInt(NW_WIDTH.W))
    val decode_tmask   = Input(UInt(NUM_THREADS.W))
    val decode_PC      = Input(UInt(PC_BITS.W))
    val decode_ex_type = Input(UInt(EX_BITS.W))
    val decode_op_type = Input(UInt(INST_OP_BITS.W))
    val decode_op_args = Input(UInt(INST_ARGS_BITS.W))
    val decode_wb      = Input(Bool())
    val decode_used_rs = Input(UInt(NUM_SRC_OPDS.W))
    val decode_rd      = Input(UInt(NUM_REGS_BITS.W))
    val decode_rs1     = Input(UInt(NUM_REGS_BITS.W))
    val decode_rs2     = Input(UInt(NUM_REGS_BITS.W))
    val decode_rs3     = Input(UInt(NUM_REGS_BITS.W))

    // Writeback (slave)
    val wb_valid  = Input(Bool())
    val wb_uuid   = Input(UInt(UUID_WIDTH.W))
    val wb_wis    = Input(UInt(ISSUE_WIS_W.W))
    val wb_sid    = Input(UInt(SIMD_IDX_W.W))
    val wb_tmask  = Input(UInt(SIMD_WIDTH.W))
    val wb_PC     = Input(UInt(PC_BITS.W))
    val wb_rd     = Input(UInt(NUM_REGS_BITS.W))
    val wb_data   = Input(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val wb_sop    = Input(Bool())
    val wb_eop    = Input(Bool())

    // Dispatch outputs (NUM_EX_UNITS master channels)
    val dispatch_valid    = Output(Vec(NUM_EX_UNITS, Bool()))
    val dispatch_ready    = Input(Vec(NUM_EX_UNITS, Bool()))
    val dispatch_uuid     = Output(Vec(NUM_EX_UNITS, UInt(UUID_WIDTH.W)))
    val dispatch_wis      = Output(Vec(NUM_EX_UNITS, UInt(ISSUE_WIS_W.W)))
    val dispatch_sid      = Output(Vec(NUM_EX_UNITS, UInt(SIMD_IDX_W.W)))
    val dispatch_tmask    = Output(Vec(NUM_EX_UNITS, UInt(SIMD_WIDTH.W)))
    val dispatch_PC       = Output(Vec(NUM_EX_UNITS, UInt(PC_BITS.W)))
    val dispatch_op_type  = Output(Vec(NUM_EX_UNITS, UInt(4.W)))
    val dispatch_op_args  = Output(Vec(NUM_EX_UNITS, UInt(INST_ARGS_BITS.W)))
    val dispatch_wb       = Output(Vec(NUM_EX_UNITS, Bool()))
    val dispatch_rd       = Output(Vec(NUM_EX_UNITS, UInt(NUM_REGS_BITS.W)))
    val dispatch_rs1_data = Output(Vec(NUM_EX_UNITS, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs2_data = Output(Vec(NUM_EX_UNITS, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs3_data = Output(Vec(NUM_EX_UNITS, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_sop      = Output(Vec(NUM_EX_UNITS, Bool()))
    val dispatch_eop      = Output(Vec(NUM_EX_UNITS, Bool()))

    // Issue→scheduler feedback
    val issue_sched_valid = Output(Bool())
    val issue_sched_wis   = Output(UInt(ISSUE_WIS_W.W))
  })

  // -------------------------------------------------------------------------
  // VX_ibuffer
  // -------------------------------------------------------------------------
  val ibuffer = Module(new VxIbuffer(issueId))
  ibuffer.io.decode_valid   := io.decode_valid
  io.decode_ready           := ibuffer.io.decode_ready
  ibuffer.io.decode_uuid    := io.decode_uuid
  ibuffer.io.decode_wid     := io.decode_wid
  ibuffer.io.decode_tmask   := io.decode_tmask
  ibuffer.io.decode_PC      := io.decode_PC
  ibuffer.io.decode_ex_type := io.decode_ex_type
  ibuffer.io.decode_op_type := io.decode_op_type
  ibuffer.io.decode_op_args := io.decode_op_args
  ibuffer.io.decode_wb      := io.decode_wb
  ibuffer.io.decode_used_rs := io.decode_used_rs
  ibuffer.io.decode_rd      := io.decode_rd
  ibuffer.io.decode_rs1     := io.decode_rs1
  ibuffer.io.decode_rs2     := io.decode_rs2
  ibuffer.io.decode_rs3     := io.decode_rs3

  // Build per-warp IbufferBundle from VxIbuffer's individual field outputs
  val ibuf_valid  = Wire(Vec(PER_ISSUE_WARPS, Bool()))
  val ibuf_ready  = Wire(Vec(PER_ISSUE_WARPS, Bool()))
  val ibuf_bundle = Wire(Vec(PER_ISSUE_WARPS, new IbufferBundle))

  for (i <- 0 until PER_ISSUE_WARPS) {
    ibuf_valid(i)             := ibuffer.io.ibuf_valid(i)
    ibuffer.io.ibuf_ready(i) := ibuf_ready(i)
    ibuf_bundle(i).uuid    := ibuffer.io.ibuf_uuid(i)
    ibuf_bundle(i).tmask   := ibuffer.io.ibuf_tmask(i)
    ibuf_bundle(i).PC      := ibuffer.io.ibuf_PC(i)
    ibuf_bundle(i).ex_type := ibuffer.io.ibuf_ex_type(i)
    ibuf_bundle(i).op_type := ibuffer.io.ibuf_op_type(i)
    ibuf_bundle(i).op_args.bits := ibuffer.io.ibuf_op_args(i)
    ibuf_bundle(i).wb      := ibuffer.io.ibuf_wb(i)
    ibuf_bundle(i).used_rs := ibuffer.io.ibuf_used_rs(i)
    ibuf_bundle(i).rd      := ibuffer.io.ibuf_rd(i)
    ibuf_bundle(i).rs1     := ibuffer.io.ibuf_rs1(i)
    ibuf_bundle(i).rs2     := ibuffer.io.ibuf_rs2(i)
    ibuf_bundle(i).rs3     := ibuffer.io.ibuf_rs3(i)
  }

  // -------------------------------------------------------------------------
  // VX_scoreboard
  // -------------------------------------------------------------------------
  val scoreboard = Module(new VxScoreboard(issueId))

  // Writeback: build WritebackBundle from flat wb_* IO
  scoreboard.io.writeback_valid      := io.wb_valid
  scoreboard.io.writeback_data.uuid  := io.wb_uuid
  scoreboard.io.writeback_data.wis   := io.wb_wis
  scoreboard.io.writeback_data.sid   := io.wb_sid
  scoreboard.io.writeback_data.tmask := io.wb_tmask
  scoreboard.io.writeback_data.PC    := io.wb_PC
  scoreboard.io.writeback_data.rd    := io.wb_rd
  scoreboard.io.writeback_data.data  := io.wb_data
  scoreboard.io.writeback_data.sop   := io.wb_sop
  scoreboard.io.writeback_data.eop   := io.wb_eop

  // ibuffer → scoreboard
  for (i <- 0 until PER_ISSUE_WARPS) {
    scoreboard.io.ibuffer_valid(i) := ibuf_valid(i)
    ibuf_ready(i)                  := scoreboard.io.ibuffer_ready(i)
    scoreboard.io.ibuffer_data(i)  := ibuf_bundle(i)
  }

  // Scoreboard output wires (for VxOpcUnit flat interface)
  val sb_valid = Wire(Bool())
  val sb_ready = Wire(Bool())
  val sb_data  = Wire(new ScoreboardBundle)

  sb_valid                        := scoreboard.io.scoreboard_valid
  scoreboard.io.scoreboard_ready  := sb_ready
  sb_data                         := scoreboard.io.scoreboard_data

  // -------------------------------------------------------------------------
  // VX_operands (VxOpcUnit) — unchanged flat interface
  // -------------------------------------------------------------------------
  val operands = Module(new VxOpcUnit(instanceId + "-operands"))
  operands.io.wb_valid   := io.wb_valid
  operands.io.wb_uuid    := io.wb_uuid
  operands.io.wb_wis     := io.wb_wis
  operands.io.wb_sid     := io.wb_sid
  operands.io.wb_tmask   := io.wb_tmask
  operands.io.wb_PC      := io.wb_PC
  operands.io.wb_rd      := io.wb_rd
  operands.io.wb_data    := io.wb_data
  operands.io.wb_sop     := io.wb_sop
  operands.io.wb_eop     := io.wb_eop

  // Scoreboard → operands (unpack ScoreboardBundle to flat fields)
  operands.io.sb_valid   := sb_valid
  sb_ready               := operands.io.sb_ready
  operands.io.sb_uuid    := sb_data.uuid
  operands.io.sb_wis     := sb_data.wis
  operands.io.sb_tmask   := sb_data.tmask
  operands.io.sb_PC      := sb_data.PC
  operands.io.sb_ex_type := sb_data.ex_type
  operands.io.sb_op_type := sb_data.op_type
  operands.io.sb_op_args := sb_data.op_args.bits
  operands.io.sb_wb      := sb_data.wb
  operands.io.sb_used_rs := sb_data.used_rs
  operands.io.sb_rd      := sb_data.rd
  operands.io.sb_rs1     := sb_data.rs1
  operands.io.sb_rs2     := sb_data.rs2
  operands.io.sb_rs3     := sb_data.rs3

  // -------------------------------------------------------------------------
  // VX_dispatch (Dispatch)
  // -------------------------------------------------------------------------
  val dispatch = Module(new Dispatch(instanceId + "-dispatch", issueId))

  // Build OperandsBundle from VxOpcUnit flat outputs
  val opd_bundle = Wire(new OperandsBundle)
  opd_bundle.uuid          := operands.io.opd_uuid
  opd_bundle.wis           := operands.io.opd_wis
  opd_bundle.sid           := operands.io.opd_sid
  opd_bundle.tmask         := operands.io.opd_tmask
  opd_bundle.PC            := operands.io.opd_PC
  opd_bundle.ex_type       := operands.io.opd_ex_type
  opd_bundle.op_type       := operands.io.opd_op_type
  opd_bundle.op_args.bits  := operands.io.opd_op_args
  opd_bundle.wb            := operands.io.opd_wb
  opd_bundle.rd            := operands.io.opd_rd
  opd_bundle.rs1_data      := operands.io.opd_rs1_data
  opd_bundle.rs2_data      := operands.io.opd_rs2_data
  opd_bundle.rs3_data      := operands.io.opd_rs3_data
  opd_bundle.sop           := operands.io.opd_sop
  opd_bundle.eop           := operands.io.opd_eop

  dispatch.io.op_valid  := operands.io.opd_valid
  operands.io.opd_ready := dispatch.io.op_ready
  dispatch.io.op_data   := opd_bundle

  // Unpack DispatchBundle to flat IssueSlice dispatch outputs
  for (i <- 0 until NUM_EX_UNITS) {
    io.dispatch_valid(i)    := dispatch.io.dispatch_valid(i)
    dispatch.io.dispatch_ready(i) := io.dispatch_ready(i)
    io.dispatch_uuid(i)     := dispatch.io.dispatch_data(i).uuid
    io.dispatch_wis(i)      := dispatch.io.dispatch_data(i).wis
    io.dispatch_sid(i)      := dispatch.io.dispatch_data(i).sid
    io.dispatch_tmask(i)    := dispatch.io.dispatch_data(i).tmask
    io.dispatch_PC(i)       := dispatch.io.dispatch_data(i).PC
    io.dispatch_op_type(i)  := dispatch.io.dispatch_data(i).op_type
    io.dispatch_op_args(i)  := dispatch.io.dispatch_data(i).op_args.bits
    io.dispatch_wb(i)       := dispatch.io.dispatch_data(i).wb
    io.dispatch_rd(i)       := dispatch.io.dispatch_data(i).rd
    io.dispatch_rs1_data(i) := dispatch.io.dispatch_data(i).rs1_data
    io.dispatch_rs2_data(i) := dispatch.io.dispatch_data(i).rs2_data
    io.dispatch_rs3_data(i) := dispatch.io.dispatch_data(i).rs3_data
    io.dispatch_sop(i)      := dispatch.io.dispatch_data(i).sop
    io.dispatch_eop(i)      := dispatch.io.dispatch_data(i).eop
  }

  // -------------------------------------------------------------------------
  // Scheduler feedback: fire on sop departure from operands stage
  // -------------------------------------------------------------------------
  io.issue_sched_valid := operands.io.opd_valid && dispatch.io.op_ready && operands.io.opd_sop
  io.issue_sched_wis   := operands.io.opd_wis
}
