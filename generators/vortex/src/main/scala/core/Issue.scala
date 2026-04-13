// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_issue.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_issue: top-level issue stage.
 *
 *  Distributes the decode stream across ISSUE_WIDTH issue slices and collects
 *  their dispatch outputs into the transposed dispatch_if array expected by
 *  VX_execute.
 *
 *  Each issue slice handles PER_ISSUE_WARPS warps.  The incoming decode
 *  stream is steered to the appropriate slice by comparing decode_wid's
 *  issue-slot-within-width (ISW) index.
 *
 *  Dispatch output transposition:
 *    per-slice dispatch_if[ex_id] → dispatch_if[ex_id * ISSUE_WIDTH + issue_id]
 *
 *  Mirrors the generate loop in VX_issue.sv.
 */
class VxIssue(val instanceId: String = "") extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val numDispatch = NUM_EX_UNITS * ISSUE_WIDTH

  val io = IO(new Bundle {
    // Decode input (single stream, master sends wid to steer)
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

    // Writeback inputs (one per issue slot)
    val wb_valid  = Input(Vec(ISSUE_WIDTH, Bool()))
    val wb_uuid   = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val wb_wis    = Input(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val wb_sid    = Input(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val wb_tmask  = Input(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val wb_PC     = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val wb_rd     = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val wb_data   = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val wb_sop    = Input(Vec(ISSUE_WIDTH, Bool()))
    val wb_eop    = Input(Vec(ISSUE_WIDTH, Bool()))

    // Dispatch outputs (NUM_EX_UNITS * ISSUE_WIDTH, transposed)
    val dispatch_valid    = Output(Vec(numDispatch, Bool()))
    val dispatch_ready    = Input(Vec(numDispatch, Bool()))
    val dispatch_uuid     = Output(Vec(numDispatch, UInt(UUID_WIDTH.W)))
    val dispatch_wis      = Output(Vec(numDispatch, UInt(ISSUE_WIS_W.W)))
    val dispatch_sid      = Output(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
    val dispatch_tmask    = Output(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
    val dispatch_PC       = Output(Vec(numDispatch, UInt(PC_BITS.W)))
    val dispatch_op_type  = Output(Vec(numDispatch, UInt(4.W)))
    val dispatch_op_args  = Output(Vec(numDispatch, UInt(INST_ARGS_BITS.W)))
    val dispatch_wb       = Output(Vec(numDispatch, Bool()))
    val dispatch_rd       = Output(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
    val dispatch_rs1_data = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs2_data = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs3_data = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_sop      = Output(Vec(numDispatch, Bool()))
    val dispatch_eop      = Output(Vec(numDispatch, Bool()))

    // Issue→scheduler feedback (one per issue slot)
    val issue_sched_valid = Output(Vec(ISSUE_WIDTH, Bool()))
    val issue_sched_wis   = Output(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
  })

  // -------------------------------------------------------------------------
  // Compute which issue slot this decode belongs to.
  // decode_isw = wid_to_isw(decode_wid)
  // -------------------------------------------------------------------------
  val decodeIsw = wid_to_isw(io.decode_wid)

  // Per-slice decode ready signals; the overall decode_ready is the one
  // matching the current ISW.
  val decodeReadyIn = Wire(Vec(ISSUE_WIDTH, Bool()))
  io.decode_ready := decodeReadyIn(decodeIsw)

  for (issueId <- 0 until ISSUE_WIDTH) {
    val slice = Module(new VxIssueSlice(
      instanceId = s"${instanceId}${issueId}",
      issueId    = issueId
    ))

    // Steer decode to matching slice only
    slice.io.decode_valid   := io.decode_valid && (decodeIsw === issueId.U)
    slice.io.decode_uuid    := io.decode_uuid
    slice.io.decode_wid     := io.decode_wid
    slice.io.decode_tmask   := io.decode_tmask
    slice.io.decode_PC      := io.decode_PC
    slice.io.decode_ex_type := io.decode_ex_type
    slice.io.decode_op_type := io.decode_op_type
    slice.io.decode_op_args := io.decode_op_args
    slice.io.decode_wb      := io.decode_wb
    slice.io.decode_used_rs := io.decode_used_rs
    slice.io.decode_rd      := io.decode_rd
    slice.io.decode_rs1     := io.decode_rs1
    slice.io.decode_rs2     := io.decode_rs2
    slice.io.decode_rs3     := io.decode_rs3
    decodeReadyIn(issueId)  := slice.io.decode_ready

    // Writeback to this slice
    slice.io.wb_valid  := io.wb_valid(issueId)
    slice.io.wb_uuid   := io.wb_uuid(issueId)
    slice.io.wb_wis    := io.wb_wis(issueId)
    slice.io.wb_sid    := io.wb_sid(issueId)
    slice.io.wb_tmask  := io.wb_tmask(issueId)
    slice.io.wb_PC     := io.wb_PC(issueId)
    slice.io.wb_rd     := io.wb_rd(issueId)
    slice.io.wb_data   := io.wb_data(issueId)
    slice.io.wb_sop    := io.wb_sop(issueId)
    slice.io.wb_eop    := io.wb_eop(issueId)

    // Scheduler feedback
    io.issue_sched_valid(issueId) := slice.io.issue_sched_valid
    io.issue_sched_wis(issueId)   := slice.io.issue_sched_wis

    // Transpose dispatch: slice.dispatch[ex_id] → dispatch[ex_id*ISSUE_WIDTH + issueId]
    for (exId <- 0 until NUM_EX_UNITS) {
      val dispIdx = exId * ISSUE_WIDTH + issueId
      io.dispatch_valid(dispIdx)    := slice.io.dispatch_valid(exId)
      slice.io.dispatch_ready(exId) := io.dispatch_ready(dispIdx)
      io.dispatch_uuid(dispIdx)     := slice.io.dispatch_uuid(exId)
      io.dispatch_wis(dispIdx)      := slice.io.dispatch_wis(exId)
      io.dispatch_sid(dispIdx)      := slice.io.dispatch_sid(exId)
      io.dispatch_tmask(dispIdx)    := slice.io.dispatch_tmask(exId)
      io.dispatch_PC(dispIdx)       := slice.io.dispatch_PC(exId)
      io.dispatch_op_type(dispIdx)  := slice.io.dispatch_op_type(exId)
      io.dispatch_op_args(dispIdx)  := slice.io.dispatch_op_args(exId)
      io.dispatch_wb(dispIdx)       := slice.io.dispatch_wb(exId)
      io.dispatch_rd(dispIdx)       := slice.io.dispatch_rd(exId)
      io.dispatch_rs1_data(dispIdx) := slice.io.dispatch_rs1_data(exId)
      io.dispatch_rs2_data(dispIdx) := slice.io.dispatch_rs2_data(exId)
      io.dispatch_rs3_data(dispIdx) := slice.io.dispatch_rs3_data(exId)
      io.dispatch_sop(dispIdx)      := slice.io.dispatch_sop(exId)
      io.dispatch_eop(dispIdx)      := slice.io.dispatch_eop(exId)
    }
  }
}
