// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_issue_top.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_issue_top: flat-port wrapper around VX_issue.
 *
 *  Converts all interface-style ports to individual scalar/vector signals,
 *  mirroring the explicit assign statements in VX_issue_top.sv.  This is the
 *  form used when VX_issue needs to be instantiated from outside the Vortex
 *  pipeline hierarchy (e.g. simulation drivers or integration tests).
 */
class VxIssueTop(val instanceId: String = "issue") extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val numDispatch = NUM_EX_UNITS * ISSUE_WIDTH

  val io = IO(new Bundle {
    // Decode input (flat)
    val decode_valid   = Input(Bool())
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
    val decode_ready   = Output(Bool())

    // Writeback (flat, ISSUE_WIDTH)
    val writeback_valid = Input(Vec(ISSUE_WIDTH, Bool()))
    val writeback_uuid  = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val writeback_wis   = Input(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val writeback_sid   = Input(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val writeback_tmask = Input(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val writeback_PC    = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val writeback_rd    = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val writeback_data  = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val writeback_sop   = Input(Vec(ISSUE_WIDTH, Bool()))
    val writeback_eop   = Input(Vec(ISSUE_WIDTH, Bool()))

    // Dispatch (flat, NUM_EX_UNITS * ISSUE_WIDTH)
    val dispatch_valid    = Output(Vec(numDispatch, Bool()))
    val dispatch_uuid     = Output(Vec(numDispatch, UInt(UUID_WIDTH.W)))
    val dispatch_wis      = Output(Vec(numDispatch, UInt(ISSUE_WIS_W.W)))
    val dispatch_sid      = Output(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
    val dispatch_tmask    = Output(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
    val dispatch_PC       = Output(Vec(numDispatch, UInt(PC_BITS.W)))
    val dispatch_op_type  = Output(Vec(numDispatch, UInt(4.W)))  // INST_ALU_BITS
    val dispatch_op_args  = Output(Vec(numDispatch, UInt(INST_ARGS_BITS.W)))
    val dispatch_wb       = Output(Vec(numDispatch, Bool()))
    val dispatch_rd       = Output(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
    val dispatch_rs1_data = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs2_data = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs3_data = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_sop      = Output(Vec(numDispatch, Bool()))
    val dispatch_eop      = Output(Vec(numDispatch, Bool()))
    val dispatch_ready    = Input(Vec(numDispatch, Bool()))

    // Issue→scheduler feedback
    val issue_sched_wis   = Output(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val issue_sched_valid = Output(Vec(ISSUE_WIDTH, Bool()))
  })

  // -------------------------------------------------------------------------
  // Instantiate VxIssue
  // -------------------------------------------------------------------------
  val issue = Module(new VxIssue(instanceId))

  // Decode
  issue.io.decode_valid   := io.decode_valid
  io.decode_ready         := issue.io.decode_ready
  issue.io.decode_uuid    := io.decode_uuid
  issue.io.decode_wid     := io.decode_wid
  issue.io.decode_tmask   := io.decode_tmask
  issue.io.decode_PC      := io.decode_PC
  issue.io.decode_ex_type := io.decode_ex_type
  issue.io.decode_op_type := io.decode_op_type
  issue.io.decode_op_args := io.decode_op_args
  issue.io.decode_wb      := io.decode_wb
  issue.io.decode_used_rs := io.decode_used_rs
  issue.io.decode_rd      := io.decode_rd
  issue.io.decode_rs1     := io.decode_rs1
  issue.io.decode_rs2     := io.decode_rs2
  issue.io.decode_rs3     := io.decode_rs3

  // Writeback
  for (i <- 0 until ISSUE_WIDTH) {
    issue.io.wb_valid(i)  := io.writeback_valid(i)
    issue.io.wb_uuid(i)   := io.writeback_uuid(i)
    issue.io.wb_wis(i)    := io.writeback_wis(i)
    issue.io.wb_sid(i)    := io.writeback_sid(i)
    issue.io.wb_tmask(i)  := io.writeback_tmask(i)
    issue.io.wb_PC(i)     := io.writeback_PC(i)
    issue.io.wb_rd(i)     := io.writeback_rd(i)
    issue.io.wb_data(i)   := io.writeback_data(i)
    issue.io.wb_sop(i)    := io.writeback_sop(i)
    issue.io.wb_eop(i)    := io.writeback_eop(i)
  }

  // Dispatch
  for (i <- 0 until numDispatch) {
    io.dispatch_valid(i)    := issue.io.dispatch_valid(i)
    io.dispatch_uuid(i)     := issue.io.dispatch_uuid(i)
    io.dispatch_wis(i)      := issue.io.dispatch_wis(i)
    io.dispatch_sid(i)      := issue.io.dispatch_sid(i)
    io.dispatch_tmask(i)    := issue.io.dispatch_tmask(i)
    io.dispatch_PC(i)       := issue.io.dispatch_PC(i)
    io.dispatch_op_type(i)  := issue.io.dispatch_op_type(i)
    io.dispatch_op_args(i)  := issue.io.dispatch_op_args(i)
    io.dispatch_wb(i)       := issue.io.dispatch_wb(i)
    io.dispatch_rd(i)       := issue.io.dispatch_rd(i)
    io.dispatch_rs1_data(i) := issue.io.dispatch_rs1_data(i)
    io.dispatch_rs2_data(i) := issue.io.dispatch_rs2_data(i)
    io.dispatch_rs3_data(i) := issue.io.dispatch_rs3_data(i)
    io.dispatch_sop(i)      := issue.io.dispatch_sop(i)
    io.dispatch_eop(i)      := issue.io.dispatch_eop(i)
    issue.io.dispatch_ready(i) := io.dispatch_ready(i)
  }

  // Scheduler feedback
  for (i <- 0 until ISSUE_WIDTH) {
    io.issue_sched_wis(i)   := issue.io.issue_sched_wis(i)
    io.issue_sched_valid(i) := issue.io.issue_sched_valid(i)
  }
}
