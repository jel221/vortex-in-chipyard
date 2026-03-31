// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_sfu_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Special Function Unit (SFU).
 *
 *  Corresponds to VX_sfu_unit.sv.
 *
 *  The SFU receives dispatched SFU instructions and routes them to one of
 *  two sub-units:
 *    - PE_IDX_WCTL (0): WctlUnit  — warp control (TMC, WSPAWN, SPLIT, JOIN, BAR, PRED)
 *    - PE_IDX_CSRS (1): CSR unit  — CSR read/write
 *
 *  The CSR unit is not translated here; its interface is exposed as raw IO
 *  ports so it can be connected externally.
 *
 *  @param coreId  CORE_ID (unused in combinational logic, kept for completeness)
 */
class SfuUnit(val coreId: Int = 0) extends Module {

  private val numLanes  = NUM_SFU_LANES
  private val pidBits   = math.max(0, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
  private val pidWidth  = math.max(1, pidBits)

  val io = IO(new Bundle {
    // Dispatch interfaces (ISSUE_WIDTH slots) – slave
    val dispatch_valid    = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_ready    = Output(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_uuid     = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val dispatch_wid      = Input(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
    val dispatch_tmask    = Input(Vec(ISSUE_WIDTH, UInt(numLanes.W)))
    val dispatch_PC       = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val dispatch_rd       = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val dispatch_wb       = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_pid      = Input(Vec(ISSUE_WIDTH, UInt(pidWidth.W)))
    val dispatch_sop      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_eop      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_op_type  = Input(Vec(ISSUE_WIDTH, UInt(INST_SFU_BITS.W)))
    val dispatch_op_args  = Input(Vec(ISSUE_WIDTH, UInt(INST_ARGS_BITS.W)))
    val dispatch_rs1_data = Input(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))
    val dispatch_rs2_data = Input(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))
    val dispatch_rs3_data = Input(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))

    // Commit interfaces (ISSUE_WIDTH slots) – master
    val commit_valid  = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_ready  = Input(Vec(ISSUE_WIDTH, Bool()))
    val commit_uuid   = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val commit_wid    = Output(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
    val commit_tmask  = Output(Vec(ISSUE_WIDTH, UInt(numLanes.W)))
    val commit_PC     = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val commit_rd     = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val commit_wb     = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_pid    = Output(Vec(ISSUE_WIDTH, UInt(pidWidth.W)))
    val commit_sop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_eop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_data   = Output(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))

    // Warp control outputs (from WctlUnit, aggregated)
    val warp_ctl_valid            = Output(Bool())
    val warp_ctl_wid              = Output(UInt(NW_WIDTH.W))
    val warp_ctl_tmc_valid        = Output(Bool())
    val warp_ctl_tmc_tmask        = Output(UInt(NUM_THREADS.W))
    val warp_ctl_wspawn_valid     = Output(Bool())
    val warp_ctl_wspawn_wmask     = Output(UInt(NUM_WARPS.W))
    val warp_ctl_wspawn_pc        = Output(UInt(PC_BITS.W))
    val warp_ctl_split_valid      = Output(Bool())
    val warp_ctl_split_is_dvg     = Output(Bool())
    val warp_ctl_split_then_tmask = Output(UInt(NUM_THREADS.W))
    val warp_ctl_split_else_tmask = Output(UInt(NUM_THREADS.W))
    val warp_ctl_split_next_pc    = Output(UInt(PC_BITS.W))
    val warp_ctl_join_valid       = Output(Bool())
    val warp_ctl_join_stack_ptr   = Output(UInt(DV_STACK_SIZEW.W))
    val warp_ctl_bar_valid        = Output(Bool())
    val warp_ctl_bar_id           = Output(UInt(NB_WIDTH.W))
    val warp_ctl_bar_is_global    = Output(Bool())
    val warp_ctl_bar_size_m1      = Output(UInt(NW_WIDTH.W))
    val warp_ctl_bar_is_noop      = Output(Bool())
    val warp_ctl_dvstack_wid      = Output(UInt(NW_WIDTH.W))
    val warp_ctl_dvstack_ptr      = Input(UInt(DV_STACK_SIZEW.W))

    // CSR sub-unit execute interface (forwarded out so external CSR unit can connect)
    val csr_execute_valid    = Output(Bool())
    val csr_execute_ready    = Input(Bool())
    val csr_execute_uuid     = Output(UInt(UUID_WIDTH.W))
    val csr_execute_wid      = Output(UInt(NW_WIDTH.W))
    val csr_execute_tmask    = Output(UInt(numLanes.W))
    val csr_execute_PC       = Output(UInt(PC_BITS.W))
    val csr_execute_rd       = Output(UInt(NUM_REGS_BITS.W))
    val csr_execute_wb       = Output(Bool())
    val csr_execute_pid      = Output(UInt(pidWidth.W))
    val csr_execute_sop      = Output(Bool())
    val csr_execute_eop      = Output(Bool())
    val csr_execute_op_type  = Output(UInt(INST_SFU_BITS.W))
    val csr_execute_op_args  = Output(UInt(INST_ARGS_BITS.W))
    val csr_execute_rs1_data = Output(Vec(numLanes, UInt(XLEN.W)))
    val csr_execute_rs2_data = Output(Vec(numLanes, UInt(XLEN.W)))

    // CSR sub-unit result interface (external CSR unit drives these back in)
    val csr_result_valid = Input(Bool())
    val csr_result_ready = Output(Bool())
    val csr_result_uuid  = Input(UInt(UUID_WIDTH.W))
    val csr_result_wid   = Input(UInt(NW_WIDTH.W))
    val csr_result_tmask = Input(UInt(numLanes.W))
    val csr_result_PC    = Input(UInt(PC_BITS.W))
    val csr_result_rd    = Input(UInt(NUM_REGS_BITS.W))
    val csr_result_wb    = Input(Bool())
    val csr_result_pid   = Input(UInt(pidWidth.W))
    val csr_result_sop   = Input(Bool())
    val csr_result_eop   = Input(Bool())
    val csr_result_data  = Input(Vec(numLanes, UInt(XLEN.W)))
  })

  // The SFU has BLOCK_SIZE=1, so only one dispatch slot feeds into one PE switch.
  // We use dispatch slot 0 (the SV always picks [0] when BLOCK_SIZE=1).
  // For ISSUE_WIDTH > 1 the remaining slots are tied off.

  // -------------------------------------------------------------------------
  // PE select: is this a CSR op?
  // -------------------------------------------------------------------------
  val isCsrOp = inst_sfu_is_csr(io.dispatch_op_type(0))

  // -------------------------------------------------------------------------
  // WctlUnit
  // -------------------------------------------------------------------------
  val wctlUnit = Module(new WctlUnit(numLanes))

  wctlUnit.io.execute_valid    := io.dispatch_valid(0) && !isCsrOp
  wctlUnit.io.execute_uuid     := io.dispatch_uuid(0)
  wctlUnit.io.execute_wid      := io.dispatch_wid(0)
  wctlUnit.io.execute_tmask    := io.dispatch_tmask(0)
  wctlUnit.io.execute_PC       := io.dispatch_PC(0)
  wctlUnit.io.execute_rd       := io.dispatch_rd(0)
  wctlUnit.io.execute_wb       := io.dispatch_wb(0)
  wctlUnit.io.execute_pid      := io.dispatch_pid(0)
  wctlUnit.io.execute_sop      := io.dispatch_sop(0)
  wctlUnit.io.execute_eop      := io.dispatch_eop(0)
  wctlUnit.io.execute_op_type  := io.dispatch_op_type(0)
  wctlUnit.io.execute_op_args  := io.dispatch_op_args(0)
  wctlUnit.io.execute_rs1_data := io.dispatch_rs1_data(0)
  wctlUnit.io.execute_rs2_data := io.dispatch_rs2_data(0)
  wctlUnit.io.warp_ctl_dvstack_ptr := io.warp_ctl_dvstack_ptr

  // Warp control forwarding
  io.warp_ctl_valid            := wctlUnit.io.warp_ctl_valid
  io.warp_ctl_wid              := wctlUnit.io.warp_ctl_wid
  io.warp_ctl_tmc_valid        := wctlUnit.io.warp_ctl_tmc_valid
  io.warp_ctl_tmc_tmask        := wctlUnit.io.warp_ctl_tmc_tmask
  io.warp_ctl_wspawn_valid     := wctlUnit.io.warp_ctl_wspawn_valid
  io.warp_ctl_wspawn_wmask     := wctlUnit.io.warp_ctl_wspawn_wmask
  io.warp_ctl_wspawn_pc        := wctlUnit.io.warp_ctl_wspawn_pc
  io.warp_ctl_split_valid      := wctlUnit.io.warp_ctl_split_valid
  io.warp_ctl_split_is_dvg     := wctlUnit.io.warp_ctl_split_is_dvg
  io.warp_ctl_split_then_tmask := wctlUnit.io.warp_ctl_split_then_tmask
  io.warp_ctl_split_else_tmask := wctlUnit.io.warp_ctl_split_else_tmask
  io.warp_ctl_split_next_pc    := wctlUnit.io.warp_ctl_split_next_pc
  io.warp_ctl_join_valid       := wctlUnit.io.warp_ctl_join_valid
  io.warp_ctl_join_stack_ptr   := wctlUnit.io.warp_ctl_join_stack_ptr
  io.warp_ctl_bar_valid        := wctlUnit.io.warp_ctl_bar_valid
  io.warp_ctl_bar_id           := wctlUnit.io.warp_ctl_bar_id
  io.warp_ctl_bar_is_global    := wctlUnit.io.warp_ctl_bar_is_global
  io.warp_ctl_bar_size_m1      := wctlUnit.io.warp_ctl_bar_size_m1
  io.warp_ctl_bar_is_noop      := wctlUnit.io.warp_ctl_bar_is_noop
  io.warp_ctl_dvstack_wid      := wctlUnit.io.warp_ctl_dvstack_wid

  // -------------------------------------------------------------------------
  // CSR sub-unit forwarding (execute side)
  // -------------------------------------------------------------------------
  io.csr_execute_valid    := io.dispatch_valid(0) && isCsrOp
  io.csr_execute_uuid     := io.dispatch_uuid(0)
  io.csr_execute_wid      := io.dispatch_wid(0)
  io.csr_execute_tmask    := io.dispatch_tmask(0)
  io.csr_execute_PC       := io.dispatch_PC(0)
  io.csr_execute_rd       := io.dispatch_rd(0)
  io.csr_execute_wb       := io.dispatch_wb(0)
  io.csr_execute_pid      := io.dispatch_pid(0)
  io.csr_execute_sop      := io.dispatch_sop(0)
  io.csr_execute_eop      := io.dispatch_eop(0)
  io.csr_execute_op_type  := io.dispatch_op_type(0)
  io.csr_execute_op_args  := io.dispatch_op_args(0)
  io.csr_execute_rs1_data := io.dispatch_rs1_data(0)
  io.csr_execute_rs2_data := io.dispatch_rs2_data(0)

  // -------------------------------------------------------------------------
  // dispatch_ready for slot 0: route to appropriate sub-unit
  // -------------------------------------------------------------------------
  io.dispatch_ready(0) := Mux(isCsrOp,
    io.csr_execute_ready,
    wctlUnit.io.execute_ready)

  // Tie off extra dispatch slots (if ISSUE_WIDTH > 1)
  for (i <- 1 until ISSUE_WIDTH) {
    io.dispatch_ready(i) := false.B
  }

  // -------------------------------------------------------------------------
  // Result arbitration: priority wctl then csr (mirrors PE_SWITCH "R" arbiter)
  // For result output we pick whichever is valid, wctl first.
  // -------------------------------------------------------------------------
  val wctlValid = wctlUnit.io.result_valid
  val csrValid  = io.csr_result_valid

  val selWctl = wctlValid
  val outValid = wctlValid || csrValid

  // Feed result_ready back to whichever sub-unit is selected
  wctlUnit.io.result_ready := io.commit_ready(0) && selWctl
  io.csr_result_ready      := io.commit_ready(0) && !selWctl

  // Commit output slot 0
  io.commit_valid(0) := outValid
  io.commit_uuid(0)  := Mux(selWctl, wctlUnit.io.result_uuid,  io.csr_result_uuid)
  io.commit_wid(0)   := Mux(selWctl, wctlUnit.io.result_wid,   io.csr_result_wid)
  io.commit_tmask(0) := Mux(selWctl, wctlUnit.io.result_tmask, io.csr_result_tmask)
  io.commit_PC(0)    := Mux(selWctl, wctlUnit.io.result_PC,    io.csr_result_PC)
  io.commit_rd(0)    := Mux(selWctl, wctlUnit.io.result_rd,    io.csr_result_rd)
  io.commit_wb(0)    := Mux(selWctl, wctlUnit.io.result_wb,    io.csr_result_wb)
  io.commit_pid(0)   := Mux(selWctl, wctlUnit.io.result_pid,   io.csr_result_pid)
  io.commit_sop(0)   := Mux(selWctl, wctlUnit.io.result_sop,   io.csr_result_sop)
  io.commit_eop(0)   := Mux(selWctl, wctlUnit.io.result_eop,   io.csr_result_eop)
  io.commit_data(0)  := Mux(selWctl, wctlUnit.io.result_data,  io.csr_result_data)

  // Tie off extra commit slots
  for (i <- 1 until ISSUE_WIDTH) {
    io.commit_valid(i) := false.B
    io.commit_uuid(i)  := 0.U
    io.commit_wid(i)   := 0.U
    io.commit_tmask(i) := 0.U
    io.commit_PC(i)    := 0.U
    io.commit_rd(i)    := 0.U
    io.commit_wb(i)    := false.B
    io.commit_pid(i)   := 0.U
    io.commit_sop(i)   := false.B
    io.commit_eop(i)   := false.B
    io.commit_data(i)  := VecInit(Seq.fill(numLanes)(0.U(XLEN.W)))
  }
}
