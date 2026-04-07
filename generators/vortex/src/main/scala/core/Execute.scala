// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_execute.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_execute: top-level execution-stage assembler.
 *
 *  Instantiates ALU, LSU, FPU (when EXT_F_ENABLE), TCU (when EXT_TCU_ENABLE),
 *  and SFU sub-units and connects dispatch/commit interfaces.
 *
 *  The Chisel translation keeps the structural hierarchy faithful to the SV
 *  source.  All `ifdef / `ifndef guards that select optional units are replaced
 *  by Scala if-conditions on the compile-time constants EXT_F_ENABLED and
 *  EXT_TCU_ENABLED.
 *
 *  Interface naming follows the flat-port convention used throughout this
 *  translated codebase:
 *    - Arrays of dispatch/commit interfaces are represented as Vec of valid/
 *      ready/data groups.
 *    - All bus fields are spelled out individually.
 *
 *  @param instanceId  debug string (unused in synthesis)
 *  @param coreId      core index
 */
class VxExecute(val instanceId: String = "", val coreId: Int = 0) extends Module {
  // -------------------------------------------------------------------------
  // Port counts
  // -------------------------------------------------------------------------
  private val numDispatch = NUM_EX_UNITS * ISSUE_WIDTH
  private val numLsuBlocks = NUM_LSU_BLOCKS
  private val numAluBlocks = NUM_ALU_BLOCKS

  // -----------------------------------------------------------------------
  // I/O
  // -----------------------------------------------------------------------
  val io = IO(new Bundle {
    // LSU → D-cache bus  (NUM_LSU_BLOCKS interfaces, flattened)
    val lsu_mem_req_valid  = Output(Vec(numLsuBlocks, Bool()))
    val lsu_mem_req_ready  = Input(Vec(numLsuBlocks, Bool()))
    val lsu_mem_req_rw     = Output(Vec(numLsuBlocks, Bool()))
    val lsu_mem_req_mask   = Output(Vec(numLsuBlocks, UInt(NUM_LSU_LANES.W)))
    val lsu_mem_req_byteen = Output(Vec(numLsuBlocks, Vec(NUM_LSU_LANES, UInt(LSU_WORD_SIZE.W))))
    val lsu_mem_req_addr   = Output(Vec(numLsuBlocks, Vec(NUM_LSU_LANES, UInt(LSU_ADDR_WIDTH.W))))
    val lsu_mem_req_flags  = Output(Vec(numLsuBlocks, Vec(NUM_LSU_LANES, UInt(MEM_FLAGS_WIDTH.W))))
    val lsu_mem_req_data   = Output(Vec(numLsuBlocks, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
    val lsu_mem_req_tag    = Output(Vec(numLsuBlocks, UInt(LSU_TAG_WIDTH.W)))
    val lsu_mem_rsp_valid  = Input(Vec(numLsuBlocks, Bool()))
    val lsu_mem_rsp_ready  = Output(Vec(numLsuBlocks, Bool()))
    val lsu_mem_rsp_mask   = Input(Vec(numLsuBlocks, UInt(NUM_LSU_LANES.W)))
    val lsu_mem_rsp_data   = Input(Vec(numLsuBlocks, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
    val lsu_mem_rsp_tag    = Input(Vec(numLsuBlocks, UInt(LSU_TAG_WIDTH.W)))

    // Dispatch interface  (NUM_EX_UNITS * ISSUE_WIDTH slots)
    val dispatch_valid    = Input(Vec(numDispatch, Bool()))
    val dispatch_ready    = Output(Vec(numDispatch, Bool()))
    val dispatch_uuid     = Input(Vec(numDispatch, UInt(UUID_WIDTH.W)))
    val dispatch_wis      = Input(Vec(numDispatch, UInt(ISSUE_WIS_W.W)))
    val dispatch_sid      = Input(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
    val dispatch_tmask    = Input(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
    val dispatch_PC       = Input(Vec(numDispatch, UInt(PC_BITS.W)))
    val dispatch_op_type  = Input(Vec(numDispatch, UInt(4.W)))
    val dispatch_op_args  = Input(Vec(numDispatch, UInt(INST_ARGS_BITS.W)))
    val dispatch_wb       = Input(Vec(numDispatch, Bool()))
    val dispatch_rd       = Input(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
    val dispatch_rs1_data = Input(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs2_data = Input(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs3_data = Input(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_sop      = Input(Vec(numDispatch, Bool()))
    val dispatch_eop      = Input(Vec(numDispatch, Bool()))

    // Commit interface  (same count as dispatch)
    val commit_valid  = Output(Vec(numDispatch, Bool()))
    val commit_ready  = Input(Vec(numDispatch, Bool()))
    val commit_uuid   = Output(Vec(numDispatch, UInt(UUID_WIDTH.W)))
    val commit_wid    = Output(Vec(numDispatch, UInt(NW_WIDTH.W)))
    val commit_sid    = Output(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
    val commit_tmask  = Output(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
    val commit_PC     = Output(Vec(numDispatch, UInt(PC_BITS.W)))
    val commit_wb     = Output(Vec(numDispatch, Bool()))
    val commit_rd     = Output(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
    val commit_data   = Output(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val commit_sop    = Output(Vec(numDispatch, Bool()))
    val commit_eop    = Output(Vec(numDispatch, Bool()))

    // MPM class — direct from RoCC (same pattern as startup_addr)
    val mpm_class = Input(UInt(8.W))

    // Scheduler CSR interface (slave)
    val sched_csr_cycles        = Input(UInt(PERF_CTR_BITS.W))
    val sched_csr_active_warps  = Input(UInt(NUM_WARPS.W))
    val sched_csr_thread_masks  = Input(Vec(NUM_WARPS, UInt(NUM_THREADS.W)))
    val sched_csr_alm_empty     = Input(Bool())
    val sched_csr_alm_empty_wid = Output(UInt(NW_WIDTH.W))
    val sched_csr_unlock_wid    = Output(UInt(NW_WIDTH.W))
    val sched_csr_unlock_warp   = Output(Bool())

    // Branch control outputs  (NUM_ALU_BLOCKS)
    val branch_ctl_valid = Output(Vec(numAluBlocks, Bool()))
    val branch_ctl_wid   = Output(Vec(numAluBlocks, UInt(NW_WIDTH.W)))
    val branch_ctl_taken = Output(Vec(numAluBlocks, Bool()))
    val branch_ctl_dest  = Output(Vec(numAluBlocks, UInt(PC_BITS.W)))

    // Warp control outputs
    val warp_ctl_valid    = Output(Bool())
    val warp_ctl_wid      = Output(UInt(NW_WIDTH.W))
    // tmc
    val warp_ctl_tmc_valid = Output(Bool())
    val warp_ctl_tmc_tmask = Output(UInt(NUM_THREADS.W))
    // wspawn
    val warp_ctl_wspawn_valid = Output(Bool())
    val warp_ctl_wspawn_wmask = Output(UInt(NUM_WARPS.W))
    val warp_ctl_wspawn_pc    = Output(UInt(PC_BITS.W))
    // split
    val warp_ctl_split_valid      = Output(Bool())
    val warp_ctl_split_is_dvg     = Output(Bool())
    val warp_ctl_split_then_tmask = Output(UInt(NUM_THREADS.W))
    val warp_ctl_split_else_tmask = Output(UInt(NUM_THREADS.W))
    val warp_ctl_split_next_pc    = Output(UInt(PC_BITS.W))
    // join
    val warp_ctl_join_valid     = Output(Bool())
    val warp_ctl_join_stack_ptr = Output(UInt(DV_STACK_SIZEW.W))
    // barrier
    val warp_ctl_barrier_valid    = Output(Bool())
    val warp_ctl_barrier_id       = Output(UInt(NB_WIDTH.W))
    val warp_ctl_barrier_is_global = Output(Bool())
    val warp_ctl_barrier_size_m1  = Output(UInt(NW_WIDTH.W))
    val warp_ctl_barrier_is_noop  = Output(Bool())
    val warp_ctl_dvstack_wid      = Output(UInt(NW_WIDTH.W))
    val warp_ctl_dvstack_ptr      = Input(UInt(DV_STACK_SIZEW.W))

    // Commit CSR interface (slave – receives instret from commit stage)
    val commit_csr_instret = Input(UInt(PERF_CTR_BITS.W))
  })

  // -------------------------------------------------------------------------
  // Sub-unit instantiation.
  //
  // The SV module instantiates VX_alu_unit, VX_lsu_unit, VX_fpu_unit (ifdef),
  // VX_tcu_unit (ifdef), VX_sfu_unit.  In the Chisel translation each of
  // those is a separate Module class defined in the corresponding file.
  //
  // For the structural translation we connect the I/O ports directly.
  // The sub-unit classes are declared in their respective files.
  // -------------------------------------------------------------------------

  // ALU unit
  val aluUnit = Module(new AluUnit)
  for (i <- 0 until ISSUE_WIDTH) {
    val dispIdx = EX_ALU * ISSUE_WIDTH + i
    aluUnit.io.dispatch_valid(i)    := io.dispatch_valid(dispIdx)
    io.dispatch_ready(dispIdx)      := aluUnit.io.dispatch_ready(i)
    aluUnit.io.dispatch_uuid(i)     := io.dispatch_uuid(dispIdx)
    aluUnit.io.dispatch_wid(i)      := wis_to_wid(io.dispatch_wis(dispIdx), i.U(ISSUE_ISW_W.W))
    aluUnit.io.dispatch_pid(i)      := io.dispatch_sid(dispIdx)
    aluUnit.io.dispatch_tmask(i)    := io.dispatch_tmask(dispIdx)
    aluUnit.io.dispatch_PC(i)       := io.dispatch_PC(dispIdx)
    aluUnit.io.dispatch_op_type(i)  := io.dispatch_op_type(dispIdx)
    aluUnit.io.dispatch_op_args(i)  := io.dispatch_op_args(dispIdx)
    aluUnit.io.dispatch_wb(i)       := io.dispatch_wb(dispIdx)
    aluUnit.io.dispatch_rd(i)       := io.dispatch_rd(dispIdx)
    aluUnit.io.dispatch_rs1_data(i) := io.dispatch_rs1_data(dispIdx)
    aluUnit.io.dispatch_rs2_data(i) := io.dispatch_rs2_data(dispIdx)
    aluUnit.io.dispatch_rs3_data(i) := io.dispatch_rs3_data(dispIdx)
    aluUnit.io.dispatch_sop(i)      := io.dispatch_sop(dispIdx)
    aluUnit.io.dispatch_eop(i)      := io.dispatch_eop(dispIdx)
    io.commit_valid(dispIdx)        := aluUnit.io.commit_valid(i)
    aluUnit.io.commit_ready(i)      := io.commit_ready(dispIdx)
    io.commit_uuid(dispIdx)         := aluUnit.io.commit_uuid(i)
    io.commit_wid(dispIdx)          := aluUnit.io.commit_wid(i)
    io.commit_sid(dispIdx)          := aluUnit.io.commit_pid(i)
    io.commit_tmask(dispIdx)        := aluUnit.io.commit_tmask(i)
    io.commit_PC(dispIdx)           := aluUnit.io.commit_PC(i)
    io.commit_wb(dispIdx)           := aluUnit.io.commit_wb(i)
    io.commit_rd(dispIdx)           := aluUnit.io.commit_rd(i)
    io.commit_data(dispIdx)         := aluUnit.io.commit_data(i)
    io.commit_sop(dispIdx)          := aluUnit.io.commit_sop(i)
    io.commit_eop(dispIdx)          := aluUnit.io.commit_eop(i)
  }
  for (i <- 0 until numAluBlocks) {
    io.branch_ctl_valid(i) := aluUnit.io.branch_valid(i)
    io.branch_ctl_wid(i)   := aluUnit.io.branch_wid(i)
    io.branch_ctl_taken(i) := aluUnit.io.branch_taken(i)
    io.branch_ctl_dest(i)  := aluUnit.io.branch_dest(i)
  }

  // LSU unit
  val lsuUnit = Module(new LsuUnit(instanceId + "-lsu"))
  for (i <- 0 until ISSUE_WIDTH) {
    val dispIdx = EX_LSU * ISSUE_WIDTH + i
    val lsuArgs = io.dispatch_op_args(dispIdx).asTypeOf(new LsuArgsBundle)
    lsuUnit.io.dispatch_valid(i)  := io.dispatch_valid(dispIdx)
    io.dispatch_ready(dispIdx)    := lsuUnit.io.dispatch_ready(i)
    lsuUnit.io.d_uuid(i)          := io.dispatch_uuid(dispIdx)
    lsuUnit.io.d_wis(i)           := io.dispatch_wis(dispIdx)
    lsuUnit.io.d_sid(i)           := io.dispatch_sid(dispIdx)
    lsuUnit.io.d_tmask(i)         := io.dispatch_tmask(dispIdx)
    lsuUnit.io.d_PC(i)            := io.dispatch_PC(dispIdx)
    lsuUnit.io.d_op_type(i)       := io.dispatch_op_type(dispIdx)(INST_LSU_BITS - 1, 0)
    lsuUnit.io.d_wb(i)            := io.dispatch_wb(dispIdx)
    lsuUnit.io.d_rd(i)            := io.dispatch_rd(dispIdx)
    lsuUnit.io.d_rs1_data(i)      := io.dispatch_rs1_data(dispIdx)
    lsuUnit.io.d_rs2_data(i)      := io.dispatch_rs2_data(dispIdx)
    lsuUnit.io.d_lsu_offset(i)    := lsuArgs.offset
    lsuUnit.io.d_lsu_is_store(i)  := lsuArgs.is_store
    lsuUnit.io.d_pid(i)           := io.dispatch_sid(dispIdx)
    lsuUnit.io.d_sop(i)           := io.dispatch_sop(dispIdx)
    lsuUnit.io.d_eop(i)           := io.dispatch_eop(dispIdx)
    io.commit_valid(dispIdx)      := lsuUnit.io.commit_valid(i)
    lsuUnit.io.commit_ready(i)    := io.commit_ready(dispIdx)
    io.commit_uuid(dispIdx)       := lsuUnit.io.c_uuid(i)
    io.commit_wid(dispIdx)        := lsuUnit.io.c_wid(i)
    io.commit_sid(dispIdx)        := lsuUnit.io.c_pid(i)
    io.commit_tmask(dispIdx)      := lsuUnit.io.c_tmask(i)
    io.commit_PC(dispIdx)         := lsuUnit.io.c_PC(i)
    io.commit_wb(dispIdx)         := lsuUnit.io.c_wb(i)
    io.commit_rd(dispIdx)         := lsuUnit.io.c_rd(i)
    io.commit_data(dispIdx)       := lsuUnit.io.c_data(i)
    io.commit_sop(dispIdx)        := lsuUnit.io.c_sop(i)
    io.commit_eop(dispIdx)        := lsuUnit.io.c_eop(i)
  }
  for (i <- 0 until numLsuBlocks) {
    io.lsu_mem_req_valid(i)        := lsuUnit.io.mem_req_valid(i)
    lsuUnit.io.mem_req_ready(i)    := io.lsu_mem_req_ready(i)
    io.lsu_mem_req_rw(i)           := lsuUnit.io.mem_req_rw(i)
    io.lsu_mem_req_mask(i)         := lsuUnit.io.mem_req_mask(i)
    io.lsu_mem_req_byteen(i)       := lsuUnit.io.mem_req_byteen(i)
    io.lsu_mem_req_addr(i)         := lsuUnit.io.mem_req_addr(i)
    io.lsu_mem_req_flags(i)        := lsuUnit.io.mem_req_flags(i)
    io.lsu_mem_req_data(i)         := lsuUnit.io.mem_req_data(i)
    io.lsu_mem_req_tag(i)          := lsuUnit.io.mem_req_tag(i)
    lsuUnit.io.mem_rsp_valid(i)    := io.lsu_mem_rsp_valid(i)
    io.lsu_mem_rsp_ready(i)        := lsuUnit.io.mem_rsp_ready(i)
    lsuUnit.io.mem_rsp_mask(i)     := io.lsu_mem_rsp_mask(i)
    lsuUnit.io.mem_rsp_data(i)     := io.lsu_mem_rsp_data(i)
    lsuUnit.io.mem_rsp_tag(i)      := io.lsu_mem_rsp_tag(i)
    lsuUnit.io.mem_rsp_sop(i)      := false.B
    lsuUnit.io.mem_rsp_eop(i)      := false.B
  }

  // SFU unit (WctlUnit internally) + CSR unit
  val sfuUnit = Module(new SfuUnit(coreId))
  val csrUnit = Module(new CsrUnit(coreId, NUM_SFU_LANES, instanceId + "-csr"))

  // tie off perf/sysmem inputs (no perf counters collected yet)
  csrUnit.io.sysmem_perf   := 0.U.asTypeOf(new SysmemPerfBundle)
  csrUnit.io.pipeline_perf := 0.U.asTypeOf(new PipelinePerfBundle)
  csrUnit.io.mpm_class     := io.mpm_class

  // FPU unit — instantiated after csrUnit so CSR cross-connections can be made
  if (EXT_F_ENABLED == 1) {
    val fpuUnit = Module(new VxFpuUnit(instanceId + "-fpu"))
    for (i <- 0 until ISSUE_WIDTH) {
      val dispIdx = EX_FPU * ISSUE_WIDTH + i
      fpuUnit.io.dispatch_valid(i)    := io.dispatch_valid(dispIdx)
      io.dispatch_ready(dispIdx)      := fpuUnit.io.dispatch_ready(i)
      fpuUnit.io.dispatch_uuid(i)     := io.dispatch_uuid(dispIdx)
      fpuUnit.io.dispatch_wis(i)      := io.dispatch_wis(dispIdx)
      fpuUnit.io.dispatch_sid(i)      := io.dispatch_sid(dispIdx)
      fpuUnit.io.dispatch_tmask(i)    := io.dispatch_tmask(dispIdx)
      fpuUnit.io.dispatch_PC(i)       := io.dispatch_PC(dispIdx)
      fpuUnit.io.dispatch_op_type(i)  := io.dispatch_op_type(dispIdx)
      fpuUnit.io.dispatch_op_args(i)  := io.dispatch_op_args(dispIdx)
      fpuUnit.io.dispatch_wb(i)       := io.dispatch_wb(dispIdx)
      fpuUnit.io.dispatch_rd(i)       := io.dispatch_rd(dispIdx)
      fpuUnit.io.dispatch_rs1_data(i) := io.dispatch_rs1_data(dispIdx)
      fpuUnit.io.dispatch_rs2_data(i) := io.dispatch_rs2_data(dispIdx)
      fpuUnit.io.dispatch_rs3_data(i) := io.dispatch_rs3_data(dispIdx)
      fpuUnit.io.dispatch_sop(i)      := io.dispatch_sop(dispIdx)
      fpuUnit.io.dispatch_eop(i)      := io.dispatch_eop(dispIdx)
      io.commit_valid(dispIdx)        := fpuUnit.io.commit_valid(i)
      fpuUnit.io.commit_ready(i)      := io.commit_ready(dispIdx)
      io.commit_uuid(dispIdx)         := fpuUnit.io.commit_uuid(i)
      io.commit_wid(dispIdx)          := fpuUnit.io.commit_wis(i)
      io.commit_sid(dispIdx)          := fpuUnit.io.commit_sid(i)
      io.commit_tmask(dispIdx)        := fpuUnit.io.commit_tmask(i)
      io.commit_PC(dispIdx)           := fpuUnit.io.commit_PC(i)
      io.commit_wb(dispIdx)           := fpuUnit.io.commit_wb(i)
      io.commit_rd(dispIdx)           := fpuUnit.io.commit_rd(i)
      io.commit_data(dispIdx)         := fpuUnit.io.commit_data(i)
      io.commit_sop(dispIdx)          := fpuUnit.io.commit_sop(i)
      io.commit_eop(dispIdx)          := fpuUnit.io.commit_eop(i)
    }
    // FPU ↔ CSR cross-connections
    for (b <- 0 until NUM_FPU_BLOCKS) {
      csrUnit.io.fpu_write_enable(b) := fpuUnit.io.fpu_csr_write_enable(b)
      csrUnit.io.fpu_write_wid(b)    := fpuUnit.io.fpu_csr_write_wid(b)
      csrUnit.io.fpu_write_fflags(b) := fpuUnit.io.fpu_csr_write_fflags(b)
      csrUnit.io.fpu_read_wid(b)     := fpuUnit.io.fpu_csr_read_wid(b)
      fpuUnit.io.fpu_csr_read_frm(b) := csrUnit.io.fpu_read_frm(b)
    }
  } else {
    csrUnit.io.fpu_write_enable := VecInit(Seq.fill(NUM_FPU_BLOCKS)(false.B))
    csrUnit.io.fpu_write_wid    := VecInit(Seq.fill(NUM_FPU_BLOCKS)(0.U))
    csrUnit.io.fpu_write_fflags := VecInit(Seq.fill(NUM_FPU_BLOCKS)(0.U))
    csrUnit.io.fpu_read_wid     := VecInit(Seq.fill(NUM_FPU_BLOCKS)(0.U))
  }

  // commit instret
  csrUnit.io.commit_instret := io.commit_csr_instret

  // scheduler CSR sideband → CsrUnit
  csrUnit.io.cycles        := io.sched_csr_cycles
  csrUnit.io.active_warps  := io.sched_csr_active_warps
  csrUnit.io.thread_masks  := io.sched_csr_thread_masks
  csrUnit.io.alm_empty     := io.sched_csr_alm_empty
  io.sched_csr_alm_empty_wid := csrUnit.io.alm_empty_wid
  io.sched_csr_unlock_wid    := csrUnit.io.unlock_wid
  io.sched_csr_unlock_warp   := csrUnit.io.unlock_warp

  // Dispatch to SfuUnit
  for (i <- 0 until ISSUE_WIDTH) {
    val dispIdx = EX_SFU * ISSUE_WIDTH + i
    sfuUnit.io.dispatch_valid(i)    := io.dispatch_valid(dispIdx)
    io.dispatch_ready(dispIdx)      := sfuUnit.io.dispatch_ready(i)
    sfuUnit.io.dispatch_uuid(i)     := io.dispatch_uuid(dispIdx)
    sfuUnit.io.dispatch_wid(i)      := wis_to_wid(io.dispatch_wis(dispIdx), i.U(ISSUE_ISW_W.W))
    sfuUnit.io.dispatch_tmask(i)    := io.dispatch_tmask(dispIdx)
    sfuUnit.io.dispatch_PC(i)       := io.dispatch_PC(dispIdx)
    sfuUnit.io.dispatch_op_type(i)  := io.dispatch_op_type(dispIdx)
    sfuUnit.io.dispatch_op_args(i)  := io.dispatch_op_args(dispIdx)
    sfuUnit.io.dispatch_wb(i)       := io.dispatch_wb(dispIdx)
    sfuUnit.io.dispatch_rd(i)       := io.dispatch_rd(dispIdx)
    sfuUnit.io.dispatch_pid(i)      := io.dispatch_sid(dispIdx)
    sfuUnit.io.dispatch_rs1_data(i) := io.dispatch_rs1_data(dispIdx)
    sfuUnit.io.dispatch_rs2_data(i) := io.dispatch_rs2_data(dispIdx)
    sfuUnit.io.dispatch_rs3_data(i) := io.dispatch_rs3_data(dispIdx)
    sfuUnit.io.dispatch_sop(i)      := io.dispatch_sop(dispIdx)
    sfuUnit.io.dispatch_eop(i)      := io.dispatch_eop(dispIdx)
    io.commit_valid(dispIdx)        := sfuUnit.io.commit_valid(i)
    sfuUnit.io.commit_ready(i)      := io.commit_ready(dispIdx)
    io.commit_uuid(dispIdx)         := sfuUnit.io.commit_uuid(i)
    io.commit_wid(dispIdx)          := sfuUnit.io.commit_wid(i)
    io.commit_sid(dispIdx)          := sfuUnit.io.commit_pid(i)
    io.commit_tmask(dispIdx)        := sfuUnit.io.commit_tmask(i)
    io.commit_PC(dispIdx)           := sfuUnit.io.commit_PC(i)
    io.commit_wb(dispIdx)           := sfuUnit.io.commit_wb(i)
    io.commit_rd(dispIdx)           := sfuUnit.io.commit_rd(i)
    io.commit_data(dispIdx)         := sfuUnit.io.commit_data(i)
    io.commit_sop(dispIdx)          := sfuUnit.io.commit_sop(i)
    io.commit_eop(dispIdx)          := sfuUnit.io.commit_eop(i)
  }

  // SFU warp_ctl outputs
  io.warp_ctl_valid                     := sfuUnit.io.warp_ctl_valid
  io.warp_ctl_wid                       := sfuUnit.io.warp_ctl_wid
  io.warp_ctl_tmc_valid                 := sfuUnit.io.warp_ctl_tmc_valid
  io.warp_ctl_tmc_tmask                 := sfuUnit.io.warp_ctl_tmc_tmask
  io.warp_ctl_wspawn_valid              := sfuUnit.io.warp_ctl_wspawn_valid
  io.warp_ctl_wspawn_wmask              := sfuUnit.io.warp_ctl_wspawn_wmask
  io.warp_ctl_wspawn_pc                 := sfuUnit.io.warp_ctl_wspawn_pc
  io.warp_ctl_split_valid               := sfuUnit.io.warp_ctl_split_valid
  io.warp_ctl_split_is_dvg              := sfuUnit.io.warp_ctl_split_is_dvg
  io.warp_ctl_split_then_tmask          := sfuUnit.io.warp_ctl_split_then_tmask
  io.warp_ctl_split_else_tmask          := sfuUnit.io.warp_ctl_split_else_tmask
  io.warp_ctl_split_next_pc             := sfuUnit.io.warp_ctl_split_next_pc
  io.warp_ctl_join_valid                := sfuUnit.io.warp_ctl_join_valid
  io.warp_ctl_join_stack_ptr            := sfuUnit.io.warp_ctl_join_stack_ptr
  io.warp_ctl_barrier_valid             := sfuUnit.io.warp_ctl_bar_valid
  io.warp_ctl_barrier_id                := sfuUnit.io.warp_ctl_bar_id
  io.warp_ctl_barrier_is_global         := sfuUnit.io.warp_ctl_bar_is_global
  io.warp_ctl_barrier_size_m1           := sfuUnit.io.warp_ctl_bar_size_m1
  io.warp_ctl_barrier_is_noop           := sfuUnit.io.warp_ctl_bar_is_noop
  io.warp_ctl_dvstack_wid               := sfuUnit.io.warp_ctl_dvstack_wid
  sfuUnit.io.warp_ctl_dvstack_ptr       := io.warp_ctl_dvstack_ptr

  // Connect SfuUnit CSR ports to CsrUnit
  val csrArgs = sfuUnit.io.csr_execute_op_args.asTypeOf(new CsrArgsBundle)
  csrUnit.io.execute_valid     := sfuUnit.io.csr_execute_valid
  sfuUnit.io.csr_execute_ready := csrUnit.io.execute_ready
  csrUnit.io.ex_uuid           := sfuUnit.io.csr_execute_uuid
  csrUnit.io.ex_wid            := sfuUnit.io.csr_execute_wid
  csrUnit.io.ex_tmask          := sfuUnit.io.csr_execute_tmask
  csrUnit.io.ex_PC             := sfuUnit.io.csr_execute_PC
  csrUnit.io.ex_op_type        := sfuUnit.io.csr_execute_op_type
  csrUnit.io.ex_wb             := sfuUnit.io.csr_execute_wb
  csrUnit.io.ex_rd             := sfuUnit.io.csr_execute_rd
  csrUnit.io.ex_rs1_data       := sfuUnit.io.csr_execute_rs1_data
  csrUnit.io.ex_csr_addr       := csrArgs.addr
  csrUnit.io.ex_csr_imm        := csrArgs.imm
  csrUnit.io.ex_csr_use_imm   := csrArgs.use_imm
  csrUnit.io.ex_pid            := sfuUnit.io.csr_execute_pid
  csrUnit.io.ex_sop            := sfuUnit.io.csr_execute_sop
  csrUnit.io.ex_eop            := sfuUnit.io.csr_execute_eop
  // CsrUnit result → SfuUnit
  sfuUnit.io.csr_result_valid := csrUnit.io.result_valid
  csrUnit.io.result_ready     := sfuUnit.io.csr_result_ready
  sfuUnit.io.csr_result_uuid  := csrUnit.io.res_uuid
  sfuUnit.io.csr_result_wid   := csrUnit.io.res_wid
  sfuUnit.io.csr_result_tmask := csrUnit.io.res_tmask
  sfuUnit.io.csr_result_PC    := csrUnit.io.res_PC
  sfuUnit.io.csr_result_rd    := csrUnit.io.res_rd
  sfuUnit.io.csr_result_wb    := csrUnit.io.res_wb
  sfuUnit.io.csr_result_pid   := csrUnit.io.res_pid
  sfuUnit.io.csr_result_sop   := csrUnit.io.res_sop
  sfuUnit.io.csr_result_eop   := csrUnit.io.res_eop
  sfuUnit.io.csr_result_data  := csrUnit.io.res_data
}

