// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_core.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_core: top-level GPGPU core assembler.
 *
 *  Instantiates and connects:
 *    VX_dcr_data   → VxDcrData   (BB stub)
 *    VX_schedule   → VxSchedule  (BB stub)
 *    VX_fetch      → VxFetch     (VxFetch, already translated)
 *    VX_decode     → VxDecode    (BB stub)
 *    VX_issue      → VxIssue     (translated above)
 *    VX_execute    → VxExecute   (translated above)
 *    VX_commit     → VxCommit    (BB stub)
 *    VX_mem_unit   → VxMemUnit   (translated above)
 *
 *  @param coreId     CORE_ID parameter
 *  @param instanceId debug string
 */
class VxCore(val coreId: Int = 0, val instanceId: String = "") extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val numDispatch = NUM_EX_UNITS * ISSUE_WIDTH

  val io = IO(new Bundle {
    // DCR bus (slave)
    val dcr_write_valid = Input(Bool())
    val dcr_write_addr  = Input(UInt(VX_DCR_ADDR_WIDTH.W))
    val dcr_write_data  = Input(UInt(32.W))

    // D-cache bus (DCACHE_NUM_REQS master ports)
    val dcache_req_valid  = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_req_ready  = Input(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_req_rw     = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_req_byteen = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_WORD_SIZE.W)))
    val dcache_req_addr   = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_ADDR_WIDTH.W)))
    val dcache_req_flags  = Output(Vec(DCACHE_NUM_REQS, UInt(MEM_FLAGS_WIDTH.W)))
    val dcache_req_data   = Output(Vec(DCACHE_NUM_REQS, UInt((DCACHE_WORD_SIZE * 8).W)))
    val dcache_req_tag    = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_TAG_WIDTH.W)))
    val dcache_rsp_valid  = Input(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_rsp_ready  = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_rsp_data   = Input(Vec(DCACHE_NUM_REQS, UInt((DCACHE_WORD_SIZE * 8).W)))
    val dcache_rsp_tag    = Input(Vec(DCACHE_NUM_REQS, UInt(DCACHE_TAG_WIDTH.W)))

    // I-cache bus (1 master port)
    val icache_req_valid  = Output(Bool())
    val icache_req_ready  = Input(Bool())
    val icache_req_rw     = Output(Bool())
    val icache_req_byteen = Output(UInt(ICACHE_WORD_SIZE.W))
    val icache_req_addr   = Output(UInt(ICACHE_ADDR_WIDTH.W))
    val icache_req_data   = Output(UInt((ICACHE_WORD_SIZE * 8).W))
    val icache_req_tag    = Output(UInt(ICACHE_TAG_WIDTH.W))
    val icache_rsp_valid  = Input(Bool())
    val icache_rsp_ready  = Output(Bool())
    val icache_rsp_data   = Input(UInt((ICACHE_WORD_SIZE * 8).W))
    val icache_rsp_tag    = Input(UInt(ICACHE_TAG_WIDTH.W))

    // Status
    val busy = Output(Bool())
  })

  // -------------------------------------------------------------------------
  // Internal wires (pipeline inter-stage signals)
  // -------------------------------------------------------------------------

  // schedule → fetch
  val sched_valid = Wire(Bool())
  val sched_ready = Wire(Bool())
  val sched_uuid  = Wire(UInt(UUID_WIDTH.W))
  val sched_wid   = Wire(UInt(NW_WIDTH.W))
  val sched_tmask = Wire(UInt(NUM_THREADS.W))
  val sched_PC    = Wire(UInt(PC_BITS.W))

  // fetch → decode
  val fetch_valid = Wire(Bool())
  val fetch_ready = Wire(Bool())
  val fetch_uuid  = Wire(UInt(UUID_WIDTH.W))
  val fetch_wid   = Wire(UInt(NW_WIDTH.W))
  val fetch_tmask = Wire(UInt(NUM_THREADS.W))
  val fetch_PC    = Wire(UInt(PC_BITS.W))
  val fetch_instr = Wire(UInt(32.W))

  // decode → issue
  val decode_valid   = Wire(Bool())
  val decode_ready   = Wire(Bool())
  val decode_uuid    = Wire(UInt(UUID_WIDTH.W))
  val decode_wid     = Wire(UInt(NW_WIDTH.W))
  val decode_tmask   = Wire(UInt(NUM_THREADS.W))
  val decode_PC      = Wire(UInt(PC_BITS.W))
  val decode_ex_type = Wire(UInt(EX_BITS.W))
  val decode_op_type = Wire(UInt(INST_OP_BITS.W))
  val decode_op_args = Wire(UInt(INST_ARGS_BITS.W))
  val decode_wb      = Wire(Bool())
  val decode_used_rs = Wire(UInt(NUM_SRC_OPDS.W))
  val decode_rd      = Wire(UInt(NUM_REGS_BITS.W))
  val decode_rs1     = Wire(UInt(NUM_REGS_BITS.W))
  val decode_rs2     = Wire(UInt(NUM_REGS_BITS.W))
  val decode_rs3     = Wire(UInt(NUM_REGS_BITS.W))
  val decode_ibuf_pop = Wire(UInt(NUM_WARPS.W))

  // decode → scheduler sideband
  val decode_sched_valid  = Wire(Bool())
  val decode_sched_unlock = Wire(Bool())
  val decode_sched_wid    = Wire(UInt(NW_WIDTH.W))

  // issue → scheduler sideband
  val issue_sched_valid = Wire(Vec(ISSUE_WIDTH, Bool()))
  val issue_sched_wis   = Wire(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))

  // commit → scheduler sideband
  val commit_sched_warps = Wire(UInt(NUM_WARPS.W))

  // scheduler → CSR sideband
  val sched_csr_cycles       = Wire(UInt(PERF_CTR_BITS.W))
  val sched_csr_active_warps = Wire(UInt(NUM_WARPS.W))
  val sched_csr_thread_masks = Wire(Vec(NUM_WARPS, UInt(NUM_THREADS.W)))
  val sched_csr_alm_empty    = Wire(Bool())
  val sched_csr_alm_empty_wid = Wire(UInt(NW_WIDTH.W))
  val sched_csr_unlock_wid   = Wire(UInt(NW_WIDTH.W))
  val sched_csr_unlock_warp  = Wire(Bool())

  // warp control (execute → scheduler)
  val warp_ctl_valid           = Wire(Bool())
  val warp_ctl_wid             = Wire(UInt(NW_WIDTH.W))
  val warp_ctl_tmc_valid       = Wire(Bool())
  val warp_ctl_tmc_tmask       = Wire(UInt(NUM_THREADS.W))
  val warp_ctl_wspawn_valid    = Wire(Bool())
  val warp_ctl_wspawn_wmask    = Wire(UInt(NUM_WARPS.W))
  val warp_ctl_wspawn_pc       = Wire(UInt(PC_BITS.W))
  val warp_ctl_split_valid     = Wire(Bool())
  val warp_ctl_split_is_dvg    = Wire(Bool())
  val warp_ctl_split_then_tmask = Wire(UInt(NUM_THREADS.W))
  val warp_ctl_split_else_tmask = Wire(UInt(NUM_THREADS.W))
  val warp_ctl_split_next_pc   = Wire(UInt(PC_BITS.W))
  val warp_ctl_join_valid      = Wire(Bool())
  val warp_ctl_join_stack_ptr  = Wire(UInt(DV_STACK_SIZEW.W))
  val warp_ctl_barrier_valid   = Wire(Bool())
  val warp_ctl_barrier_id      = Wire(UInt(NB_WIDTH.W))
  val warp_ctl_barrier_is_global = Wire(Bool())
  val warp_ctl_barrier_size_m1 = Wire(UInt(NW_WIDTH.W))
  val warp_ctl_barrier_is_noop = Wire(Bool())
  val warp_ctl_dvstack_wid     = Wire(UInt(NW_WIDTH.W))
  val warp_ctl_dvstack_ptr     = Wire(UInt(DV_STACK_SIZEW.W))

  // branch control (execute → scheduler, NUM_ALU_BLOCKS)
  val branch_ctl_valid = Wire(Vec(NUM_ALU_BLOCKS, Bool()))
  val branch_ctl_wid   = Wire(Vec(NUM_ALU_BLOCKS, UInt(NW_WIDTH.W)))
  val branch_ctl_taken = Wire(Vec(NUM_ALU_BLOCKS, Bool()))
  val branch_ctl_dest  = Wire(Vec(NUM_ALU_BLOCKS, UInt(PC_BITS.W)))

  // issue → execute dispatch
  val dispatch_valid    = Wire(Vec(numDispatch, Bool()))
  val dispatch_ready    = Wire(Vec(numDispatch, Bool()))
  val dispatch_uuid     = Wire(Vec(numDispatch, UInt(UUID_WIDTH.W)))
  val dispatch_wis      = Wire(Vec(numDispatch, UInt(ISSUE_WIS_W.W)))
  val dispatch_sid      = Wire(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
  val dispatch_tmask    = Wire(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
  val dispatch_PC       = Wire(Vec(numDispatch, UInt(PC_BITS.W)))
  val dispatch_op_type  = Wire(Vec(numDispatch, UInt(4.W)))
  val dispatch_op_args  = Wire(Vec(numDispatch, UInt(INST_ARGS_BITS.W)))
  val dispatch_wb       = Wire(Vec(numDispatch, Bool()))
  val dispatch_rd       = Wire(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
  val dispatch_rs1_data = Wire(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
  val dispatch_rs2_data = Wire(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
  val dispatch_rs3_data = Wire(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
  val dispatch_sop      = Wire(Vec(numDispatch, Bool()))
  val dispatch_eop      = Wire(Vec(numDispatch, Bool()))

  // execute → commit
  val commit_valid  = Wire(Vec(numDispatch, Bool()))
  val commit_ready  = Wire(Vec(numDispatch, Bool()))
  val commit_uuid   = Wire(Vec(numDispatch, UInt(UUID_WIDTH.W)))
  val commit_wid    = Wire(Vec(numDispatch, UInt(NW_WIDTH.W)))
  val commit_sid    = Wire(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
  val commit_tmask  = Wire(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
  val commit_PC     = Wire(Vec(numDispatch, UInt(PC_BITS.W)))
  val commit_wb     = Wire(Vec(numDispatch, Bool()))
  val commit_rd     = Wire(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
  val commit_data   = Wire(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
  val commit_sop    = Wire(Vec(numDispatch, Bool()))
  val commit_eop    = Wire(Vec(numDispatch, Bool()))

  // commit → issue writeback
  val wb_valid  = Wire(Vec(ISSUE_WIDTH, Bool()))
  val wb_uuid   = Wire(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
  val wb_wis    = Wire(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
  val wb_sid    = Wire(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
  val wb_tmask  = Wire(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
  val wb_PC     = Wire(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
  val wb_rd     = Wire(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
  val wb_data   = Wire(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
  val wb_sop    = Wire(Vec(ISSUE_WIDTH, Bool()))
  val wb_eop    = Wire(Vec(ISSUE_WIDTH, Bool()))

  // commit → CSR
  val commit_csr_instret = Wire(UInt(PERF_CTR_BITS.W))

  // DCR → execute
  val base_dcrs = Wire(new BaseDcrsBundle)

  // LSU ↔ mem_unit
  val lsu_req_valid  = Wire(Vec(NUM_LSU_BLOCKS, Bool()))
  val lsu_req_ready  = Wire(Vec(NUM_LSU_BLOCKS, Bool()))
  val lsu_req_rw     = Wire(Vec(NUM_LSU_BLOCKS, Bool()))
  val lsu_req_mask   = Wire(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
  val lsu_req_byteen = Wire(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_WORD_SIZE.W))))
  val lsu_req_addr   = Wire(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_ADDR_WIDTH.W))))
  val lsu_req_flags  = Wire(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(MEM_FLAGS_WIDTH.W))))
  val lsu_req_data   = Wire(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
  val lsu_req_tag    = Wire(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))
  val lsu_rsp_valid  = Wire(Vec(NUM_LSU_BLOCKS, Bool()))
  val lsu_rsp_ready  = Wire(Vec(NUM_LSU_BLOCKS, Bool()))
  val lsu_rsp_mask   = Wire(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
  val lsu_rsp_data   = Wire(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
  val lsu_rsp_tag    = Wire(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))

  // -------------------------------------------------------------------------
  // VX_dcr_data
  // -------------------------------------------------------------------------
  val dcrData = Module(new VxDcrDataBB)
  dcrData.io.dcr_write_valid := io.dcr_write_valid
  dcrData.io.dcr_write_addr  := io.dcr_write_addr
  dcrData.io.dcr_write_data  := io.dcr_write_data
  base_dcrs                  := dcrData.io.base_dcrs

  // -------------------------------------------------------------------------
  // VX_schedule
  // -------------------------------------------------------------------------
  val schedule = Module(new VxScheduleBB(instanceId + "-schedule", coreId))
  schedule.io.base_dcrs          := base_dcrs
  schedule.io.warp_ctl_valid     := warp_ctl_valid
  schedule.io.warp_ctl_wid       := warp_ctl_wid
  schedule.io.warp_ctl_tmc_valid := warp_ctl_tmc_valid
  schedule.io.warp_ctl_tmc_tmask := warp_ctl_tmc_tmask
  schedule.io.warp_ctl_wspawn_valid := warp_ctl_wspawn_valid
  schedule.io.warp_ctl_wspawn_wmask := warp_ctl_wspawn_wmask
  schedule.io.warp_ctl_wspawn_pc    := warp_ctl_wspawn_pc
  schedule.io.warp_ctl_split_valid  := warp_ctl_split_valid
  schedule.io.warp_ctl_split_is_dvg := warp_ctl_split_is_dvg
  schedule.io.warp_ctl_split_then_tmask := warp_ctl_split_then_tmask
  schedule.io.warp_ctl_split_else_tmask := warp_ctl_split_else_tmask
  schedule.io.warp_ctl_split_next_pc    := warp_ctl_split_next_pc
  schedule.io.warp_ctl_join_valid       := warp_ctl_join_valid
  schedule.io.warp_ctl_join_stack_ptr   := warp_ctl_join_stack_ptr
  schedule.io.warp_ctl_barrier_valid    := warp_ctl_barrier_valid
  schedule.io.warp_ctl_barrier_id       := warp_ctl_barrier_id
  schedule.io.warp_ctl_barrier_is_global := warp_ctl_barrier_is_global
  schedule.io.warp_ctl_barrier_size_m1  := warp_ctl_barrier_size_m1
  schedule.io.warp_ctl_barrier_is_noop  := warp_ctl_barrier_is_noop
  schedule.io.warp_ctl_dvstack_wid      := warp_ctl_dvstack_wid
  warp_ctl_dvstack_ptr                  := schedule.io.warp_ctl_dvstack_ptr
  for (i <- 0 until NUM_ALU_BLOCKS) {
    schedule.io.branch_ctl_valid(i) := branch_ctl_valid(i)
    schedule.io.branch_ctl_wid(i)   := branch_ctl_wid(i)
    schedule.io.branch_ctl_taken(i) := branch_ctl_taken(i)
    schedule.io.branch_ctl_dest(i)  := branch_ctl_dest(i)
  }
  // decode_sched flows FROM decode TO scheduler (wired after decode is instantiated)
  for (i <- 0 until ISSUE_WIDTH) {
    schedule.io.issue_sched_valid(i) := issue_sched_valid(i)
    schedule.io.issue_sched_wis(i)   := issue_sched_wis(i)
  }
  schedule.io.commit_sched_warps := commit_sched_warps
  sched_valid              := schedule.io.sched_valid
  schedule.io.sched_ready  := sched_ready
  sched_uuid               := schedule.io.sched_uuid
  sched_wid                := schedule.io.sched_wid
  sched_tmask              := schedule.io.sched_tmask
  sched_PC                 := schedule.io.sched_PC
  sched_csr_cycles         := schedule.io.sched_csr_cycles
  sched_csr_active_warps   := schedule.io.sched_csr_active_warps
  sched_csr_thread_masks   := schedule.io.sched_csr_thread_masks
  sched_csr_alm_empty      := schedule.io.sched_csr_alm_empty
  schedule.io.sched_csr_alm_empty_wid := sched_csr_alm_empty_wid
  schedule.io.sched_csr_unlock_wid    := sched_csr_unlock_wid
  schedule.io.sched_csr_unlock_warp   := sched_csr_unlock_warp
  io.busy                  := schedule.io.busy

  // -------------------------------------------------------------------------
  // VX_fetch (translated in Fetch.scala)
  // -------------------------------------------------------------------------
  val fetch = Module(new VxFetch)
  fetch.io.schedule_valid := sched_valid
  sched_ready             := fetch.io.schedule_ready
  fetch.io.schedule_uuid  := sched_uuid
  fetch.io.schedule_wid   := sched_wid
  fetch.io.schedule_tmask := sched_tmask
  fetch.io.schedule_PC    := sched_PC
  // ibuf_pop (non-L1 path) not used in default L1-enabled config
  fetch.io.ibuf_pop       := 0.U

  io.icache_req_valid  := fetch.io.icache_req_valid
  fetch.io.icache_req_ready := io.icache_req_ready
  io.icache_req_rw     := fetch.io.icache_req_rw
  io.icache_req_byteen := fetch.io.icache_req_byteen
  io.icache_req_addr   := fetch.io.icache_req_addr
  io.icache_req_data   := fetch.io.icache_req_data
  io.icache_req_tag    := fetch.io.icache_req_tag
  fetch.io.icache_rsp_valid := io.icache_rsp_valid
  io.icache_rsp_ready  := fetch.io.icache_rsp_ready
  fetch.io.icache_rsp_data  := io.icache_rsp_data
  fetch.io.icache_rsp_tag   := io.icache_rsp_tag

  fetch_valid := fetch.io.fetch_valid
  fetch.io.fetch_ready := fetch_ready
  fetch_uuid  := fetch.io.fetch_uuid
  fetch_wid   := fetch.io.fetch_wid
  fetch_tmask := fetch.io.fetch_tmask
  fetch_PC    := fetch.io.fetch_PC
  fetch_instr := fetch.io.fetch_instr

  // -------------------------------------------------------------------------
  // VX_decode
  // -------------------------------------------------------------------------
  val decode = Module(new VxDecodeBB(instanceId + "-decode"))
  decode.io.fetch_valid := fetch_valid
  fetch_ready           := decode.io.fetch_ready
  decode.io.fetch_uuid  := fetch_uuid
  decode.io.fetch_wid   := fetch_wid
  decode.io.fetch_tmask := fetch_tmask
  decode.io.fetch_PC    := fetch_PC
  decode.io.fetch_instr := fetch_instr
  decode_valid          := decode.io.decode_valid
  decode.io.decode_ready := decode_ready
  decode_uuid           := decode.io.decode_uuid
  decode_wid            := decode.io.decode_wid
  decode_tmask          := decode.io.decode_tmask
  decode_PC             := decode.io.decode_PC
  decode_ex_type        := decode.io.decode_ex_type
  decode_op_type        := decode.io.decode_op_type
  decode_op_args        := decode.io.decode_op_args
  decode_wb             := decode.io.decode_wb
  decode_used_rs        := decode.io.decode_used_rs
  decode_rd             := decode.io.decode_rd
  decode_rs1            := decode.io.decode_rs1
  decode_rs2            := decode.io.decode_rs2
  decode_rs3            := decode.io.decode_rs3
  decode_sched_valid    := decode.io.decode_sched_valid
  decode_sched_unlock   := decode.io.decode_sched_unlock
  decode_sched_wid      := decode.io.decode_sched_wid
  // Feed decode_sched signals into the scheduler
  schedule.io.decode_sched_valid  := decode_sched_valid
  schedule.io.decode_sched_unlock := decode_sched_unlock
  schedule.io.decode_sched_wid    := decode_sched_wid
  decode.io.ibuf_pop    := decode_ibuf_pop

  // -------------------------------------------------------------------------
  // VX_issue
  // -------------------------------------------------------------------------
  val issueStage = Module(new VxIssue(instanceId + "-issue"))
  issueStage.io.decode_valid   := decode_valid
  decode_ready                 := issueStage.io.decode_ready
  issueStage.io.decode_uuid    := decode_uuid
  issueStage.io.decode_wid     := decode_wid
  issueStage.io.decode_tmask   := decode_tmask
  issueStage.io.decode_PC      := decode_PC
  issueStage.io.decode_ex_type := decode_ex_type
  issueStage.io.decode_op_type := decode_op_type
  issueStage.io.decode_op_args := decode_op_args
  issueStage.io.decode_wb      := decode_wb
  issueStage.io.decode_used_rs := decode_used_rs
  issueStage.io.decode_rd      := decode_rd
  issueStage.io.decode_rs1     := decode_rs1
  issueStage.io.decode_rs2     := decode_rs2
  issueStage.io.decode_rs3     := decode_rs3
  decode_ibuf_pop              := issueStage.io.decode_ibuf_pop
  for (i <- 0 until ISSUE_WIDTH) {
    issueStage.io.wb_valid(i)  := wb_valid(i)
    issueStage.io.wb_uuid(i)   := wb_uuid(i)
    issueStage.io.wb_wis(i)    := wb_wis(i)
    issueStage.io.wb_sid(i)    := wb_sid(i)
    issueStage.io.wb_tmask(i)  := wb_tmask(i)
    issueStage.io.wb_PC(i)     := wb_PC(i)
    issueStage.io.wb_rd(i)     := wb_rd(i)
    issueStage.io.wb_data(i)   := wb_data(i)
    issueStage.io.wb_sop(i)    := wb_sop(i)
    issueStage.io.wb_eop(i)    := wb_eop(i)
    issue_sched_valid(i)       := issueStage.io.issue_sched_valid(i)
    issue_sched_wis(i)         := issueStage.io.issue_sched_wis(i)
  }
  for (i <- 0 until numDispatch) {
    dispatch_valid(i)    := issueStage.io.dispatch_valid(i)
    issueStage.io.dispatch_ready(i) := dispatch_ready(i)
    dispatch_uuid(i)     := issueStage.io.dispatch_uuid(i)
    dispatch_wis(i)      := issueStage.io.dispatch_wis(i)
    dispatch_sid(i)      := issueStage.io.dispatch_sid(i)
    dispatch_tmask(i)    := issueStage.io.dispatch_tmask(i)
    dispatch_PC(i)       := issueStage.io.dispatch_PC(i)
    dispatch_op_type(i)  := issueStage.io.dispatch_op_type(i)
    dispatch_op_args(i)  := issueStage.io.dispatch_op_args(i)
    dispatch_wb(i)       := issueStage.io.dispatch_wb(i)
    dispatch_rd(i)       := issueStage.io.dispatch_rd(i)
    dispatch_rs1_data(i) := issueStage.io.dispatch_rs1_data(i)
    dispatch_rs2_data(i) := issueStage.io.dispatch_rs2_data(i)
    dispatch_rs3_data(i) := issueStage.io.dispatch_rs3_data(i)
    dispatch_sop(i)      := issueStage.io.dispatch_sop(i)
    dispatch_eop(i)      := issueStage.io.dispatch_eop(i)
  }

  // -------------------------------------------------------------------------
  // VX_execute
  // -------------------------------------------------------------------------
  val execute = Module(new VxExecute(instanceId + "-execute", coreId))
  execute.io.base_dcrs := base_dcrs
  for (i <- 0 until numDispatch) {
    execute.io.dispatch_valid(i)    := dispatch_valid(i)
    dispatch_ready(i)               := execute.io.dispatch_ready(i)
    execute.io.dispatch_uuid(i)     := dispatch_uuid(i)
    execute.io.dispatch_wis(i)      := dispatch_wis(i)
    execute.io.dispatch_sid(i)      := dispatch_sid(i)
    execute.io.dispatch_tmask(i)    := dispatch_tmask(i)
    execute.io.dispatch_PC(i)       := dispatch_PC(i)
    execute.io.dispatch_op_type(i)  := dispatch_op_type(i)
    execute.io.dispatch_op_args(i)  := dispatch_op_args(i)
    execute.io.dispatch_wb(i)       := dispatch_wb(i)
    execute.io.dispatch_rd(i)       := dispatch_rd(i)
    execute.io.dispatch_rs1_data(i) := dispatch_rs1_data(i)
    execute.io.dispatch_rs2_data(i) := dispatch_rs2_data(i)
    execute.io.dispatch_rs3_data(i) := dispatch_rs3_data(i)
    execute.io.dispatch_sop(i)      := dispatch_sop(i)
    execute.io.dispatch_eop(i)      := dispatch_eop(i)
    commit_valid(i)  := execute.io.commit_valid(i)
    execute.io.commit_ready(i) := commit_ready(i)
    commit_uuid(i)   := execute.io.commit_uuid(i)
    commit_wid(i)    := execute.io.commit_wid(i)
    commit_sid(i)    := execute.io.commit_sid(i)
    commit_tmask(i)  := execute.io.commit_tmask(i)
    commit_PC(i)     := execute.io.commit_PC(i)
    commit_wb(i)     := execute.io.commit_wb(i)
    commit_rd(i)     := execute.io.commit_rd(i)
    commit_data(i)   := execute.io.commit_data(i)
    commit_sop(i)    := execute.io.commit_sop(i)
    commit_eop(i)    := execute.io.commit_eop(i)
  }
  execute.io.sched_csr_cycles        := sched_csr_cycles
  execute.io.sched_csr_active_warps  := sched_csr_active_warps
  execute.io.sched_csr_thread_masks  := sched_csr_thread_masks
  execute.io.sched_csr_alm_empty     := sched_csr_alm_empty
  sched_csr_alm_empty_wid            := execute.io.sched_csr_alm_empty_wid
  sched_csr_unlock_wid               := execute.io.sched_csr_unlock_wid
  sched_csr_unlock_warp              := execute.io.sched_csr_unlock_warp
  execute.io.commit_csr_instret      := commit_csr_instret
  warp_ctl_valid           := execute.io.warp_ctl_valid
  warp_ctl_wid             := execute.io.warp_ctl_wid
  warp_ctl_tmc_valid       := execute.io.warp_ctl_tmc_valid
  warp_ctl_tmc_tmask       := execute.io.warp_ctl_tmc_tmask
  warp_ctl_wspawn_valid    := execute.io.warp_ctl_wspawn_valid
  warp_ctl_wspawn_wmask    := execute.io.warp_ctl_wspawn_wmask
  warp_ctl_wspawn_pc       := execute.io.warp_ctl_wspawn_pc
  warp_ctl_split_valid     := execute.io.warp_ctl_split_valid
  warp_ctl_split_is_dvg    := execute.io.warp_ctl_split_is_dvg
  warp_ctl_split_then_tmask := execute.io.warp_ctl_split_then_tmask
  warp_ctl_split_else_tmask := execute.io.warp_ctl_split_else_tmask
  warp_ctl_split_next_pc   := execute.io.warp_ctl_split_next_pc
  warp_ctl_join_valid      := execute.io.warp_ctl_join_valid
  warp_ctl_join_stack_ptr  := execute.io.warp_ctl_join_stack_ptr
  warp_ctl_barrier_valid   := execute.io.warp_ctl_barrier_valid
  warp_ctl_barrier_id      := execute.io.warp_ctl_barrier_id
  warp_ctl_barrier_is_global := execute.io.warp_ctl_barrier_is_global
  warp_ctl_barrier_size_m1 := execute.io.warp_ctl_barrier_size_m1
  warp_ctl_barrier_is_noop := execute.io.warp_ctl_barrier_is_noop
  warp_ctl_dvstack_wid     := execute.io.warp_ctl_dvstack_wid
  execute.io.warp_ctl_dvstack_ptr := warp_ctl_dvstack_ptr
  for (i <- 0 until NUM_ALU_BLOCKS) {
    branch_ctl_valid(i) := execute.io.branch_ctl_valid(i)
    branch_ctl_wid(i)   := execute.io.branch_ctl_wid(i)
    branch_ctl_taken(i) := execute.io.branch_ctl_taken(i)
    branch_ctl_dest(i)  := execute.io.branch_ctl_dest(i)
  }
  for (i <- 0 until NUM_LSU_BLOCKS) {
    lsu_req_valid(i)  := execute.io.lsu_mem_req_valid(i)
    execute.io.lsu_mem_req_ready(i) := lsu_req_ready(i)
    lsu_req_rw(i)     := execute.io.lsu_mem_req_rw(i)
    lsu_req_mask(i)   := execute.io.lsu_mem_req_mask(i)
    lsu_req_byteen(i) := execute.io.lsu_mem_req_byteen(i)
    lsu_req_addr(i)   := execute.io.lsu_mem_req_addr(i)
    lsu_req_flags(i)  := execute.io.lsu_mem_req_flags(i)
    lsu_req_data(i)   := execute.io.lsu_mem_req_data(i)
    lsu_req_tag(i)    := execute.io.lsu_mem_req_tag(i)
    execute.io.lsu_mem_rsp_valid(i) := lsu_rsp_valid(i)
    lsu_rsp_ready(i)  := execute.io.lsu_mem_rsp_ready(i)
    execute.io.lsu_mem_rsp_mask(i)  := lsu_rsp_mask(i)
    execute.io.lsu_mem_rsp_data(i)  := lsu_rsp_data(i)
    execute.io.lsu_mem_rsp_tag(i)   := lsu_rsp_tag(i)
  }

  // -------------------------------------------------------------------------
  // VX_commit
  // -------------------------------------------------------------------------
  val commit = Module(new VxCommitBB(instanceId + "-commit"))
  for (i <- 0 until numDispatch) {
    commit.io.commit_valid(i) := commit_valid(i)
    commit_ready(i)           := commit.io.commit_ready(i)
    commit.io.commit_uuid(i)  := commit_uuid(i)
    commit.io.commit_wid(i)   := commit_wid(i)
    commit.io.commit_sid(i)   := commit_sid(i)
    commit.io.commit_tmask(i) := commit_tmask(i)
    commit.io.commit_PC(i)    := commit_PC(i)
    commit.io.commit_wb(i)    := commit_wb(i)
    commit.io.commit_rd(i)    := commit_rd(i)
    commit.io.commit_data(i)  := commit_data(i)
    commit.io.commit_sop(i)   := commit_sop(i)
    commit.io.commit_eop(i)   := commit_eop(i)
  }
  for (i <- 0 until ISSUE_WIDTH) {
    wb_valid(i)  := commit.io.wb_valid(i)
    wb_uuid(i)   := commit.io.wb_uuid(i)
    wb_wis(i)    := commit.io.wb_wis(i)
    wb_sid(i)    := commit.io.wb_sid(i)
    wb_tmask(i)  := commit.io.wb_tmask(i)
    wb_PC(i)     := commit.io.wb_PC(i)
    wb_rd(i)     := commit.io.wb_rd(i)
    wb_data(i)   := commit.io.wb_data(i)
    wb_sop(i)    := commit.io.wb_sop(i)
    wb_eop(i)    := commit.io.wb_eop(i)
  }
  commit_csr_instret    := commit.io.commit_csr_instret
  commit_sched_warps    := commit.io.commit_sched_warps

  // -------------------------------------------------------------------------
  // VX_mem_unit
  // -------------------------------------------------------------------------
  val memUnit = Module(new VxMemUnit(instanceId))
  for (i <- 0 until NUM_LSU_BLOCKS) {
    memUnit.io.lsu_req_valid(i)  := lsu_req_valid(i)
    lsu_req_ready(i)             := memUnit.io.lsu_req_ready(i)
    memUnit.io.lsu_req_rw(i)     := lsu_req_rw(i)
    memUnit.io.lsu_req_mask(i)   := lsu_req_mask(i)
    memUnit.io.lsu_req_byteen(i) := lsu_req_byteen(i)
    memUnit.io.lsu_req_addr(i)   := lsu_req_addr(i)
    memUnit.io.lsu_req_flags(i)  := lsu_req_flags(i)
    memUnit.io.lsu_req_data(i)   := lsu_req_data(i)
    memUnit.io.lsu_req_tag(i)    := lsu_req_tag(i)
    lsu_rsp_valid(i)             := memUnit.io.lsu_rsp_valid(i)
    memUnit.io.lsu_rsp_ready(i)  := lsu_rsp_ready(i)
    lsu_rsp_mask(i)              := memUnit.io.lsu_rsp_mask(i)
    lsu_rsp_data(i)              := memUnit.io.lsu_rsp_data(i)
    lsu_rsp_tag(i)               := memUnit.io.lsu_rsp_tag(i)
  }
  for (i <- 0 until DCACHE_NUM_REQS) {
    io.dcache_req_valid(i)       := memUnit.io.dcache_req_valid(i)
    memUnit.io.dcache_req_ready(i) := io.dcache_req_ready(i)
    io.dcache_req_rw(i)          := memUnit.io.dcache_req_rw(i)
    io.dcache_req_byteen(i)      := memUnit.io.dcache_req_byteen(i)
    io.dcache_req_addr(i)        := memUnit.io.dcache_req_addr(i)
    io.dcache_req_flags(i)       := memUnit.io.dcache_req_flags(i)
    io.dcache_req_data(i)        := memUnit.io.dcache_req_data(i)
    io.dcache_req_tag(i)         := memUnit.io.dcache_req_tag(i)
    memUnit.io.dcache_rsp_valid(i) := io.dcache_rsp_valid(i)
    io.dcache_rsp_ready(i)       := memUnit.io.dcache_rsp_ready(i)
    memUnit.io.dcache_rsp_data(i)  := io.dcache_rsp_data(i)
    memUnit.io.dcache_rsp_tag(i)   := io.dcache_rsp_tag(i)
  }
}

// ---------------------------------------------------------------------------
// BlackBox stubs for modules not yet translated
// ---------------------------------------------------------------------------

class VxDcrDataBB extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  val io = IO(new Bundle {
    val dcr_write_valid = Input(Bool())
    val dcr_write_addr  = Input(UInt(VX_DCR_ADDR_WIDTH.W))
    val dcr_write_data  = Input(UInt(32.W))
    val base_dcrs       = Output(new BaseDcrsBundle)
  })
  val inner = Module(new DcrData)
  inner.io.dcr_write_valid := io.dcr_write_valid
  inner.io.dcr_write_addr  := io.dcr_write_addr
  inner.io.dcr_write_data  := io.dcr_write_data
  io.base_dcrs.startup_addr := inner.io.base_dcrs_startup_addr
  io.base_dcrs.startup_arg  := inner.io.base_dcrs_startup_arg
  io.base_dcrs.mpm_class    := inner.io.base_dcrs_mpm_class
}

/** Thin wrapper forwarding flat IO to VxSchedule bundles.
 *  decode_sched_* are Inputs (decode → scheduler), not Outputs. */
class VxScheduleBB(instanceId: String, coreId: Int) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  val io = IO(new Bundle {
    val base_dcrs = Input(new BaseDcrsBundle)
    val warp_ctl_valid            = Input(Bool())
    val warp_ctl_wid              = Input(UInt(NW_WIDTH.W))
    val warp_ctl_tmc_valid        = Input(Bool())
    val warp_ctl_tmc_tmask        = Input(UInt(NUM_THREADS.W))
    val warp_ctl_wspawn_valid     = Input(Bool())
    val warp_ctl_wspawn_wmask     = Input(UInt(NUM_WARPS.W))
    val warp_ctl_wspawn_pc        = Input(UInt(PC_BITS.W))
    val warp_ctl_split_valid      = Input(Bool())
    val warp_ctl_split_is_dvg     = Input(Bool())
    val warp_ctl_split_then_tmask = Input(UInt(NUM_THREADS.W))
    val warp_ctl_split_else_tmask = Input(UInt(NUM_THREADS.W))
    val warp_ctl_split_next_pc    = Input(UInt(PC_BITS.W))
    val warp_ctl_join_valid       = Input(Bool())
    val warp_ctl_join_stack_ptr   = Input(UInt(DV_STACK_SIZEW.W))
    val warp_ctl_barrier_valid    = Input(Bool())
    val warp_ctl_barrier_id       = Input(UInt(NB_WIDTH.W))
    val warp_ctl_barrier_is_global = Input(Bool())
    val warp_ctl_barrier_size_m1  = Input(UInt(NW_WIDTH.W))
    val warp_ctl_barrier_is_noop  = Input(Bool())
    val warp_ctl_dvstack_wid      = Input(UInt(NW_WIDTH.W))
    val warp_ctl_dvstack_ptr      = Output(UInt(DV_STACK_SIZEW.W))
    val branch_ctl_valid = Input(Vec(NUM_ALU_BLOCKS, Bool()))
    val branch_ctl_wid   = Input(Vec(NUM_ALU_BLOCKS, UInt(NW_WIDTH.W)))
    val branch_ctl_taken = Input(Vec(NUM_ALU_BLOCKS, Bool()))
    val branch_ctl_dest  = Input(Vec(NUM_ALU_BLOCKS, UInt(PC_BITS.W)))
    // decode → scheduler (corrected to Input)
    val decode_sched_valid  = Input(Bool())
    val decode_sched_unlock = Input(Bool())
    val decode_sched_wid    = Input(UInt(NW_WIDTH.W))
    val issue_sched_valid   = Input(Vec(ISSUE_WIDTH, Bool()))
    val issue_sched_wis     = Input(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val commit_sched_warps  = Input(UInt(NUM_WARPS.W))
    val sched_valid  = Output(Bool())
    val sched_ready  = Input(Bool())
    val sched_uuid   = Output(UInt(UUID_WIDTH.W))
    val sched_wid    = Output(UInt(NW_WIDTH.W))
    val sched_tmask  = Output(UInt(NUM_THREADS.W))
    val sched_PC     = Output(UInt(PC_BITS.W))
    val sched_csr_cycles       = Output(UInt(PERF_CTR_BITS.W))
    val sched_csr_active_warps = Output(UInt(NUM_WARPS.W))
    val sched_csr_thread_masks = Output(Vec(NUM_WARPS, UInt(NUM_THREADS.W)))
    val sched_csr_alm_empty    = Output(Bool())
    val sched_csr_alm_empty_wid = Input(UInt(NW_WIDTH.W))
    val sched_csr_unlock_wid    = Input(UInt(NW_WIDTH.W))
    val sched_csr_unlock_warp   = Input(Bool())
    val busy = Output(Bool())
  })

  val inner = Module(new VxSchedule(coreId))

  inner.io.base_dcrs := io.base_dcrs

  // warp_ctl (flat → WarpCtlBundle)
  inner.io.warp_ctl.valid                := io.warp_ctl_valid
  inner.io.warp_ctl.wid                  := io.warp_ctl_wid
  inner.io.warp_ctl.tmc.valid            := io.warp_ctl_tmc_valid
  inner.io.warp_ctl.tmc.tmask            := io.warp_ctl_tmc_tmask
  inner.io.warp_ctl.wspawn.valid         := io.warp_ctl_wspawn_valid
  inner.io.warp_ctl.wspawn.wmask         := io.warp_ctl_wspawn_wmask
  inner.io.warp_ctl.wspawn.pc            := io.warp_ctl_wspawn_pc
  inner.io.warp_ctl.split.valid          := io.warp_ctl_split_valid
  inner.io.warp_ctl.split.is_dvg         := io.warp_ctl_split_is_dvg
  inner.io.warp_ctl.split.then_tmask     := io.warp_ctl_split_then_tmask
  inner.io.warp_ctl.split.else_tmask     := io.warp_ctl_split_else_tmask
  inner.io.warp_ctl.split.next_pc        := io.warp_ctl_split_next_pc
  inner.io.warp_ctl.sjoin.valid          := io.warp_ctl_join_valid
  inner.io.warp_ctl.sjoin.stack_ptr      := io.warp_ctl_join_stack_ptr
  inner.io.warp_ctl.barrier.valid        := io.warp_ctl_barrier_valid
  inner.io.warp_ctl.barrier.id           := io.warp_ctl_barrier_id
  inner.io.warp_ctl.barrier.is_global    := io.warp_ctl_barrier_is_global
  inner.io.warp_ctl.barrier.size_m1      := io.warp_ctl_barrier_size_m1
  inner.io.warp_ctl.barrier.is_noop      := io.warp_ctl_barrier_is_noop
  inner.io.warp_ctl.dvstack_wid          := io.warp_ctl_dvstack_wid
  io.warp_ctl_dvstack_ptr                := inner.io.warp_ctl_dvstack_ptr

  // branch_ctl
  for (i <- 0 until NUM_ALU_BLOCKS) {
    inner.io.branch_ctl(i).valid := io.branch_ctl_valid(i)
    inner.io.branch_ctl(i).wid   := io.branch_ctl_wid(i)
    inner.io.branch_ctl(i).taken := io.branch_ctl_taken(i)
    inner.io.branch_ctl(i).dest  := io.branch_ctl_dest(i)
  }

  // decode_sched (decode → scheduler)
  inner.io.decode_sched.valid  := io.decode_sched_valid
  inner.io.decode_sched.unlock := io.decode_sched_unlock
  inner.io.decode_sched.wid    := io.decode_sched_wid

  // issue_sched
  for (i <- 0 until ISSUE_WIDTH) {
    inner.io.issue_sched(i).valid := io.issue_sched_valid(i)
    inner.io.issue_sched(i).wis   := io.issue_sched_wis(i)
  }

  // commit_sched
  inner.io.commit_sched.committed_warps := io.commit_sched_warps

  // schedule output
  io.sched_valid          := inner.io.schedule_valid
  inner.io.schedule_ready := io.sched_ready
  io.sched_uuid           := inner.io.schedule_data.uuid
  io.sched_wid            := inner.io.schedule_data.wid
  io.sched_tmask          := inner.io.schedule_data.tmask
  io.sched_PC             := inner.io.schedule_data.PC

  // sched_csr (mixed direction)
  io.sched_csr_cycles                  := inner.io.sched_csr.cycles
  io.sched_csr_active_warps            := inner.io.sched_csr.active_warps
  io.sched_csr_thread_masks            := inner.io.sched_csr.thread_masks
  io.sched_csr_alm_empty               := inner.io.sched_csr.alm_empty
  inner.io.sched_csr.alm_empty_wid     := io.sched_csr_alm_empty_wid
  inner.io.sched_csr.unlock_wid        := io.sched_csr_unlock_wid
  inner.io.sched_csr.unlock_warp       := io.sched_csr_unlock_warp

  io.busy := inner.io.busy
}

class VxDecodeBB(instanceId: String) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  val io = IO(new Bundle {
    val fetch_valid = Input(Bool())
    val fetch_ready = Output(Bool())
    val fetch_uuid  = Input(UInt(UUID_WIDTH.W))
    val fetch_wid   = Input(UInt(NW_WIDTH.W))
    val fetch_tmask = Input(UInt(NUM_THREADS.W))
    val fetch_PC    = Input(UInt(PC_BITS.W))
    val fetch_instr = Input(UInt(32.W))
    val decode_valid   = Output(Bool())
    val decode_ready   = Input(Bool())
    val decode_uuid    = Output(UInt(UUID_WIDTH.W))
    val decode_wid     = Output(UInt(NW_WIDTH.W))
    val decode_tmask   = Output(UInt(NUM_THREADS.W))
    val decode_PC      = Output(UInt(PC_BITS.W))
    val decode_ex_type = Output(UInt(EX_BITS.W))
    val decode_op_type = Output(UInt(INST_OP_BITS.W))
    val decode_op_args = Output(UInt(INST_ARGS_BITS.W))
    val decode_wb      = Output(Bool())
    val decode_used_rs = Output(UInt(NUM_SRC_OPDS.W))
    val decode_rd      = Output(UInt(NUM_REGS_BITS.W))
    val decode_rs1     = Output(UInt(NUM_REGS_BITS.W))
    val decode_rs2     = Output(UInt(NUM_REGS_BITS.W))
    val decode_rs3     = Output(UInt(NUM_REGS_BITS.W))
    val decode_sched_valid  = Output(Bool())
    val decode_sched_unlock = Output(Bool())
    val decode_sched_wid    = Output(UInt(NW_WIDTH.W))
    val ibuf_pop       = Input(UInt(NUM_WARPS.W))
  })
  val inner = Module(new VxDecode)
  inner.io.fetch_valid      := io.fetch_valid
  io.fetch_ready            := inner.io.fetch_ready
  inner.io.fetch_uuid       := io.fetch_uuid
  inner.io.fetch_wid        := io.fetch_wid
  inner.io.fetch_tmask      := io.fetch_tmask
  inner.io.fetch_PC         := io.fetch_PC
  inner.io.fetch_instr      := io.fetch_instr
  inner.io.decode_ready     := io.decode_ready
  inner.io.decode_ibuf_pop  := io.ibuf_pop
  io.decode_valid           := inner.io.decode_valid
  io.decode_uuid            := inner.io.decode_uuid
  io.decode_wid             := inner.io.decode_wid
  io.decode_tmask           := inner.io.decode_tmask
  io.decode_PC              := inner.io.decode_PC
  io.decode_ex_type         := inner.io.decode_ex_type
  io.decode_op_type         := inner.io.decode_op_type
  io.decode_op_args         := inner.io.decode_op_args
  io.decode_wb              := inner.io.decode_wb
  io.decode_used_rs         := inner.io.decode_used_rs
  io.decode_rd              := inner.io.decode_rd
  io.decode_rs1             := inner.io.decode_rs1
  io.decode_rs2             := inner.io.decode_rs2
  io.decode_rs3             := inner.io.decode_rs3
  io.decode_sched_valid     := inner.io.decode_sched_valid
  io.decode_sched_unlock    := inner.io.decode_sched_unlock
  io.decode_sched_wid       := inner.io.decode_sched_wid
}

class VxCommitBB(instanceId: String) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  private val numDispatch = NUM_EX_UNITS * ISSUE_WIDTH
  val io = IO(new Bundle {
    val commit_valid  = Input(Vec(numDispatch, Bool()))
    val commit_ready  = Output(Vec(numDispatch, Bool()))
    val commit_uuid   = Input(Vec(numDispatch, UInt(UUID_WIDTH.W)))
    val commit_wid    = Input(Vec(numDispatch, UInt(NW_WIDTH.W)))
    val commit_sid    = Input(Vec(numDispatch, UInt(SIMD_IDX_W.W)))
    val commit_tmask  = Input(Vec(numDispatch, UInt(SIMD_WIDTH.W)))
    val commit_PC     = Input(Vec(numDispatch, UInt(PC_BITS.W)))
    val commit_wb     = Input(Vec(numDispatch, Bool()))
    val commit_rd     = Input(Vec(numDispatch, UInt(NUM_REGS_BITS.W)))
    val commit_data   = Input(Vec(numDispatch, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val commit_sop    = Input(Vec(numDispatch, Bool()))
    val commit_eop    = Input(Vec(numDispatch, Bool()))
    val wb_valid  = Output(Vec(ISSUE_WIDTH, Bool()))
    val wb_uuid   = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val wb_wis    = Output(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val wb_sid    = Output(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val wb_tmask  = Output(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val wb_PC     = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val wb_rd     = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val wb_data   = Output(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val wb_sop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val wb_eop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_csr_instret = Output(UInt(PERF_CTR_BITS.W))
    val commit_sched_warps = Output(UInt(NUM_WARPS.W))
  })
  // Forward to the real Commit module.
  // Flat index k = j * ISSUE_WIDTH + i, where j = EU unit, i = issue slot.
  val inner = Module(new Commit)
  for (k <- 0 until numDispatch) {
    val i = k % ISSUE_WIDTH   // issue slot
    val j = k / ISSUE_WIDTH   // EU unit index
    inner.io.commit_in_valid(i)(j) := io.commit_valid(k)
    io.commit_ready(k)             := inner.io.commit_in_ready(i)(j)
    inner.io.commit_in_uuid(i)(j)  := io.commit_uuid(k)
    inner.io.commit_in_wid(i)(j)   := io.commit_wid(k)
    inner.io.commit_in_sid(i)(j)   := io.commit_sid(k)
    inner.io.commit_in_tmask(i)(j) := io.commit_tmask(k)
    inner.io.commit_in_PC(i)(j)    := io.commit_PC(k)
    inner.io.commit_in_wb(i)(j)    := io.commit_wb(k)
    inner.io.commit_in_rd(i)(j)    := io.commit_rd(k)
    inner.io.commit_in_data(i)(j)  := io.commit_data(k)
    inner.io.commit_in_sop(i)(j)   := io.commit_sop(k)
    inner.io.commit_in_eop(i)(j)   := io.commit_eop(k)
  }
  for (i <- 0 until ISSUE_WIDTH) {
    io.wb_valid(i) := inner.io.writeback_valid(i)
    io.wb_uuid(i)  := inner.io.writeback_uuid(i)
    io.wb_wis(i)   := inner.io.writeback_wis(i)
    io.wb_sid(i)   := inner.io.writeback_sid(i)
    io.wb_tmask(i) := inner.io.writeback_tmask(i)
    io.wb_PC(i)    := inner.io.writeback_PC(i)
    io.wb_rd(i)    := inner.io.writeback_rd(i)
    io.wb_data(i)  := inner.io.writeback_data(i)
    io.wb_sop(i)   := inner.io.writeback_sop(i)
    io.wb_eop(i)   := inner.io.writeback_eop(i)
  }
  io.commit_csr_instret := inner.io.commit_csr_instret
  io.commit_sched_warps := inner.io.commit_sched_committed_warps
}
