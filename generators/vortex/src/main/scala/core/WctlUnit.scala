// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_wctl_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Warp Control Unit.
 *
 *  Corresponds to VX_wctl_unit.sv.
 *
 *  Decodes SFU warp-control instructions (TMC, WSPAWN, SPLIT, JOIN, BAR, PRED)
 *  and drives the warp_ctl_if signals.  Results are forwarded through a
 *  2-entry elastic buffer.
 *
 *  @param numLanes number of SIMD lanes
 */
class WctlUnit(val numLanes: Int = NUM_SFU_LANES) extends Module {

  private val pidBits      = math.max(0, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
  private val pidWidth     = math.max(1, pidBits)

  val io = IO(new Bundle {
    // Execute interface (slave)
    val execute_valid    = Input(Bool())
    val execute_ready    = Output(Bool())
    val execute_uuid     = Input(UInt(UUID_WIDTH.W))
    val execute_wid      = Input(UInt(NW_WIDTH.W))
    val execute_tmask    = Input(UInt(numLanes.W))
    val execute_PC       = Input(UInt(PC_BITS.W))
    val execute_rd       = Input(UInt(NUM_REGS_BITS.W))
    val execute_wb       = Input(Bool())
    val execute_pid      = Input(UInt(pidWidth.W))
    val execute_sop      = Input(Bool())
    val execute_eop      = Input(Bool())
    val execute_op_type  = Input(UInt(INST_SFU_BITS.W))
    val execute_op_args  = Input(UInt(INST_ARGS_BITS.W))  // wctl_args_t: is_neg
    val execute_rs1_data = Input(Vec(numLanes, UInt(XLEN.W)))
    val execute_rs2_data = Input(Vec(numLanes, UInt(XLEN.W)))

    // Result interface (master)
    val result_valid = Output(Bool())
    val result_ready = Input(Bool())
    val result_uuid  = Output(UInt(UUID_WIDTH.W))
    val result_wid   = Output(UInt(NW_WIDTH.W))
    val result_tmask = Output(UInt(numLanes.W))
    val result_PC    = Output(UInt(PC_BITS.W))
    val result_rd    = Output(UInt(NUM_REGS_BITS.W))
    val result_wb    = Output(Bool())
    val result_pid   = Output(UInt(pidWidth.W))
    val result_sop   = Output(Bool())
    val result_eop   = Output(Bool())
    val result_data  = Output(Vec(numLanes, UInt(XLEN.W)))

    // Warp control interface (master push, driven combinatorially then registered)
    val warp_ctl_valid       = Output(Bool())
    val warp_ctl_wid         = Output(UInt(NW_WIDTH.W))
    // tmc
    val warp_ctl_tmc_valid   = Output(Bool())
    val warp_ctl_tmc_tmask   = Output(UInt(NUM_THREADS.W))
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
    val warp_ctl_bar_valid    = Output(Bool())
    val warp_ctl_bar_id       = Output(UInt(NB_WIDTH.W))
    val warp_ctl_bar_is_global = Output(Bool())
    val warp_ctl_bar_size_m1  = Output(UInt(NW_WIDTH.W))
    val warp_ctl_bar_is_noop  = Output(Bool())
    // dvstack (bidirectional: wid driven by us, ptr driven back by scheduler)
    val warp_ctl_dvstack_wid  = Output(UInt(NW_WIDTH.W))
    val warp_ctl_dvstack_ptr  = Input(UInt(DV_STACK_SIZEW.W))
  })

  // -------------------------------------------------------------------------
  // Decode op type
  // -------------------------------------------------------------------------
  val opType  = io.execute_op_type
  val isWspawn = (opType === INST_SFU_WSPAWN.U)
  val isTmc    = (opType === INST_SFU_TMC.U)
  val isPred   = (opType === INST_SFU_PRED.U)
  val isSplit  = (opType === INST_SFU_SPLIT.U)
  val isJoin   = (opType === INST_SFU_JOIN.U)
  val isBar    = (opType === INST_SFU_BAR.U)

  // -------------------------------------------------------------------------
  // Last active thread ID (highest set bit in tmask)
  // -------------------------------------------------------------------------
  val lastTid = Wire(UInt(math.max(1, log2Ceil(numLanes)).W))
  if (numLanes > 1) {
    val revMask = Reverse(io.execute_tmask)
    lastTid := (numLanes - 1).U - PriorityEncoder(revMask)
  } else {
    lastTid := 0.U
  }

  val rs1Data = io.execute_rs1_data(lastTid)
  val rs2Data = io.execute_rs2_data(lastTid)

  // is_neg from wctl_args_t (lowest bit of raw op_args)
  val notPred  = io.execute_op_args(0)

  // -------------------------------------------------------------------------
  // Per-lane "taken" for SPLIT / PRED
  // -------------------------------------------------------------------------
  val taken = VecInit(io.execute_rs1_data.map(r => r(0) ^ notPred))

  // -------------------------------------------------------------------------
  // Divergence then/else masks
  // When PID_BITS != 0, accumulate across SIMD groups.
  // For simplicity we track a per-warp table of 2*NUM_THREADS bits.
  // -------------------------------------------------------------------------
  val thenTmask = Wire(UInt(NUM_THREADS.W))
  val elseTmask = Wire(UInt(NUM_THREADS.W))

  if (pidBits != 0) {
    // Table indexed by warp id: stores {else_tmask, then_tmask}
    val tmaskTable = RegInit(VecInit(Seq.fill(NUM_WARPS)(0.U((2 * NUM_THREADS).W))))
    val tmaskR     = tmaskTable(io.execute_wid)

    val thenAccum = Wire(UInt(NUM_THREADS.W))
    val elseAccum = Wire(UInt(NUM_THREADS.W))
    // When sop, start fresh; otherwise accumulate
    val baseVal = Mux(io.execute_sop, 0.U((2 * NUM_THREADS).W), tmaskR)
    thenAccum := baseVal(NUM_THREADS - 1, 0)
    elseAccum := baseVal(2 * NUM_THREADS - 1, NUM_THREADS)

    // Write the PID group
    val pidBase     = io.execute_pid * numLanes.U
    val takenBits   = (taken.asUInt & io.execute_tmask)(numLanes - 1, 0)
    val notTakenBits = (~taken.asUInt & io.execute_tmask)(numLanes - 1, 0)

    val thenNew = Wire(UInt(NUM_THREADS.W))
    val elseNew = Wire(UInt(NUM_THREADS.W))
    thenNew := thenAccum
    elseNew := elseAccum
    // Insert lane results into the pid-group slice
    // Use a bit-mask insertion
    val fullTaken    = (takenBits.asUInt << pidBase)(NUM_THREADS - 1, 0)
    val fullNotTaken = (notTakenBits.asUInt << pidBase)(NUM_THREADS - 1, 0)
    val pidMask      = ((1.U << numLanes.U) - 1.U) << pidBase
    thenTmask := (thenNew & ~pidMask) | fullTaken
    elseTmask := (elseNew & ~pidMask) | fullNotTaken

    when (io.execute_valid) {
      tmaskTable(io.execute_wid) := Cat(elseTmask, thenTmask)
    }
  } else {
    thenTmask := (taken.asUInt & io.execute_tmask)(NUM_THREADS - 1, 0)
    elseTmask := (~taken.asUInt & io.execute_tmask)(NUM_THREADS - 1, 0)
  }

  val hasThen  = thenTmask.orR
  val hasElse  = elseTmask.orR

  // -------------------------------------------------------------------------
  // TMC / PRED
  // -------------------------------------------------------------------------
  val predMask = Mux(hasThen, thenTmask, rs2Data(NUM_THREADS - 1, 0))
  val tmcValid = isTmc || isPred
  val tmcTmask = Mux(isPred, predMask, rs1Data(NUM_THREADS - 1, 0))

  // -------------------------------------------------------------------------
  // SPLIT – pick then_tmask as "larger" group
  // -------------------------------------------------------------------------
  val thenCnt  = PopCount(thenTmask)
  val elseCnt  = PopCount(elseTmask)
  val thenFirst = thenCnt >= elseCnt
  val takenTmask   = Mux(thenFirst, thenTmask, elseTmask)
  val ntakenTmask  = Mux(thenFirst, elseTmask, thenTmask)

  val splitNextPC = (io.execute_PC + 4.U)(PC_BITS - 1, 0)  // PC+4 (from_fullPC for RV32 = identity)

  // -------------------------------------------------------------------------
  // JOIN
  // -------------------------------------------------------------------------
  val joinStackPtr = rs1Data(DV_STACK_SIZEW - 1, 0)

  // -------------------------------------------------------------------------
  // BARRIER
  // -------------------------------------------------------------------------
  val barSize_m1 = rs2Data(NW_WIDTH - 1, 0) - 1.U
  val barIsNoop  = (rs2Data(NW_WIDTH - 1, 0) === 1.U)

  // -------------------------------------------------------------------------
  // WSPAWN
  // -------------------------------------------------------------------------
  val wspawnWmask = VecInit(Seq.tabulate(NUM_WARPS) { i =>
    (i.U < rs1Data(log2Ceil(NUM_WARPS), 0)) && (i.U =/= io.execute_wid)
  }).asUInt
  val wspawnPC = rs2Data(PC_BITS - 1, 0)

  // -------------------------------------------------------------------------
  // dvstack_wid – driven immediately (registered in the SV; here we pass through)
  // -------------------------------------------------------------------------
  io.warp_ctl_dvstack_wid := io.execute_wid

  // -------------------------------------------------------------------------
  // 2-entry elastic buffer (execute → result)
  // -------------------------------------------------------------------------
  val regValid   = RegInit(false.B)
  val regUuid    = Reg(UInt(UUID_WIDTH.W))
  val regWid     = Reg(UInt(NW_WIDTH.W))
  val regTmask   = Reg(UInt(numLanes.W))
  val regPC      = Reg(UInt(PC_BITS.W))
  val regRd      = Reg(UInt(NUM_REGS_BITS.W))
  val regWb      = Reg(Bool())
  val regPid     = Reg(UInt(pidWidth.W))
  val regSop     = Reg(Bool())
  val regEop     = Reg(Bool())
  // dvstack_ptr captured at buffer input
  val regDvstack = Reg(UInt(DV_STACK_SIZEW.W))

  val canAccept = !regValid || io.result_ready
  io.execute_ready := canAccept

  when (io.execute_valid && canAccept) {
    regValid   := true.B
    regUuid    := io.execute_uuid
    regWid     := io.execute_wid
    regTmask   := io.execute_tmask
    regPC      := io.execute_PC
    regRd      := io.execute_rd
    regWb      := io.execute_wb
    regPid     := io.execute_pid
    regSop     := io.execute_sop
    regEop     := io.execute_eop
    regDvstack := io.warp_ctl_dvstack_ptr
  } .elsewhen (io.result_ready) {
    regValid := false.B
  }

  io.result_valid := regValid
  io.result_uuid  := regUuid
  io.result_wid   := regWid
  io.result_tmask := regTmask
  io.result_PC    := regPC
  io.result_rd    := regRd
  io.result_wb    := regWb
  io.result_pid   := regPid
  io.result_sop   := regSop
  io.result_eop   := regEop
  // Result data = dvstack_ptr broadcast to all lanes
  io.result_data  := VecInit(Seq.fill(numLanes)(regDvstack.pad(XLEN)))

  // -------------------------------------------------------------------------
  // Warp control outputs (registered via VX_pipe_register, RESETW=1 on valid)
  // -------------------------------------------------------------------------
  val executeFire = io.execute_valid && io.execute_ready
  val wctlFire    = executeFire && io.execute_eop

  val regCtlValid         = RegNext(wctlFire,       init = false.B)
  val regCtlWid           = RegNext(io.execute_wid)
  // tmc
  val regTmcValid         = RegNext(tmcValid)
  val regTmcTmask         = RegNext(tmcTmask)
  // wspawn
  val regWspawnValid      = RegNext(isWspawn)
  val regWspawnWmask      = RegNext(wspawnWmask)
  val regWspawnPC         = RegNext(wspawnPC)
  // split
  val regSplitValid       = RegNext(isSplit)
  val regSplitIsDvg       = RegNext(hasThen && hasElse)
  val regSplitThenTmask   = RegNext(takenTmask)
  val regSplitElseTmask   = RegNext(ntakenTmask)
  val regSplitNextPC      = RegNext(splitNextPC)
  // join
  val regJoinValid        = RegNext(isJoin)
  val regJoinStackPtr     = RegNext(joinStackPtr)
  // barrier
  val regBarValid         = RegNext(isBar)
  val regBarId            = RegNext(rs1Data(NB_WIDTH - 1, 0))
  val regBarIsGlobal      = RegNext(false.B)  // GBAR_ENABLE not set
  val regBarSizeM1        = RegNext(barSize_m1)
  val regBarIsNoop        = RegNext(barIsNoop)

  io.warp_ctl_valid            := regCtlValid
  io.warp_ctl_wid              := regCtlWid
  io.warp_ctl_tmc_valid        := regTmcValid
  io.warp_ctl_tmc_tmask        := regTmcTmask
  io.warp_ctl_wspawn_valid     := regWspawnValid
  io.warp_ctl_wspawn_wmask     := regWspawnWmask
  io.warp_ctl_wspawn_pc        := regWspawnPC
  io.warp_ctl_split_valid      := regSplitValid
  io.warp_ctl_split_is_dvg     := regSplitIsDvg
  io.warp_ctl_split_then_tmask := regSplitThenTmask
  io.warp_ctl_split_else_tmask := regSplitElseTmask
  io.warp_ctl_split_next_pc    := regSplitNextPC
  io.warp_ctl_join_valid       := regJoinValid
  io.warp_ctl_join_stack_ptr   := regJoinStackPtr
  io.warp_ctl_bar_valid        := regBarValid
  io.warp_ctl_bar_id           := regBarId
  io.warp_ctl_bar_is_global    := regBarIsGlobal
  io.warp_ctl_bar_size_m1      := regBarSizeM1
  io.warp_ctl_bar_is_noop      := regBarIsNoop
}
