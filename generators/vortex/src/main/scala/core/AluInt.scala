// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_alu_int.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Integer ALU – purely combinational per-lane arithmetic and logic.
 *
 *  Corresponds to VX_alu_int.sv.
 *
 *  @param numLanes  number of SIMD lanes (NUM_ALU_LANES in the SV)
 *  @param blockIdx  ALU block index (used to derive the wire-backed warp-id for branch output)
 */
class AluInt(
    val numLanes: Int = NUM_ALU_LANES,
    val blockIdx: Int = 0
) extends Module {

  // Derived local parameters
  private val pidBits      = math.max(0, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
  private val pidWidth     = math.max(1, pidBits)
  private val shiftImmBits = log2Ceil(XLEN)           // CLOG2(XLEN) = 5 for RV32

  val io = IO(new Bundle {
    // Execute interface (slave)
    val execute_valid  = Input(Bool())
    val execute_ready  = Output(Bool())
    val execute_uuid   = Input(UInt(UUID_WIDTH.W))
    val execute_wid    = Input(UInt(NW_WIDTH.W))
    val execute_tmask  = Input(UInt(numLanes.W))
    val execute_PC     = Input(UInt(PC_BITS.W))
    val execute_rd     = Input(UInt(NUM_REGS_BITS.W))
    val execute_wb     = Input(Bool())
    val execute_pid    = Input(UInt(pidWidth.W))
    val execute_sop    = Input(Bool())
    val execute_eop    = Input(Bool())
    val execute_op_type = Input(UInt(INST_ALU_BITS.W))
    // op_args interpreted as AluArgsBundle
    val execute_op_args = Input(UInt(INST_ARGS_BITS.W))
    val execute_rs1_data = Input(Vec(numLanes, UInt(XLEN.W)))
    val execute_rs2_data = Input(Vec(numLanes, UInt(XLEN.W)))
    // rs3 unused but present to match interface

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

    // Branch control output (master, registered one cycle)
    val branch_valid = Output(Bool())
    val branch_wid   = Output(UInt(NW_WIDTH.W))
    val branch_taken = Output(Bool())
    val branch_dest  = Output(UInt(PC_BITS.W))
  })

  // -------------------------------------------------------------------------
  // Decode op_args as AluArgsBundle
  // Layout (LSB first): imm[XLEN-1:0], xtype[1:0], is_w, use_imm, use_PC
  // -------------------------------------------------------------------------
  val args      = io.execute_op_args.asTypeOf(new AluArgsBundle)
  val aluOp     = io.execute_op_type
  val brOp      = io.execute_op_type
  val isBrOp    = (args.xtype === ALU_TYPE_BRANCH.U)
  val isSubOp   = inst_alu_is_sub(aluOp)
  val isSigned  = inst_alu_signed(aluOp)

  // op_class: for branch ops use inst_br_class, otherwise inst_alu_class
  val opClass   = Mux(isBrOp, inst_br_class(aluOp), inst_alu_class(aluOp))

  // is_w is always 0 for XLEN=32
  val isAluW    = false.B

  // -------------------------------------------------------------------------
  // Input muxing (PC / immediate substitution)
  // -------------------------------------------------------------------------
  val aluIn1 = io.execute_rs1_data
  val aluIn2 = io.execute_rs2_data

  // Full PC reconstructed from PC_BITS-wide compressed PC (for RV32 PC_BITS=XLEN so no shift)
  val fullPC = io.execute_PC  // for RV32, to_fullPC is identity (PC_BITS == XLEN)

  val aluIn1PC  = VecInit(aluIn1.map(r => Mux(args.use_PC, fullPC, r)))
  val aluIn2Imm = VecInit(aluIn2.map(r => Mux(args.use_imm, args.imm, r)))
  // For branch ops, imm is NOT substituted into the second operand
  val aluIn2Br  = VecInit((aluIn2 zip aluIn2Imm).map { case (r2, r2i) =>
    Mux(args.use_imm && !isBrOp, args.imm, r2)
  })

  // -------------------------------------------------------------------------
  // Per-lane arithmetic
  // -------------------------------------------------------------------------

  // ADD
  val addResult = VecInit(aluIn1PC.zip(aluIn2Imm).map { case (a, b) => a + b })

  // SUB (with +1 guard bit for signed-ness of comparison)
  val subResult = VecInit((aluIn1 zip aluIn2Br).map { case (a, b) =>
    val a_ext = Cat(isSigned & a(XLEN - 1), a)
    val b_ext = Cat(isSigned & b(XLEN - 1), b)
    a_ext - b_ext   // (XLEN+1) bits
  })

  // SHR / CZERO (combined)
  val shrZicResult = VecInit(aluIn1.zipWithIndex.map { case (a, _) =>
    val shrIn1  = Cat(isSigned && a(XLEN - 1), a)  // XLEN+1 bits
    val shAmt   = aluIn2Imm(0)(shiftImmBits - 1, 0) // use lane-0 shift from imm; per-lane same imm
    // Use same imm from aluIn2Imm per lane
    val shAmtL  = aluIn2Imm(0)(shiftImmBits - 1, 0)
    // Per lane shift amount (for SRL/SRA use per-lane aluIn2Imm)
    val perLaneAmt = aluIn2Imm.zipWithIndex.map { case (v, idx) => if (idx == 0) v else v }
    val sa      = aluIn2Imm(0)(shiftImmBits - 1, 0)  // will re-do per lane below
    (shrIn1.asSInt >> aluIn2Imm(0)(shiftImmBits - 1, 0)).asUInt(XLEN - 1, 0)
  })

  // Re-do shrZicResult properly per lane
  val shrResult = VecInit(aluIn1.zipWithIndex.map { case (a, i) =>
    val shrIn1 = Cat(isSigned && a(XLEN - 1), a)
    val sa     = aluIn2Imm(i)(shiftImmBits - 1, 0)
    // Zicond: CZEQ (op[1:0]=2'b10) and CZNE (op[1:0]=2'b11)
    val isZicond = (aluOp(3, 1) === "b101".U)
    val czeqResult = Mux(aluOp(0), Mux(io.execute_rs2_data(i).orR, 0.U, a),
                                   Mux(io.execute_rs2_data(i).orR, a, 0.U))
    val sraResult  = (shrIn1.asSInt >> sa).asUInt(XLEN - 1, 0)
    Mux(isZicond, czeqResult, sraResult)
  })

  // MSC: AND/OR/XOR/SLL
  val mscResult = VecInit((aluIn1 zip aluIn2Imm).zipWithIndex.map { case ((a, b), _) =>
    val sa = b(shiftImmBits - 1, 0)
    MuxLookup(aluOp(1, 0), 0.U(XLEN.W))(Seq(
      "b00".U -> (a & b),
      "b01".U -> (a | b),
      "b10".U -> (a ^ b),
      "b11".U -> (a << sa)(XLEN - 1, 0)
    ))
  })

  // -------------------------------------------------------------------------
  // VOTE instructions
  // -------------------------------------------------------------------------
  val voteTrue  = VecInit(aluIn1.zipWithIndex.map { case (a, i) =>
    io.execute_tmask(i) && a(0)
  })
  val voteFalse = VecInit(aluIn1.zipWithIndex.map { case (a, i) =>
    io.execute_tmask(i) && !a(0)
  })
  val hasVoteTrue  = voteTrue.asUInt.orR
  val hasVoteFalse = voteFalse.asUInt.orR
  val voteAll      = !hasVoteFalse
  val voteAny      = hasVoteTrue
  val voteNone     = !hasVoteTrue
  val voteUni      = voteAll || voteNone

  val voteResult = VecInit(Seq.tabulate(numLanes) { _ =>
    MuxLookup(aluOp(1, 0), 0.U(XLEN.W))(Seq(
      INST_VOTE_ALL.U -> voteAll.asUInt,
      INST_VOTE_ANY.U -> voteAny.asUInt,
      INST_VOTE_UNI.U -> voteUni.asUInt,
      INST_VOTE_BAL.U -> voteTrue.asUInt
    ))
  })

  // -------------------------------------------------------------------------
  // SHFL instructions (only meaningful when numLanes > 1)
  // -------------------------------------------------------------------------
  val shflResult: Vec[UInt] = if (numLanes > 1) {
    val laneBits = log2Ceil(numLanes)
    VecInit(Seq.tabulate(numLanes) { i =>
      val bval    = aluIn2(i)(laneBits - 1, 0)
      val cval    = aluIn2(i)(6 + laneBits - 1, 6)
      val mask    = aluIn2(i)(12 + laneBits - 1, 12)
      val minLane = i.U(laneBits.W) & mask
      val maxLane = minLane | (cval & ~mask)

      val laneUp   = i.U(laneBits.W) -% bval
      val laneDown = i.U(laneBits.W) +% bval
      val laneBfly = i.U(laneBits.W) ^ bval
      val laneIdx  = minLane | (bval & ~mask)

      val laneReg = Wire(UInt(laneBits.W))
      laneReg := i.U(laneBits.W)   // default: self

      switch (aluOp(1, 0)) {
        is (INST_SHFL_UP.U) {
          when (laneUp.asSInt >= minLane.asSInt) {
            laneReg := laneUp
          }
        }
        is (INST_SHFL_DOWN.U) {
          when (laneDown <= maxLane) {
            laneReg := laneDown
          }
        }
        is (INST_SHFL_BFLY.U) {
          when (laneBfly <= maxLane) {
            laneReg := laneBfly
          }
        }
        is (INST_SHFL_IDX.U) {
          when (laneIdx <= maxLane) {
            laneReg := laneIdx
          }
        }
      }

      Mux(io.execute_tmask(laneReg), aluIn1(laneReg), aluIn1(i))
    })
  } else {
    VecInit(Seq(aluIn1(0)))
  }

  // -------------------------------------------------------------------------
  // Final ALU result mux
  // -------------------------------------------------------------------------
  val aluResult = VecInit(Seq.tabulate(numLanes) { i =>
    val subFull      = subResult(i)                               // XLEN+1 bits
    val sltBrResult  = Cat(
      isBrOp && !subFull(XLEN - 1, 0).orR,  // equal flag
      subFull(XLEN)                           // less-than (sign of signed difference)
    ).asUInt
    val subSltBrResult = Mux(isSubOp && !isBrOp, subFull(XLEN - 1, 0), sltBrResult)

    val normalResult = MuxLookup(opClass, 0.U(XLEN.W))(Seq(
      "b00".U -> addResult(i),        // ADD, LUI, AUIPC
      "b01".U -> subSltBrResult,      // SUB, SLT, SLTU, branches
      "b10".U -> shrResult(i),        // SRL, SRA, SRLI, SRAI, CZERO
      "b11".U -> mscResult(i)         // AND, OR, XOR, SLL
    ))

    Mux(args.xtype === ALU_TYPE_OTHER.U,
      Mux(aluOp(2), shflResult(i), voteResult(i)),
      normalResult
    )
  })

  // -------------------------------------------------------------------------
  // Elastic buffer (mirrors VX_elastic_buffer): 2-entry FIFO / skid buffer
  // We model this as a 1-stage pipe register with valid/ready handshake.
  // -------------------------------------------------------------------------
  // Registered pipeline stage
  val regValid   = RegInit(false.B)
  val regUuid    = Reg(UInt(UUID_WIDTH.W))
  val regWid     = Reg(UInt(NW_WIDTH.W))
  val regTmask   = Reg(UInt(numLanes.W))
  val regRd      = Reg(UInt(NUM_REGS_BITS.W))
  val regWb      = Reg(Bool())
  val regPid     = Reg(UInt(pidWidth.W))
  val regSop     = Reg(Bool())
  val regEop     = Reg(Bool())
  val regData    = Reg(Vec(numLanes, UInt(XLEN.W)))
  val regPC      = Reg(UInt(PC_BITS.W))
  val regCbrDest = Reg(UInt(PC_BITS.W))
  val regIsBr    = Reg(Bool())
  val regBrOp    = Reg(UInt(INST_BR_BITS.W))
  val regLastTid = Reg(UInt(math.max(1, log2Ceil(numLanes)).W))

  val outReady = io.result_ready
  val canAccept = !regValid || outReady

  io.execute_ready := canAccept

  // Last active thread ID (highest set bit of tmask)
  val lastTid = Wire(UInt(math.max(1, log2Ceil(numLanes)).W))
  if (numLanes > 1) {
    val revMask = Reverse(io.execute_tmask)
    lastTid := (numLanes - 1).U - PriorityEncoder(revMask)
  } else {
    lastTid := 0.U
  }

  // cbr_dest: branch target from ADD result lane 0 (for conditional branch)
  val cbrDest = addResult(0)(PC_BITS - 1, 0)

  when (io.execute_valid && canAccept) {
    regValid   := true.B
    regUuid    := io.execute_uuid
    regWid     := io.execute_wid
    regTmask   := io.execute_tmask
    regRd      := io.execute_rd
    regWb      := io.execute_wb
    regPid     := io.execute_pid
    regSop     := io.execute_sop
    regEop     := io.execute_eop
    regData    := aluResult
    regPC      := io.execute_PC
    regCbrDest := cbrDest
    regIsBr    := isBrOp
    regBrOp    := brOp
    regLastTid := lastTid
  } .elsewhen (outReady) {
    regValid := false.B
  }

  // -------------------------------------------------------------------------
  // Result outputs
  // -------------------------------------------------------------------------
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

  // For static branches (JAL/JALR/ECALL/etc.) the writeback data is PC+4
  val pcNext = (regPC + 4.U)(XLEN - 1, 0)
  io.result_data := VecInit(regData.map(d =>
    Mux(regIsBr && inst_br_is_static(regBrOp), pcNext, d)
  ))

  // -------------------------------------------------------------------------
  // Branch resolution
  // -------------------------------------------------------------------------
  val resultFire  = regValid && io.result_ready
  val brEnable    = resultFire && regIsBr && regEop

  val isBrNeg    = inst_br_is_neg(regBrOp)
  val isBrLess   = inst_br_is_less(regBrOp)
  val isBrStatic = inst_br_is_static(regBrOp)

  val brResult   = regData(regLastTid)
  val isLess     = brResult(0)
  val isEqual    = brResult(1)

  val brTaken = ((Mux(isBrLess, isLess, isEqual) ^ isBrNeg) | isBrStatic)
  // For static branches dest comes from data result[0]; for conditional, from cbr_dest_r
  val brDest  = Mux(isBrStatic, regData(0)(PC_BITS - 1, 0), regCbrDest)

  // Compute block-adjusted warp id: assign_blocked_wid
  // br_wid = result_wid with upper bits from block_idx
  val numAluBlocks = NUM_ALU_BLOCKS
  val brWid = if (numAluBlocks > 1) {
    Cat(io.result_wid(NW_WIDTH - 1, log2Ceil(numAluBlocks)),
        blockIdx.U(log2Ceil(numAluBlocks).W))
  } else {
    io.result_wid
  }

  // Registered branch output (VX_pipe_register with RESETW=1 on valid)
  val branchValid = RegNext(brEnable,         init = false.B)
  val branchWid   = RegNext(brWid)
  val branchTaken = RegNext(brTaken)
  val branchDest  = RegNext(brDest)

  io.branch_valid := branchValid
  io.branch_wid   := branchWid
  io.branch_taken := branchTaken
  io.branch_dest  := branchDest
}
