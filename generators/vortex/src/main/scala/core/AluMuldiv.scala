// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_alu_muldiv.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_alu_muldiv: Integer Multiply-Divide unit.
 *
 *  Translates the synthesisable (non-DPI) code path from VX_alu_muldiv.sv.
 *  IMUL_DPI and IDIV_DPI blocks are intentionally omitted.
 *  Only the RV32 (non-XLEN_64) path is implemented.
 *
 *  Sub-modules used:
 *    VxMultiplier     (one per lane, LATENCY_IMUL=3 stage pipeline)
 *    VxShiftRegister  (propagates mul valid+tag+flags)
 *    VXElasticAdapter (div valid/ready ↔ serial div strobe/busy)
 *    VxSerialDiv      (iterative divider, one per lane)
 *    VxStreamArb      (2-input priority arbitration into result_if)
 *
 *  @param numLanes number of SIMD lanes
 */
class AluMuldiv(val numLanes: Int = NUM_ALU_LANES) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val pidBits  = log2Ceil(math.max(1, NUM_THREADS / numLanes))
  private val pidWidth = math.max(1, pidBits)
  // LATENCY_IMUL = 3 (from VX_platform.vh)
  private val LATENCY_IMUL = 3
  // TAG_WIDTH = UUID_WIDTH + NW_WIDTH + numLanes + PC_BITS + NUM_REGS_BITS + 1(wb) + pidWidth + 1(sop) + 1(eop)
  private val tagW = UUID_WIDTH + NW_WIDTH + numLanes + PC_BITS + NUM_REGS_BITS + 1 + pidWidth + 1 + 1

  // -------------------------------------------------------------------------
  // Mul shift-register data layout (packed from LSB):
  //   [0]                          isAluW  (always 0 for RV32)
  //   [1]                          isMulh
  //   [2]                          eop
  //   [3]                          sop
  //   [3+pidWidth : 4]             pid
  //   [4+pidWidth]                 wb
  //   [4+pidWidth+NUM_REGS_BITS-1 : 5+pidWidth]  rd  (wait, let me redo)
  // To avoid errors, use cumulative offset constants:
  private val M_ISALUW = 0
  private val M_ISMULH = M_ISALUW + 1
  private val M_EOP    = M_ISMULH + 1
  private val M_SOP    = M_EOP    + 1
  private val M_PID    = M_SOP    + 1
  private val M_WB     = M_PID    + pidWidth
  private val M_RD     = M_WB     + 1
  private val M_PC     = M_RD     + NUM_REGS_BITS
  private val M_TMASK  = M_PC     + PC_BITS
  private val M_WID    = M_TMASK  + numLanes
  private val M_UUID   = M_WID    + NW_WIDTH
  private val M_VALID  = M_UUID   + UUID_WIDTH
  // Total shift-reg width: M_VALID + 1 = 1 + tagW + 2
  private val mulShiftW = M_VALID + 1

  // -------------------------------------------------------------------------
  // Div tag-register data layout (packed from LSB, no valid bit):
  //   [0]                          eop
  //   [1]                          sop
  //   [1+pidWidth : 2]             pid
  //   [2+pidWidth]                 isAluW  (always 0 for RV32)
  //   [3+pidWidth]                 isRem
  //   [4+pidWidth]                 wb
  //   [4+pidWidth+NUM_REGS_BITS-1 : 5+pidWidth]  rd
  //   ... PC, tmask, wid, uuid
  private val D_EOP    = 0
  private val D_SOP    = D_EOP    + 1
  private val D_PID    = D_SOP    + 1
  private val D_ISALUW = D_PID    + pidWidth
  private val D_ISREM  = D_ISALUW + 1
  private val D_WB     = D_ISREM  + 1
  private val D_RD     = D_WB     + 1
  private val D_PC     = D_RD     + NUM_REGS_BITS
  private val D_TMASK  = D_PC     + PC_BITS
  private val D_WID    = D_TMASK  + numLanes
  private val D_UUID   = D_WID    + NW_WIDTH
  // Total: D_UUID + UUID_WIDTH = tagW + 2
  private val divTagW = D_UUID + UUID_WIDTH

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
    val execute_op_type  = Input(UInt(INST_M_BITS.W))
    val execute_op_args  = Input(UInt(INST_ARGS_BITS.W))
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
  })

  // -------------------------------------------------------------------------
  // Decode M-extension op
  // -------------------------------------------------------------------------
  val muldivOp   = io.execute_op_type
  val isMulxOp   = inst_m_is_mulx(muldivOp)
  val isSignedOp = inst_m_signed(muldivOp)
  val isMulhIn   = inst_m_is_mulh(muldivOp)
  val isSignedA  = inst_m_signed_a(muldivOp)
  val isSignedB  = isSignedOp
  val isRemOp    = inst_m_is_rem(muldivOp)
  val isAluW     = false.B   // RV32: no W-type variants

  val mulValidIn = io.execute_valid && isMulxOp
  val divValidIn = io.execute_valid && !isMulxOp

  // =========================================================================
  // Multiply path  (non-DPI, non-XLEN_64)
  //
  //   One VxMultiplier per lane, pipelined at LATENCY_IMUL stages.
  //   VxShiftRegister propagates valid + tag + {isMulh, isAluW}.
  //   mul_ready_in = mul_ready_out || ~mul_valid_out
  // =========================================================================

  val mulReadyOut = Wire(Bool())   // driven by arbiter below
  val mulValidOut = Wire(Bool())   // driven by shift-register output
  val mulReadyIn  = !mulValidOut || mulReadyOut

  // Per-lane product results (2*(XLEN+1) bits each)
  val mulResultTmp = Wire(Vec(numLanes, UInt((2 * (XLEN + 1)).W)))
  for (i <- 0 until numLanes) {
    val mulIn1 = Cat(isSignedA && io.execute_rs1_data(i)(XLEN - 1), io.execute_rs1_data(i))
    val mulIn2 = Cat(isSignedB && io.execute_rs2_data(i)(XLEN - 1), io.execute_rs2_data(i))
    val mult = Module(new VxMultiplier(
      aWidth   = XLEN + 1,
      bWidth   = XLEN + 1,
      rWidth   = 2 * (XLEN + 1),
      isSigned = true,
      latency  = LATENCY_IMUL
    ))
    mult.io.enable := mulReadyIn
    mult.io.dataa  := mulIn1
    mult.io.datab  := mulIn2
    mulResultTmp(i) := mult.io.result
  }

  // Shift register: DATAW = mulShiftW, DEPTH = LATENCY_IMUL, RESETW = 1
  val mulShiftReg = Module(new VxShiftRegister(
    dataw  = mulShiftW,
    resetw = 1,
    depth  = LATENCY_IMUL
  ))
  mulShiftReg.io.enable := mulReadyIn
  // Pack input (MSB first in Cat = MSB in result)
  mulShiftReg.io.dataIn := Cat(
    mulValidIn,
    io.execute_uuid, io.execute_wid, io.execute_tmask, io.execute_PC,
    io.execute_rd, io.execute_wb, io.execute_pid, io.execute_sop, io.execute_eop,
    isMulhIn, isAluW
  )

  val mulShiftOut = mulShiftReg.io.dataOut(0)
  mulValidOut := mulShiftOut(M_VALID)

  val mulUuidOut  = mulShiftOut(M_UUID  + UUID_WIDTH    - 1, M_UUID)
  val mulWidOut   = mulShiftOut(M_WID   + NW_WIDTH      - 1, M_WID)
  val mulTmaskOut = mulShiftOut(M_TMASK + numLanes      - 1, M_TMASK)
  val mulPCOut    = mulShiftOut(M_PC    + PC_BITS       - 1, M_PC)
  val mulRdOut    = mulShiftOut(M_RD    + NUM_REGS_BITS - 1, M_RD)
  val mulWbOut    = mulShiftOut(M_WB)
  val mulPidOut   = mulShiftOut(M_PID   + pidWidth      - 1, M_PID)
  val mulSopOut   = mulShiftOut(M_SOP)
  val mulEopOut   = mulShiftOut(M_EOP)
  val isMulhOut   = mulShiftOut(M_ISMULH)

  // Select high or low half of product
  // SV: is_mulh_out ? result_tmp[2*XLEN-1:XLEN] : result_tmp[XLEN-1:0]
  val mulResultOut = VecInit(mulResultTmp.map { prod =>
    Mux(isMulhOut,
      prod(2 * XLEN - 1, XLEN),   // MULH: upper XLEN bits
      prod(XLEN - 1, 0)            // MUL:  lower XLEN bits
    )
  })

  // =========================================================================
  // Divide path  (non-DPI)
  //
  //   VXElasticAdapter decouples the valid/ready handshake from the
  //   multi-cycle VxSerialDiv.  A tag register captures metadata on fire.
  // =========================================================================

  val divReadyOut = Wire(Bool())   // driven by arbiter below
  val divValidOut = Wire(Bool())
  val divReadyIn  = Wire(Bool())

  val divElastic = Module(new VXElasticAdapter)
  divElastic.io.valid_in  := divValidIn
  divReadyIn              := divElastic.io.ready_in
  divElastic.io.ready_out := divReadyOut
  divValidOut             := divElastic.io.valid_out

  val divSerial = Module(new VxSerialDiv(
    widthn = XLEN,
    widthd = XLEN,
    widthq = XLEN,
    widthr = XLEN,
    lanes  = numLanes
  ))
  divSerial.io.strobe   := divElastic.io.strobe
  divElastic.io.busy    := divSerial.io.busy
  divSerial.io.isSigned := isSignedOp
  for (i <- 0 until numLanes) {
    divSerial.io.numer(i) := io.execute_rs1_data(i)
    divSerial.io.denom(i) := io.execute_rs2_data(i)
  }

  // Tag register (captures on div fire)
  // SV: {uuid, wid, tmask, PC, rd, wb, is_rem_op, is_alu_w, pid, sop, eop}
  val divTagReg = Reg(UInt(divTagW.W))
  when(divValidIn && divReadyIn) {
    divTagReg := Cat(
      io.execute_uuid, io.execute_wid, io.execute_tmask, io.execute_PC,
      io.execute_rd, io.execute_wb, isRemOp, isAluW,
      io.execute_pid, io.execute_sop, io.execute_eop
    )
  }

  val divUuidOut  = divTagReg(D_UUID  + UUID_WIDTH    - 1, D_UUID)
  val divWidOut   = divTagReg(D_WID   + NW_WIDTH      - 1, D_WID)
  val divTmaskOut = divTagReg(D_TMASK + numLanes      - 1, D_TMASK)
  val divPCOut    = divTagReg(D_PC    + PC_BITS       - 1, D_PC)
  val divRdOut    = divTagReg(D_RD    + NUM_REGS_BITS - 1, D_RD)
  val divWbOut    = divTagReg(D_WB)
  val isRemOut    = divTagReg(D_ISREM)
  val divPidOut   = divTagReg(D_PID   + pidWidth      - 1, D_PID)
  val divSopOut   = divTagReg(D_SOP)
  val divEopOut   = divTagReg(D_EOP)

  // SV: is_rem_op_out ? remainder[i] : quotient[i]
  val divResultOut = VecInit(Seq.tabulate(numLanes) { i =>
    Mux(isRemOut, divSerial.io.remainder(i), divSerial.io.quotient(i))
  })

  // =========================================================================
  // Output arbiter: 2-input priority, outBuf=2
  // Mirrors: VX_stream_arb #(.NUM_INPUTS(2), .ARBITER("P"), .OUT_BUF(2))
  //
  // Data packing per input (MSB→LSB): uuid|wid|tmask|PC|rd|wb|pid|sop|eop|data
  // where data = Cat(result[numLanes-1..0]) (lane 0 at LSB)
  // =========================================================================
  private val arbDataW = tagW + numLanes * XLEN

  val rspArb = Module(new VxStreamArb(
    numInputs  = 2,
    numOutputs = 1,
    dataw      = arbDataW,
    arbiter    = "P",
    outBuf     = 2
  ))

  // Input 0 = mul, input 1 = div  (matches SV {div_valid, mul_valid} bit ordering)
  rspArb.io.validIn(0) := mulValidOut
  rspArb.io.validIn(1) := divValidOut
  mulReadyOut          := rspArb.io.readyIn(0)
  divReadyOut          := rspArb.io.readyIn(1)

  rspArb.io.dataIn(0) := Cat(
    mulUuidOut, mulWidOut, mulTmaskOut, mulPCOut, mulRdOut, mulWbOut,
    mulPidOut, mulSopOut, mulEopOut,
    mulResultOut.asUInt
  )
  rspArb.io.dataIn(1) := Cat(
    divUuidOut, divWidOut, divTmaskOut, divPCOut, divRdOut, divWbOut,
    divPidOut, divSopOut, divEopOut,
    divResultOut.asUInt
  )

  rspArb.io.readyOut(0) := io.result_ready
  io.result_valid       := rspArb.io.validOut(0)

  // Unpack result output
  // Layout (MSB→LSB): uuid|wid|tmask|PC|rd|wb|pid|sop|eop|data[numLanes*XLEN-1:0]
  private val R_DATA  = 0
  private val R_EOP   = R_DATA  + numLanes * XLEN
  private val R_SOP   = R_EOP   + 1
  private val R_PID   = R_SOP   + 1
  private val R_WB    = R_PID   + pidWidth
  private val R_RD    = R_WB    + 1
  private val R_PC    = R_RD    + NUM_REGS_BITS
  private val R_TMASK = R_PC    + PC_BITS
  private val R_WID   = R_TMASK + numLanes
  private val R_UUID  = R_WID   + NW_WIDTH

  val arbOut = rspArb.io.dataOut(0)
  io.result_uuid  := arbOut(R_UUID  + UUID_WIDTH    - 1, R_UUID)
  io.result_wid   := arbOut(R_WID   + NW_WIDTH      - 1, R_WID)
  io.result_tmask := arbOut(R_TMASK + numLanes      - 1, R_TMASK)
  io.result_PC    := arbOut(R_PC    + PC_BITS       - 1, R_PC)
  io.result_rd    := arbOut(R_RD    + NUM_REGS_BITS - 1, R_RD)
  io.result_wb    := arbOut(R_WB)
  io.result_pid   := arbOut(R_PID   + pidWidth      - 1, R_PID)
  io.result_sop   := arbOut(R_SOP)
  io.result_eop   := arbOut(R_EOP)
  for (i <- 0 until numLanes) {
    io.result_data(i) := arbOut(R_DATA + (i + 1) * XLEN - 1, R_DATA + i * XLEN)
  }

  // =========================================================================
  // Input ready
  // =========================================================================
  io.execute_ready := Mux(isMulxOp, mulReadyIn, divReadyIn)
}
