// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_pe_switch.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_pe_switch: 1→N request switch + N→1 response arbiter for PE arrays.
 *
 *  Translates a single execute_in_if to one of PE_COUNT execute_out_if[i]
 *  based on the pe_sel signal.  Responses from any of the PE_COUNT
 *  result_in_if[i] are arbitrated back to a single result_out_if.
 *
 *  The SV instantiates VX_stream_switch (1→N) and VX_stream_arb (N→1).
 *  Those are library modules from the Vortex repo; here we use BlackBox
 *  stubs that capture the interface accurately.
 *
 *  @param peCount    PE_COUNT
 *  @param numLanes   NUM_LANES per PE
 *  @param reqOutBuf  REQ_OUT_BUF  (buffering on request side)
 *  @param rspOutBuf  RSP_OUT_BUF  (buffering on response side)
 *  @param arbiter    ARBITER string ("R" = round-robin, "P" = priority)
 */
class VxPeSwitch(
    val peCount:   Int    = 1,
    val numLanes:  Int    = 1,
    val reqOutBuf: Int    = 0,
    val rspOutBuf: Int    = 0,
    val arbiter:   String = "R"
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val peSelBits  = log2Ceil(math.max(1, peCount))
  private val peSelWidth = math.max(1, peSelBits)

  // Execute-data width (matches REQ_DATAW in VX_pe_switch.sv)
  private val pidBits  = log2Ceil(math.max(1, NUM_THREADS / numLanes))
  private val pidWidth = math.max(1, pidBits)
  private val reqDataW = UUID_WIDTH + NW_WIDTH + numLanes + PC_BITS + INST_ALU_BITS +
                         INST_ARGS_BITS + 1 + NUM_REGS_BITS + 3 * numLanes * XLEN +
                         pidWidth + 1 + 1

  // Result-data width (matches RSP_DATAW in VX_pe_switch.sv)
  private val rspDataW = UUID_WIDTH + NW_WIDTH + numLanes + PC_BITS + NUM_REGS_BITS +
                         1 + numLanes * XLEN + pidWidth + 1 + 1

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    val pe_sel = Input(UInt(peSelWidth.W))

    // Execute input (from dispatch, slave)
    val execute_in_valid  = Input(Bool())
    val execute_in_ready  = Output(Bool())
    val execute_in_data   = Input(UInt(reqDataW.W))

    // Result output (to gather/commit, master)
    val result_out_valid  = Output(Bool())
    val result_out_ready  = Input(Bool())
    val result_out_data   = Output(UInt(rspDataW.W))

    // Execute outputs (to each PE, master)
    val execute_out_valid = Output(Vec(peCount, Bool()))
    val execute_out_ready = Input(Vec(peCount, Bool()))
    val execute_out_data  = Output(Vec(peCount, UInt(reqDataW.W)))

    // Result inputs (from each PE, slave)
    val result_in_valid   = Input(Vec(peCount, Bool()))
    val result_in_ready   = Output(Vec(peCount, Bool()))
    val result_in_data    = Input(Vec(peCount, UInt(rspDataW.W)))
  })

  // -------------------------------------------------------------------------
  // Request switch (1→PE_COUNT): VX_stream_switch
  // -------------------------------------------------------------------------
  val reqSwitch = Module(new VxStreamSwitch(
    dataw      = reqDataW,
    numInputs  = 1,
    numOutputs = peCount,
    outBuf     = reqOutBuf
  ))
  reqSwitch.io.selIn    := io.pe_sel
  reqSwitch.io.validIn  := VecInit(Seq(io.execute_in_valid))
  io.execute_in_ready    := reqSwitch.io.readyIn(0)
  reqSwitch.io.dataIn   := VecInit(Seq(io.execute_in_data))
  for (i <- 0 until peCount) {
    io.execute_out_valid(i)  := reqSwitch.io.validOut(i)
    io.execute_out_data(i)   := reqSwitch.io.dataOut(i)
    reqSwitch.io.readyOut(i) := io.execute_out_ready(i)
  }

  // -------------------------------------------------------------------------
  // Response arbiter (PE_COUNT→1): VX_stream_arb
  // -------------------------------------------------------------------------
  val rspArb = Module(new VxStreamArb(
    numInputs = peCount,
    dataw     = rspDataW,
    arbiter   = arbiter,
    outBuf    = rspOutBuf
  ))
  for (i <- 0 until peCount) {
    rspArb.io.validIn(i)  := io.result_in_valid(i)
    io.result_in_ready(i)  := rspArb.io.readyIn(i)
    rspArb.io.dataIn(i)   := io.result_in_data(i)
  }
  io.result_out_valid      := rspArb.io.validOut
  rspArb.io.readyOut      := io.result_out_ready
  io.result_out_data       := rspArb.io.dataOut
}
