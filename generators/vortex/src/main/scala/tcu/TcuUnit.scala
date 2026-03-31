// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_tcu_unit.sv and VX_tcu_top.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._
import TcuPkg._

/** TCU (Tensor Core Unit) execution unit.
 *  Mirrors VX_tcu_unit.sv.
 *
 *  With EXT_TCU_ENABLE=0 (default), this module is not instantiated.
 *  Provided as a structural stub for completeness.
 */
class VxTcuUnit(val issueWidth: Int = ISSUE_WIDTH) extends Module {
  private val numLanes  = NUM_THREADS
  private val blockSize = issueWidth

  // TCU execute/result data widths (from tcu_exe_t / tcu_res_t)
  private val tcuExeBits = TCU_EXE_BITS
  private val tcuResBits = TCU_RES_BITS

  val io = IO(new Bundle {
    // Dispatch inputs (one per issue slot)
    val dispatch_valid = Input(Vec(issueWidth, Bool()))
    val dispatch_ready = Output(Vec(issueWidth, Bool()))
    val dispatch_data  = Input(Vec(issueWidth, UInt(DISPATCH_DATA_BITS.W)))

    // Commit outputs (one per issue slot)
    val commit_valid = Output(Vec(issueWidth, Bool()))
    val commit_ready = Input(Vec(issueWidth, Bool()))
    val commit_data  = Output(Vec(issueWidth, UInt(COMMIT_DATA_BITS.W)))
  })

  // Stub: tie off outputs
  for (i <- 0 until issueWidth) {
    io.dispatch_ready(i) := true.B
    io.commit_valid(i)   := false.B
    io.commit_data(i)    := 0.U
  }
}

/** Flat-port TCU top wrapper.
 *  Mirrors VX_tcu_top.sv.
 */
class VxTcuTop extends Module {
  val io = IO(new Bundle {
    val execute_valid = Input(Bool())
    val execute_data  = Input(UInt(TCU_EXE_BITS.W))
    val execute_ready = Output(Bool())

    val result_valid  = Output(Bool())
    val result_data   = Output(UInt(TCU_RES_BITS.W))
    val result_ready  = Input(Bool())
  })

  // Stub: pass-through
  io.execute_ready := true.B
  io.result_valid  := false.B
  io.result_data   := 0.U
}
