// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_uuid_gen.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Generates a unique instruction ID (UUID) for each issued instruction.
 *  Mirrors VX_uuid_gen.sv.
 *
 *  @param coreId CORE_ID parameter
 */
class VxUuidGen(val coreId: Int = 0) extends Module {
  val io = IO(new Bundle {
    val incr = Input(Bool())
    val wid  = Input(UInt(NW_WIDTH.W))
    val uuid = Output(UInt(UUID_WIDTH.W))
  })

  val gnwWidth = UUID_WIDTH - 32

  // Per-warp 32-bit counters and initialisation flags
  val uuidCntrs   = RegInit(VecInit(Seq.fill(NUM_WARPS)(0.U(32.W))))
  val hasUuidCntrs = RegInit(0.U(NUM_WARPS.W))

  when (io.incr) {
    hasUuidCntrs := hasUuidCntrs | (1.U << io.wid)
    val prev = uuidCntrs(io.wid)
    uuidCntrs(io.wid) := Mux(hasUuidCntrs(io.wid), prev + 1.U, 1.U)
  }

  // g_wid = (CORE_ID << NW_BITS) + wid  (combinational)
  val gWid = (coreId.U(gnwWidth.W) << NW_BITS.U).asUInt //+ io.wid(gnwWidth - 1, 0)

  val counterVal = Mux(hasUuidCntrs(io.wid), uuidCntrs(io.wid), 0.U(32.W))
  io.uuid := Cat(gWid, counterVal)
}
