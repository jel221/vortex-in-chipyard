// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_core_top.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_core_top: flat-port wrapper around VX_core.
 *
 *  Converts all interface-style connections of VX_core into individual
 *  scalar/vector ports, mirroring the explicit assign statements in
 *  VX_core_top.sv.  This is the boundary used when the core needs to be
 *  instantiated from an AXI shim or simulation driver.
 *
 *  @param coreId   CORE_ID parameter (default 0)
 */
class VxCoreTop(val coreId: Int = 0) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  val io = IO(new Bundle {
    // Direct from RoCC (same pattern as startup_addr)
    val startup_addr = Input(UInt(XLEN.W))
    val mpm_class    = Input(UInt(8.W))

    // D-cache bus (DCACHE_NUM_REQS master ports)
    val dcache_req_valid  = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_req_rw     = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_req_byteen = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_WORD_SIZE.W)))
    val dcache_req_addr   = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_ADDR_WIDTH.W)))
    val dcache_req_flags  = Output(Vec(DCACHE_NUM_REQS, UInt(MEM_FLAGS_WIDTH.W)))
    val dcache_req_data   = Output(Vec(DCACHE_NUM_REQS, UInt((DCACHE_WORD_SIZE * 8).W)))
    val dcache_req_tag    = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_TAG_WIDTH.W)))
    val dcache_req_ready  = Input(Vec(DCACHE_NUM_REQS, Bool()))

    val dcache_rsp_valid  = Input(Vec(DCACHE_NUM_REQS, Bool()))
    val dcache_rsp_data   = Input(Vec(DCACHE_NUM_REQS, UInt((DCACHE_WORD_SIZE * 8).W)))
    val dcache_rsp_tag    = Input(Vec(DCACHE_NUM_REQS, UInt(DCACHE_TAG_WIDTH.W)))
    val dcache_rsp_ready  = Output(Vec(DCACHE_NUM_REQS, Bool()))

    // I-cache bus (single master port)
    val icache_req_valid  = Output(Bool())
    val icache_req_rw     = Output(Bool())
    val icache_req_byteen = Output(UInt(ICACHE_WORD_SIZE.W))
    val icache_req_addr   = Output(UInt(ICACHE_ADDR_WIDTH.W))
    val icache_req_data   = Output(UInt((ICACHE_WORD_SIZE * 8).W))
    val icache_req_tag    = Output(UInt(ICACHE_TAG_WIDTH.W))
    val icache_req_ready  = Input(Bool())

    val icache_rsp_valid  = Input(Bool())
    val icache_rsp_data   = Input(UInt((ICACHE_WORD_SIZE * 8).W))
    val icache_rsp_tag    = Input(UInt(ICACHE_TAG_WIDTH.W))
    val icache_rsp_ready  = Output(Bool())

    // Busy status
    val busy = Output(Bool())
  })

  // -------------------------------------------------------------------------
  // Instantiate VxCore
  // -------------------------------------------------------------------------
  val core = Module(new VxCore(coreId))

  core.io.startup_addr := io.startup_addr
  core.io.mpm_class    := io.mpm_class

  // D-cache request / response
  for (i <- 0 until DCACHE_NUM_REQS) {
    io.dcache_req_valid(i)  := core.io.dcache_req_valid(i)
    io.dcache_req_rw(i)     := core.io.dcache_req_rw(i)
    io.dcache_req_byteen(i) := core.io.dcache_req_byteen(i)
    io.dcache_req_addr(i)   := core.io.dcache_req_addr(i)
    io.dcache_req_flags(i)  := core.io.dcache_req_flags(i)
    io.dcache_req_data(i)   := core.io.dcache_req_data(i)
    io.dcache_req_tag(i)    := core.io.dcache_req_tag(i)
    core.io.dcache_req_ready(i) := io.dcache_req_ready(i)

    core.io.dcache_rsp_valid(i) := io.dcache_rsp_valid(i)
    core.io.dcache_rsp_data(i)  := io.dcache_rsp_data(i)
    core.io.dcache_rsp_tag(i)   := io.dcache_rsp_tag(i)
    io.dcache_rsp_ready(i)      := core.io.dcache_rsp_ready(i)
  }

  // I-cache request / response
  io.icache_req_valid         := core.io.icache_req_valid
  io.icache_req_rw            := core.io.icache_req_rw
  io.icache_req_byteen        := core.io.icache_req_byteen
  io.icache_req_addr          := core.io.icache_req_addr
  io.icache_req_data          := core.io.icache_req_data
  io.icache_req_tag           := core.io.icache_req_tag
  core.io.icache_req_ready    := io.icache_req_ready

  core.io.icache_rsp_valid    := io.icache_rsp_valid
  core.io.icache_rsp_data     := io.icache_rsp_data
  core.io.icache_rsp_tag      := io.icache_rsp_tag
  io.icache_rsp_ready         := core.io.icache_rsp_ready

  // Busy
  io.busy := core.io.busy
}
