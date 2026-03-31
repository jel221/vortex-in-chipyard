// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_mem_unit_top.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_mem_unit_top: flat-port wrapper around VX_mem_unit.
 *
 *  Converts the interface-style connections of VX_mem_unit into individual
 *  scalar/vector ports, making the module suitable as a top-level boundary
 *  when flat ports are required (e.g. for simulation drivers or the Verilog
 *  shim in VortexAXIWrapper).
 *
 *  The module instantiates VxMemUnit and wires every field individually,
 *  mirroring the explicit assign statements in VX_mem_unit_top.sv.
 */
class VxMemUnitTop(val instanceId: String = "") extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val lsuWordWidth = LSU_WORD_SIZE * 8

  val io = IO(new Bundle {
    // LSU memory request (flat, NUM_LSU_BLOCKS)
    val lsu_req_valid  = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_rw     = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_mask   = Input(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
    val lsu_req_byteen = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_WORD_SIZE.W))))
    val lsu_req_addr   = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_ADDR_WIDTH.W))))
    val lsu_req_flags  = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(MEM_FLAGS_WIDTH.W))))
    val lsu_req_data   = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(lsuWordWidth.W))))
    val lsu_req_tag    = Input(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))
    val lsu_req_ready  = Output(Vec(NUM_LSU_BLOCKS, Bool()))

    // LSU memory response (flat, NUM_LSU_BLOCKS)
    val lsu_rsp_valid  = Output(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_rsp_mask   = Output(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
    val lsu_rsp_data   = Output(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(lsuWordWidth.W))))
    val lsu_rsp_tag    = Output(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))
    val lsu_rsp_ready  = Input(Vec(NUM_LSU_BLOCKS, Bool()))

    // Memory bus request (flat, DCACHE_NUM_REQS)
    val mem_req_valid  = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val mem_req_rw     = Output(Vec(DCACHE_NUM_REQS, Bool()))
    val mem_req_byteen = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_WORD_SIZE.W)))
    val mem_req_addr   = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_ADDR_WIDTH.W)))
    val mem_req_flags  = Output(Vec(DCACHE_NUM_REQS, UInt(MEM_FLAGS_WIDTH.W)))
    val mem_req_data   = Output(Vec(DCACHE_NUM_REQS, UInt((DCACHE_WORD_SIZE * 8).W)))
    val mem_req_tag    = Output(Vec(DCACHE_NUM_REQS, UInt(DCACHE_TAG_WIDTH.W)))
    val mem_req_ready  = Input(Vec(DCACHE_NUM_REQS, Bool()))

    // Memory bus response (flat, DCACHE_NUM_REQS)
    val mem_rsp_valid  = Input(Vec(DCACHE_NUM_REQS, Bool()))
    val mem_rsp_data   = Input(Vec(DCACHE_NUM_REQS, UInt((DCACHE_WORD_SIZE * 8).W)))
    val mem_rsp_tag    = Input(Vec(DCACHE_NUM_REQS, UInt(DCACHE_TAG_WIDTH.W)))
    val mem_rsp_ready  = Output(Vec(DCACHE_NUM_REQS, Bool()))
  })

  // -------------------------------------------------------------------------
  // Instantiate VxMemUnit
  // -------------------------------------------------------------------------
  val memUnit = Module(new VxMemUnit(instanceId))

  // LSU request
  for (i <- 0 until NUM_LSU_BLOCKS) {
    memUnit.io.lsu_req_valid(i)  := io.lsu_req_valid(i)
    memUnit.io.lsu_req_rw(i)     := io.lsu_req_rw(i)
    memUnit.io.lsu_req_mask(i)   := io.lsu_req_mask(i)
    memUnit.io.lsu_req_byteen(i) := io.lsu_req_byteen(i)
    memUnit.io.lsu_req_addr(i)   := io.lsu_req_addr(i)
    memUnit.io.lsu_req_flags(i)  := io.lsu_req_flags(i)
    memUnit.io.lsu_req_data(i)   := io.lsu_req_data(i)
    memUnit.io.lsu_req_tag(i)    := io.lsu_req_tag(i)
    io.lsu_req_ready(i)          := memUnit.io.lsu_req_ready(i)
  }

  // LSU response
  for (i <- 0 until NUM_LSU_BLOCKS) {
    io.lsu_rsp_valid(i)          := memUnit.io.lsu_rsp_valid(i)
    io.lsu_rsp_mask(i)           := memUnit.io.lsu_rsp_mask(i)
    io.lsu_rsp_data(i)           := memUnit.io.lsu_rsp_data(i)
    io.lsu_rsp_tag(i)            := memUnit.io.lsu_rsp_tag(i)
    memUnit.io.lsu_rsp_ready(i)  := io.lsu_rsp_ready(i)
  }

  // Memory bus request
  for (i <- 0 until DCACHE_NUM_REQS) {
    io.mem_req_valid(i)          := memUnit.io.dcache_req_valid(i)
    io.mem_req_rw(i)             := memUnit.io.dcache_req_rw(i)
    io.mem_req_byteen(i)         := memUnit.io.dcache_req_byteen(i)
    io.mem_req_addr(i)           := memUnit.io.dcache_req_addr(i)
    io.mem_req_flags(i)          := memUnit.io.dcache_req_flags(i)
    io.mem_req_data(i)           := memUnit.io.dcache_req_data(i)
    io.mem_req_tag(i)            := memUnit.io.dcache_req_tag(i)
    memUnit.io.dcache_req_ready(i) := io.mem_req_ready(i)
  }

  // Memory bus response
  for (i <- 0 until DCACHE_NUM_REQS) {
    memUnit.io.dcache_rsp_valid(i) := io.mem_rsp_valid(i)
    memUnit.io.dcache_rsp_data(i)  := io.mem_rsp_data(i)
    memUnit.io.dcache_rsp_tag(i)   := io.mem_rsp_tag(i)
    io.mem_rsp_ready(i)            := memUnit.io.dcache_rsp_ready(i)
  }
}
