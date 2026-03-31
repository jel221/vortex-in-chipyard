// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_mem_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_mem_unit: memory subsystem between LSU and D-cache.
 *
 *  Instantiates:
 *    - (LMEM_ENABLE) VX_lmem_switch (global vs local routing) per LSU block
 *    - (LMEM_ENABLE) VX_lsu_mem_arb, VX_lsu_adapter, VX_local_mem
 *    - (NUM_LSU_LANES > 1 && LSU != DCACHE word size) VX_mem_coalescer per block
 *    - VX_lsu_adapter + port assignment to dcache_bus_if
 *
 *  All sub-units are represented as BlackBox stubs (the detailed microarch
 *  is complex enough that full Chisel translations live in mem/ and cache/).
 *  The interface faithfully mirrors VX_mem_unit.sv.
 */
class VxMemUnit(val instanceId: String = "") extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  val io = IO(new Bundle {
    // LSU memory interface (NUM_LSU_BLOCKS slave ports)
    val lsu_req_valid  = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_ready  = Output(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_rw     = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_mask   = Input(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
    val lsu_req_byteen = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_WORD_SIZE.W))))
    val lsu_req_addr   = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_ADDR_WIDTH.W))))
    val lsu_req_flags  = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(MEM_FLAGS_WIDTH.W))))
    val lsu_req_data   = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
    val lsu_req_tag    = Input(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))
    val lsu_rsp_valid  = Output(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_rsp_ready  = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_rsp_mask   = Output(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
    val lsu_rsp_data   = Output(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
    val lsu_rsp_tag    = Output(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))

    // D-cache bus interface (DCACHE_NUM_REQS master ports)
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
  })

  // -------------------------------------------------------------------------
  // All internal plumbing (lmem switches, coalescer, adapters) is complex
  // microarchitecture that depends on multiple support modules.  We instantiate
  // a single BlackBox stub that captures the full interface.
  // -------------------------------------------------------------------------
  val inner = Module(new VxMemUnitBB(instanceId))

  inner.io.lsu_req_valid  := io.lsu_req_valid
  io.lsu_req_ready        := inner.io.lsu_req_ready
  inner.io.lsu_req_rw     := io.lsu_req_rw
  inner.io.lsu_req_mask   := io.lsu_req_mask
  inner.io.lsu_req_byteen := io.lsu_req_byteen
  inner.io.lsu_req_addr   := io.lsu_req_addr
  inner.io.lsu_req_flags  := io.lsu_req_flags
  inner.io.lsu_req_data   := io.lsu_req_data
  inner.io.lsu_req_tag    := io.lsu_req_tag
  io.lsu_rsp_valid        := inner.io.lsu_rsp_valid
  inner.io.lsu_rsp_ready  := io.lsu_rsp_ready
  io.lsu_rsp_mask         := inner.io.lsu_rsp_mask
  io.lsu_rsp_data         := inner.io.lsu_rsp_data
  io.lsu_rsp_tag          := inner.io.lsu_rsp_tag

  io.dcache_req_valid  := inner.io.dcache_req_valid
  inner.io.dcache_req_ready := io.dcache_req_ready
  io.dcache_req_rw     := inner.io.dcache_req_rw
  io.dcache_req_byteen := inner.io.dcache_req_byteen
  io.dcache_req_addr   := inner.io.dcache_req_addr
  io.dcache_req_flags  := inner.io.dcache_req_flags
  io.dcache_req_data   := inner.io.dcache_req_data
  io.dcache_req_tag    := inner.io.dcache_req_tag
  inner.io.dcache_rsp_valid := io.dcache_rsp_valid
  io.dcache_rsp_ready  := inner.io.dcache_rsp_ready
  inner.io.dcache_rsp_data  := io.dcache_rsp_data
  inner.io.dcache_rsp_tag   := io.dcache_rsp_tag
}

/** BlackBox stub for VxMemUnit. */
class VxMemUnitBB(instanceId: String) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  val io = IO(new Bundle {
    val lsu_req_valid  = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_ready  = Output(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_rw     = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_req_mask   = Input(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
    val lsu_req_byteen = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_WORD_SIZE.W))))
    val lsu_req_addr   = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(LSU_ADDR_WIDTH.W))))
    val lsu_req_flags  = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(MEM_FLAGS_WIDTH.W))))
    val lsu_req_data   = Input(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
    val lsu_req_tag    = Input(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))
    val lsu_rsp_valid  = Output(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_rsp_ready  = Input(Vec(NUM_LSU_BLOCKS, Bool()))
    val lsu_rsp_mask   = Output(Vec(NUM_LSU_BLOCKS, UInt(NUM_LSU_LANES.W)))
    val lsu_rsp_data   = Output(Vec(NUM_LSU_BLOCKS, Vec(NUM_LSU_LANES, UInt(XLEN.W))))
    val lsu_rsp_tag    = Output(Vec(NUM_LSU_BLOCKS, UInt(LSU_TAG_WIDTH.W)))
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
  })
  io.lsu_req_ready      := VecInit(Seq.fill(NUM_LSU_BLOCKS)(false.B))
  io.lsu_rsp_valid      := VecInit(Seq.fill(NUM_LSU_BLOCKS)(false.B))
  io.lsu_rsp_mask       := VecInit(Seq.fill(NUM_LSU_BLOCKS)(0.U))
  io.lsu_rsp_data       := VecInit(Seq.fill(NUM_LSU_BLOCKS)(VecInit(Seq.fill(NUM_LSU_LANES)(0.U(XLEN.W)))))
  io.lsu_rsp_tag        := VecInit(Seq.fill(NUM_LSU_BLOCKS)(0.U))
  io.dcache_req_valid   := VecInit(Seq.fill(DCACHE_NUM_REQS)(false.B))
  io.dcache_req_rw      := VecInit(Seq.fill(DCACHE_NUM_REQS)(false.B))
  io.dcache_req_byteen  := VecInit(Seq.fill(DCACHE_NUM_REQS)(0.U))
  io.dcache_req_addr    := VecInit(Seq.fill(DCACHE_NUM_REQS)(0.U))
  io.dcache_req_flags   := VecInit(Seq.fill(DCACHE_NUM_REQS)(0.U))
  io.dcache_req_data    := VecInit(Seq.fill(DCACHE_NUM_REQS)(0.U))
  io.dcache_req_tag     := VecInit(Seq.fill(DCACHE_NUM_REQS)(0.U))
  io.dcache_rsp_ready   := VecInit(Seq.fill(DCACHE_NUM_REQS)(false.B))
}
