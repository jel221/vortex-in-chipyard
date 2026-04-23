// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_fetch.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Instruction Fetch unit.
 *  Mirrors VX_fetch.sv.
 *  Converts scheduled warp PC into an I-cache request and reassembles the
 *  response into a FetchDataBundle for the decode stage.
 */
class VxFetch extends Module {
  // Derived widths (matching VX_define / VX_gpu_pkg defaults)
  private val icacheWordSize  = 4   // bytes
  private val icacheAddrW     = MEM_ADDR_WIDTH - log2Ceil(icacheWordSize)
  private val icacheTagW      = UUID_WIDTH + NW_WIDTH

  val io = IO(new Bundle {
    // I-cache bus
    val icache_req_valid  = Output(Bool())
    val icache_req_ready  = Input(Bool())
    val icache_req_addr   = Output(UInt(icacheAddrW.W))
    val icache_req_tag    = Output(UInt(icacheTagW.W))
    val icache_req_rw     = Output(Bool())
    val icache_req_byteen = Output(UInt(icacheWordSize.W))
    val icache_req_data   = Output(UInt((icacheWordSize * 8).W))
    val icache_req_flags  = Output(UInt(MEM_FLAGS_WIDTH.W))
    val icache_rsp_valid  = Input(Bool())
    val icache_rsp_ready  = Output(Bool())
    val icache_rsp_tag    = Input(UInt(icacheTagW.W))
    val icache_rsp_data   = Input(UInt((icacheWordSize * 8).W))

    // Schedule interface (slave)
    val schedule_valid = Input(Bool())
    val schedule_ready = Output(Bool())
    val schedule_wid   = Input(UInt(NW_WIDTH.W))
    val schedule_tmask = Input(UInt(NUM_THREADS.W))
    val schedule_PC    = Input(UInt(PC_BITS.W))
    val schedule_uuid  = Input(UInt(UUID_WIDTH.W))

    // Fetch output (master)
    val fetch_valid = Output(Bool())
    val fetch_ready = Input(Bool())
    val fetch_wid   = Output(UInt(NW_WIDTH.W))
    val fetch_tmask = Output(UInt(NUM_THREADS.W))
    val fetch_PC    = Output(UInt(PC_BITS.W))
    val fetch_instr = Output(UInt(32.W))
    val fetch_uuid  = Output(UInt(UUID_WIDTH.W))
  })

  // -------------------------------------------------------------------------
  // Tag store: maps NW_WIDTH warp-ID → {PC, tmask} for in-flight requests
  // -------------------------------------------------------------------------
  // VX_dp_ram with LUTRAM=1 uses asynchronous (combinational) reads — use Mem, not SyncReadMem.
  val tagMem = Mem(NUM_WARPS, UInt((PC_BITS + NUM_THREADS).W))

  val reqTag  = io.schedule_wid
  val icacheReqFire = Wire(Bool())

  // Write on every I-cache request fire
  when (icacheReqFire) {
    tagMem.write(reqTag, Cat(io.schedule_PC, io.schedule_tmask))
  }

  // Split incoming response tag into UUID and warp-ID
  val rspUuid = io.icache_rsp_tag(icacheTagW - 1, NW_WIDTH)
  val rspTag  = io.icache_rsp_tag(NW_WIDTH - 1, 0)

  // Combinational read — valid in the same cycle as icache_rsp_valid
  val rspData  = tagMem.read(rspTag)
  val rspPC    = rspData(PC_BITS + NUM_THREADS - 1, NUM_THREADS)
  val rspTmask = rspData(NUM_THREADS - 1, 0)

  // -------------------------------------------------------------------------
  // I-cache request — Queue(2, pipe=true) matches VX_elastic_buffer(SIZE=2, OUT_REG=1)
  // -------------------------------------------------------------------------
  val icacheReqAddr = io.schedule_PC(PC_BITS - 1, log2Ceil(icacheWordSize))
  val icacheReqTag  = Cat(io.schedule_uuid, reqTag)

  class IcacheReqBundle extends Bundle {
    val addr = UInt(icacheAddrW.W)
    val tag  = UInt(icacheTagW.W)
  }

  val enq = Wire(Decoupled(new IcacheReqBundle))
  enq.valid      := io.schedule_valid
  enq.bits.addr  := icacheReqAddr
  enq.bits.tag   := icacheReqTag
  io.schedule_ready := enq.ready
  icacheReqFire     := enq.fire

  val deq = Queue(enq, entries = 2, pipe = true)

  io.icache_req_valid  := deq.valid
  io.icache_req_addr   := deq.bits.addr
  io.icache_req_tag    := deq.bits.tag
  deq.ready            := io.icache_req_ready
  io.icache_req_rw     := false.B
  io.icache_req_byteen := Fill(icacheWordSize, 1.U(1.W))
  io.icache_req_data   := 0.U
  io.icache_req_flags  := 0.U

  // -------------------------------------------------------------------------
  // I-cache response → fetch output
  // -------------------------------------------------------------------------
  io.fetch_valid := io.icache_rsp_valid
  io.fetch_wid   := rspTag
  io.fetch_tmask := rspTmask
  io.fetch_PC    := rspPC
  io.fetch_instr := io.icache_rsp_data(31, 0)
  io.fetch_uuid  := rspUuid
  io.icache_rsp_ready := io.fetch_ready
}
