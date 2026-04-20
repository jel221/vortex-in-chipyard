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
 *  Instantiates (LMEM_ENABLE assumed true):
 *    - LmemSwitch per LSU block
 *    - LsuMemArb (NUM_LSU_BLOCKS → 1) + LsuAdapter + LocalMem for the local memory path
 *    - VXMemCoalescer per block (NUM_LSU_LANES > 1 && LSU_WORD_SIZE != DCACHE_WORD_SIZE)
 *    - LsuAdapter per block → dcache_bus ports
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

  private val LMEM_ADDR_WIDTH = LMEM_LOG_SIZE - log2Ceil(LSU_WORD_SIZE)

  // ---- LmemSwitches: one per LSU block, splits global vs local requests -----
  val lmemSwitches = Seq.tabulate(NUM_LSU_BLOCKS) { _ =>
    Module(new LmemSwitch(
      numLanes   = NUM_LSU_LANES,
      dataSize   = LSU_WORD_SIZE,
      addrWidth  = LSU_ADDR_WIDTH,
      flagsWidth = MEM_FLAGS_WIDTH,
      tagWidth   = LSU_TAG_WIDTH,
      uuidWidth  = UUID_WIDTH
    ))
  }

  for (i <- 0 until NUM_LSU_BLOCKS) {
    val sw = lmemSwitches(i)
    sw.io.lsuIn.req.valid       := io.lsu_req_valid(i)
    sw.io.lsuIn.req.bits.mask   := io.lsu_req_mask(i)
    sw.io.lsuIn.req.bits.rw     := io.lsu_req_rw(i)
    sw.io.lsuIn.req.bits.byteen := io.lsu_req_byteen(i)
    sw.io.lsuIn.req.bits.addr   := io.lsu_req_addr(i)
    sw.io.lsuIn.req.bits.flags  := io.lsu_req_flags(i)
    sw.io.lsuIn.req.bits.data   := io.lsu_req_data(i)
    sw.io.lsuIn.req.bits.tag    := io.lsu_req_tag(i).asTypeOf(new MemBusTagBundle(LSU_TAG_WIDTH, UUID_WIDTH))
    io.lsu_req_ready(i)          := sw.io.lsuIn.req.ready

    io.lsu_rsp_valid(i)          := sw.io.lsuIn.rsp.valid
    io.lsu_rsp_mask(i)           := sw.io.lsuIn.rsp.bits.mask
    io.lsu_rsp_data(i)           := sw.io.lsuIn.rsp.bits.data
    io.lsu_rsp_tag(i)            := sw.io.lsuIn.rsp.bits.tag.asUInt
    sw.io.lsuIn.rsp.ready        := io.lsu_rsp_ready(i)
  }

  // ---- LMEM path: LsuMemArb → LsuAdapter → LocalMem -----------------------
  val lmemArb = Module(new LsuMemArb(
    numInputs  = NUM_LSU_BLOCKS,
    numOutputs = 1,
    numLanes   = NUM_LSU_LANES,
    dataSize   = LSU_WORD_SIZE,
    addrWidth  = LSU_ADDR_WIDTH,
    flagsWidth = MEM_FLAGS_WIDTH,
    tagWidth   = LSU_TAG_WIDTH,
    tagSelIdx  = 0,
    uuidWidth  = UUID_WIDTH
  ))

  for (i <- 0 until NUM_LSU_BLOCKS) {
    val sw = lmemSwitches(i)
    lmemArb.io.busIn(i).req.valid       := sw.io.localOut.req.valid
    lmemArb.io.busIn(i).req.bits        := sw.io.localOut.req.bits
    sw.io.localOut.req.ready             := lmemArb.io.busIn(i).req.ready
    sw.io.localOut.rsp.valid             := lmemArb.io.busIn(i).rsp.valid
    sw.io.localOut.rsp.bits              := lmemArb.io.busIn(i).rsp.bits
    lmemArb.io.busIn(i).rsp.ready       := sw.io.localOut.rsp.ready
  }

  // LMEM_TAG_WIDTH = LSU_TAG_WIDTH + log2Ceil(NUM_LSU_BLOCKS)
  // When NUM_LSU_BLOCKS=1: log2Ceil(1)=0, so LMEM_TAG_WIDTH = LSU_TAG_WIDTH = lmemArb output tag width
  val lmemAdapter = Module(new LsuAdapter(
    numLanes   = NUM_LSU_LANES,
    dataSize   = LSU_WORD_SIZE,
    addrWidth  = LSU_ADDR_WIDTH,
    flagsWidth = MEM_FLAGS_WIDTH,
    tagWidth   = LMEM_TAG_WIDTH,
    tagSelBits = LMEM_TAG_WIDTH - UUID_WIDTH,
    uuidWidth  = UUID_WIDTH
  ))

  lmemAdapter.io.lsuMem.req.valid       := lmemArb.io.busOut(0).req.valid
  lmemAdapter.io.lsuMem.req.bits        := lmemArb.io.busOut(0).req.bits
  lmemArb.io.busOut(0).req.ready        := lmemAdapter.io.lsuMem.req.ready
  lmemArb.io.busOut(0).rsp.valid        := lmemAdapter.io.lsuMem.rsp.valid
  lmemArb.io.busOut(0).rsp.bits         := lmemAdapter.io.lsuMem.rsp.bits
  lmemAdapter.io.lsuMem.rsp.ready       := lmemArb.io.busOut(0).rsp.ready

  val localMem = Module(new LocalMem(
    size       = 1 << LMEM_LOG_SIZE,
    numReqs    = NUM_LSU_LANES,
    numBanks   = LMEM_NUM_BANKS,
    wordSize   = LSU_WORD_SIZE,
    tagWidth   = LMEM_TAG_WIDTH,
    flagsWidth = MEM_FLAGS_WIDTH,
    uuidWidth  = UUID_WIDTH
  ))

  // LocalMem uses an internally-derived addrWidth (LMEM_ADDR_WIDTH), so the
  // per-lane addresses must be truncated from LSU_ADDR_WIDTH to LMEM_ADDR_WIDTH.
  for (j <- 0 until NUM_LSU_LANES) {
    localMem.io.memBus(j).req.valid       := lmemAdapter.io.memBus(j).req.valid
    localMem.io.memBus(j).req.bits.rw     := lmemAdapter.io.memBus(j).req.bits.rw
    localMem.io.memBus(j).req.bits.addr   := lmemAdapter.io.memBus(j).req.bits.addr(LMEM_ADDR_WIDTH - 1, 0)
    localMem.io.memBus(j).req.bits.data   := lmemAdapter.io.memBus(j).req.bits.data
    localMem.io.memBus(j).req.bits.byteen := lmemAdapter.io.memBus(j).req.bits.byteen
    localMem.io.memBus(j).req.bits.flags  := lmemAdapter.io.memBus(j).req.bits.flags
    localMem.io.memBus(j).req.bits.tag    := lmemAdapter.io.memBus(j).req.bits.tag
    lmemAdapter.io.memBus(j).req.ready    := localMem.io.memBus(j).req.ready
    lmemAdapter.io.memBus(j).rsp.valid    := localMem.io.memBus(j).rsp.valid
    lmemAdapter.io.memBus(j).rsp.bits     := localMem.io.memBus(j).rsp.bits
    localMem.io.memBus(j).rsp.ready       := lmemAdapter.io.memBus(j).rsp.ready
  }

  // ---- DCache path: VXMemCoalescer per block → LsuAdapter → dcache_bus -----
  // Coalescer enabled: NUM_LSU_LANES=4 > 1 && LSU_WORD_SIZE=4 != DCACHE_WORD_SIZE=16
  for (i <- 0 until NUM_LSU_BLOCKS) {
    val sw = lmemSwitches(i)

    val coalescer = Module(new VXMemCoalescer(
      numReqs     = NUM_LSU_LANES,
      addrWidth   = LSU_ADDR_WIDTH,
      flagsWidth  = MEM_FLAGS_WIDTH,
      dataInSize  = LSU_WORD_SIZE,
      dataOutSize = DCACHE_WORD_SIZE,
      tagWidth    = LSU_TAG_WIDTH,
      uuidWidth   = UUID_WIDTH,
      queueSize   = LSUQ_OUT_SIZE
    ))

    // LmemSwitch globalOut → coalescer input
    coalescer.io.in_req_valid  := sw.io.globalOut.req.valid
    coalescer.io.in_req_mask   := sw.io.globalOut.req.bits.mask
    coalescer.io.in_req_rw     := sw.io.globalOut.req.bits.rw
    coalescer.io.in_req_byteen := sw.io.globalOut.req.bits.byteen
    coalescer.io.in_req_addr   := sw.io.globalOut.req.bits.addr
    coalescer.io.in_req_flags  := sw.io.globalOut.req.bits.flags
    coalescer.io.in_req_data   := sw.io.globalOut.req.bits.data
    coalescer.io.in_req_tag    := sw.io.globalOut.req.bits.tag.asUInt
    sw.io.globalOut.req.ready  := coalescer.io.in_req_ready

    sw.io.globalOut.rsp.valid            := coalescer.io.in_rsp_valid
    sw.io.globalOut.rsp.bits.mask        := coalescer.io.in_rsp_mask
    sw.io.globalOut.rsp.bits.data        := coalescer.io.in_rsp_data
    sw.io.globalOut.rsp.bits.tag         := coalescer.io.in_rsp_tag.asTypeOf(new MemBusTagBundle(LSU_TAG_WIDTH, UUID_WIDTH))
    coalescer.io.in_rsp_ready            := sw.io.globalOut.rsp.ready

    // coalescer output → dcache LsuAdapter
    // outReqs = DCACHE_CHANNELS, outAddrWidth = DCACHE_ADDR_WIDTH, outTagWidth = DCACHE_TAG_WIDTH
    val dcacheAdapter = Module(new LsuAdapter(
      numLanes   = DCACHE_CHANNELS,
      dataSize   = DCACHE_WORD_SIZE,
      addrWidth  = DCACHE_ADDR_WIDTH,
      flagsWidth = MEM_FLAGS_WIDTH,
      tagWidth   = DCACHE_TAG_WIDTH,
      tagSelBits = DCACHE_TAG_WIDTH - UUID_WIDTH,
      uuidWidth  = UUID_WIDTH
    ))

    dcacheAdapter.io.lsuMem.req.valid       := coalescer.io.out_req_valid
    dcacheAdapter.io.lsuMem.req.bits.mask   := coalescer.io.out_req_mask
    dcacheAdapter.io.lsuMem.req.bits.rw     := coalescer.io.out_req_rw
    dcacheAdapter.io.lsuMem.req.bits.byteen := coalescer.io.out_req_byteen
    dcacheAdapter.io.lsuMem.req.bits.addr   := coalescer.io.out_req_addr
    dcacheAdapter.io.lsuMem.req.bits.flags  := coalescer.io.out_req_flags
    dcacheAdapter.io.lsuMem.req.bits.data   := coalescer.io.out_req_data
    dcacheAdapter.io.lsuMem.req.bits.tag    := coalescer.io.out_req_tag.asTypeOf(new MemBusTagBundle(DCACHE_TAG_WIDTH, UUID_WIDTH))
    coalescer.io.out_req_ready              := dcacheAdapter.io.lsuMem.req.ready

    coalescer.io.out_rsp_valid              := dcacheAdapter.io.lsuMem.rsp.valid
    coalescer.io.out_rsp_mask               := dcacheAdapter.io.lsuMem.rsp.bits.mask
    coalescer.io.out_rsp_data               := dcacheAdapter.io.lsuMem.rsp.bits.data
    coalescer.io.out_rsp_tag                := dcacheAdapter.io.lsuMem.rsp.bits.tag.asUInt
    dcacheAdapter.io.lsuMem.rsp.ready       := coalescer.io.out_rsp_ready

    // dcacheAdapter memBus → dcache_bus_if[i * DCACHE_CHANNELS + j]
    for (j <- 0 until DCACHE_CHANNELS) {
      val idx = i * DCACHE_CHANNELS + j
      io.dcache_req_valid(idx)             := dcacheAdapter.io.memBus(j).req.valid
      io.dcache_req_rw(idx)               := dcacheAdapter.io.memBus(j).req.bits.rw
      io.dcache_req_byteen(idx)           := dcacheAdapter.io.memBus(j).req.bits.byteen
      io.dcache_req_addr(idx)             := dcacheAdapter.io.memBus(j).req.bits.addr
      io.dcache_req_flags(idx)            := dcacheAdapter.io.memBus(j).req.bits.flags
      io.dcache_req_data(idx)             := dcacheAdapter.io.memBus(j).req.bits.data
      io.dcache_req_tag(idx)              := dcacheAdapter.io.memBus(j).req.bits.tag.asUInt
      dcacheAdapter.io.memBus(j).req.ready := io.dcache_req_ready(idx)

      dcacheAdapter.io.memBus(j).rsp.valid      := io.dcache_rsp_valid(idx)
      dcacheAdapter.io.memBus(j).rsp.bits.data  := io.dcache_rsp_data(idx)
      dcacheAdapter.io.memBus(j).rsp.bits.tag   := io.dcache_rsp_tag(idx).asTypeOf(new MemBusTagBundle(DCACHE_TAG_WIDTH, UUID_WIDTH))
      io.dcache_rsp_ready(idx)                  := dcacheAdapter.io.memBus(j).rsp.ready
    }
  }
}
