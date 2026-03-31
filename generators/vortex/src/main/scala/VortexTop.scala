// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from Vortex.sv, VX_cluster.sv, VX_socket.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// =============================================================================
// Locally computed cache topology constants
// =============================================================================
private object VortexTopParams {
  // DCACHE banks: MIN(DCACHE_NUM_REQS, 16)
  val DCACHE_NUM_BANKS: Int = math.min(DCACHE_NUM_REQS, 16)

  // L1 mem-side tag width (what CacheCluster exposes)
  //   mshrAddrW = log2Ceil(DCACHE_MSHR_SIZE)
  //   bankSelBits = max(1, log2Ceil(DCACHE_NUM_BANKS))
  //   L1_MEM_TAG_WIDTH = UUID_WIDTH + mshrAddrW + bankSelBits
  val DCACHE_MSHR_ADR_W: Int = log2Ceil(DCACHE_MSHR_SIZE)
  val DCACHE_BANK_SEL:   Int = math.max(1, log2Ceil(math.max(1, DCACHE_NUM_BANKS)))
  val L1_MEM_TAG_WIDTH:  Int = UUID_WIDTH + DCACHE_MSHR_ADR_W + DCACHE_BANK_SEL

  // L2 topology (L2 disabled by default → passthru)
  val L2_WORD_SIZE:     Int = L2_LINE_SIZE          // L2 word = full L1 line
  val L2_NUM_BANKS:     Int = 1
  val L2_MSHR_ADR_W:    Int = log2Ceil(L2_MSHR_SIZE)
  val L2_BANK_SEL:      Int = math.max(1, log2Ceil(math.max(1, L2_NUM_BANKS)))
  val L2_MEM_TAG_WIDTH: Int = UUID_WIDTH + L2_MSHR_ADR_W + L2_BANK_SEL

  // L3 topology (L3 disabled by default → passthru)
  val L3_WORD_SIZE:     Int = L3_LINE_SIZE
  val L3_NUM_BANKS:     Int = 1
  val L3_MSHR_ADR_W:    Int = log2Ceil(L3_MSHR_SIZE)
  val L3_BANK_SEL:      Int = math.max(1, log2Ceil(math.max(1, L3_NUM_BANKS)))
  val L3_MEM_TAG_WIDTH: Int = UUID_WIDTH + L3_MSHR_ADR_W + L3_BANK_SEL

  // Core-side request tag widths (for L2/L3 "word" address = line address)
  val L2_TAG_WIDTH: Int = UUID_WIDTH + log2Ceil(math.max(1, L2_MSHR_SIZE))
  val L3_TAG_WIDTH: Int = UUID_WIDTH + log2Ceil(math.max(1, L3_MSHR_SIZE))

  // Number of L2 request inputs (one L1 mem port per socket)
  val L2_NUM_REQS: Int = NUM_SOCKETS * L1_MEM_PORTS
  // Number of L3 request inputs (one L2 mem port per cluster)
  val L3_NUM_REQS: Int = NUM_CLUSTERS * L2_MEM_PORTS
}

// =============================================================================
// VxSocket — mirrors VX_socket.sv
// One socket contains SOCKET_SIZE cores sharing I/D caches.
// =============================================================================
class VxSocket(val socketId: Int = 0) extends Module {
  import VortexTopParams._

  private val l1LineSize  = L1_LINE_SIZE
  private val l1AddrWidth = MEM_ADDR_WIDTH - log2Ceil(l1LineSize)
  private val flagsWidth  = 3

  val io = IO(new Bundle {
    // DCR write bus (slave)
    val dcr_write_valid = Input(Bool())
    val dcr_write_addr  = Input(UInt(VX_DCR_ADDR_BITS.W))
    val dcr_write_data  = Input(UInt(32.W))

    // L1 memory bus (master)
    val mem_bus = Vec(L1_MEM_PORTS,
      new MemBusBundle(l1LineSize, l1AddrWidth, flagsWidth, L1_MEM_TAG_WIDTH, UUID_WIDTH))

    val busy = Output(Bool())
  })

  // Instantiate SOCKET_SIZE cores
  val cores = Seq.tabulate(SOCKET_SIZE)(i => Module(new VxCoreTop(
    coreId = socketId * SOCKET_SIZE + i
  )))

  // I-cache cluster (read-only, 1 req per core)
  val icache = Module(new CacheCluster(
    p          = CacheParams(
      cacheSize    = ICACHE_SIZE,
      lineSize     = L1_LINE_SIZE,
      numBanks     = 1,
      numWays      = ICACHE_NUM_WAYS,
      wordSize     = ICACHE_WORD_SIZE,
      memAddrWidth = MEM_ADDR_WIDTH,
      writeEnable  = 0
    ),
    numUnits   = NUM_ICACHES,
    numInputs  = SOCKET_SIZE,
    numReqs    = 1,
    memPorts   = ICACHE_MEM_PORTS,
    tagWidth   = ICACHE_TAG_WIDTH
  ))

  // D-cache cluster
  val dcache = Module(new CacheCluster(
    p          = CacheParams(
      cacheSize    = DCACHE_SIZE,
      lineSize     = L1_LINE_SIZE,
      numBanks     = DCACHE_NUM_BANKS,
      numWays      = DCACHE_NUM_WAYS,
      wordSize     = DCACHE_WORD_SIZE,
      memAddrWidth = MEM_ADDR_WIDTH,
      writeEnable  = 1,
      writeback    = DCACHE_WRITEBACK,
      dirtyBytes   = DCACHE_DIRTYBYTES,
      replPolicy   = DCACHE_REPL_POLICY
    ),
    numUnits   = NUM_DCACHES,
    numInputs  = SOCKET_SIZE,
    numReqs    = DCACHE_NUM_REQS,
    memPorts   = L1_MEM_PORTS,
    tagWidth   = DCACHE_TAG_WIDTH,
    ncEnable   = true
  ))

  // Broadcast DCR to all cores
  for (i <- 0 until SOCKET_SIZE) {
    cores(i).io.dcr_write_valid := io.dcr_write_valid
    cores(i).io.dcr_write_addr  := io.dcr_write_addr
    cores(i).io.dcr_write_data  := io.dcr_write_data
  }

  // Connect cores → icache core_bus
  for (i <- 0 until SOCKET_SIZE) {
    icache.io.core_bus(i).req.valid        := cores(i).io.icache_req_valid
    icache.io.core_bus(i).req.bits.rw      := false.B
    icache.io.core_bus(i).req.bits.addr    := cores(i).io.icache_req_addr
    icache.io.core_bus(i).req.bits.byteen  := Fill(ICACHE_WORD_SIZE, 1.U(1.W))
    icache.io.core_bus(i).req.bits.data    := 0.U
    icache.io.core_bus(i).req.bits.flags   := 0.U
    icache.io.core_bus(i).req.bits.tag     := cores(i).io.icache_req_tag.asTypeOf(
                                                icache.io.core_bus(i).req.bits.tag)
    cores(i).io.icache_req_ready           := icache.io.core_bus(i).req.ready
    cores(i).io.icache_rsp_valid           := icache.io.core_bus(i).rsp.valid
    cores(i).io.icache_rsp_data            := icache.io.core_bus(i).rsp.bits.data
    cores(i).io.icache_rsp_tag             := icache.io.core_bus(i).rsp.bits.tag.asUInt
    icache.io.core_bus(i).rsp.ready        := cores(i).io.icache_rsp_ready
  }

  // Connect cores → dcache core_bus (DCACHE_NUM_REQS per core)
  for (i <- 0 until SOCKET_SIZE) {
    for (j <- 0 until DCACHE_NUM_REQS) {
      val port = i * DCACHE_NUM_REQS + j
      dcache.io.core_bus(port).req.valid       := cores(i).io.dcache_req_valid(j)
      dcache.io.core_bus(port).req.bits.rw     := cores(i).io.dcache_req_rw(j)
      dcache.io.core_bus(port).req.bits.addr   := cores(i).io.dcache_req_addr(j)
      dcache.io.core_bus(port).req.bits.byteen := cores(i).io.dcache_req_byteen(j)
      dcache.io.core_bus(port).req.bits.data   := cores(i).io.dcache_req_data(j)
      dcache.io.core_bus(port).req.bits.flags  := cores(i).io.dcache_req_flags(j)
      dcache.io.core_bus(port).req.bits.tag    := cores(i).io.dcache_req_tag(j).asTypeOf(
                                                    dcache.io.core_bus(port).req.bits.tag)
      cores(i).io.dcache_req_ready(j)          := dcache.io.core_bus(port).req.ready
      cores(i).io.dcache_rsp_valid(j)          := dcache.io.core_bus(port).rsp.valid
      cores(i).io.dcache_rsp_data(j)           := dcache.io.core_bus(port).rsp.bits.data
      cores(i).io.dcache_rsp_tag(j)            := dcache.io.core_bus(port).rsp.bits.tag.asUInt
      dcache.io.core_bus(port).rsp.ready        := cores(i).io.dcache_rsp_ready(j)
    }
  }

  // Tie off icache mem ports (icache shares mem via L2 in real impl; stub here)
  for (p <- 0 until ICACHE_MEM_PORTS) {
    icache.io.mem_bus(p).req.ready    := false.B
    icache.io.mem_bus(p).rsp.valid    := false.B
    icache.io.mem_bus(p).rsp.bits    := 0.U.asTypeOf(icache.io.mem_bus(p).rsp.bits)
  }

  // Connect dcache mem ports → socket mem_bus
  for (p <- 0 until L1_MEM_PORTS) {
    io.mem_bus(p).req.valid       := dcache.io.mem_bus(p).req.valid
    io.mem_bus(p).req.bits        := dcache.io.mem_bus(p).req.bits.asTypeOf(io.mem_bus(p).req.bits)
    dcache.io.mem_bus(p).req.ready := io.mem_bus(p).req.ready
    dcache.io.mem_bus(p).rsp.valid := io.mem_bus(p).rsp.valid
    dcache.io.mem_bus(p).rsp.bits  := io.mem_bus(p).rsp.bits.asTypeOf(dcache.io.mem_bus(p).rsp.bits)
    io.mem_bus(p).rsp.ready        := dcache.io.mem_bus(p).rsp.ready
  }

  io.busy := cores.map(_.io.busy).reduce(_ || _)
}

// =============================================================================
// VxCluster — mirrors VX_cluster.sv
// One cluster contains NUM_SOCKETS sockets sharing an L2 cache.
// =============================================================================
class VxCluster(val clusterId: Int = 0) extends Module {
  import VortexTopParams._

  private val l2LineSize  = L2_LINE_SIZE
  private val l2AddrWidth = MEM_ADDR_WIDTH - log2Ceil(l2LineSize)
  private val flagsWidth  = 3

  val io = IO(new Bundle {
    val dcr_write_valid = Input(Bool())
    val dcr_write_addr  = Input(UInt(VX_DCR_ADDR_BITS.W))
    val dcr_write_data  = Input(UInt(32.W))

    val mem_bus = Vec(L2_MEM_PORTS,
      new MemBusBundle(l2LineSize, l2AddrWidth, flagsWidth, L2_MEM_TAG_WIDTH, UUID_WIDTH))

    val busy = Output(Bool())
  })

  val sockets = Seq.tabulate(NUM_SOCKETS)(i => Module(new VxSocket(
    socketId = clusterId * NUM_SOCKETS + i
  )))

  val l2cache = Module(new CacheWrap(
    p          = CacheParams(
      cacheSize    = L2_CACHE_SIZE,
      lineSize     = L2_LINE_SIZE,
      numBanks     = L2_NUM_BANKS,
      numWays      = L2_NUM_WAYS,
      wordSize     = L2_WORD_SIZE,
      memAddrWidth = MEM_ADDR_WIDTH,
      writeEnable  = 1,
      writeback    = L2_WRITEBACK,
      dirtyBytes   = L2_DIRTYBYTES,
      replPolicy   = L2_REPL_POLICY
    ),
    numReqs    = L2_NUM_REQS,
    memPorts   = L2_MEM_PORTS,
    tagWidth   = L2_TAG_WIDTH,
    ncEnable   = true,
    passthru   = (L2_ENABLED == 0)
  ))

  for (s <- sockets) {
    s.io.dcr_write_valid := io.dcr_write_valid
    s.io.dcr_write_addr  := io.dcr_write_addr
    s.io.dcr_write_data  := io.dcr_write_data
  }

  for (i <- 0 until NUM_SOCKETS) {
    for (p <- 0 until L1_MEM_PORTS) {
      val port = i * L1_MEM_PORTS + p
      l2cache.io.core_bus(port).req.valid       := sockets(i).io.mem_bus(p).req.valid
      l2cache.io.core_bus(port).req.bits        := sockets(i).io.mem_bus(p).req.bits.asTypeOf(
                                                     l2cache.io.core_bus(port).req.bits)
      sockets(i).io.mem_bus(p).req.ready        := l2cache.io.core_bus(port).req.ready
      sockets(i).io.mem_bus(p).rsp.valid        := l2cache.io.core_bus(port).rsp.valid
      sockets(i).io.mem_bus(p).rsp.bits         := l2cache.io.core_bus(port).rsp.bits.asTypeOf(
                                                     sockets(i).io.mem_bus(p).rsp.bits)
      l2cache.io.core_bus(port).rsp.ready        := sockets(i).io.mem_bus(p).rsp.ready
    }
  }

  for (p <- 0 until L2_MEM_PORTS) {
    io.mem_bus(p).req.valid        := l2cache.io.mem_bus(p).req.valid
    io.mem_bus(p).req.bits         := l2cache.io.mem_bus(p).req.bits.asTypeOf(io.mem_bus(p).req.bits)
    l2cache.io.mem_bus(p).req.ready := io.mem_bus(p).req.ready
    l2cache.io.mem_bus(p).rsp.valid := io.mem_bus(p).rsp.valid
    l2cache.io.mem_bus(p).rsp.bits  := io.mem_bus(p).rsp.bits.asTypeOf(l2cache.io.mem_bus(p).rsp.bits)
    io.mem_bus(p).rsp.ready         := l2cache.io.mem_bus(p).rsp.ready
  }

  io.busy := sockets.map(_.io.busy).reduce(_ || _)
}

// =============================================================================
// VortexTop — mirrors Vortex.sv
// Top-level GPU: NUM_CLUSTERS clusters through an L3 cache to memory ports.
// =============================================================================
class VortexTop extends Module {
  import VortexTopParams._

  private val memLineSize  = L3_LINE_SIZE
  private val memAddrWidth = VX_MEM_ADDR_WIDTH
  private val memDataWidth = VX_MEM_DATA_WIDTH
  private val flagsWidth   = 3

  val io = IO(new Bundle {
    val mem_bus = Vec(L3_MEM_PORTS,
      new MemBusBundle(memLineSize, memAddrWidth, flagsWidth, L3_MEM_TAG_WIDTH, UUID_WIDTH))

    val dcr_write_valid = Input(Bool())
    val dcr_write_addr  = Input(UInt(VX_DCR_ADDR_BITS.W))
    val dcr_write_data  = Input(UInt(32.W))

    val busy = Output(Bool())
  })

  val clusters = Seq.tabulate(NUM_CLUSTERS)(i => Module(new VxCluster(clusterId = i)))

  val l3cache = Module(new CacheWrap(
    p          = CacheParams(
      cacheSize    = L3_CACHE_SIZE,
      lineSize     = L3_LINE_SIZE,
      numBanks     = L3_NUM_BANKS,
      numWays      = L3_NUM_WAYS,
      wordSize     = L3_WORD_SIZE,
      memAddrWidth = MEM_ADDR_WIDTH,
      writeEnable  = 1,
      writeback    = L3_WRITEBACK,
      dirtyBytes   = L3_DIRTYBYTES,
      replPolicy   = L3_REPL_POLICY
    ),
    numReqs    = L3_NUM_REQS,
    memPorts   = L3_MEM_PORTS,
    tagWidth   = L3_TAG_WIDTH,
    ncEnable   = true,
    passthru   = (L3_ENABLED == 0)
  ))

  for (c <- clusters) {
    c.io.dcr_write_valid := io.dcr_write_valid
    c.io.dcr_write_addr  := io.dcr_write_addr
    c.io.dcr_write_data  := io.dcr_write_data
  }

  for (i <- 0 until NUM_CLUSTERS) {
    for (p <- 0 until L2_MEM_PORTS) {
      val port = i * L2_MEM_PORTS + p
      l3cache.io.core_bus(port).req.valid        := clusters(i).io.mem_bus(p).req.valid
      l3cache.io.core_bus(port).req.bits         := clusters(i).io.mem_bus(p).req.bits.asTypeOf(
                                                      l3cache.io.core_bus(port).req.bits)
      clusters(i).io.mem_bus(p).req.ready        := l3cache.io.core_bus(port).req.ready
      clusters(i).io.mem_bus(p).rsp.valid        := l3cache.io.core_bus(port).rsp.valid
      clusters(i).io.mem_bus(p).rsp.bits         := l3cache.io.core_bus(port).rsp.bits.asTypeOf(
                                                      clusters(i).io.mem_bus(p).rsp.bits)
      l3cache.io.core_bus(port).rsp.ready         := clusters(i).io.mem_bus(p).rsp.ready
    }
  }

  for (p <- 0 until L3_MEM_PORTS) {
    io.mem_bus(p).req.valid         := l3cache.io.mem_bus(p).req.valid
    io.mem_bus(p).req.bits          := l3cache.io.mem_bus(p).req.bits.asTypeOf(io.mem_bus(p).req.bits)
    l3cache.io.mem_bus(p).req.ready  := io.mem_bus(p).req.ready
    l3cache.io.mem_bus(p).rsp.valid  := io.mem_bus(p).rsp.valid
    l3cache.io.mem_bus(p).rsp.bits   := io.mem_bus(p).rsp.bits.asTypeOf(l3cache.io.mem_bus(p).rsp.bits)
    io.mem_bus(p).rsp.ready          := l3cache.io.mem_bus(p).rsp.ready
  }

  io.busy := clusters.map(_.io.busy).reduce(_ || _)
}
