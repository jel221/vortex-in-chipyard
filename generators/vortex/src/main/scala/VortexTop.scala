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
  // L1 D-cache mem-side tag width.
  //   bankSelBits = if (numBanks > 1) log2Ceil(numBanks) else 0  (mirrors Cache.scala)
  val DCACHE_MSHR_ADR_W: Int = log2Ceil(DCACHE_MSHR_SIZE)
  val DCACHE_BANK_SEL:   Int = if (DCACHE_NUM_BANKS > 1) log2Ceil(DCACHE_NUM_BANKS) else 0
  val L1_MEM_TAG_WIDTH:  Int = UUID_WIDTH + DCACHE_MSHR_ADR_W + DCACHE_BANK_SEL

  // I-cache mem-side tag width (numBanks=1 → bankSelBits=0)
  val ICACHE_MSHR_ADR_W:    Int = math.max(1, log2Ceil(ICACHE_MSHR_SIZE))
  val ICACHE_BANK_SEL:      Int = 0
  val ICACHE_MEM_TAG_WIDTH: Int = UUID_WIDTH + ICACHE_MSHR_ADR_W + ICACHE_BANK_SEL

  // After merging icache+dcache at socket level with a priority arbiter (VX_mem_arb),
  // the socket output tag adds 1 arb-selector bit at the LSB (0=icache, 1=dcache).
  val L1_MEM_TAG_UNIFIED:   Int = math.max(L1_MEM_TAG_WIDTH, ICACHE_MEM_TAG_WIDTH)
  val L1_MEM_ARB_TAG_WIDTH: Int = L1_MEM_TAG_UNIFIED + 1

  // L2 topology
  val L2_WORD_SIZE:     Int = L2_LINE_SIZE
  val L2_NUM_BANKS:     Int = 1
  val L2_MSHR_ADR_W:    Int = log2Ceil(L2_MSHR_SIZE)
  val L2_BANK_SEL:      Int = math.max(1, log2Ceil(math.max(1, L2_NUM_BANKS)))
  val L2_MEM_TAG_WIDTH: Int = UUID_WIDTH + L2_MSHR_ADR_W + L2_BANK_SEL

  // L3 topology
  val L3_WORD_SIZE:     Int = L3_LINE_SIZE
  val L3_NUM_BANKS:     Int = 1
  val L3_MSHR_ADR_W:    Int = log2Ceil(L3_MSHR_SIZE)
  val L3_BANK_SEL:      Int = math.max(1, log2Ceil(math.max(1, L3_NUM_BANKS)))
  val L3_MEM_TAG_WIDTH: Int = UUID_WIDTH + L3_MSHR_ADR_W + L3_BANK_SEL

  // Core-side request tag widths fed into CacheWrap:
  //   L2 receives L1_MEM_ARB_TAG_WIDTH-wide tags from sockets
  //   L3 receives L2_MEM_TAG_WIDTH-wide tags from clusters
  val L2_TAG_WIDTH: Int = L1_MEM_ARB_TAG_WIDTH
  val L3_TAG_WIDTH: Int = L2_MEM_TAG_WIDTH

  // Number of L2 request inputs: one merged mem port per socket (icache folded in at socket level)
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
    // Direct from RoCC (same pattern as startup_addr)
    val startup_addr = Input(UInt(XLEN.W))
    val mpm_class    = Input(UInt(8.W))

    // Merged icache+dcache memory bus (master).
    // Icache is merged with dcache[0] via a priority arbiter; arb selector at tag.value LSB.
    val mem_bus = Vec(L1_MEM_PORTS,
      new MemBusBundle(l1LineSize, l1AddrWidth, flagsWidth, L1_MEM_ARB_TAG_WIDTH, UUID_WIDTH))

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
    tagWidth   = ICACHE_TAG_WIDTH,
    uuidWidth  = UUID_WIDTH
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
    uuidWidth  = UUID_WIDTH
  ))

  // Broadcast startup_addr and mpm_class to all cores
  for (i <- 0 until SOCKET_SIZE) {
    cores(i).io.startup_addr := io.startup_addr
    cores(i).io.mpm_class    := io.mpm_class
  }

  // Connect cores → icache core_bus
  for (i <- 0 until SOCKET_SIZE) {
    icache.io.core_bus(i).req.valid        := cores(i).io.icache_req_valid
    icache.io.core_bus(i).req.bits.rw      := false.B
    icache.io.core_bus(i).req.bits.addr    := cores(i).io.icache_req_addr
    icache.io.core_bus(i).req.bits.byteen  := Fill(ICACHE_WORD_SIZE, 1.U(1.W))
    icache.io.core_bus(i).req.bits.data    := 0.U
    icache.io.core_bus(i).req.bits.flags   := 0.U
    icache.io.core_bus(i).req.bits.tag.uuid  := cores(i).io.icache_req_tag(ICACHE_TAG_WIDTH - 1, ICACHE_TAG_WIDTH - UUID_WIDTH)
    icache.io.core_bus(i).req.bits.tag.value := cores(i).io.icache_req_tag(ICACHE_TAG_WIDTH - UUID_WIDTH - 1, 0)
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
      dcache.io.core_bus(port).req.bits.tag.uuid  := cores(i).io.dcache_req_tag(j)(DCACHE_TAG_WIDTH - 1, DCACHE_TAG_WIDTH - UUID_WIDTH)
      dcache.io.core_bus(port).req.bits.tag.value := cores(i).io.dcache_req_tag(j)(DCACHE_TAG_WIDTH - UUID_WIDTH - 1, 0)
      cores(i).io.dcache_req_ready(j)          := dcache.io.core_bus(port).req.ready
      cores(i).io.dcache_rsp_valid(j)          := dcache.io.core_bus(port).rsp.valid
      cores(i).io.dcache_rsp_data(j)           := dcache.io.core_bus(port).rsp.bits.data
      cores(i).io.dcache_rsp_tag(j)            := dcache.io.core_bus(port).rsp.bits.tag.asUInt
      dcache.io.core_bus(port).rsp.ready        := cores(i).io.dcache_rsp_ready(j)
    }
  }


  val ich = icache.io.mem_bus(0)
  val dch = dcache.io.mem_bus(0)
  val out = io.mem_bus(0)
  val icache_wins = ich.req.valid

  out.req.valid          := ich.req.valid || dch.req.valid
  out.req.bits.rw        := Mux(icache_wins, false.B,                dch.req.bits.rw)
  out.req.bits.addr      := Mux(icache_wins, ich.req.bits.addr,      dch.req.bits.addr)
  out.req.bits.data      := Mux(icache_wins, ich.req.bits.data,      dch.req.bits.data)
  out.req.bits.byteen    := Mux(icache_wins, ich.req.bits.byteen,    dch.req.bits.byteen)
  out.req.bits.flags     := Mux(icache_wins, ich.req.bits.flags,     dch.req.bits.flags)
  out.req.bits.tag.uuid  := Mux(icache_wins, ich.req.bits.tag.uuid,  dch.req.bits.tag.uuid)
  out.req.bits.tag.value := Cat(
      Mux(icache_wins,
      ich.req.bits.tag.value.pad(L1_MEM_TAG_UNIFIED - UUID_WIDTH),
      dch.req.bits.tag.value.pad(L1_MEM_TAG_UNIFIED - UUID_WIDTH)),
      Mux(icache_wins, 0.U(1.W), 1.U(1.W))
  )
  ich.req.ready := out.req.ready &&  icache_wins
  dch.req.ready := out.req.ready && !icache_wins

  val rsp_arb_bit = out.rsp.bits.tag.value(0)
  val rsp_val     = out.rsp.bits.tag.value(L1_MEM_ARB_TAG_WIDTH - UUID_WIDTH - 1, 1)
  ich.rsp.valid          := out.rsp.valid && !rsp_arb_bit
  ich.rsp.bits.data      := out.rsp.bits.data
  ich.rsp.bits.tag.uuid  := out.rsp.bits.tag.uuid
  ich.rsp.bits.tag.value := rsp_val(ICACHE_MEM_TAG_WIDTH - UUID_WIDTH - 1, 0)
  dch.rsp.valid          := out.rsp.valid &&  rsp_arb_bit
  dch.rsp.bits.data      := out.rsp.bits.data
  dch.rsp.bits.tag.uuid  := out.rsp.bits.tag.uuid
  dch.rsp.bits.tag.value := rsp_val(L1_MEM_TAG_WIDTH - UUID_WIDTH - 1, 0)
  out.rsp.ready          := Mux(!rsp_arb_bit, ich.rsp.ready, dch.rsp.ready)


  // Ports 1..L1_MEM_PORTS-1: dcache only.
  // Mirrors ASSIGN_VX_MEM_BUS_IF_EX(mem_bus_if[i], dcache_mem_bus_if[i],
  //   L1_MEM_ARB_TAG_WIDTH, DCACHE_MEM_TAG_WIDTH, UUID_WIDTH) where TD > TS:
  //   req: {uuid, zeros(TD-TS), dcache_value}  → zero-extend value at MSB
  //   rsp: dcache.value = out.value[TS-UUID-1:0]  → truncate to lower bits
  for (p <- 1 until L1_MEM_PORTS) {
    val dch = dcache.io.mem_bus(p)
    val out = io.mem_bus(p)
    out.req.valid          := dch.req.valid
    out.req.bits.rw        := dch.req.bits.rw
    out.req.bits.addr      := dch.req.bits.addr
    out.req.bits.data      := dch.req.bits.data
    out.req.bits.byteen    := dch.req.bits.byteen
    out.req.bits.flags     := dch.req.bits.flags
    out.req.bits.tag.uuid  := dch.req.bits.tag.uuid
    out.req.bits.tag.value := dch.req.bits.tag.value.pad(L1_MEM_ARB_TAG_WIDTH - UUID_WIDTH)
    dch.req.ready          := out.req.ready
    dch.rsp.valid          := out.rsp.valid
    dch.rsp.bits.data      := out.rsp.bits.data
    dch.rsp.bits.tag.uuid  := out.rsp.bits.tag.uuid
    dch.rsp.bits.tag.value := out.rsp.bits.tag.value(L1_MEM_TAG_WIDTH - UUID_WIDTH - 1, 0)
    out.rsp.ready          := dch.rsp.ready
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
    // Direct from RoCC (same pattern as startup_addr)
    val startup_addr = Input(UInt(XLEN.W))
    val mpm_class    = Input(UInt(8.W))

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
    uuidWidth  = 0,
  ))

  for (s <- sockets) {
    s.io.startup_addr := io.startup_addr
    s.io.mpm_class    := io.mpm_class
  }

  for (i <- 0 until NUM_SOCKETS) {
    for (p <- 0 until L1_MEM_PORTS) {
      val port = i * L1_MEM_PORTS + p
      // l2cache uses uuidWidth=0: pack {uuid,value} into flat tag.value on requests;
      // unpack flat tag.value back into {uuid,value} on responses.
      l2cache.io.core_bus(port).req.valid          := sockets(i).io.mem_bus(p).req.valid
      l2cache.io.core_bus(port).req.bits.rw        := sockets(i).io.mem_bus(p).req.bits.rw
      l2cache.io.core_bus(port).req.bits.addr      := sockets(i).io.mem_bus(p).req.bits.addr
      l2cache.io.core_bus(port).req.bits.data      := sockets(i).io.mem_bus(p).req.bits.data
      l2cache.io.core_bus(port).req.bits.byteen    := sockets(i).io.mem_bus(p).req.bits.byteen
      l2cache.io.core_bus(port).req.bits.flags     := sockets(i).io.mem_bus(p).req.bits.flags
      l2cache.io.core_bus(port).req.bits.tag.uuid  := DontCare  // uuidWidth=0, field is 0-wide
      l2cache.io.core_bus(port).req.bits.tag.value := Cat(sockets(i).io.mem_bus(p).req.bits.tag.uuid,
                                                          sockets(i).io.mem_bus(p).req.bits.tag.value)
      sockets(i).io.mem_bus(p).req.ready           := l2cache.io.core_bus(port).req.ready
      sockets(i).io.mem_bus(p).rsp.valid           := l2cache.io.core_bus(port).rsp.valid
      sockets(i).io.mem_bus(p).rsp.bits.data       := l2cache.io.core_bus(port).rsp.bits.data
      val coreRspTag = l2cache.io.core_bus(port).rsp.bits.tag.value
      sockets(i).io.mem_bus(p).rsp.bits.tag.uuid   := coreRspTag(L2_TAG_WIDTH - 1, L2_TAG_WIDTH - UUID_WIDTH)
      sockets(i).io.mem_bus(p).rsp.bits.tag.value  := coreRspTag(L2_TAG_WIDTH - UUID_WIDTH - 1, 0)
      l2cache.io.core_bus(port).rsp.ready          := sockets(i).io.mem_bus(p).rsp.ready
    }
  }

  for (p <- 0 until L2_MEM_PORTS) {
    // l2cache mem_bus uses uuidWidth=0; actual tag is wider than L2_MEM_TAG_WIDTH by 1 NC bit.
    // Split the lower L2_MEM_TAG_WIDTH bits into {uuid, value}; NC bit (MSB) is always 0.
    io.mem_bus(p).req.valid          := l2cache.io.mem_bus(p).req.valid
    io.mem_bus(p).req.bits.rw        := l2cache.io.mem_bus(p).req.bits.rw
    io.mem_bus(p).req.bits.addr      := l2cache.io.mem_bus(p).req.bits.addr
    io.mem_bus(p).req.bits.data      := l2cache.io.mem_bus(p).req.bits.data
    io.mem_bus(p).req.bits.byteen    := l2cache.io.mem_bus(p).req.bits.byteen
    io.mem_bus(p).req.bits.flags     := l2cache.io.mem_bus(p).req.bits.flags
    io.mem_bus(p).req.bits.tag.uuid  := l2cache.io.mem_bus(p).req.bits.tag.uuid // l2memReqTag(L2_MEM_TAG_WIDTH - 1, L2_MEM_TAG_WIDTH - UUID_WIDTH)
    io.mem_bus(p).req.bits.tag.value := l2cache.io.mem_bus(p).req.bits.tag.value
    l2cache.io.mem_bus(p).req.ready   := io.mem_bus(p).req.ready
    l2cache.io.mem_bus(p).rsp.valid   := io.mem_bus(p).rsp.valid
    l2cache.io.mem_bus(p).rsp.bits.data      := io.mem_bus(p).rsp.bits.data
    l2cache.io.mem_bus(p).rsp.bits.tag.uuid  := DontCare  // uuidWidth=0, field is 0-wide
    l2cache.io.mem_bus(p).rsp.bits.tag.value := Cat(io.mem_bus(p).rsp.bits.tag.uuid,
                                                     io.mem_bus(p).rsp.bits.tag.value)
    io.mem_bus(p).rsp.ready           := l2cache.io.mem_bus(p).rsp.ready
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

    // Direct from RoCC (same pattern as startup_addr)
    val startup_addr = Input(UInt(XLEN.W))
    val mpm_class    = Input(UInt(8.W))

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
    uuidWidth  = 0,
  ))

  for (c <- clusters) {
    c.io.startup_addr := io.startup_addr
    c.io.mpm_class    := io.mpm_class
  }

  for (i <- 0 until NUM_CLUSTERS) {
    for (p <- 0 until L2_MEM_PORTS) {
      val port = i * L2_MEM_PORTS + p
      // l3cache uses uuidWidth=0: pack {uuid,value} into flat tag.value on requests;
      // unpack flat tag.value back into {uuid,value} on responses.
      l3cache.io.core_bus(port).req.valid          := clusters(i).io.mem_bus(p).req.valid
      l3cache.io.core_bus(port).req.bits.rw        := clusters(i).io.mem_bus(p).req.bits.rw
      l3cache.io.core_bus(port).req.bits.addr      := clusters(i).io.mem_bus(p).req.bits.addr
      l3cache.io.core_bus(port).req.bits.data      := clusters(i).io.mem_bus(p).req.bits.data
      l3cache.io.core_bus(port).req.bits.byteen    := clusters(i).io.mem_bus(p).req.bits.byteen
      l3cache.io.core_bus(port).req.bits.flags     := clusters(i).io.mem_bus(p).req.bits.flags
      l3cache.io.core_bus(port).req.bits.tag.uuid  := DontCare  // uuidWidth=0, field is 0-wide
      l3cache.io.core_bus(port).req.bits.tag.value := Cat(clusters(i).io.mem_bus(p).req.bits.tag.uuid,
                                                          clusters(i).io.mem_bus(p).req.bits.tag.value)
      clusters(i).io.mem_bus(p).req.ready          := l3cache.io.core_bus(port).req.ready
      clusters(i).io.mem_bus(p).rsp.valid          := l3cache.io.core_bus(port).rsp.valid
      clusters(i).io.mem_bus(p).rsp.bits.data      := l3cache.io.core_bus(port).rsp.bits.data
      val coreRspTag = l3cache.io.core_bus(port).rsp.bits.tag.value
      clusters(i).io.mem_bus(p).rsp.bits.tag.uuid  := coreRspTag(L3_TAG_WIDTH - 1, L3_TAG_WIDTH - UUID_WIDTH)
      clusters(i).io.mem_bus(p).rsp.bits.tag.value := coreRspTag(L3_TAG_WIDTH - UUID_WIDTH - 1, 0)
      l3cache.io.core_bus(port).rsp.ready          := clusters(i).io.mem_bus(p).rsp.ready
    }
  }

  for (p <- 0 until L3_MEM_PORTS) {
    // l3cache mem_bus uses uuidWidth=0; actual tag is wider than L3_MEM_TAG_WIDTH by 1 NC bit.
    // Split the lower L3_MEM_TAG_WIDTH bits into {uuid, value}; NC bit (MSB) is always 0.
    io.mem_bus(p).req.valid          := l3cache.io.mem_bus(p).req.valid
    io.mem_bus(p).req.bits.rw        := l3cache.io.mem_bus(p).req.bits.rw
    io.mem_bus(p).req.bits.addr      := l3cache.io.mem_bus(p).req.bits.addr
    io.mem_bus(p).req.bits.data      := l3cache.io.mem_bus(p).req.bits.data
    io.mem_bus(p).req.bits.byteen    := l3cache.io.mem_bus(p).req.bits.byteen
    io.mem_bus(p).req.bits.flags     := l3cache.io.mem_bus(p).req.bits.flags
    io.mem_bus(p).req.bits.tag.uuid  := l3cache.io.mem_bus(p).req.bits.tag.uuid // l3memReqTag(L3_MEM_TAG_WIDTH - 1, L3_MEM_TAG_WIDTH - UUID_WIDTH)
    io.mem_bus(p).req.bits.tag.value := l3cache.io.mem_bus(p).req.bits.tag.value //l3memReqTag(L3_MEM_TAG_WIDTH - UUID_WIDTH - 1, 0)
    l3cache.io.mem_bus(p).req.ready   := io.mem_bus(p).req.ready
    l3cache.io.mem_bus(p).rsp.valid   := io.mem_bus(p).rsp.valid
    l3cache.io.mem_bus(p).rsp.bits.data      := io.mem_bus(p).rsp.bits.data
    l3cache.io.mem_bus(p).rsp.bits.tag.uuid  := DontCare  // uuidWidth=0, field is 0-wide
    l3cache.io.mem_bus(p).rsp.bits.tag.value := Cat(io.mem_bus(p).rsp.bits.tag.uuid,
                                                     io.mem_bus(p).rsp.bits.tag.value)
    io.mem_bus(p).rsp.ready           := l3cache.io.mem_bus(p).rsp.ready
  }

  io.busy := clusters.map(_.io.busy).reduce(_ || _)
}
