// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Chisel translations of:
//   VX_cache_top.sv    → CacheTop
//   VX_cache_wrap.sv   → CacheWrap
//   VX_cache_cluster.sv → CacheCluster

package vortex

import chisel3._
import chisel3.util._

// =============================================================================
// CacheWrap
//
// Mirrors VX_cache_wrap.sv.
//
// Adds optional bypass (NC_ENABLE / PASSTHRU) around the Cache core.
// When passthru=true the Cache is not instantiated and all requests go
// directly through CacheBypass.
//
// Parameters:
//   p            – CacheParams
//   numReqs      – NUM_REQS
//   memPorts     – MEM_PORTS
//   tagSelIdx    – TAG_SEL_IDX
//   ncEnable     – NC_ENABLE
//   passthru     – PASSTHRU
//   crsqSize/etc – queue depths
//   tagWidth     – TAG_WIDTH
//   uuidWidth    – UUID sub-field width
//   coreOutBuf   – CORE_OUT_BUF
//   memOutBuf    – MEM_OUT_BUF
// =============================================================================
class CacheWrap(
  p:          CacheParams,
  numReqs:    Int = 4,
  memPorts:   Int = 1,
  tagSelIdx:  Int = 0,
  ncEnable:   Boolean = false,
  passthru:   Boolean = false,
  crsqSize:   Int = 4,
  mshrSize:   Int = 16,
  mrsqSize:   Int = 4,
  mreqSize:   Int = 4,
  tagWidth:   Int = 8,
  uuidWidth:  Int = 0,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val numBanks       = p.numBanks
  private val lineSize       = p.lineSize
  private val wordSize       = p.wordSize
  private val lineWidth      = p.lineWidth
  private val memAddrWidthCS = p.memAddrWidthCS
  private val flagsWidth     = 3
  private val mshrAddrWidth  = math.max(1, log2Ceil(mshrSize))
  private val bankSelBits    = p.bankSelBits
  private val bankMemTagWidth = uuidWidth + mshrAddrWidth
  // cache mem tag width (without bypass)
  private val cacheMTagWidth = bankMemTagWidth + bankSelBits
  // bypass tag width  (simplified; see CacheBypass)
  private val bypassTagWidth = uuidWidth + tagWidth + (if (p.wordsPerLine > 1) log2Ceil(p.wordsPerLine) else 0)
  private val memTagWidth    = if (passthru) bypassTagWidth
                               else if (ncEnable) math.max(cacheMTagWidth, bypassTagWidth) + 1
                               else cacheMTagWidth

  val io = IO(new Bundle {
    val core_bus = Vec(numReqs, Flipped(new MemBusBundle(wordSize, p.wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
    val mem_bus  = Vec(memPorts, new MemBusBundle(lineSize, memAddrWidthCS, flagsWidth, memTagWidth, uuidWidth))
  })

  val bypassEnable = ncEnable || passthru

  // ---- wires for the core bus that goes into the cache core ----------------
  val core_bus_cache_valid  = Wire(Vec(numReqs, Bool()))
  val core_bus_cache_rdata  = Wire(Vec(numReqs, new MemBusReqBundle(wordSize, p.wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
  val core_bus_cache_ready  = Wire(Vec(numReqs, Bool()))
  val core_bus_cache_rsp_v  = Wire(Vec(numReqs, Bool()))
  val core_bus_cache_rsp_d  = Wire(Vec(numReqs, new MemBusRspBundle(wordSize, tagWidth, uuidWidth)))
  val core_bus_cache_rsp_r  = Wire(Vec(numReqs, Bool()))

  // ---- cache mem bus (before bypass merge) ---------------------------------
  val mem_bus_cache_valid   = Wire(Vec(memPorts, Bool()))
  val mem_bus_cache_req     = Wire(Vec(memPorts, new MemBusReqBundle(lineSize, memAddrWidthCS, flagsWidth, cacheMTagWidth, uuidWidth)))
  val mem_bus_cache_ready   = Wire(Vec(memPorts, Bool()))
  val mem_bus_cache_rsp_v   = Wire(Vec(memPorts, Bool()))
  val mem_bus_cache_rsp_d   = Wire(Vec(memPorts, new MemBusRspBundle(lineSize, cacheMTagWidth, uuidWidth)))
  val mem_bus_cache_rsp_r   = Wire(Vec(memPorts, Bool()))

  if (bypassEnable) {
    val bypass = Module(new CacheBypass(
      numReqs       = numReqs,
      memPorts      = memPorts,
      tagSelIdx     = tagSelIdx,
      cacheEnable   = !passthru,
      wordSize      = wordSize,
      lineSize      = lineSize,
      coreAddrWidth = p.wordAddrWidth,
      coreTagWidth  = tagWidth,
      memAddrWidth  = memAddrWidthCS,
      memTagInWidth = cacheMTagWidth,
      uuidWidth     = uuidWidth,
      flagsWidth    = flagsWidth
    ))

    // Core bus in → bypass
    for (i <- 0 until numReqs) {
      bypass.io.core_bus_in(i) <> io.core_bus(i)
    }
    // Bypass cache path → cache core input
    for (i <- 0 until numReqs) {
      core_bus_cache_valid(i) := bypass.io.core_bus_out(i).req.valid
      core_bus_cache_rdata(i) := bypass.io.core_bus_out(i).req.bits
      bypass.io.core_bus_out(i).req.ready := core_bus_cache_ready(i)
      bypass.io.core_bus_out(i).rsp.valid := core_bus_cache_rsp_v(i)
      bypass.io.core_bus_out(i).rsp.bits  := core_bus_cache_rsp_d(i)
      core_bus_cache_rsp_r(i) := bypass.io.core_bus_out(i).rsp.ready
    }
    // Cache mem bus → bypass mem in
    for (i <- 0 until memPorts) {
      bypass.io.mem_bus_in(i).req.valid := mem_bus_cache_valid(i)
      bypass.io.mem_bus_in(i).req.bits  := mem_bus_cache_req(i)
      mem_bus_cache_ready(i) := bypass.io.mem_bus_in(i).req.ready
      mem_bus_cache_rsp_v(i) := bypass.io.mem_bus_in(i).rsp.valid
      mem_bus_cache_rsp_d(i) := bypass.io.mem_bus_in(i).rsp.bits
      bypass.io.mem_bus_in(i).rsp.ready := mem_bus_cache_rsp_r(i)
    }
    // Bypass mem out → io.mem_bus
    // Use field-by-field assignment (mirrors ASSIGN_VX_MEM_BUS_IF_EX in VX_define.vh).
    // The bypass output tag is memTagOutWidth bits while io.mem_bus tag is memTagWidth
    // bits (one wider for the NC discriminator).  A bulk asTypeOf would reinterpret the
    // raw bits and shift every field by one position, corrupting addr and all others.
    for (i <- 0 until memPorts) {
      io.mem_bus(i).req.valid          := bypass.io.mem_bus_out(i).req.valid
      io.mem_bus(i).req.bits.rw        := bypass.io.mem_bus_out(i).req.bits.rw
      io.mem_bus(i).req.bits.addr      := bypass.io.mem_bus_out(i).req.bits.addr
      io.mem_bus(i).req.bits.data      := bypass.io.mem_bus_out(i).req.bits.data
      io.mem_bus(i).req.bits.byteen    := bypass.io.mem_bus_out(i).req.bits.byteen
      io.mem_bus(i).req.bits.flags     := bypass.io.mem_bus_out(i).req.bits.flags
      io.mem_bus(i).req.bits.tag       := bypass.io.mem_bus_out(i).req.bits.tag.asUInt
                                          .asTypeOf(io.mem_bus(i).req.bits.tag)
      bypass.io.mem_bus_out(i).req.ready := io.mem_bus(i).req.ready
      bypass.io.mem_bus_out(i).rsp.valid := io.mem_bus(i).rsp.valid
      bypass.io.mem_bus_out(i).rsp.bits.data := io.mem_bus(i).rsp.bits.data
      bypass.io.mem_bus_out(i).rsp.bits.tag  := io.mem_bus(i).rsp.bits.tag.asUInt
                                                .asTypeOf(bypass.io.mem_bus_out(i).rsp.bits.tag)
      io.mem_bus(i).rsp.ready := bypass.io.mem_bus_out(i).rsp.ready
    }
  } else {
    // No bypass: direct connection
    for (i <- 0 until numReqs) {
      core_bus_cache_valid(i) := io.core_bus(i).req.valid
      core_bus_cache_rdata(i) := io.core_bus(i).req.bits
      io.core_bus(i).req.ready := core_bus_cache_ready(i)
      io.core_bus(i).rsp.valid := core_bus_cache_rsp_v(i)
      io.core_bus(i).rsp.bits  := core_bus_cache_rsp_d(i)
      core_bus_cache_rsp_r(i) := io.core_bus(i).rsp.ready
    }
    for (i <- 0 until memPorts) {
      io.mem_bus(i).req.valid := mem_bus_cache_valid(i)
      io.mem_bus(i).req.bits  := mem_bus_cache_req(i).asTypeOf(io.mem_bus(i).req.bits)
      mem_bus_cache_ready(i) := io.mem_bus(i).req.ready
      mem_bus_cache_rsp_v(i) := io.mem_bus(i).rsp.valid
      mem_bus_cache_rsp_d(i) := io.mem_bus(i).rsp.bits
                                  .asTypeOf(mem_bus_cache_rsp_d(i))
      io.mem_bus(i).rsp.ready := mem_bus_cache_rsp_r(i)
    }
  }

  if (!passthru) {
    val cache = Module(new Cache(
      p          = p,
      numReqs    = numReqs,
      memPorts   = memPorts,
      crsqSize   = crsqSize,
      mshrSize   = mshrSize,
      mrsqSize   = mrsqSize,
      mreqSize   = mreqSize,
      tagWidth   = tagWidth,
      uuidWidth  = uuidWidth,
      coreOutBuf = if (bypassEnable) 1 else coreOutBuf,
      memOutBuf  = if (bypassEnable) 1 else memOutBuf
    ))

    for (i <- 0 until numReqs) {
      cache.io.core_bus(i).req.valid := core_bus_cache_valid(i)
      cache.io.core_bus(i).req.bits  := core_bus_cache_rdata(i)
      core_bus_cache_ready(i)        := cache.io.core_bus(i).req.ready
      core_bus_cache_rsp_v(i)        := cache.io.core_bus(i).rsp.valid
      core_bus_cache_rsp_d(i)        := cache.io.core_bus(i).rsp.bits
      cache.io.core_bus(i).rsp.ready := core_bus_cache_rsp_r(i)
    }
    for (i <- 0 until memPorts) {
      mem_bus_cache_valid(i) := cache.io.mem_bus(i).req.valid
      mem_bus_cache_req(i)   := cache.io.mem_bus(i).req.bits
                                  .asTypeOf(mem_bus_cache_req(i))
      cache.io.mem_bus(i).req.ready := mem_bus_cache_ready(i)
      cache.io.mem_bus(i).rsp.valid := mem_bus_cache_rsp_v(i)
      cache.io.mem_bus(i).rsp.bits  := mem_bus_cache_rsp_d(i)
                                        .asTypeOf(cache.io.mem_bus(i).rsp.bits)
      mem_bus_cache_rsp_r(i) := cache.io.mem_bus(i).rsp.ready
    }
  } else {
    // Passthru: cache core not instantiated; tie off cache side
    for (i <- 0 until numReqs) {
      core_bus_cache_ready(i) := false.B
      core_bus_cache_rsp_v(i) := false.B
      core_bus_cache_rsp_d(i) := DontCare
    }
    for (i <- 0 until memPorts) {
      mem_bus_cache_valid(i) := false.B
      mem_bus_cache_req(i)   := DontCare
      mem_bus_cache_rsp_r(i) := false.B
    }
  }
}


// =============================================================================
// CacheTop
//
// Mirrors VX_cache_top.sv.
//
// Flat port interface (arrays of scalar/UInt ports) that wraps CacheWrap.
// Useful as a black-box boundary or for top-level simulation.
// =============================================================================
class CacheTop(
  p:          CacheParams,
  numReqs:    Int = 4,
  memPorts:   Int = 1,
  crsqSize:   Int = 8,
  mshrSize:   Int = 16,
  mrsqSize:   Int = 8,
  mreqSize:   Int = 8,
  tagWidth:   Int = 32,
  uuidWidth:  Int = 0,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val lineSize       = p.lineSize
  private val wordSize       = p.wordSize
  private val wordWidth      = p.wordWidth
  private val lineWidth      = p.lineWidth
  private val wordAddrWidth  = p.wordAddrWidth
  private val memAddrWidthCS = p.memAddrWidthCS
  private val flagsWidth     = 3
  private val mshrAddrWidth  = math.max(1, log2Ceil(mshrSize))
  private val bankSelBits    = p.bankSelBits
  private val bankMemTagWidth = uuidWidth + mshrAddrWidth
  private val memTagWidth    = bankMemTagWidth + bankSelBits

  val io = IO(new Bundle {
    // Core request
    val core_req_valid  = Input(Vec(numReqs, Bool()))
    val core_req_rw     = Input(Vec(numReqs, Bool()))
    val core_req_byteen = Input(Vec(numReqs, UInt(wordSize.W)))
    val core_req_addr   = Input(Vec(numReqs, UInt(wordAddrWidth.W)))
    val core_req_flags  = Input(Vec(numReqs, UInt(flagsWidth.W)))
    val core_req_data   = Input(Vec(numReqs, UInt(wordWidth.W)))
    val core_req_tag    = Input(Vec(numReqs, UInt(tagWidth.W)))
    val core_req_ready  = Output(Vec(numReqs, Bool()))
    // Core response
    val core_rsp_valid  = Output(Vec(numReqs, Bool()))
    val core_rsp_data   = Output(Vec(numReqs, UInt(wordWidth.W)))
    val core_rsp_tag    = Output(Vec(numReqs, UInt(tagWidth.W)))
    val core_rsp_ready  = Input(Vec(numReqs, Bool()))
    // Memory request
    val mem_req_valid   = Output(Vec(memPorts, Bool()))
    val mem_req_rw      = Output(Vec(memPorts, Bool()))
    val mem_req_byteen  = Output(Vec(memPorts, UInt(lineSize.W)))
    val mem_req_addr    = Output(Vec(memPorts, UInt(memAddrWidthCS.W)))
    val mem_req_data    = Output(Vec(memPorts, UInt(lineWidth.W)))
    val mem_req_tag     = Output(Vec(memPorts, UInt(memTagWidth.W)))
    val mem_req_ready   = Input(Vec(memPorts, Bool()))
    // Memory response
    val mem_rsp_valid   = Input(Vec(memPorts, Bool()))
    val mem_rsp_data    = Input(Vec(memPorts, UInt(lineWidth.W)))
    val mem_rsp_tag     = Input(Vec(memPorts, UInt(memTagWidth.W)))
    val mem_rsp_ready   = Output(Vec(memPorts, Bool()))
  })

  val wrap = Module(new CacheWrap(
    p          = p,
    numReqs    = numReqs,
    memPorts   = memPorts,
    crsqSize   = crsqSize,
    mshrSize   = mshrSize,
    mrsqSize   = mrsqSize,
    mreqSize   = mreqSize,
    tagWidth   = tagWidth,
    uuidWidth  = uuidWidth,
    coreOutBuf = coreOutBuf,
    memOutBuf  = memOutBuf
  ))

  // Core request
  for (i <- 0 until numReqs) {
    wrap.io.core_bus(i).req.valid        := io.core_req_valid(i)
    wrap.io.core_bus(i).req.bits.rw      := io.core_req_rw(i)
    wrap.io.core_bus(i).req.bits.byteen  := io.core_req_byteen(i)
    wrap.io.core_bus(i).req.bits.addr    := io.core_req_addr(i)
    wrap.io.core_bus(i).req.bits.flags   := io.core_req_flags(i).asTypeOf(
                                             wrap.io.core_bus(i).req.bits.flags)
    wrap.io.core_bus(i).req.bits.data    := io.core_req_data(i)
    wrap.io.core_bus(i).req.bits.tag     := io.core_req_tag(i).asTypeOf(
                                             wrap.io.core_bus(i).req.bits.tag)
    io.core_req_ready(i)                 := wrap.io.core_bus(i).req.ready
  }
  // Core response
  for (i <- 0 until numReqs) {
    io.core_rsp_valid(i)               := wrap.io.core_bus(i).rsp.valid
    io.core_rsp_data(i)                := wrap.io.core_bus(i).rsp.bits.data
    io.core_rsp_tag(i)                 := wrap.io.core_bus(i).rsp.bits.tag.asUInt
    wrap.io.core_bus(i).rsp.ready      := io.core_rsp_ready(i)
  }
  // Memory request
  for (i <- 0 until memPorts) {
    io.mem_req_valid(i)  := wrap.io.mem_bus(i).req.valid
    io.mem_req_rw(i)     := wrap.io.mem_bus(i).req.bits.rw
    io.mem_req_byteen(i) := wrap.io.mem_bus(i).req.bits.byteen
    io.mem_req_addr(i)   := wrap.io.mem_bus(i).req.bits.addr
    io.mem_req_data(i)   := wrap.io.mem_bus(i).req.bits.data
    io.mem_req_tag(i)    := wrap.io.mem_bus(i).req.bits.tag.asUInt
    wrap.io.mem_bus(i).req.ready := io.mem_req_ready(i)
  }
  // Memory response
  for (i <- 0 until memPorts) {
    wrap.io.mem_bus(i).rsp.valid       := io.mem_rsp_valid(i)
    wrap.io.mem_bus(i).rsp.bits.data   := io.mem_rsp_data(i)
    wrap.io.mem_bus(i).rsp.bits.tag    := io.mem_rsp_tag(i).asTypeOf(
                                          wrap.io.mem_bus(i).rsp.bits.tag)
    io.mem_rsp_ready(i)                := wrap.io.mem_bus(i).rsp.ready
  }
}


// =============================================================================
// CacheCluster
//
// Mirrors VX_cache_cluster.sv.
//
// Instantiates UP(NUM_UNITS) CacheWrap instances, places per-request-type
// arbiters on the core-bus input side, and a per-port arbiter on the memory
// bus output side.  When NUM_UNITS == 0 every cache is in passthru mode.
//
// Parameters:
//   numUnits  – NUM_UNITS (0 = all passthru)
//   numInputs – NUM_INPUTS (total input lanes = numInputs * numReqs)
//   tagSelIdx – TAG_SEL_IDX
//   ... all other params same as CacheWrap
// =============================================================================
class CacheCluster(
  p:          CacheParams,
  numUnits:   Int = 1,
  numInputs:  Int = 1,
  tagSelIdx:  Int = 0,
  numReqs:    Int = 4,
  memPorts:   Int = 1,
  crsqSize:   Int = 4,
  mshrSize:   Int = 16,
  mrsqSize:   Int = 4,
  mreqSize:   Int = 4,
  tagWidth:   Int = 8,
  uuidWidth:  Int = 0,
  ncEnable:   Boolean = false,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val numCaches   = math.max(1, numUnits)
  private val isPassthru  = (numUnits == 0)
  // Tag grows by ARB_SEL_BITS(numInputs, numCaches)
  private val arbSelBits  = if (numInputs > numCaches) log2Ceil((numInputs + numCaches - 1) / numCaches) else 0
  private val arbTagWidth = tagWidth + arbSelBits

  private val lineSize        = p.lineSize
  private val wordSize        = p.wordSize
  private val wordAddrWidth   = p.wordAddrWidth
  private val mshrAddrWidth   = log2Ceil(mshrSize)
  private val bankSelBits     = p.bankSelBits
  private val bankMemTagWidth = uuidWidth + mshrAddrWidth
  // Simplified mem tag width for the cluster (just cache side + bank sel)
  private val cacheMTagWidth  = bankMemTagWidth + bankSelBits
  private val memAddrWidthCS  = p.memAddrWidthCS
  private val flagsWidth      = 3
  private val memTagWidth     = cacheMTagWidth   // passthru and NC not implemented here

  require(numInputs >= numCaches, s"CacheCluster: numInputs ($numInputs) must be >= numCaches ($numCaches)")

  val io = IO(new Bundle {
    // Total inputs: numInputs * numReqs
    val core_bus = Vec(numInputs * numReqs,
                       Flipped(new MemBusBundle(wordSize, wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
    val mem_bus  = Vec(memPorts,
                       new MemBusBundle(lineSize, memAddrWidthCS, flagsWidth, memTagWidth, uuidWidth))
  })

  // ---- Per-request-type arbiters (numInputs → numCaches) per req slot ------
  // arb_core_bus: one bundle per (cache × req_slot)
  val arb_core_bus_valid  = Wire(Vec(numCaches * numReqs, Bool()))
  val arb_core_bus_req    = Wire(Vec(numCaches * numReqs, new MemBusReqBundle(wordSize, wordAddrWidth, flagsWidth, arbTagWidth, uuidWidth)))
  val arb_core_bus_ready  = Wire(Vec(numCaches * numReqs, Bool()))
  val arb_core_bus_rsp_v  = Wire(Vec(numCaches * numReqs, Bool()))
  val arb_core_bus_rsp_d  = Wire(Vec(numCaches * numReqs, new MemBusRspBundle(wordSize, arbTagWidth, uuidWidth)))
  val arb_core_bus_rsp_r  = Wire(Vec(numCaches * numReqs, Bool()))

  for (req <- 0 until numReqs) {
    // Collect input buses for this req slot
    val inp_valid   = Wire(Vec(numInputs, Bool()))
    val inp_req     = Wire(Vec(numInputs, new MemBusReqBundle(wordSize, wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
    val inp_ready   = Wire(Vec(numInputs, Bool()))
    val inp_rsp_v   = Wire(Vec(numInputs, Bool()))
    val inp_rsp_d   = Wire(Vec(numInputs, new MemBusRspBundle(wordSize, tagWidth, uuidWidth)))
    val inp_rsp_r   = Wire(Vec(numInputs, Bool()))

    for (j <- 0 until numInputs) {
      inp_valid(j)  := io.core_bus(j * numReqs + req).req.valid
      inp_req(j)    := io.core_bus(j * numReqs + req).req.bits
      io.core_bus(j * numReqs + req).req.ready := inp_ready(j)
      io.core_bus(j * numReqs + req).rsp.valid := inp_rsp_v(j)
      io.core_bus(j * numReqs + req).rsp.bits  := inp_rsp_d(j)
      inp_rsp_r(j) := io.core_bus(j * numReqs + req).rsp.ready
    }

    // Round-robin arbiter: numInputs → numCaches
    val arb_rr = RegInit(VecInit.fill(numCaches)(0.U(log2Ceil(numInputs).W)))

    // Output arbitration-tag buses (arbTagWidth = tagWidth + arbSelBits)
    val out_valid = Wire(Vec(numCaches, Bool()))
    val out_req   = Wire(Vec(numCaches, new MemBusReqBundle(wordSize, wordAddrWidth, flagsWidth, arbTagWidth, uuidWidth)))
    val out_ready = Wire(Vec(numCaches, Bool()))
    val out_rsp_v = Wire(Vec(numCaches, Bool()))
    val out_rsp_d = Wire(Vec(numCaches, new MemBusRspBundle(wordSize, arbTagWidth, uuidWidth)))
    val out_rsp_r = Wire(Vec(numCaches, Bool()))

    for (k <- 0 until numCaches) {
      val base   = arb_rr(k)
      val cands  = (0 until numInputs).map(j => inp_valid(j))
      val winner = MuxCase(0.U, cands.zipWithIndex.map { case (c, j) => c -> j.U })
      val w_val  = cands.reduce(_ || _)

      out_valid(k) := w_val
      // Append arbiter-select bits to tag
      out_req(k).rw      := inp_req(winner).rw
      out_req(k).addr    := inp_req(winner).addr
      out_req(k).data    := inp_req(winner).data
      out_req(k).byteen  := inp_req(winner).byteen
      out_req(k).flags   := inp_req(winner).flags
      // Embed winner index into tag
      if (arbSelBits > 0) {
        val raw_tag = inp_req(winner).tag.asUInt
        out_req(k).tag   := Cat(raw_tag(tagWidth - 1, tagSelIdx + arbSelBits),
                                winner(arbSelBits - 1, 0),
                                raw_tag(tagSelIdx - 1, 0)).asTypeOf(out_req(k).tag)
      } else {
        out_req(k).tag   := inp_req(winner).tag.asTypeOf(out_req(k).tag)
      }

      // Back-pressure
      for (j <- 0 until numInputs) {
        inp_ready(j) := out_ready(k) && (winner === j.U) && w_val
      }
      when (w_val && out_ready(k)) {
        arb_rr(k) := (winner + 1.U) % numInputs.U
      }

      // Response: extract original requester from tag
      val rsp_tag_raw = out_rsp_d(k).tag.asUInt
      val rsp_src     = if (arbSelBits > 0) rsp_tag_raw(tagSelIdx + arbSelBits - 1, tagSelIdx) else 0.U
      val rsp_tag_orig = if (arbSelBits > 0)
        Cat(rsp_tag_raw(arbTagWidth - 1, tagSelIdx + arbSelBits), rsp_tag_raw(tagSelIdx - 1, 0))
      else rsp_tag_raw

      for (j <- 0 until numInputs) {
        inp_rsp_v(j) := out_rsp_v(k) && (rsp_src === j.U)
        inp_rsp_d(j).data := out_rsp_d(k).data
        inp_rsp_d(j).tag  := rsp_tag_orig(tagWidth - 1, 0).asTypeOf(inp_rsp_d(j).tag)
      }
      out_rsp_r(k) := inp_rsp_r(MuxCase(0.U, (0 until numInputs).map(j =>
        (out_rsp_v(k) && (rsp_src === j.U)) -> j.U)))

      // Connect to arb_core_bus
      arb_core_bus_valid(k * numReqs + req) := out_valid(k)
      arb_core_bus_req(k * numReqs + req)   := out_req(k)
      out_ready(k)  := arb_core_bus_ready(k * numReqs + req)
      out_rsp_v(k)  := arb_core_bus_rsp_v(k * numReqs + req)
      out_rsp_d(k)  := arb_core_bus_rsp_d(k * numReqs + req)
      arb_core_bus_rsp_r(k * numReqs + req) := out_rsp_r(k)
    }
    // Default inp_ready for slots not served
    for (j <- 0 until numInputs) {
      when (!VecInit((0 until numCaches).map(k =>
        out_valid(k) && (arb_rr(k) === j.U)
      )).asUInt.orR) {
        inp_ready(j) := false.B
      }
    }
  }

  // ---- Cache wrap instantiation --------------------------------------------
  val cache_mem_valid = Wire(Vec(numCaches * memPorts, Bool()))
  val cache_mem_req   = Wire(Vec(numCaches * memPorts, new MemBusReqBundle(lineSize, memAddrWidthCS, flagsWidth, memTagWidth, uuidWidth)))
  val cache_mem_ready = Wire(Vec(numCaches * memPorts, Bool()))
  val cache_mem_rsp_v = Wire(Vec(numCaches * memPorts, Bool()))
  val cache_mem_rsp_d = Wire(Vec(numCaches * memPorts, new MemBusRspBundle(lineSize, memTagWidth, uuidWidth)))
  val cache_mem_rsp_r = Wire(Vec(numCaches * memPorts, Bool()))

  for (k <- 0 until numCaches) {
    val cw = Module(new CacheWrap(
      p          = p,
      numReqs    = numReqs,
      memPorts   = memPorts,
      tagSelIdx  = tagSelIdx,
      ncEnable   = ncEnable,
      passthru   = isPassthru,
      crsqSize   = crsqSize,
      mshrSize   = mshrSize,
      mrsqSize   = mrsqSize,
      mreqSize   = mreqSize,
      tagWidth   = arbTagWidth,
      uuidWidth  = uuidWidth,
      coreOutBuf = if (numInputs != numCaches) 2 else coreOutBuf,
      memOutBuf  = if (numCaches > 1) 2 else memOutBuf
    ))

    for (req <- 0 until numReqs) {
      cw.io.core_bus(req).req.valid := arb_core_bus_valid(k * numReqs + req)
      cw.io.core_bus(req).req.bits  := arb_core_bus_req(k * numReqs + req)
      arb_core_bus_ready(k * numReqs + req) := cw.io.core_bus(req).req.ready
      arb_core_bus_rsp_v(k * numReqs + req) := cw.io.core_bus(req).rsp.valid
      arb_core_bus_rsp_d(k * numReqs + req) := cw.io.core_bus(req).rsp.bits
                                                .asTypeOf(arb_core_bus_rsp_d(k * numReqs + req))
      cw.io.core_bus(req).rsp.ready := arb_core_bus_rsp_r(k * numReqs + req)
    }

    for (mp <- 0 until memPorts) {
      cache_mem_valid(k * memPorts + mp) := cw.io.mem_bus(mp).req.valid
      cache_mem_req(k * memPorts + mp)   := cw.io.mem_bus(mp).req.bits
                                             .asTypeOf(cache_mem_req(k * memPorts + mp))
      cw.io.mem_bus(mp).req.ready := cache_mem_ready(k * memPorts + mp)
      cw.io.mem_bus(mp).rsp.valid := cache_mem_rsp_v(k * memPorts + mp)
      cw.io.mem_bus(mp).rsp.bits  := cache_mem_rsp_d(k * memPorts + mp)
                                      .asTypeOf(cw.io.mem_bus(mp).rsp.bits)
      cache_mem_rsp_r(k * memPorts + mp) := cw.io.mem_bus(mp).rsp.ready
    }
  }

  // ---- Memory bus arbitration (numCaches → memPorts) -----------------------
  for (mp <- 0 until memPorts) {
    val arb_rr = RegInit(VecInit.fill(1)(0.U(log2Ceil(numCaches).W)))

    val cands  = (0 until numCaches).map(k => cache_mem_valid(k * memPorts + mp))
    val winner = MuxCase(0.U, cands.zipWithIndex.map { case (c, k) => c -> k.U })
    val w_val  = cands.reduce(_ || _)

    val mem_buf = Module(new Queue(new Bundle {
      val rw     = Bool()
      val byteen = UInt(lineSize.W)
      val addr   = UInt(memAddrWidthCS.W)
      val data   = UInt(p.lineWidth.W)
      val tag    = UInt(memTagWidth.W)
      val flags  = UInt(flagsWidth.W)
    }, math.max(1, if (numCaches > 1) memOutBuf else 1), pipe = true))

    val selIdx = winner * memPorts.U + mp.U
    mem_buf.io.enq.valid       := w_val
    mem_buf.io.enq.bits.rw     := MuxLookup(selIdx, false.B)(
                                     (0 until numCaches * memPorts).map(k => k.U -> cache_mem_req(k).rw))
    mem_buf.io.enq.bits.byteen := MuxLookup(selIdx, 0.U)(
                                     (0 until numCaches * memPorts).map(k => k.U -> cache_mem_req(k).byteen))
    mem_buf.io.enq.bits.addr   := MuxLookup(selIdx, 0.U)(
                                     (0 until numCaches * memPorts).map(k => k.U -> cache_mem_req(k).addr))
    mem_buf.io.enq.bits.data   := MuxLookup(selIdx, 0.U)(
                                     (0 until numCaches * memPorts).map(k => k.U -> cache_mem_req(k).data))
    mem_buf.io.enq.bits.tag    := MuxLookup(selIdx, 0.U)(
                                     (0 until numCaches * memPorts).map(k => k.U -> cache_mem_req(k).tag.asUInt(memTagWidth - 1, 0)))
    mem_buf.io.enq.bits.flags  := MuxLookup(selIdx, 0.U)(
                                     (0 until numCaches * memPorts).map(k => k.U -> cache_mem_req(k).flags.asUInt(flagsWidth - 1, 0)))

    io.mem_bus(mp).req.valid       := mem_buf.io.deq.valid
    io.mem_bus(mp).req.bits.rw     := mem_buf.io.deq.bits.rw
    io.mem_bus(mp).req.bits.byteen := mem_buf.io.deq.bits.byteen
    io.mem_bus(mp).req.bits.addr   := mem_buf.io.deq.bits.addr
    io.mem_bus(mp).req.bits.data   := mem_buf.io.deq.bits.data
    io.mem_bus(mp).req.bits.tag    := mem_buf.io.deq.bits.tag.asTypeOf(io.mem_bus(mp).req.bits.tag)
    io.mem_bus(mp).req.bits.flags  := mem_buf.io.deq.bits.flags.asTypeOf(io.mem_bus(mp).req.bits.flags)
    mem_buf.io.deq.ready           := io.mem_bus(mp).req.ready

    for (k <- 0 until numCaches) {
      cache_mem_ready(k * memPorts + mp) := mem_buf.io.enq.ready && (winner === k.U) && w_val
    }

    when (w_val && mem_buf.io.enq.ready) {
      arb_rr(0) := (winner + 1.U) % numCaches.U
    }

    // Response: broadcast to all caches; cache selects by tag routing
    for (k <- 0 until numCaches) {
      cache_mem_rsp_v(k * memPorts + mp) := io.mem_bus(mp).rsp.valid
      cache_mem_rsp_d(k * memPorts + mp) := io.mem_bus(mp).rsp.bits
                                             .asTypeOf(cache_mem_rsp_d(k * memPorts + mp))
    }
    io.mem_bus(mp).rsp.ready := VecInit((0 until numCaches).map(k =>
      cache_mem_rsp_r(k * memPorts + mp)
    )).asUInt.orR || !io.mem_bus(mp).rsp.valid
  }
}
