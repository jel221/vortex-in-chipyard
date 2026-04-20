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
// Parameters:
//   p            – CacheParams
//   numReqs      – NUM_REQS
//   memPorts     – MEM_PORTS
//   tagSelIdx    – TAG_SEL_IDX
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
  crsqSize:   Int = 4,
  mshrSize:   Int = 16,
  mrsqSize:   Int = 4,
  mreqSize:   Int = 4,
  tagWidth:   Int = 8,
  uuidWidth:  Int = 1,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val lineSize       = p.lineSize
  private val wordSize       = p.wordSize
  private val memAddrWidthCS = p.memAddrWidthCS
  private val flagsWidth     = 3
  private val mshrAddrWidth  = math.max(1, log2Ceil(mshrSize))
  private val bankSelBits    = p.bankSelBits
  // SV: CACHE_MEM_TAG_WIDTH = UUID_WIDTH + CLOG2(MSHR_SIZE) + CLOG2(NUM_BANKS)
  private val cacheMTagWidth = uuidWidth + mshrAddrWidth + bankSelBits
  private val memTagWidth    = cacheMTagWidth

  val io = IO(new Bundle {
    val core_bus = Vec(numReqs, Flipped(new MemBusBundle(wordSize, p.wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
    val mem_bus  = Vec(memPorts, new MemBusBundle(lineSize, memAddrWidthCS, flagsWidth, memTagWidth, uuidWidth))
  })

  // Direct connections: core_bus → cache → mem_bus (no bypass/NC path)
  val core_bus_cache_valid  = Wire(Vec(numReqs, Bool()))
  val core_bus_cache_rdata  = Wire(Vec(numReqs, new MemBusReqBundle(wordSize, p.wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
  val core_bus_cache_ready  = Wire(Vec(numReqs, Bool()))
  val core_bus_cache_rsp_v  = Wire(Vec(numReqs, Bool()))
  val core_bus_cache_rsp_d  = Wire(Vec(numReqs, new MemBusRspBundle(wordSize, tagWidth, uuidWidth)))
  val core_bus_cache_rsp_r  = Wire(Vec(numReqs, Bool()))

  val mem_bus_cache_valid   = Wire(Vec(memPorts, Bool()))
  val mem_bus_cache_req     = Wire(Vec(memPorts, new MemBusReqBundle(lineSize, memAddrWidthCS, flagsWidth, cacheMTagWidth, uuidWidth)))
  val mem_bus_cache_ready   = Wire(Vec(memPorts, Bool()))
  val mem_bus_cache_rsp_v   = Wire(Vec(memPorts, Bool()))
  val mem_bus_cache_rsp_d   = Wire(Vec(memPorts, new MemBusRspBundle(lineSize, cacheMTagWidth, uuidWidth)))
  val mem_bus_cache_rsp_r   = Wire(Vec(memPorts, Bool()))

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
    mem_bus_cache_rsp_d(i) := io.mem_bus(i).rsp.bits.asTypeOf(mem_bus_cache_rsp_d(i))
    io.mem_bus(i).rsp.ready := mem_bus_cache_rsp_r(i)
  }

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
    coreOutBuf = coreOutBuf,
    memOutBuf  = memOutBuf
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
    mem_bus_cache_req(i)   := cache.io.mem_bus(i).req.bits.asTypeOf(mem_bus_cache_req(i))
    cache.io.mem_bus(i).req.ready := mem_bus_cache_ready(i)
    cache.io.mem_bus(i).rsp.valid := mem_bus_cache_rsp_v(i)
    cache.io.mem_bus(i).rsp.bits  := mem_bus_cache_rsp_d(i).asTypeOf(cache.io.mem_bus(i).rsp.bits)
    mem_bus_cache_rsp_r(i) := cache.io.mem_bus(i).rsp.ready
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
  uuidWidth:  Int = 1,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val numCaches   = math.max(1, numUnits)
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
  private val memTagWidth     = cacheMTagWidth 

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
