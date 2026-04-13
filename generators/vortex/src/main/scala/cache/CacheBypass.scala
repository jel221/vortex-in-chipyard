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

// Chisel translation of VX_cache_bypass.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * CacheBypass – non-cacheable / IO bypass path.
 *
 * Mirrors VX_cache_bypass.sv.
 *
 * When CACHE_ENABLE is true, requests flagged as IO (flags[MEM_REQ_FLAG_IO])
 * are routed around the cache and sent directly to memory at word granularity.
 * Requests to cacheable addresses go to the cache.
 *
 * When CACHE_ENABLE is false every request bypasses the cache (passthrough).
 *
 * The implementation connects the structural parts (switches, arbiters) that
 * the SV version instantiates from the Vortex library.  Those library modules
 * are modelled here as straightforward Chisel logic faithful to their
 * documented behaviour.
 *
 * Parameters mirror the SV module parameters.
 *   numReqs       – NUM_REQS
 *   memPorts      – MEM_PORTS
 *   tagSelIdx     – TAG_SEL_IDX (position in tag where arbitration bits are inserted)
 *   cacheEnable   – CACHE_ENABLE
 *   wordSize      – WORD_SIZE (bytes)
 *   lineSize      – LINE_SIZE (bytes)
 *   coreAddrWidth – CORE_ADDR_WIDTH (CS_WORD_ADDR_WIDTH)
 *   coreTagWidth  – CORE_TAG_WIDTH
 *   memAddrWidth  – MEM_ADDR_WIDTH (CS_MEM_ADDR_WIDTH)
 *   memTagInWidth – MEM_TAG_IN_WIDTH (CACHE_MEM_TAG_WIDTH)
 *   uuidWidth     – UUID sub-field width
 */
class CacheBypass(
  numReqs:       Int,
  memPorts:      Int,
  tagSelIdx:     Int  = 0,
  cacheEnable:   Boolean = false,
  wordSize:      Int  = 4,
  lineSize:      Int  = 64,
  coreAddrWidth: Int  = 28,
  coreTagWidth:  Int  = 8,
  memAddrWidth:  Int  = 26,
  memTagInWidth: Int  = 8,
  uuidWidth:     Int  = 0,
  flagsWidth:    Int  = 3
) extends Module {

  private val wordsPerLine  = lineSize / wordSize
  private val wselBits      = log2Ceil(wordsPerLine)
  private val coreDataWidth = wordSize * 8

  // NC tag widths (match SV computation)
  private val coreTagIdWidth = coreTagWidth - uuidWidth
  private val arbSelBits     = log2Ceil(math.max(1, (numReqs + memPorts - 1) / memPorts))
  private val memTagIdWidth  = arbSelBits + coreTagIdWidth
  private val memTagNc1Width = uuidWidth + memTagIdWidth
  private val memTagNc2Width = memTagNc1Width + wselBits
  private val memTagOutWidth = if (cacheEnable) math.max(memTagInWidth, memTagNc2Width)
                               else memTagNc2Width

  val io = IO(new Bundle {
    // Core side
    val core_bus_in  = Vec(numReqs, Flipped(new MemBusBundle(wordSize, coreAddrWidth, flagsWidth, coreTagWidth, uuidWidth)))
    val core_bus_out = Vec(numReqs, new MemBusBundle(wordSize, coreAddrWidth, flagsWidth, coreTagWidth, uuidWidth))
    // Memory side
    val mem_bus_in   = Vec(memPorts, Flipped(new MemBusBundle(lineSize, memAddrWidth, flagsWidth, memTagInWidth, uuidWidth)))
    val mem_bus_out  = Vec(memPorts, new MemBusBundle(lineSize, memAddrWidth, flagsWidth, memTagOutWidth, uuidWidth))
  })

  // ---- split cacheable vs non-cacheable core requests ----------------------
  // Non-cacheable (NC) selector: for CACHE_ENABLE=true select on ~IO flag (bit 2 = MEM_REQ_FLAG_IO).
  // MEM_REQ_FLAG_IO is bit index 2 in the SV (flags[2]).
  // In bypass, NC requests are those without the IO flag (cacheable bit), i.e., they bypass the cache.
  // Here we replicate the SV logic: core_req_nc_sel[i] = ~flags[MEM_REQ_FLAG_IO] when CACHE_ENABLE.

  // For simplicity: when cacheEnable, split requests; otherwise all go NC.
  // core_bus_nc_if carries the non-cacheable (bypass) requests.
  // core_bus_out   carries the cacheable requests (pass-through to cache).

  val nc_sel = Wire(Vec(numReqs, Bool()))
  for (i <- 0 until numReqs) {
    // MEM_REQ_FLAG_IO = bit 2; when this flag is set, request is IO/non-cacheable → nc_sel=1 (goes to bypass)
    // SV: nc_sel = ~flags[MEM_REQ_FLAG_IO] when CACHE_ENABLE (so cacheable reqs go to cache, non-cacheable bypass)
    if (cacheEnable) {
      nc_sel(i) := !io.core_bus_in(i).req.bits.flags(2)  // non-IO → cacheable → goes to core_bus_out
    } else {
      nc_sel(i) := false.B  // all bypass
    }
  }

  // Create temporary wires for NC path (non-cacheable / bypass requests)
  val core_nc_req_valid  = Wire(Vec(numReqs, Bool()))
  val core_nc_req_bits   = Wire(Vec(numReqs, new MemBusReqBundle(wordSize, coreAddrWidth, flagsWidth, coreTagWidth, uuidWidth)))
  val core_nc_req_ready  = WireDefault(VecInit.fill(numReqs)(false.B))
  val core_nc_rsp_valid  = Wire(Vec(numReqs, Bool()))
  val core_nc_rsp_bits   = Wire(Vec(numReqs, new MemBusRspBundle(wordSize, coreTagWidth, uuidWidth)))
  val core_nc_rsp_ready  = Wire(Vec(numReqs, Bool()))

  for (i <- 0 until numReqs) {
    if (cacheEnable) {
      // Route non-cacheable (NC) requests: nc_sel false → NC path; true → cache
      // NC path: !nc_sel (i.e., IO requests bypass the cache)
      // Actually invert: nc_sel = cacheable → cache; !nc_sel = IO → bypass
      val is_nc = !nc_sel(i)  // IO requests go to bypass
      core_nc_req_valid(i) := io.core_bus_in(i).req.valid && is_nc
      core_nc_req_bits(i)  := io.core_bus_in(i).req.bits
      // Cacheable path goes to core_bus_out
      io.core_bus_out(i).req.valid := io.core_bus_in(i).req.valid && nc_sel(i)
      io.core_bus_out(i).req.bits  := io.core_bus_in(i).req.bits
      io.core_bus_in(i).req.ready  := Mux(is_nc, core_nc_req_ready(i), io.core_bus_out(i).req.ready)
      // Response mux
      io.core_bus_in(i).rsp.valid := Mux(is_nc, core_nc_rsp_valid(i), io.core_bus_out(i).rsp.valid)
      io.core_bus_in(i).rsp.bits  := Mux(is_nc, core_nc_rsp_bits(i),  io.core_bus_out(i).rsp.bits)
      core_nc_rsp_ready(i)        := io.core_bus_in(i).rsp.ready && is_nc
      io.core_bus_out(i).rsp.ready := io.core_bus_in(i).rsp.ready && nc_sel(i)
    } else {
      // All traffic bypasses cache
      core_nc_req_valid(i)   := io.core_bus_in(i).req.valid
      core_nc_req_bits(i)    := io.core_bus_in(i).req.bits
      io.core_bus_in(i).req.ready  := core_nc_req_ready(i)
      io.core_bus_in(i).rsp.valid  := core_nc_rsp_valid(i)
      io.core_bus_in(i).rsp.bits   := core_nc_rsp_bits(i)
      core_nc_rsp_ready(i)         := io.core_bus_in(i).rsp.ready
      // core_bus_out is unused
      io.core_bus_out(i).req.valid := false.B
      io.core_bus_out(i).req.bits  := DontCare
      io.core_bus_out(i).rsp.ready := false.B
    }
  }

  // ---- simple round-robin arbitration of NC requests to memPorts -----------
  // We model VX_mem_arb (N-to-M) as a set of round-robin arbiters.
  // For each mem port, select one NC request; append sel bits to the tag.

  val arb_grant  = Wire(Vec(memPorts, UInt(log2Ceil(numReqs).W)))
  val arb_valid  = Wire(Vec(memPorts, Bool()))

  // Round-robin per-port selection
  val rr_ptr     = RegInit(VecInit.fill(memPorts)(0.U(log2Ceil(numReqs).W)))

  for (p <- 0 until memPorts) {
    // Find which NC request wins this port
    val base   = rr_ptr(p)
    val prio   = (0 until numReqs).map { i => ((base + i.U) % numReqs.U).asUInt }
    val grants = prio.map { i => core_nc_req_valid(i) }
    val winner = MuxCase(0.U, grants.zipWithIndex.map { case (g, i) => g -> prio(i) })
    arb_valid(p)   := grants.reduce(_ || _)
    arb_grant(p)   := winner
  }

  // ---- expand word-level NC request to line-level memory request -----------
  for (p <- 0 until memPorts) {
    val req_valid = arb_valid(p)
    val req_bits  = core_nc_req_bits(arb_grant(p))

    // Build memory-width request
    val mem_req_valid_w = req_valid
    val wsel     = if (wordsPerLine > 1) req_bits.addr(wselBits - 1, 0) else 0.U

    val byteen_w = Wire(Vec(wordsPerLine, UInt(wordSize.W)))
    val data_w   = Wire(Vec(wordsPerLine, UInt(coreDataWidth.W)))
    byteen_w := VecInit(Seq.fill(wordsPerLine)(0.U(wordSize.W)))
    data_w   := VecInit(Seq.fill(wordsPerLine)(0.U(coreDataWidth.W)))
    if (wordsPerLine > 1) {
      byteen_w(wsel) := req_bits.byteen
      data_w(wsel)   := req_bits.data
    } else {
      byteen_w(0) := req_bits.byteen
      data_w(0)   := req_bits.data
    }

    val mem_addr_w = if (wordsPerLine > 1) req_bits.addr(coreAddrWidth - 1, wselBits) else req_bits.addr
    // Insert wsel into tag at tagSelIdx position, then prepend UUID
    // Simplified: pack UUID ++ tag_value ++ wsel
    val raw_tag = req_bits.tag.asUInt
    // wordsPerLine > 1: insert wsel at tagSelIdx (adds wselBits to the tag)
    // wordsPerLine == 1 with arb bits needed: append arb_grant at bottom (adds arbSelBits)
    // wordsPerLine == 1, no arb bits: tag width is already memTagNc2Width
    val tag_w   = if (wordsPerLine > 1)
                    Cat(raw_tag(coreTagWidth-1, tagSelIdx), wsel, raw_tag(tagSelIdx-1, 0))
                  else if (arbSelBits > 0)
                    Cat(raw_tag, arb_grant(p)(arbSelBits - 1, 0))
                  else
                    raw_tag

    io.mem_bus_out(p).req.valid        := mem_req_valid_w
    io.mem_bus_out(p).req.bits.rw      := req_bits.rw
    io.mem_bus_out(p).req.bits.addr    := mem_addr_w.asUInt(memAddrWidth - 1, 0)
    io.mem_bus_out(p).req.bits.data    := data_w.asUInt
    io.mem_bus_out(p).req.bits.byteen  := byteen_w.asUInt
    io.mem_bus_out(p).req.bits.flags   := req_bits.flags
    io.mem_bus_out(p).req.bits.tag.asUInt  // assigned below

    // Tag: zero-extend or truncate to memTagOutWidth
    val tag_out = Wire(UInt(memTagOutWidth.W))
    if (cacheEnable) {
      // NC tag is in lower memTagNc2Width bits; cache tag is in upper bits; OR them
      tag_out := tag_w.asUInt(memTagOutWidth - 1, 0)
    } else {
      tag_out := tag_w.asUInt(memTagOutWidth - 1, 0)
    }
    // mem_bus_out tag is a flat UInt embedded in the MemBusTagBundle
    io.mem_bus_out(p).req.bits.tag := tag_out.asTypeOf(io.mem_bus_out(p).req.bits.tag)

    // Back-pressure to the winning NC requester (default is false from WireDefault)
    core_nc_req_ready(arb_grant(p)) := io.mem_bus_out(p).req.ready && arb_valid(p)

    // ---- memory response → core response ------------------------------------
    val rsp_valid  = io.mem_bus_out(p).rsp.valid
    val rsp_tag    = io.mem_bus_out(p).rsp.bits.tag.asUInt
    val rsp_data   = io.mem_bus_out(p).rsp.bits.data

    // Extract wsel from tag and recover original word data
    val rsp_wsel   = if (wordsPerLine > 1) rsp_tag(tagSelIdx + wselBits - 1, tagSelIdx) else 0.U
    val rsp_tag_orig = if (wordsPerLine > 1)
      Cat(rsp_tag(memTagNc2Width-1, tagSelIdx+wselBits), rsp_tag(tagSelIdx-1, 0))
    else rsp_tag(memTagNc1Width-1, 0)

    val rsp_word   = if (wordsPerLine > 1)
      rsp_data.asTypeOf(Vec(wordsPerLine, UInt(coreDataWidth.W)))(rsp_wsel)
    else rsp_data(coreDataWidth - 1, 0)

    // Route to original requester: use low bits of tag_orig to identify request index
    val rsp_req_id = if (numReqs > 1) rsp_tag_orig(log2Ceil(numReqs) - 1, 0) else 0.U

    for (r <- 0 until numReqs) {
      core_nc_rsp_valid(r) := rsp_valid && (rsp_req_id === r.U)
      core_nc_rsp_bits(r).data := rsp_word
      core_nc_rsp_bits(r).tag  := rsp_tag_orig.asTypeOf(core_nc_rsp_bits(r).tag)
    }
    io.mem_bus_out(p).rsp.ready := VecInit((0 until numReqs).map(r =>
      core_nc_rsp_valid(r) && core_nc_rsp_ready(r)
    )).asUInt.orR || !rsp_valid

    // If cacheEnable, merge with cache mem bus
    if (cacheEnable) {
      // mem_bus_in carries the cache memory requests; arbitrate with NC
      io.mem_bus_in(p).req.ready := false.B  // handled in the arbiter below
      // The SV uses VX_mem_arb to merge; here we give cache priority
      when (!req_valid && io.mem_bus_in(p).req.valid) {
        io.mem_bus_out(p).req.valid       := io.mem_bus_in(p).req.valid
        io.mem_bus_out(p).req.bits.rw     := io.mem_bus_in(p).req.bits.rw
        io.mem_bus_out(p).req.bits.addr   := io.mem_bus_in(p).req.bits.addr
        io.mem_bus_out(p).req.bits.data   := io.mem_bus_in(p).req.bits.data
        io.mem_bus_out(p).req.bits.byteen := io.mem_bus_in(p).req.bits.byteen
        io.mem_bus_out(p).req.bits.flags  := io.mem_bus_in(p).req.bits.flags
        io.mem_bus_out(p).req.bits.tag    := io.mem_bus_in(p).req.bits.tag.asUInt
                                              .asTypeOf(io.mem_bus_out(p).req.bits.tag)
        io.mem_bus_in(p).req.ready        := io.mem_bus_out(p).req.ready
      } .otherwise {
        io.mem_bus_in(p).req.ready := false.B
      }
      // Response: tag MSB indicates NC vs cache path.
      // The memory-side tag is {uuid, tag_id} (memTagOutWidth bits); the MSB
      // distinguishes cache (0) from NC (1) responses when both share a port.
      val mem_rsp_tag = io.mem_bus_out(p).rsp.bits.tag.asUInt
      io.mem_bus_in(p).rsp.valid := io.mem_bus_out(p).rsp.valid && (mem_rsp_tag(memTagOutWidth-1) === 0.U)
      io.mem_bus_in(p).rsp.bits  := io.mem_bus_out(p).rsp.bits.asTypeOf(io.mem_bus_in(p).rsp.bits)
    } else {
      io.mem_bus_in(p).req.ready := false.B
      io.mem_bus_in(p).rsp.valid := false.B
      io.mem_bus_in(p).rsp.bits  := DontCare
    }
  }
}
