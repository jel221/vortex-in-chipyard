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

// Chisel translation of VX_cache.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * Cache – top-level multi-bank cache (without bypass/NC logic).
 *
 * Mirrors VX_cache.sv.
 *
 * Architecture:
 *   - VX_cache_init  : flush sequencer at the core-bus input
 *   - NUM_BANKS × VX_cache_bank : the actual cache banks
 *   - VX_stream_xbar (modelled): core-req dispatch to banks, core-rsp gather
 *   - VX_stream_arb  (modelled): bank mem-req arbitration to MEM_PORTS outputs
 *   - mem-rsp crossbar: demux mem responses back to the correct bank
 *
 * The SV crossbar/arbiter primitives (VX_stream_xbar, VX_stream_arb,
 * VX_stream_omega) are modelled as straightforward Chisel combinational
 * logic; for a cycle-accurate match a proper registered version would be
 * needed but the structural behaviour is faithful.
 *
 * Parameters:
 *   p          – CacheParams struct (contains CACHE_SIZE, LINE_SIZE, …)
 *   numReqs    – NUM_REQS
 *   memPorts   – MEM_PORTS
 *   crsqSize   – CRSQ_SIZE
 *   mshrSize   – MSHR_SIZE
 *   mrsqSize   – MRSQ_SIZE
 *   mreqSize   – MREQ_SIZE
 *   tagWidth   – TAG_WIDTH
 *   uuidWidth  – UUID sub-field width
 *   coreOutBuf – CORE_OUT_BUF
 *   memOutBuf  – MEM_OUT_BUF
 */
class Cache(
  p:          CacheParams,
  numReqs:    Int = 4,
  memPorts:   Int = 1,
  crsqSize:   Int = 4,
  mshrSize:   Int = 16,
  mrsqSize:   Int = 4,
  mreqSize:   Int = 4,
  tagWidth:   Int = 8,
  uuidWidth:  Int = 0,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val numBanks      = p.numBanks
  private val lineSize      = p.lineSize
  private val wordSize      = p.wordSize
  private val wordWidth     = p.wordWidth
  private val lineWidth     = p.lineWidth
  private val lineAddrWidth = p.lineAddrWidth
  private val memAddrWidthCS = p.memAddrWidthCS
  private val wordSelBits   = p.wordSelBits
  private val bankSelBits   = p.bankSelBits
  private val mshrAddrWidth = math.max(1, log2Ceil(mshrSize))
  private val reqSelWidth   = math.max(1, log2Ceil(numReqs))
  private val wordSelWidth  = p.wordSelWidth
  private val bankSelWidth  = math.max(1, bankSelBits)
  private val memTagWidth   = uuidWidth + mshrAddrWidth + (if (numBanks > 1) bankSelBits else 0) +
                              (if (numBanks > memPorts) log2Ceil(numBanks / memPorts) else 0)
  // Simplified: use bank-mem tag width = uuid + mshr_addr + bank_sel_bits
  private val bankMemTagWidth = uuidWidth + mshrAddrWidth
  private val flagsWidth    = 3

  val io = IO(new Bundle {
    // Core bus (slave)
    val core_bus = Vec(numReqs, Flipped(new MemBusBundle(wordSize, p.wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
    // Memory bus (master)
    val mem_bus  = Vec(memPorts, new MemBusBundle(lineSize, memAddrWidthCS, flagsWidth,
                                                   bankMemTagWidth + bankSelBits, uuidWidth))
  })

  // =========================================================================
  // CacheInit – flush sequencer
  // =========================================================================
  val cacheInit = Module(new CacheInit(
    numReqs        = numReqs,
    numBanks       = numBanks,
    tagWidth       = tagWidth,
    uuidWidth      = uuidWidth,
    bankSelLatency = if (numReqs > 2) 2 else 0,
    dataSize       = wordSize,
    addrWidth      = p.wordAddrWidth,
    flagsWidth     = flagsWidth
  ))

  for (i <- 0 until numReqs) {
    cacheInit.io.core_bus_in(i)  <> io.core_bus(i)
  }

  // per_bank_flush wires
  val per_bank_flush_begin = cacheInit.io.flush_begin
  val flush_uuid           = cacheInit.io.flush_uuid
  val per_bank_flush_end   = Wire(UInt(numBanks.W))
  cacheInit.io.flush_end   := per_bank_flush_end

  // =========================================================================
  // Memory response queue per port
  // =========================================================================
  // Queue up memory responses, then demux to the correct bank.
  val mem_rsp_q_valid = Wire(Vec(memPorts, Bool()))
  val mem_rsp_q_data  = Wire(Vec(memPorts, UInt((lineWidth + bankMemTagWidth + bankSelBits).W)))
  val mem_rsp_q_ready = WireDefault(VecInit.fill(memPorts)(false.B))

  for (i <- 0 until memPorts) {
    val rsp_q = Module(new Queue(UInt((lineWidth + bankMemTagWidth + bankSelBits).W), mrsqSize))
    val packed = Cat(io.mem_bus(i).rsp.bits.data,
                     io.mem_bus(i).rsp.bits.tag.asUInt)
    rsp_q.io.enq.valid  := io.mem_bus(i).rsp.valid
    rsp_q.io.enq.bits   := packed
    io.mem_bus(i).rsp.ready := rsp_q.io.enq.ready
    mem_rsp_q_valid(i)  := rsp_q.io.deq.valid
    mem_rsp_q_data(i)   := rsp_q.io.deq.bits
    rsp_q.io.deq.ready  := mem_rsp_q_ready(i)
  }

  // =========================================================================
  // Core request dispatch crossbar (NUM_REQS → NUM_BANKS)
  // Requests are steered by bank-select bits in the word address.
  // =========================================================================
  private val wordsPerLine  = p.wordsPerLine
  private val lineAddrW     = lineAddrWidth

  // Unpack core bus2 (post-init)
  val core_req_valid  = Wire(Vec(numReqs, Bool()))
  val core_req_addr   = Wire(Vec(numReqs, UInt(p.wordAddrWidth.W)))
  val core_req_rw     = Wire(Vec(numReqs, Bool()))
  val core_req_byteen = Wire(Vec(numReqs, UInt(wordSize.W)))
  val core_req_data   = Wire(Vec(numReqs, UInt(wordWidth.W)))
  val core_req_tag    = Wire(Vec(numReqs, UInt(tagWidth.W)))
  val core_req_flags  = Wire(Vec(numReqs, UInt(math.max(1, flagsWidth).W)))
  val core_req_ready  = WireDefault(VecInit.fill(numReqs)(false.B))

  for (i <- 0 until numReqs) {
    core_req_valid(i)  := cacheInit.io.core_bus_out(i).req.valid
    core_req_addr(i)   := cacheInit.io.core_bus_out(i).req.bits.addr
    core_req_rw(i)     := cacheInit.io.core_bus_out(i).req.bits.rw
    core_req_byteen(i) := cacheInit.io.core_bus_out(i).req.bits.byteen
    core_req_data(i)   := cacheInit.io.core_bus_out(i).req.bits.data
    core_req_tag(i)    := cacheInit.io.core_bus_out(i).req.bits.tag.asUInt
    core_req_flags(i)  := cacheInit.io.core_bus_out(i).req.bits.flags.asUInt
    cacheInit.io.core_bus_out(i).req.ready := core_req_ready(i)
  }

  // Extract bank-select bits and word-select bits from word address
  val core_req_bid  = Wire(Vec(numReqs, UInt(bankSelWidth.W)))
  val core_req_wsel = Wire(Vec(numReqs, UInt(wordSelWidth.W)))
  val core_req_laddr = Wire(Vec(numReqs, UInt(lineAddrW.W)))
  for (i <- 0 until numReqs) {
    core_req_wsel(i)  := (if (wordsPerLine > 1) core_req_addr(i)(wordSelBits - 1, 0) else 0.U)
    core_req_bid(i)   := (if (numBanks > 1) core_req_addr(i)(wordSelBits + bankSelBits - 1, wordSelBits) else 0.U)
    core_req_laddr(i) := core_req_addr(i)(p.wordAddrWidth - 1, wordSelBits + bankSelBits)
  }

  // Per-bank request wires
  val pb_core_req_valid  = Wire(Vec(numBanks, Bool()))
  val pb_core_req_addr   = Wire(Vec(numBanks, UInt(lineAddrW.W)))
  val pb_core_req_rw     = Wire(Vec(numBanks, Bool()))
  val pb_core_req_wsel   = Wire(Vec(numBanks, UInt(wordSelWidth.W)))
  val pb_core_req_byteen = Wire(Vec(numBanks, UInt(wordSize.W)))
  val pb_core_req_data   = Wire(Vec(numBanks, UInt(wordWidth.W)))
  val pb_core_req_tag    = Wire(Vec(numBanks, UInt(tagWidth.W)))
  val pb_core_req_idx    = Wire(Vec(numBanks, UInt(reqSelWidth.W)))
  val pb_core_req_flags  = Wire(Vec(numBanks, UInt(math.max(1, flagsWidth).W)))
  val pb_core_req_ready  = Wire(Vec(numBanks, Bool()))

  val pb_core_rsp_valid  = Wire(Vec(numBanks, Bool()))
  val pb_core_rsp_data   = Wire(Vec(numBanks, UInt(wordWidth.W)))
  val pb_core_rsp_tag    = Wire(Vec(numBanks, UInt(tagWidth.W)))
  val pb_core_rsp_idx    = Wire(Vec(numBanks, UInt(reqSelWidth.W)))
  val pb_core_rsp_ready  = Wire(Vec(numBanks, Bool()))

  val pb_mem_req_valid   = Wire(Vec(numBanks, Bool()))
  val pb_mem_req_addr    = Wire(Vec(numBanks, UInt(lineAddrW.W)))
  val pb_mem_req_rw      = Wire(Vec(numBanks, Bool()))
  val pb_mem_req_byteen  = Wire(Vec(numBanks, UInt(lineSize.W)))
  val pb_mem_req_data    = Wire(Vec(numBanks, UInt(lineWidth.W)))
  val pb_mem_req_tag     = Wire(Vec(numBanks, UInt(bankMemTagWidth.W)))
  val pb_mem_req_flags   = Wire(Vec(numBanks, UInt(math.max(1, flagsWidth).W)))
  val pb_mem_req_ready   = Wire(Vec(numBanks, Bool()))

  val pb_mem_rsp_valid   = Wire(Vec(numBanks, Bool()))
  val pb_mem_rsp_data    = Wire(Vec(numBanks, UInt(lineWidth.W)))
  val pb_mem_rsp_tag     = Wire(Vec(numBanks, UInt(bankMemTagWidth.W)))
  val pb_mem_rsp_ready   = Wire(Vec(numBanks, Bool()))

  // Simple round-robin crossbar: numReqs → numBanks
  // Each bank gets at most one request per cycle.
  val xbar_rr = RegInit(VecInit.fill(numBanks)(0.U(log2Ceil(numReqs).W)))
  for (b <- 0 until numBanks) {
    // Find the highest-priority request targeting this bank
    val base = xbar_rr(b)
    val cands = (0 until numReqs).map { i =>
      core_req_valid(i) && (core_req_bid(i) === b.U)
    }
    val winner_idx = MuxCase(0.U, cands.zipWithIndex.map { case (c, i) => c -> i.U })
    val winner_valid = cands.reduce(_ || _)

    pb_core_req_valid(b)  := winner_valid
    pb_core_req_addr(b)   := Mux(winner_valid, core_req_laddr(winner_idx), 0.U)
    pb_core_req_rw(b)     := Mux(winner_valid, core_req_rw(winner_idx), false.B)
    pb_core_req_wsel(b)   := Mux(winner_valid, core_req_wsel(winner_idx), 0.U)
    pb_core_req_byteen(b) := Mux(winner_valid, core_req_byteen(winner_idx), 0.U)
    pb_core_req_data(b)   := Mux(winner_valid, core_req_data(winner_idx), 0.U)
    pb_core_req_tag(b)    := Mux(winner_valid, core_req_tag(winner_idx), 0.U)
    pb_core_req_idx(b)    := Mux(winner_valid, winner_idx, 0.U)
    pb_core_req_flags(b)  := Mux(winner_valid, core_req_flags(winner_idx), 0.U)

    // Advance RR pointer when a request fires
    when (winner_valid && pb_core_req_ready(b)) {
      xbar_rr(b) := (winner_idx + 1.U) % numReqs.U
    }

    // back-pressure
    for (r <- 0 until numReqs) {
      when (winner_idx === r.U && winner_valid) {
        core_req_ready(r) := pb_core_req_ready(b)
      }
    }
  }


  // Track which banks got a request fire (for CacheInit)
  val per_bank_core_req_fire = VecInit((0 until numBanks).map { b =>
    pb_core_req_valid(b) && pb_core_req_ready(b)
  }).asUInt
  cacheInit.io.bank_req_fire := per_bank_core_req_fire

  // =========================================================================
  // Bank instantiation
  // =========================================================================
  val flush_end_vec = Wire(Vec(numBanks, Bool()))
  for (b <- 0 until numBanks) {
    val bank = Module(new CacheBank(
      p          = p,
      bankId     = b,
      numReqs    = numReqs,
      crsqSize   = crsqSize,
      mshrSize   = mshrSize,
      mreqSize   = mreqSize,
      tagWidth   = tagWidth,
      uuidWidth  = uuidWidth,
      coreOutReg = 0,
      memOutReg  = 0
    ))

    bank.io.core_req_valid   := pb_core_req_valid(b)
    bank.io.core_req_addr    := pb_core_req_addr(b)
    bank.io.core_req_rw      := pb_core_req_rw(b)
    bank.io.core_req_wsel    := pb_core_req_wsel(b)
    bank.io.core_req_byteen  := pb_core_req_byteen(b)
    bank.io.core_req_data    := pb_core_req_data(b)
    bank.io.core_req_tag     := pb_core_req_tag(b)
    bank.io.core_req_idx     := pb_core_req_idx(b)
    bank.io.core_req_flags   := pb_core_req_flags(b)
    pb_core_req_ready(b)     := bank.io.core_req_ready

    pb_core_rsp_valid(b) := bank.io.core_rsp_valid
    pb_core_rsp_data(b)  := bank.io.core_rsp_data
    pb_core_rsp_tag(b)   := bank.io.core_rsp_tag
    pb_core_rsp_idx(b)   := bank.io.core_rsp_idx
    bank.io.core_rsp_ready := pb_core_rsp_ready(b)

    pb_mem_req_valid(b) := bank.io.mem_req_valid
    pb_mem_req_addr(b)  := bank.io.mem_req_addr
    pb_mem_req_rw(b)    := bank.io.mem_req_rw
    pb_mem_req_byteen(b) := bank.io.mem_req_byteen
    pb_mem_req_data(b)  := bank.io.mem_req_data
    pb_mem_req_tag(b)   := bank.io.mem_req_tag
    pb_mem_req_flags(b) := bank.io.mem_req_flags
    bank.io.mem_req_ready := pb_mem_req_ready(b)

    bank.io.mem_rsp_valid := pb_mem_rsp_valid(b)
    bank.io.mem_rsp_data  := pb_mem_rsp_data(b)
    bank.io.mem_rsp_tag   := pb_mem_rsp_tag(b)
    pb_mem_rsp_ready(b)  := bank.io.mem_rsp_ready

    bank.io.flush_begin := per_bank_flush_begin(b)
    bank.io.flush_uuid  := flush_uuid
    flush_end_vec(b)    := bank.io.flush_end
  }
  per_bank_flush_end := flush_end_vec.asUInt

  // =========================================================================
  // Core response crossbar: numBanks → numReqs
  // Route each bank response to the original requester (indexed by rsp_idx).
  // =========================================================================
  val rsp_rr = RegInit(VecInit.fill(numReqs)(0.U(log2Ceil(numBanks).W)))
  for (r <- 0 until numReqs) {
    val base = rsp_rr(r)
    val cands = (0 until numBanks).map { b =>
      pb_core_rsp_valid(b) && (pb_core_rsp_idx(b) === r.U)
    }
    val winner_b   = MuxCase(0.U, cands.zipWithIndex.map { case (c, b) => c -> b.U })
    val winner_val = cands.reduce(_ || _)

    // Buffer output
    val rsp_buf = Module(new Queue(new Bundle {
      val data = UInt(wordWidth.W)
      val tag  = UInt(tagWidth.W)
    }, math.max(1, if ((numBanks != 1) || (numReqs != 1)) coreOutBuf else 1),
      pipe = true))

    rsp_buf.io.enq.valid    := winner_val
    rsp_buf.io.enq.bits.data := pb_core_rsp_data(winner_b)
    rsp_buf.io.enq.bits.tag  := pb_core_rsp_tag(winner_b)
    cacheInit.io.core_bus_out(r).rsp.valid     := rsp_buf.io.deq.valid
    cacheInit.io.core_bus_out(r).rsp.bits.data := rsp_buf.io.deq.bits.data
    cacheInit.io.core_bus_out(r).rsp.bits.tag  := rsp_buf.io.deq.bits.tag
                                                    .asTypeOf(cacheInit.io.core_bus_out(r).rsp.bits.tag)
    rsp_buf.io.deq.ready := cacheInit.io.core_bus_out(r).rsp.ready

    for (b <- 0 until numBanks) {
      pb_core_rsp_ready(b) := rsp_buf.io.enq.ready && (winner_b === b.U) && winner_val
    }

    when (winner_val && rsp_buf.io.enq.ready) {
      rsp_rr(r) := (winner_b + 1.U) % numBanks.U
    }
  }

  // =========================================================================
  // Memory request arbitration: numBanks → memPorts
  // Simple round-robin per port; append bank-id bits to tag.
  // =========================================================================
  val mem_req_rr = RegInit(VecInit.fill(memPorts)(0.U(log2Ceil(numBanks).W)))
  for (p_idx <- 0 until memPorts) {
    val base   = mem_req_rr(p_idx)
    val cands  = (0 until numBanks).map { b => pb_mem_req_valid(b) }
    val winner = MuxCase(0.U, cands.zipWithIndex.map { case (c, b) => c -> b.U })
    val w_val  = cands.reduce(_ || _)

    // Build full memory address: Cat(lineAddr, bankId)
    val full_addr  = if (numBanks > 1)
      Cat(pb_mem_req_addr(winner), winner(bankSelBits - 1, 0))
    else pb_mem_req_addr(winner)

    // Tag: Cat(bankTag, bankId selector bits) for response routing
    val full_tag = if (numBanks > 1)
      Cat(pb_mem_req_tag(winner), winner(bankSelBits - 1, 0))
    else pb_mem_req_tag(winner)

    val mem_req_buf = Module(new Queue(new Bundle {
      val rw      = Bool()
      val byteen  = UInt(lineSize.W)
      val addr    = UInt(memAddrWidthCS.W)
      val data    = UInt(lineWidth.W)
      val tag     = UInt((bankMemTagWidth + bankSelBits).W)
      val flags   = UInt(math.max(1, flagsWidth).W)
    }, math.max(1, if (numBanks != 1) memOutBuf else 1), pipe = true))

    mem_req_buf.io.enq.valid        := w_val
    mem_req_buf.io.enq.bits.rw      := pb_mem_req_rw(winner)
    mem_req_buf.io.enq.bits.byteen  := pb_mem_req_byteen(winner)
    mem_req_buf.io.enq.bits.addr    := full_addr
    mem_req_buf.io.enq.bits.data    := pb_mem_req_data(winner)
    mem_req_buf.io.enq.bits.tag     := full_tag
    mem_req_buf.io.enq.bits.flags   := pb_mem_req_flags(winner)

    io.mem_bus(p_idx).req.valid        := mem_req_buf.io.deq.valid
    io.mem_bus(p_idx).req.bits.rw      := mem_req_buf.io.deq.bits.rw
    io.mem_bus(p_idx).req.bits.byteen  := mem_req_buf.io.deq.bits.byteen
    io.mem_bus(p_idx).req.bits.addr    := mem_req_buf.io.deq.bits.addr
    io.mem_bus(p_idx).req.bits.data    := mem_req_buf.io.deq.bits.data
    io.mem_bus(p_idx).req.bits.tag     := mem_req_buf.io.deq.bits.tag
                                          .asTypeOf(io.mem_bus(p_idx).req.bits.tag)
    io.mem_bus(p_idx).req.bits.flags   := mem_req_buf.io.deq.bits.flags.asTypeOf(
                                          io.mem_bus(p_idx).req.bits.flags)
    mem_req_buf.io.deq.ready := io.mem_bus(p_idx).req.ready

    for (b <- 0 until numBanks) {
      pb_mem_req_ready(b) := mem_req_buf.io.enq.ready && (winner === b.U) && w_val
    }

    when (w_val && mem_req_buf.io.enq.ready) {
      mem_req_rr(p_idx) := (winner + 1.U) % numBanks.U
    }
  }

  // =========================================================================
  // Memory response demux: memPorts → numBanks
  // The bank-id is encoded in the low bankSelBits of the tag.
  // =========================================================================
  for (b <- 0 until numBanks) {
    // Find which memory port (if any) has a response for this bank
    val src_port = (0 until memPorts).map { p_idx =>
      val tag = mem_rsp_q_data(p_idx)(bankMemTagWidth + bankSelBits - 1, 0)
      val bid = if (numBanks > 1) tag(bankSelBits - 1, 0) else 0.U
      mem_rsp_q_valid(p_idx) && (bid === b.U)
    }
    val src_valid = src_port.reduce(_ || _)
    val src_idx   = MuxCase(0.U, src_port.zipWithIndex.map { case (v, i) => v -> i.U })

    val rsp_packed  = mem_rsp_q_data(src_idx)
    val rsp_tag_raw = rsp_packed(bankMemTagWidth + bankSelBits - 1, 0)
    val rsp_bank_tag = if (numBanks > 1) rsp_tag_raw >> bankSelBits else rsp_tag_raw
    val rsp_data    = rsp_packed(lineWidth + bankMemTagWidth + bankSelBits - 1,
                                 bankMemTagWidth + bankSelBits)

    pb_mem_rsp_valid(b) := src_valid
    pb_mem_rsp_data(b)  := rsp_data
    pb_mem_rsp_tag(b)   := rsp_bank_tag(bankMemTagWidth - 1, 0)

    mem_rsp_q_ready(src_idx) := pb_mem_rsp_ready(b) && src_valid
  }

}
