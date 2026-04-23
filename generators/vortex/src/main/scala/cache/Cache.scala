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

class CacheMemReqArbBundle(
  val addrW:   Int,
  val dataW:   Int,
  val byteenW: Int,
  val flagsW:  Int,
  val tagW:    Int
) extends Bundle {
  val rw     = Bool()
  val addr   = UInt(addrW.W)
  val data   = UInt(dataW.W)
  val byteen = UInt(byteenW.W)
  val flags  = UInt(flagsW.W)
  val tag    = UInt(tagW.W)
}

class CacheMemReqBufBundle(
  val byteenW: Int,
  val addrW:   Int,
  val dataW:   Int,
  val tagW:    Int,
  val flagsW:  Int
) extends Bundle {
  val rw     = Bool()
  val byteen = UInt(byteenW.W)
  val addr   = UInt(addrW.W)
  val data   = UInt(dataW.W)
  val tag    = UInt(tagW.W)
  val flags  = UInt(flagsW.W)
}

/**
 * Cache – top-level multi-bank cache (without bypass/NC logic).
 *
 * Mirrors VX_cache.sv.
 *
 * Architecture (matching SV):
 *   - VX_cache_init  : flush sequencer at the core-bus input
 *   - NUM_BANKS × VX_cache_bank : the actual cache banks
 *   - VX_stream_xbar (REQ): core-req dispatch to banks (NUM_REQS → NUM_BANKS)
 *   - VX_stream_xbar (RSP): core-rsp gather (NUM_BANKS → NUM_REQS)
 *   - VX_stream_arb         : bank mem-req arbitration (NUM_BANKS → MEM_PORTS)
 *   - VX_stream_omega       : mem-rsp crossbar (MEM_PORTS → NUM_BANKS)
 *
 * SV notes:
 *   per_bank_core_req_fire = per_bank_core_req_valid & per_bank_mem_req_ready
 *   (NOT per_bank_core_req_valid & per_bank_core_req_ready)
 *
 *   REQ_XBAR_BUF = (NUM_REQS > 2) ? 2 : 0
 *   BANK_SEL_LATENCY = TO_OUT_BUF_REG(REQ_XBAR_BUF)
 *   CORE_RSP_BUF_ENABLE = (NUM_BANKS != 1) || (NUM_REQS != 1)
 *   MEM_REQ_BUF_ENABLE  = (NUM_BANKS != 1)
 *
 *   MEM_TAG_WIDTH = CACHE_MEM_TAG_WIDTH(MSHR_SIZE, NUM_BANKS, MEM_PORTS, UUID_WIDTH)
 *                 = UUID_WIDTH + LOG2UP(MSHR_SIZE) + CLOG2(NUM_BANKS/MEM_PORTS) + CLOG2(MEM_PORTS)
 *   BANK_MEM_TAG_WIDTH = UUID_WIDTH + MSHR_ADDR_WIDTH
 *   MEM_ARB_SEL_BITS   = CLOG2(CDIV(NUM_BANKS, MEM_PORTS))
 *   MEM_PORTS_SEL_BITS = CLOG2(MEM_PORTS)
 *
 * The primitives VX_stream_xbar, VX_stream_arb, VX_stream_omega are modelled
 * as straightforward round-robin logic (structurally faithful).
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
  uuidWidth:  Int = 1,
  coreOutBuf: Int = 3,
  memOutBuf:  Int = 3
) extends Module {

  private val numBanks        = p.numBanks
  private val lineSize        = p.lineSize
  private val wordSize        = p.wordSize
  private val wordWidth       = p.wordWidth
  private val lineWidth       = p.lineWidth
  private val lineAddrWidth   = p.lineAddrWidth
  private val memAddrWidthCS  = p.memAddrWidthCS
  private val wordSelBits     = p.wordSelBits
  private val bankSelBits     = p.bankSelBits
  private val wordSelWidth    = p.wordSelWidth
  private val mshrAddrWidth   = math.max(1, log2Ceil(mshrSize))
  private val reqSelBits      = log2Ceil(numReqs)
  private val reqSelWidth     = math.max(1, reqSelBits)
  private val bankSelWidth    = math.max(1, bankSelBits)
  private val flagsWidth      = 3

  // SV: BANK_MEM_TAG_WIDTH = UUID_WIDTH + MSHR_ADDR_WIDTH
  private val bankMemTagWidth = uuidWidth + mshrAddrWidth

  // SV: MEM_ARB_SEL_BITS = CLOG2(CDIV(NUM_BANKS, MEM_PORTS))
  private val memArbSelBits   = log2Ceil(math.max(1, (numBanks + memPorts - 1) / memPorts))
  // SV: MEM_PORTS_SEL_BITS = CLOG2(MEM_PORTS)
  private val memPortsSelBits = log2Ceil(math.max(1, memPorts))
  // SV: MEM_TAG_WIDTH = CACHE_MEM_TAG_WIDTH = UUID_WIDTH + MSHR_ADDR_WIDTH + CLOG2(NUM_BANKS/MEM_PORTS) + CLOG2(MEM_PORTS)
  //                   = BANK_MEM_TAG_WIDTH + BANK_SEL_BITS
  private val memTagWidth     = bankMemTagWidth + bankSelBits

  // SV: REQ_XBAR_BUF = (NUM_REQS > 2) ? 2 : 0
  private val reqXbarBuf      = if (numReqs > 2) 2 else 0
  // SV: BANK_SEL_LATENCY = TO_OUT_BUF_REG(REQ_XBAR_BUF)
  //     TO_OUT_BUF_REG(n) = (n > 1) ? 1 : n  [i.e., 0→0, 1→1, 2→1]
  private val bankSelLatency  = if (reqXbarBuf > 1) 1 else reqXbarBuf
  // SV: CORE_RSP_BUF_ENABLE = (NUM_BANKS != 1) || (NUM_REQS != 1)
  private val coreRspBufEnable = (numBanks != 1) || (numReqs != 1)
  // SV: MEM_REQ_BUF_ENABLE = (NUM_BANKS != 1)
  private val memReqBufEnable  = (numBanks != 1)

  val io = IO(new Bundle {
    val core_bus = Vec(numReqs,  Flipped(new MemBusBundle(wordSize, p.wordAddrWidth, flagsWidth, tagWidth, uuidWidth)))
    val mem_bus  = Vec(memPorts, new MemBusBundle(lineSize, memAddrWidthCS, flagsWidth, memTagWidth, uuidWidth))
  })

  // =========================================================================
  // CacheInit
  // =========================================================================
  val cacheInit = Module(new CacheInit(
    numReqs        = numReqs,
    numBanks       = numBanks,
    tagWidth       = tagWidth,
    uuidWidth      = uuidWidth,
    bankSelLatency = bankSelLatency,
    dataSize       = wordSize,
    addrWidth      = p.wordAddrWidth,
    flagsWidth     = flagsWidth
  ))
  for (i <- 0 until numReqs) {
    cacheInit.io.core_bus_in(i) <> io.core_bus(i)
  }

  val per_bank_flush_begin = cacheInit.io.flush_begin
  val flush_uuid           = cacheInit.io.flush_uuid
  val per_bank_flush_end   = Wire(UInt(numBanks.W))
  cacheInit.io.flush_end   := per_bank_flush_end

  // =========================================================================
  // Extract fields from core bus (post-init)
  // =========================================================================
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
    // Response from bank crossbar to core bus (filled below)
    cacheInit.io.core_bus_out(i).rsp.valid := false.B
    cacheInit.io.core_bus_out(i).rsp.bits  := DontCare
  }

  // SV: address decomposition
  //   core_req_wsel[i]       = WORDS_PER_LINE > 1 ? addr[WORD_SEL_BITS-1:0] : '0
  //   core_req_bid[i]        = NUM_BANKS > 1 ? addr[WORD_SEL_BITS +: BANK_SEL_BITS] : '0
  //   core_req_line_addr[i]  = addr[(BANK_SEL_BITS + WORD_SEL_BITS) +: LINE_ADDR_WIDTH]
  val core_req_wsel      = Wire(Vec(numReqs, UInt(wordSelWidth.W)))
  val core_req_bid       = Wire(Vec(numReqs, UInt(bankSelWidth.W)))
  val core_req_line_addr = Wire(Vec(numReqs, UInt(lineAddrWidth.W)))
  for (i <- 0 until numReqs) {
    core_req_wsel(i) := (if (p.wordsPerLine > 1) core_req_addr(i)(wordSelBits - 1, 0) else 0.U)
    core_req_bid(i)  := (if (numBanks > 1) core_req_addr(i)(wordSelBits + bankSelBits - 1, wordSelBits) else 0.U)
    core_req_line_addr(i) := core_req_addr(i)(p.wordAddrWidth - 1, wordSelBits + bankSelBits)
  }

  // SV: pack into CORE_REQ_DATAW bundle
  // CORE_REQ_DATAW = LINE_ADDR_WIDTH + 1 + WORD_SEL_WIDTH + WORD_SIZE + WORD_WIDTH + TAG_WIDTH + UP(MEM_FLAGS_WIDTH)
  private val coreReqDataW = lineAddrWidth + 1 + wordSelWidth + wordSize + wordWidth + tagWidth + math.max(1, flagsWidth)

  val core_req_data_in = Wire(Vec(numReqs, UInt(coreReqDataW.W)))
  for (i <- 0 until numReqs) {
    core_req_data_in(i) := Cat(core_req_line_addr(i), core_req_rw(i), core_req_wsel(i),
                                core_req_byteen(i), core_req_data(i), core_req_tag(i),
                                core_req_flags(i))
  }

  // =========================================================================
  // Per-bank wires
  // =========================================================================
  val pb_core_req_valid  = Wire(Vec(numBanks, Bool()))
  val pb_core_req_data   = Wire(Vec(numBanks, UInt(coreReqDataW.W)))
  val pb_core_req_idx    = Wire(Vec(numBanks, UInt(reqSelWidth.W)))
  val pb_core_req_ready  = Wire(Vec(numBanks, Bool()))

  val pb_core_req_addr   = Wire(Vec(numBanks, UInt(lineAddrWidth.W)))
  val pb_core_req_rw     = Wire(Vec(numBanks, Bool()))
  val pb_core_req_wsel   = Wire(Vec(numBanks, UInt(wordSelWidth.W)))
  val pb_core_req_byteen = Wire(Vec(numBanks, UInt(wordSize.W)))
  val pb_core_req_ddata  = Wire(Vec(numBanks, UInt(wordWidth.W)))
  val pb_core_req_tag    = Wire(Vec(numBanks, UInt(tagWidth.W)))
  val pb_core_req_flags  = Wire(Vec(numBanks, UInt(math.max(1, flagsWidth).W)))

  val pb_core_rsp_valid  = Wire(Vec(numBanks, Bool()))
  val pb_core_rsp_data   = Wire(Vec(numBanks, UInt(wordWidth.W)))
  val pb_core_rsp_tag    = Wire(Vec(numBanks, UInt(tagWidth.W)))
  val pb_core_rsp_idx    = Wire(Vec(numBanks, UInt(reqSelWidth.W)))
  val pb_core_rsp_ready  = WireDefault(VecInit.fill(numBanks)(false.B))

  val pb_mem_req_valid   = Wire(Vec(numBanks, Bool()))
  val pb_mem_req_addr    = Wire(Vec(numBanks, UInt(lineAddrWidth.W)))
  val pb_mem_req_rw      = Wire(Vec(numBanks, Bool()))
  val pb_mem_req_byteen  = Wire(Vec(numBanks, UInt(lineSize.W)))
  val pb_mem_req_data    = Wire(Vec(numBanks, UInt(lineWidth.W)))
  val pb_mem_req_tag     = Wire(Vec(numBanks, UInt(bankMemTagWidth.W)))
  val pb_mem_req_flags   = Wire(Vec(numBanks, UInt(math.max(1, flagsWidth).W)))
  val pb_mem_req_ready   = WireDefault(VecInit.fill(numBanks)(false.B))

  val pb_mem_rsp_valid   = WireDefault(VecInit.fill(numBanks)(false.B))
  val pb_mem_rsp_data    = Wire(Vec(numBanks, UInt(lineWidth.W)))
  val pb_mem_rsp_tag     = Wire(Vec(numBanks, UInt(bankMemTagWidth.W)))
  val pb_mem_rsp_ready   = Wire(Vec(numBanks, Bool()))

  // Default undriven wires
  for (b <- 0 until numBanks) {
    pb_mem_rsp_data(b) := 0.U
    pb_mem_rsp_tag(b)  := 0.U
  }

  // =========================================================================
  // Core-request dispatch crossbar: NUM_REQS → NUM_BANKS (VX_stream_xbar)
  //
  // SV uses VX_stream_xbar with ARBITER="R" (round-robin), OUT_BUF=REQ_XBAR_BUF.
  // Each bank gets at most one request per cycle, selected by bank-id.
  //
  // SV: per_bank_core_req_fire = per_bank_core_req_valid & per_bank_mem_req_ready
  //   (This is NOT core_req_valid & core_req_ready — it's used for tracking
  //    in-flight requests for the flush sequencer's BANK_SEL_LATENCY counter.)
  // =========================================================================

  // Simple round-robin crossbar: for each bank, pick one request targeting it.
  val xbar_rr = RegInit(VecInit.fill(numBanks)(0.U(math.max(1, log2Ceil(numReqs)).W)))

  for (b <- 0 until numBanks) {
    val base = xbar_rr(b)

    // Find candidates (This creates a Scala Seq[Bool])
    val cands_valid_seq = (0 until numReqs).map { i => core_req_valid(i) && (core_req_bid(i) === b.U) }
    
    // Convert to a hardware Vec[Bool] so it can be indexed dynamically
    val cands_valid_vec = VecInit(cands_valid_seq)

    // Build rotated priority starting from rr pointer
    val rotated_valid = Wire(Vec(numReqs, Bool()))
    for (i <- 0 until numReqs) {
      // Now it is perfectly legal to index with your UInt math!
      rotated_valid(i) := cands_valid_vec((base + i.U) % numReqs.U)
    }
    val winner_rot = PriorityEncoder(rotated_valid.asUInt)
    val winner_abs = ((base + winner_rot) % numReqs.U).asUInt
    val winner_val = rotated_valid.asUInt.orR

    pb_core_req_valid(b) := winner_val
    pb_core_req_data(b)  := Mux(winner_val, core_req_data_in(winner_abs), 0.U)
    pb_core_req_idx(b)   := winner_abs

    // Advance RR pointer when a request fires
    when (winner_val && pb_core_req_ready(b)) {
      xbar_rr(b) := ((winner_abs + 1.U) % numReqs.U).asUInt
    }

    // Back-pressure to original requester
    for (r <- 0 until numReqs) {
      when (winner_abs === r.U && winner_val) {
        core_req_ready(r) := pb_core_req_ready(b)
      }
    }
  }

  // Unpack bank request data
  for (b <- 0 until numBanks) {
    val d = pb_core_req_data(b)
    var lo = 0
    pb_core_req_flags(b) := d(lo + math.max(1, flagsWidth) - 1, lo); lo += math.max(1, flagsWidth)
    pb_core_req_tag(b)   := d(lo + tagWidth - 1, lo);  lo += tagWidth
    pb_core_req_ddata(b) := d(lo + wordWidth - 1, lo);  lo += wordWidth
    pb_core_req_byteen(b):= d(lo + wordSize - 1, lo);  lo += wordSize
    pb_core_req_wsel(b)  := d(lo + wordSelWidth - 1, lo); lo += wordSelWidth
    pb_core_req_rw(b)    := d(lo);  lo += 1
    pb_core_req_addr(b)  := d(lo + lineAddrWidth - 1, lo)
  }

  // SV: per_bank_core_req_fire = per_bank_core_req_valid & per_bank_mem_req_ready
  // (used by CacheInit in-flight counter; NOT core_req_ready)
  val per_bank_core_req_fire = VecInit((0 until numBanks).map { b =>
    pb_core_req_valid(b) && pb_mem_req_ready(b)
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
      // SV: CORE_OUT_REG = CORE_RSP_BUF_ENABLE ? 0 : TO_OUT_BUF_REG(CORE_OUT_BUF)
      coreOutReg = if (coreRspBufEnable) 0 else (if (coreOutBuf > 1) 1 else coreOutBuf),
      // SV: MEM_OUT_REG  = MEM_REQ_BUF_ENABLE  ? 0 : TO_OUT_BUF_REG(MEM_OUT_BUF)
      memOutReg  = if (memReqBufEnable)  0 else (if (memOutBuf  > 1) 1 else memOutBuf)
    ))

    bank.io.core_req_valid   := pb_core_req_valid(b)
    bank.io.core_req_addr    := pb_core_req_addr(b)
    bank.io.core_req_rw      := pb_core_req_rw(b)
    bank.io.core_req_wsel    := pb_core_req_wsel(b)
    bank.io.core_req_byteen  := pb_core_req_byteen(b)
    bank.io.core_req_data    := pb_core_req_ddata(b)
    bank.io.core_req_tag     := pb_core_req_tag(b)
    bank.io.core_req_idx     := pb_core_req_idx(b)
    bank.io.core_req_flags   := pb_core_req_flags(b)
    pb_core_req_ready(b)      := bank.io.core_req_ready

    pb_core_rsp_valid(b) := bank.io.core_rsp_valid
    pb_core_rsp_data(b)  := bank.io.core_rsp_data
    pb_core_rsp_tag(b)   := bank.io.core_rsp_tag
    pb_core_rsp_idx(b)   := bank.io.core_rsp_idx
    bank.io.core_rsp_ready := pb_core_rsp_ready(b)

    pb_mem_req_valid(b)  := bank.io.mem_req_valid
    pb_mem_req_addr(b)   := bank.io.mem_req_addr
    pb_mem_req_rw(b)     := bank.io.mem_req_rw
    pb_mem_req_byteen(b) := bank.io.mem_req_byteen
    pb_mem_req_data(b)   := bank.io.mem_req_data
    pb_mem_req_tag(b)    := bank.io.mem_req_tag
    pb_mem_req_flags(b)  := bank.io.mem_req_flags
    bank.io.mem_req_ready := pb_mem_req_ready(b)

    bank.io.mem_rsp_valid := pb_mem_rsp_valid(b)
    bank.io.mem_rsp_data  := pb_mem_rsp_data(b)
    bank.io.mem_rsp_tag   := pb_mem_rsp_tag(b)
    pb_mem_rsp_ready(b)   := bank.io.mem_rsp_ready

    bank.io.flush_begin := per_bank_flush_begin(b)
    bank.io.flush_uuid  := flush_uuid
    flush_end_vec(b)    := bank.io.flush_end
  }
  per_bank_flush_end := flush_end_vec.asUInt

  // =========================================================================
  // Core-response gather crossbar: NUM_BANKS → NUM_REQS (VX_stream_xbar)
  //
  // SV: CORE_RSP_DATAW = WORD_WIDTH + TAG_WIDTH
  //     core_rsp_data_in[i] = {per_bank_core_rsp_data[i], per_bank_core_rsp_tag[i]}
  // sel_in = per_bank_core_rsp_idx (routes each bank rsp to original requester)
  //
  // After xbar, output goes through VX_elastic_buffer (core_rsp_buf).
  // =========================================================================

  // SV: CORE_RSP_DATAW = WORD_WIDTH + TAG_WIDTH
  private val coreRspDataW = wordWidth + tagWidth

  val rsp_rr = RegInit(VecInit.fill(numReqs)(0.U(math.max(1, log2Ceil(numBanks)).W)))

for (r <- 0 until numReqs) {
    val base = rsp_rr(r)

    // 1. Find banks that have a response for requester r (Scala Seq[Bool])
    val cands_seq = (0 until numBanks).map { b =>
      pb_core_rsp_valid(b) && (pb_core_rsp_idx(b) === r.U)
    }
    
    // 2. Convert to Hardware Vec
    val cands_vec = VecInit(cands_seq)

    // 3. Round-robin selection using UInt index
    val rotated = Wire(Vec(numBanks, Bool()))
    for (i <- 0 until numBanks) {
      // Safely indexed by hardware UInt math!
      rotated(i) := cands_vec((base + i.U) % numBanks.U)
    }
    val winner_rot = PriorityEncoder(rotated.asUInt)
    val winner_b   = ((base + winner_rot) % numBanks.U).asUInt
    val winner_val = rotated.asUInt.orR

    // Elastic buffer (VX_elastic_buffer): buffered output for core response
    val bufSize = if (coreRspBufEnable) {
      // TO_OUT_BUF_SIZE(CORE_OUT_BUF): 0→0, 1→1, 2→2, 3→3
      math.max(1, coreOutBuf)
    } else {
      0  // no buffering when only 1 bank and 1 req
    }
    val rsp_buf = Module(new Queue(new Bundle {
      val data = UInt(wordWidth.W)
      val tag  = UInt(tagWidth.W)
    }, if (bufSize > 0) bufSize else 1,
       pipe = (bufSize <= 1), flow = false))

    rsp_buf.io.enq.valid    := winner_val
    rsp_buf.io.enq.bits.data := pb_core_rsp_data(winner_b)
    rsp_buf.io.enq.bits.tag  := pb_core_rsp_tag(winner_b)

    // Connect to cacheInit core_bus_out response
    cacheInit.io.core_bus_out(r).rsp.valid := rsp_buf.io.deq.valid
    cacheInit.io.core_bus_out(r).rsp.bits.data := rsp_buf.io.deq.bits.data
    cacheInit.io.core_bus_out(r).rsp.bits.tag  := rsp_buf.io.deq.bits.tag
                                                   .asTypeOf(cacheInit.io.core_bus_out(r).rsp.bits.tag)
    rsp_buf.io.deq.ready := cacheInit.io.core_bus_out(r).rsp.ready

    // Back-pressure to winning bank
    for (b <- 0 until numBanks) {
      when (winner_b === b.U && winner_val) {
        pb_core_rsp_ready(b) := rsp_buf.io.enq.ready
      }
    }

    when (winner_val && rsp_buf.io.enq.ready) {
      rsp_rr(r) := ((winner_b + 1.U) % numBanks.U).asUInt
    }
  }

  // =========================================================================
  // Memory request arbitration: NUM_BANKS → MEM_PORTS (VX_stream_arb)
  //
  // SV: MEM_REQ_DATAW = CS_LINE_ADDR_WIDTH + 1 + LINE_SIZE + CS_LINE_WIDTH +
  //                     BANK_MEM_TAG_WIDTH + UP(MEM_FLAGS_WIDTH)
  //     Pack: {rw, addr, data, byteen, flags, tag}
  //
  // After arbitration, tag is augmented with bank-id:
  //   if NUM_BANKS > 1 && NUM_BANKS != MEM_PORTS:
  //     bank_id = Cat(mem_req_sel_out[i], MEM_PORTS_SEL_WIDTH'(i))
  //     mem_req_addr_w = Cat(mem_req_addr, bank_id)
  //     mem_req_tag_w  = Cat(mem_req_tag, mem_req_sel_out[i])
  //   if NUM_BANKS > 1 && NUM_BANKS == MEM_PORTS:
  //     bank_id = MEM_PORTS_SEL_WIDTH'(i)
  //     mem_req_addr_w = Cat(mem_req_addr, MEM_PORTS_SEL_WIDTH'(i))
  //     mem_req_tag_w  = MEM_TAG_WIDTH'(mem_req_tag)
  //   if NUM_BANKS == 1:
  //     mem_req_addr_w = mem_req_addr
  //     mem_req_tag_w  = MEM_TAG_WIDTH'(mem_req_tag)
  //
  // Output goes through VX_elastic_buffer (mem_req_buf).
  // =========================================================================

  val memReqArb = Module(new VxStreamArb(
    new CacheMemReqArbBundle(lineAddrWidth, lineWidth, lineSize, math.max(1, flagsWidth), bankMemTagWidth),
    numInputs  = numBanks,
    numOutputs = memPorts,
    arbiter    = "R",
    outBuf     = 0
  ))

  for (b <- 0 until numBanks) {
    val d = Wire(new CacheMemReqArbBundle(lineAddrWidth, lineWidth, lineSize, math.max(1, flagsWidth), bankMemTagWidth))
    d.rw     := pb_mem_req_rw(b)
    d.addr   := pb_mem_req_addr(b)
    d.data   := pb_mem_req_data(b)
    d.byteen := pb_mem_req_byteen(b)
    d.flags  := pb_mem_req_flags(b)
    d.tag    := pb_mem_req_tag(b)
    memReqArb.io.validIn(b) := pb_mem_req_valid(b)
    memReqArb.io.dataIn(b)  := d
    pb_mem_req_ready(b)     := memReqArb.io.readyIn(b)
  }

  for (pi <- 0 until memPorts) {
    val arbOut = memReqArb.io.dataOut(pi)

    val mem_req_addr_w = Wire(UInt(memAddrWidthCS.W))
    val mem_req_tag_w  = Wire(UInt(memTagWidth.W))

    if (numBanks > 1) {
      if (numBanks != memPorts) {
        // selOut(pi) = group index r; actual bank = r * memPorts + pi
        val selOut  = memReqArb.io.selOut(pi)
        val bank_id = Cat(selOut, pi.U(memPortsSelBits.W))(bankSelBits - 1, 0)
        mem_req_addr_w := Cat(arbOut.addr, bank_id)(memAddrWidthCS - 1, 0)
        mem_req_tag_w  := Cat(arbOut.tag, selOut)(memTagWidth - 1, 0)
      } else {
        // NUM_BANKS == MEM_PORTS: 1:1, bank_id = port index
        mem_req_addr_w := Cat(arbOut.addr, pi.U(memPortsSelBits.W))(memAddrWidthCS - 1, 0)
        mem_req_tag_w  := arbOut.tag(memTagWidth - 1, 0)
      }
    } else {
      mem_req_addr_w := arbOut.addr(memAddrWidthCS - 1, 0)
      mem_req_tag_w  := arbOut.tag(memTagWidth - 1, 0)
    }

    val mem_req_buf = Wire(Decoupled(
      new CacheMemReqBufBundle(lineSize, memAddrWidthCS, lineWidth, memTagWidth, math.max(1, flagsWidth))
    ))

    mem_req_buf.valid       := memReqArb.io.validOut(pi)
    mem_req_buf.bits.rw     := arbOut.rw
    mem_req_buf.bits.byteen := arbOut.byteen
    mem_req_buf.bits.addr   := mem_req_addr_w
    mem_req_buf.bits.data   := arbOut.data
    mem_req_buf.bits.tag    := mem_req_tag_w
    mem_req_buf.bits.flags  := arbOut.flags
    mem_req_buf.ready       := io.mem_bus(pi).req.ready

    memReqArb.io.readyOut(pi)       := mem_req_buf.ready

    io.mem_bus(pi).req.valid        := mem_req_buf.valid
    io.mem_bus(pi).req.bits.rw      := mem_req_buf.bits.rw
    io.mem_bus(pi).req.bits.byteen  := mem_req_buf.bits.byteen
    io.mem_bus(pi).req.bits.addr    := mem_req_buf.bits.addr
    io.mem_bus(pi).req.bits.data    := mem_req_buf.bits.data
    io.mem_bus(pi).req.bits.tag     := mem_req_buf.bits.tag
                                       .asTypeOf(io.mem_bus(pi).req.bits.tag)
    io.mem_bus(pi).req.bits.flags   := mem_req_buf.bits.flags
                                       .asTypeOf(io.mem_bus(pi).req.bits.flags)
  }

  // =========================================================================
  // Memory response demux: MEM_PORTS → NUM_BANKS (VX_stream_omega)
  //
  // SV: the memory tag encodes which bank this response belongs to.
  //
  //   MEM_TAG_WIDTH = BANK_MEM_TAG_WIDTH + BANK_SEL_BITS
  //   bank_id is encoded in the lower BANK_SEL_BITS of the tag.
  //
  // More precisely:
  //   if NUM_BANKS > 1 && NUM_BANKS != MEM_PORTS:
  //     tag[MEM_ARB_SEL_BITS-1:0] = arb_sel  → bank within port
  //     tag[BANK_SEL_BITS-1:MEM_ARB_SEL_BITS] is port_sel (from port i)
  //     bank_id = Cat(tag[MEM_ARB_SEL_BITS-1:0], port_i_bits)
  //   if NUM_BANKS > 1 && NUM_BANKS == MEM_PORTS:
  //     bank_id = port_index  (i)
  //   if NUM_BANKS == 1:
  //     bank_id = 0
  //
  // SV: mem_rsp_queue_sel[i] = bank_id extracted from mem_rsp_queue_data[i]
  //
  // First, the responses go through an elastic buffer (mem_rsp_queue).
  // =========================================================================

  // Per-port memory response elastic buffer (MRSQ_SIZE)
  val mem_rsp_q_valid = Wire(Vec(memPorts, Bool()))
  val mem_rsp_q_data  = Wire(Vec(memPorts, UInt((lineWidth + memTagWidth).W)))
  val mem_rsp_q_ready = WireDefault(VecInit.fill(memPorts)(false.B))

  for (pi <- 0 until memPorts) {
    val bufSize = if (mrsqSize > 2) mrsqSize else mrsqSize  // always buffer
    val rsp_q = Module(new Queue(UInt((lineWidth + memTagWidth).W), bufSize,
                                  pipe = false, flow = false))
    val packed = Cat(io.mem_bus(pi).rsp.bits.data,
                     io.mem_bus(pi).rsp.bits.tag.asUInt)
    rsp_q.io.enq.valid  := io.mem_bus(pi).rsp.valid
    rsp_q.io.enq.bits   := packed
    io.mem_bus(pi).rsp.ready := rsp_q.io.enq.ready
    mem_rsp_q_valid(pi) := rsp_q.io.deq.valid
    mem_rsp_q_data(pi)  := rsp_q.io.deq.bits
    rsp_q.io.deq.ready  := mem_rsp_q_ready(pi)
  }

  // Route responses to banks
  for (b <- 0 until numBanks) {
    // Find which port (if any) has a response for bank b
    val src_valid = Wire(Vec(memPorts, Bool()))
    for (pi <- 0 until memPorts) {
      val tag_bits = mem_rsp_q_data(pi)(memTagWidth - 1, 0)
      val bank_id = Wire(UInt(bankSelWidth.W))
      if (numBanks > 1) {
        if (numBanks != memPorts) {
          // bank_id = Cat(tag[MEM_ARB_SEL_BITS-1:0], port_sel_bits)
          val arb_bits  = tag_bits(memArbSelBits - 1, 0)
          val port_bits = pi.U(memPortsSelBits.W)
          bank_id := Cat(arb_bits, port_bits)(bankSelBits - 1, 0)
        } else {
          // bank_id = port index
          bank_id := pi.U
        }
      } else {
        bank_id := 0.U
      }
      src_valid(pi) := mem_rsp_q_valid(pi) && (bank_id === b.U)
    }

    val any_valid = src_valid.asUInt.orR
    val src_idx   = PriorityEncoder(src_valid.asUInt)

    val rsp_packed  = mem_rsp_q_data(src_idx)
    // SV: {per_bank_mem_rsp_data[i], per_bank_mem_rsp_tag[i]} = per_bank_mem_rsp_pdata[i]
    //     where per_bank_mem_rsp_pdata stripes off the arb-sel bits from the tag.
    // tag after stripping arb bits = bank_mem_tag
    val raw_tag     = rsp_packed(memTagWidth - 1, 0)
    // SV: mem_rsp_tag_s = raw_tag[MEM_TAG_WIDTH-1:MEM_ARB_SEL_BITS]  = BANK_MEM_TAG_WIDTH bits
    val bank_tag    = if (memArbSelBits > 0) raw_tag(memTagWidth - 1, memArbSelBits)
                      else raw_tag(bankMemTagWidth - 1, 0)
    val rsp_data    = rsp_packed(lineWidth + memTagWidth - 1, memTagWidth)

    pb_mem_rsp_valid(b) := any_valid
    pb_mem_rsp_data(b)  := rsp_data
    pb_mem_rsp_tag(b)   := bank_tag(bankMemTagWidth - 1, 0)

    mem_rsp_q_ready(src_idx) := pb_mem_rsp_ready(b) && any_valid
  }
}
