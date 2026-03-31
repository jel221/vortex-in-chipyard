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

// Chisel translation of VX_cache_bank.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * CacheBank – single-bank, 2-stage cache pipeline.
 *
 * Mirrors VX_cache_bank.sv faithfully.
 *
 * Pipeline stages:
 *   Stage 0 (SEL → ST0) : arbitration, tag lookup, repl lookup
 *   Stage 1 (ST0 → ST1) : tag-match result, data read, MSHR alloc
 *
 * Input priority (highest first):
 *   init > replay > fill (mem_rsp) > flush > core_req
 *
 * Parameters: taken from CacheParams plus bank-specific extras.
 *   numReqs      – NUM_REQS  (number of core request ports that multiplex to this bank)
 *   crsqSize     – core response queue depth
 *   mshrSize     – MSHR entries
 *   mreqSize     – memory request queue depth
 *   tagWidth     – core request tag width
 *   uuidWidth    – UUID sub-field width
 *   coreOutReg   – core response output register enable (0 = no reg)
 *   memOutReg    – memory request output register enable (0 = no reg)
 */
class CacheBank(
  p:          CacheParams,
  bankId:     Int = 0,
  numReqs:    Int = 1,
  crsqSize:   Int = 1,
  mshrSize:   Int = 4,
  mreqSize:   Int = 4,
  tagWidth:   Int = 8,
  uuidWidth:  Int = 0,
  coreOutReg: Int = 0,
  memOutReg:  Int = 0
) extends Module {

  // Derived widths
  private val lineSelBits   = p.lineSelBits
  private val tagSelBits    = p.tagSelBits
  private val lineAddrWidth = p.lineAddrWidth
  private val wordWidth     = p.wordWidth
  private val lineWidth     = p.lineWidth
  private val lineSize      = p.lineSize
  private val wordSize      = p.wordSize
  private val wordsPerLine  = p.wordsPerLine
  private val waySelWidth   = p.waySelWidth
  private val numWays       = p.numWays
  private val mshrAddrWidth = math.max(1, log2Ceil(mshrSize))
  private val reqSelWidth   = math.max(1, log2Ceil(numReqs))
  private val wordSelWidth  = p.wordSelWidth
  private val memTagWidth   = uuidWidth + mshrAddrWidth
  private val memFlagsWidth = 3  // MEM_FLAGS_WIDTH constant

  val io = IO(new Bundle {
    // Core request (from bank crossbar)
    val core_req_valid   = Input(Bool())
    val core_req_addr    = Input(UInt(lineAddrWidth.W))
    val core_req_rw      = Input(Bool())
    val core_req_wsel    = Input(UInt(wordSelWidth.W))
    val core_req_byteen  = Input(UInt(wordSize.W))
    val core_req_data    = Input(UInt(wordWidth.W))
    val core_req_tag     = Input(UInt(tagWidth.W))
    val core_req_idx     = Input(UInt(reqSelWidth.W))
    val core_req_flags   = Input(UInt(math.max(1, memFlagsWidth).W))
    val core_req_ready   = Output(Bool())

    // Core response
    val core_rsp_valid   = Output(Bool())
    val core_rsp_data    = Output(UInt(wordWidth.W))
    val core_rsp_tag     = Output(UInt(tagWidth.W))
    val core_rsp_idx     = Output(UInt(reqSelWidth.W))
    val core_rsp_ready   = Input(Bool())

    // Memory request (to lower cache / DRAM)
    val mem_req_valid    = Output(Bool())
    val mem_req_addr     = Output(UInt(lineAddrWidth.W))
    val mem_req_rw       = Output(Bool())
    val mem_req_byteen   = Output(UInt(lineSize.W))
    val mem_req_data     = Output(UInt(lineWidth.W))
    val mem_req_tag      = Output(UInt(memTagWidth.W))
    val mem_req_flags    = Output(UInt(math.max(1, memFlagsWidth).W))
    val mem_req_ready    = Input(Bool())

    // Memory response (fill)
    val mem_rsp_valid    = Input(Bool())
    val mem_rsp_data     = Input(UInt(lineWidth.W))
    val mem_rsp_tag      = Input(UInt(memTagWidth.W))
    val mem_rsp_ready    = Output(Bool())

    // Flush
    val flush_begin      = Input(Bool())
    val flush_uuid       = Input(UInt(math.max(1, uuidWidth).W))
    val flush_end        = Output(Bool())
  })

  // =========================================================================
  // Flush unit
  // =========================================================================
  val cacheFlush = Module(new CacheFlush(p, bankId))
  val flush_valid    = cacheFlush.io.flush_valid
  val init_valid     = cacheFlush.io.flush_init
  val flush_sel      = cacheFlush.io.flush_line
  val flush_way      = cacheFlush.io.flush_way
  val flush_ready    = Wire(Bool())
  cacheFlush.io.flush_begin := io.flush_begin
  cacheFlush.io.flush_ready := flush_ready
  io.flush_end              := cacheFlush.io.flush_end

  // =========================================================================
  // MSHR
  // =========================================================================
  private val mshrDataWidth = wordSelWidth + wordSize + wordWidth + tagWidth + reqSelWidth

  val cacheMshr = Module(new CacheMshr(mshrSize, mshrDataWidth, lineAddrWidth, p.writeback))

  // Replay wires (dequeue output from MSHR)
  val replay_valid  = cacheMshr.io.dequeue_valid
  val replay_addr   = cacheMshr.io.dequeue_addr
  val replay_rw     = cacheMshr.io.dequeue_rw
  val replay_id     = cacheMshr.io.dequeue_id
  val replay_ready  = Wire(Bool())
  cacheMshr.io.dequeue_ready := replay_ready

  // Unpack replay data: {word_idx, byteen, data, tag, idx}
  val replay_data_packed = cacheMshr.io.dequeue_data
  val replay_idx    = replay_data_packed(reqSelWidth - 1, 0)
  val replay_tag    = replay_data_packed(reqSelWidth + tagWidth - 1, reqSelWidth)
  val replay_data   = replay_data_packed(reqSelWidth + tagWidth + wordWidth - 1, reqSelWidth + tagWidth)
  val replay_byteen = replay_data_packed(reqSelWidth + tagWidth + wordWidth + wordSize - 1, reqSelWidth + tagWidth + wordWidth)
  val replay_wsel   = replay_data_packed(reqSelWidth + tagWidth + wordWidth + wordSize + wordSelWidth - 1, reqSelWidth + tagWidth + wordWidth + wordSize)

  // MSHR fill (triggered by memory response)
  val mem_rsp_id = io.mem_rsp_tag(mshrAddrWidth - 1, 0)
  cacheMshr.io.fill_valid := false.B  // connected below
  cacheMshr.io.fill_id    := mem_rsp_id

  // =========================================================================
  // MSHR pending-size tracker (for almost-full signalling)
  // =========================================================================
  val mshr_empty    = Wire(Bool())
  val mshr_alm_full = Wire(Bool())
  cacheFlush.io.mshr_empty := mshr_empty

  // =========================================================================
  // Memory request queue
  // =========================================================================
  val mreq_queue_push  = Wire(Bool())
  val mreq_queue_pop   = Wire(Bool())
  val mreq_queue_in    = Wire(new Bundle {
    val rw      = Bool()
    val addr    = UInt(lineAddrWidth.W)
    val byteen  = UInt(lineSize.W)
    val data    = UInt(lineWidth.W)
    val tag     = UInt(memTagWidth.W)
    val flags   = UInt(math.max(1, memFlagsWidth).W)
  })
  val mreq_queue_out   = Wire(mreq_queue_in.cloneType)
  val mreq_queue_empty  = Wire(Bool())
  val mreq_queue_alm_full = Wire(Bool())

  val mreq_fifo = Module(new Queue(chiselTypeOf(mreq_queue_in), mreqSize, pipe = false, flow = false))
  mreq_fifo.io.enq.valid := mreq_queue_push
  mreq_fifo.io.enq.bits  := mreq_queue_in
  mreq_queue_pop         := mreq_fifo.io.deq.valid && io.mem_req_ready
  mreq_fifo.io.deq.ready := io.mem_req_ready
  mreq_queue_out         := mreq_fifo.io.deq.bits
  mreq_queue_empty       := !mreq_fifo.io.deq.valid
  // Almost-full = entries >= mreqSize - PIPELINE_STAGES(2)
  mreq_queue_alm_full    := (mreq_fifo.io.count >= (mreqSize - 2).U)

  // =========================================================================
  // Core response queue (CRSQ)
  // =========================================================================
  val crsp_queue_valid = Wire(Bool())
  val crsp_queue_data  = Wire(UInt(wordWidth.W))
  val crsp_queue_idx   = Wire(UInt(reqSelWidth.W))
  val crsp_queue_tag   = Wire(UInt(tagWidth.W))
  val crsp_queue_ready = Wire(Bool())
  val crsp_queue_stall = Wire(Bool())

  val crsp_q = Module(new Queue(new Bundle {
    val tag  = UInt(tagWidth.W)
    val data = UInt(wordWidth.W)
    val idx  = UInt(reqSelWidth.W)
  }, crsqSize, pipe = false, flow = false))

  crsp_q.io.enq.valid     := crsp_queue_valid
  crsp_q.io.enq.bits.tag  := crsp_queue_tag
  crsp_q.io.enq.bits.data := crsp_queue_data
  crsp_q.io.enq.bits.idx  := crsp_queue_idx
  crsp_queue_ready        := crsp_q.io.enq.ready
  io.core_rsp_valid       := crsp_q.io.deq.valid
  io.core_rsp_data        := crsp_q.io.deq.bits.data
  io.core_rsp_tag         := crsp_q.io.deq.bits.tag
  io.core_rsp_idx         := crsp_q.io.deq.bits.idx
  crsp_q.io.deq.ready     := io.core_rsp_ready

  crsp_queue_stall := crsp_queue_valid && !crsp_queue_ready
  val pipe_stall   = crsp_queue_stall

  // =========================================================================
  // Memory response address recovery
  // =========================================================================
  val mem_rsp_addr = cacheMshr.io.fill_addr

  // =========================================================================
  // Input arbitration
  // =========================================================================
  val replay_grant  = !init_valid
  val replay_enable = replay_grant && replay_valid
  val fill_grant    = !init_valid && !replay_enable
  val fill_enable   = fill_grant && io.mem_rsp_valid
  val flush_grant   = !init_valid && !replay_enable && !fill_enable
  val flush_enable  = flush_grant && flush_valid
  val creq_grant    = !init_valid && !replay_enable && !fill_enable && !flush_enable
  val creq_enable   = creq_grant && io.core_req_valid

  replay_ready  := replay_grant &&
                   !(!(p.writeback != 0).B && replay_rw && mreq_queue_alm_full) &&
                   !pipe_stall
  io.mem_rsp_ready := fill_grant &&
                      !((p.writeback != 0).B && mreq_queue_alm_full) &&
                      !pipe_stall
  flush_ready   := flush_grant &&
                   !((p.writeback != 0).B && mreq_queue_alm_full) &&
                   !pipe_stall
  io.core_req_ready := creq_grant &&
                       !mreq_queue_alm_full &&
                       !mshr_alm_full &&
                       !pipe_stall

  val init_fire     = init_valid
  val replay_fire   = replay_valid && replay_ready
  val mem_rsp_fire  = io.mem_rsp_valid && io.mem_rsp_ready
  val flush_fire    = flush_valid && flush_ready
  val core_req_fire = io.core_req_valid && io.core_req_ready

  // =========================================================================
  // flush_tag for pipeline
  // =========================================================================
  val flush_tag = Wire(UInt(tagWidth.W))
  if (uuidWidth != 0) {
    flush_tag := Cat(io.flush_uuid, 0.U((tagWidth - uuidWidth).W))
  } else {
    flush_tag := 0.U
  }

  // =========================================================================
  // Pipeline stage 0 mux (SEL)
  // =========================================================================
  val valid_sel    = init_fire || replay_fire || mem_rsp_fire || flush_fire || core_req_fire
  val rw_sel       = Mux(replay_valid, replay_rw,      io.core_req_rw)
  val byteen_sel   = Mux(replay_valid, replay_byteen,  io.core_req_byteen)
  val addr_sel     = Mux(init_valid || flush_valid,
                         flush_sel.asUInt,
                         Mux(replay_valid, replay_addr,
                             Mux(io.mem_rsp_valid, mem_rsp_addr, io.core_req_addr)))
  val word_idx_sel = Mux(replay_valid, replay_wsel,    io.core_req_wsel)
  val req_idx_sel  = Mux(replay_valid, replay_idx,     io.core_req_idx)
  val tag_sel      = Mux(init_valid || flush_valid,
                         Mux(flush_valid, flush_tag, 0.U),
                         Mux(replay_valid, replay_tag,
                             Mux(io.mem_rsp_valid, {
                               // mem_rsp_tag_s: pad or cut mem_rsp_tag to tagWidth
                               val mt = io.mem_rsp_tag
                               if (tagWidth > memTagWidth)
                                 Cat(mt, 0.U((tagWidth - memTagWidth).W))
                               else
                                 mt(memTagWidth - 1, memTagWidth - tagWidth)
                             }, io.core_req_tag)))
  val flags_sel    = Mux(io.core_req_valid, io.core_req_flags, 0.U)

  // data sel: fill data for fill/flush; otherwise word replicated
  val data_sel = Wire(UInt(lineWidth.W))
  if (p.writeEnable != 0) {
    val word_data = Wire(UInt(lineWidth.W))
    if (wordWidth < lineWidth) {
      word_data := Fill(wordsPerLine, Mux(replay_valid, replay_data, io.core_req_data))
    } else {
      word_data := Mux(replay_valid, replay_data, io.core_req_data)
    }
    // lower wordWidth bits from replay/creq; upper from mem_rsp_data
    val merged = Wire(UInt(lineWidth.W))
    val lo = Mux(replay_valid, replay_data, Mux(io.mem_rsp_valid, io.mem_rsp_data(wordWidth-1, 0), io.core_req_data))
    if (lineWidth > wordWidth) {
      merged := Cat(io.mem_rsp_data(lineWidth-1, wordWidth), lo)
    } else {
      merged := lo
    }
    data_sel := Mux(io.mem_rsp_valid && !replay_valid, io.mem_rsp_data, word_data)
  } else {
    data_sel := io.mem_rsp_data
  }

  val is_init_sel   = init_valid
  val is_creq_sel   = creq_enable || replay_enable
  val is_fill_sel   = fill_enable
  val is_flush_sel  = flush_enable
  val is_replay_sel = replay_enable

  // =========================================================================
  // Pipe register 0 → stage 0
  // =========================================================================
  val valid_st0     = RegNext(Mux(pipe_stall, false.B, valid_sel), false.B)
  val is_init_st0   = RegEnable(is_init_sel,   !pipe_stall)
  val is_fill_st0   = RegEnable(is_fill_sel,   !pipe_stall)
  val is_flush_st0  = RegEnable(is_flush_sel,  !pipe_stall)
  val is_creq_st0   = RegEnable(is_creq_sel,   !pipe_stall)
  val is_replay_st0 = RegEnable(is_replay_sel, !pipe_stall)
  val flags_st0     = RegEnable(flags_sel,     !pipe_stall)
  val flush_way_st0 = RegEnable(flush_way,     !pipe_stall)
  val addr_st0      = RegEnable(addr_sel,      !pipe_stall)
  val data_st0      = RegEnable(data_sel,      !pipe_stall)
  val rw_st0        = RegEnable(rw_sel,        !pipe_stall)
  val byteen_st0    = RegEnable(byteen_sel,    !pipe_stall)
  val word_idx_st0  = RegEnable(word_idx_sel,  !pipe_stall)
  val req_idx_st0   = RegEnable(req_idx_sel,   !pipe_stall)
  val tag_st0       = RegEnable(tag_sel,       !pipe_stall)
  val replay_id_st0 = RegEnable(replay_id,     !pipe_stall)

  val line_idx_sel  = addr_sel(lineSelBits - 1, 0)
  val line_idx_st0  = addr_st0(lineSelBits - 1, 0)
  val line_tag_st0  = addr_st0(lineAddrWidth - 1, lineSelBits)

  val write_word_st0 = data_st0(wordWidth - 1, 0)

  val is_read_st0  = is_creq_st0 && !rw_st0
  val is_write_st0 = is_creq_st0 && rw_st0
  val do_init_st0  = valid_st0 && is_init_st0
  val do_flush_st0 = valid_st0 && is_flush_st0
  val do_read_st0  = valid_st0 && is_read_st0
  val do_write_st0 = valid_st0 && is_write_st0
  val do_fill_st0  = valid_st0 && is_fill_st0

  // =========================================================================
  // Replacement policy
  // =========================================================================
  val cacheRepl = Module(new CacheRepl(p))
  cacheRepl.io.stall        := pipe_stall
  cacheRepl.io.init         := do_init_st0

  // Connections filled in after stage-1 wires defined below
  val victim_way_st0 = cacheRepl.io.repl_way

  // =========================================================================
  // Tag lookup
  // =========================================================================
  val cacheTags = Module(new CacheTags(p))
  val is_hit_st0    = Wire(Bool())
  val tag_matches_st0 = Wire(UInt(numWays.W))
  val evict_way_st0 = Wire(UInt(waySelWidth.W))
  val is_dirty_st0  = Wire(Bool())
  val evict_tag_st0 = Wire(UInt(tagSelBits.W))

  // way_idx_st1 is registered from way_idx_st0 (declared with placeholder, filled after stage-0)
  val way_idx_st1 = RegInit(0.U(waySelWidth.W))

  cacheTags.io.stall       := pipe_stall
  cacheTags.io.init        := do_init_st0
  cacheTags.io.flush       := do_flush_st0 && !pipe_stall
  cacheTags.io.fill        := do_fill_st0  && !pipe_stall
  cacheTags.io.read        := do_read_st0  && !pipe_stall
  cacheTags.io.write       := do_write_st0 && !pipe_stall
  cacheTags.io.line_idx    := line_idx_st0
  cacheTags.io.line_idx_n  := line_idx_sel
  cacheTags.io.line_tag    := line_tag_st0
  cacheTags.io.evict_way   := evict_way_st0

  tag_matches_st0 := cacheTags.io.tag_matches
  is_dirty_st0    := cacheTags.io.evict_dirty
  evict_tag_st0   := cacheTags.io.evict_tag

  // One-hot to binary for hit way
  val hit_idx_st0 = OHToUInt(tag_matches_st0)
  val way_idx_st0 = Mux(is_creq_st0, hit_idx_st0, evict_way_st0)
  is_hit_st0      := tag_matches_st0.orR

  // =========================================================================
  // Repl hookup (needs ST1 info, filled after ST1 wire declarations)
  // =========================================================================

  // =========================================================================
  // MSHR allocate at stage 0
  // =========================================================================
  val mshr_alloc_id_st0   = cacheMshr.io.allocate_id
  val mshr_pending_st0    = cacheMshr.io.allocate_pending
  val mshr_previd_st0     = cacheMshr.io.allocate_previd
  val mshr_id_st0         = Mux(is_replay_st0, replay_id_st0, mshr_alloc_id_st0)

  cacheMshr.io.allocate_valid   := (valid_st0 && is_creq_st0 && !is_replay_st0) && !pipe_stall
  cacheMshr.io.allocate_addr    := addr_st0
  cacheMshr.io.allocate_rw      := rw_st0
  cacheMshr.io.allocate_data    := Cat(word_idx_st0, byteen_st0, write_word_st0, tag_st0, req_idx_st0)

  // =========================================================================
  // Pipe register 1 → stage 1
  // =========================================================================
  val valid_st1     = RegNext(Mux(pipe_stall, false.B, valid_st0), false.B)
  val is_fill_st1   = RegEnable(is_fill_st0,   !pipe_stall)
  val is_flush_st1  = RegEnable(is_flush_st0,  !pipe_stall)
  val is_creq_st1   = RegEnable(is_creq_st0,   !pipe_stall)
  val is_replay_st1 = RegEnable(is_replay_st0, !pipe_stall)
  val is_dirty_st1  = RegEnable(is_dirty_st0,  !pipe_stall)
  val is_hit_st1    = RegEnable(is_hit_st0,    !pipe_stall)
  val rw_st1        = RegEnable(rw_st0,        !pipe_stall)
  val flags_st1     = RegEnable(flags_st0,     !pipe_stall)
  when (!pipe_stall) { way_idx_st1 := way_idx_st0 }
  val evict_tag_st1 = RegEnable(evict_tag_st0, !pipe_stall)
  val line_tag_st1  = RegEnable(line_tag_st0,  !pipe_stall)
  val line_idx_st1  = RegEnable(line_idx_st0,  !pipe_stall)
  val data_st1      = RegEnable(data_st0,      !pipe_stall)
  val byteen_st1    = RegEnable(byteen_st0,    !pipe_stall)
  val word_idx_st1  = RegEnable(word_idx_st0,  !pipe_stall)
  val req_idx_st1   = RegEnable(req_idx_st0,   !pipe_stall)
  val tag_st1       = RegEnable(tag_st0,       !pipe_stall)
  val mshr_id_st1   = RegEnable(mshr_id_st0,   !pipe_stall)
  val mshr_previd_st1 = RegEnable(mshr_previd_st0, !pipe_stall)
  val mshr_pending_st1 = RegEnable(mshr_pending_st0, !pipe_stall)

  val addr_st1  = Cat(line_tag_st1, line_idx_st1)
  val write_word_st1 = data_st1(wordWidth - 1, 0)

  val is_read_st1  = is_creq_st1 && !rw_st1
  val is_write_st1 = is_creq_st1 && rw_st1
  val do_read_st1  = valid_st1 && is_read_st1
  val do_write_st1 = valid_st1 && is_write_st1
  val do_lookup_st1 = do_read_st1 || do_write_st1

  // Now wire up the repl lookups that need stage-1 data
  cacheRepl.io.lookup_valid := do_lookup_st1 && !pipe_stall
  cacheRepl.io.lookup_hit   := is_hit_st1
  cacheRepl.io.lookup_line  := line_idx_st1
  cacheRepl.io.lookup_way   := way_idx_st1
  cacheRepl.io.repl_valid   := do_fill_st0 && !pipe_stall
  cacheRepl.io.repl_line    := line_idx_st0

  evict_way_st0 := Mux(is_fill_st0, victim_way_st0, flush_way_st0)

  // =========================================================================
  // Data RAM
  // =========================================================================
  val cacheData = Module(new CacheData(p))
  val fill_data_vec = io.mem_rsp_data.asTypeOf(Vec(wordsPerLine, UInt(wordWidth.W)))  // used during reg stage

  cacheData.io.init        := do_init_st0
  cacheData.io.fill        := do_fill_st0  && !pipe_stall
  cacheData.io.flush       := do_flush_st0 && !pipe_stall
  cacheData.io.read        := do_read_st0  && !pipe_stall
  cacheData.io.write       := do_write_st0 && !pipe_stall
  cacheData.io.evict_way   := evict_way_st0
  cacheData.io.tag_matches := tag_matches_st0
  cacheData.io.line_idx    := line_idx_st0
  cacheData.io.fill_data   := data_st0.asTypeOf(Vec(wordsPerLine, UInt(wordWidth.W)))
  cacheData.io.write_word  := write_word_st0
  cacheData.io.word_idx    := word_idx_st0
  cacheData.io.write_byteen := byteen_st0
  cacheData.io.way_idx_r   := way_idx_st1

  val read_data_st1   = cacheData.io.read_data
  val evict_byteen_st1 = cacheData.io.evict_byteen

  // =========================================================================
  // MSHR finalize at stage 1
  // =========================================================================
  val mshr_finalize_st1 = valid_st1 && is_creq_st1 && !is_replay_st1

  val mshr_release_st1 = Wire(Bool())
  if (p.writeback != 0) {
    mshr_release_st1 := is_hit_st1
  } else {
    mshr_release_st1 := is_hit_st1 || (rw_st1 && !mshr_pending_st1)
  }

  val mshr_release_fire = mshr_finalize_st1 && mshr_release_st1 && !pipe_stall

  cacheMshr.io.finalize_valid      := mshr_finalize_st1 && !pipe_stall
  cacheMshr.io.finalize_is_release := mshr_release_st1
  cacheMshr.io.finalize_is_pending := mshr_pending_st1
  cacheMshr.io.finalize_id         := mshr_id_st1
  cacheMshr.io.finalize_previd     := mshr_previd_st1

  // fill triggers MSHR dequeue
  cacheMshr.io.fill_valid := mem_rsp_fire

  // =========================================================================
  // MSHR pending-size counter (approximate)
  // =========================================================================
  val mshr_count = RegInit(0.U((log2Ceil(mshrSize) + 1).W))
  val dequeue_count = PopCount(Cat(replay_fire, mshr_release_fire))
  when (reset.asBool) {
    mshr_count := 0.U
  } .elsewhen (core_req_fire) {
    mshr_count := mshr_count + 1.U - dequeue_count
  } .otherwise {
    mshr_count := mshr_count - dequeue_count
  }
  mshr_empty    := (mshr_count === 0.U)
  mshr_alm_full := (mshr_count >= mshrSize.U)

  cacheFlush.io.bank_empty := !valid_st0 && !valid_st1 && mreq_queue_empty

  // =========================================================================
  // Core response scheduling
  // =========================================================================
  crsp_queue_valid := do_read_st1 && is_hit_st1
  crsp_queue_idx   := req_idx_st1
  crsp_queue_data  := read_data_st1(word_idx_st1)
  crsp_queue_tag   := tag_st1

  // =========================================================================
  // Memory request scheduling
  // =========================================================================
  val mreq_push    = Wire(Bool())
  val mreq_addr    = Wire(UInt(lineAddrWidth.W))
  val mreq_rw      = Wire(Bool())
  val mreq_data    = Wire(UInt(lineWidth.W))
  val mreq_byteen  = Wire(UInt(lineSize.W))
  val mreq_tag     = Wire(UInt(memTagWidth.W))
  val mreq_flags   = Wire(UInt(math.max(1, memFlagsWidth).W))

  val is_fill_or_flush_st1 = is_fill_st1 || (is_flush_st1 && (p.writeback != 0).B)
  val do_fill_or_flush_st1 = valid_st1 && is_fill_or_flush_st1
  val do_writeback_st1     = do_fill_or_flush_st1 && is_dirty_st1
  val evict_addr_st1       = Cat(evict_tag_st1, line_idx_st1)
  val do_lookup_st0        = (valid_st0 && is_creq_st0 && !is_init_st0)  // unused but matches SV

  if (p.writeEnable != 0) {
    if (p.writeback != 0) {
      mreq_push   := ((do_lookup_st1 && !is_hit_st1 && !mshr_pending_st1) ||
                      do_writeback_st1) && !pipe_stall
      mreq_addr   := Mux(is_fill_or_flush_st1, evict_addr_st1, addr_st1)
      mreq_rw     := is_fill_or_flush_st1
      mreq_data   := read_data_st1.asUInt
      mreq_byteen := Mux(is_fill_or_flush_st1, evict_byteen_st1, Fill(lineSize, 1.U))
    } else {
      // write-through: send fill on read miss, send write-through on write hit
      val line_byteen = Wire(Vec(wordsPerLine, UInt(wordSize.W)))
      for (w <- 0 until wordsPerLine) {
        line_byteen(w) := Mux(word_idx_st1 === w.U, byteen_st1, 0.U(wordSize.W))
      }
      mreq_push   := ((do_read_st1 && !is_hit_st1 && !mshr_pending_st1) ||
                      do_write_st1) && !pipe_stall
      mreq_addr   := addr_st1
      mreq_rw     := rw_st1
      mreq_data   := Fill(wordsPerLine, write_word_st1)
      mreq_byteen := Mux(rw_st1, line_byteen.asUInt, Fill(lineSize, 1.U))
    }
  } else {
    // read-only cache: fill on read miss only
    mreq_push   := (do_read_st1 && !is_hit_st1 && !mshr_pending_st1) && !pipe_stall
    mreq_addr   := addr_st1
    mreq_rw     := false.B
    mreq_data   := 0.U
    mreq_byteen := Fill(lineSize, 1.U)
  }

  if (uuidWidth != 0) {
    val uuid = tag_st1(tagWidth - 1, tagWidth - uuidWidth)
    mreq_tag := Cat(uuid, mshr_id_st1)
  } else {
    mreq_tag := mshr_id_st1
  }
  mreq_flags := flags_st1

  mreq_queue_push      := mreq_push
  mreq_queue_in.rw     := mreq_rw
  mreq_queue_in.addr   := mreq_addr
  mreq_queue_in.byteen := mreq_byteen
  mreq_queue_in.data   := mreq_data
  mreq_queue_in.tag    := mreq_tag
  mreq_queue_in.flags  := mreq_flags

  // =========================================================================
  // Memory request output
  // =========================================================================
  io.mem_req_valid  := !mreq_queue_empty
  io.mem_req_rw     := mreq_queue_out.rw
  io.mem_req_addr   := mreq_queue_out.addr
  io.mem_req_byteen := mreq_queue_out.byteen
  io.mem_req_data   := mreq_queue_out.data
  io.mem_req_tag    := mreq_queue_out.tag
  io.mem_req_flags  := mreq_queue_out.flags
}

// Suppress unused warnings by providing a dummy implicit clock connection
// (Chisel modules already get implicit clock/reset)
