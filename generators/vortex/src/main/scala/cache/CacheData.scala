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

// Chisel translation of VX_cache_data.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * CacheData – per-bank data RAM array.
 *
 * Mirrors VX_cache_data.sv exactly.
 *
 * Underlying SRAMs are synchronous-read (registered-output) with byte-enable
 * write, matching VX_sp_ram OUT_REG=1, RDW_MODE="R".
 *
 * Port names are kept identical to the SV original.
 */
class CacheData(p: CacheParams) extends Module {
  private val wordsPerLine = p.wordsPerLine
  private val lineSelBits  = p.lineSelBits
  private val wordSelWidth = p.wordSelWidth
  private val waySelWidth  = p.waySelWidth
  private val wordWidth    = p.wordWidth
  private val lineWidth    = p.lineWidth
  private val linesPerBank = p.linesPerBank
  private val lineSize     = p.lineSize   // bytes
  private val wordSize     = p.wordSize   // bytes
  private val numWays      = p.numWays

  val io = IO(new Bundle {
    // control
    val init         = Input(Bool())
    val fill         = Input(Bool())
    val flush        = Input(Bool())
    val read         = Input(Bool())
    val write        = Input(Bool())
    // addressing
    val line_idx     = Input(UInt(lineSelBits.W))
    val evict_way    = Input(UInt(waySelWidth.W))
    val tag_matches  = Input(UInt(numWays.W))
    // fill data: wordsPerLine × wordWidth (flat, matches data_st0 in bank)
    val fill_data    = Input(Vec(wordsPerLine, UInt(wordWidth.W)))
    // write word
    val write_word   = Input(UInt(wordWidth.W))
    val write_byteen = Input(UInt(wordSize.W))
    val word_idx     = Input(UInt(wordSelWidth.W))
    // read (registered, valid one cycle after read issued)
    val way_idx_r    = Input(UInt(waySelWidth.W))
    val read_data    = Output(Vec(wordsPerLine, UInt(wordWidth.W)))
    val evict_byteen = Output(UInt(lineSize.W))
  })

  // ---- write-mask per word -------------------------------------------------
  // SV:
  //   wire word_en = (CS_WORDS_PER_LINE == 1) || (word_idx == i);
  //   assign write_mask[i] = write_byteen & {WORD_SIZE{word_en}};
  val write_mask = Wire(Vec(wordsPerLine, UInt(wordSize.W)))
  for (i <- 0 until wordsPerLine) {
    val word_en = if (wordsPerLine == 1) true.B else (io.word_idx === i.U)
    write_mask(i) := io.write_byteen & Fill(wordSize, word_en)
  }

  // ---- dirty-byte (byteen) store -------------------------------------------
  // Only instantiated when DIRTY_BYTES != 0
  if (p.dirtyBytes != 0) {
    val byteen_rdata = Wire(Vec(numWays, UInt(lineSize.W)))

    for (i <- 0 until numWays) {
      // Byte-enable RAM: lineSize bits wide, one per byte in the cache line.
      val byteen_ram = SyncReadMem(linesPerBank, Vec(lineSize, Bool()))

      val way_en = if (numWays == 1) true.B else (io.evict_way === i.U)

      // byteen_wdata: all-1 on write, 0 on init/fill/flush
      // SV: wire [LINE_SIZE-1:0] byteen_wdata = {LINE_SIZE{write}};
      val byteen_wdata_bits = VecInit(Seq.fill(lineSize)(io.write))

      // byteen_wren: cleared (0) on init/fill/flush for the whole line,
      //             set per byte on write.
      // SV: wire [LINE_SIZE-1:0] byteen_wren = {LINE_SIZE{init || fill || flush}} | write_mask;
      // write_mask is Vec(wordsPerLine, UInt(wordSize.W)).
      // When flat-cast to lineSize bits: byte i of word j → bit j*wordSize+i.
      val fill_flush_mask = Fill(lineSize, io.init || io.fill || io.flush)
      val write_mask_flat = write_mask.asUInt   // Vec→UInt: word0[byte0..N] ++ word1[...] etc.
      val byteen_wren_bits = (fill_flush_mask | write_mask_flat).asBools

      // byteen_write: which ways get updated
      // SV: ((fill || flush) && ((NUM_WAYS == 1) || (evict_way == i)))
      //  || (write && tag_matches[i])
      //  || init;
      val byteen_write = ((io.fill || io.flush) && way_en) ||
                         (io.write && io.tag_matches(i)) ||
                         io.init

      // byteen_read: fill or flush (to get evict_byteen)
      val byteen_read = io.fill || io.flush

      when (byteen_write) {
        byteen_ram.write(io.line_idx, byteen_wdata_bits, byteen_wren_bits)
      }
      // OUT_REG=1, RDW_MODE="R" → SyncReadMem read with enable
      byteen_rdata(i) := byteen_ram.read(io.line_idx, byteen_read).asUInt
    }

    io.evict_byteen := byteen_rdata(io.way_idx_r)

  } else {
    // DIRTY_BYTES == 0: evict whole line (all bytes valid)
    io.evict_byteen := Fill(lineSize, 1.U)
  }

  // ---- data store ----------------------------------------------------------
  val line_rdata = Wire(Vec(numWays, Vec(wordsPerLine, UInt(wordWidth.W))))

  for (i <- 0 until numWays) {
    // WRENW = WRITE_ENABLE ? LINE_SIZE : 1
    val wrenW = if (p.writeEnable != 0) lineSize else 1

    val line_wdata = Wire(Vec(wordsPerLine, UInt(wordWidth.W)))
    val line_wren  = Wire(UInt(wrenW.W))

    if (p.writeEnable != 0) {
      // SV: line_wdata = fill ? fill_data : {CS_WORDS_PER_LINE{write_word}};
      //     line_wren  = {LINE_SIZE{fill}} | write_mask;
      when (io.fill) {
        line_wdata := io.fill_data
      } .otherwise {
        for (w <- 0 until wordsPerLine) line_wdata(w) := io.write_word
      }
      line_wren := Fill(lineSize, io.fill) | write_mask.asUInt
    } else {
      // read-only cache: only fill data, full-line write enable
      line_wdata := io.fill_data
      line_wren  := 1.U
    }

    val way_en = if (numWays == 1) true.B else (io.evict_way === i.U)

    // SV: line_write = (fill && ((NUM_WAYS == 1) || (evict_way == i)))
    //              || (write && tag_matches[i] && WRITE_ENABLE);
    val line_write = (io.fill && way_en) ||
                     (io.write && io.tag_matches(i) && (p.writeEnable != 0).B)

    // SV: line_read = read || ((fill || flush) && WRITEBACK);
    val line_read = io.read || ((io.fill || io.flush) && (p.writeback != 0).B)

    // Model as byte-enable SRAM: store as Vec(lineSize, UInt(8.W)) with byte-enables.
    if (p.writeEnable != 0) {
      // wrenW == lineSize → byte-granularity enables
      val data_ram = SyncReadMem(linesPerBank, Vec(lineSize, UInt(8.W)))
      // Flatten line_wdata (Vec of words) to Vec of bytes
      // Each word is wordWidth bits = wordSize bytes.
      val flat_wdata = Wire(Vec(lineSize, UInt(8.W)))
      for (w <- 0 until wordsPerLine) {
        for (b <- 0 until wordSize) {
          flat_wdata(w * wordSize + b) := line_wdata(w)(b * 8 + 7, b * 8)
        }
      }
      val wmask = line_wren.asBools
      when (line_write) {
        data_ram.write(io.line_idx, flat_wdata, wmask)
      }
      val rdata_bytes = data_ram.read(io.line_idx, line_read)
      // Reassemble bytes → words
      for (w <- 0 until wordsPerLine) {
        val word_bytes = Wire(Vec(wordSize, UInt(8.W)))
        for (b <- 0 until wordSize) {
          word_bytes(b) := rdata_bytes(w * wordSize + b)
        }
        line_rdata(i)(w) := word_bytes.asUInt
      }
    } else {
      // wrenW == 1 → single write-enable (full line write)
      val data_ram = SyncReadMem(linesPerBank, Vec(wordsPerLine, UInt(wordWidth.W)))
      when (line_write) {
        data_ram.write(io.line_idx, line_wdata)
      }
      line_rdata(i) := data_ram.read(io.line_idx, line_read)
    }
  }

  io.read_data := line_rdata(io.way_idx_r)
}
