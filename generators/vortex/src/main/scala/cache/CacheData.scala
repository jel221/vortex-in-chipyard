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
 * Mirrors VX_cache_data.sv.  The underlying SRAMs are modelled as
 * synchronous-read (registered-output) Mem blocks with byte-enable
 * write, matching the VX_sp_ram OUT_REG=1, RDW_MODE="R" configuration.
 *
 * Port names are kept identical to the SV original.
 */
class CacheData(p: CacheParams) extends Module {
  // Derived sizes
  private val wordsPerLine = p.wordsPerLine
  private val lineSelBits  = p.lineSelBits
  private val wordSelWidth = p.wordSelWidth
  private val waySelWidth  = p.waySelWidth
  private val wordWidth    = p.wordWidth
  private val lineWidth    = p.lineWidth
  private val linesPerBank = p.linesPerBank
  private val lineSize     = p.lineSize
  private val wordSize     = p.wordSize
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
    // fill data: wordsPerLine × wordWidth
    val fill_data    = Input(Vec(wordsPerLine, UInt(wordWidth.W)))
    // write word
    val write_word   = Input(UInt(wordWidth.W))
    val write_byteen = Input(UInt(wordSize.W))
    val word_idx     = Input(UInt(wordSelWidth.W))
    // read (registered, valid one cycle after read)
    val way_idx_r    = Input(UInt(waySelWidth.W))
    val read_data    = Output(Vec(wordsPerLine, UInt(wordWidth.W)))
    val evict_byteen = Output(UInt(lineSize.W))
  })

  // ---- write-mask per word -------------------------------------------------
  // For each word slot i in a line:
  //   write_mask[i] = write_byteen if word_idx == i (or wordsPerLine == 1)
  //                   0            otherwise
  val write_mask = Wire(Vec(wordsPerLine, UInt(wordSize.W)))
  for (i <- 0 until wordsPerLine) {
    val word_en = if (wordsPerLine == 1) true.B else (io.word_idx === i.U)
    write_mask(i) := io.write_byteen & Fill(wordSize, word_en.asUInt)
  }

  // ---- dirty-byte (byteen) store -------------------------------------------
  // Only instantiated when DIRTY_BYTES != 0
  if (p.dirtyBytes != 0) {
    val byteen_rdata = Wire(Vec(numWays, UInt(lineSize.W)))
    for (i <- 0 until numWays) {
      // Byte-enable RAM storing whether each byte in the line is dirty.
      // Width = lineSize bits (one bit per byte).
      val byteen_ram = SyncReadMem(linesPerBank, Vec(lineSize, Bool()))

      val way_en        = if (numWays == 1) true.B else (io.evict_way === i.U)
      val byteen_wdata  = Fill(lineSize, io.write.asUInt)  // all-1 on write, 0 otherwise
      // wren: cleared on init/fill/flush, set per write_mask
      val wren_fill_flush = Fill(lineSize, (io.init || io.fill || io.flush).asUInt)
      val wren_write      = write_mask.asTypeOf(UInt(lineSize.W))
      val byteen_wren   = wren_fill_flush | wren_write
      val byteen_write  = ((io.fill || io.flush) && way_en) ||
                          (io.write && io.tag_matches(i)) ||
                          io.init
      val byteen_read   = io.fill || io.flush

      val wmask = byteen_wren.asBools
      val wdata = byteen_wdata.asBools

      when (byteen_write) {
        byteen_ram.write(io.line_idx, VecInit(wdata), wmask)
      }
      byteen_rdata(i) := byteen_ram.read(io.line_idx, byteen_read).asUInt
    }
    io.evict_byteen := byteen_rdata(io.way_idx_r)
  } else {
    // No dirty bytes: evict whole line
    io.evict_byteen := Fill(lineSize, 1.U)
  }

  // ---- data store ----------------------------------------------------------
  val line_rdata = Wire(Vec(numWays, Vec(wordsPerLine, UInt(wordWidth.W))))

  for (i <- 0 until numWays) {
    // Each "cell" in the RAM is lineWidth bits, with byte-enable per byte.
    val wrenW = if (p.writeEnable != 0) lineSize else 1

    // Write data and write-enable
    val line_wdata = Wire(Vec(wordsPerLine, UInt(wordWidth.W)))
    val line_wren  = Wire(UInt(wrenW.W))

    if (p.writeEnable != 0) {
      // fill → use fill_data; otherwise replicate write_word across all slots
      when (io.fill) {
        line_wdata := io.fill_data
      } .otherwise {
        for (w <- 0 until wordsPerLine) line_wdata(w) := io.write_word
      }
      line_wren := (Fill(lineSize, io.fill.asUInt)) | write_mask.asTypeOf(UInt(lineSize.W))
    } else {
      line_wdata := io.fill_data
      line_wren  := 1.U
    }

    val way_en    = if (numWays == 1) true.B else (io.evict_way === i.U)
    val line_write = (io.fill && way_en) ||
                     (io.write && io.tag_matches(i) && (p.writeEnable != 0).B)

    val line_read  = io.read || ((io.fill || io.flush) && (p.writeback != 0).B)

    // Model as a synchronous-read (registered-output) SRAM.
    // We store each line as a flat UInt of lineWidth bits.
    // Byte enables are applied word-granularity (byte = wordSize bytes).
    if (p.writeEnable != 0) {
      val data_ram = SyncReadMem(linesPerBank, Vec(lineSize, UInt(8.W)))
      val flat_wdata = line_wdata.asTypeOf(Vec(lineSize, UInt(8.W)))
      val wmask      = line_wren.asBools
      when (line_write) {
        data_ram.write(io.line_idx, flat_wdata, wmask)
      }
      line_rdata(i) := data_ram.read(io.line_idx, line_read).asTypeOf(Vec(wordsPerLine, UInt(wordWidth.W)))
    } else {
      val data_ram = SyncReadMem(linesPerBank, Vec(wordsPerLine, UInt(wordWidth.W)))
      when (line_write) {
        data_ram.write(io.line_idx, line_wdata)
      }
      line_rdata(i) := data_ram.read(io.line_idx, line_read)
    }
  }

  io.read_data := line_rdata(io.way_idx_r)
}
