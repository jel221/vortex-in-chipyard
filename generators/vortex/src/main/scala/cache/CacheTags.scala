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

// Chisel translation of VX_cache_tags.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * CacheTags – per-bank tag RAM array.
 *
 * Mirrors VX_cache_tags.sv.  One dual-port RAM (write + registered-read) per
 * way.  Read-During-Write hazard is handled identically to the SV version via
 * rdw_fill / rdw_write forwarding registers.
 *
 * tagWidth inside the RAM = 1 (valid) [+ 1 dirty when WRITEBACK] + tagSelBits
 */
class CacheTags(p: CacheParams) extends Module {
  private val lineSelBits  = p.lineSelBits
  private val tagSelBits   = p.tagSelBits
  private val waySelWidth  = p.waySelWidth
  private val linesPerBank = p.linesPerBank
  private val numWays      = p.numWays
  private val tagRamWidth  = p.tagRamWidth  // 1 + writeback + tagSelBits

  val io = IO(new Bundle {
    // control
    val stall       = Input(Bool())
    val init        = Input(Bool())
    val flush       = Input(Bool())
    val fill        = Input(Bool())
    val read        = Input(Bool())
    val write       = Input(Bool())
    // addressing
    val line_idx    = Input(UInt(lineSelBits.W))   // current cycle address (write side)
    val line_idx_n  = Input(UInt(lineSelBits.W))   // next cycle address  (read side)
    val line_tag    = Input(UInt(tagSelBits.W))
    val evict_way   = Input(UInt(waySelWidth.W))
    // outputs (registered – valid the cycle after the read)
    val tag_matches = Output(UInt(numWays.W))
    val evict_dirty = Output(Bool())
    val evict_tag   = Output(UInt(tagSelBits.W))
  })

  val tag_match_vec = Wire(Vec(numWays, Bool()))
  val read_tag      = Wire(Vec(numWays, UInt(tagSelBits.W)))
  val read_valid    = Wire(Vec(numWays, Bool()))
  val read_dirty    = Wire(Vec(numWays, Bool()))

  for (i <- 0 until numWays) {
    val way_en   = if (numWays == 1) true.B else (io.evict_way === i.U)
    val do_init  = io.init
    val do_fill  = io.fill && way_en
    // In write-through: flush all ways; in writeback: flush only evict_way
    val do_flush = io.flush && (if (p.writeback != 0) way_en else true.B)
    val do_write = (p.writeback != 0).B && io.write && tag_match_vec(i)

    val line_write = do_init || do_fill || do_flush || do_write
    val line_valid = io.fill || io.write

    // Compose write data
    val line_wdata = Wire(UInt(tagRamWidth.W))
    if (p.writeback != 0) {
      // {valid, dirty, tag}
      line_wdata := Cat(line_valid, io.write, io.line_tag)
    } else {
      // {valid, tag}
      line_wdata := Cat(line_valid, io.line_tag)
    }

    // Dual-port RAM: write at line_idx, read at line_idx_n
    // RDW_MODE "R" = read-first (no forwarding in RAM itself)
    val tag_ram = SyncReadMem(linesPerBank, UInt(tagRamWidth.W))

    when (line_write) {
      tag_ram.write(io.line_idx, line_wdata)
    }
    val line_rdata = tag_ram.read(io.line_idx_n, !io.stall)

    // ---- RDW (read-during-write) hazard registers --------------------------
    // rdw_fill: plain register of do_fill (SV: `BUFFER(rdw_fill, do_fill))
    // The fill input already has !pipe_stall gating at the bank level;
    // adding !io.stall here would double-gate the stall (Bug #2 fix).
    val rdw_fill = RegNext(do_fill, false.B)
    // rdw_write: asserted the cycle after do_write fired to the same line
    val rdw_write = RegNext(do_write && (io.line_idx === io.line_idx_n), false.B)

    // Unpack rdata
    if (p.writeback != 0) {
      read_tag(i)   := line_rdata(tagSelBits - 1, 0)
      read_dirty(i) := line_rdata(tagSelBits) || rdw_write
      read_valid(i) := line_rdata(tagSelBits + 1)
    } else {
      read_tag(i)   := line_rdata(tagSelBits - 1, 0)
      read_valid(i) := line_rdata(tagSelBits)
      read_dirty(i) := false.B
    }

    tag_match_vec(i) := (read_valid(i) && (io.line_tag === read_tag(i))) || rdw_fill
  }

  io.tag_matches := tag_match_vec.asUInt

  if (p.writeback != 0) {
    io.evict_dirty := read_dirty(io.evict_way)
    io.evict_tag   := read_tag(io.evict_way)
  } else {
    io.evict_dirty := false.B
    io.evict_tag   := 0.U
  }
}
