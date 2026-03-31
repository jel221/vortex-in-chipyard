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

// Chisel translation of VX_cache_repl.sv
// (includes plru_decoder / plru_encoder helpers)

package vortex

import chisel3._
import chisel3.util._

/**
 * PlruDecoder – combinational PLRU tree update logic.
 *
 * Given a way index, produces the (data, mask) pair to merge into the
 * stored LRU state.  Ported faithfully from the BaseJump STL logic in
 * plru_decoder.
 *
 * LRU_WIDTH = max(1, numWays-1)
 */
class PlruDecoder(numWays: Int) extends Module {
  private val wayIdxBits  = log2Ceil(numWays)
  private val wayIdxWidth = math.max(1, wayIdxBits)
  private val lruWidth    = math.max(1, numWays - 1)

  val io = IO(new Bundle {
    val way_idx  = Input(UInt(wayIdxWidth.W))
    val lru_data = Output(UInt(lruWidth.W))
    val lru_mask = Output(UInt(lruWidth.W))
  })

  if (numWays > 1) {
    val data = Wire(Vec(numWays - 1, Bool()))
    val mask = Wire(Vec(numWays - 1, Bool()))

    for (i <- 0 until numWays - 1) {
      if (i == 0) {
        mask(i) := true.B
      } else if (i % 2 == 1) {
        // odd child: parent = (i-1)/2; left branch
        val parentBit = wayIdxBits - log2Ceil(i + 2) + 1
        val parentMaskBit = if (parentBit < wayIdxBits && parentBit >= 0) io.way_idx(parentBit) else false.B
        mask(i) := mask((i - 1) / 2) && !parentMaskBit
      } else {
        // even child: parent = (i-2)/2; right branch
        val parentBit = wayIdxBits - log2Ceil(i + 2) + 1
        val parentMaskBit = if (parentBit < wayIdxBits && parentBit >= 0) io.way_idx(parentBit) else false.B
        mask(i) := mask((i - 2) / 2) && parentMaskBit
      }
      val dataBit = wayIdxBits - log2Ceil(i + 2)
      data(i) := (if (dataBit >= 0 && dataBit < wayIdxBits) !io.way_idx(dataBit) else false.B)
    }

    io.lru_data := data.asUInt
    io.lru_mask := mask.asUInt
  } else {
    io.lru_data := 0.U
    io.lru_mask := 0.U
  }
}

/**
 * PlruEncoder – combinational PLRU tree read logic.
 *
 * Given the stored LRU state, produces the victim way index.
 * Ported from plru_encoder in VX_cache_repl.sv.
 */
class PlruEncoder(numWays: Int) extends Module {
  private val wayIdxBits  = log2Ceil(numWays)
  private val wayIdxWidth = math.max(1, wayIdxBits)
  private val lruWidth    = math.max(1, numWays - 1)

  val io = IO(new Bundle {
    val lru_in  = Input(UInt(lruWidth.W))
    val way_idx = Output(UInt(wayIdxWidth.W))
  })

  if (numWays > 1) {
    val tmp = Wire(Vec(wayIdxBits, Bool()))

    for (i <- 0 until wayIdxBits) {
      if (i == 0) {
        tmp(wayIdxWidth - 1) := io.lru_in(0)
      } else {
        // Mux2^i inputs from lru_in[(2^i - 1) +: 2^i], selected by tmp[wayIdxBits-1 downto wayIdxBits-i]
        val muxN    = 1 << i
        val base    = muxN - 1
        val selBits = i
        val sel     = tmp.asUInt(wayIdxBits - 1, wayIdxBits - selBits)
        val inputs  = io.lru_in(base + muxN - 1, base)
        tmp(wayIdxBits - 1 - i) := inputs(sel)
      }
    }

    io.way_idx := tmp.asUInt
  } else {
    io.way_idx := 0.U
  }
}

/**
 * CacheRepl – replacement policy unit.
 *
 * Mirrors VX_cache_repl.sv.  Supports RANDOM, FIFO, and PLRU policies.
 * When numWays == 1 the output is always 0.
 */
class CacheRepl(p: CacheParams) extends Module {
  private val lineSelBits  = p.lineSelBits
  private val waySelWidth  = p.waySelWidth
  private val linesPerBank = p.linesPerBank
  private val numWays      = p.numWays

  val io = IO(new Bundle {
    val stall         = Input(Bool())
    val init          = Input(Bool())
    val lookup_valid  = Input(Bool())
    val lookup_hit    = Input(Bool())
    val lookup_line   = Input(UInt(lineSelBits.W))
    val lookup_way    = Input(UInt(waySelWidth.W))
    val repl_valid    = Input(Bool())
    val repl_line     = Input(UInt(lineSelBits.W))
    val repl_way      = Output(UInt(waySelWidth.W))
  })

  if (numWays > 1) {
    p.replPolicy match {
      // ---- PLRU -------------------------------------------------------------
      case CacheReplPolicy.PLRU =>
        val lruWidth = math.max(1, numWays - 1)

        val dec = Module(new PlruDecoder(numWays))
        val enc = Module(new PlruEncoder(numWays))

        val plru_wdata = dec.io.lru_data
        val plru_wmask = dec.io.lru_mask

        dec.io.way_idx := io.lookup_way

        // Dual-port RAM storing LRU state per line.
        // RADDR_REG=1 → registered read address (read latency 1 cycle with address registered).
        val plru_ram = SyncReadMem(linesPerBank, Vec(lruWidth, Bool()))

        val do_write = io.init || (io.lookup_valid && io.lookup_hit)
        val wdata    = Mux(io.init, 0.U(lruWidth.W), plru_wdata).asBools
        val wmask    = Mux(io.init, Fill(lruWidth, 1.U), plru_wmask).asBools
        when (do_write) {
          plru_ram.write(io.lookup_line, VecInit(wdata), wmask)
        }
        // RADDR_REG=1: register the read address then do a combinational read
        val repl_line_r = RegNext(io.repl_line)
        val plru_rdata  = plru_ram.read(repl_line_r)

        enc.io.lru_in  := plru_rdata.asUInt
        io.repl_way    := enc.io.way_idx

      // ---- FIFO -------------------------------------------------------------
      case CacheReplPolicy.FIFO =>
        // Registered-address read: latch repl_line one cycle before reading.
        val fifo_ram = SyncReadMem(linesPerBank, UInt(waySelWidth.W))

        val repl_line_r = RegNext(io.repl_line)
        val fifo_rdata  = fifo_ram.read(repl_line_r)
        val fifo_wdata  = fifo_rdata +& 1.U  // wraps naturally at waySelWidth bits

        when (io.init || io.repl_valid) {
          val wd = Mux(io.init, 0.U(waySelWidth.W), fifo_wdata(waySelWidth - 1, 0))
          fifo_ram.write(io.repl_line, wd)
        }

        io.repl_way := fifo_rdata

      // ---- RANDOM (default) -------------------------------------------------
      case _ =>
        val victim_idx = RegInit(0.U(waySelWidth.W))
        when (!io.stall) {
          victim_idx := victim_idx + 1.U
        }
        io.repl_way := victim_idx
    }
  } else {
    io.repl_way := 0.U
  }
}
