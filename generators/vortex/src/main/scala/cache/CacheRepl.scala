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
 * Ported faithfully from the BaseJump STL logic in plru_decoder.
 *
 * LRU_WIDTH = max(1, numWays-1)
 *
 * SV loops:
 *   i == 0:      mask[i] = 1
 *   i odd:       mask[i] = mask[(i-1)/2] & ~way_idx[WAY_IDX_BITS - clog2(i+2) + 1]
 *   i even (>0): mask[i] = mask[(i-2)/2] &  way_idx[WAY_IDX_BITS - clog2(i+2) + 1]
 *   data[i] = ~way_idx[WAY_IDX_BITS - clog2(i+2)]
 */
class PlruDecoder(numWays: Int) extends Module {
  private val wayIdxBits  = log2Ceil(numWays)            // $clog2(NUM_WAYS)
  private val wayIdxWidth = math.max(1, wayIdxBits)      // UP(WAY_IDX_BITS)
  private val lruWidth    = math.max(1, numWays - 1)     // UP(NUM_WAYS-1)

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
        // odd: parent = (i-1)/2; mask bit = ~way_idx[WAY_IDX_BITS - clog2(i+2) + 1]
        val parentBit = wayIdxBits - log2Ceil(i + 2) + 1
        val sel = if (parentBit >= 0 && parentBit < wayIdxBits) !io.way_idx(parentBit)
                  else true.B
        mask(i) := mask((i - 1) / 2) && sel
      } else {
        // even (> 0): parent = (i-2)/2; mask bit = way_idx[WAY_IDX_BITS - clog2(i+2) + 1]
        val parentBit = wayIdxBits - log2Ceil(i + 2) + 1
        val sel = if (parentBit >= 0 && parentBit < wayIdxBits) io.way_idx(parentBit)
                  else false.B
        mask(i) := mask((i - 2) / 2) && sel
      }
      // data[i] = ~way_idx[WAY_IDX_BITS - clog2(i+2)]
      val dataBit = wayIdxBits - log2Ceil(i + 2)
      if (dataBit >= 0 && dataBit < wayIdxBits) {
        data(i) :=  !io.way_idx(dataBit)
      } else { 
        data(i) := true.B
      }
    }

    io.lru_data := data.asUInt
    io.lru_mask := mask.asUInt
  } else {
    io.lru_data := 0.U
    io.lru_mask := 0.U
  }
}

/**
 * PlruEncoder – combinational PLRU tree read (victim way selection).
 *
 * SV plru_encoder:
 *   i == 0:  tmp[WAY_IDX_WIDTH-1] = lru_in[0]
 *   i > 0:   tmp[WAY_IDX_BITS-1-i] = MUX2^i( lru_in[(2^i-1) +: 2^i],
 *                                             tmp[WAY_IDX_BITS-1 : WAY_IDX_BITS-i] )
 * way_idx = tmp
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
        // SV: tmp[WAY_IDX_WIDTH-1] = lru_in[0]
        tmp(wayIdxWidth - 1) := io.lru_in(0)
      } else {
        // SV: VX_mux #(.N(2^i)) mux(
        //       .data_in  (lru_in[(2^i - 1) +: 2^i]),
        //       .sel_in   (tmp[WAY_IDX_BITS-1 downto WAY_IDX_BITS-i]),
        //       .data_out (tmp[WAY_IDX_BITS-1-i])
        //     );
        // val muxN   = 1 << i               // 2^i inputs
        // val base   = muxN - 1             // first index in lru_in
        // val inputs = io.lru_in(base + muxN - 1, base)   // [base +: muxN]
        // // selector: tmp bits [WAY_IDX_BITS-1 downto WAY_IDX_BITS-i]  (i bits, MSB first)
        // // In Vec indexing, tmp(wayIdxBits-1) is MSB, tmp(wayIdxBits-1-(i-1)) is LSB of sel.
        // val selBits = (wayIdxBits - 1 downto wayIdxBits - i).map(tmp(_))
        // val sel     = selBits.reverse.foldLeft(false.B)((acc, b) => Cat(acc, b).apply(0))
        // // Build UInt selector from selBits vector (MSB is tmp(wayIdxBits-1))
        // val selUInt = VecInit((wayIdxBits - i until wayIdxBits).map(tmp(_))).asUInt
        // tmp(wayIdxBits - 1 - i) := inputs(selUInt)
        val muxN   = 1 << i                                     // 2^i inputs
        val base   = muxN - 1                                   // first index in lru_in
        val inputs = io.lru_in(base + muxN - 1, base)           // Equivalent to +: indexing

        // Build UInt selector from tmp bits (MSB is tmp(wayIdxBits-1))
        // VecInit maps the ascending range to indices 0..n. 
        // .asUInt makes index 0 the LSB, matching SV endianness perfectly.
        val selUInt = VecInit((wayIdxBits - i until wayIdxBits).map(tmp(_))).asUInt

        // Infer the Multiplexer and assign to tmp
        tmp(wayIdxBits - 1 - i) := inputs(selUInt)
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
 *
 * SV notes on the FIFO RAM:
 *   VX_sp_ram with RADDR_REG=1 → the read address is registered before
 *   the memory lookup (read port is not directly driven by raddr; instead
 *   raddr is latched on the clock edge preceding the read).
 *   For the FIFO implementation this means:
 *     - write: when (init || repl_valid), addr = repl_line, data = init ? 0 : fifo_rdata+1
 *     - read:  when repl_valid,           addr = repl_line (registered → arrives next cycle)
 *   So fifo_rdata is the value read at the *previous* repl_line.
 *
 * SV notes on PLRU RAM:
 *   VX_dp_ram with RADDR_REG=1 (registered read address).
 *   write: when (init || (lookup_valid && lookup_hit)), waddr = lookup_line
 *   read:  when repl_valid, raddr = repl_line
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

        dec.io.way_idx := io.lookup_way

        val plru_wdata = dec.io.lru_data
        val plru_wmask = dec.io.lru_mask

        // VX_dp_ram with OUT_REG=0 (default), RDW_MODE="R":
        //   RADDR_REG is an unused synthesis hint; OUT_REG=0 means async read.
        //   Simulation: assign rdata = ram[raddr] (with write-hazard bypass).
        //   write: when init || (lookup_valid && lookup_hit), waddr=lookup_line
        //   read:  combinational from raddr=repl_line
        val plru_ram = Mem(linesPerBank, Vec(lruWidth, Bool()))

        val do_write  = io.init || (io.lookup_valid && io.lookup_hit)
        val wdata_vec = Mux(io.init, VecInit(Seq.fill(lruWidth)(false.B)),
                                     VecInit(plru_wdata.asBools))
        val wmask_vec = Mux(io.init, VecInit(Seq.fill(lruWidth)(true.B)),
                                     VecInit(plru_wmask.asBools))
        when (do_write) {
          plru_ram.write(io.lookup_line, wdata_vec, wmask_vec)
        }
        // Async read: combinational output from repl_line
        val plru_rdata = plru_ram(io.repl_line)

        enc.io.lru_in := plru_rdata.asUInt
        io.repl_way   := enc.io.way_idx

      // ---- FIFO -------------------------------------------------------------
      case CacheReplPolicy.FIFO =>
        // VX_sp_ram with OUT_REG=0 (default), RDW_MODE="R":
        //   RADDR_REG is an unused synthesis hint; OUT_REG=0 means async read.
        //   Simulation: assign rdata = ram[addr] (with write-hazard bypass).
        //   write: when (init || repl_valid), addr=repl_line
        //          data = init ? 0 : fifo_rdata + 1
        //   read:  combinational from addr=repl_line
        val fifo_ram   = Mem(linesPerBank, UInt(waySelWidth.W))

        // Async read: combinational output from repl_line
        val fifo_rdata = fifo_ram(io.repl_line)

        // fifo_wdata = fifo_rdata + 1 (wraps at waySelWidth bits)
        val fifo_wdata = (fifo_rdata + 1.U)(waySelWidth - 1, 0)

        when (io.init || io.repl_valid) {
          fifo_ram.write(io.repl_line, Mux(io.init, 0.U(waySelWidth.W), fifo_wdata))
        }

        io.repl_way := fifo_rdata

      // ---- RANDOM (default) -------------------------------------------------
      case _ =>
        // SV: always @(posedge clk) if (reset) victim_idx <= 0; else if (~stall) victim_idx += 1;
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
