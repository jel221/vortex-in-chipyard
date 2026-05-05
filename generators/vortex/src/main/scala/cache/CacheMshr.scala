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

// Chisel translation of VX_cache_mshr.sv

package vortex

import chisel3._
import chisel3.util._

// MSHR data payload.  Field order matches the Cat pack used in VX_cache_bank.sv:
//   MSB: wsel | byteen | data | tag | idx :LSB
// (Chisel serialises Bundle fields LSB-first, so first field = LSB.)
class MshrDataBundle(
  reqSelWidth:  Int,
  tagWidth:     Int,
  wordWidth:    Int,
  wordSize:     Int,
  wordSelWidth: Int
) extends Bundle {
  val idx    = UInt(reqSelWidth.W)
  val tag    = UInt(tagWidth.W)
  val data   = UInt(wordWidth.W)
  val byteen = UInt(wordSize.W)
  val wsel   = UInt(wordSelWidth.W)
}

/**
 * CacheMshr – Miss Status Holding Register for the Vortex cache bank pipeline.
 *
 * Faithfully mirrors VX_cache_mshr.sv.
 *
 * Key operations:
 *   fill     – a memory fill response arrives; starts dequeue of all MSHR
 *              entries chained to fill_id.
 *   dequeue  – replay the next pending entry; follows the next_index chain.
 *   allocate – reserve a slot for an incoming miss; returns the allocated id,
 *              whether there is already a pending entry for the same address
 *              (allocate_pending), and the id of the tail of the chain
 *              (allocate_previd).
 *   finalize – either release the allocated slot (hit) or persist it (miss,
 *              sets next_table link in the chain).
 *
 * Important implementation note from SV:
 *   The combinational always block computes *_n signals from the REGISTERED
 *   values (*_table, dequeue_val, etc.) — it does NOT accumulate updates
 *   within the block. When both dequeue_fire and finalize_valid modify
 *   valid_table_n, each modification starts from the same registered
 *   valid_table value (not from each other's output). This matches the SV
 *   behaviour exactly.
 *
 */
class CacheMshr(
  mshrSize:      Int,
  lineAddrWidth: Int,
  reqSelWidth:   Int,
  tagWidth:      Int,
  wordWidth:     Int,
  wordSize:      Int,
  wordSelWidth:  Int,
  writeback:     Int = 0
) extends Module {
  private val mshrAddrWidth = math.max(1, log2Ceil(mshrSize))

  val io = IO(new Bundle {
    // memory fill
    val fill_valid    = Input(Bool())
    val fill_id       = Input(UInt(mshrAddrWidth.W))
    val fill_addr     = Output(UInt(lineAddrWidth.W))

    // dequeue (replay)
    val dequeue_valid = Output(Bool())
    val dequeue_addr  = Output(UInt(lineAddrWidth.W))
    val dequeue_rw    = Output(Bool())
    val dequeue_data  = Output(new MshrDataBundle(reqSelWidth, tagWidth, wordWidth, wordSize, wordSelWidth))
    val dequeue_id    = Output(UInt(mshrAddrWidth.W))
    val dequeue_ready = Input(Bool())

    // allocate
    val allocate_valid   = Input(Bool())
    val allocate_addr    = Input(UInt(lineAddrWidth.W))
    val allocate_rw      = Input(Bool())
    val allocate_data    = Input(new MshrDataBundle(reqSelWidth, tagWidth, wordWidth, wordSize, wordSelWidth))
    val allocate_id      = Output(UInt(mshrAddrWidth.W))
    val allocate_pending = Output(Bool())
    val allocate_previd  = Output(UInt(mshrAddrWidth.W))
    val allocate_ready   = Output(Bool())

    // finalize
    val finalize_valid      = Input(Bool())
    val finalize_is_release = Input(Bool())
    val finalize_is_pending = Input(Bool())
    val finalize_previd     = Input(UInt(mshrAddrWidth.W))
    val finalize_id         = Input(UInt(mshrAddrWidth.W))
  })

  // ---- next-value wires (declared before registers so RegNext can reference them)
  val valid_table_n = Wire(UInt(mshrSize.W))
  val next_table_x  = Wire(UInt(mshrSize.W))
  val next_table_n  = Wire(UInt(mshrSize.W))
  val dequeue_val_n = Wire(Bool())
  val dequeue_id_n  = Wire(UInt(mshrAddrWidth.W))

  val allocate_id_n  = PriorityEncoder(~valid_table_n)
  val allocate_rdy_n = (~valid_table_n).orR

  // ---- state registers
  val addr_table    = Reg(Vec(mshrSize, UInt(lineAddrWidth.W)))
  val next_index    = Reg(Vec(mshrSize, UInt(mshrAddrWidth.W)))
  val valid_table   = RegNext(valid_table_n,  0.U(mshrSize.W))
  val next_table    = RegNext(next_table_n,   0.U(mshrSize.W))
  val write_table   = RegInit(0.U(mshrSize.W))
  val allocate_rdy  = RegNext(allocate_rdy_n, false.B)
  val allocate_id_r = RegNext(allocate_id_n,  0.U(mshrAddrWidth.W))
  val dequeue_val   = RegNext(dequeue_val_n,  false.B)
  val dequeue_id_r  = RegNext(dequeue_id_n,   0.U(mshrAddrWidth.W))

  // ---- defaults: next = current (hold state)
  valid_table_n := valid_table
  next_table_x  := next_table
  next_table_n  := next_table_x
  dequeue_val_n := dequeue_val
  dequeue_id_n  := dequeue_id_r

  // ---- address-match vector ------------------------------------------------
  val addr_matches = VecInit((0 until mshrSize).map { i =>
    valid_table(i) && (addr_table(i) === io.allocate_addr)
  }).asUInt

  val tail_candidates = addr_matches & ~next_table_x
  val prev_idx        = PriorityEncoder(tail_candidates)

  val allocate_fire = io.allocate_valid && allocate_rdy
  val dequeue_fire  = dequeue_val && io.dequeue_ready

  // SV combinational block order:
  // 1) fill_valid → set dequeue_val_n, dequeue_id_n
  when (io.fill_valid) {
    dequeue_val_n := true.B
    dequeue_id_n  := io.fill_id
  }

  // 2) dequeue_fire → clear valid bit, advance chain
  when (dequeue_fire) {
    valid_table_n := valid_table & ~UIntToOH(io.dequeue_id, mshrSize)
    when (next_table(io.dequeue_id)) {
      dequeue_id_n := next_index(io.dequeue_id)
    } .elsewhen (io.finalize_valid && io.finalize_is_pending &&
                 (io.finalize_previd === io.dequeue_id)) {
      dequeue_id_n := io.finalize_id
    } .otherwise {
      dequeue_val_n := false.B
    }
  }

  // 3) finalize_valid
  when (io.finalize_valid) {
    when (io.finalize_is_release) {
      valid_table_n := valid_table & ~UIntToOH(io.finalize_id, mshrSize)
    }
    // next_table_x is set unconditionally for finalize_is_pending to reduce
    // timing; wrong updates are cleared by allocate_fire below.
    when (io.finalize_is_pending) {
      next_table_x := next_table | UIntToOH(io.finalize_previd, mshrSize)
    }
  }

  // 4) allocate_fire → override next_table_n and valid_table_n
  when (allocate_fire) {
    valid_table_n := valid_table | UIntToOH(allocate_id_r, mshrSize)
    next_table_n  := next_table_x & ~UIntToOH(allocate_id_r, mshrSize)
  }

  when (allocate_fire) {
    addr_table(allocate_id_r) := io.allocate_addr
    write_table := Mux(io.allocate_rw,
                       write_table | UIntToOH(allocate_id_r, mshrSize),
                       write_table & ~UIntToOH(allocate_id_r, mshrSize))
  }

  when (io.finalize_valid && io.finalize_is_pending) {
    next_index(io.finalize_previd) := io.finalize_id
  }

  // ---- MSHR data RAM -------------------------------------------------------
  // VX_dp_ram with RDW_MODE="R", RADDR_REG=1:
  //   write: when allocate_valid, waddr = allocate_id_r, wdata = allocate_data
  //   read:  always (read=1'b1),  raddr = dequeue_id_r (already registered)
  //
  // Because RADDR_REG=1 means the read address is registered into the RAM,
  // and dequeue_id_r is already a Reg, we use dequeue_id_r directly as the
  // read address for a SyncReadMem (which registers the address internally).
  val mshr_ram = Mem(mshrSize, new MshrDataBundle(reqSelWidth, tagWidth, wordWidth, wordSize, wordSelWidth))
  when (io.allocate_valid) {
    mshr_ram.write(allocate_id_r, io.allocate_data)
  }
  // read=1'b1 means always reading; SyncReadMem with no enable reads every cycle.
  val dequeue_data_out = mshr_ram.read(dequeue_id_r)

  // ---- outputs -------------------------------------------------------------
  io.fill_addr := addr_table(io.fill_id)

  io.dequeue_valid := dequeue_val
  io.dequeue_addr  := addr_table(dequeue_id_r)
  io.dequeue_rw    := write_table(dequeue_id_r)
  io.dequeue_data  := dequeue_data_out
  io.dequeue_id    := dequeue_id_r

  io.allocate_ready   := allocate_rdy
  io.allocate_id      := allocate_id_r
  io.allocate_previd  := prev_idx

  // SV: if (WRITEBACK) allocate_pending = |addr_matches
  //     else           allocate_pending = |(addr_matches & ~write_table)
  if (writeback != 0) {
    io.allocate_pending := addr_matches.orR
  } else {
    io.allocate_pending := (addr_matches & ~write_table).orR
  }
}
