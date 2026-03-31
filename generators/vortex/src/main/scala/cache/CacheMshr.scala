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
 * Parameters:
 *   mshrSize  – number of MSHR entries
 *   dataWidth – width of the opaque per-entry data stored in the MSHR RAM
 *               (typically word_sel + byteen + write_word + tag + req_idx)
 *   lineAddrWidth – CS_LINE_ADDR_WIDTH
 *   writeback – 1 = writeback mode; affects allocate_pending
 */
class CacheMshr(
  mshrSize:      Int,
  dataWidth:     Int,
  lineAddrWidth: Int,
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
    val dequeue_data  = Output(UInt(dataWidth.W))
    val dequeue_id    = Output(UInt(mshrAddrWidth.W))
    val dequeue_ready = Input(Bool())

    // allocate
    val allocate_valid   = Input(Bool())
    val allocate_addr    = Input(UInt(lineAddrWidth.W))
    val allocate_rw      = Input(Bool())
    val allocate_data    = Input(UInt(dataWidth.W))
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

  // ---- state tables --------------------------------------------------------
  val addr_table  = Reg(Vec(mshrSize, UInt(lineAddrWidth.W)))
  val next_index  = Reg(Vec(mshrSize, UInt(mshrAddrWidth.W)))
  val valid_table = RegInit(0.U(mshrSize.W))
  val next_table  = RegInit(0.U(mshrSize.W))
  val write_table = RegInit(0.U(mshrSize.W))

  val allocate_rdy   = RegInit(false.B)
  val allocate_id_r  = RegInit(0.U(mshrAddrWidth.W))
  val dequeue_val    = RegInit(false.B)
  val dequeue_id_r   = RegInit(0.U(mshrAddrWidth.W))

  // ---- address-match vector ------------------------------------------------
  val addr_matches = Wire(UInt(mshrSize.W))
  addr_matches := VecInit((0 until mshrSize).map { i =>
    valid_table(i) && (addr_table(i) === io.allocate_addr)
  }).asUInt

  // ---- combinational next-state logic --------------------------------------
  val valid_table_n = WireDefault(valid_table)
  val next_table_x  = WireDefault(next_table)
  val dequeue_val_n = WireDefault(dequeue_val)
  val dequeue_id_n  = WireDefault(dequeue_id_r)

  // Priority encoder: find first free slot → next allocate id
  val free_slots      = Wire(UInt(mshrSize.W))
  free_slots         := ~valid_table_n
  val allocate_id_n   = Wire(UInt(mshrAddrWidth.W))
  val allocate_rdy_n  = Wire(Bool())
  allocate_id_n  := PriorityEncoder(free_slots)
  allocate_rdy_n := free_slots.orR

  // Priority encoder: find tail entry for the same address (has no next)
  val tail_candidates = Wire(UInt(mshrSize.W))
  tail_candidates    := addr_matches & ~next_table_x
  val prev_idx        = PriorityEncoder(tail_candidates)

  val allocate_fire = io.allocate_valid && allocate_rdy
  val dequeue_fire  = dequeue_val && io.dequeue_ready

  // fill: start dequeue chain
  when (io.fill_valid) {
    dequeue_val_n := true.B
    dequeue_id_n  := io.fill_id
  }

  // dequeue: advance or end chain
  when (dequeue_fire) {
    valid_table_n := valid_table & ~UIntToOH(io.dequeue_id)
    when (next_table(io.dequeue_id)) {
      dequeue_id_n := next_index(io.dequeue_id)
    } .elsewhen (io.finalize_valid && io.finalize_is_pending &&
                 (io.finalize_previd === io.dequeue_id)) {
      dequeue_id_n := io.finalize_id
    } .otherwise {
      dequeue_val_n := false.B
    }
  }

  // finalize
  when (io.finalize_valid) {
    when (io.finalize_is_release) {
      valid_table_n := valid_table & ~UIntToOH(io.finalize_id)
    }
    when (io.finalize_is_pending) {
      next_table_x := next_table | UIntToOH(io.finalize_previd)
    }
  }

  // allocate: persist the new entry
  val next_table_n = WireDefault(next_table_x)
  when (allocate_fire) {
    valid_table_n := valid_table | UIntToOH(allocate_id_r)
    next_table_n  := next_table_x  & ~UIntToOH(allocate_id_r)
  }

  // ---- sequential update ---------------------------------------------------
  when (reset.asBool) {
    valid_table  := 0.U
    allocate_rdy := false.B
    dequeue_val  := false.B
  } .otherwise {
    valid_table  := valid_table_n
    allocate_rdy := allocate_rdy_n
    dequeue_val  := dequeue_val_n
  }

  when (allocate_fire) {
    addr_table(allocate_id_r)  := io.allocate_addr
    write_table                := Mux(io.allocate_rw,
                                    write_table | UIntToOH(allocate_id_r),
                                    write_table & ~UIntToOH(allocate_id_r))
  }

  when (io.finalize_valid && io.finalize_is_pending) {
    next_index(io.finalize_previd) := io.finalize_id
  }

  dequeue_id_r  := dequeue_id_n
  allocate_id_r := allocate_id_n
  next_table    := next_table_n

  // ---- MSHR data RAM (stores the per-entry opaque data blob) ---------------
  // RDW_MODE "R", RADDR_REG=1 → registered read address
  val mshr_ram = SyncReadMem(mshrSize, UInt(dataWidth.W))
  when (io.allocate_valid) {
    mshr_ram.write(allocate_id_r, io.allocate_data)
  }
  val dequeue_data_out = mshr_ram.read(dequeue_id_r)

  // ---- outputs -------------------------------------------------------------
  io.fill_addr      := addr_table(io.fill_id)

  io.dequeue_valid  := dequeue_val
  io.dequeue_addr   := addr_table(dequeue_id_r)
  io.dequeue_rw     := write_table(dequeue_id_r)
  io.dequeue_data   := dequeue_data_out
  io.dequeue_id     := dequeue_id_r

  io.allocate_ready   := allocate_rdy
  io.allocate_id      := allocate_id_r
  io.allocate_previd  := prev_idx

  if (writeback != 0) {
    io.allocate_pending := addr_matches.orR
  } else {
    // exclude write requests in write-through mode
    io.allocate_pending := (addr_matches & ~write_table).orR
  }
}
