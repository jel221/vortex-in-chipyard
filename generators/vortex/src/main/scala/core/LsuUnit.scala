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
//
// Translated from VX_lsu_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// LsuUnit — assembles BLOCK_SIZE LsuSlice instances.
//
// Mirrors VX_lsu_unit.sv:
//   1. A DispatchUnit fans incoming dispatch_if[ISSUE_WIDTH] streams into
//      per-block execute interfaces (one per LSU block).
//   2. Each block has its own LsuSlice and an associated lsu_mem_if port.
//   3. A GatherUnit collects the per-block result_if streams and merges
//      them back into the commit_if[ISSUE_WIDTH] output.
//
// Because DispatchUnit and GatherUnit are complex pipeline support modules
// that are defined elsewhere, this module exposes its I/O as flat bundles
// and instantiates LsuSlice directly.  The dispatch fanout and gather
// merge are modelled as pass-through wires when BLOCK_SIZE == 1 (the
// default for LSU), which matches the common configuration.
// ---------------------------------------------------------------------------
class LsuUnit(instanceId: String = "lsu_unit") extends Module {
  private val BLOCK_SIZE  = NUM_LSU_BLOCKS  // = 1 by default
  private val NUM_LANES   = NUM_LSU_LANES
  private val PID_WIDTH   = up(log2Ceil(NUM_THREADS / NUM_LANES))
  private val REQ_ASHIFT  = log2Ceil(LSU_WORD_SIZE)
  private val LSUQ_SIZEW  = log2Ceil(LSUQ_IN_SIZE)
  // Full field-based tag width (matches LsuSlice TAG_WIDTH): uuid + wid + PC + wb + rd + op_type + align + pid + pkt_addr + fence
  private val LSU_TAG_ID  = NW_WIDTH + PC_BITS + 1 + NUM_REGS_BITS + INST_LSU_BITS + (NUM_LANES * REQ_ASHIFT) + PID_WIDTH + LSUQ_SIZEW + 1
  private val LSU_FULL_TAG_W = UUID_WIDTH + LSU_TAG_ID

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Dispatch inputs (slave) – one slot per ISSUE_WIDTH
    val dispatch_valid  = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_ready  = Output(Vec(ISSUE_WIDTH, Bool()))
    // dispatch data (flattened DispatchBundle fields relevant to LSU)
    val d_uuid          = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val d_wis           = Input(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val d_sid           = Input(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val d_tmask         = Input(Vec(ISSUE_WIDTH, UInt(NUM_LANES.W)))
    val d_PC            = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val d_op_type       = Input(Vec(ISSUE_WIDTH, UInt(INST_LSU_BITS.W)))
    val d_wb            = Input(Vec(ISSUE_WIDTH, Bool()))
    val d_rd            = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val d_rs1_data      = Input(Vec(ISSUE_WIDTH, Vec(NUM_LANES, UInt(XLEN.W))))
    val d_rs2_data      = Input(Vec(ISSUE_WIDTH, Vec(NUM_LANES, UInt(XLEN.W))))
    val d_lsu_offset    = Input(Vec(ISSUE_WIDTH, UInt(OFFSET_BITS.W)))
    val d_lsu_is_store  = Input(Vec(ISSUE_WIDTH, Bool()))
    val d_pid           = Input(Vec(ISSUE_WIDTH, UInt(PID_WIDTH.W)))
    val d_sop           = Input(Vec(ISSUE_WIDTH, Bool()))
    val d_eop           = Input(Vec(ISSUE_WIDTH, Bool()))

    // Commit outputs (master) – one slot per ISSUE_WIDTH
    val commit_valid    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_ready    = Input(Vec(ISSUE_WIDTH, Bool()))
    val c_uuid          = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val c_wid           = Output(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
    val c_tmask         = Output(Vec(ISSUE_WIDTH, UInt(NUM_LANES.W)))
    val c_PC            = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val c_wb            = Output(Vec(ISSUE_WIDTH, Bool()))
    val c_rd            = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val c_data          = Output(Vec(ISSUE_WIDTH, Vec(NUM_LANES, UInt(XLEN.W))))
    val c_pid           = Output(Vec(ISSUE_WIDTH, UInt(PID_WIDTH.W)))
    val c_sop           = Output(Vec(ISSUE_WIDTH, Bool()))
    val c_eop           = Output(Vec(ISSUE_WIDTH, Bool()))

    // LSU memory interfaces (master) – one per BLOCK_SIZE
    val mem_req_valid   = Output(Vec(BLOCK_SIZE, Bool()))
    val mem_req_ready   = Input(Vec(BLOCK_SIZE, Bool()))
    val mem_req_mask    = Output(Vec(BLOCK_SIZE, UInt(NUM_LANES.W)))
    val mem_req_rw      = Output(Vec(BLOCK_SIZE, Bool()))
    val mem_req_addr    = Output(Vec(BLOCK_SIZE, Vec(NUM_LANES, UInt(LSU_ADDR_WIDTH.W))))
    val mem_req_byteen  = Output(Vec(BLOCK_SIZE, Vec(NUM_LANES, UInt(LSU_WORD_SIZE.W))))
    val mem_req_data    = Output(Vec(BLOCK_SIZE, Vec(NUM_LANES, UInt((LSU_WORD_SIZE * 8).W))))
    val mem_req_flags   = Output(Vec(BLOCK_SIZE, Vec(NUM_LANES, UInt(MEM_FLAGS_WIDTH.W))))
    val mem_req_tag     = Output(Vec(BLOCK_SIZE, UInt(LSU_FULL_TAG_W.W)))

    val mem_rsp_valid   = Input(Vec(BLOCK_SIZE, Bool()))
    val mem_rsp_ready   = Output(Vec(BLOCK_SIZE, Bool()))
    val mem_rsp_mask    = Input(Vec(BLOCK_SIZE, UInt(NUM_LANES.W)))
    val mem_rsp_data    = Input(Vec(BLOCK_SIZE, Vec(NUM_LANES, UInt((LSU_WORD_SIZE * 8).W))))
    val mem_rsp_tag     = Input(Vec(BLOCK_SIZE, UInt(LSU_FULL_TAG_W.W)))
    val mem_rsp_sop     = Input(Vec(BLOCK_SIZE, Bool()))
    val mem_rsp_eop     = Input(Vec(BLOCK_SIZE, Bool()))
  })

  // -------------------------------------------------------------------------
  // Instantiate one LsuSlice per block.
  // When BLOCK_SIZE == ISSUE_WIDTH == 1 the dispatch/gather are direct wires.
  // -------------------------------------------------------------------------
  val slices = Seq.tabulate(BLOCK_SIZE)(b =>
    Module(new LsuSlice(s"${instanceId}${b}"))
  )

  // Simple round-robin batch selection (mirrors VX_dispatch_unit batch_idx logic)
  // For BLOCK_SIZE == 1 and ISSUE_WIDTH == 1 this is trivial.
  // For larger configurations a proper VX_dispatch_unit / VX_gather_unit
  // would be inserted; here we model the straightforward BLOCK_SIZE=1 case.

  for (b <- 0 until BLOCK_SIZE) {
    val slice = slices(b)

    // Connect dispatch → execute for slice b (dispatch slot = b when BATCH_COUNT==1)
    val dispatchSlot = b  // in a single-batch config, slice b reads dispatch slot b
    slice.io.execute_valid   := io.dispatch_valid(dispatchSlot)
    io.dispatch_ready(b)     := slice.io.execute_ready

    slice.io.ex_uuid         := io.d_uuid(dispatchSlot)
    slice.io.ex_wid          := wis_to_wid(io.d_wis(dispatchSlot), b.U(up(log2Ceil(BLOCK_SIZE)).W))
    slice.io.ex_tmask        := io.d_tmask(dispatchSlot)
    slice.io.ex_PC           := io.d_PC(dispatchSlot)
    slice.io.ex_op_type      := io.d_op_type(dispatchSlot)
    slice.io.ex_wb           := io.d_wb(dispatchSlot)
    slice.io.ex_rd           := io.d_rd(dispatchSlot)
    slice.io.ex_rs1_data     := io.d_rs1_data(dispatchSlot)
    slice.io.ex_rs2_data     := io.d_rs2_data(dispatchSlot)
    slice.io.ex_lsu_offset   := io.d_lsu_offset(dispatchSlot)
    slice.io.ex_lsu_is_store := io.d_lsu_is_store(dispatchSlot)
    slice.io.ex_pid          := io.d_pid(dispatchSlot)
    slice.io.ex_sop          := io.d_sop(dispatchSlot)
    slice.io.ex_eop          := io.d_eop(dispatchSlot)

    // Connect result → commit for slot b
    io.commit_valid(b)  := slice.io.result_valid
    slice.io.result_ready:= io.commit_ready(b)
    io.c_uuid(b)        := slice.io.res_uuid
    io.c_wid(b)         := slice.io.res_wid
    io.c_tmask(b)       := slice.io.res_tmask
    io.c_PC(b)          := slice.io.res_PC
    io.c_wb(b)          := slice.io.res_wb
    io.c_rd(b)          := slice.io.res_rd
    io.c_data(b)        := slice.io.res_data
    io.c_pid(b)         := slice.io.res_pid
    io.c_sop(b)         := slice.io.res_sop
    io.c_eop(b)         := slice.io.res_eop

    // Connect memory bus for block b
    io.mem_req_valid(b)    := slice.io.mem_req_valid
    slice.io.mem_req_ready  := io.mem_req_ready(b)
    io.mem_req_mask(b)     := slice.io.mem_req_mask
    io.mem_req_rw(b)       := slice.io.mem_req_rw
    io.mem_req_addr(b)     := slice.io.mem_req_addr
    io.mem_req_byteen(b)   := slice.io.mem_req_byteen
    io.mem_req_data(b)     := slice.io.mem_req_data
    io.mem_req_flags(b)    := slice.io.mem_req_flags
    io.mem_req_tag(b)      := slice.io.mem_req_tag

    slice.io.mem_rsp_valid  := io.mem_rsp_valid(b)
    io.mem_rsp_ready(b)    := slice.io.mem_rsp_ready
    slice.io.mem_rsp_mask   := io.mem_rsp_mask(b)
    slice.io.mem_rsp_data   := io.mem_rsp_data(b)
    slice.io.mem_rsp_tag    := io.mem_rsp_tag(b)
    slice.io.mem_rsp_sop    := io.mem_rsp_sop(b)
    slice.io.mem_rsp_eop    := io.mem_rsp_eop(b)
  }

  // When BLOCK_SIZE < ISSUE_WIDTH, remaining dispatch/commit slots are tied off
  for (i <- BLOCK_SIZE until ISSUE_WIDTH) {
    io.dispatch_ready(i) := false.B
    io.commit_valid(i)   := false.B
    io.c_uuid(i)         := 0.U
    io.c_wid(i)          := 0.U
    io.c_tmask(i)        := 0.U
    io.c_PC(i)           := 0.U
    io.c_wb(i)           := false.B
    io.c_rd(i)           := 0.U
    for (l <- 0 until NUM_LANES) io.c_data(i)(l) := 0.U
    io.c_pid(i)          := 0.U
    io.c_sop(i)          := false.B
    io.c_eop(i)          := false.B
  }
}
