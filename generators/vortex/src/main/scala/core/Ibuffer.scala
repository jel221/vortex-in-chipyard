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
// Translated from VX_ibuffer.sv and VX_uop_sequencer.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// =============================================================================
// VX_ibuffer
//
// One elastic buffer per warp-in-issue-slot (PER_ISSUE_WARPS entries total).
// Each buffer holds up to IBUF_SIZE decoded instructions and feeds a
// VxUopSequencer that produces the ibuffer_if outputs.
//
// The decode interface delivers one instruction per cycle addressed to a
// specific warp-in-issue-slot (decoded from decode_if.data.wid).
// decode_if.ready is high only when the targeted buffer slot is ready to accept.
//
// L1_ENABLE assumed (ibuf_pop sideband omitted – matches `ifndef L1_ENABLE`).
// PERF_ENABLE: perf_stalls output included (gated by enablePerf constructor arg).
// =============================================================================
class VxIbuffer(
  val issueId:    Int     = 0,
  val enablePerf: Boolean = false
) extends Module {

  private val ibufDataW = (new IbufferBundle).getWidth

  val io = IO(new Bundle {
    // Performance counter (optional)
    val perf_stalls = if (enablePerf) Some(Output(UInt(PERF_CTR_BITS.W))) else None

    // Decode interface (slave)
    val decode_valid   = Input(Bool())
    val decode_ready   = Output(Bool())
    val decode_uuid    = Input(UInt(UUID_WIDTH.W))
    val decode_wid     = Input(UInt(NW_WIDTH.W))
    val decode_tmask   = Input(UInt(NUM_THREADS.W))
    val decode_PC      = Input(UInt(PC_BITS.W))
    val decode_ex_type = Input(UInt(EX_BITS.W))
    val decode_op_type = Input(UInt(INST_OP_BITS.W))
    val decode_op_args = Input(UInt(INST_ARGS_BITS.W))
    val decode_wb      = Input(Bool())
    val decode_used_rs = Input(UInt(NUM_SRC_OPDS.W))
    val decode_rd      = Input(UInt(NUM_REGS_BITS.W))
    val decode_rs1     = Input(UInt(NUM_REGS_BITS.W))
    val decode_rs2     = Input(UInt(NUM_REGS_BITS.W))
    val decode_rs3     = Input(UInt(NUM_REGS_BITS.W))
    // ibuf_pop sideband (L1-disabled path); unused when L1 is enabled
    val decode_ibuf_pop = Output(UInt(NUM_WARPS.W))

    // Ibuffer outputs: PER_ISSUE_WARPS parallel channels
    val ibuf_valid    = Output(Vec(PER_ISSUE_WARPS, Bool()))
    val ibuf_ready    = Input(Vec(PER_ISSUE_WARPS, Bool()))
    val ibuf_uuid     = Output(Vec(PER_ISSUE_WARPS, UInt(UUID_WIDTH.W)))
    val ibuf_tmask    = Output(Vec(PER_ISSUE_WARPS, UInt(NUM_THREADS.W)))
    val ibuf_PC       = Output(Vec(PER_ISSUE_WARPS, UInt(PC_BITS.W)))
    val ibuf_ex_type  = Output(Vec(PER_ISSUE_WARPS, UInt(EX_BITS.W)))
    val ibuf_op_type  = Output(Vec(PER_ISSUE_WARPS, UInt(INST_OP_BITS.W)))
    val ibuf_op_args  = Output(Vec(PER_ISSUE_WARPS, UInt(INST_ARGS_BITS.W)))
    val ibuf_wb       = Output(Vec(PER_ISSUE_WARPS, Bool()))
    val ibuf_used_rs  = Output(Vec(PER_ISSUE_WARPS, UInt(NUM_SRC_OPDS.W)))
    val ibuf_rd       = Output(Vec(PER_ISSUE_WARPS, UInt(NUM_REGS_BITS.W)))
    val ibuf_rs1      = Output(Vec(PER_ISSUE_WARPS, UInt(NUM_REGS_BITS.W)))
    val ibuf_rs2      = Output(Vec(PER_ISSUE_WARPS, UInt(NUM_REGS_BITS.W)))
    val ibuf_rs3      = Output(Vec(PER_ISSUE_WARPS, UInt(NUM_REGS_BITS.W)))
  })

  // -------------------------------------------------------------------------
  // Extract warp-in-slot index from the incoming warp ID
  // -------------------------------------------------------------------------
  val decode_wis = wid_to_wis(io.decode_wid)

  // -------------------------------------------------------------------------
  // Build the packed ibuffer_t input word from decode fields
  // -------------------------------------------------------------------------
  val decode_ibuf = Wire(new IbufferBundle)
  decode_ibuf.uuid    := io.decode_uuid
  decode_ibuf.tmask   := io.decode_tmask
  decode_ibuf.PC      := io.decode_PC
  decode_ibuf.ex_type := io.decode_ex_type
  decode_ibuf.op_type := io.decode_op_type
  decode_ibuf.op_args.bits := io.decode_op_args
  decode_ibuf.wb      := io.decode_wb
  decode_ibuf.used_rs := io.decode_used_rs
  decode_ibuf.rd      := io.decode_rd
  decode_ibuf.rs1     := io.decode_rs1
  decode_ibuf.rs2     := io.decode_rs2
  decode_ibuf.rs3     := io.decode_rs3

  // -------------------------------------------------------------------------
  // Per-warp elastic buffers + uop sequencers
  //
  // SV: VX_elastic_buffer #(.DATAW(OUT_DATAW), .SIZE(IBUF_SIZE), .OUT_REG(1))
  // IBUF_SIZE = 4 (from VortexConfigConstants), OUT_REG=1 → registered output
  // -------------------------------------------------------------------------
  val ibuf_ready_in = Wire(Vec(PER_ISSUE_WARPS, Bool()))

  // decode_if.ready = ibuf_ready_in[decode_wis]
  io.decode_ready := ibuf_ready_in(decode_wis)

  // ibuf_pop is unused (L1 enabled by default)
  io.decode_ibuf_pop := 0.U

  for (w <- 0 until PER_ISSUE_WARPS) {

    // -----------------------------------------------------------------------
    // Elastic buffer: DATAW=ibufDataW, SIZE=IBUF_SIZE, OUT_REG=1
    // -----------------------------------------------------------------------
    val ibuf = Module(new VXElasticBuffer(
      dataw  = ibufDataW,
      size   = IBUF_SIZE,
      outReg = 1
    ))

    // Input side: accept from decode only when this is the targeted warp slot
    ibuf.io.valid_in := io.decode_valid && (decode_wis === w.U(ISSUE_WIS_W.W))
    ibuf.io.data_in  := decode_ibuf.asUInt
    ibuf_ready_in(w) := ibuf.io.ready_in

    // -----------------------------------------------------------------------
    // UopSequencer: with TCU disabled this is a pure pass-through
    // -----------------------------------------------------------------------
    val uopSeq = Module(new VxUopSequencer)
    uopSeq.io.input_valid  := ibuf.io.valid_out
    uopSeq.io.input_data   := ibuf.io.data_out
    ibuf.io.ready_out   := uopSeq.io.input_ready

    // Wire sequencer output to ibuffer_if[w]
    uopSeq.io.output_ready := io.ibuf_ready(w)

    val outData = uopSeq.io.output_data.asTypeOf(new IbufferBundle)
    io.ibuf_valid(w)   := uopSeq.io.output_valid
    io.ibuf_uuid(w)    := outData.uuid
    io.ibuf_tmask(w)   := outData.tmask
    io.ibuf_PC(w)      := outData.PC
    io.ibuf_ex_type(w) := outData.ex_type
    io.ibuf_op_type(w) := outData.op_type
    io.ibuf_op_args(w) := outData.op_args.bits
    io.ibuf_wb(w)      := outData.wb
    io.ibuf_used_rs(w) := outData.used_rs
    io.ibuf_rd(w)      := outData.rd
    io.ibuf_rs1(w)     := outData.rs1
    io.ibuf_rs2(w)     := outData.rs2
    io.ibuf_rs3(w)     := outData.rs3
  }

  // -------------------------------------------------------------------------
  // Performance counter: count decode stall cycles
  // -------------------------------------------------------------------------
  io.perf_stalls.foreach { out =>
    val perf_ibf_stalls = RegInit(0.U(PERF_CTR_BITS.W))
    val decode_stall    = io.decode_valid && !io.decode_ready
    perf_ibf_stalls := perf_ibf_stalls + decode_stall.asUInt
    out := perf_ibf_stalls
  }
}
