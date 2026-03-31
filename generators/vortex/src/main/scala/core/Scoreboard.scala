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
// Translated from VX_scoreboard.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_scoreboard – per-issue-slot in-flight register hazard tracker.
 *
 *  For each warp in the issue slot (PER_ISSUE_WARPS warps), a staging buffer
 *  holds the decoded instruction.  A per-warp bitmask (inuse_regs) tracks
 *  destination registers of in-flight instructions.  Issue is gated until all
 *  source/destination registers are free of hazards.  A cyclic arbiter selects
 *  among hazard-free warps and forwards to the scoreboard output.
 *
 *  Mirrors VX_scoreboard.sv.
 *
 *  @param issueId  ISSUE_ID parameter (informational; not used in logic)
 */
class VxScoreboard(val issueId: Int = 0) extends Module {

  // NUM_OPDS = NUM_SRC_OPDS + 1 = rd + rs1 + rs2 + rs3
  val NUM_OPDS: Int = NUM_SRC_OPDS + 1   // 4

  // Serialized width of IbufferBundle used for staging buffers and arbiters
  val ibufW: Int = (new IbufferBundle).getWidth

  val io = IO(new Bundle {
    // Writeback (push, no back-pressure; eop qualifies register release)
    val writeback_valid = Input(Bool())
    val writeback_data  = Input(new WritebackBundle)

    // Per-warp ibuffer ports (one per warp in this issue slot)
    val ibuffer_valid   = Input(Vec(PER_ISSUE_WARPS, Bool()))
    val ibuffer_data    = Input(Vec(PER_ISSUE_WARPS, new IbufferBundle))
    val ibuffer_ready   = Output(Vec(PER_ISSUE_WARPS, Bool()))

    // Scoreboard output: first hazard-free instruction across all warps
    val scoreboard_valid = Output(Bool())
    val scoreboard_data  = Output(new ScoreboardBundle)
    val scoreboard_ready = Input(Bool())

    // Performance counter: stall cycles
    val perf_stalls      = Output(UInt(PERF_CTR_BITS.W))
  })

  // -------------------------------------------------------------------------
  // Staging buffers: VXPipeBuffer(depth=1) per warp.
  // Mirrors: VX_pipe_buffer #(.DATAW(IN_DATAW)) stanging_buf
  // -------------------------------------------------------------------------

  val staging_valid = Wire(Vec(PER_ISSUE_WARPS, Bool()))
  val staging_data  = Wire(Vec(PER_ISSUE_WARPS, new IbufferBundle))
  val staging_ready = Wire(Vec(PER_ISSUE_WARPS, Bool()))

  for (w <- 0 until PER_ISSUE_WARPS) {
    val buf = Module(new VXPipeBuffer(dataw = ibufW, depth = 1))
    buf.io.valid_in       := io.ibuffer_valid(w)
    buf.io.data_in        := io.ibuffer_data(w).asUInt
    io.ibuffer_ready(w)   := buf.io.ready_in
    staging_valid(w)      := buf.io.valid_out
    staging_data(w)       := buf.io.data_out.asTypeOf(new IbufferBundle)
    buf.io.ready_out      := staging_ready(w)
  }

  // -------------------------------------------------------------------------
  // Per-warp scoreboard logic
  // -------------------------------------------------------------------------

  // operands_ready(w): registered flag — true when staging_data(w) has no
  // in-flight register hazard.
  val operands_ready = Wire(Vec(PER_ISSUE_WARPS, Bool()))

  for (w <- 0 until PER_ISSUE_WARPS) {

    // ------------------------------------------------------------------
    // inuse_regs: NUM_REGS-wide bitmask; bit [rd] set while rd is in flight.
    // Layout: [NUM_REGS-1 : RV_REGS] = float file, [RV_REGS-1 : 0] = int file
    // (matches REG_TYPES * RV_REGS layout of the SV packed array)
    // ------------------------------------------------------------------
    val inuse_regs = RegInit(0.U(NUM_REGS.W))

    // Handshake pulses
    val ibuffer_fire = io.ibuffer_valid(w) && io.ibuffer_ready(w)
    val staging_fire = staging_valid(w)    && staging_ready(w)

    // writeback from this issue slot: eop qualifies the register release
    val writeback_fire =
      io.writeback_valid &&
      (io.writeback_data.wis === w.U(ISSUE_WIS_W.W)) &&
      io.writeback_data.eop

    // ------------------------------------------------------------------
    // Operand register indices (unified NUM_REGS_BITS-wide numbers)
    //   opds(0) = rd, opds(1) = rs1, opds(2) = rs2, opds(3) = rs3
    // ------------------------------------------------------------------
    val ibf_opds = Wire(Vec(NUM_OPDS, UInt(NUM_REGS_BITS.W)))
    val stg_opds = Wire(Vec(NUM_OPDS, UInt(NUM_REGS_BITS.W)))

    ibf_opds(0) := io.ibuffer_data(w).rd
    ibf_opds(1) := io.ibuffer_data(w).rs1
    ibf_opds(2) := io.ibuffer_data(w).rs2
    ibf_opds(3) := io.ibuffer_data(w).rs3

    stg_opds(0) := staging_data(w).rd
    stg_opds(1) := staging_data(w).rs1
    stg_opds(2) := staging_data(w).rs2
    stg_opds(3) := staging_data(w).rs3

    // used_rs: per-operand valid flag
    //   SV: ibf_used_rs = {ibuffer_if[w].data.used_rs, ibuffer_if[w].data.wb}
    //   Bit 0 = wb (rd active), bits[3:1] = used_rs[2:0] (rs1/rs2/rs3)
    val ibf_used = Wire(Vec(NUM_OPDS, Bool()))
    val stg_used = Wire(Vec(NUM_OPDS, Bool()))

    ibf_used(0) := io.ibuffer_data(w).wb
    ibf_used(1) := io.ibuffer_data(w).used_rs(0)
    ibf_used(2) := io.ibuffer_data(w).used_rs(1)
    ibf_used(3) := io.ibuffer_data(w).used_rs(2)

    stg_used(0) := staging_data(w).wb
    stg_used(1) := staging_data(w).used_rs(0)
    stg_used(2) := staging_data(w).used_rs(1)
    stg_used(3) := staging_data(w).used_rs(2)

    // ------------------------------------------------------------------
    // opd_mask(i)(j): one-hot RV_REGS-wide mask for operand i in reg-file j
    //   j=0 → integer, j=1 → float
    //   = (1 << get_reg_idx(opds[i])) masked by (used[i] && type==j)
    // ------------------------------------------------------------------
    val ibf_opd_mask = Wire(Vec(NUM_OPDS, Vec(REG_TYPES, UInt(RV_REGS.W))))
    val stg_opd_mask = Wire(Vec(NUM_OPDS, Vec(REG_TYPES, UInt(RV_REGS.W))))

    for (i <- 0 until NUM_OPDS) {
      for (j <- 0 until REG_TYPES) {
        val ibf_idx    = get_reg_idx(ibf_opds(i))
        val ibf_active = ibf_used(i) && (get_reg_type(ibf_opds(i)) === j.U)
        ibf_opd_mask(i)(j) := Mux(ibf_active, 1.U(RV_REGS.W) << ibf_idx, 0.U(RV_REGS.W))

        val stg_idx    = get_reg_idx(stg_opds(i))
        val stg_active = stg_used(i) && (get_reg_type(stg_opds(i)) === j.U)
        stg_opd_mask(i)(j) := Mux(stg_active, 1.U(RV_REGS.W) << stg_idx, 0.U(RV_REGS.W))
      }
    }

    // ------------------------------------------------------------------
    // Build full NUM_REGS-wide mask from opd_mask for operand 0 (rd):
    //   {stg_opd_mask(0)(REG_TYPES-1), ..., stg_opd_mask(0)(0)}
    // ------------------------------------------------------------------
    val stg_rd_full_mask = Wire(UInt(NUM_REGS.W))
    stg_rd_full_mask := VecInit(
      (0 until REG_TYPES).map(j => stg_opd_mask(0)(j))
    ).asUInt   // Cat of per-type masks: [float_part | int_part] = NUM_REGS bits

    // ------------------------------------------------------------------
    // Combinational next-state for inuse_regs.
    // The SV always_comb priority: writeback clears first, then staging
    // sets (staging set wins if the same bit is both cleared and set).
    // We implement as: (inuse_regs & ~clear_mask) | set_mask
    // ------------------------------------------------------------------

    // Clear mask: clear bit at rd position on writeback
    val wb_clear_mask = Mux(writeback_fire,
                            1.U(NUM_REGS.W) << io.writeback_data.rd,
                            0.U(NUM_REGS.W))

    // Set mask: set rd bits on staging fire when wb=1
    val stg_set_mask = Mux(staging_fire && staging_data(w).wb,
                           stg_rd_full_mask, 0.U(NUM_REGS.W))

    val inuse_regs_n = (inuse_regs & ~wb_clear_mask) | stg_set_mask

    // ------------------------------------------------------------------
    // Conflict detection:
    //   For each register type j:
    //     reg_mask(j) = OR of all ibf (or stg) opd masks for type j
    //     in_use_mask(j) = inuse_regs_n[j*RV_REGS +: RV_REGS] & reg_mask(j)
    //     regs_busy(j)   = (in_use_mask(j) != 0)
    //   operands_ready = ~|regs_busy   (registered)
    //
    //   Note: when ibuffer_fire just fired we use the ibuf-side masks so the
    //   forwarding path catches same-cycle conflicts.
    // ------------------------------------------------------------------
    val regs_busy = Wire(Vec(REG_TYPES, Bool()))

    for (j <- 0 until REG_TYPES) {
      // OR of all operand masks for this reg file type (ibuffer side)
      var ibf_union = 0.U(RV_REGS.W)
      for (i <- 0 until NUM_OPDS) ibf_union = ibf_union | ibf_opd_mask(i)(j)

      // OR of all operand masks for this reg file type (staging side)
      var stg_union = 0.U(RV_REGS.W)
      for (i <- 0 until NUM_OPDS) stg_union = stg_union | stg_opd_mask(i)(j)

      // Forwarding: use ibuf mask when ibuffer_fire just happened
      val reg_mask = Mux(ibuffer_fire, ibf_union, stg_union)

      // Slice of in-use bits for this register type from the updated state
      val inuse_slice = inuse_regs_n(j * RV_REGS + RV_REGS - 1, j * RV_REGS)

      regs_busy(j) := (inuse_slice & reg_mask) =/= 0.U
    }

    // operands_ready is registered (1-cycle latency matching SV always_ff)
    val operands_ready_r = RegNext(!regs_busy.asUInt.orR, init = true.B)
    operands_ready(w) := operands_ready_r

    // ------------------------------------------------------------------
    // Register update
    // ------------------------------------------------------------------
    when (reset.asBool) {
      inuse_regs := 0.U
    } .otherwise {
      inuse_regs := inuse_regs_n
    }
  }

  // -------------------------------------------------------------------------
  // Output arbiter: cyclic (round-robin) arbitration across PER_ISSUE_WARPS
  // Mirrors: VX_stream_arb #(.NUM_INPUTS(PER_ISSUE_WARPS), .DATAW(IN_DATAW),
  //                          .ARBITER("C"), .OUT_BUF(3))
  //
  // Each input is gated by operands_ready.
  // sel_out from the arbiter provides the wis for the winning warp.
  // -------------------------------------------------------------------------

  val arb_valid_in = Wire(Vec(PER_ISSUE_WARPS, Bool()))
  val arb_data_in  = Wire(Vec(PER_ISSUE_WARPS, UInt(ibufW.W)))
  val arb_ready_in = Wire(Vec(PER_ISSUE_WARPS, Bool()))

  for (w <- 0 until PER_ISSUE_WARPS) {
    arb_valid_in(w) := staging_valid(w) && operands_ready(w)
    arb_data_in(w)  := staging_data(w).asUInt
    // staging_ready: downstream grants AND no hazard
    staging_ready(w) := arb_ready_in(w) && operands_ready(w)
  }

  val outArb = Module(new VxStreamArb(
    numInputs  = PER_ISSUE_WARPS,
    numOutputs = 1,
    dataw      = ibufW,
    arbiter    = "C",
    outBuf     = 3
  ))

  outArb.io.validIn     := arb_valid_in
  outArb.io.dataIn      := arb_data_in
  arb_ready_in          := outArb.io.readyIn
  outArb.io.readyOut(0) := io.scoreboard_ready

  io.scoreboard_valid := outArb.io.validOut(0)

  // Unpack the winning ibuffer entry and attach the wis (the arbiter's sel index)
  val selData = outArb.io.dataOut(0).asTypeOf(new IbufferBundle)
  val selWis  = outArb.io.selOut(0)

  io.scoreboard_data.uuid    := selData.uuid
  io.scoreboard_data.wis     := selWis(ISSUE_WIS_W - 1, 0)
  io.scoreboard_data.tmask   := selData.tmask
  io.scoreboard_data.PC      := selData.PC
  io.scoreboard_data.ex_type := selData.ex_type
  io.scoreboard_data.op_type := selData.op_type
  io.scoreboard_data.op_args := selData.op_args
  io.scoreboard_data.wb      := selData.wb
  io.scoreboard_data.used_rs := selData.used_rs
  io.scoreboard_data.rd      := selData.rd
  io.scoreboard_data.rs1     := selData.rs1
  io.scoreboard_data.rs2     := selData.rs2
  io.scoreboard_data.rs3     := selData.rs3

  // -------------------------------------------------------------------------
  // Performance counter: stall cycle = at least one warp valid but none ready
  // SV: perf_stall_per_cycle = (|stg_valid_in) && ~(|(stg_valid_in & operands_ready))
  // -------------------------------------------------------------------------

  val anyStgValid  = staging_valid.asUInt.orR
  val anyStgReady  = VecInit((0 until PER_ISSUE_WARPS).map(w =>
                       staging_valid(w) && operands_ready(w))).asUInt.orR
  val perf_stall   = anyStgValid && !anyStgReady

  val perf_stalls_r = RegInit(0.U(PERF_CTR_BITS.W))
  when (!reset.asBool) {
    perf_stalls_r := perf_stalls_r + perf_stall.asUInt
  }
  io.perf_stalls := perf_stalls_r
}
