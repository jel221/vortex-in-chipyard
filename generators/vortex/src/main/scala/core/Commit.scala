// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_commit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Commit stage.
 *
 *  Corresponds to VX_commit.sv.
 *
 *  Responsibilities:
 *    1. Arbitrate NUM_EX_UNITS commit streams per issue slot into one
 *       commit_arb stream.
 *    2. Count committed instructions (instret CSR counter update).
 *    3. Produce a committed_warps bitmask for the scheduler.
 *    4. Drive writeback interfaces for register-file writeback.
 *
 *  All NUM_EX_UNITS * ISSUE_WIDTH commit inputs are flattened as individual
 *  IO vectors.  The indexing convention matches the SV:
 *    commit_if[j * ISSUE_WIDTH + i]  →  commit_in_*(i)(j)
 */
class Commit extends Module {

  private val numLanes    = SIMD_WIDTH
  private val pidBits     = math.max(0, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
  private val pidWidth    = math.max(1, pidBits)
  private val commitSizeW = log2Ceil(SIMD_WIDTH + 1)           // CLOG2(SIMD_WIDTH + 1)
  private val commitAllW  = commitSizeW + ISSUE_WIDTH - 1      // matches COMMIT_ALL_SIZEW

  val io = IO(new Bundle {
    // Commit inputs: [ISSUE_WIDTH] slots x [NUM_EX_UNITS] execution units
    // Indexed as: commit_in_valid(issue)(unit)
    val commit_in_valid  = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, Bool())))
    val commit_in_ready  = Output(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, Bool())))
    val commit_in_uuid   = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, UInt(UUID_WIDTH.W))))
    val commit_in_wid    = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, UInt(NW_WIDTH.W))))
    val commit_in_sid    = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, UInt(SIMD_IDX_W.W))))
    val commit_in_tmask  = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, UInt(numLanes.W))))
    val commit_in_PC     = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, UInt(PC_BITS.W))))
    val commit_in_wb     = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, Bool())))
    val commit_in_rd     = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, UInt(NUM_REGS_BITS.W))))
    val commit_in_data   = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, Vec(numLanes, UInt(XLEN.W)))))
    val commit_in_sop    = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, Bool())))
    val commit_in_eop    = Input(Vec(ISSUE_WIDTH, Vec(NUM_EX_UNITS, Bool())))

    // Writeback outputs (one per ISSUE_WIDTH, no back-pressure)
    val writeback_valid  = Output(Vec(ISSUE_WIDTH, Bool()))
    val writeback_uuid   = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val writeback_wis    = Output(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val writeback_sid    = Output(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val writeback_PC     = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val writeback_tmask  = Output(Vec(ISSUE_WIDTH, UInt(numLanes.W)))
    val writeback_rd     = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val writeback_data   = Output(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))
    val writeback_sop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val writeback_eop    = Output(Vec(ISSUE_WIDTH, Bool()))

    // Commit CSR output (instret)
    val commit_csr_instret = Output(UInt(PERF_CTR_BITS.W))

    // Commit scheduler output (committed warp bitmask)
    val commit_sched_committed_warps = Output(UInt(NUM_WARPS.W))
  })

  // =========================================================================
  // Per-issue commit arbitration (NUM_EX_UNITS → 1)
  // VX_stream_arb with ARBITER="P" (priority): unit 0 has highest priority.
  // We model this as a priority encoder over the valid inputs with a 1-entry
  // output register (OUT_BUF=1).
  // =========================================================================

  // Arbitrated streams (post arb, before writeback)
  val arbValid  = Wire(Vec(ISSUE_WIDTH, Bool()))
  val arbReady  = Wire(Vec(ISSUE_WIDTH, Bool()))   // always 1 in the SV (commit_arb_if.ready = 1)
  val arbUuid   = Wire(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
  val arbWid    = Wire(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
  val arbSid    = Wire(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
  val arbTmask  = Wire(Vec(ISSUE_WIDTH, UInt(numLanes.W)))
  val arbPC     = Wire(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
  val arbWb     = Wire(Vec(ISSUE_WIDTH, Bool()))
  val arbRd     = Wire(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
  val arbData   = Wire(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))
  val arbSop    = Wire(Vec(ISSUE_WIDTH, Bool()))
  val arbEop    = Wire(Vec(ISSUE_WIDTH, Bool()))

  for (i <- 0 until ISSUE_WIDTH) {
    // Priority arbitration: lowest-index unit wins
    val valids = VecInit((0 until NUM_EX_UNITS).map(j => io.commit_in_valid(i)(j)))
    val sel    = PriorityEncoder(valids.asUInt)
    val anyValid = valids.asUInt.orR

    // Registered output buffer (OUT_BUF=1)
    val bufValid = RegInit(false.B)
    val bufUuid  = Reg(UInt(UUID_WIDTH.W))
    val bufWid   = Reg(UInt(NW_WIDTH.W))
    val bufSid   = Reg(UInt(SIMD_IDX_W.W))
    val bufTmask = Reg(UInt(numLanes.W))
    val bufPC    = Reg(UInt(PC_BITS.W))
    val bufWb    = Reg(Bool())
    val bufRd    = Reg(UInt(NUM_REGS_BITS.W))
    val bufData  = Reg(Vec(numLanes, UInt(XLEN.W)))
    val bufSop   = Reg(Bool())
    val bufEop   = Reg(Bool())

    // The SV always has commit_arb_if[i].ready = 1 (writeback never stalls)
    val outReady = true.B
    val canLoad  = !bufValid || outReady

    for (j <- 0 until NUM_EX_UNITS) {
      io.commit_in_ready(i)(j) := canLoad && (sel === j.U) && anyValid
    }

    when (anyValid && canLoad) {
      bufValid := true.B
      bufUuid  := MuxLookup(sel, 0.U)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_uuid(i)(j)))
      bufWid   := MuxLookup(sel, 0.U)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_wid(i)(j)))
      bufSid   := MuxLookup(sel, 0.U)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_sid(i)(j)))
      bufTmask := MuxLookup(sel, 0.U)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_tmask(i)(j)))
      bufPC    := MuxLookup(sel, 0.U)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_PC(i)(j)))
      bufWb    := MuxLookup(sel, false.B)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_wb(i)(j)))
      bufRd    := MuxLookup(sel, 0.U)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_rd(i)(j)))
      bufData  := MuxLookup(sel, VecInit(Seq.fill(numLanes)(0.U(XLEN.W))))(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_data(i)(j)))
      bufSop   := MuxLookup(sel, false.B)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_sop(i)(j)))
      bufEop   := MuxLookup(sel, false.B)(
        (0 until NUM_EX_UNITS).map(j => j.U -> io.commit_in_eop(i)(j)))
    } .elsewhen (outReady) {
      bufValid := false.B
    }

    arbValid(i)  := bufValid
    arbReady(i)  := outReady
    arbUuid(i)   := bufUuid
    arbWid(i)    := bufWid
    arbSid(i)    := bufSid
    arbTmask(i)  := bufTmask
    arbPC(i)     := bufPC
    arbWb(i)     := bufWb
    arbRd(i)     := bufRd
    arbData(i)   := bufData
    arbSop(i)    := bufSop
    arbEop(i)    := bufEop
  }

  // =========================================================================
  // Per-issue commit fire signals
  // =========================================================================
  val perIssueCommitFire  = VecInit((0 until ISSUE_WIDTH).map(i => arbValid(i) && arbReady(i)))
  val perIssueCommitTmask = VecInit((0 until ISSUE_WIDTH).map(i =>
    Mux(perIssueCommitFire(i), arbTmask(i), 0.U(numLanes.W))))
  val perIssueCommitWid   = VecInit((0 until ISSUE_WIDTH).map(i => arbWid(i)))
  val perIssueCommitEop   = VecInit((0 until ISSUE_WIDTH).map(i => arbEop(i)))

  val commitFireAny = perIssueCommitFire.asUInt.orR

  // =========================================================================
  // Instruction retire counter (instret)
  // Per-issue popcount pipeline (2 pipe stages to match SV latency)
  // =========================================================================
  val commitSize   = VecInit((0 until ISSUE_WIDTH).map(i =>
    PopCount(perIssueCommitTmask(i))(commitSizeW - 1, 0)))

  // Stage 1 register
  val commitFireR  = RegNext(commitFireAny, false.B)
  val commitSizeR  = RegNext(commitSize)

  // Reduce sum of per-issue commit sizes
  val commitSizeAllR = commitSizeR.map(_.asUInt).reduce(_ +& _)(commitAllW - 1, 0)

  // Stage 2 register
  val commitFireRR    = RegNext(commitFireR, false.B)
  val commitSizeAllRR = RegNext(commitSizeAllR)

  val instret = RegInit(0.U(PERF_CTR_BITS.W))
  when (commitFireRR) {
    instret := instret + commitSizeAllRR.asUInt
  }
  io.commit_csr_instret := instret

  // =========================================================================
  // Committed warps bitmask (for scheduler)
  // =========================================================================
  val committedWarps = Wire(UInt(NUM_WARPS.W))
  val cwBits = Wire(Vec(NUM_WARPS, Bool()))
  for (w <- 0 until NUM_WARPS) {
    cwBits(w) := (0 until ISSUE_WIDTH).map(i =>
      perIssueCommitFire(i) && perIssueCommitEop(i) && (perIssueCommitWid(i) === w.U)
    ).reduce(_ || _)
  }
  committedWarps := cwBits.asUInt

  val committedWarpsR = RegNext(committedWarps, 0.U(NUM_WARPS.W))
  io.commit_sched_committed_warps := committedWarpsR

  // =========================================================================
  // Writeback
  // commit_arb_if[i].ready = 1 always in the SV (writeback never stalls)
  // =========================================================================
  for (i <- 0 until ISSUE_WIDTH) {
    io.writeback_valid(i)  := arbValid(i) && arbWb(i)
    io.writeback_uuid(i)   := arbUuid(i)
    io.writeback_wis(i)    := wid_to_wis(arbWid(i))
    io.writeback_sid(i)    := arbSid(i)
    io.writeback_PC(i)     := arbPC(i)
    io.writeback_tmask(i)  := arbTmask(i)
    io.writeback_rd(i)     := arbRd(i)
    io.writeback_data(i)   := arbData(i)
    io.writeback_sop(i)    := arbSop(i)
    io.writeback_eop(i)    := arbEop(i)
  }
}
