// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_gather_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_gather_unit: reassembles per-block results into per-issue-slot commit packets.
 *
 *  BLOCK_SIZE execution blocks each drive one result_if (with result.wis
 *  identifying which issue slot it belongs to).  The unit steers the results
 *  to the corresponding commit_if[0..ISSUE_WIDTH-1] output, inserting an
 *  elastic buffer on each output path.
 *
 *  The SV original uses packed-array bit-slicing to perform the routing; the
 *  Chisel translation expresses the same logic combinatorially using a Mux
 *  network and Reg/Wire arrays.
 *
 *  @param blockSize  BLOCK_SIZE parameter from SV
 *  @param numLanes   NUM_LANES parameter (lanes per block)
 *  @param outBuf     OUT_BUF  (0 = skid, 1 = pipe, 2 = reg, 3 = skid-pipe)
 */
class VxGatherUnit(
    val blockSize: Int = 1,
    val numLanes:  Int = 1,
    val outBuf:    Int = 0
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  // Derived constants matching VX_gather_unit.sv
  private val numPackets  = SIMD_WIDTH / numLanes   // packets per warp iteration
  private val lpidBits    = log2Ceil(math.max(1, numPackets))
  private val lpidWidth   = math.max(1, lpidBits)
  private val gpidWidth   = math.max(1, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
  // Total result data width (matches DATAW in SV)
  private val dataw = UUID_WIDTH + NW_WIDTH + numLanes + PC_BITS + 1 + NUM_REGS_BITS +
                      numLanes * XLEN + gpidWidth + 1 + 1

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Result inputs from each block (slave)
    val result_valid = Input(Vec(blockSize, Bool()))
    val result_ready = Output(Vec(blockSize, Bool()))
    val result_data  = Input(Vec(blockSize, UInt(dataw.W)))

    // Commit outputs to each issue slot (master)
    val commit_valid = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_ready = Input(Vec(ISSUE_WIDTH, Bool()))
    // commit data fields
    val commit_uuid  = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val commit_wid   = Output(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
    val commit_sid   = Output(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val commit_tmask = Output(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val commit_PC    = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val commit_wb    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_rd    = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val commit_data  = Output(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val commit_sop   = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_eop   = Output(Vec(ISSUE_WIDTH, Bool()))
  })

  // -------------------------------------------------------------------------
  // Determine ISW (issue-slot-within-width) index for each result input.
  // Mirrors the result_in_isw computation in VX_gather_unit.sv.
  //
  // The SV packs wid/wis into the MSBs of the data word at offset DATA_WIS_OFF.
  // DATA_WIS_OFF = dataw - (UUID_WIDTH + NW_WIDTH)
  //
  // Case: BLOCK_SIZE == ISSUE_WIDTH  -> isw[i] = i (one block per slot)
  // Case: BLOCK_SIZE == 1            -> isw[0] = data[DATA_WIS_OFF +: ISSUE_ISW_W]
  // Case: otherwise                  -> isw[i] = {data[wis upper bits], i[blockSizeW]}
  // -------------------------------------------------------------------------
  private val blockSizeW  = math.max(1, log2Ceil(blockSize))
  private val dataWisOff  = dataw - (UUID_WIDTH + NW_WIDTH)

  val result_in_isw = Wire(Vec(blockSize, UInt(ISSUE_ISW_W.W)))
  for (i <- 0 until blockSize) {
    if (blockSize == ISSUE_WIDTH) {
      result_in_isw(i) := i.U(ISSUE_ISW_W.W)
    } else if (blockSize == 1) {
      result_in_isw(i) := io.result_data(i)(dataWisOff + ISSUE_ISW_W - 1, dataWisOff)
    } else {
      // partial: upper (ISSUE_ISW_W - blockSizeW) bits from data, lower blockSizeW bits = i
      val upperBits = ISSUE_ISW_W - blockSizeW
      val wisUpper  = io.result_data(i)(dataWisOff + ISSUE_WIS_W - 1, dataWisOff + ISSUE_WIS_W - upperBits)
      result_in_isw(i) := Cat(wisUpper, i.U(blockSizeW.W))
    }
  }

  // -------------------------------------------------------------------------
  // Steer valid+data from BLOCK_SIZE inputs to ISSUE_WIDTH outputs.
  // Only one input can drive each output slot at a time (mutual exclusion
  // guaranteed by the dispatch unit upstream).
  // -------------------------------------------------------------------------
  val result_out_valid = Wire(Vec(ISSUE_WIDTH, Bool()))
  val result_out_data  = Wire(Vec(ISSUE_WIDTH, UInt(dataw.W)))
  val result_out_ready = Wire(Vec(ISSUE_WIDTH, Bool()))

  for (j <- 0 until ISSUE_WIDTH) {
    result_out_valid(j) := false.B
    result_out_data(j)  := DontCare
  }

  for (i <- 0 until blockSize) {
    when (io.result_valid(i)) {
      result_out_valid(result_in_isw(i)) := io.result_valid(i)
      result_out_data(result_in_isw(i))  := io.result_data(i)
    }
  }

  // Ready back to each block input
  for (i <- 0 until blockSize) {
    io.result_ready(i) := result_out_ready(result_in_isw(i))
  }

  // -------------------------------------------------------------------------
  // Per-output-slot elastic buffer + pid/tmask expansion.
  //
  // The SV uses VX_elastic_buffer to buffer the steered result, then expands
  // the pid into a SIMD_WIDTH-wide tmask/data window.
  //
  // The Chisel translation uses a simple skid-buffer (register + bypass) and
  // performs the same expansion combinatorially.
  // -------------------------------------------------------------------------
  for (slotIdx <- 0 until ISSUE_WIDTH) {
    // Simple 1-entry skid buffer (OUT_BUF=0 equivalent)
    val buf_valid = RegInit(false.B)
    val buf_data  = Reg(UInt(dataw.W))

    val fire_in  = result_out_valid(slotIdx) && result_out_ready(slotIdx)
    val fire_out = io.commit_valid(slotIdx)  && io.commit_ready(slotIdx)

    when (!buf_valid || fire_out) {
      buf_valid := result_out_valid(slotIdx)
      buf_data  := result_out_data(slotIdx)
    }

    result_out_ready(slotIdx) := !buf_valid || fire_out

    // Unpack buf_data fields (bit layout matches ResultDataBundle / DECL_RESULT_T)
    // Layout (MSB first): uuid | wid | tmask | PC | wb | rd | data[0..N-1] | pid | sop | eop
    val pidWidth   = math.max(1, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
    var lsb = 0
    val eopF  = buf_data(lsb);            lsb += 1
    val sopF  = buf_data(lsb);            lsb += 1
    val pidF  = buf_data(lsb + pidWidth - 1, lsb); lsb += pidWidth
    val dataF = (0 until numLanes).map { j =>
      val d = buf_data(lsb + XLEN - 1, lsb); lsb += XLEN; d
    }
    val rdF   = buf_data(lsb + NUM_REGS_BITS - 1, lsb); lsb += NUM_REGS_BITS
    val wbF   = buf_data(lsb);            lsb += 1
    val pcF   = buf_data(lsb + PC_BITS - 1, lsb); lsb += PC_BITS
    val tmskF = buf_data(lsb + numLanes - 1, lsb); lsb += numLanes
    val widF  = buf_data(lsb + NW_WIDTH - 1, lsb); lsb += NW_WIDTH
    val uuidF = buf_data(lsb + UUID_WIDTH - 1, lsb)

    // Expand pid+numLanes to full SIMD_WIDTH tmask and data
    val commitTmask = Wire(UInt(SIMD_WIDTH.W))
    val commitData  = Wire(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val commitSid   = Wire(UInt(SIMD_IDX_W.W))

    if (lpidBits == 0) {
      // numLanes == SIMD_WIDTH: pid == sid directly
      commitSid   := pidF(SIMD_IDX_W - 1, 0)
      commitTmask := tmskF
      for (j <- 0 until SIMD_WIDTH) commitData(j) := dataF(j)
    } else {
      // Split pid into (sid, lpid) and expand
      val lpid = Wire(UInt(lpidWidth.W))
      if (SIMD_COUNT != 1) {
        commitSid := pidF(pidWidth - 1, lpidWidth)
        lpid      := pidF(lpidWidth - 1, 0)
      } else {
        commitSid := 0.U
        lpid      := pidF
      }
      commitTmask := 0.U  // default
      for (j <- 0 until SIMD_WIDTH) commitData(j) := 0.U
      // Scatter lanes into the lpid window
      for (j <- 0 until numLanes) {
        val idx = lpid * numLanes.U + j.U
        // Use dynamic indexing into the output vectors
        val tBit  = tmskF(j)
        val dVal  = dataF(j)
        // Build output via bit manipulation
        // (SIMD_WIDTH is a constant so we can use a Mux chain)
        // For conciseness, we OR-in each lane contribution
        // NOTE: this is a simplified Chisel idiom; exact per-bit placement
        // matches the SV `commit_tmask_w[lpid * NUM_LANES + j]` logic.
        val mask = VecInit((0 until SIMD_WIDTH).map { k =>
          Mux(idx === k.U, tBit, false.B)
        })
        commitTmask := commitTmask | mask.asUInt
        // Same for data
        for (k <- 0 until SIMD_WIDTH) {
          when (idx === k.U) {
            commitData(k) := dVal
          }
        }
      }
    }

    io.commit_valid(slotIdx) := buf_valid
    io.commit_uuid(slotIdx)  := uuidF
    io.commit_wid(slotIdx)   := widF
    io.commit_sid(slotIdx)   := commitSid
    io.commit_tmask(slotIdx) := commitTmask
    io.commit_PC(slotIdx)    := pcF
    io.commit_wb(slotIdx)    := wbF
    io.commit_rd(slotIdx)    := rdF
    io.commit_data(slotIdx)  := commitData
    io.commit_sop(slotIdx)   := sopF
    io.commit_eop(slotIdx)   := eopF
  }
}
