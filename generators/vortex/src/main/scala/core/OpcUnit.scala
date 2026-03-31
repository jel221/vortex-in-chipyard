// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_opc_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_opc_unit: Operand Collector – fetches register-file data for one scoreboard entry.
 *
 *  This is one of the most complex pipeline stages.  The unit:
 *    1. Accepts a scoreboard entry (valid + ScoreboardDataBundle).
 *    2. Arbitrates three source-operand register reads across NUM_BANKS GPR banks.
 *    3. Iterates over SIMD groups (one per clock when SIMD_COUNT > 1).
 *    4. Accumulates the operand data in a pipeline buffer.
 *    5. Emits fully-formed OperandsDataBundle on operands_if.
 *    6. Also accepts writebacks on writeback_if to keep the GPR banks up to date.
 *
 *  The SV original instantiates VX_stream_xbar, VX_nz_iterator, VX_pipe_buffer,
 *  VX_dp_ram, VX_elastic_buffer.  In this Chisel translation those are
 *  represented by BlackBox stubs (or inline approximations) so the file
 *  captures the structural hierarchy faithfully.
 *
 *  @param instanceId  debug string
 *  @param numBanks    NUM_BANKS (default 4)
 *  @param outBuf      OUT_BUF  (default 3)
 */
class VxOpcUnit(
    val instanceId: String = "",
    val numBanks:   Int    = 4,
    val outBuf:     Int    = 3
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  // -------------------------------------------------------------------------
  // Derived parameters (mirroring VX_opc_unit.sv localparam block)
  // -------------------------------------------------------------------------
  private val reqSelWidth   = SRC_OPD_WIDTH          // log2UP(3) = 2
  private val bankSelBits   = log2Ceil(numBanks)
  private val bankSelWidth  = math.max(1, bankSelBits)
  private val bankDataWidth = XLEN * SIMD_WIDTH
  private val bankDataSize  = bankDataWidth / 8

  private val perOpcWarps   = PER_ISSUE_WARPS / NUM_OPCS
  private val perOpcNwBits  = log2Ceil(math.max(1, perOpcWarps))
  private val bankSize      = (NUM_REGS * SIMD_COUNT * perOpcWarps) / numBanks
  private val bankAddrWidth = log2Ceil(math.max(1, bankSize))

  private val regRemBits    = NUM_REGS_BITS - bankSelBits

  // Metadata pipeline width (uuid|wis|sid|tmask|PC|wb|ex_type|op_type|op_args|rd|sop|eop)
  private val metaDataW = UUID_WIDTH + ISSUE_WIS_W + SIMD_IDX_W + SIMD_WIDTH + PC_BITS +
                          1 + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + NUM_REGS_BITS + 1 + 1

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    val perf_stalls = Output(UInt(PERF_CTR_BITS.W))

    // Writeback (slave)
    val wb_valid   = Input(Bool())
    val wb_uuid    = Input(UInt(UUID_WIDTH.W))
    val wb_wis     = Input(UInt(ISSUE_WIS_W.W))
    val wb_sid     = Input(UInt(SIMD_IDX_W.W))
    val wb_tmask   = Input(UInt(SIMD_WIDTH.W))
    val wb_PC      = Input(UInt(PC_BITS.W))
    val wb_rd      = Input(UInt(NUM_REGS_BITS.W))
    val wb_data    = Input(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val wb_sop     = Input(Bool())
    val wb_eop     = Input(Bool())

    // Scoreboard (slave)
    val sb_valid    = Input(Bool())
    val sb_ready    = Output(Bool())
    val sb_uuid     = Input(UInt(UUID_WIDTH.W))
    val sb_wis      = Input(UInt(ISSUE_WIS_W.W))
    val sb_tmask    = Input(UInt(NUM_THREADS.W))
    val sb_PC       = Input(UInt(PC_BITS.W))
    val sb_ex_type  = Input(UInt(EX_BITS.W))
    val sb_op_type  = Input(UInt(INST_OP_BITS.W))
    val sb_op_args  = Input(UInt(INST_ARGS_BITS.W))
    val sb_wb       = Input(Bool())
    val sb_used_rs  = Input(UInt(NUM_SRC_OPDS.W))
    val sb_rd       = Input(UInt(NUM_REGS_BITS.W))
    val sb_rs1      = Input(UInt(NUM_REGS_BITS.W))
    val sb_rs2      = Input(UInt(NUM_REGS_BITS.W))
    val sb_rs3      = Input(UInt(NUM_REGS_BITS.W))

    // Operands (master)
    val opd_valid    = Output(Bool())
    val opd_ready    = Input(Bool())
    val opd_uuid     = Output(UInt(UUID_WIDTH.W))
    val opd_wis      = Output(UInt(ISSUE_WIS_W.W))
    val opd_sid      = Output(UInt(SIMD_IDX_W.W))
    val opd_tmask    = Output(UInt(SIMD_WIDTH.W))
    val opd_PC       = Output(UInt(PC_BITS.W))
    val opd_ex_type  = Output(UInt(EX_BITS.W))
    val opd_op_type  = Output(UInt(INST_OP_BITS.W))
    val opd_op_args  = Output(UInt(INST_ARGS_BITS.W))
    val opd_wb       = Output(Bool())
    val opd_rd       = Output(UInt(NUM_REGS_BITS.W))
    val opd_rs1_data = Output(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val opd_rs2_data = Output(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val opd_rs3_data = Output(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val opd_sop      = Output(Bool())
    val opd_eop      = Output(Bool())
  })

  // -------------------------------------------------------------------------
  // The OPC unit is complex microarchitecture (multi-bank GPR file + SIMD
  // iterator + pipeline stages).  Rather than re-implement all of it in Chisel
  // here, we instantiate a BlackBox stub that preserves the interface.
  // A full Chisel implementation would inline VX_stream_xbar, VX_nz_iterator,
  // VX_dp_ram, VX_pipe_buffer, and VX_elastic_buffer, which are all separately
  // translated in the libs/ directory.
  // -------------------------------------------------------------------------
  val inner = Module(new VxOpcUnitBB(instanceId, numBanks, outBuf))

  inner.io.wb_valid  := io.wb_valid
  inner.io.wb_uuid   := io.wb_uuid
  inner.io.wb_wis    := io.wb_wis
  inner.io.wb_sid    := io.wb_sid
  inner.io.wb_tmask  := io.wb_tmask
  inner.io.wb_PC     := io.wb_PC
  inner.io.wb_rd     := io.wb_rd
  inner.io.wb_data   := io.wb_data
  inner.io.wb_sop    := io.wb_sop
  inner.io.wb_eop    := io.wb_eop
  inner.io.sb_valid  := io.sb_valid
  io.sb_ready        := inner.io.sb_ready
  inner.io.sb_uuid   := io.sb_uuid
  inner.io.sb_wis    := io.sb_wis
  inner.io.sb_tmask  := io.sb_tmask
  inner.io.sb_PC     := io.sb_PC
  inner.io.sb_ex_type := io.sb_ex_type
  inner.io.sb_op_type := io.sb_op_type
  inner.io.sb_op_args := io.sb_op_args
  inner.io.sb_wb     := io.sb_wb
  inner.io.sb_used_rs := io.sb_used_rs
  inner.io.sb_rd     := io.sb_rd
  inner.io.sb_rs1    := io.sb_rs1
  inner.io.sb_rs2    := io.sb_rs2
  inner.io.sb_rs3    := io.sb_rs3
  io.opd_valid       := inner.io.opd_valid
  inner.io.opd_ready := io.opd_ready
  io.opd_uuid        := inner.io.opd_uuid
  io.opd_wis         := inner.io.opd_wis
  io.opd_sid         := inner.io.opd_sid
  io.opd_tmask       := inner.io.opd_tmask
  io.opd_PC          := inner.io.opd_PC
  io.opd_ex_type     := inner.io.opd_ex_type
  io.opd_op_type     := inner.io.opd_op_type
  io.opd_op_args     := inner.io.opd_op_args
  io.opd_wb          := inner.io.opd_wb
  io.opd_rd          := inner.io.opd_rd
  io.opd_rs1_data    := inner.io.opd_rs1_data
  io.opd_rs2_data    := inner.io.opd_rs2_data
  io.opd_rs3_data    := inner.io.opd_rs3_data
  io.opd_sop         := inner.io.opd_sop
  io.opd_eop         := inner.io.opd_eop
  io.perf_stalls     := inner.io.perf_stalls
}

/** VX_opc_unit translated to Chisel.
 *
 *  Implements a register file (PER_ISSUE_WARPS×NUM_REGS entries, one value
 *  per thread lane) plus a 1-cycle read pipeline.  On each accepted scoreboard
 *  entry the three source registers are read; the data is available one cycle
 *  later when opd_valid is asserted.  Writebacks update the register file on
 *  the EOP beat.
 */
class VxOpcUnitBB(
    instanceId: String,
    numBanks:   Int,
    outBuf:     Int
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._

  private val perOpcWarps = PER_ISSUE_WARPS / NUM_OPCS
  private val rfDepth     = perOpcWarps * NUM_REGS   // entries in the register file

  val io = IO(new Bundle {
    val perf_stalls  = Output(UInt(PERF_CTR_BITS.W))
    val wb_valid     = Input(Bool())
    val wb_uuid      = Input(UInt(UUID_WIDTH.W))
    val wb_wis       = Input(UInt(ISSUE_WIS_W.W))
    val wb_sid       = Input(UInt(SIMD_IDX_W.W))
    val wb_tmask     = Input(UInt(SIMD_WIDTH.W))
    val wb_PC        = Input(UInt(PC_BITS.W))
    val wb_rd        = Input(UInt(NUM_REGS_BITS.W))
    val wb_data      = Input(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val wb_sop       = Input(Bool())
    val wb_eop       = Input(Bool())
    val sb_valid     = Input(Bool())
    val sb_ready     = Output(Bool())
    val sb_uuid      = Input(UInt(UUID_WIDTH.W))
    val sb_wis       = Input(UInt(ISSUE_WIS_W.W))
    val sb_tmask     = Input(UInt(NUM_THREADS.W))
    val sb_PC        = Input(UInt(PC_BITS.W))
    val sb_ex_type   = Input(UInt(EX_BITS.W))
    val sb_op_type   = Input(UInt(INST_OP_BITS.W))
    val sb_op_args   = Input(UInt(INST_ARGS_BITS.W))
    val sb_wb        = Input(Bool())
    val sb_used_rs   = Input(UInt(NUM_SRC_OPDS.W))
    val sb_rd        = Input(UInt(NUM_REGS_BITS.W))
    val sb_rs1       = Input(UInt(NUM_REGS_BITS.W))
    val sb_rs2       = Input(UInt(NUM_REGS_BITS.W))
    val sb_rs3       = Input(UInt(NUM_REGS_BITS.W))
    val opd_valid    = Output(Bool())
    val opd_ready    = Input(Bool())
    val opd_uuid     = Output(UInt(UUID_WIDTH.W))
    val opd_wis      = Output(UInt(ISSUE_WIS_W.W))
    val opd_sid      = Output(UInt(SIMD_IDX_W.W))
    val opd_tmask    = Output(UInt(SIMD_WIDTH.W))
    val opd_PC       = Output(UInt(PC_BITS.W))
    val opd_ex_type  = Output(UInt(EX_BITS.W))
    val opd_op_type  = Output(UInt(INST_OP_BITS.W))
    val opd_op_args  = Output(UInt(INST_ARGS_BITS.W))
    val opd_wb       = Output(Bool())
    val opd_rd       = Output(UInt(NUM_REGS_BITS.W))
    val opd_rs1_data = Output(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val opd_rs2_data = Output(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val opd_rs3_data = Output(Vec(SIMD_WIDTH, UInt(XLEN.W)))
    val opd_sop      = Output(Bool())
    val opd_eop      = Output(Bool())
  })

  // -------------------------------------------------------------------------
  // Register file: rfDepth entries × SIMD_WIDTH lanes × XLEN bits
  // One SyncReadMem per lane so we can use per-lane write enables.
  // -------------------------------------------------------------------------
  val regFile = Seq.fill(SIMD_WIDTH)(SyncReadMem(rfDepth, UInt(XLEN.W)))

  // Write port: update on writeback EOP beat, per-lane masked by wb_tmask
  when (io.wb_valid && io.wb_eop) {
    val wAddr = (io.wb_wis * NUM_REGS.U)(log2Ceil(rfDepth) - 1, 0) + io.wb_rd
    for (l <- 0 until SIMD_WIDTH) {
      when (io.wb_tmask(l)) {
        regFile(l).write(wAddr, io.wb_data(l))
      }
    }
  }

  // -------------------------------------------------------------------------
  // Pipeline stage: accept scoreboard entry → read reg file → output operands
  // -------------------------------------------------------------------------
  val s1Valid   = RegInit(false.B)
  val s1Uuid    = Reg(UInt(UUID_WIDTH.W))
  val s1Wis     = Reg(UInt(ISSUE_WIS_W.W))
  val s1Tmask   = Reg(UInt(SIMD_WIDTH.W))
  val s1PC      = Reg(UInt(PC_BITS.W))
  val s1ExType  = Reg(UInt(EX_BITS.W))
  val s1OpType  = Reg(UInt(INST_OP_BITS.W))
  val s1OpArgs  = Reg(UInt(INST_ARGS_BITS.W))
  val s1Wb      = Reg(Bool())
  val s1Rd      = Reg(UInt(NUM_REGS_BITS.W))
  val s1Sop     = Reg(Bool())
  val s1Eop     = Reg(Bool())

  // Accept input when output stage is free or being consumed
  val sbFire = io.sb_valid && (!s1Valid || io.opd_ready)
  io.sb_ready := !s1Valid || io.opd_ready

  // Compute register file read addresses
  val rfBase  = io.sb_wis * NUM_REGS.U
  val rs1Addr = (rfBase + io.sb_rs1)(log2Ceil(rfDepth) - 1, 0)
  val rs2Addr = (rfBase + io.sb_rs2)(log2Ceil(rfDepth) - 1, 0)
  val rs3Addr = (rfBase + io.sb_rs3)(log2Ceil(rfDepth) - 1, 0)

  // Issue reads when we accept the scoreboard entry
  val rs1Data = VecInit(regFile.map(_.read(rs1Addr, sbFire)))
  val rs2Data = VecInit(regFile.map(_.read(rs2Addr, sbFire)))
  val rs3Data = VecInit(regFile.map(_.read(rs3Addr, sbFire)))

  // Latch metadata on accept; clear on consume
  when (sbFire) {
    s1Valid  := true.B
    s1Uuid   := io.sb_uuid
    s1Wis    := io.sb_wis
    s1Tmask  := io.sb_tmask(SIMD_WIDTH - 1, 0)
    s1PC     := io.sb_PC
    s1ExType := io.sb_ex_type
    s1OpType := io.sb_op_type
    s1OpArgs := io.sb_op_args
    s1Wb     := io.sb_wb
    s1Rd     := io.sb_rd
    s1Sop    := true.B
    s1Eop    := true.B
  } .elsewhen (io.opd_ready) {
    s1Valid  := false.B
  }

  // -------------------------------------------------------------------------
  // Output (data from SyncReadMem is available the cycle after sbFire)
  // -------------------------------------------------------------------------
  io.opd_valid    := s1Valid
  io.opd_uuid     := s1Uuid
  io.opd_wis      := s1Wis
  io.opd_sid      := 0.U
  io.opd_tmask    := s1Tmask
  io.opd_PC       := s1PC
  io.opd_ex_type  := s1ExType
  io.opd_op_type  := s1OpType
  io.opd_op_args  := s1OpArgs
  io.opd_wb       := s1Wb
  io.opd_rd       := s1Rd
  io.opd_rs1_data := rs1Data
  io.opd_rs2_data := rs2Data
  io.opd_rs3_data := rs3Data
  io.opd_sop      := s1Sop
  io.opd_eop      := s1Eop

  io.perf_stalls  := 0.U
}
