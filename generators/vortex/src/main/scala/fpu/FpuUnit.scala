// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_fpu_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._
import FpuPkg._

/** VX_fpu_unit: floating-point execution unit.
 *
 *  Structural translation of VX_fpu_unit.sv.  The unit is only elaborated
 *  when EXT_F_ENABLED == 1 (guarded at instantiation site in Execute.scala).
 *
 *  Pipeline per block:
 *    dispatch_if → [tag_store] → FPU backend BB → [rsp_buf] → result_if
 *    result_if   → gather_unit → commit_if
 *    FPU CSR write: fflags accumulated per-warp, registered to fpu_csr port
 *
 *  BLOCK_SIZE = NUM_FPU_BLOCKS = ISSUE_WIDTH
 *  NUM_LANES  = NUM_FPU_LANES  = SIMD_WIDTH
 *
 *  All complex sub-units (VX_dispatch_unit, VX_index_buffer, FPU backends,
 *  VX_gather_unit) are wrapped as BlackBox stubs since they have not been
 *  individually translated yet.
 */
class VxFpuUnit(val instanceId: String = "") extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  import FpuPkg._

  private val BLOCK_SIZE = NUM_FPU_BLOCKS
  private val NUM_LANES  = NUM_FPU_LANES
  // PID_BITS = clog2(NUM_THREADS / NUM_LANES); PID_WIDTH = UP(PID_BITS) = max(1, PID_BITS)
  private val PID_BITS   = log2Ceil(math.max(1, NUM_THREADS / NUM_LANES))
  private val PID_WIDTH  = math.max(1, PID_BITS)
  private val TAG_WIDTH  = log2Ceil(FPUQ_SIZE)
  // IBUF_DATAW = UUID_WIDTH + NW_WIDTH + NUM_LANES + PC_BITS + NUM_REGS_BITS + PID_WIDTH + 1 + 1
  private val IBUF_DATAW = UUID_WIDTH + NW_WIDTH + NUM_LANES + PC_BITS + NUM_REGS_BITS + PID_WIDTH + 1 + 1

  val io = IO(new Bundle {
    // Dispatch inputs (ISSUE_WIDTH = BLOCK_SIZE slots)
    val dispatch_valid    = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_uuid     = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val dispatch_wis      = Input(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val dispatch_sid      = Input(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val dispatch_tmask    = Input(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val dispatch_PC       = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val dispatch_op_type  = Input(Vec(ISSUE_WIDTH, UInt(INST_FPU_BITS.W)))
    val dispatch_op_args  = Input(Vec(ISSUE_WIDTH, UInt(INST_ARGS_BITS.W)))
    val dispatch_wb       = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_rd       = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val dispatch_rs1_data = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs2_data = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs3_data = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_sop      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_eop      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_ready    = Output(Vec(ISSUE_WIDTH, Bool()))

    // Commit outputs (ISSUE_WIDTH slots)
    val commit_valid  = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_uuid   = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val commit_wis    = Output(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val commit_sid    = Output(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val commit_tmask  = Output(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val commit_PC     = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val commit_rd     = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val commit_data   = Output(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val commit_wb     = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_sop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_eop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_ready  = Input(Vec(ISSUE_WIDTH, Bool()))

    // FPU CSR: fflags write (NUM_FPU_BLOCKS = BLOCK_SIZE ports)
    val fpu_csr_write_enable = Output(Vec(BLOCK_SIZE, Bool()))
    val fpu_csr_write_wid    = Output(Vec(BLOCK_SIZE, UInt(NW_WIDTH.W)))
    val fpu_csr_write_fflags = Output(Vec(BLOCK_SIZE, UInt(FP_FLAGS_BITS.W)))
    // FPU CSR: frm read (one per block)
    val fpu_csr_read_wid     = Output(Vec(BLOCK_SIZE, UInt(NW_WIDTH.W)))
    val fpu_csr_read_frm     = Input(Vec(BLOCK_SIZE, UInt(INST_FRM_BITS.W)))
  })

  // -------------------------------------------------------------------------
  // Instantiate the BlackBox stub that covers the whole VX_fpu_unit body.
  // -------------------------------------------------------------------------
  val fpuUnitBB = Module(new VxFpuUnitBB(instanceId, BLOCK_SIZE, NUM_LANES, PID_WIDTH, TAG_WIDTH, IBUF_DATAW))

  // Dispatch
  for (i <- 0 until ISSUE_WIDTH) {
    fpuUnitBB.io.dispatch_valid(i)    := io.dispatch_valid(i)
    fpuUnitBB.io.dispatch_uuid(i)     := io.dispatch_uuid(i)
    fpuUnitBB.io.dispatch_wis(i)      := io.dispatch_wis(i)
    fpuUnitBB.io.dispatch_sid(i)      := io.dispatch_sid(i)
    fpuUnitBB.io.dispatch_tmask(i)    := io.dispatch_tmask(i)
    fpuUnitBB.io.dispatch_PC(i)       := io.dispatch_PC(i)
    fpuUnitBB.io.dispatch_op_type(i)  := io.dispatch_op_type(i)
    fpuUnitBB.io.dispatch_op_args(i)  := io.dispatch_op_args(i)
    fpuUnitBB.io.dispatch_wb(i)       := io.dispatch_wb(i)
    fpuUnitBB.io.dispatch_rd(i)       := io.dispatch_rd(i)
    fpuUnitBB.io.dispatch_rs1_data(i) := io.dispatch_rs1_data(i)
    fpuUnitBB.io.dispatch_rs2_data(i) := io.dispatch_rs2_data(i)
    fpuUnitBB.io.dispatch_rs3_data(i) := io.dispatch_rs3_data(i)
    fpuUnitBB.io.dispatch_sop(i)      := io.dispatch_sop(i)
    fpuUnitBB.io.dispatch_eop(i)      := io.dispatch_eop(i)
    io.dispatch_ready(i)              := fpuUnitBB.io.dispatch_ready(i)
  }

  // Commit
  for (i <- 0 until ISSUE_WIDTH) {
    io.commit_valid(i)  := fpuUnitBB.io.commit_valid(i)
    io.commit_uuid(i)   := fpuUnitBB.io.commit_uuid(i)
    io.commit_wis(i)    := fpuUnitBB.io.commit_wis(i)
    io.commit_sid(i)    := fpuUnitBB.io.commit_sid(i)
    io.commit_tmask(i)  := fpuUnitBB.io.commit_tmask(i)
    io.commit_PC(i)     := fpuUnitBB.io.commit_PC(i)
    io.commit_rd(i)     := fpuUnitBB.io.commit_rd(i)
    io.commit_data(i)   := fpuUnitBB.io.commit_data(i)
    io.commit_wb(i)     := fpuUnitBB.io.commit_wb(i)
    io.commit_sop(i)    := fpuUnitBB.io.commit_sop(i)
    io.commit_eop(i)    := fpuUnitBB.io.commit_eop(i)
    fpuUnitBB.io.commit_ready(i) := io.commit_ready(i)
  }

  // FPU CSR
  for (i <- 0 until BLOCK_SIZE) {
    io.fpu_csr_write_enable(i) := fpuUnitBB.io.fpu_csr_write_enable(i)
    io.fpu_csr_write_wid(i)    := fpuUnitBB.io.fpu_csr_write_wid(i)
    io.fpu_csr_write_fflags(i) := fpuUnitBB.io.fpu_csr_write_fflags(i)
    io.fpu_csr_read_wid(i)     := fpuUnitBB.io.fpu_csr_read_wid(i)
    fpuUnitBB.io.fpu_csr_read_frm(i) := io.fpu_csr_read_frm(i)
  }
}

// =============================================================================
// BlackBox stub for VX_fpu_unit
// =============================================================================
class VxFpuUnitBB(
    instanceId: String,
    blockSize:  Int = 1,
    numLanes:   Int = VortexConfigConstants.SIMD_WIDTH,
    pidWidth:   Int = math.max(1, chisel3.util.log2Ceil(math.max(1, VortexConfigConstants.NUM_THREADS / VortexConfigConstants.SIMD_WIDTH))),
    tagWidth:   Int = 8,
    ibufDataw:  Int = 64
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  import FpuPkg._

  val io = IO(new Bundle {
    val dispatch_valid    = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_uuid     = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val dispatch_wis      = Input(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val dispatch_sid      = Input(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val dispatch_tmask    = Input(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val dispatch_PC       = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val dispatch_op_type  = Input(Vec(ISSUE_WIDTH, UInt(INST_FPU_BITS.W)))
    val dispatch_op_args  = Input(Vec(ISSUE_WIDTH, UInt(INST_ARGS_BITS.W)))
    val dispatch_wb       = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_rd       = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val dispatch_rs1_data = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs2_data = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_rs3_data = Input(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val dispatch_sop      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_eop      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_ready    = Output(Vec(ISSUE_WIDTH, Bool()))

    val commit_valid  = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_uuid   = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val commit_wis    = Output(Vec(ISSUE_WIDTH, UInt(ISSUE_WIS_W.W)))
    val commit_sid    = Output(Vec(ISSUE_WIDTH, UInt(SIMD_IDX_W.W)))
    val commit_tmask  = Output(Vec(ISSUE_WIDTH, UInt(SIMD_WIDTH.W)))
    val commit_PC     = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val commit_rd     = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val commit_data   = Output(Vec(ISSUE_WIDTH, Vec(SIMD_WIDTH, UInt(XLEN.W))))
    val commit_wb     = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_sop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_eop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_ready  = Input(Vec(ISSUE_WIDTH, Bool()))

    val fpu_csr_write_enable = Output(Vec(blockSize, Bool()))
    val fpu_csr_write_wid    = Output(Vec(blockSize, UInt(NW_WIDTH.W)))
    val fpu_csr_write_fflags = Output(Vec(blockSize, UInt(FP_FLAGS_BITS.W)))
    val fpu_csr_read_wid     = Output(Vec(blockSize, UInt(NW_WIDTH.W)))
    val fpu_csr_read_frm     = Input(Vec(blockSize, UInt(INST_FRM_BITS.W)))
  })

  // Stub: all outputs driven to 0
  for (i <- 0 until ISSUE_WIDTH) {
    io.dispatch_ready(i)  := false.B
    io.commit_valid(i)    := false.B
    io.commit_uuid(i)     := 0.U
    io.commit_wis(i)      := 0.U
    io.commit_sid(i)      := 0.U
    io.commit_tmask(i)    := 0.U
    io.commit_PC(i)       := 0.U
    io.commit_rd(i)       := 0.U
    io.commit_data(i)     := VecInit(Seq.fill(SIMD_WIDTH)(0.U(XLEN.W)))
    io.commit_wb(i)       := false.B
    io.commit_sop(i)      := false.B
    io.commit_eop(i)      := false.B
  }
  for (i <- 0 until blockSize) {
    io.fpu_csr_write_enable(i) := false.B
    io.fpu_csr_write_wid(i)    := 0.U
    io.fpu_csr_write_fflags(i) := 0.U
    io.fpu_csr_read_wid(i)     := 0.U
  }
}
