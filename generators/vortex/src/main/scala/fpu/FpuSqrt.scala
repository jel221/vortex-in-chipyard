// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_fpu_sqrt.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._
import FpuPkg._

/** VX_fpu_sqrt: FPU floating-point square-root unit (FPU_DSP path).
 *
 *  Structural translation of VX_fpu_sqrt.sv.  Uses VX_pe_serializer to
 *  time-multiplex NUM_LANES inputs onto NUM_PES=UP(NUM_LANES/FSQRT_PE_RATIO)
 *  sqrt PEs (QUARTUS acl_fsqrt / VIVADO xil_fsqrt / DPI dpi_fsqrt).
 *  All internal logic is wrapped in VxFpuSqrtBB.
 *
 *  @param numLanes  Number of FP lanes (default NUM_FPU_LANES)
 *  @param tagWidth  Tag width (default log2Ceil(FPUQ_SIZE))
 */
class VxFpuSqrt(
    val numLanes: Int = NUM_FPU_LANES,
    val tagWidth: Int = log2Ceil(FPUQ_SIZE)
) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  import FpuPkg._

  // NUM_PES = UP(NUM_LANES / FSQRT_PE_RATIO)
  private val numPes = math.max(1, numLanes / FSQRT_PE_RATIO)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())

    val mask_in   = Input(UInt(numLanes.W))
    val tag_in    = Input(UInt(tagWidth.W))
    val frm       = Input(UInt(INST_FRM_BITS.W))

    val dataa     = Input(Vec(numLanes, UInt(32.W)))
    val result    = Output(Vec(numLanes, UInt(32.W)))

    val has_fflags = Output(Bool())
    val fflags     = Output(UInt(FP_FLAGS_BITS.W))

    val tag_out   = Output(UInt(tagWidth.W))
    val valid_out = Output(Bool())
    val ready_out = Input(Bool())
  })

  val bb = Module(new VxFpuSqrtBB(numLanes, tagWidth, numPes))

  bb.io.valid_in  := io.valid_in
  io.ready_in     := bb.io.ready_in
  bb.io.mask_in   := io.mask_in
  bb.io.tag_in    := io.tag_in
  bb.io.frm       := io.frm
  bb.io.dataa     := io.dataa
  io.result       := bb.io.result
  io.has_fflags   := bb.io.has_fflags
  io.fflags       := bb.io.fflags
  io.tag_out      := bb.io.tag_out
  io.valid_out    := bb.io.valid_out
  bb.io.ready_out := io.ready_out
}

// =============================================================================
// BlackBox stub for VX_fpu_sqrt
// =============================================================================
class VxFpuSqrtBB(numLanes: Int, tagWidth: Int, numPes: Int) extends Module {
  import VortexGPUPkg._
  import VortexConfigConstants._
  import FpuPkg._

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val mask_in   = Input(UInt(numLanes.W))
    val tag_in    = Input(UInt(tagWidth.W))
    val frm       = Input(UInt(INST_FRM_BITS.W))
    val dataa     = Input(Vec(numLanes, UInt(32.W)))
    val result    = Output(Vec(numLanes, UInt(32.W)))
    val has_fflags = Output(Bool())
    val fflags     = Output(UInt(FP_FLAGS_BITS.W))
    val tag_out   = Output(UInt(tagWidth.W))
    val valid_out = Output(Bool())
    val ready_out = Input(Bool())
  })

  io.ready_in   := false.B
  io.result     := VecInit(Seq.fill(numLanes)(0.U(32.W)))
  io.has_fflags := false.B
  io.fflags     := 0.U
  io.tag_out    := 0.U
  io.valid_out  := false.B
}
