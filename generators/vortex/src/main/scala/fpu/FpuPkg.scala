// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_fpu_pkg.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_fpu_pkg: FPU data-type and constant definitions.
 *
 *  Mirrors the packed-struct definitions in VX_fpu_pkg.sv:
 *    fclass_t  — floating-point classification flags (7 bits)
 *    fflags_t  — floating-point exception flags (5 bits: NV|DZ|OF|UF|NX)
 *
 *  fpu_exe_t / fpu_res_t from `DECL_EXECUTE_T / `DECL_RESULT_T are
 *  represented as their constituent flat fields used by VxFpuUnit.
 *
 *  FP_FLAGS_BITS = 5 (matches $bits(fflags_t) in SV).
 */
object FpuPkg {
  // Width of the fflags field (NV | DZ | OF | UF | NX)
  val FP_FLAGS_BITS: Int = 5

  // Bit indices within fflags (packed MSB→LSB: NV=4, DZ=3, OF=2, UF=1, NX=0)
  val FP_FLAG_NX: Int = 0   // Inexact
  val FP_FLAG_UF: Int = 1   // Underflow
  val FP_FLAG_OF: Int = 2   // Overflow
  val FP_FLAG_DZ: Int = 3   // Divide by zero
  val FP_FLAG_NV: Int = 4   // Invalid

  // Width of the fclass field
  val FCLASS_BITS: Int = 7

  // Data-bus widths used in FPU execute/result pipelines
  // fpu_exe_t: uuid + wid + tmask + PC + rd + pid + sop + eop + op_type + op_args + rs1/rs2/rs3_data
  val FPU_EXE_DATAW: Int = (
    UUID_WIDTH + NW_WIDTH + NUM_FPU_LANES + PC_BITS + NUM_REGS_BITS
      + log2Up(NUM_THREADS / NUM_FPU_LANES + 1)  // PID_WIDTH = UP(PID_BITS)
      + 1 + 1                                    // sop + eop
      + INST_FPU_BITS + INST_FRM_BITS + INST_FMT_BITS  // op_type + frm + fmt
      + 3 * NUM_FPU_LANES * XLEN                 // rs1/rs2/rs3 data
  )

  // fpu_res_t: uuid + wid + tmask + PC + rd + pid + sop + eop + data (no wb in result_t)
  val FPU_RES_DATAW: Int = (
    UUID_WIDTH + NW_WIDTH + NUM_FPU_LANES + PC_BITS + NUM_REGS_BITS
      + log2Up(NUM_THREADS / NUM_FPU_LANES + 1)  // PID_WIDTH
      + 1 + 1                                    // sop + eop
      + NUM_FPU_LANES * XLEN                     // result data
  )
}
