// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_tcu_pkg.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_tcu_pkg: TCU (Tensor Core Unit) dimension and constant definitions.
 *
 *  Mirrors the localparam computations in VX_tcu_pkg.sv.
 *  TCU_NT = NUM_THREADS, TCU_NR = 8, TCU_DP = 0 (default config).
 *
 *  Tile and block dimension formulas are preserved verbatim from the SV source.
 */
object TcuPkg {

  // Configuration base parameters
  val TCU_NT: Int = NUM_THREADS   // = 4
  val TCU_NR: Int = 8
  val TCU_DP: Int = 0

  // Supported data type IDs (fmt field values)
  val TCU_FP32_ID: Int = 0
  val TCU_FP16_ID: Int = 1
  val TCU_BF16_ID: Int = 2
  val TCU_I32_ID:  Int = 8
  val TCU_I8_ID:   Int = 9
  val TCU_U8_ID:   Int = 10
  val TCU_I4_ID:   Int = 11
  val TCU_U4_ID:   Int = 12

  // ---------------------------------------------------------------------------
  // Tile dimensions
  // TCU_TILE_CAP = TCU_NT * TCU_NR
  // TCU_LG_TILE_CAP = log2(TCU_TILE_CAP)
  // TCU_TILE_EN = TCU_LG_TILE_CAP / 2  (integer division)
  // TCU_TILE_EM = TCU_LG_TILE_CAP - TCU_TILE_EN
  // TCU_TILE_M = 1 << TCU_TILE_EM
  // TCU_TILE_N = 1 << TCU_TILE_EN
  // TCU_TILE_K = TCU_TILE_CAP / max(TCU_TILE_M, TCU_TILE_N)
  // ---------------------------------------------------------------------------
  val TCU_TILE_CAP:    Int = TCU_NT * TCU_NR
  val TCU_LG_TILE_CAP: Int = log2Ceil(TCU_TILE_CAP)
  val TCU_TILE_EN:     Int = TCU_LG_TILE_CAP / 2
  val TCU_TILE_EM:     Int = TCU_LG_TILE_CAP - TCU_TILE_EN
  val TCU_TILE_M:      Int = 1 << TCU_TILE_EM
  val TCU_TILE_N:      Int = 1 << TCU_TILE_EN
  val TCU_TILE_K:      Int = TCU_TILE_CAP / math.max(TCU_TILE_M, TCU_TILE_N)

  // ---------------------------------------------------------------------------
  // Block (TC) dimensions
  // TCU_BLOCK_CAP = TCU_NT
  // TCU_LG_BLOCK_CAP = log2(TCU_BLOCK_CAP)
  // TCU_BLOCK_EN = TCU_LG_BLOCK_CAP / 2
  // TCU_BLOCK_EM = TCU_LG_BLOCK_CAP - TCU_BLOCK_EN
  // TCU_TC_M = 1 << TCU_BLOCK_EM
  // TCU_TC_N = 1 << TCU_BLOCK_EN
  // TCU_TC_K = TCU_DP != 0 ? TCU_DP : TCU_BLOCK_CAP / max(TCU_TC_M, TCU_TC_N)
  // ---------------------------------------------------------------------------
  val TCU_BLOCK_CAP:    Int = TCU_NT
  val TCU_LG_BLOCK_CAP: Int = log2Ceil(TCU_BLOCK_CAP)
  val TCU_BLOCK_EN:     Int = TCU_LG_BLOCK_CAP / 2
  val TCU_BLOCK_EM:     Int = TCU_LG_BLOCK_CAP - TCU_BLOCK_EN
  val TCU_TC_M:         Int = 1 << TCU_BLOCK_EM
  val TCU_TC_N:         Int = 1 << TCU_BLOCK_EN
  val TCU_TC_K:         Int =
    if (TCU_DP != 0) TCU_DP
    else TCU_BLOCK_CAP / math.max(TCU_TC_M, TCU_TC_N)

  // Step counts (how many TC micro-tiles span one WMMA tile)
  val TCU_M_STEPS: Int = TCU_TILE_M / TCU_TC_M
  val TCU_N_STEPS: Int = TCU_TILE_N / TCU_TC_N
  val TCU_K_STEPS: Int = TCU_TILE_K / TCU_TC_K

  // A micro-tiling
  val TCU_A_BLOCK_SIZE: Int = TCU_TC_M * TCU_TC_K
  val TCU_A_SUB_BLOCKS: Int = TCU_BLOCK_CAP / TCU_A_BLOCK_SIZE

  // B micro-tiling
  val TCU_B_BLOCK_SIZE: Int = TCU_TC_K * TCU_TC_N
  val TCU_B_SUB_BLOCKS: Int = TCU_BLOCK_CAP / TCU_B_BLOCK_SIZE

  // Register base addresses
  val TCU_NRB: Int = (TCU_TILE_N * TCU_TILE_K) / TCU_NT
  val TCU_RA:  Int = 0
  val TCU_RB:  Int = if (TCU_NRB == 4) 28 else 10
  val TCU_RC:  Int = if (TCU_NRB == 4) 10 else 24

  // Total number of WMMA micro-ops per WMMA instruction
  val TCU_UOPS: Int = TCU_M_STEPS * TCU_N_STEPS * TCU_K_STEPS

  // TCU op_type encodings
  val INST_TCU_WMMA: Int = 0   // WMMA (only TCU instruction)
  val INST_TCU_BITS: Int = 1   // Width of TCU op field

  // TCU execute/result packed-struct widths (tcu_exe_t / tcu_res_t)
  // Sized to hold the TCU operand tiles; exact value depends on XLEN/config.
  val TCU_EXE_BITS: Int = 64   // placeholder width for tcu_exe_t
  val TCU_RES_BITS: Int = 32   // placeholder width for tcu_res_t

  // Dispatch/commit data widths used by the TCU unit stub
  val DISPATCH_DATA_BITS: Int = 64
  val COMMIT_DATA_BITS:   Int = 32
}
