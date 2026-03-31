// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_dcr_data.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** DCR (Device Configuration Register) data block.
 *
 *  Corresponds to VX_dcr_data.sv.
 *
 *  Receives posted writes on the DCR bus and latches them into a
 *  base_dcrs_t register bank.  The output is a combinational view of
 *  the current register state.
 *
 *  For XLEN=32 (assumed), startup_addr and startup_arg are 32-bit
 *  registers, and the XLEN_64 high-half registers are absent.
 *
 *  DCR address constants (from VortexConfigConstants):
 *    VX_DCR_BASE_STARTUP_ADDR0 = 0x001
 *    VX_DCR_BASE_STARTUP_ARG0  = 0x003
 *    VX_DCR_BASE_MPM_CLASS     = 0x005
 */
class DcrData extends Module {

  val io = IO(new Bundle {
    // DCR bus (slave)
    val dcr_write_valid = Input(Bool())
    val dcr_write_addr  = Input(UInt(VX_DCR_ADDR_WIDTH.W))
    val dcr_write_data  = Input(UInt(VX_DCR_DATA_WIDTH.W))

    // Base DCR outputs
    val base_dcrs_startup_addr = Output(UInt(XLEN.W))
    val base_dcrs_startup_arg  = Output(UInt(XLEN.W))
    val base_dcrs_mpm_class    = Output(UInt(8.W))
  })

  // -------------------------------------------------------------------------
  // DCR register bank (base_dcrs_t)
  // -------------------------------------------------------------------------
  val startup_addr = RegInit(0.U(XLEN.W))
  val startup_arg  = RegInit(0.U(XLEN.W))
  val mpm_class    = RegInit(0.U(8.W))

  when (io.dcr_write_valid) {
    switch (io.dcr_write_addr) {
      // VX_DCR_BASE_STARTUP_ADDR0 = 0x001 — startup address bits [31:0]
      is (VX_DCR_BASE_STARTUP_ADDR0.U) {
        startup_addr := io.dcr_write_data
      }
      // VX_DCR_BASE_STARTUP_ARG0 = 0x003 — startup argument bits [31:0]
      is (VX_DCR_BASE_STARTUP_ARG0.U) {
        startup_arg := io.dcr_write_data
      }
      // VX_DCR_BASE_MPM_CLASS = 0x005 — performance counter class (8 bits)
      is (VX_DCR_BASE_MPM_CLASS.U) {
        mpm_class := io.dcr_write_data(7, 0)
      }
    }
  }

  // -------------------------------------------------------------------------
  // Outputs
  // -------------------------------------------------------------------------
  io.base_dcrs_startup_addr := startup_addr
  io.base_dcrs_startup_arg  := startup_arg
  io.base_dcrs_mpm_class    := mpm_class
}
