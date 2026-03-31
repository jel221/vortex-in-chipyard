// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_alu_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Top-level ALU unit.
 *
 *  Corresponds to VX_alu_unit.sv.
 *
 *  The SV module:
 *    1. Receives dispatched instructions via ISSUE_WIDTH dispatch interfaces.
 *    2. Routes them through a VX_dispatch_unit (which handles partial-bandwidth
 *       lane-packing) into BLOCK_SIZE=NUM_ALU_BLOCKS per-block execute interfaces.
 *    3. Each block contains:
 *         - An AluInt integer unit
 *         - Optionally an AluMuldiv multiply/divide unit (EXT_M_ENABLE)
 *       with a PE switch that selects the appropriate sub-unit.
 *    4. Results are gathered back through VX_gather_unit into ISSUE_WIDTH
 *       commit interfaces.
 *
 *  In this Chisel translation we flatten the dispatch/gather units into
 *  direct wiring.  The PE switch becomes a priority-arbitrated mux on the
 *  result side.  For simplicity, when EXT_M_ENABLED=1 we include AluMuldiv
 *  alongside AluInt in every block; the ALU_TYPE_MULDIV op_type field
 *  selects which sub-unit accepts each instruction.
 *
 *  Constants (from VortexConfigConstants / VortexGPUPkg):
 *    BLOCK_SIZE = NUM_ALU_BLOCKS = ISSUE_WIDTH  (typically 1)
 *    NUM_LANES  = NUM_ALU_LANES  = SIMD_WIDTH   (typically 4)
 */
class AluUnit extends Module {

  private val blockSize  = NUM_ALU_BLOCKS
  private val numLanes   = NUM_ALU_LANES
  private val pidBits    = math.max(0, log2Ceil(math.max(1, NUM_THREADS / numLanes)))
  private val pidWidth   = math.max(1, pidBits)

  val io = IO(new Bundle {
    // Dispatch interfaces (one per ISSUE_WIDTH slot) – slave
    val dispatch_valid   = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_ready   = Output(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_uuid    = Input(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val dispatch_wid     = Input(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
    val dispatch_tmask   = Input(Vec(ISSUE_WIDTH, UInt(numLanes.W)))
    val dispatch_PC      = Input(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val dispatch_rd      = Input(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val dispatch_wb      = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_pid     = Input(Vec(ISSUE_WIDTH, UInt(pidWidth.W)))
    val dispatch_sop     = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_eop     = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_op_type = Input(Vec(ISSUE_WIDTH, UInt(INST_ALU_BITS.W)))
    val dispatch_op_args = Input(Vec(ISSUE_WIDTH, UInt(INST_ARGS_BITS.W)))
    val dispatch_rs1_data = Input(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))
    val dispatch_rs2_data = Input(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))
    val dispatch_rs3_data = Input(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))

    // Commit interfaces (one per ISSUE_WIDTH slot) – master
    val commit_valid  = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_ready  = Input(Vec(ISSUE_WIDTH, Bool()))
    val commit_uuid   = Output(Vec(ISSUE_WIDTH, UInt(UUID_WIDTH.W)))
    val commit_wid    = Output(Vec(ISSUE_WIDTH, UInt(NW_WIDTH.W)))
    val commit_tmask  = Output(Vec(ISSUE_WIDTH, UInt(numLanes.W)))
    val commit_PC     = Output(Vec(ISSUE_WIDTH, UInt(PC_BITS.W)))
    val commit_rd     = Output(Vec(ISSUE_WIDTH, UInt(NUM_REGS_BITS.W)))
    val commit_wb     = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_pid    = Output(Vec(ISSUE_WIDTH, UInt(pidWidth.W)))
    val commit_sop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_eop    = Output(Vec(ISSUE_WIDTH, Bool()))
    val commit_data   = Output(Vec(ISSUE_WIDTH, Vec(numLanes, UInt(XLEN.W))))

    // Branch control (one per ALU block) – master
    val branch_valid  = Output(Vec(blockSize, Bool()))
    val branch_wid    = Output(Vec(blockSize, UInt(NW_WIDTH.W)))
    val branch_taken  = Output(Vec(blockSize, Bool()))
    val branch_dest   = Output(Vec(blockSize, UInt(PC_BITS.W)))
  })

  // Instantiate one block per NUM_ALU_BLOCKS
  // For ISSUE_WIDTH == blockSize, each dispatch slot maps 1:1 to a block.
  val aluIntUnits    = Seq.tabulate(blockSize)(i => Module(new AluInt(numLanes, i)))
  val aluMuldivUnits = if (EXT_M_ENABLED != 0) {
    Some(Seq.tabulate(blockSize)(_ => Module(new AluMuldiv(numLanes))))
  } else None

  for (b <- 0 until blockSize) {
    // Dispatch slot index (1:1 mapping when blockSize == ISSUE_WIDTH)
    val dispIdx = if (ISSUE_WIDTH == blockSize) b else 0

    // Decode op_args to determine if this is a MULDIV op
    val opArgs    = io.dispatch_op_args(dispIdx).asTypeOf(new AluArgsBundle)
    val isMuldiv  = (opArgs.xtype === ALU_TYPE_MULDIV.U) && (EXT_M_ENABLED != 0).B

    // -----------------------------------------------------------------------
    // Integer ALU
    // -----------------------------------------------------------------------
    val intUnit = aluIntUnits(b)
    intUnit.io.execute_valid    := io.dispatch_valid(dispIdx) && !isMuldiv
    intUnit.io.execute_uuid     := io.dispatch_uuid(dispIdx)
    intUnit.io.execute_wid      := io.dispatch_wid(dispIdx)
    intUnit.io.execute_tmask    := io.dispatch_tmask(dispIdx)
    intUnit.io.execute_PC       := io.dispatch_PC(dispIdx)
    intUnit.io.execute_rd       := io.dispatch_rd(dispIdx)
    intUnit.io.execute_wb       := io.dispatch_wb(dispIdx)
    intUnit.io.execute_pid      := io.dispatch_pid(dispIdx)
    intUnit.io.execute_sop      := io.dispatch_sop(dispIdx)
    intUnit.io.execute_eop      := io.dispatch_eop(dispIdx)
    intUnit.io.execute_op_type  := io.dispatch_op_type(dispIdx)
    intUnit.io.execute_op_args  := io.dispatch_op_args(dispIdx)
    intUnit.io.execute_rs1_data := io.dispatch_rs1_data(dispIdx)
    intUnit.io.execute_rs2_data := io.dispatch_rs2_data(dispIdx)

    // Branch outputs
    io.branch_valid(b)  := intUnit.io.branch_valid
    io.branch_wid(b)    := intUnit.io.branch_wid
    io.branch_taken(b)  := intUnit.io.branch_taken
    io.branch_dest(b)   := intUnit.io.branch_dest

    // -----------------------------------------------------------------------
    // Muldiv (optional)
    // -----------------------------------------------------------------------
    aluMuldivUnits match {
      case Some(mdUnits) =>
        val mdUnit = mdUnits(b)
        mdUnit.io.execute_valid    := io.dispatch_valid(dispIdx) && isMuldiv
        mdUnit.io.execute_uuid     := io.dispatch_uuid(dispIdx)
        mdUnit.io.execute_wid      := io.dispatch_wid(dispIdx)
        mdUnit.io.execute_tmask    := io.dispatch_tmask(dispIdx)
        mdUnit.io.execute_PC       := io.dispatch_PC(dispIdx)
        mdUnit.io.execute_rd       := io.dispatch_rd(dispIdx)
        mdUnit.io.execute_wb       := io.dispatch_wb(dispIdx)
        mdUnit.io.execute_pid      := io.dispatch_pid(dispIdx)
        mdUnit.io.execute_sop      := io.dispatch_sop(dispIdx)
        mdUnit.io.execute_eop      := io.dispatch_eop(dispIdx)
        mdUnit.io.execute_op_type  := io.dispatch_op_type(dispIdx)
        mdUnit.io.execute_op_args  := io.dispatch_op_args(dispIdx)
        mdUnit.io.execute_rs1_data := io.dispatch_rs1_data(dispIdx)
        mdUnit.io.execute_rs2_data := io.dispatch_rs2_data(dispIdx)

        // Ready: when muldiv, give muldiv's ready; when int, give int's ready
        io.dispatch_ready(dispIdx) := Mux(isMuldiv,
          mdUnit.io.execute_ready,
          intUnit.io.execute_ready)

        // Result arbitration: priority to int result, then muldiv
        val intValid = intUnit.io.result_valid
        val mdValid  = mdUnit.io.result_valid

        // commit_ready feeds back to whichever is active
        intUnit.io.result_ready := io.commit_ready(dispIdx) && (!mdValid || intValid)
        mdUnit.io.result_ready  := io.commit_ready(dispIdx) && !intValid

        val selInt = intValid
        io.commit_valid(dispIdx) := intValid || mdValid
        io.commit_uuid(dispIdx)  := Mux(selInt, intUnit.io.result_uuid,  mdUnit.io.result_uuid)
        io.commit_wid(dispIdx)   := Mux(selInt, intUnit.io.result_wid,   mdUnit.io.result_wid)
        io.commit_tmask(dispIdx) := Mux(selInt, intUnit.io.result_tmask, mdUnit.io.result_tmask)
        io.commit_PC(dispIdx)    := Mux(selInt, intUnit.io.result_PC,    mdUnit.io.result_PC)
        io.commit_rd(dispIdx)    := Mux(selInt, intUnit.io.result_rd,    mdUnit.io.result_rd)
        io.commit_wb(dispIdx)    := Mux(selInt, intUnit.io.result_wb,    mdUnit.io.result_wb)
        io.commit_pid(dispIdx)   := Mux(selInt, intUnit.io.result_pid,   mdUnit.io.result_pid)
        io.commit_sop(dispIdx)   := Mux(selInt, intUnit.io.result_sop,   mdUnit.io.result_sop)
        io.commit_eop(dispIdx)   := Mux(selInt, intUnit.io.result_eop,   mdUnit.io.result_eop)
        io.commit_data(dispIdx)  := Mux(selInt, intUnit.io.result_data,  mdUnit.io.result_data)

      case None =>
        // No muldiv; int unit handles everything
        io.dispatch_ready(dispIdx) := intUnit.io.execute_ready
        intUnit.io.result_ready    := io.commit_ready(dispIdx)

        io.commit_valid(dispIdx) := intUnit.io.result_valid
        io.commit_uuid(dispIdx)  := intUnit.io.result_uuid
        io.commit_wid(dispIdx)   := intUnit.io.result_wid
        io.commit_tmask(dispIdx) := intUnit.io.result_tmask
        io.commit_PC(dispIdx)    := intUnit.io.result_PC
        io.commit_rd(dispIdx)    := intUnit.io.result_rd
        io.commit_wb(dispIdx)    := intUnit.io.result_wb
        io.commit_pid(dispIdx)   := intUnit.io.result_pid
        io.commit_sop(dispIdx)   := intUnit.io.result_sop
        io.commit_eop(dispIdx)   := intUnit.io.result_eop
        io.commit_data(dispIdx)  := intUnit.io.result_data
    }
  }
}
