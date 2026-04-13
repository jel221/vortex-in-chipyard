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
// Translated from VX_decode.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_decode – pure-combinational RISC-V instruction decoder.
 *
 *  Mirrors VX_decode.sv faithfully.  The module is combinational:
 *  the only register is the output elastic buffer (size=0 → pass-through,
 *  matching VX_decode.sv's VX_elastic_buffer #(.SIZE(0))).
 *
 *  Ports use flat signals rather than Bundle interfaces so this module
 *  can be wired by the parent (VX_core) without depending on the
 *  interface wrappers defined in PipelineBundles.scala.
 */
class VxDecode extends Module {

  // -------------------------------------------------------------------------
  // Derived widths
  // -------------------------------------------------------------------------
  private val outDataW = (new DecodeBundle).getWidth

  val io = IO(new Bundle {
    // Fetch interface (slave)
    val fetch_valid = Input(Bool())
    val fetch_ready = Output(Bool())
    val fetch_uuid  = Input(UInt(UUID_WIDTH.W))
    val fetch_wid   = Input(UInt(NW_WIDTH.W))
    val fetch_tmask = Input(UInt(NUM_THREADS.W))
    val fetch_PC    = Input(UInt(PC_BITS.W))
    val fetch_instr = Input(UInt(32.W))

    // Decode interface (master)
    val decode_valid   = Output(Bool())
    val decode_ready   = Input(Bool())
    val decode_uuid    = Output(UInt(UUID_WIDTH.W))
    val decode_wid     = Output(UInt(NW_WIDTH.W))
    val decode_tmask   = Output(UInt(NUM_THREADS.W))
    val decode_PC      = Output(UInt(PC_BITS.W))
    val decode_ex_type = Output(UInt(EX_BITS.W))
    val decode_op_type = Output(UInt(INST_OP_BITS.W))
    val decode_op_args = Output(UInt(INST_ARGS_BITS.W))
    val decode_wb      = Output(Bool())
    val decode_used_rs = Output(UInt(NUM_SRC_OPDS.W))
    val decode_rd      = Output(UInt(NUM_REGS_BITS.W))
    val decode_rs1     = Output(UInt(NUM_REGS_BITS.W))
    val decode_rs2     = Output(UInt(NUM_REGS_BITS.W))
    val decode_rs3     = Output(UInt(NUM_REGS_BITS.W))

    // Decode-to-scheduler feedback
    val decode_sched_valid  = Output(Bool())
    val decode_sched_unlock = Output(Bool())
    val decode_sched_wid    = Output(UInt(NW_WIDTH.W))
  })

  // -------------------------------------------------------------------------
  // Instruction field extraction
  // -------------------------------------------------------------------------
  val instr  = io.fetch_instr
  val opcode = instr(6, 0)
  val funct2 = instr(26, 25)
  val funct3 = instr(14, 12)
  val funct5 = instr(31, 27)
  val funct7 = instr(31, 25)
  val u_12   = instr(31, 20)

  val rd  = instr(11, 7)
  val rs1 = instr(19, 15)
  val rs2 = instr(24, 20)
  val rs3 = instr(31, 27)   // same bits as funct5

  // Shift-type immediate detection: is_itype_sh = funct3[0] && ~funct3[1]
  val is_itype_sh = funct3(0) && !funct3(1)

  // FPU CSR detection: u_12 <= VX_CSR_FCSR
  val is_fpu_csr = u_12 <= VX_CSR_FCSR.U

  // Immediates
  val ui_imm  = instr(31, 12)  // 20-bit upper immediate
  // RV32: i_imm uses 5-bit shift amount for shift instructions
  val i_imm   = Mux(is_itype_sh, Cat(0.U(7.W), instr(24, 20)), u_12)
  val s_imm   = Cat(funct7, rd)
  val b_imm   = Cat(instr(31), instr(7), instr(30, 25), instr(11, 8), 0.U(1.W))
  val jal_imm = Cat(instr(31), instr(19, 12), instr(20), instr(30, 21), 0.U(1.W))

  // Sign extension helpers
  def sext(width: Int, v: UInt): UInt = v.asSInt.pad(width).asUInt

  // -------------------------------------------------------------------------
  // r_type: ALU op from funct3 (+funct7[5] for SUB/SRA)
  // -------------------------------------------------------------------------
  val r_type = Wire(UInt(INST_ALU_BITS.W))
  r_type := MuxLookup(funct3, INST_ALU_ADD.U)(Seq(
    0.U -> Mux(opcode(5) && funct7(5), INST_ALU_SUB.U, INST_ALU_ADD.U),
    1.U -> INST_ALU_SLL.U,
    2.U -> INST_ALU_SLT.U,
    3.U -> INST_ALU_SLTU.U,
    4.U -> INST_ALU_XOR.U,
    5.U -> Mux(funct7(5), INST_ALU_SRA.U, INST_ALU_SRL.U),
    6.U -> INST_ALU_OR.U,
    7.U -> INST_ALU_AND.U
  ))

  // -------------------------------------------------------------------------
  // b_type: branch op from funct3
  // -------------------------------------------------------------------------
  val b_type = Wire(UInt(INST_BR_BITS.W))
  b_type := MuxLookup(funct3, INST_BR_BEQ.U)(Seq(
    0.U -> INST_BR_BEQ.U,
    1.U -> INST_BR_BNE.U,
    4.U -> INST_BR_BLT.U,
    5.U -> INST_BR_BGE.U,
    6.U -> INST_BR_BLTU.U,
    7.U -> INST_BR_BGEU.U
  ))

  // -------------------------------------------------------------------------
  // s_type: SYSTEM instruction sub-type (ECALL/EBREAK/xRET)
  // -------------------------------------------------------------------------
  val s_type = Wire(UInt(INST_OP_BITS.W))
  s_type := MuxLookup(u_12, 0.U)(Seq(
    0x000.U -> INST_BR_ECALL.U,
    0x001.U -> INST_BR_EBREAK.U,
    0x002.U -> INST_BR_URET.U,
    0x102.U -> INST_BR_SRET.U,
    0x302.U -> INST_BR_MRET.U
  ))

  // -------------------------------------------------------------------------
  // m_type: M-extension multiply/divide from funct3
  // EXT_M_ENABLE is always true in this translation (EXT_M_ENABLED=1)
  // -------------------------------------------------------------------------
  val m_type = Wire(UInt(INST_M_BITS.W))
  m_type := MuxLookup(funct3, INST_M_MUL.U)(Seq(
    0.U -> INST_M_MUL.U,
    1.U -> INST_M_MULH.U,
    2.U -> INST_M_MULHSU.U,
    3.U -> INST_M_MULHU.U,
    4.U -> INST_M_DIV.U,
    5.U -> INST_M_DIVU.U,
    6.U -> INST_M_REM.U,
    7.U -> INST_M_REMU.U
  ))

  // -------------------------------------------------------------------------
  // Main combinational decode
  //
  // We represent op_args as a flat UInt (INST_ARGS_BITS wide) and pack fields
  // by position, matching the SV packed union layout:
  //
  //   alu_args_t  [INST_ARGS_BITS-1:0] =
  //     { use_PC(1), use_imm(1), is_w(1), xtype(ALU_TYPE_BITS), imm(XLEN) }
  //     bit positions:  [36] [35] [34] [33:32] [31:0]   (for XLEN=32)
  //
  //   lsu_args_t  [INST_ARGS_BITS-1:0] =
  //     { padding, is_store(1), is_float(1), offset(12) }
  //     bit positions: upper bits, [13], [12], [11:0]
  //
  //   csr_args_t  [INST_ARGS_BITS-1:0] =
  //     { padding, use_imm(1), addr(12), imm(5) }
  //
  //   wctl_args_t [INST_ARGS_BITS-1:0] =
  //     { padding, is_neg(1) }
  //
  //   fpu_args_t  [INST_ARGS_BITS-1:0] =
  //     { padding, frm(3), fmt(2) }
  //
  // Helper functions to build each args word:
  // -------------------------------------------------------------------------

  // ALU args: { use_PC, use_imm, is_w, xtype[1:0], imm[XLEN-1:0] }
  def mkAluArgs(usePC: Bool, useImm: Bool, isW: Bool,
                xtype: UInt, imm: UInt): UInt = {
    Cat(usePC, useImm, isW, xtype(ALU_TYPE_BITS-1,0), imm(XLEN-1,0))
  }

  // LSU args: { padding, is_store, is_float, offset[11:0] }
  def mkLsuArgs(isStore: Bool, isFloat: Bool, offset: UInt): UInt = {
    val padW = INST_ARGS_BITS - 1 - 1 - OFFSET_BITS
    Cat(0.U(padW.W), isStore, isFloat, offset(OFFSET_BITS-1, 0))
  }

  // CSR args: { padding, use_imm, addr[11:0], imm[4:0] }
  def mkCsrArgs(useImm: Bool, addr: UInt, imm: UInt): UInt = {
    val padW = INST_ARGS_BITS - 1 - VX_CSR_ADDR_BITS - 5
    Cat(0.U(padW.W), useImm, addr(VX_CSR_ADDR_BITS-1,0), imm(4,0))
  }

  // Wctl args: { padding, is_neg }
  def mkWctlArgs(isNeg: Bool): UInt = {
    val padW = INST_ARGS_BITS - 1
    Cat(0.U(padW.W), isNeg)
  }

  // FPU args: { padding, frm[2:0], fmt[1:0] }
  def mkFpuArgs(frm: UInt, fmt: UInt): UInt = {
    val padW = INST_ARGS_BITS - INST_FRM_BITS - INST_FMT_BITS
    Cat(0.U(padW.W), frm(INST_FRM_BITS-1,0), fmt(INST_FMT_BITS-1,0))
  }

  // Unified register number: { rtype[0], idx[4:0] }
  def makeRegNum(rtype: Bool, idx: UInt): UInt = Cat(rtype, idx(4,0))
  def iReg(idx: UInt): UInt = makeRegNum(false.B, idx)
  def fReg(idx: UInt): UInt = makeRegNum(true.B,  idx)

  // -------------------------------------------------------------------------
  // Combinational decode outputs
  // -------------------------------------------------------------------------
  val ex_type   = Wire(UInt(EX_BITS.W))
  val op_type   = Wire(UInt(INST_OP_BITS.W))
  val op_args   = Wire(UInt(INST_ARGS_BITS.W))
  val rd_v      = Wire(UInt(NUM_REGS_BITS.W))
  val rs1_v     = Wire(UInt(NUM_REGS_BITS.W))
  val rs2_v     = Wire(UInt(NUM_REGS_BITS.W))
  val rs3_v     = Wire(UInt(NUM_REGS_BITS.W))
  val use_rd    = Wire(Bool())
  val use_rs1   = Wire(Bool())
  val use_rs2   = Wire(Bool())
  val use_rs3   = Wire(Bool())
  val is_wstall = Wire(Bool())

  // Defaults (equivalent to SV 'x assignments – use 0 here for safety)
  ex_type   := 0.U
  op_type   := 0.U
  op_args   := 0.U
  rd_v      := 0.U
  rs1_v     := 0.U
  rs2_v     := 0.U
  rs3_v     := 0.U
  use_rd    := false.B
  use_rs1   := false.B
  use_rs2   := false.B
  use_rs3   := false.B
  is_wstall := false.B

  // Main opcode switch
  switch (opcode) {

    // -----------------------------------------------------------------------
    // INST_I – Immediate instructions (ADDI, SLTI, …, SLLI, SRLI, SRAI)
    // -----------------------------------------------------------------------
    is (INST_I.U) {
      ex_type := EX_ALU.U
      op_type := r_type
      op_args := mkAluArgs(false.B, true.B, false.B, ALU_TYPE_ARITH.U,
                           sext(XLEN, i_imm))
      rd_v    := iReg(rd);   use_rd  := true.B
      rs1_v   := iReg(rs1);  use_rs1 := true.B
    }

    // -----------------------------------------------------------------------
    // INST_R – Register instructions (ADD, SUB, SLL, …, MUL, DIV, CZERO)
    // -----------------------------------------------------------------------
    is (INST_R.U) {
      ex_type := EX_ALU.U
      op_args := mkAluArgs(false.B, false.B, false.B, ALU_TYPE_ARITH.U, 0.U)
      rd_v    := iReg(rd);   use_rd  := true.B
      rs1_v   := iReg(rs1);  use_rs1 := true.B
      rs2_v   := iReg(rs2);  use_rs2 := true.B
      // funct7 determines whether it's M-ext or Zicond or standard R-type
      when (funct7 === INST_R_F7_MUL.U) {
        // EXT_M_ENABLE: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU
        op_type := m_type
        op_args := mkAluArgs(false.B, false.B, false.B, ALU_TYPE_MULDIV.U, 0.U)
      } .elsewhen (funct7 === INST_R_F7_ZICOND.U) {
        // EXT_ZICOND_ENABLE: CZERO.EQZ, CZERO.NEZ
        op_type := Mux(funct3(1), INST_ALU_CZNE.U, INST_ALU_CZEQ.U)
        op_args := mkAluArgs(false.B, false.B, false.B, ALU_TYPE_ARITH.U, 0.U)
      } .otherwise {
        op_type := r_type
        op_args := mkAluArgs(false.B, false.B, false.B, ALU_TYPE_ARITH.U, 0.U)
      }
    }

    // -----------------------------------------------------------------------
    // INST_LUI – Load Upper Immediate
    // -----------------------------------------------------------------------
    is (INST_LUI.U) {
      ex_type := EX_ALU.U
      op_type := INST_ALU_LUI.U
      // imm = sign-extended { ui_imm[19], ui_imm[18:0], 12'h000 }
      // The SV sign-extends the 31-bit quantity { ui_imm[19:0], 12'h000 }[30:0]
      // to XLEN.  Since ui_imm[19] is bit 31 of the result and we have 32-bit
      // XLEN, the result is simply { ui_imm[19:0], 12'h000 } with no
      // additional sign extension needed (already XLEN wide).
      val lui_imm = Cat(ui_imm, 0.U(12.W))
      op_args := mkAluArgs(false.B, true.B, false.B, ALU_TYPE_ARITH.U, lui_imm)
      rd_v    := iReg(rd);  use_rd := true.B
    }

    // -----------------------------------------------------------------------
    // INST_AUIPC – Add Upper Immediate to PC
    // -----------------------------------------------------------------------
    is (INST_AUIPC.U) {
      ex_type := EX_ALU.U
      op_type := INST_ALU_AUIPC.U
      val auipc_imm = Cat(ui_imm, 0.U(12.W))
      op_args := mkAluArgs(true.B, true.B, false.B, ALU_TYPE_ARITH.U, auipc_imm)
      rd_v    := iReg(rd);  use_rd := true.B
    }

    // -----------------------------------------------------------------------
    // INST_JAL – Jump And Link
    // -----------------------------------------------------------------------
    is (INST_JAL.U) {
      ex_type   := EX_ALU.U
      op_type   := INST_BR_JAL.U
      op_args   := mkAluArgs(true.B, true.B, false.B, ALU_TYPE_BRANCH.U,
                             sext(XLEN, jal_imm))
      is_wstall := true.B
      rd_v      := iReg(rd);  use_rd := true.B
    }

    // -----------------------------------------------------------------------
    // INST_JALR – Jump And Link Register
    // -----------------------------------------------------------------------
    is (INST_JALR.U) {
      ex_type   := EX_ALU.U
      op_type   := INST_BR_JALR.U
      op_args   := mkAluArgs(false.B, true.B, false.B, ALU_TYPE_BRANCH.U,
                             sext(XLEN, u_12))
      is_wstall := true.B
      rd_v      := iReg(rd);   use_rd  := true.B
      rs1_v     := iReg(rs1);  use_rs1 := true.B
    }

    // -----------------------------------------------------------------------
    // INST_B – Branch instructions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
    // -----------------------------------------------------------------------
    is (INST_B.U) {
      ex_type   := EX_ALU.U
      op_type   := b_type
      op_args   := mkAluArgs(true.B, true.B, false.B, ALU_TYPE_BRANCH.U,
                             sext(XLEN, b_imm))
      is_wstall := true.B
      rs1_v     := iReg(rs1);  use_rs1 := true.B
      rs2_v     := iReg(rs2);  use_rs2 := true.B
    }

    // -----------------------------------------------------------------------
    // INST_FENCE – Fence instructions
    // -----------------------------------------------------------------------
    is (INST_FENCE.U) {
      ex_type := EX_LSU.U
      op_type := INST_LSU_FENCE.U
      op_args := mkLsuArgs(false.B, false.B, 0.U)
    }

    // -----------------------------------------------------------------------
    // INST_SYS – System instructions (CSR and ECALL/EBREAK/xRET)
    // -----------------------------------------------------------------------
    is (INST_SYS.U) {
      when (funct3(1, 0) =/= 0.U) {
        // CSR instructions (CSRRW/CSRRS/CSRRC and their immediate variants)
        ex_type   := EX_SFU.U
        op_type   := inst_sfu_csr(funct3)
        is_wstall := is_fpu_csr   // stall only for FPU CSRs
        rd_v      := iReg(rd);  use_rd := true.B
        when (funct3(2)) {
          // CSRRWI/CSRRSI/CSRRCI: immediate in rs1 field
          op_args := mkCsrArgs(true.B, u_12, rs1)
        } .otherwise {
          // CSRRW/CSRRS/CSRRC: register rs1
          op_args := mkCsrArgs(false.B, u_12, 0.U)
          rs1_v   := iReg(rs1);  use_rs1 := true.B
        }
      } .otherwise {
        // ECALL / EBREAK / xRET – treated as branch-class by ALU
        ex_type   := EX_ALU.U
        op_type   := s_type
        op_args   := mkAluArgs(true.B, true.B, false.B, ALU_TYPE_BRANCH.U, 4.U(XLEN.W))
        is_wstall := true.B
        rd_v      := iReg(rd);  use_rd := true.B
      }
    }

    // -----------------------------------------------------------------------
    // INST_L / INST_FL – Load instructions (integer and float)
    // EXT_F_ENABLE: INST_FL shares this case (opcode[2]=1 → float)
    // -----------------------------------------------------------------------
    is (INST_L.U, INST_FL.U) {
      ex_type := EX_LSU.U
      op_type := Cat(0.U(1.W), funct3)
      op_args := mkLsuArgs(false.B, opcode(2), u_12)
      rs1_v   := iReg(rs1);  use_rs1 := true.B
      // Destination register type depends on opcode[2] (float bit)
      when (opcode(2)) {
        rd_v  := fReg(rd)
      } .otherwise {
        rd_v  := iReg(rd)
      }
      use_rd := true.B
    }

    // -----------------------------------------------------------------------
    // INST_S / INST_FS – Store instructions (integer and float)
    // -----------------------------------------------------------------------
    is (INST_S.U, INST_FS.U) {
      ex_type := EX_LSU.U
      op_type := Cat(1.U(1.W), funct3)
      op_args := mkLsuArgs(true.B, opcode(2), s_imm)
      rs1_v   := iReg(rs1);  use_rs1 := true.B
      // Source register type depends on opcode[2]
      when (opcode(2)) {
        rs2_v := fReg(rs2)
      } .otherwise {
        rs2_v := iReg(rs2)
      }
      use_rs2 := true.B
    }

    // -----------------------------------------------------------------------
    // INST_FMADD / INST_FMSUB / INST_FNMSUB / INST_FNMADD
    // Fused Multiply-Add variants (EXT_F_ENABLE)
    // op_type = { 2'b00, 1'b1, opcode[3] }
    // fmt[0] = funct2[0]  (float/double)
    // fmt[1] = opcode[3] ^ opcode[2]  (SUB flag)
    // -----------------------------------------------------------------------
    is (INST_FMADD.U, INST_FMSUB.U, INST_FNMSUB.U, INST_FNMADD.U) {
      ex_type := EX_FPU.U
      op_type := Cat(0.U(2.W), 1.U(1.W), opcode(3))
      val fmt = Cat(opcode(3) ^ opcode(2), funct2(0))
      op_args := mkFpuArgs(funct3, fmt)
      rd_v    := fReg(rd);   use_rd  := true.B
      rs1_v   := fReg(rs1);  use_rs1 := true.B
      rs2_v   := fReg(rs2);  use_rs2 := true.B
      rs3_v   := fReg(rs3);  use_rs3 := true.B
    }

    // -----------------------------------------------------------------------
    // INST_FCI – Float common instructions (EXT_F_ENABLE)
    // -----------------------------------------------------------------------
    is (INST_FCI.U) {
      ex_type := EX_FPU.U
      // default fmt fields (overridden per-case below)
      val fmt_base = Cat(rs2(1), funct2(0))
      op_args := mkFpuArgs(funct3, fmt_base)

      switch (funct5) {

        // FADD (funct5=5'b00000), FSUB (5'b00001), FMUL (5'b00010)
        is (0.U, 1.U, 2.U) {
          // op_type = { 2'b00, 1'b0, funct5[1] }
          // fmt[1] = funct5[0]  (SUB flag), fmt[0] = funct2[0]
          op_type := Cat(0.U(2.W), 0.U(1.W), funct5(1))
          val fmt = Cat(funct5(0), funct2(0))
          op_args := mkFpuArgs(funct3, fmt)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
          rs2_v   := fReg(rs2);  use_rs2 := true.B
        }

        // FSGNJ / FSGNJN / FSGNJX (funct5=5'b00100)
        // NCP: frm = funct3[1:0]
        is (4.U) {
          op_type := INST_FPU_MISC.U
          op_args := mkFpuArgs(Cat(0.U(1.W), funct3(1,0)), fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
          rs2_v   := fReg(rs2);  use_rs2 := true.B
        }

        // FMIN / FMAX (funct5=5'b00101)
        // NCP: FMIN=frm6, FMAX=frm7
        is (5.U) {
          op_type := INST_FPU_MISC.U
          val fminmax_frm = Mux(funct3(0), 7.U(INST_FRM_BITS.W), 6.U(INST_FRM_BITS.W))
          op_args := mkFpuArgs(fminmax_frm, fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
          rs2_v   := fReg(rs2);  use_rs2 := true.B
        }

        // FDIV (funct5=5'b00011)
        is (3.U) {
          op_type := INST_FPU_DIV.U
          op_args := mkFpuArgs(funct3, fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
          rs2_v   := fReg(rs2);  use_rs2 := true.B
        }

        // FSQRT (funct5=5'b01011)
        is (11.U) {
          op_type := INST_FPU_SQRT.U
          op_args := mkFpuArgs(funct3, fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
        }

        // FCMP – FEQ/FLT/FLE (funct5=5'b10100)
        is (20.U) {
          op_type := INST_FPU_CMP.U
          op_args := mkFpuArgs(funct3, fmt_base)
          rd_v    := iReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
          rs2_v   := fReg(rs2);  use_rs2 := true.B
        }

        // FCVT.W.X / FCVT.WU.X (funct5=5'b11000)  – F→I or F→U
        is (24.U) {
          op_type := Mux(rs2(0), INST_FPU_F2U.U, INST_FPU_F2I.U)
          op_args := mkFpuArgs(funct3, fmt_base)
          rd_v    := iReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
        }

        // FCVT.X.W / FCVT.X.WU (funct5=5'b11010)  – I→F or U→F
        is (26.U) {
          op_type := Mux(rs2(0), INST_FPU_U2F.U, INST_FPU_I2F.U)
          op_args := mkFpuArgs(funct3, fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := iReg(rs1);  use_rs1 := true.B
        }

        // FCLASS / FMV.X.W (funct5=5'b11100)
        is (28.U) {
          op_type := INST_FPU_MISC.U
          val misc_frm = Mux(funct3(0), 3.U(INST_FRM_BITS.W), 4.U(INST_FRM_BITS.W))
          op_args := mkFpuArgs(misc_frm, fmt_base)
          rd_v    := iReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
        }

        // FMV.W.X (funct5=5'b11110)
        is (30.U) {
          op_type := INST_FPU_MISC.U
          op_args := mkFpuArgs(5.U(INST_FRM_BITS.W), fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := iReg(rs1);  use_rs1 := true.B
        }

        // FCVT.S.D / FCVT.D.S (funct5=5'b01000)  – F2F conversion
        is (8.U) {
          op_type := INST_FPU_F2F.U
          op_args := mkFpuArgs(funct3, fmt_base)
          rd_v    := fReg(rd);   use_rd  := true.B
          rs1_v   := fReg(rs1);  use_rs1 := true.B
        }
      }
    }

    // -----------------------------------------------------------------------
    // INST_EXT1 – Custom-0 extension (GPU-specific instructions)
    // -----------------------------------------------------------------------
    is (INST_EXT1.U) {
      switch (funct7) {

        // funct7=7'h00: GPU warp-control instructions
        is (0.U) {
          ex_type   := EX_SFU.U
          is_wstall := true.B
          switch (funct3) {
            is (0.U) { // TMC
              op_type := INST_SFU_TMC.U
              op_args := mkWctlArgs(false.B)
              rs1_v   := iReg(rs1);  use_rs1 := true.B
            }
            is (1.U) { // WSPAWN
              op_type := INST_SFU_WSPAWN.U
              op_args := mkWctlArgs(false.B)
              rs1_v   := iReg(rs1);  use_rs1 := true.B
              rs2_v   := iReg(rs2);  use_rs2 := true.B
            }
            is (2.U) { // SPLIT
              op_type := INST_SFU_SPLIT.U
              op_args := mkWctlArgs(rs2(0))
              rs1_v   := iReg(rs1);  use_rs1 := true.B
              rd_v    := iReg(rd);   use_rd  := true.B
            }
            is (3.U) { // JOIN
              op_type := INST_SFU_JOIN.U
              op_args := mkWctlArgs(false.B)
              rs1_v   := iReg(rs1);  use_rs1 := true.B
            }
            is (4.U) { // BAR
              op_type := INST_SFU_BAR.U
              op_args := mkWctlArgs(false.B)
              rs1_v   := iReg(rs1);  use_rs1 := true.B
              rs2_v   := iReg(rs2);  use_rs2 := true.B
            }
            is (5.U) { // PRED
              op_type := INST_SFU_PRED.U
              op_args := mkWctlArgs(rd(0))
              rs1_v   := iReg(rs1);  use_rs1 := true.B
              rs2_v   := iReg(rs2);  use_rs2 := true.B
            }
          }
        }

        // funct7=7'h01: VOTE / SHFL (ALU_TYPE_OTHER)
        is (1.U) {
          ex_type := EX_ALU.U
          op_type := funct3
          op_args := mkAluArgs(false.B, false.B, false.B, ALU_TYPE_OTHER.U, 0.U)
          rd_v    := iReg(rd);   use_rd  := true.B
          rs1_v   := iReg(rs1);  use_rs1 := true.B
          when (funct3(2)) {
            rs2_v  := iReg(rs2);  use_rs2 := true.B
          }
        }

        // funct7=7'h02: TCU (EXT_TCU_ENABLE=0, so this arm is dead in default config)
        // Included for completeness; EX_TCU == EX_FPU == 3 when TCU disabled.
        is (2.U) {
          // Only funct3=3'h0 (WMMA) is currently defined
          when (funct3 === 0.U) {
            ex_type := EX_TCU.U
            op_type := 0.U   // INST_TCU_WMMA = 0
            op_args := 0.U
            rd_v    := iReg(rd);   use_rd  := true.B
            rs1_v   := iReg(rs1);  use_rs1 := true.B
            rs2_v   := iReg(rs2);  use_rs2 := true.B
            rs3_v   := iReg(rs3);  use_rs3 := true.B
          }
        }
      }
    }
  }

  // -------------------------------------------------------------------------
  // Writeback enable: must use rd AND rd != x0 (r0)
  // -------------------------------------------------------------------------
  val wb      = use_rd && (rd_v =/= 0.U)
  val used_rs = Cat(use_rs3, use_rs2, use_rs1)

  // -------------------------------------------------------------------------
  // Output elastic buffer (SIZE=0 → pass-through, matching SV instantiation)
  // -------------------------------------------------------------------------
  val buf = Module(new VXElasticBuffer(dataw = outDataW, size = 0))

  // Pack fetch passthrough data and decoded fields into a single bus
  val dataIn = Wire(new DecodeBundle)
  dataIn.uuid    := io.fetch_uuid
  dataIn.wid     := io.fetch_wid
  dataIn.tmask   := io.fetch_tmask
  dataIn.PC      := io.fetch_PC
  dataIn.ex_type := ex_type
  dataIn.op_type := op_type
  dataIn.op_args.bits := op_args
  dataIn.wb      := wb
  dataIn.used_rs := used_rs
  dataIn.rd      := rd_v
  dataIn.rs1     := rs1_v
  dataIn.rs2     := rs2_v
  dataIn.rs3     := rs3_v

  buf.io.valid_in := io.fetch_valid
  io.fetch_ready  := buf.io.ready_in
  buf.io.data_in  := dataIn.asUInt
  buf.io.ready_out := io.decode_ready

  val dataOut = buf.io.data_out.asTypeOf(new DecodeBundle)
  io.decode_valid   := buf.io.valid_out
  io.decode_uuid    := dataOut.uuid
  io.decode_wid     := dataOut.wid
  io.decode_tmask   := dataOut.tmask
  io.decode_PC      := dataOut.PC
  io.decode_ex_type := dataOut.ex_type
  io.decode_op_type := dataOut.op_type
  io.decode_op_args := dataOut.op_args.bits
  io.decode_wb      := dataOut.wb
  io.decode_used_rs := dataOut.used_rs
  io.decode_rd      := dataOut.rd
  io.decode_rs1     := dataOut.rs1
  io.decode_rs2     := dataOut.rs2
  io.decode_rs3     := dataOut.rs3

  // -------------------------------------------------------------------------
  // Decode-to-scheduler feedback
  // -------------------------------------------------------------------------
  val fetch_fire = io.fetch_valid && io.fetch_ready
  io.decode_sched_valid  := fetch_fire
  io.decode_sched_wid    := io.fetch_wid
  io.decode_sched_unlock := !is_wstall
}
