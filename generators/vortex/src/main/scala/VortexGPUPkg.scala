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
// Translated from VX_gpu_pkg.sv
// Assumes XLEN_32, EXT_F_ENABLE (REG_TYPES=2), EXT_M_ENABLE, EXT_ZICOND_ENABLE,
//         LMEM_ENABLE, ICACHE_ENABLE, DCACHE_ENABLE, no EXT_TCU_ENABLE,
//         no GBAR_ENABLE, NDEBUG not defined (UUID_WIDTH=44, PC_BITS=XLEN).

package vortex

import chisel3._
import chisel3.util._

object VortexGPUPkg {

  import VortexConfigConstants._

  // -------------------------------------------------------------------------
  // Utility mirrors (also available via VortexConfigConstants)
  // -------------------------------------------------------------------------

  /** Ensure at least 1: equivalent to `UP macro */
  def up(x: Int): Int = math.max(1, x)

  // -------------------------------------------------------------------------
  // Core / Warp / Thread bit widths  (localparam NC_BITS … NT_WIDTH)
  // -------------------------------------------------------------------------

  val NC_BITS:  Int = log2Ceil(NUM_CORES)     // log2Ceil(NUM_CORES)
  val NW_BITS:  Int = log2Ceil(NUM_WARPS)     // log2Ceil(NUM_WARPS)
  val NT_BITS:  Int = log2Ceil(NUM_THREADS)   // log2Ceil(NUM_THREADS)
  val NB_BITS:  Int = log2Ceil(NUM_BARRIERS)  // log2Ceil(NUM_BARRIERS)

  val NC_WIDTH: Int = up(NC_BITS)   // UP(NC_BITS)
  val NW_WIDTH: Int = up(NW_BITS)   // UP(NW_BITS)
  val NT_WIDTH: Int = up(NT_BITS)   // UP(NT_BITS)
  val NB_WIDTH: Int = up(NB_BITS)   // UP(NB_BITS)

  // XLEN in bytes
  val XLENB: Int = XLEN / 8   // XLEN / 8

  // -------------------------------------------------------------------------
  // Register file parameters
  // -------------------------------------------------------------------------

  val RV_REGS:      Int = 32   // Number of RISC-V integer registers
  val RV_REGS_BITS: Int = 5    // Bit width for RV register index

  val REG_TYPE_I: Int = 0   // Integer register type
  val REG_TYPE_F: Int = 1   // Floating-point register type

  // EXT_F_ENABLE is assumed: REG_TYPES = 2
  val REG_TYPES: Int     = 2   // EXT_F_ENABLE => 2 register files

  val NUM_REGS: Int      = REG_TYPES * RV_REGS   // Total logical registers

  val REG_TYPE_BITS: Int = log2Up(REG_TYPES)     // LOG2UP(REG_TYPES)
  val NUM_REGS_BITS: Int = log2Ceil(NUM_REGS)        // CLOG2(NUM_REGS)

  // Divergence stack size: UP(NUM_THREADS - 1), UP(CLOG2(DV_STACK_SIZE))
  val DV_STACK_SIZE:  Int = up(NUM_THREADS - 1)
  val DV_STACK_SIZEW: Int = up(log2Ceil(DV_STACK_SIZE))

  val PERF_CTR_BITS: Int = 44   // Performance counter width

  // -------------------------------------------------------------------------
  // SIMD parameters
  // -------------------------------------------------------------------------

  val SIMD_COUNT:    Int = NUM_THREADS / SIMD_WIDTH   // Number of SIMD groups per warp
  val SIMD_IDX_BITS: Int = log2Ceil(SIMD_COUNT)           // CLOG2(SIMD_COUNT)
  val SIMD_IDX_W:    Int = up(SIMD_IDX_BITS)            // UP(SIMD_IDX_BITS)

  // -------------------------------------------------------------------------
  // Operand collector parameters
  // -------------------------------------------------------------------------

  val NUM_OPCS_BITS: Int = log2Ceil(NUM_OPCS)   // CLOG2(NUM_OPCS)
  val NUM_OPCS_W:    Int = up(NUM_OPCS_BITS)  // UP(NUM_OPCS_BITS)

  // -------------------------------------------------------------------------
  // UUID / PC widths
  // NDEBUG is not defined (debug build): UUID_WIDTH=44, PC_BITS=XLEN
  // -------------------------------------------------------------------------

  val UUID_WIDTH: Int = 44    // UUID width (NDEBUG not defined)
  val PC_BITS:    Int = XLEN  // PC width (NDEBUG not defined: full XLEN)

  // -------------------------------------------------------------------------
  // Miscellaneous pipeline widths
  // -------------------------------------------------------------------------

  val OFFSET_BITS: Int     = 12   // 12-bit immediate offset field

  val NUM_SRC_OPDS:   Int = 3             // Number of source operands
  val SRC_OPD_BITS:   Int = log2Ceil(NUM_SRC_OPDS)
  val SRC_OPD_WIDTH:  Int = up(SRC_OPD_BITS)

  // Number of sockets: UP(NUM_CORES / SOCKET_SIZE)
  val NUM_SOCKETS: Int = up(NUM_CORES / SOCKET_SIZE)   // UP(NUM_CORES / SOCKET_SIZE)

  // Memory request flags
  val MEM_REQ_FLAG_FLUSH: Int  = 0   // Flush flag bit position
  val MEM_REQ_FLAG_IO:    Int  = 1   // I/O flag bit position
  val MEM_REQ_FLAG_LOCAL: Int  = 2   // Local (LMEM) flag bit position — optional last flag
  // LMEM_ENABLED = 1 (assumed), so MEM_FLAGS_WIDTH = MEM_REQ_FLAG_LOCAL + 1 = 3
  val MEM_FLAGS_WIDTH: Int     = MEM_REQ_FLAG_LOCAL + LMEM_ENABLED

  // DCR interface widths (from VX_types.vh)
  val VX_DCR_ADDR_WIDTH: Int   = VX_DCR_ADDR_BITS   // VX_DCR_ADDR_BITS = 12
  val VX_DCR_DATA_WIDTH: Int   = 32                  // DCR data width always 32 bits

  // Stall timeout: 100000 * (1 ** (L2_ENABLED + L3_ENABLED))  => 100000 (both disabled)
  val STALL_TIMEOUT: Int       = 100000   // STALL_TIMEOUT

  // -------------------------------------------------------------------------
  // Execution unit type encodings  (EX_ALU … EX_TCU)
  // EXT_F_ENABLED=1 => EX_FPU = EX_SFU + 1 = 3
  // EXT_TCU_ENABLED=0 => EX_TCU = EX_FPU + 0 = 3
  // -------------------------------------------------------------------------

  val EX_ALU: Int = 0                              // ALU execution unit
  val EX_LSU: Int = 1                              // LSU execution unit
  val EX_SFU: Int = 2                              // SFU execution unit
  val EX_FPU: Int = EX_SFU + EXT_F_ENABLED        // FPU execution unit (EXT_F_ENABLED=1 => 3)
  val EX_TCU: Int = EX_FPU + EXT_TCU_ENABLED      // TCU execution unit (EXT_TCU_ENABLED=0 => 3)

  val NUM_EX_UNITS: Int = EX_TCU + 1              // Total number of execution unit types
  val EX_BITS:      Int = log2Ceil(NUM_EX_UNITS)      // CLOG2(NUM_EX_UNITS)
  val EX_WIDTH:     Int = up(EX_BITS)               // UP(EX_BITS)

  // SFU sub-unit types
  val SFU_CSRS: Int = 0   // CSR sub-unit
  val SFU_WCTL: Int = 1   // Warp-control sub-unit

  val NUM_SFU_UNITS: Int = 2              // Number of SFU sub-units
  val SFU_BITS:      Int = log2Ceil(NUM_SFU_UNITS)
  val SFU_WIDTH:     Int = up(SFU_BITS)

  // -------------------------------------------------------------------------
  // RISC-V opcode encodings (7-bit base opcode field)
  // -------------------------------------------------------------------------

  val INST_LUI:    Int = 0x37   // 7'b0110111 — Load Upper Immediate
  val INST_AUIPC:  Int = 0x17   // 7'b0010111 — Add Upper Immediate to PC
  val INST_JAL:    Int = 0x6F   // 7'b1101111 — Jump And Link
  val INST_JALR:   Int = 0x67   // 7'b1100111 — Jump And Link Register
  val INST_B:      Int = 0x63   // 7'b1100011 — Branch instructions
  val INST_L:      Int = 0x03   // 7'b0000011 — Load instructions
  val INST_S:      Int = 0x23   // 7'b0100011 — Store instructions
  val INST_I:      Int = 0x13   // 7'b0010011 — Immediate instructions
  val INST_R:      Int = 0x33   // 7'b0110011 — Register instructions
  val INST_V:      Int = 0x57   // 7'b1010111 — Vector instructions
  val INST_FENCE:  Int = 0x0F   // 7'b0001111 — Fence instructions
  val INST_SYS:    Int = 0x73   // 7'b1110011 — System instructions

  // RV64I W-type opcodes
  val INST_I_W:    Int = 0x1B   // 7'b0011011 — W-type immediate (RV64I)
  val INST_R_W:    Int = 0x3B   // 7'b0111011 — W-type register  (RV64I)

  // Floating-point opcodes
  val INST_FL:     Int = 0x07   // 7'b0000111 — Float load
  val INST_FS:     Int = 0x27   // 7'b0100111 — Float store
  val INST_FMADD:  Int = 0x43   // 7'b1000011 — Fused Multiply-Add
  val INST_FMSUB:  Int = 0x47   // 7'b1000111 — Fused Multiply-Subtract
  val INST_FNMSUB: Int = 0x4B   // 7'b1001011 — Fused Neg. Multiply-Subtract
  val INST_FNMADD: Int = 0x4F   // 7'b1001111 — Fused Neg. Multiply-Add
  val INST_FCI:    Int = 0x53   // 7'b1010011 — Float common instructions

  // Custom extension opcodes
  val INST_EXT1:   Int = 0x0B   // 7'b0001011 — Custom-0
  val INST_EXT2:   Int = 0x2B   // 7'b0101011 — Custom-1
  val INST_EXT3:   Int = 0x5B   // 7'b1011011 — Custom-2
  val INST_EXT4:   Int = 0x7B   // 7'b1111011 — Custom-3

  // Opcode extensions (funct7 fields)
  val INST_R_F7_MUL:    Int = 0x01   // 7'b0000001 — M extension multiply
  val INST_R_F7_ZICOND: Int = 0x07   // 7'b0000111 — Zicond extension

  // -------------------------------------------------------------------------
  // Floating-point rounding modes  (INST_FRM_*)
  // -------------------------------------------------------------------------

  val INST_FRM_RNE:  Int = 0x0   // 3'b000 — Round to Nearest Even
  val INST_FRM_RTZ:  Int = 0x1   // 3'b001 — Round to Zero
  val INST_FRM_RDN:  Int = 0x2   // 3'b010 — Round Down (-inf)
  val INST_FRM_RUP:  Int = 0x3   // 3'b011 — Round Up (+inf)
  val INST_FRM_RMM:  Int = 0x4   // 3'b100 — Round to Nearest Max Magnitude
  val INST_FRM_DYN:  Int = 0x7   // 3'b111 — Dynamic rounding mode
  val INST_FRM_BITS: Int = 3     // Width of FRM field

  // -------------------------------------------------------------------------
  // Instruction field widths
  // -------------------------------------------------------------------------

  val INST_OP_BITS:  Int = 4   // op_type field width
  val INST_FMT_BITS: Int = 2   // fmt field width

  // -------------------------------------------------------------------------
  // ALU op_type encodings  (INST_ALU_*)
  // -------------------------------------------------------------------------

  val INST_ALU_ADD:   Int = 0x0   // 4'b0000 — Add
  val INST_ALU_LUI:   Int = 0x2   // 4'b0010 — Load Upper Immediate
  val INST_ALU_AUIPC: Int = 0x3   // 4'b0011 — Add Upper Immediate to PC
  val INST_ALU_SLTU:  Int = 0x4   // 4'b0100 — Set Less Than Unsigned
  val INST_ALU_SLT:   Int = 0x5   // 4'b0101 — Set Less Than (signed)
  val INST_ALU_SUB:   Int = 0x7   // 4'b0111 — Subtract
  val INST_ALU_SRL:   Int = 0x8   // 4'b1000 — Shift Right Logical
  val INST_ALU_SRA:   Int = 0x9   // 4'b1001 — Shift Right Arithmetic
  val INST_ALU_CZEQ:  Int = 0xA   // 4'b1010 — Conditional Zero if Equal (Zicond)
  val INST_ALU_CZNE:  Int = 0xB   // 4'b1011 — Conditional Zero if Not Equal (Zicond)
  val INST_ALU_AND:   Int = 0xC   // 4'b1100 — Bitwise AND
  val INST_ALU_OR:    Int = 0xD   // 4'b1101 — Bitwise OR
  val INST_ALU_XOR:   Int = 0xE   // 4'b1110 — Bitwise XOR
  val INST_ALU_SLL:   Int = 0xF   // 4'b1111 — Shift Left Logical
  val INST_ALU_BITS:  Int = 4     // Width of ALU op field

  // ALU type sub-classification (bits [3:2] of INST_ALU_*)
  val ALU_TYPE_BITS:   Int = 2   // Width of ALU type field
  val ALU_TYPE_ARITH:  Int = 0   // Arithmetic operations
  val ALU_TYPE_BRANCH: Int = 1   // Branch operations
  val ALU_TYPE_MULDIV: Int = 2   // Multiply/Divide operations
  val ALU_TYPE_OTHER:  Int = 3   // Other (e.g. CZERO) operations

  // ALU class: extracts bits [3:2] of the op field (= op >> 2)
  // Equivalent to: function inst_alu_class — return op[3:2]
  def inst_alu_class(op: UInt): UInt = op(3, 2)

  // ALU signed flag: bit [0] of the op field
  // Equivalent to: function inst_alu_signed — return op[0]
  def inst_alu_signed(op: UInt): Bool = op(0)

  // ALU is-subtract flag: bit [1] of the op field
  // Equivalent to: function inst_alu_is_sub — return op[1]
  def inst_alu_is_sub(op: UInt): Bool = op(1)

  // ALU is-czero flag: op[3:1] == 3'b101
  // Equivalent to: function inst_alu_is_czero — return (op[3:1] == 3'b101)
  def inst_alu_is_czero(op: UInt): Bool = op(3, 1) === "b101".U

  // -------------------------------------------------------------------------
  // Branch op_type encodings  (INST_BR_*)
  // -------------------------------------------------------------------------

  val INST_BR_BEQ:    Int = 0x0   // 4'b0000 — Branch Equal
  val INST_BR_BNE:    Int = 0x2   // 4'b0010 — Branch Not Equal
  val INST_BR_BLTU:   Int = 0x4   // 4'b0100 — Branch Less Than Unsigned
  val INST_BR_BGEU:   Int = 0x6   // 4'b0110 — Branch Greater-Equal Unsigned
  val INST_BR_BLT:    Int = 0x5   // 4'b0101 — Branch Less Than (signed)
  val INST_BR_BGE:    Int = 0x7   // 4'b0111 — Branch Greater-Equal (signed)
  val INST_BR_JAL:    Int = 0x8   // 4'b1000 — Jump And Link
  val INST_BR_JALR:   Int = 0x9   // 4'b1001 — Jump And Link Register
  val INST_BR_ECALL:  Int = 0xA   // 4'b1010 — Environment Call
  val INST_BR_EBREAK: Int = 0xB   // 4'b1011 — Environment Break
  val INST_BR_URET:   Int = 0xC   // 4'b1100 — U-mode Return
  val INST_BR_SRET:   Int = 0xD   // 4'b1101 — S-mode Return
  val INST_BR_MRET:   Int = 0xE   // 4'b1110 — M-mode Return
  val INST_BR_OTHER:  Int = 0xF   // 4'b1111 — Other (fence, etc.)
  val INST_BR_BITS:   Int = 4     // Width of branch op field

  // Branch class: {0, ~op[3]}  — conditional=1 if bit3 is 0
  // Equivalent to: function inst_br_class — return {1'b0, ~op[3]}
  def inst_br_class(op: UInt): UInt = Cat(0.U(1.W), ~op(3))

  // Branch is-neg: bit [1] indicates negation of comparison
  // Equivalent to: function inst_br_is_neg — return op[1]
  def inst_br_is_neg(op: UInt): Bool = op(1)

  // Branch is-less-than: bit [2]
  // Equivalent to: function inst_br_is_less — return op[2]
  def inst_br_is_less(op: UInt): Bool = op(2)

  // Branch is-static (JAL/JALR/ECALL/…): bit [3]
  // Equivalent to: function inst_br_is_static — return op[3]
  def inst_br_is_static(op: UInt): Bool = op(3)

  // -------------------------------------------------------------------------
  // Vote / Shuffle extension encodings  (INST_VOTE_*, INST_SHFL_*)
  // -------------------------------------------------------------------------

  val INST_VOTE_ALL:  Int = 0x0   // 2'b00 — Vote All
  val INST_VOTE_ANY:  Int = 0x1   // 2'b01 — Vote Any
  val INST_VOTE_UNI:  Int = 0x2   // 2'b10 — Vote Uniform
  val INST_VOTE_BAL:  Int = 0x3   // 2'b11 — Ballot

  val INST_SHFL_UP:   Int = 0x0   // 2'b00 — Shuffle Up
  val INST_SHFL_DOWN: Int = 0x1   // 2'b01 — Shuffle Down
  val INST_SHFL_BFLY: Int = 0x2   // 2'b10 — Shuffle Butterfly
  val INST_SHFL_IDX:  Int = 0x3   // 2'b11 — Shuffle Index

  val INST_VOTE_BITS: Int = 2   // Width of vote op field
  val INST_SHFL_BITS: Int = 2   // Width of shuffle op field

  // -------------------------------------------------------------------------
  // M-extension (integer multiply/divide) op_type encodings  (INST_M_*)
  // -------------------------------------------------------------------------

  val INST_M_MUL:    Int = 0x0   // 3'b000 — Multiply (low bits)
  val INST_M_MULHU:  Int = 0x1   // 3'b001 — Multiply High Unsigned×Unsigned
  val INST_M_MULH:   Int = 0x2   // 3'b010 — Multiply High Signed×Signed
  val INST_M_MULHSU: Int = 0x3   // 3'b011 — Multiply High Signed×Unsigned
  val INST_M_DIV:    Int = 0x4   // 3'b100 — Divide (signed)
  val INST_M_DIVU:   Int = 0x5   // 3'b101 — Divide Unsigned
  val INST_M_REM:    Int = 0x6   // 3'b110 — Remainder (signed)
  val INST_M_REMU:   Int = 0x7   // 3'b111 — Remainder Unsigned
  val INST_M_BITS:   Int = 3     // Width of M op field

  // M signed: ~op[0]  — equivalent to function inst_m_signed
  def inst_m_signed(op: UInt): Bool = ~op(0)

  // M is-mulx (multiply, not divide): ~op[2]
  // Equivalent to: function inst_m_is_mulx — return ~op[2]
  def inst_m_is_mulx(op: UInt): Bool = ~op(2)

  // M is-mulh (high result): op[1:0] != 0
  // Equivalent to: function inst_m_is_mulh — return (op[1:0] != 0)
  def inst_m_is_mulh(op: UInt): Bool = op(1, 0) =/= 0.U

  // M signed-A operand: op[1:0] != 1
  // Equivalent to: function inst_m_signed_a — return (op[1:0] != 1)
  def inst_m_signed_a(op: UInt): Bool = op(1, 0) =/= 1.U

  // M is-rem (remainder, not quotient): op[1]
  // Equivalent to: function inst_m_is_rem — return op[1]
  def inst_m_is_rem(op: UInt): Bool = op(1)

  // -------------------------------------------------------------------------
  // LSU format and op_type encodings  (LSU_FMT_*, INST_LSU_*)
  // -------------------------------------------------------------------------

  // Load/store size encoding (3-bit fmt field = funct3)
  val LSU_FMT_B:  Int = 0x0   // 3'b000 — Byte (signed)
  val LSU_FMT_H:  Int = 0x1   // 3'b001 — Halfword (signed)
  val LSU_FMT_W:  Int = 0x2   // 3'b010 — Word (signed)
  val LSU_FMT_D:  Int = 0x3   // 3'b011 — Doubleword
  val LSU_FMT_BU: Int = 0x4   // 3'b100 — Byte (unsigned)
  val LSU_FMT_HU: Int = 0x5   // 3'b101 — Halfword (unsigned)
  val LSU_FMT_WU: Int = 0x6   // 3'b110 — Word (unsigned, RV64I)

  // 4-bit internal LSU op encoding
  val INST_LSU_LB:    Int = 0x0   // 4'b0000 — Load Byte
  val INST_LSU_LH:    Int = 0x1   // 4'b0001 — Load Halfword
  val INST_LSU_LW:    Int = 0x2   // 4'b0010 — Load Word
  val INST_LSU_LD:    Int = 0x3   // 4'b0011 — Load Doubleword (RV64I)
  val INST_LSU_LBU:   Int = 0x4   // 4'b0100 — Load Byte Unsigned
  val INST_LSU_LHU:   Int = 0x5   // 4'b0101 — Load Halfword Unsigned
  val INST_LSU_LWU:   Int = 0x6   // 4'b0110 — Load Word Unsigned (RV64I)
  val INST_LSU_SB:    Int = 0x8   // 4'b1000 — Store Byte
  val INST_LSU_SH:    Int = 0x9   // 4'b1001 — Store Halfword
  val INST_LSU_SW:    Int = 0xA   // 4'b1010 — Store Word
  val INST_LSU_SD:    Int = 0xB   // 4'b1011 — Store Doubleword (RV64I)
  val INST_LSU_FENCE: Int = 0xF   // 4'b1111 — Fence
  val INST_LSU_BITS:  Int = 4     // Width of LSU op field

  // Fence sub-type
  val INST_FENCE_BITS: Int = 1   // Width of fence sub-type field
  val INST_FENCE_D:    Int = 0   // 1'h0 — Data fence
  val INST_FENCE_I:    Int = 1   // 1'h1 — Instruction fence

  // LSU format (lower 3 bits of op = funct3-equivalent)
  // Equivalent to: function inst_lsu_fmt — return op[2:0]
  def inst_lsu_fmt(op: UInt): UInt = op(2, 0)

  // LSU write size (lower 2 bits = byte/halfword/word/double)
  // Equivalent to: function inst_lsu_wsize — return op[1:0]
  def inst_lsu_wsize(op: UInt): UInt = op(1, 0)

  // LSU is-fence: op[3:2] == 3
  // Equivalent to: function inst_lsu_is_fence — return (op[3:2] == 3)
  def inst_lsu_is_fence(op: UInt): Bool = op(3, 2) === 3.U

  // -------------------------------------------------------------------------
  // FPU op_type encodings  (INST_FPU_*)
  // -------------------------------------------------------------------------

  val INST_FPU_ADD:   Int = 0x0   // 4'b0000 — Add (SUB when fmt[1]=1)
  val INST_FPU_MUL:   Int = 0x1   // 4'b0001 — Multiply
  val INST_FPU_MADD:  Int = 0x2   // 4'b0010 — Multiply-Add (SUB when fmt[1]=1)
  val INST_FPU_NMADD: Int = 0x3   // 4'b0011 — Neg. Multiply-Add (SUB when fmt[1]=1)
  val INST_FPU_DIV:   Int = 0x4   // 4'b0100 — Divide
  val INST_FPU_SQRT:  Int = 0x5   // 4'b0101 — Square Root
  val INST_FPU_F2I:   Int = 0x8   // 4'b1000 — Float to Int
  val INST_FPU_F2U:   Int = 0x9   // 4'b1001 — Float to UInt
  val INST_FPU_I2F:   Int = 0xA   // 4'b1010 — Int to Float
  val INST_FPU_U2F:   Int = 0xB   // 4'b1011 — UInt to Float
  val INST_FPU_CMP:   Int = 0xC   // 4'b1100 — Compare (frm: LE=0, LT=1, EQ=2)
  val INST_FPU_F2F:   Int = 0xD   // 4'b1101 — Float-to-Float conversion
  val INST_FPU_MISC:  Int = 0xE   // 4'b1110 — Misc (SGNJ/SGNJN/SGNJX/CLASS/MVXW/MVWX/FMIN/FMAX)
  val INST_FPU_BITS:  Int = 4     // Width of FPU op field

  // FPU is-class (FCLASS): op == INST_FPU_MISC && frm == 3
  // Equivalent to: function inst_fpu_is_class
  def inst_fpu_is_class(op: UInt, frm: UInt): Bool =
    (op === INST_FPU_MISC.U) && (frm === 3.U)

  // FPU is-mvxw (move FP word to integer): op == INST_FPU_MISC && frm == 4
  // Equivalent to: function inst_fpu_is_mvxw
  def inst_fpu_is_mvxw(op: UInt, frm: UInt): Bool =
    (op === INST_FPU_MISC.U) && (frm === 4.U)

  // -------------------------------------------------------------------------
  // SFU op_type encodings  (INST_SFU_*)
  // -------------------------------------------------------------------------

  val INST_SFU_TMC:    Int = 0x0   // 4'h0 — Thread Mask Control
  val INST_SFU_WSPAWN: Int = 0x1   // 4'h1 — Warp Spawn
  val INST_SFU_SPLIT:  Int = 0x2   // 4'h2 — Divergence Split
  val INST_SFU_JOIN:   Int = 0x3   // 4'h3 — Divergence Join
  val INST_SFU_BAR:    Int = 0x4   // 4'h4 — Barrier
  val INST_SFU_PRED:   Int = 0x5   // 4'h5 — Predicate
  val INST_SFU_CSRRW:  Int = 0x6   // 4'h6 — CSR Read/Write
  val INST_SFU_CSRRS:  Int = 0x7   // 4'h7 — CSR Read and Set
  val INST_SFU_CSRRC:  Int = 0x8   // 4'h8 — CSR Read and Clear
  val INST_SFU_BITS:   Int = 4     // Width of SFU op field

  // SFU CSR op from funct3: (4'h6 + funct3[1:0] - 4'h1) => maps CSRRW=1,CSRRS=2,CSRRC=3 to 6,7,8
  // Equivalent to: function inst_sfu_csr — return (4'h6 + funct3[1:0] - 4'h1)
  def inst_sfu_csr(funct3: UInt): UInt = 6.U(4.W) + funct3(1, 0) - 1.U(4.W)

  // SFU is-wctl (warp-control): op <= 5
  // Equivalent to: function inst_sfu_is_wctl — return (op <= 5)
  def inst_sfu_is_wctl(op: UInt): Bool = op <= 5.U

  // SFU is-csr: op >= 6 && op <= 8
  // Equivalent to: function inst_sfu_is_csr — return (op >= 6 && op <= 8)
  def inst_sfu_is_csr(op: UInt): Bool = (op >= 6.U) && (op <= 8.U)

  // -------------------------------------------------------------------------
  // Issue / scheduling parameters
  // -------------------------------------------------------------------------

  val ISSUE_ISW_BITS: Int = log2Ceil(ISSUE_WIDTH)         // CLOG2(ISSUE_WIDTH)
  val ISSUE_ISW_W:    Int = up(ISSUE_ISW_BITS)           // UP(ISSUE_ISW_BITS)
  val PER_ISSUE_WARPS: Int = NUM_WARPS / ISSUE_WIDTH    // Warps per issue slot
  val ISSUE_WIS_BITS: Int = log2Ceil(PER_ISSUE_WARPS)      // CLOG2(PER_ISSUE_WARPS)
  val ISSUE_WIS_W:    Int = up(ISSUE_WIS_BITS)           // UP(ISSUE_WIS_BITS)

  // wis + isw -> wid  (function wis_to_wid)
  // Combines warp-in-slot and issue-slot indices into a full warp ID.
  def wis_to_wid(wis: UInt, isw: UInt): UInt = {
    if (ISSUE_WIS_BITS == 0)       isw(NW_WIDTH - 1, 0)
    else if (ISSUE_ISW_BITS == 0)  wis(NW_WIDTH - 1, 0)
    else                           Cat(wis, isw)(NW_WIDTH - 1, 0)
  }

  // wid -> isw  (function wid_to_isw)
  // Extracts the issue-slot index from a full warp ID.
  def wid_to_isw(wid: UInt): UInt = {
    if (ISSUE_ISW_BITS != 0) wid(ISSUE_ISW_W - 1, 0)
    else                     0.U(ISSUE_ISW_W.W)
  }

  // wid -> wis  (function wid_to_wis)
  // Extracts the warp-in-slot index from a full warp ID.
  def wid_to_wis(wid: UInt): UInt = {
    if (ISSUE_WIS_BITS != 0) (wid >> ISSUE_ISW_BITS)(ISSUE_WIS_W - 1, 0)
    else                     0.U(ISSUE_WIS_W.W)
  }

  // -------------------------------------------------------------------------
  // Instruction argument field widths
  // INST_ARGS_BITS = ALU_TYPE_BITS + XLEN + 3  =  2 + 32 + 3 = 37
  // -------------------------------------------------------------------------

  val INST_ARGS_BITS: Int = ALU_TYPE_BITS + XLEN + 3   // Width of op_args union (37 for XLEN=32)

  // -------------------------------------------------------------------------
  // LSU memory parameters  (from VX_gpu_pkg.sv)
  // -------------------------------------------------------------------------

  val LSU_WORD_SIZE:   Int = XLENB                                    // LSU word size in bytes
  val LSU_ADDR_WIDTH:  Int = MEM_ADDR_WIDTH - log2Ceil(LSU_WORD_SIZE)   // LSU address width
  val LSU_MEM_BATCHES: Int = 1                                        // LSU memory batches
  val LSU_TAG_ID_BITS: Int = log2Ceil(LSUQ_IN_SIZE) + log2Ceil(LSU_MEM_BATCHES)
  val LSU_TAG_WIDTH:   Int = UUID_WIDTH + LSU_TAG_ID_BITS
  val LSU_NUM_REQS:    Int = NUM_LSU_BLOCKS * NUM_LSU_LANES
  val LMEM_TAG_WIDTH:  Int = LSU_TAG_WIDTH + log2Ceil(NUM_LSU_BLOCKS)

  // -------------------------------------------------------------------------
  // ICache derived parameters  (from VX_gpu_pkg.sv)
  // -------------------------------------------------------------------------

  val ICACHE_WORD_SIZE:  Int = 4                                          // ICache word size (bytes)
  val ICACHE_ADDR_WIDTH: Int = MEM_ADDR_WIDTH - log2Ceil(ICACHE_WORD_SIZE)  // ICache address width
  val ICACHE_LINE_SIZE:  Int = L1_LINE_SIZE                               // ICache line size (bytes)
  val ICACHE_TAG_ID_BITS:Int = NW_WIDTH                                   // ICache core-req tag ID bits
  val ICACHE_TAG_WIDTH:  Int = UUID_WIDTH + ICACHE_TAG_ID_BITS            // ICache core-req tag width
  val ICACHE_MEM_DATA_WIDTH: Int = ICACHE_LINE_SIZE * 8                   // ICache memory data width

  // -------------------------------------------------------------------------
  // DCache derived parameters  (from VX_gpu_pkg.sv)
  // -------------------------------------------------------------------------

  val DCACHE_WORD_SIZE:  Int = LSU_LINE_SIZE                              // DCache word size (= LSU_LINE_SIZE)
  val DCACHE_ADDR_WIDTH: Int = MEM_ADDR_WIDTH - log2Ceil(DCACHE_WORD_SIZE)  // DCache address width
  val DCACHE_LINE_SIZE:  Int = L1_LINE_SIZE                               // DCache line size (bytes)
  val DCACHE_CHANNELS:   Int = up((NUM_LSU_LANES * LSU_WORD_SIZE) / DCACHE_WORD_SIZE)
  val DCACHE_NUM_REQS:   Int = NUM_LSU_BLOCKS * DCACHE_CHANNELS
  val DCACHE_NUM_BANKS:  Int = math.min(DCACHE_NUM_REQS, 16)

  // L1/L2/L3 memory port counts — mirrors VX_gpu_pkg.sv port-count formulas.
  // Moved here from VortexConfigConstants so DCACHE_NUM_REQS is available.
  val L1_MEM_PORTS: Int = math.min(DCACHE_NUM_BANKS, PLATFORM_MEMORY_NUM_BANKS)
  private val _L2_NUM_REQS:  Int = NUM_SOCKETS * L1_MEM_PORTS
  private val _L2_NUM_BANKS: Int = math.min(_L2_NUM_REQS, 16)
  val L2_MEM_PORTS: Int = math.min(_L2_NUM_BANKS, PLATFORM_MEMORY_NUM_BANKS)
  private val _L3_NUM_REQS:  Int = NUM_CLUSTERS * L2_MEM_PORTS
  private val _L3_NUM_BANKS: Int = math.min(_L3_NUM_REQS, 16)
  val L3_MEM_PORTS: Int = math.min(_L3_NUM_BANKS, PLATFORM_MEMORY_NUM_BANKS)

  val DCACHE_MERGED_REQS: Int = (NUM_LSU_LANES * LSU_WORD_SIZE) / DCACHE_WORD_SIZE
  // CDIV helper: ceiling division
  val DCACHE_MEM_BATCHES: Int = (DCACHE_MERGED_REQS + DCACHE_CHANNELS - 1) / DCACHE_CHANNELS
  val DCACHE_TAG_ID_BITS: Int = log2Ceil(LSUQ_OUT_SIZE) + log2Ceil(DCACHE_MEM_BATCHES)
  val DCACHE_TAG_WIDTH:   Int = UUID_WIDTH + DCACHE_TAG_ID_BITS
  val DCACHE_MEM_DATA_WIDTH: Int = DCACHE_LINE_SIZE * 8

  // -------------------------------------------------------------------------
  // Top-level VX memory interface parameters  (from VX_gpu_pkg.sv)
  // -------------------------------------------------------------------------

  val VX_MEM_PORTS:        Int = L3_MEM_PORTS                           // Number of memory ports
  val VX_MEM_BYTEEN_WIDTH: Int = L3_LINE_SIZE                           // Byte-enable width
  val VX_MEM_ADDR_WIDTH:   Int = MEM_ADDR_WIDTH - log2Ceil(L3_LINE_SIZE)  // Memory address width
  val VX_MEM_DATA_WIDTH:   Int = L3_LINE_SIZE * 8                       // Memory data width (bits)

  // -------------------------------------------------------------------------
  // Miscellaneous helper functions
  // -------------------------------------------------------------------------

  // op_to_sfu_type: maps SFU op_type to SFU sub-unit (CSR or WCTL)
  // Equivalent to: function op_to_sfu_type
  def op_to_sfu_type(op_type: UInt): UInt = {
    val isCSR = (op_type === INST_SFU_CSRRW.U) ||
                (op_type === INST_SFU_CSRRS.U) ||
                (op_type === INST_SFU_CSRRC.U)
    Mux(isCSR, SFU_CSRS.U(SFU_WIDTH.W), SFU_WCTL.U(SFU_WIDTH.W))
  }

  // make_reg_num: combine register type and index into a unified register number
  // Equivalent to: function make_reg_num — (rtype << RV_REGS_BITS) | idx
  def make_reg_num(rtype: UInt, idx: UInt): UInt =
    Cat(rtype(REG_TYPE_BITS - 1, 0), idx(RV_REGS_BITS - 1, 0))

  // get_reg_type: extract the register type from a unified register number
  // Equivalent to: function get_reg_type — reg_num >> RV_REGS_BITS
  def get_reg_type(reg_num: UInt): UInt =
    reg_num(NUM_REGS_BITS - 1, RV_REGS_BITS)

  // get_reg_idx: extract the register index from a unified register number
  // Equivalent to: function get_reg_idx — reg_num[RV_REGS_BITS-1:0]
  def get_reg_idx(reg_num: UInt): UInt =
    reg_num(RV_REGS_BITS - 1, 0)
}

// =============================================================================
// Chisel Bundle definitions — translated from struct packed in VX_gpu_pkg.sv
//
// All field widths are derived from VortexGPUPkg constants.
// Parameterised structs take their topology parameters as constructor args.
// =============================================================================

import VortexGPUPkg._
import VortexConfigConstants._

// -----------------------------------------------------------------------------
// base_dcrs_t — Base device-configuration-register state
// -----------------------------------------------------------------------------
class BaseDcrsBundle extends Bundle {
  val startup_addr = UInt(XLEN.W)   // startup address (XLEN bits)
  val startup_arg  = UInt(XLEN.W)   // startup argument (XLEN bits)
  val mpm_class    = UInt(8.W)      // performance-counter class (8 bits)
}

// -----------------------------------------------------------------------------
// alu_args_t — ALU instruction argument fields
// Total = use_PC(1) + use_imm(1) + is_w(1) + xtype(ALU_TYPE_BITS) + imm(XLEN)
//       = 1 + 1 + 1 + 2 + 32 = 37 = INST_ARGS_BITS  ✓
// -----------------------------------------------------------------------------
class AluArgsBundle extends Bundle {
  val use_PC  = Bool()                    // use PC as source operand
  val use_imm = Bool()                    // use immediate as source operand
  val is_w    = Bool()                    // RV64I W-type instruction
  val xtype   = UInt(ALU_TYPE_BITS.W)    // ALU type sub-class
  val imm     = UInt(XLEN.W)             // immediate value
}

// -----------------------------------------------------------------------------
// fpu_args_t — FPU instruction argument fields
// Padding fills INST_ARGS_BITS - INST_FRM_BITS - INST_FMT_BITS bits.
// -----------------------------------------------------------------------------
class FpuArgsBundle extends Bundle {
  val padding = UInt((INST_ARGS_BITS - INST_FRM_BITS - INST_FMT_BITS).W)  // unused upper bits
  val frm     = UInt(INST_FRM_BITS.W)   // rounding mode
  val fmt     = UInt(INST_FMT_BITS.W)   // format (FP width encoding)
}

// -----------------------------------------------------------------------------
// lsu_args_t — LSU instruction argument fields
// Padding = INST_ARGS_BITS - 1(is_store) - 1(is_float) - OFFSET_BITS
// -----------------------------------------------------------------------------
class LsuArgsBundle extends Bundle {
  val padding  = UInt((INST_ARGS_BITS - 1 - 1 - OFFSET_BITS).W)   // unused upper bits
  val is_store = Bool()                  // is a store instruction
  val is_float = Bool()                  // is a floating-point load/store
  val offset   = UInt(OFFSET_BITS.W)    // 12-bit signed offset
}

// -----------------------------------------------------------------------------
// csr_args_t — CSR instruction argument fields
// Padding = INST_ARGS_BITS - 1(use_imm) - VX_CSR_ADDR_BITS - 5(imm)
// -----------------------------------------------------------------------------
class CsrArgsBundle extends Bundle {
  val padding = UInt((INST_ARGS_BITS - 1 - VX_CSR_ADDR_BITS - 5).W)   // unused upper bits
  val use_imm = Bool()                          // use 5-bit immediate (CSRI variant)
  val addr    = UInt(VX_CSR_ADDR_BITS.W)       // CSR address (12 bits)
  val imm     = UInt(5.W)                       // 5-bit CSR immediate (uimm[4:0])
}

// -----------------------------------------------------------------------------
// wctl_args_t — Warp-control instruction argument fields
// Padding = INST_ARGS_BITS - 1(is_neg)
// -----------------------------------------------------------------------------
class WctlArgsBundle extends Bundle {
  val padding = UInt((INST_ARGS_BITS - 1).W)   // unused upper bits
  val is_neg  = Bool()                           // negate condition
}

// -----------------------------------------------------------------------------
// fetch_t — Instruction fetch pipeline packet
// -----------------------------------------------------------------------------
class FetchBundle extends Bundle {
  val uuid  = UInt(UUID_WIDTH.W)    // unique instruction ID (debug/trace)
  val wid   = UInt(NW_WIDTH.W)     // warp ID
  val tmask = UInt(NUM_THREADS.W)  // active thread mask
  val PC    = UInt(PC_BITS.W)      // program counter
  val instr = UInt(32.W)           // raw 32-bit instruction word
}

// -----------------------------------------------------------------------------
// decode_t — Decoder output pipeline packet
// -----------------------------------------------------------------------------
class DecodeBundle extends Bundle {
  val uuid    = UInt(UUID_WIDTH.W)       // unique instruction ID
  val wid     = UInt(NW_WIDTH.W)        // warp ID
  val tmask   = UInt(NUM_THREADS.W)     // active thread mask
  val PC      = UInt(PC_BITS.W)         // program counter
  val ex_type = UInt(EX_BITS.W)         // execution unit type
  val op_type = UInt(INST_OP_BITS.W)    // operation type within unit
  val op_args = new OpArgsBundle        // operation arguments (union)
  val wb      = Bool()                  // writeback enable
  val used_rs = UInt(NUM_SRC_OPDS.W)   // source operand valid mask
  val rd      = UInt(NUM_REGS_BITS.W)  // destination register
  val rs1     = UInt(NUM_REGS_BITS.W)  // source register 1
  val rs2     = UInt(NUM_REGS_BITS.W)  // source register 2
  val rs3     = UInt(NUM_REGS_BITS.W)  // source register 3
}

// -----------------------------------------------------------------------------
// ibuffer_t — Instruction buffer entry (wid replaced by implicit issue-slot context)
// -----------------------------------------------------------------------------
class IbufferBundle extends Bundle {
  val uuid    = UInt(UUID_WIDTH.W)       // unique instruction ID
  val tmask   = UInt(NUM_THREADS.W)     // active thread mask
  val PC      = UInt(PC_BITS.W)         // program counter
  val ex_type = UInt(EX_BITS.W)         // execution unit type
  val op_type = UInt(INST_OP_BITS.W)    // operation type
  val op_args = new OpArgsBundle        // operation arguments
  val wb      = Bool()                  // writeback enable
  val used_rs = UInt(NUM_SRC_OPDS.W)   // source operand valid mask
  val rd      = UInt(NUM_REGS_BITS.W)  // destination register
  val rs1     = UInt(NUM_REGS_BITS.W)  // source register 1
  val rs2     = UInt(NUM_REGS_BITS.W)  // source register 2
  val rs3     = UInt(NUM_REGS_BITS.W)  // source register 3
}

// -----------------------------------------------------------------------------
// scoreboard_t — Scoreboard tracking entry (uses wis instead of wid)
// -----------------------------------------------------------------------------
class ScoreboardBundle extends Bundle {
  val uuid    = UInt(UUID_WIDTH.W)       // unique instruction ID
  val wis     = UInt(ISSUE_WIS_W.W)     // warp-in-slot index
  val tmask   = UInt(NUM_THREADS.W)     // active thread mask
  val PC      = UInt(PC_BITS.W)         // program counter
  val ex_type = UInt(EX_BITS.W)         // execution unit type
  val op_type = UInt(INST_OP_BITS.W)    // operation type
  val op_args = new OpArgsBundle        // operation arguments
  val wb      = Bool()                  // writeback enable
  val used_rs = UInt(NUM_SRC_OPDS.W)   // source operand valid mask
  val rd      = UInt(NUM_REGS_BITS.W)  // destination register
  val rs1     = UInt(NUM_REGS_BITS.W)  // source register 1
  val rs2     = UInt(NUM_REGS_BITS.W)  // source register 2
  val rs3     = UInt(NUM_REGS_BITS.W)  // source register 3
}

// -----------------------------------------------------------------------------
// operands_t — Operand-collected pipeline packet (SIMD-width data vectors)
// simdWidth: number of SIMD lanes (= SIMD_WIDTH constant)
// -----------------------------------------------------------------------------
class OperandsBundle(simdWidth: Int = SIMD_WIDTH) extends Bundle {
  val uuid     = UInt(UUID_WIDTH.W)                    // unique instruction ID
  val wis      = UInt(ISSUE_WIS_W.W)                  // warp-in-slot index
  val sid      = UInt(SIMD_IDX_W.W)                   // SIMD group index within warp
  val tmask    = UInt(simdWidth.W)                     // active thread mask for this SIMD group
  val PC       = UInt(PC_BITS.W)                       // program counter
  val ex_type  = UInt(EX_BITS.W)                       // execution unit type
  val op_type  = UInt(INST_OP_BITS.W)                  // operation type
  val op_args  = new OpArgsBundle                      // operation arguments
  val wb       = Bool()                                 // writeback enable
  val rd       = UInt(NUM_REGS_BITS.W)                 // destination register
  val rs1_data = Vec(simdWidth, UInt(XLEN.W))          // source operand 1 data (SIMD lanes)
  val rs2_data = Vec(simdWidth, UInt(XLEN.W))          // source operand 2 data (SIMD lanes)
  val rs3_data = Vec(simdWidth, UInt(XLEN.W))          // source operand 3 data (SIMD lanes)
  val sop      = Bool()                                 // start-of-packet (first SIMD group)
  val eop      = Bool()                                 // end-of-packet   (last  SIMD group)
}

// -----------------------------------------------------------------------------
// dispatch_t — Dispatch pipeline packet sent to a specific execution unit.
// Layout must not be modified without updating VX_dispatch_unit.
// simdWidth: number of SIMD lanes (= SIMD_WIDTH constant)
// -----------------------------------------------------------------------------
class DispatchBundle(simdWidth: Int = SIMD_WIDTH) extends Bundle {
  val uuid     = UInt(UUID_WIDTH.W)                    // unique instruction ID
  val wis      = UInt(ISSUE_WIS_W.W)                  // warp-in-slot index
  val sid      = UInt(SIMD_IDX_W.W)                   // SIMD group index within warp
  val tmask    = UInt(simdWidth.W)                     // active thread mask for this SIMD group
  val PC       = UInt(PC_BITS.W)                       // program counter
  val op_type  = UInt(INST_ALU_BITS.W)                 // unit-specific op type (4 bits)
  val op_args  = new OpArgsBundle                      // operation arguments
  val wb       = Bool()                                 // writeback enable
  val rd       = UInt(NUM_REGS_BITS.W)                 // destination register
  val rs1_data = Vec(simdWidth, UInt(XLEN.W))          // source operand 1 data (SIMD lanes)
  val rs2_data = Vec(simdWidth, UInt(XLEN.W))          // source operand 2 data (SIMD lanes)
  val rs3_data = Vec(simdWidth, UInt(XLEN.W))          // source operand 3 data (SIMD lanes)
  val sop      = Bool()                                 // start-of-packet
  val eop      = Bool()                                 // end-of-packet
}

// -----------------------------------------------------------------------------
// commit_t — Execution-unit commit packet (writeback to RF / retire)
// simdWidth: number of SIMD lanes (= SIMD_WIDTH constant)
// -----------------------------------------------------------------------------
class CommitBundle(simdWidth: Int = SIMD_WIDTH) extends Bundle {
  val uuid  = UInt(UUID_WIDTH.W)             // unique instruction ID
  val wid   = UInt(NW_WIDTH.W)              // warp ID (full, not wis)
  val sid   = UInt(SIMD_IDX_W.W)            // SIMD group index
  val tmask = UInt(simdWidth.W)             // active thread mask
  val PC    = UInt(PC_BITS.W)               // program counter
  val wb    = Bool()                         // writeback enable
  val rd    = UInt(NUM_REGS_BITS.W)         // destination register
  val data  = Vec(simdWidth, UInt(XLEN.W)) // result data (SIMD lanes)
  val sop   = Bool()                         // start-of-packet
  val eop   = Bool()                         // end-of-packet
}

// -----------------------------------------------------------------------------
// writeback_t — Register-file writeback packet
// simdWidth: number of SIMD lanes (= SIMD_WIDTH constant)
// -----------------------------------------------------------------------------
class WritebackBundle(simdWidth: Int = SIMD_WIDTH) extends Bundle {
  val uuid  = UInt(UUID_WIDTH.W)             // unique instruction ID
  val wis   = UInt(ISSUE_WIS_W.W)           // warp-in-slot index
  val sid   = UInt(SIMD_IDX_W.W)            // SIMD group index
  val tmask = UInt(simdWidth.W)             // active thread mask
  val PC    = UInt(PC_BITS.W)               // program counter
  val rd    = UInt(NUM_REGS_BITS.W)         // destination register
  val data  = Vec(simdWidth, UInt(XLEN.W)) // writeback data (SIMD lanes)
  val sop   = Bool()                         // start-of-packet
  val eop   = Bool()                         // end-of-packet
}

// -----------------------------------------------------------------------------
// schedule_t — Warp scheduler output packet
// -----------------------------------------------------------------------------
class ScheduleBundle extends Bundle {
  val uuid  = UInt(UUID_WIDTH.W)    // unique instruction ID
  val wid   = UInt(NW_WIDTH.W)     // warp ID
  val tmask = UInt(NUM_THREADS.W)  // active thread mask
  val PC    = UInt(PC_BITS.W)      // program counter
}

// =============================================================================
// Performance counter Bundles — translated from perf_t structs in VX_gpu_pkg.sv
// =============================================================================

// cache_perf_t — Per-cache performance counters
class CachePerfBundle extends Bundle {
  val reads       = UInt(PERF_CTR_BITS.W)   // read  requests
  val writes      = UInt(PERF_CTR_BITS.W)   // write requests
  val read_misses = UInt(PERF_CTR_BITS.W)   // read  misses
  val write_misses= UInt(PERF_CTR_BITS.W)   // write misses
  val bank_stalls = UInt(PERF_CTR_BITS.W)   // bank conflict stall cycles
  val mshr_stalls = UInt(PERF_CTR_BITS.W)   // MSHR stall cycles
  val mem_stalls  = UInt(PERF_CTR_BITS.W)   // memory stall cycles
  val crsp_stalls = UInt(PERF_CTR_BITS.W)   // core-response stall cycles
}

// lmem_perf_t — Local-memory performance counters
class LmemPerfBundle extends Bundle {
  val reads       = UInt(PERF_CTR_BITS.W)   // read  requests
  val writes      = UInt(PERF_CTR_BITS.W)   // write requests
  val bank_stalls = UInt(PERF_CTR_BITS.W)   // bank conflict stall cycles
  val crsp_stalls = UInt(PERF_CTR_BITS.W)   // core-response stall cycles
}

// coalescer_perf_t — Memory coalescer performance counters
class CoalescerPerfBundle extends Bundle {
  val misses = UInt(PERF_CTR_BITS.W)   // coalescer misses
}

// mem_perf_t — Main-memory performance counters
class MemPerfBundle extends Bundle {
  val reads   = UInt(PERF_CTR_BITS.W)   // read  requests
  val writes  = UInt(PERF_CTR_BITS.W)   // write requests
  val latency = UInt(PERF_CTR_BITS.W)   // cumulative latency cycles
}

// sched_perf_t — Scheduler performance counters
class SchedPerfBundle extends Bundle {
  val idles  = UInt(PERF_CTR_BITS.W)   // idle cycles (no ready warp)
  val stalls = UInt(PERF_CTR_BITS.W)   // stall cycles (warp blocked)
}

// issue_perf_t — Issue stage performance counters
class IssuePerfBundle extends Bundle {
  val ibf_stalls  = UInt(PERF_CTR_BITS.W)                    // instruction-buffer stall cycles
  val scb_stalls  = UInt(PERF_CTR_BITS.W)                    // scoreboard stall cycles
  val opd_stalls  = UInt(PERF_CTR_BITS.W)                    // operand-collector stall cycles
  val units_uses  = Vec(NUM_EX_UNITS, UInt(PERF_CTR_BITS.W)) // per-execution-unit use counts
  val sfu_uses    = Vec(NUM_SFU_UNITS, UInt(PERF_CTR_BITS.W))// per-SFU-sub-unit use counts
}

// sysmem_perf_t — Aggregate system-memory performance counters
class SysmemPerfBundle extends Bundle {
  val icache    = new CachePerfBundle      // instruction cache
  val dcache    = new CachePerfBundle      // data cache
  val l2cache   = new CachePerfBundle      // L2 cache
  val l3cache   = new CachePerfBundle      // L3 cache
  val lmem      = new LmemPerfBundle       // local (shared) memory
  val coalescer = new CoalescerPerfBundle  // memory coalescer
  val mem       = new MemPerfBundle        // main memory
}

// pipeline_perf_t — Aggregate pipeline performance counters
class PipelinePerfBundle extends Bundle {
  val sched          = new SchedPerfBundle          // scheduler
  val issue          = new IssuePerfBundle          // issue stage
  val ifetches       = UInt(PERF_CTR_BITS.W)       // total instruction fetches
  val loads          = UInt(PERF_CTR_BITS.W)        // total load operations
  val stores         = UInt(PERF_CTR_BITS.W)        // total store operations
  val ifetch_latency = UInt(PERF_CTR_BITS.W)        // cumulative ifetch latency
  val load_latency   = UInt(PERF_CTR_BITS.W)        // cumulative load latency
}
