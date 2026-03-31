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
// Translated from VX_config.vh and VX_types.vh
// Assumes XLEN_32, EXT_F_ENABLE, EXT_M_ENABLE, EXT_ZICOND_ENABLE,
//         LMEM_ENABLE, ICACHE_ENABLE, DCACHE_ENABLE (all defaults).
//         L2/L3 disabled by default (no L2_ENABLE / L3_ENABLE defined).
//         No EXT_TCU_ENABLE, no EXT_D_ENABLE, no EXT_A_ENABLE, no EXT_C_ENABLE.

package vortex

object VortexConfigConstants {

  // Utility: max of two ints
  def max(x: Int, y: Int): Int = if (x > y) x else y

  // Utility: min of two ints
  def min(x: Int, y: Int): Int = if (x < y) x else y

  // ---------------------------------------------------------------------------
  // Core topology  (VX_config.vh defaults)
  // ---------------------------------------------------------------------------

  val NUM_CLUSTERS: Int  = 1   // `NUM_CLUSTERS
  val NUM_CORES:    Int  = 1   // `NUM_CORES
  val NUM_WARPS:    Int  = 8   // `NUM_WARPS
  val NUM_THREADS:  Int  = 4   // `NUM_THREADS

  // `NUM_BARRIERS — default: UP(NUM_WARPS/2)
  val NUM_BARRIERS: Int  = up(NUM_WARPS / 2)

  // `SOCKET_SIZE — default: MIN(4, NUM_CORES)
  val SOCKET_SIZE: Int   = min(4, NUM_CORES)

  // ---------------------------------------------------------------------------
  // XLEN / FLEN
  // ---------------------------------------------------------------------------

  val XLEN: Int          = 32   // `XLEN  (XLEN_32 assumed)
  val FLEN: Int          = 32   // `FLEN  (FLEN_32 assumed, no EXT_D_ENABLE)

  // VLEN = 4 * XLEN  (`VLEN)
  val VLEN: Int          = 4 * XLEN

  // ---------------------------------------------------------------------------
  // Memory address width
  // ---------------------------------------------------------------------------

  // `MEM_ADDR_WIDTH — 32 for XLEN_32
  val MEM_ADDR_WIDTH: Int = 32

  // `MEM_BLOCK_SIZE — default 64 bytes
  val MEM_BLOCK_SIZE: Int = 64

  // ---------------------------------------------------------------------------
  // Cache line sizes  (all default to MEM_BLOCK_SIZE)
  // ---------------------------------------------------------------------------

  val L1_LINE_SIZE: Int  = MEM_BLOCK_SIZE   // `L1_LINE_SIZE
  val L2_LINE_SIZE: Int  = MEM_BLOCK_SIZE   // `L2_LINE_SIZE
  val L3_LINE_SIZE: Int  = MEM_BLOCK_SIZE   // `L3_LINE_SIZE

  // ---------------------------------------------------------------------------
  // Platform memory
  // ---------------------------------------------------------------------------

  val PLATFORM_MEMORY_NUM_BANKS:   Int = 2    // `PLATFORM_MEMORY_NUM_BANKS
  val PLATFORM_MEMORY_ADDR_WIDTH:  Int = 32   // `PLATFORM_MEMORY_ADDR_WIDTH (XLEN_32)
  val PLATFORM_MEMORY_DATA_SIZE:   Int = 64   // `PLATFORM_MEMORY_DATA_SIZE (bytes)
  val PLATFORM_MEMORY_INTERLEAVE:  Int = 1    // `PLATFORM_MEMORY_INTERLEAVE

  // ---------------------------------------------------------------------------
  // Address space  (XLEN_32 values from VX_config.vh)
  // ---------------------------------------------------------------------------

  val STACK_BASE_ADDR: Long = 0xFFFF0000L    // `STACK_BASE_ADDR (32-bit)
  val STARTUP_ADDR:    Long = 0x80000000L    // `STARTUP_ADDR
  val USER_BASE_ADDR:  Long = 0x00010000L    // `USER_BASE_ADDR
  val IO_BASE_ADDR:    Long = 0x00000040L    // `IO_BASE_ADDR

  // IO
  val IO_COUT_SIZE:   Int  = 64             // `IO_COUT_SIZE (bytes)

  // Local memory
  val LMEM_LOG_SIZE:  Int  = 14             // `LMEM_LOG_SIZE
  val LMEM_BASE_ADDR: Long = STACK_BASE_ADDR // `LMEM_BASE_ADDR

  // Stack
  val STACK_LOG2_SIZE: Int = 13             // `STACK_LOG2_SIZE

  val MEM_PAGE_SIZE:      Int = 4096        // `MEM_PAGE_SIZE
  val MEM_PAGE_LOG2_SIZE: Int = 12          // `MEM_PAGE_LOG2_SIZE

  val RESET_DELAY: Int = 8                  // `RESET_DELAY

  // ---------------------------------------------------------------------------
  // Pipeline configuration  (VX_config.vh defaults for 4T/4W)
  // ---------------------------------------------------------------------------

  // SIMD width: default NUM_THREADS
  val SIMD_WIDTH: Int        = NUM_THREADS   // `SIMD_WIDTH

  // Issue width: default UP(NUM_WARPS / 16) => UP(4/16) = UP(0) = 1
  val ISSUE_WIDTH: Int       = up(NUM_WARPS / 16)   // `ISSUE_WIDTH

  // Operand collectors: default UP(NUM_WARPS / (4 * ISSUE_WIDTH))
  val NUM_OPCS: Int          = up(NUM_WARPS / (4 * ISSUE_WIDTH))   // `NUM_OPCS

  // GPR banks
  val NUM_GPR_BANKS:  Int    = 4   // `NUM_GPR_BANKS
  val NUM_VGPR_BANKS: Int    = 2   // `NUM_VGPR_BANKS

  // Execution unit lane counts
  val NUM_ALU_LANES:  Int    = SIMD_WIDTH    // `NUM_ALU_LANES
  val NUM_ALU_BLOCKS: Int    = ISSUE_WIDTH   // `NUM_ALU_BLOCKS
  val NUM_FPU_LANES:  Int    = SIMD_WIDTH    // `NUM_FPU_LANES
  val NUM_FPU_BLOCKS: Int    = ISSUE_WIDTH   // `NUM_FPU_BLOCKS
  val NUM_LSU_LANES:  Int    = SIMD_WIDTH    // `NUM_LSU_LANES
  val NUM_LSU_BLOCKS: Int    = 1             // `NUM_LSU_BLOCKS
  val NUM_SFU_LANES:  Int    = SIMD_WIDTH    // `NUM_SFU_LANES
  val NUM_SFU_BLOCKS: Int    = 1             // `NUM_SFU_BLOCKS (always 1)
  val NUM_VPU_LANES:  Int    = SIMD_WIDTH    // `NUM_VPU_LANES
  val NUM_VPU_BLOCKS: Int    = ISSUE_WIDTH   // `NUM_VPU_BLOCKS
  val NUM_TCU_LANES:  Int    = NUM_THREADS   // `NUM_TCU_LANES
  val NUM_TCU_BLOCKS: Int    = ISSUE_WIDTH   // `NUM_TCU_BLOCKS

  // Instruction buffer size
  val IBUF_SIZE: Int         = 4             // `IBUF_SIZE

  // LSU line size: MIN(NUM_LSU_LANES * (XLEN/8), L1_LINE_SIZE)
  val LSU_LINE_SIZE: Int     = min(NUM_LSU_LANES * (XLEN / 8), L1_LINE_SIZE)  // `LSU_LINE_SIZE

  // LSU queue sizes
  val LSUQ_IN_SIZE:  Int     = 2 * (SIMD_WIDTH / NUM_LSU_LANES)               // `LSUQ_IN_SIZE
  val LSUQ_OUT_SIZE: Int     = max(LSUQ_IN_SIZE, LSU_LINE_SIZE / (XLEN / 8))  // `LSUQ_OUT_SIZE

  // FPU queue size
  val FPUQ_SIZE: Int         = 2 * (SIMD_WIDTH / NUM_FPU_LANES)   // `FPUQ_SIZE

  // FPU latencies (DPI defaults used)
  val LATENCY_FNCP:   Int    = 2    // `LATENCY_FNCP
  val LATENCY_FMA:    Int    = 4    // `LATENCY_FMA
  val LATENCY_FDIV:   Int    = 15   // `LATENCY_FDIV  (FPU_DPI)
  val LATENCY_FSQRT:  Int    = 10   // `LATENCY_FSQRT (FPU_DPI)
  val LATENCY_FCVT:   Int    = 5    // `LATENCY_FCVT

  // FPU PE ratios
  val FMA_PE_RATIO:   Int    = 1    // `FMA_PE_RATIO
  val FDIV_PE_RATIO:  Int    = 8    // `FDIV_PE_RATIO
  val FSQRT_PE_RATIO: Int    = 8    // `FSQRT_PE_RATIO
  val FCVT_PE_RATIO:  Int    = 8    // `FCVT_PE_RATIO
  val FNCP_PE_RATIO:  Int    = 2    // `FNCP_PE_RATIO

  // ---------------------------------------------------------------------------
  // ICache knobs  (VX_config.vh defaults)
  // ---------------------------------------------------------------------------

  val ICACHE_ENABLED:     Int = 1              // `ICACHE_ENABLED
  val NUM_ICACHES:        Int = up(SOCKET_SIZE / 4)  // `NUM_ICACHES
  val ICACHE_SIZE:        Int = 16384          // `ICACHE_SIZE (bytes)
  val ICACHE_CRSQ_SIZE:   Int = 2              // `ICACHE_CRSQ_SIZE
  val ICACHE_MSHR_SIZE:   Int = 16             // `ICACHE_MSHR_SIZE
  val ICACHE_MREQ_SIZE:   Int = 4              // `ICACHE_MREQ_SIZE
  val ICACHE_MRSQ_SIZE:   Int = 0              // `ICACHE_MRSQ_SIZE
  val ICACHE_NUM_WAYS:    Int = 4              // `ICACHE_NUM_WAYS
  val ICACHE_REPL_POLICY: Int = 1              // `ICACHE_REPL_POLICY
  val ICACHE_MEM_PORTS:   Int = 1              // `ICACHE_MEM_PORTS

  // ---------------------------------------------------------------------------
  // DCache knobs  (VX_config.vh defaults)
  // ---------------------------------------------------------------------------

  val DCACHE_ENABLED:     Int = 1              // `DCACHE_ENABLED
  val NUM_DCACHES:        Int = up(SOCKET_SIZE / 4)  // `NUM_DCACHES
  val DCACHE_SIZE:        Int = 16384          // `DCACHE_SIZE (bytes)
  val DCACHE_CRSQ_SIZE:   Int = 2              // `DCACHE_CRSQ_SIZE
  val DCACHE_MSHR_SIZE:   Int = 16             // `DCACHE_MSHR_SIZE
  val DCACHE_MREQ_SIZE:   Int = 4              // `DCACHE_MREQ_SIZE
  val DCACHE_MRSQ_SIZE:   Int = 4              // `DCACHE_MRSQ_SIZE
  val DCACHE_NUM_WAYS:    Int = 4              // `DCACHE_NUM_WAYS
  val DCACHE_WRITEBACK:   Int = 0              // `DCACHE_WRITEBACK
  val DCACHE_DIRTYBYTES:  Int = 0              // `DCACHE_DIRTYBYTES (= DCACHE_WRITEBACK)
  val DCACHE_REPL_POLICY: Int = 1              // `DCACHE_REPL_POLICY

  // L1 memory ports: MIN(DCACHE_NUM_BANKS, PLATFORM_MEMORY_NUM_BANKS)
  // DCACHE_NUM_BANKS = MIN(DCACHE_NUM_REQS, 16); DCACHE_NUM_REQS derived in VortexGPUPkg
  // Use platform banks as a safe upper bound default
  val L1_MEM_PORTS: Int      = min(PLATFORM_MEMORY_NUM_BANKS, PLATFORM_MEMORY_NUM_BANKS)  // `L1_MEM_PORTS (placeholder — actual value from DCACHE_NUM_BANKS at runtime)

  // ---------------------------------------------------------------------------
  // LMEM knobs
  // ---------------------------------------------------------------------------

  val LMEM_ENABLED:    Int   = 1              // `LMEM_ENABLED
  val LMEM_NUM_BANKS:  Int   = NUM_LSU_LANES  // `LMEM_NUM_BANKS

  // ---------------------------------------------------------------------------
  // L2 cache knobs  (disabled by default)
  // ---------------------------------------------------------------------------

  val L2_ENABLED:      Int   = 0              // `L2_ENABLED (L2_ENABLE not defined)
  val L2_CACHE_SIZE:   Int   = 1048576        // `L2_CACHE_SIZE (bytes)
  val L2_CRSQ_SIZE:    Int   = 2              // `L2_CRSQ_SIZE
  val L2_MSHR_SIZE:    Int   = 16             // `L2_MSHR_SIZE
  val L2_MREQ_SIZE:    Int   = 4              // `L2_MREQ_SIZE
  val L2_MRSQ_SIZE:    Int   = 4              // `L2_MRSQ_SIZE
  val L2_NUM_WAYS:     Int   = 8              // `L2_NUM_WAYS
  val L2_WRITEBACK:    Int   = 0              // `L2_WRITEBACK
  val L2_DIRTYBYTES:   Int   = 0              // `L2_DIRTYBYTES
  val L2_REPL_POLICY:  Int   = 1              // `L2_REPL_POLICY
  // L2_MEM_PORTS: when L2 disabled = MIN(L2_NUM_REQS, PLATFORM_MEMORY_NUM_BANKS)
  // L2_NUM_REQS = NUM_SOCKETS * L1_MEM_PORTS (computed in VortexGPUPkg)
  val L2_MEM_PORTS:    Int   = PLATFORM_MEMORY_NUM_BANKS  // `L2_MEM_PORTS (simplified for disabled case)

  // ---------------------------------------------------------------------------
  // L3 cache knobs  (disabled by default)
  // ---------------------------------------------------------------------------

  val L3_ENABLED:      Int   = 0              // `L3_ENABLED (L3_ENABLE not defined)
  val L3_CACHE_SIZE:   Int   = 2097152        // `L3_CACHE_SIZE (bytes)
  val L3_CRSQ_SIZE:    Int   = 2              // `L3_CRSQ_SIZE
  val L3_MSHR_SIZE:    Int   = 16             // `L3_MSHR_SIZE
  val L3_MREQ_SIZE:    Int   = 4              // `L3_MREQ_SIZE
  val L3_MRSQ_SIZE:    Int   = 4              // `L3_MRSQ_SIZE
  val L3_NUM_WAYS:     Int   = 8              // `L3_NUM_WAYS
  val L3_WRITEBACK:    Int   = 0              // `L3_WRITEBACK
  val L3_DIRTYBYTES:   Int   = 0              // `L3_DIRTYBYTES
  val L3_REPL_POLICY:  Int   = 1              // `L3_REPL_POLICY
  // L3_MEM_PORTS: when L3 disabled = MIN(L3_NUM_REQS, PLATFORM_MEMORY_NUM_BANKS)
  // L3_NUM_REQS = NUM_CLUSTERS * L2_MEM_PORTS; for 1 cluster, 2 banks => 2
  val L3_MEM_PORTS:    Int   = 2              // `L3_MEM_PORTS (= PLATFORM_MEMORY_NUM_BANKS)

  // ---------------------------------------------------------------------------
  // Extension enables (1/0 flags)  (VX_config.vh)
  // ---------------------------------------------------------------------------

  val EXT_M_ENABLED:       Int = 1   // `EXT_M_ENABLED (EXT_M_ENABLE defined by default)
  val EXT_F_ENABLED:       Int = 1   // `EXT_F_ENABLED (EXT_F_ENABLE defined by default)
  val EXT_D_ENABLED:       Int = 0   // `EXT_D_ENABLED (no XLEN_64)
  val EXT_A_ENABLED:       Int = 0   // `EXT_A_ENABLED
  val EXT_C_ENABLED:       Int = 0   // `EXT_C_ENABLED
  val EXT_V_ENABLED:       Int = 0   // `EXT_V_ENABLED
  val EXT_ZICOND_ENABLED:  Int = 1   // `EXT_ZICOND_ENABLED (defined by default)
  val EXT_TCU_ENABLED:     Int = 0   // `EXT_TCU_ENABLED

  val GBAR_ENABLED: Int        = 0   // `GBAR_ENABLED

  // ---------------------------------------------------------------------------
  // ISA standard bit positions  (VX_config.vh `ISA_STD_*)
  // ---------------------------------------------------------------------------

  val ISA_STD_A: Int  = 0    // A - Atomic Instructions extension
  val ISA_STD_C: Int  = 2    // C - Compressed extension
  val ISA_STD_D: Int  = 3    // D - Double precision floating-point extension
  val ISA_STD_E: Int  = 4    // E - RV32E base ISA
  val ISA_STD_F: Int  = 5    // F - Single precision floating-point extension
  val ISA_STD_H: Int  = 7    // H - Hypervisor mode implemented
  val ISA_STD_I: Int  = 8    // I - RV32I/64I/128I base ISA
  val ISA_STD_N: Int  = 13   // N - User level interrupts supported
  val ISA_STD_Q: Int  = 16   // Q - Quad-precision floating-point extension
  val ISA_STD_S: Int  = 18   // S - Supervisor mode implemented
  val ISA_STD_V: Int  = 21   // V - Vector extension

  // ---------------------------------------------------------------------------
  // ISA extension bit positions  (VX_config.vh `ISA_EXT_*)
  // ---------------------------------------------------------------------------

  val ISA_EXT_ICACHE:  Int = 0   // `ISA_EXT_ICACHE
  val ISA_EXT_DCACHE:  Int = 1   // `ISA_EXT_DCACHE
  val ISA_EXT_L2CACHE: Int = 2   // `ISA_EXT_L2CACHE
  val ISA_EXT_L3CACHE: Int = 3   // `ISA_EXT_L3CACHE
  val ISA_EXT_LMEM:    Int = 4   // `ISA_EXT_LMEM
  val ISA_EXT_ZICOND:  Int = 5   // `ISA_EXT_ZICOND
  val ISA_EXT_TCU:     Int = 6   // `ISA_EXT_TCU

  // Computed MISA fields
  val MISA_EXT: Int =
    (ICACHE_ENABLED << ISA_EXT_ICACHE)  |
    (DCACHE_ENABLED << ISA_EXT_DCACHE)  |
    (L2_ENABLED     << ISA_EXT_L2CACHE) |
    (L3_ENABLED     << ISA_EXT_L3CACHE) |
    (LMEM_ENABLED   << ISA_EXT_LMEM)    |
    (EXT_ZICOND_ENABLED << ISA_EXT_ZICOND) |
    (EXT_TCU_ENABLED << ISA_EXT_TCU)

  val MISA_STD: Int =
    (EXT_A_ENABLED  <<  0) | // A - Atomic Instructions extension
    (0              <<  1) | // B - Bit operations (reserved)
    (EXT_C_ENABLED  <<  2) | // C - Compressed extension
    (EXT_D_ENABLED  <<  3) | // D - Double precision FP extension
    (0              <<  4) | // E - RV32E base ISA
    (EXT_F_ENABLED  <<  5) | // F - Single precision FP extension
    (0              <<  6) | // G - Additional standard extensions
    (0              <<  7) | // H - Hypervisor mode
    (1              <<  8) | // I - RV32I/64I/128I base ISA
    (0              <<  9) | // J - Reserved
    (0              << 10) | // K - Reserved
    (0              << 11) | // L - Bit operations (reserved)
    (EXT_M_ENABLED  << 12) | // M - Integer Multiply/Divide extension
    (0              << 13) | // N - User level interrupts
    (0              << 14) | // O - Reserved
    (0              << 15) | // P - Packed-SIMD (reserved)
    (0              << 16) | // Q - Quad-precision FP
    (0              << 17) | // R - Reserved
    (0              << 18) | // S - Supervisor mode
    (0              << 19) | // T - Transactional Memory (reserved)
    (1              << 20) | // U - User mode implemented
    (EXT_V_ENABLED  << 21) | // V - Vector extension
    (0              << 22) | // W - Reserved
    (1              << 23) | // X - Non-standard extensions present
    (0              << 24) | // Y - Reserved
    (0              << 25)   // Z - Reserved

  // ---------------------------------------------------------------------------
  // Device identification  (VX_config.vh)
  // ---------------------------------------------------------------------------

  val VENDOR_ID:         Int = 0   // `VENDOR_ID
  val ARCHITECTURE_ID:   Int = 0   // `ARCHITECTURE_ID
  val IMPLEMENTATION_ID: Int = 0   // `IMPLEMENTATION_ID

  // ---------------------------------------------------------------------------
  // VX_types.vh — CSR / DCR address constants
  // ---------------------------------------------------------------------------

  // Address widths
  val VX_CSR_ADDR_BITS: Int = 12   // `VX_CSR_ADDR_BITS
  val VX_DCR_ADDR_BITS: Int = 12   // `VX_DCR_ADDR_BITS

  // ---------------------------------------------------------------------------
  // DCR base state registers  (VX_types.vh `VX_DCR_BASE_*)
  // ---------------------------------------------------------------------------

  val VX_DCR_BASE_STATE_BEGIN:   Int = 0x001   // `VX_DCR_BASE_STATE_BEGIN
  val VX_DCR_BASE_STARTUP_ADDR0: Int = 0x001   // `VX_DCR_BASE_STARTUP_ADDR0  (low 32b)
  val VX_DCR_BASE_STARTUP_ADDR1: Int = 0x002   // `VX_DCR_BASE_STARTUP_ADDR1  (high 32b)
  val VX_DCR_BASE_STARTUP_ARG0:  Int = 0x003   // `VX_DCR_BASE_STARTUP_ARG0   (low 32b)
  val VX_DCR_BASE_STARTUP_ARG1:  Int = 0x004   // `VX_DCR_BASE_STARTUP_ARG1   (high 32b)
  val VX_DCR_BASE_MPM_CLASS:     Int = 0x005   // `VX_DCR_BASE_MPM_CLASS
  val VX_DCR_BASE_STATE_END:     Int = 0x006   // `VX_DCR_BASE_STATE_END

  val VX_DCR_BASE_STATE_COUNT: Int =
    VX_DCR_BASE_STATE_END - VX_DCR_BASE_STATE_BEGIN   // `VX_DCR_BASE_STATE_COUNT

  // Machine Performance-monitoring counter classes  (`VX_DCR_MPM_CLASS_*)
  val VX_DCR_MPM_CLASS_NONE: Int = 0   // `VX_DCR_MPM_CLASS_NONE
  val VX_DCR_MPM_CLASS_CORE: Int = 1   // `VX_DCR_MPM_CLASS_CORE
  val VX_DCR_MPM_CLASS_MEM:  Int = 2   // `VX_DCR_MPM_CLASS_MEM

  // ---------------------------------------------------------------------------
  // User Floating-Point CSRs  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_FFLAGS: Int = 0x001   // `VX_CSR_FFLAGS — floating-point accrued exceptions
  val VX_CSR_FRM:    Int = 0x002   // `VX_CSR_FRM    — floating-point dynamic rounding mode
  val VX_CSR_FCSR:   Int = 0x003   // `VX_CSR_FCSR   — floating-point control and status

  val VX_CSR_SATP:    Int = 0x180  // `VX_CSR_SATP   — supervisor address translation/protection

  val VX_CSR_PMPCFG0:  Int = 0x3A0  // `VX_CSR_PMPCFG0
  val VX_CSR_PMPADDR0: Int = 0x3B0  // `VX_CSR_PMPADDR0

  // ---------------------------------------------------------------------------
  // Machine-mode CSRs  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_MSTATUS:  Int = 0x300   // `VX_CSR_MSTATUS  — machine status
  val VX_CSR_MISA:     Int = 0x301   // `VX_CSR_MISA     — machine ISA and extensions
  val VX_CSR_MEDELEG:  Int = 0x302   // `VX_CSR_MEDELEG  — machine exception delegation
  val VX_CSR_MIDELEG:  Int = 0x303   // `VX_CSR_MIDELEG  — machine interrupt delegation
  val VX_CSR_MIE:      Int = 0x304   // `VX_CSR_MIE      — machine interrupt-enable
  val VX_CSR_MTVEC:    Int = 0x305   // `VX_CSR_MTVEC    — machine trap-handler base addr

  val VX_CSR_MSCRATCH: Int = 0x340   // `VX_CSR_MSCRATCH — machine scratch
  val VX_CSR_MEPC:     Int = 0x341   // `VX_CSR_MEPC     — machine exception program counter
  val VX_CSR_MCAUSE:   Int = 0x342   // `VX_CSR_MCAUSE   — machine trap cause

  val VX_CSR_MNSTATUS: Int = 0x744   // `VX_CSR_MNSTATUS

  val VX_CSR_MPM_BASE:   Int = 0xB00  // `VX_CSR_MPM_BASE   — machine perf counter base
  val VX_CSR_MPM_BASE_H: Int = 0xB80  // `VX_CSR_MPM_BASE_H — machine perf counter base (high)
  val VX_CSR_MPM_USER:   Int = 0xB03  // `VX_CSR_MPM_USER
  val VX_CSR_MPM_USER_H: Int = 0xB83  // `VX_CSR_MPM_USER_H

  // ---------------------------------------------------------------------------
  // Machine perf-monitoring counters — standard  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_MCYCLE:          Int = 0xB00  // `VX_CSR_MCYCLE
  val VX_CSR_MCYCLE_H:        Int = 0xB80  // `VX_CSR_MCYCLE_H
  val VX_CSR_MPM_RESERVED:    Int = 0xB01  // `VX_CSR_MPM_RESERVED
  val VX_CSR_MPM_RESERVED_H:  Int = 0xB81  // `VX_CSR_MPM_RESERVED_H
  val VX_CSR_MINSTRET:        Int = 0xB02  // `VX_CSR_MINSTRET
  val VX_CSR_MINSTRET_H:      Int = 0xB82  // `VX_CSR_MINSTRET_H

  // ---------------------------------------------------------------------------
  // Machine perf-monitoring counters — class 1 (pipeline)  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_MPM_SCHED_ID:    Int = 0xB03  // `VX_CSR_MPM_SCHED_ID   — scheduler idle cycles
  val VX_CSR_MPM_SCHED_ID_H:  Int = 0xB83  // `VX_CSR_MPM_SCHED_ID_H
  val VX_CSR_MPM_SCHED_ST:    Int = 0xB04  // `VX_CSR_MPM_SCHED_ST   — scheduler stall cycles
  val VX_CSR_MPM_SCHED_ST_H:  Int = 0xB84  // `VX_CSR_MPM_SCHED_ST_H
  val VX_CSR_MPM_IBUF_ST:     Int = 0xB05  // `VX_CSR_MPM_IBUF_ST    — ibuffer stall cycles
  val VX_CSR_MPM_IBUF_ST_H:   Int = 0xB85  // `VX_CSR_MPM_IBUF_ST_H
  val VX_CSR_MPM_SCRB_ST:     Int = 0xB06  // `VX_CSR_MPM_SCRB_ST    — scoreboard stalls
  val VX_CSR_MPM_SCRB_ST_H:   Int = 0xB86  // `VX_CSR_MPM_SCRB_ST_H
  val VX_CSR_MPM_OPDS_ST:     Int = 0xB07  // `VX_CSR_MPM_OPDS_ST    — operand stalls
  val VX_CSR_MPM_OPDS_ST_H:   Int = 0xB87  // `VX_CSR_MPM_OPDS_ST_H
  val VX_CSR_MPM_SCRB_ALU:    Int = 0xB08  // `VX_CSR_MPM_SCRB_ALU   — ALU scoreboard uses
  val VX_CSR_MPM_SCRB_ALU_H:  Int = 0xB88  // `VX_CSR_MPM_SCRB_ALU_H
  val VX_CSR_MPM_SCRB_FPU:    Int = 0xB09  // `VX_CSR_MPM_SCRB_FPU   — FPU scoreboard uses
  val VX_CSR_MPM_SCRB_FPU_H:  Int = 0xB89  // `VX_CSR_MPM_SCRB_FPU_H
  val VX_CSR_MPM_SCRB_LSU:    Int = 0xB0A  // `VX_CSR_MPM_SCRB_LSU   — LSU scoreboard uses
  val VX_CSR_MPM_SCRB_LSU_H:  Int = 0xB8A  // `VX_CSR_MPM_SCRB_LSU_H
  val VX_CSR_MPM_SCRB_SFU:    Int = 0xB0B  // `VX_CSR_MPM_SCRB_SFU   — SFU scoreboard uses
  val VX_CSR_MPM_SCRB_SFU_H:  Int = 0xB8B  // `VX_CSR_MPM_SCRB_SFU_H
  val VX_CSR_MPM_SCRB_CSRS:   Int = 0xB0C  // `VX_CSR_MPM_SCRB_CSRS  — CSR unit uses
  val VX_CSR_MPM_SCRB_CSRS_H: Int = 0xB8C  // `VX_CSR_MPM_SCRB_CSRS_H
  val VX_CSR_MPM_SCRB_WCTL:   Int = 0xB0D  // `VX_CSR_MPM_SCRB_WCTL  — warp-control unit uses
  val VX_CSR_MPM_SCRB_WCTL_H: Int = 0xB8D  // `VX_CSR_MPM_SCRB_WCTL_H
  val VX_CSR_MPM_SCRB_VPU:    Int = 0xB13  // `VX_CSR_MPM_SCRB_VPU   — VPU scoreboard uses
  val VX_CSR_MPM_SCRB_VPU_H:  Int = 0xB93  // `VX_CSR_MPM_SCRB_VPU_H
  val VX_CSR_MPM_SCRB_TCU:    Int = 0xB14  // `VX_CSR_MPM_SCRB_TCU   — TCU scoreboard uses
  val VX_CSR_MPM_SCRB_TCU_H:  Int = 0xB94  // `VX_CSR_MPM_SCRB_TCU_H

  // Memory performance (class 1)
  val VX_CSR_MPM_IFETCHES:    Int = 0xB0E  // `VX_CSR_MPM_IFETCHES   — instruction fetches
  val VX_CSR_MPM_IFETCHES_H:  Int = 0xB8E  // `VX_CSR_MPM_IFETCHES_H
  val VX_CSR_MPM_LOADS:       Int = 0xB0F  // `VX_CSR_MPM_LOADS      — load operations
  val VX_CSR_MPM_LOADS_H:     Int = 0xB8F  // `VX_CSR_MPM_LOADS_H
  val VX_CSR_MPM_STORES:      Int = 0xB10  // `VX_CSR_MPM_STORES     — store operations
  val VX_CSR_MPM_STORES_H:    Int = 0xB90  // `VX_CSR_MPM_STORES_H
  val VX_CSR_MPM_IFETCH_LT:   Int = 0xB11  // `VX_CSR_MPM_IFETCH_LT  — ifetch latency
  val VX_CSR_MPM_IFETCH_LT_H: Int = 0xB91  // `VX_CSR_MPM_IFETCH_LT_H
  val VX_CSR_MPM_LOAD_LT:     Int = 0xB12  // `VX_CSR_MPM_LOAD_LT    — load latency
  val VX_CSR_MPM_LOAD_LT_H:   Int = 0xB92  // `VX_CSR_MPM_LOAD_LT_H

  // ---------------------------------------------------------------------------
  // Machine perf-monitoring counters — class 2 (memory)  (VX_types.vh)
  // ---------------------------------------------------------------------------

  // ICache
  val VX_CSR_MPM_ICACHE_READS:    Int = 0xB03  // `VX_CSR_MPM_ICACHE_READS   — total reads
  val VX_CSR_MPM_ICACHE_READS_H:  Int = 0xB83  // `VX_CSR_MPM_ICACHE_READS_H
  val VX_CSR_MPM_ICACHE_MISS_R:   Int = 0xB04  // `VX_CSR_MPM_ICACHE_MISS_R  — read misses
  val VX_CSR_MPM_ICACHE_MISS_R_H: Int = 0xB84  // `VX_CSR_MPM_ICACHE_MISS_R_H
  val VX_CSR_MPM_ICACHE_MSHR_ST:  Int = 0xB05  // `VX_CSR_MPM_ICACHE_MSHR_ST — MSHR stalls
  val VX_CSR_MPM_ICACHE_MSHR_ST_H:Int = 0xB85  // `VX_CSR_MPM_ICACHE_MSHR_ST_H

  // DCache
  val VX_CSR_MPM_DCACHE_READS:    Int = 0xB06  // `VX_CSR_MPM_DCACHE_READS   — total reads
  val VX_CSR_MPM_DCACHE_READS_H:  Int = 0xB86  // `VX_CSR_MPM_DCACHE_READS_H
  val VX_CSR_MPM_DCACHE_WRITES:   Int = 0xB07  // `VX_CSR_MPM_DCACHE_WRITES  — total writes
  val VX_CSR_MPM_DCACHE_WRITES_H: Int = 0xB87  // `VX_CSR_MPM_DCACHE_WRITES_H
  val VX_CSR_MPM_DCACHE_MISS_R:   Int = 0xB08  // `VX_CSR_MPM_DCACHE_MISS_R  — read misses
  val VX_CSR_MPM_DCACHE_MISS_R_H: Int = 0xB88  // `VX_CSR_MPM_DCACHE_MISS_R_H
  val VX_CSR_MPM_DCACHE_MISS_W:   Int = 0xB09  // `VX_CSR_MPM_DCACHE_MISS_W  — write misses
  val VX_CSR_MPM_DCACHE_MISS_W_H: Int = 0xB89  // `VX_CSR_MPM_DCACHE_MISS_W_H
  val VX_CSR_MPM_DCACHE_BANK_ST:  Int = 0xB0A  // `VX_CSR_MPM_DCACHE_BANK_ST — bank conflicts
  val VX_CSR_MPM_DCACHE_BANK_ST_H:Int = 0xB8A  // `VX_CSR_MPM_DCACHE_BANK_ST_H
  val VX_CSR_MPM_DCACHE_MSHR_ST:  Int = 0xB0B  // `VX_CSR_MPM_DCACHE_MSHR_ST — MSHR stalls
  val VX_CSR_MPM_DCACHE_MSHR_ST_H:Int = 0xB8B  // `VX_CSR_MPM_DCACHE_MSHR_ST_H

  // L2 cache
  val VX_CSR_MPM_L2CACHE_READS:    Int = 0xB0C  // `VX_CSR_MPM_L2CACHE_READS   — total reads
  val VX_CSR_MPM_L2CACHE_READS_H:  Int = 0xB8C  // `VX_CSR_MPM_L2CACHE_READS_H
  val VX_CSR_MPM_L2CACHE_WRITES:   Int = 0xB0D  // `VX_CSR_MPM_L2CACHE_WRITES  — total writes
  val VX_CSR_MPM_L2CACHE_WRITES_H: Int = 0xB8D  // `VX_CSR_MPM_L2CACHE_WRITES_H
  val VX_CSR_MPM_L2CACHE_MISS_R:   Int = 0xB0E  // `VX_CSR_MPM_L2CACHE_MISS_R  — read misses
  val VX_CSR_MPM_L2CACHE_MISS_R_H: Int = 0xB8E  // `VX_CSR_MPM_L2CACHE_MISS_R_H
  val VX_CSR_MPM_L2CACHE_MISS_W:   Int = 0xB0F  // `VX_CSR_MPM_L2CACHE_MISS_W  — write misses
  val VX_CSR_MPM_L2CACHE_MISS_W_H: Int = 0xB8F  // `VX_CSR_MPM_L2CACHE_MISS_W_H
  val VX_CSR_MPM_L2CACHE_BANK_ST:  Int = 0xB10  // `VX_CSR_MPM_L2CACHE_BANK_ST — bank conflicts
  val VX_CSR_MPM_L2CACHE_BANK_ST_H:Int = 0xB90  // `VX_CSR_MPM_L2CACHE_BANK_ST_H
  val VX_CSR_MPM_L2CACHE_MSHR_ST:  Int = 0xB11  // `VX_CSR_MPM_L2CACHE_MSHR_ST — MSHR stalls
  val VX_CSR_MPM_L2CACHE_MSHR_ST_H:Int = 0xB91  // `VX_CSR_MPM_L2CACHE_MSHR_ST_H

  // L3 cache
  val VX_CSR_MPM_L3CACHE_READS:    Int = 0xB12  // `VX_CSR_MPM_L3CACHE_READS   — total reads
  val VX_CSR_MPM_L3CACHE_READS_H:  Int = 0xB92  // `VX_CSR_MPM_L3CACHE_READS_H
  val VX_CSR_MPM_L3CACHE_WRITES:   Int = 0xB13  // `VX_CSR_MPM_L3CACHE_WRITES  — total writes
  val VX_CSR_MPM_L3CACHE_WRITES_H: Int = 0xB93  // `VX_CSR_MPM_L3CACHE_WRITES_H
  val VX_CSR_MPM_L3CACHE_MISS_R:   Int = 0xB14  // `VX_CSR_MPM_L3CACHE_MISS_R  — read misses
  val VX_CSR_MPM_L3CACHE_MISS_R_H: Int = 0xB94  // `VX_CSR_MPM_L3CACHE_MISS_R_H
  val VX_CSR_MPM_L3CACHE_MISS_W:   Int = 0xB15  // `VX_CSR_MPM_L3CACHE_MISS_W  — write misses
  val VX_CSR_MPM_L3CACHE_MISS_W_H: Int = 0xB95  // `VX_CSR_MPM_L3CACHE_MISS_W_H
  val VX_CSR_MPM_L3CACHE_BANK_ST:  Int = 0xB16  // `VX_CSR_MPM_L3CACHE_BANK_ST — bank conflicts
  val VX_CSR_MPM_L3CACHE_BANK_ST_H:Int = 0xB96  // `VX_CSR_MPM_L3CACHE_BANK_ST_H
  val VX_CSR_MPM_L3CACHE_MSHR_ST:  Int = 0xB17  // `VX_CSR_MPM_L3CACHE_MSHR_ST — MSHR stalls
  val VX_CSR_MPM_L3CACHE_MSHR_ST_H:Int = 0xB97  // `VX_CSR_MPM_L3CACHE_MSHR_ST_H

  // Main memory
  val VX_CSR_MPM_MEM_READS:     Int = 0xB18  // `VX_CSR_MPM_MEM_READS    — total reads
  val VX_CSR_MPM_MEM_READS_H:   Int = 0xB98  // `VX_CSR_MPM_MEM_READS_H
  val VX_CSR_MPM_MEM_WRITES:    Int = 0xB19  // `VX_CSR_MPM_MEM_WRITES   — total writes
  val VX_CSR_MPM_MEM_WRITES_H:  Int = 0xB99  // `VX_CSR_MPM_MEM_WRITES_H
  val VX_CSR_MPM_MEM_LT:        Int = 0xB1A  // `VX_CSR_MPM_MEM_LT       — memory latency
  val VX_CSR_MPM_MEM_LT_H:      Int = 0xB9A  // `VX_CSR_MPM_MEM_LT_H
  val VX_CSR_MPM_MEM_BANK_ST:   Int = 0xB1E  // `VX_CSR_MPM_MEM_BANK_ST  — bank conflicts
  val VX_CSR_MPM_MEM_BANK_ST_H: Int = 0xB9E  // `VX_CSR_MPM_MEM_BANK_ST_H

  // Local memory
  val VX_CSR_MPM_LMEM_READS:    Int = 0xB1B  // `VX_CSR_MPM_LMEM_READS   — memory reads
  val VX_CSR_MPM_LMEM_READS_H:  Int = 0xB9B  // `VX_CSR_MPM_LMEM_READS_H
  val VX_CSR_MPM_LMEM_WRITES:   Int = 0xB1C  // `VX_CSR_MPM_LMEM_WRITES  — memory writes
  val VX_CSR_MPM_LMEM_WRITES_H: Int = 0xB9C  // `VX_CSR_MPM_LMEM_WRITES_H
  val VX_CSR_MPM_LMEM_BANK_ST:  Int = 0xB1D  // `VX_CSR_MPM_LMEM_BANK_ST — bank conflicts
  val VX_CSR_MPM_LMEM_BANK_ST_H:Int = 0xB9D  // `VX_CSR_MPM_LMEM_BANK_ST_H

  // Coalescer
  val VX_CSR_MPM_COALESCER_MISS:  Int = 0xB1F  // `VX_CSR_MPM_COALESCER_MISS   — coalescer misses
  val VX_CSR_MPM_COALESCER_MISS_H:Int = 0xB9F  // `VX_CSR_MPM_COALESCER_MISS_H

  // ---------------------------------------------------------------------------
  // Machine Information Registers  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_MVENDORID: Int = 0xF11   // `VX_CSR_MVENDORID
  val VX_CSR_MARCHID:   Int = 0xF12   // `VX_CSR_MARCHID
  val VX_CSR_MIMPID:    Int = 0xF13   // `VX_CSR_MIMPID
  val VX_CSR_MHARTID:   Int = 0xF14   // `VX_CSR_MHARTID

  // ---------------------------------------------------------------------------
  // Vector CSRs  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_VSTART: Int = 0x008   // `VX_CSR_VSTART
  val VX_CSR_VXSAT:  Int = 0x009   // `VX_CSR_VXSAT
  val VX_CSR_VXRM:   Int = 0x00A   // `VX_CSR_VXRM
  val VX_CSR_VCSR:   Int = 0x00F   // `VX_CSR_VCSR
  val VX_CSR_VL:     Int = 0xC20   // `VX_CSR_VL
  val VX_CSR_VTYPE:  Int = 0xC21   // `VX_CSR_VTYPE
  val VX_CSR_VLENB:  Int = 0xC22   // `VX_CSR_VLENB

  // ---------------------------------------------------------------------------
  // GPU CSRs  (VX_types.vh)
  // ---------------------------------------------------------------------------

  val VX_CSR_THREAD_ID:       Int = 0xCC0  // `VX_CSR_THREAD_ID      — current thread ID
  val VX_CSR_WARP_ID:         Int = 0xCC1  // `VX_CSR_WARP_ID        — current warp ID
  val VX_CSR_CORE_ID:         Int = 0xCC2  // `VX_CSR_CORE_ID        — current core ID
  val VX_CSR_ACTIVE_WARPS:    Int = 0xCC3  // `VX_CSR_ACTIVE_WARPS   — active warp mask
  val VX_CSR_ACTIVE_THREADS:  Int = 0xCC4  // `VX_CSR_ACTIVE_THREADS — active thread mask (also used in LLVM)

  val VX_CSR_NUM_THREADS:     Int = 0xFC0  // `VX_CSR_NUM_THREADS    — number of threads per warp
  val VX_CSR_NUM_WARPS:       Int = 0xFC1  // `VX_CSR_NUM_WARPS      — number of warps per core
  val VX_CSR_NUM_CORES:       Int = 0xFC2  // `VX_CSR_NUM_CORES      — number of cores
  val VX_CSR_LOCAL_MEM_BASE:  Int = 0xFC3  // `VX_CSR_LOCAL_MEM_BASE — local memory base address
}
