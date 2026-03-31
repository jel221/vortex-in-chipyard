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

// Chisel translations of the pipeline-stage SystemVerilog interfaces:
//   VX_fetch_if.sv          → FetchBundle / FetchIf
//   VX_decode_if.sv         → DecodeBundle / DecodeIf
//   VX_ibuffer_if.sv        → IbufferBundle / IbufferIf
//   VX_scoreboard_if.sv     → ScoreboardBundle / ScoreboardIf
//   VX_dispatch_if.sv       → DispatchBundle / DispatchIf
//   VX_operands_if.sv       → OperandsBundle / OperandsIf
//   VX_execute_if.sv        → ExecuteIf  (generic)
//   VX_result_if.sv         → ResultIf   (generic)
//   VX_writeback_if.sv      → WritebackBundle / WritebackIf
//   VX_commit_if.sv         → CommitBundle / CommitIf
//   VX_schedule_if.sv       → ScheduleBundle / ScheduleIf
//   VX_branch_ctl_if.sv     → BranchCtlBundle
//   VX_warp_ctl_if.sv       → WarpCtlBundle
//   VX_sched_csr_if.sv      → SchedCsrBundle
//   VX_commit_sched_if.sv   → CommitSchedBundle
//   VX_commit_csr_if.sv     → CommitCsrBundle
//   VX_decode_sched_if.sv   → DecodeSchedBundle
//   VX_issue_sched_if.sv    → IssueSchedBundle

package vortex

import chisel3._
import chisel3.util._

// ===========================================================================
// GPU configuration constants – mirrors the SV package / config headers.
// These are captured as constructor arguments wherever the bit-width depends
// on them, mirroring how the SV uses `NUM_WARPS, `NUM_THREADS, etc.
//
// Commonly-used derived widths (mirroring VX_gpu_pkg.sv):
//   NW_WIDTH  = max(1, log2Ceil(NUM_WARPS))   – warp-id width
//   NT_WIDTH  = max(1, log2Ceil(NUM_THREADS)) – thread-id width
//   NB_WIDTH  = max(1, log2Ceil(NUM_BARRIERS))
//   NC_WIDTH  = max(1, log2Ceil(NUM_CORES))
//
// Pipeline-specific widths (also from VX_gpu_pkg.sv):
//   PERF_CTR_BITS  = 44
//   UUID_WIDTH     = 44 (debug builds) / 1 (NDEBUG)
//   PC_BITS        = XLEN (debug) / XLEN-2 (NDEBUG); default XLEN=32
//   EX_BITS        = log2Ceil(NUM_EX_UNITS)           (typically 2 or 3)
//   INST_OP_BITS   = 4
//   INST_ARGS_BITS = ALU_TYPE_BITS + XLEN + 3         (typically 4+32+3=39)
//   NUM_REGS_BITS  = log2Ceil(NUM_REGS)               (typically 6)
//   NUM_SRC_OPDS   = 3
//   ISSUE_WIS_W    = max(1, log2Ceil(NUM_WARPS / ISSUE_WIDTH))
//   SIMD_IDX_W     = max(1, log2Ceil(NUM_THREADS / SIMD_WIDTH))
//   DV_STACK_SIZEW = max(1, log2Ceil(max(1, NUM_THREADS-1)))
// ===========================================================================

// ---------------------------------------------------------------------------
// Helper: produce max(1, log2Ceil(n)) – the SV `UP(CLOG2(n))` idiom.
// ---------------------------------------------------------------------------
private object up {
  def apply(n: Int): Int = if (n <= 1) 1 else math.max(1, log2Ceil(n))
}

// ===========================================================================
// op_args_t union (SV typedef union packed)
//
// In SV this is a union of several packed structs all the same total width
// (INST_ARGS_BITS = ALU_TYPE_BITS + XLEN + 3 = 4+32+3 = 39 bits for RV32).
// Chisel has no union type; we expose the raw bits as a single UInt with
// the same total width and let the consumer interpret sub-fields.
// ===========================================================================

/** Raw instruction argument bits (op_args_t union).
 *
 *  The width equals INST_ARGS_BITS = ALU_TYPE_BITS(4) + xlen + 3.
 *
 *  @param xlen processor word width (32 or 64)
 */
class OpArgsBundle(val xlen: Int = 32) extends Bundle {
  private val instArgsBits = 4 + xlen + 3  // ALU_TYPE_BITS=4, +3 for use_PC/use_imm/is_w
  val bits = UInt(instArgsBits.W)
}

// ===========================================================================
// Warp-control sub-structures (used by VX_warp_ctl_if)
// Translated from VX_gpu_pkg.sv typedef struct packed { ... }
// ===========================================================================

/** tmc_t – thread mask control.
 *  @param numThreads NUM_THREADS
 */
class TmcBundle(val numThreads: Int) extends Bundle {
  val valid = Bool()
  val tmask = UInt(numThreads.W)
}

/** wspawn_t – warp spawn.
 *  @param numWarps NUM_WARPS
 *  @param pcBits   PC_BITS (XLEN or XLEN-2)
 */
class WspawnBundle(val numWarps: Int, val pcBits: Int) extends Bundle {
  val valid = Bool()
  val wmask = UInt(numWarps.W)
  val pc    = UInt(pcBits.W)
}

/** split_t – divergence stack push.
 *  @param numThreads NUM_THREADS
 *  @param pcBits     PC_BITS
 */
class SplitBundle(val numThreads: Int, val pcBits: Int) extends Bundle {
  val valid      = Bool()
  val is_dvg     = Bool()
  val then_tmask = UInt(numThreads.W)
  val else_tmask = UInt(numThreads.W)
  val next_pc    = UInt(pcBits.W)
}

/** join_t – divergence stack pop.
 *  @param dvStackSizeW DV_STACK_SIZEW = UP(CLOG2(UP(NUM_THREADS-1)))
 */
class JoinBundle(val dvStackSizeW: Int) extends Bundle {
  val valid     = Bool()
  val stack_ptr = UInt(dvStackSizeW.W)
}

/** barrier_t – barrier synchronisation.
 *  @param nbWidth NB_WIDTH
 *  @param nwWidth NW_WIDTH
 */
class BarrierBundle(val nbWidth: Int, val nwWidth: Int) extends Bundle {
  val valid     = Bool()
  val id        = UInt(nbWidth.W)
  val is_global = Bool()
  // size_m1 width is MAX(NW_WIDTH, NC_WIDTH) when GBAR_ENABLE, else NW_WIDTH.
  // We always use nwWidth here (callers can widen if needed).
  val size_m1   = UInt(nwWidth.W)
  val is_noop   = Bool()
}

// ===========================================================================
// Pipeline data-type Bundles
// Translated from VX_gpu_pkg.sv typedef struct packed { ... }
// ===========================================================================

/** fetch_t – instruction fetch output.
 *
 *  @param uuidWidth  UUID_WIDTH (44 debug / 1 NDEBUG)
 *  @param nwWidth    NW_WIDTH
 *  @param numThreads NUM_THREADS
 *  @param pcBits     PC_BITS
 */
class FetchDataBundle(
    val uuidWidth:  Int,
    val nwWidth:    Int,
    val numThreads: Int,
    val pcBits:     Int
) extends Bundle {
  val uuid  = UInt(uuidWidth.W)
  val wid   = UInt(nwWidth.W)
  val tmask = UInt(numThreads.W)
  val PC    = UInt(pcBits.W)
  val instr = UInt(32.W)
}

/** decode_t – decoded instruction.
 *
 *  @param uuidWidth   UUID_WIDTH
 *  @param nwWidth     NW_WIDTH
 *  @param numThreads  NUM_THREADS
 *  @param pcBits      PC_BITS
 *  @param exBits      EX_BITS  (log2Ceil(NUM_EX_UNITS), typically 2)
 *  @param numSrcOpds  NUM_SRC_OPDS = 3
 *  @param numRegsBits NUM_REGS_BITS
 *  @param xlen        XLEN
 */
class DecodeDataBundle(
    val uuidWidth:   Int,
    val nwWidth:     Int,
    val numThreads:  Int,
    val pcBits:      Int,
    val exBits:      Int,
    val numSrcOpds:  Int = 3,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid     = UInt(uuidWidth.W)
  val wid      = UInt(nwWidth.W)
  val tmask    = UInt(numThreads.W)
  val PC       = UInt(pcBits.W)
  val ex_type  = UInt(exBits.W)
  val op_type  = UInt(4.W)          // INST_OP_BITS = 4
  val op_args  = new OpArgsBundle(xlen)
  val wb       = Bool()
  val used_rs  = UInt(numSrcOpds.W)
  val rd       = UInt(numRegsBits.W)
  val rs1      = UInt(numRegsBits.W)
  val rs2      = UInt(numRegsBits.W)
  val rs3      = UInt(numRegsBits.W)
}

/** ibuffer_t – instruction buffer entry (wid replaced by per-issue wis).
 *  Same fields as decode_t but wid is dropped.
 */
class IbufferDataBundle(
    val uuidWidth:   Int,
    val numThreads:  Int,
    val pcBits:      Int,
    val exBits:      Int,
    val numSrcOpds:  Int = 3,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid    = UInt(uuidWidth.W)
  val tmask   = UInt(numThreads.W)
  val PC      = UInt(pcBits.W)
  val ex_type = UInt(exBits.W)
  val op_type = UInt(4.W)
  val op_args = new OpArgsBundle(xlen)
  val wb      = Bool()
  val used_rs = UInt(numSrcOpds.W)
  val rd      = UInt(numRegsBits.W)
  val rs1     = UInt(numRegsBits.W)
  val rs2     = UInt(numRegsBits.W)
  val rs3     = UInt(numRegsBits.W)
}

/** scoreboard_t – scoreboard entry (has wis instead of wid).
 *
 *  @param issueWisW  ISSUE_WIS_W = UP(CLOG2(NUM_WARPS / ISSUE_WIDTH))
 */
class ScoreboardDataBundle(
    val uuidWidth:   Int,
    val issueWisW:   Int,
    val numThreads:  Int,
    val pcBits:      Int,
    val exBits:      Int,
    val numSrcOpds:  Int = 3,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid    = UInt(uuidWidth.W)
  val wis     = UInt(issueWisW.W)
  val tmask   = UInt(numThreads.W)
  val PC      = UInt(pcBits.W)
  val ex_type = UInt(exBits.W)
  val op_type = UInt(4.W)
  val op_args = new OpArgsBundle(xlen)
  val wb      = Bool()
  val used_rs = UInt(numSrcOpds.W)
  val rd      = UInt(numRegsBits.W)
  val rs1     = UInt(numRegsBits.W)
  val rs2     = UInt(numRegsBits.W)
  val rs3     = UInt(numRegsBits.W)
}

/** operands_t – operands-fetched stage payload.
 *
 *  @param simdWidth   SIMD_WIDTH (number of SIMD lanes)
 *  @param simdIdxW    SIMD_IDX_W = UP(CLOG2(NUM_THREADS / SIMD_WIDTH))
 */
class OperandsDataBundle(
    val uuidWidth:   Int,
    val issueWisW:   Int,
    val simdIdxW:    Int,
    val simdWidth:   Int,
    val pcBits:      Int,
    val exBits:      Int,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid     = UInt(uuidWidth.W)
  val wis      = UInt(issueWisW.W)
  val sid      = UInt(simdIdxW.W)
  val tmask    = UInt(simdWidth.W)
  val PC       = UInt(pcBits.W)
  val ex_type  = UInt(exBits.W)
  val op_type  = UInt(4.W)
  val op_args  = new OpArgsBundle(xlen)
  val wb       = Bool()
  val rd       = UInt(numRegsBits.W)
  val rs1_data = Vec(simdWidth, UInt(xlen.W))
  val rs2_data = Vec(simdWidth, UInt(xlen.W))
  val rs3_data = Vec(simdWidth, UInt(xlen.W))
  val sop      = Bool()
  val eop      = Bool()
}

/** dispatch_t – dispatch to execution unit.
 *  Very similar to operands_t; op_type narrows to INST_ALU_BITS (4).
 */
class DispatchDataBundle(
    val uuidWidth:   Int,
    val issueWisW:   Int,
    val simdIdxW:    Int,
    val simdWidth:   Int,
    val pcBits:      Int,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid     = UInt(uuidWidth.W)
  val wis      = UInt(issueWisW.W)
  val sid      = UInt(simdIdxW.W)
  val tmask    = UInt(simdWidth.W)
  val PC       = UInt(pcBits.W)
  val op_type  = UInt(4.W)              // INST_ALU_BITS = 4
  val op_args  = new OpArgsBundle(xlen)
  val wb       = Bool()
  val rd       = UInt(numRegsBits.W)
  val rs1_data = Vec(simdWidth, UInt(xlen.W))
  val rs2_data = Vec(simdWidth, UInt(xlen.W))
  val rs3_data = Vec(simdWidth, UInt(xlen.W))
  val sop      = Bool()
  val eop      = Bool()
}

/** commit_t – commit stage payload (uses wid, not wis; NW_WIDTH not ISSUE_WIS_W).
 *
 *  @param nwWidth  NW_WIDTH
 */
class CommitDataBundle(
    val uuidWidth:   Int,
    val nwWidth:     Int,
    val simdIdxW:    Int,
    val simdWidth:   Int,
    val pcBits:      Int,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid  = UInt(uuidWidth.W)
  val wid   = UInt(nwWidth.W)
  val sid   = UInt(simdIdxW.W)
  val tmask = UInt(simdWidth.W)
  val PC    = UInt(pcBits.W)
  val wb    = Bool()
  val rd    = UInt(numRegsBits.W)
  val data  = Vec(simdWidth, UInt(xlen.W))
  val sop   = Bool()
  val eop   = Bool()
}

/** writeback_t – writeback stage payload (uses wis).
 */
class WritebackDataBundle(
    val uuidWidth:   Int,
    val issueWisW:   Int,
    val simdIdxW:    Int,
    val simdWidth:   Int,
    val pcBits:      Int,
    val numRegsBits: Int,
    val xlen:        Int = 32
) extends Bundle {
  val uuid  = UInt(uuidWidth.W)
  val wis   = UInt(issueWisW.W)
  val sid   = UInt(simdIdxW.W)
  val tmask = UInt(simdWidth.W)
  val PC    = UInt(pcBits.W)
  val rd    = UInt(numRegsBits.W)
  val data  = Vec(simdWidth, UInt(xlen.W))
  val sop   = Bool()
  val eop   = Bool()
}

/** schedule_t – scheduler output.
 *
 *  @param nwWidth    NW_WIDTH
 *  @param numThreads NUM_THREADS
 */
class ScheduleDataBundle(
    val uuidWidth:  Int,
    val nwWidth:    Int,
    val numThreads: Int,
    val pcBits:     Int
) extends Bundle {
  val uuid  = UInt(uuidWidth.W)
  val wid   = UInt(nwWidth.W)
  val tmask = UInt(numThreads.W)
  val PC    = UInt(pcBits.W)
}

// ===========================================================================
// Execute / Result generic data types
//
// VX_gpu_pkg.sv declares execute and result types via the macros
// DECL_EXECUTE_T and DECL_RESULT_T, parameterised by the number of lanes
// (__lanes__).  We mirror this with two generic Chisel Bundles.
// ===========================================================================

/** Generic execute-stage data (from DECL_EXECUTE_T macro).
 *
 *  @param uuidWidth   UUID_WIDTH
 *  @param nwWidth     NW_WIDTH
 *  @param numLanes    number of SIMD lanes for this unit
 *  @param pcBits      PC_BITS
 *  @param numRegsBits NUM_REGS_BITS
 *  @param numThreads  NUM_THREADS (used for pid width = log2UP(NUM_THREADS/numLanes))
 *  @param xlen        XLEN
 */
class ExecuteDataBundle(
    val uuidWidth:   Int,
    val nwWidth:     Int,
    val numLanes:    Int,
    val pcBits:      Int,
    val numRegsBits: Int,
    val numThreads:  Int,
    val xlen:        Int = 32
) extends Bundle {
  private val pidBits = math.max(1, log2Ceil(math.max(1, numThreads / numLanes)))
  val uuid     = UInt(uuidWidth.W)
  val wid      = UInt(nwWidth.W)
  val tmask    = UInt(numLanes.W)
  val PC       = UInt(pcBits.W)
  val op_type  = UInt(4.W)              // INST_ALU_BITS = 4
  val op_args  = new OpArgsBundle(xlen)
  val wb       = Bool()
  val rd       = UInt(numRegsBits.W)
  val rs1_data = Vec(numLanes, UInt(xlen.W))
  val rs2_data = Vec(numLanes, UInt(xlen.W))
  val rs3_data = Vec(numLanes, UInt(xlen.W))
  val pid      = UInt(pidBits.W)
  val sop      = Bool()
  val eop      = Bool()
}

/** Generic result-stage data (from DECL_RESULT_T macro). */
class ResultDataBundle(
    val uuidWidth:   Int,
    val nwWidth:     Int,
    val numLanes:    Int,
    val pcBits:      Int,
    val numRegsBits: Int,
    val numThreads:  Int,
    val xlen:        Int = 32
) extends Bundle {
  private val pidBits = math.max(1, log2Ceil(math.max(1, numThreads / numLanes)))
  val uuid  = UInt(uuidWidth.W)
  val wid   = UInt(nwWidth.W)
  val tmask = UInt(numLanes.W)
  val PC    = UInt(pcBits.W)
  val wb    = Bool()
  val rd    = UInt(numRegsBits.W)
  val data  = Vec(numLanes, UInt(xlen.W))
  val pid   = UInt(pidBits.W)
  val sop   = Bool()
  val eop   = Bool()
}

// ===========================================================================
// Pipeline interface Bundles
//
// Each SV interface follows one of two patterns:
//   1. Handshake channel  – valid, data, ready  (modelled as DecoupledIO)
//   2. Push / valid-only  – valid, data          (modelled as ValidIO / Valid)
//
// For pattern (1) the Bundle wraps Decoupled(dataBits).
// For both patterns the "master" perspective is the default; wrap with
// Flipped() to get the "slave" perspective.
// ===========================================================================

// ---------------------------------------------------------------------------
// VX_fetch_if
// Has an optional ibuf_pop sideband (active when L1 cache is disabled).
// We always include ibuf_pop here, driven to 0 when unused.
// ---------------------------------------------------------------------------

/** VX_fetch_if – instruction fetch interface (master bundle).
 *
 *  @param numWarps  NUM_WARPS (for ibuf_pop width)
 *  @param data      FetchDataBundle instance
 */
class FetchIf(numWarps: Int, data: FetchDataBundle) extends Bundle {
  val req     = Decoupled(data.cloneType)
  // ibuf_pop flows from slave (L1 disable path) back to master
  val ibuf_pop = Flipped(UInt(numWarps.W))
}

// ---------------------------------------------------------------------------
// VX_decode_if
// Same structure as VX_fetch_if.
// ---------------------------------------------------------------------------

/** VX_decode_if – decode interface (master bundle). */
class DecodeIf(numWarps: Int, data: DecodeDataBundle) extends Bundle {
  val req      = Decoupled(data.cloneType)
  val ibuf_pop = Flipped(UInt(numWarps.W))
}

// ---------------------------------------------------------------------------
// VX_ibuffer_if
// Simple handshake, no sideband.
// ---------------------------------------------------------------------------

/** VX_ibuffer_if – instruction buffer interface (master bundle). */
class IbufferIf(data: IbufferDataBundle) extends Bundle {
  val req = Decoupled(data.cloneType)
}

// ---------------------------------------------------------------------------
// VX_scoreboard_if
// ---------------------------------------------------------------------------

/** VX_scoreboard_if – scoreboard interface (master bundle). */
class ScoreboardIf(data: ScoreboardDataBundle) extends Bundle {
  val req = Decoupled(data.cloneType)
}

// ---------------------------------------------------------------------------
// VX_dispatch_if
// ---------------------------------------------------------------------------

/** VX_dispatch_if – dispatch interface (master bundle). */
class DispatchIf(data: DispatchDataBundle) extends Bundle {
  val req = Decoupled(data.cloneType)
}

// ---------------------------------------------------------------------------
// VX_operands_if
// ---------------------------------------------------------------------------

/** VX_operands_if – operand-fetch interface (master bundle). */
class OperandsIf(data: OperandsDataBundle) extends Bundle {
  val req = Decoupled(data.cloneType)
}

// ---------------------------------------------------------------------------
// VX_execute_if  (SV: #(parameter type data_t = logic))
//
// The SV interface is generic over the data type.  We mirror this with a
// Chisel type-parameter.  The caller supplies a concrete Bundle instance.
// ---------------------------------------------------------------------------

/** VX_execute_if – generic execution-unit input interface (master bundle).
 *
 *  @param gen  a concrete Data instance describing the payload type.
 *              Typical instantiation: `new ExecuteIf(new ExecuteDataBundle(...))`
 */
class ExecuteIf[T <: Data](gen: T) extends Bundle {
  val req = Decoupled(gen.cloneType)
}

// ---------------------------------------------------------------------------
// VX_result_if  (SV: #(parameter type data_t = logic))
// ---------------------------------------------------------------------------

/** VX_result_if – generic execution-unit output interface (master bundle). */
class ResultIf[T <: Data](gen: T) extends Bundle {
  val req = Decoupled(gen.cloneType)
}

// ---------------------------------------------------------------------------
// VX_writeback_if
// No ready signal – writeback is a push (valid/data only).
// ---------------------------------------------------------------------------

/** VX_writeback_if – writeback interface (master bundle, no back-pressure). */
class WritebackIf(data: WritebackDataBundle) extends Bundle {
  val bits  = data.cloneType
  val valid = Bool()
}

// ---------------------------------------------------------------------------
// VX_commit_if
// ---------------------------------------------------------------------------

/** VX_commit_if – commit interface (master bundle). */
class CommitIf(data: CommitDataBundle) extends Bundle {
  val req = Decoupled(data.cloneType)
}

// ---------------------------------------------------------------------------
// VX_schedule_if
// ---------------------------------------------------------------------------

/** VX_schedule_if – scheduler output interface (master bundle). */
class ScheduleIf(data: ScheduleDataBundle) extends Bundle {
  val req = Decoupled(data.cloneType)
}

// ===========================================================================
// Control / CSR side-channel interfaces
// These are simpler bundles (no handshake); all signals flow in one direction
// unless noted.  The master perspective is the default; Flipped() for slave.
// ===========================================================================

// ---------------------------------------------------------------------------
// VX_branch_ctl_if
// Unidirectional push from branch unit to scheduler.
// ---------------------------------------------------------------------------

/** VX_branch_ctl_if – branch resolution control (master bundle).
 *
 *  @param nwWidth NW_WIDTH
 *  @param pcBits  PC_BITS
 */
class BranchCtlBundle(val nwWidth: Int, val pcBits: Int) extends Bundle {
  val valid = Bool()
  val wid   = UInt(nwWidth.W)
  val taken = Bool()
  val dest  = UInt(pcBits.W)
}

// ---------------------------------------------------------------------------
// VX_warp_ctl_if
// dvstack_wid flows master→slave (warp-control unit → scheduler).
// dvstack_ptr flows slave→master and is NOT part of this bundle; it is
// exposed as a separate Output port on VxSchedule to avoid mixed-direction
// bundle issues.
// ---------------------------------------------------------------------------

/** VX_warp_ctl_if – warp control interface (master bundle).
 *
 *  @param nwWidth      NW_WIDTH
 *  @param dvStackSizeW DV_STACK_SIZEW
 *  @param numThreads   NUM_THREADS
 *  @param numWarps     NUM_WARPS
 *  @param nbWidth      NB_WIDTH
 *  @param pcBits       PC_BITS
 */
class WarpCtlBundle(
    val nwWidth:      Int,
    val dvStackSizeW: Int,
    val numThreads:   Int,
    val numWarps:     Int,
    val nbWidth:      Int,
    val pcBits:       Int
) extends Bundle {
  val valid       = Bool()
  val wid         = UInt(nwWidth.W)
  val tmc         = new TmcBundle(numThreads)
  val wspawn      = new WspawnBundle(numWarps, pcBits)
  val split       = new SplitBundle(numThreads, pcBits)
  val sjoin       = new JoinBundle(dvStackSizeW)
  val barrier     = new BarrierBundle(nbWidth, nwWidth)
  val dvstack_wid = UInt(nwWidth.W)
}

// ---------------------------------------------------------------------------
// VX_sched_csr_if
// Mixed-direction between scheduler and CSR unit.
// Master (scheduler) drives: cycles, active_warps, thread_masks, alm_empty.
// Slave (CSR unit) drives back: alm_empty_wid, unlock_wid, unlock_warp.
// ---------------------------------------------------------------------------

/** VX_sched_csr_if – scheduler↔CSR interface (master = scheduler bundle).
 *
 *  @param numWarps   NUM_WARPS
 *  @param numThreads NUM_THREADS
 *  @param nwWidth    NW_WIDTH
 */
class SchedCsrBundle(
    val numWarps:   Int,
    val numThreads: Int,
    val nwWidth:    Int
) extends Bundle {
  // Scheduler → CSR
  val cycles        = UInt(44.W)                            // PERF_CTR_BITS = 44
  val active_warps  = UInt(numWarps.W)
  val thread_masks  = Vec(numWarps, UInt(numThreads.W))
  val alm_empty     = Bool()
  // CSR → Scheduler (slave-driven)
  val alm_empty_wid = Flipped(UInt(nwWidth.W))
  val unlock_wid    = Flipped(UInt(nwWidth.W))
  val unlock_warp   = Flipped(Bool())
}

// ---------------------------------------------------------------------------
// VX_commit_sched_if
// Unidirectional push: commit unit → scheduler.
// ---------------------------------------------------------------------------

/** VX_commit_sched_if – commit→scheduler feedback (master = commit bundle).
 *
 *  @param numWarps NUM_WARPS
 */
class CommitSchedBundle(val numWarps: Int) extends Bundle {
  val committed_warps = UInt(numWarps.W)
}

// ---------------------------------------------------------------------------
// VX_commit_csr_if
// Unidirectional push: commit unit → CSR.
// ---------------------------------------------------------------------------

/** VX_commit_csr_if – commit→CSR counter (master = commit bundle). */
class CommitCsrBundle extends Bundle {
  val instret = UInt(44.W)   // PERF_CTR_BITS = 44
}

// ---------------------------------------------------------------------------
// VX_decode_sched_if
// Unidirectional push: decode → scheduler feedback.
// ---------------------------------------------------------------------------

/** VX_decode_sched_if – decode→scheduler feedback (master = decode bundle).
 *
 *  @param nwWidth NW_WIDTH
 */
class DecodeSchedBundle(val nwWidth: Int) extends Bundle {
  val valid  = Bool()
  val unlock = Bool()
  val wid    = UInt(nwWidth.W)
}

// ---------------------------------------------------------------------------
// VX_issue_sched_if
// Unidirectional push: issue → scheduler feedback.
// ---------------------------------------------------------------------------

/** VX_issue_sched_if – issue→scheduler feedback (master = issue bundle).
 *
 *  @param issueWisW ISSUE_WIS_W = UP(CLOG2(NUM_WARPS / ISSUE_WIDTH))
 */
class IssueSchedBundle(val issueWisW: Int) extends Bundle {
  val valid = Bool()
  val wis   = UInt(issueWisW.W)
}
