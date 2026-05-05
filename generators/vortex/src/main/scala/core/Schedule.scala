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
// Translated from VX_schedule.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** VX_schedule – warp scheduler.
 *
 *  Tracks per-warp state (active, stalled, thread masks, PCs), selects the
 *  next ready warp via a priority encoder, and emits schedule packets.
 *  Also handles branch resolution, warp spawn, TMC, split/join, and
 *  barrier synchronisation.
 *
 *  Mirrors VX_schedule.sv.
 *
 *  @param coreId  CORE_ID parameter (passed to VxUuidGen)
 */
class VxSchedule(val coreId: Int = 0) extends Module {

  val io = IO(new Bundle {
    // configuration
    val startup_addr = Input(UInt(XLEN.W))

    // inputs
    val warp_ctl     = Input(new WarpCtlBundle(
                         nwWidth      = NW_WIDTH,
                         dvStackSizeW = DV_STACK_SIZEW,
                         numThreads   = NUM_THREADS,
                         numWarps     = NUM_WARPS,
                         nbWidth      = NB_WIDTH,
                         pcBits       = PC_BITS))
    val branch_ctl   = Input(Vec(NUM_ALU_BLOCKS,
                         new BranchCtlBundle(nwWidth = NW_WIDTH, pcBits = PC_BITS)))
    val decode_sched = Input(new DecodeSchedBundle(nwWidth = NW_WIDTH))
    val issue_sched  = Input(Vec(ISSUE_WIDTH, new IssueSchedBundle(issueWisW = ISSUE_WIS_W)))
    val commit_sched = Input(new CommitSchedBundle(numWarps = NUM_WARPS))

    // schedule output (valid/ready handshake)
    val schedule_valid = Output(Bool())
    val schedule_ready = Input(Bool())
    val schedule_data  = Output(new ScheduleBundle)

    // CSR interface (mixed-direction; Flipped fields flow inward from CSR unit)
    val sched_csr    = new SchedCsrBundle(
                         numWarps   = NUM_WARPS,
                         numThreads = NUM_THREADS,
                         nwWidth    = NW_WIDTH)

    // dvstack_ptr return path (slave→master; separate from warp_ctl bundle)
    val warp_ctl_dvstack_ptr = Output(UInt(DV_STACK_SIZEW.W))

    // busy status
    val busy         = Output(Bool())

    // performance counters
    val perf_idles   = Output(UInt(PERF_CTR_BITS.W))
    val perf_stalls  = Output(UInt(PERF_CTR_BITS.W))
  })

  // -------------------------------------------------------------------------
  // Warp state registers
  // -------------------------------------------------------------------------

  // warp 0 active and ready at startup; all others inactive
  val active_warps  = RegInit(1.U(NUM_WARPS.W))
  val stalled_warps = RegInit(0.U(NUM_WARPS.W))

  // Per-warp thread masks: thread 0 of warp 0 active initially
  val thread_masks = RegInit(VecInit(Seq.tabulate(NUM_WARPS)(i =>
    if (i == 0) 1.U(NUM_THREADS.W) else 0.U(NUM_THREADS.W)
  )))

  // Only warp_pc(0) is initialized to io.startup_addr
  val initValues = Seq(io.startup_addr) ++ Seq.fill(NUM_WARPS - 1)(0.U(PC_BITS.W))
  val warp_pcs = RegInit(VecInit(initValues))

  // -------------------------------------------------------------------------
  // Cycles counter (counts while busy)
  // -------------------------------------------------------------------------

  val cycles = RegInit(0.U(PERF_CTR_BITS.W))

  // -------------------------------------------------------------------------
  // Split/Join unit
  // -------------------------------------------------------------------------

  val splitJoin = Module(new VxSplitJoin(outReg = 1))

  splitJoin.io.valid            := io.warp_ctl.valid
  splitJoin.io.wid              := io.warp_ctl.wid
  splitJoin.io.split.valid      := io.warp_ctl.split.valid
  splitJoin.io.split.is_dvg     := io.warp_ctl.split.is_dvg
  splitJoin.io.split.then_tmask := io.warp_ctl.split.then_tmask
  splitJoin.io.split.else_tmask := io.warp_ctl.split.else_tmask
  splitJoin.io.split.next_pc    := io.warp_ctl.split.next_pc
  splitJoin.io.sjoin.valid      := io.warp_ctl.sjoin.valid
  splitJoin.io.sjoin.stack_ptr  := io.warp_ctl.sjoin.stack_ptr
  splitJoin.io.stackWid         := io.warp_ctl.dvstack_wid

  // dvstack_ptr flows back slave → master via separate Output port
  io.warp_ctl_dvstack_ptr := splitJoin.io.stackPtr

  val join_valid   = splitJoin.io.joinValid
  val join_is_dvg  = splitJoin.io.joinIsDvg
  val join_is_else = splitJoin.io.joinIsElse
  val join_wid     = splitJoin.io.joinWid
  val join_tmask   = splitJoin.io.joinTmask
  val join_pc      = splitJoin.io.joinPc

  // -------------------------------------------------------------------------
  // Priority encoder: selects next ready warp (lowest-index / LSB first)
  // -------------------------------------------------------------------------

  val ready_warps = active_warps & ~stalled_warps

  val widSelect = Module(new VxPriorityEncoder(n = NUM_WARPS))
  widSelect.io.dataIn := ready_warps

  val schedule_wid_w   = widSelect.io.indexOut    // combinational
  val schedule_valid_w = widSelect.io.validOut    // combinational
  val schedule_ready_w = Wire(Bool())             // back-pressure from elastic buffer

  val schedule_fire    = schedule_valid_w && schedule_ready_w
  val schedule_if_fire = io.schedule_valid && io.schedule_ready

  // Mux per-warp tmask/PC using schedule_wid
  val schedule_tmask = thread_masks(schedule_wid_w)
  val schedule_pc    = warp_pcs(schedule_wid_w)

  // -------------------------------------------------------------------------
  // UUID generator
  // -------------------------------------------------------------------------

  val uuidGen = Module(new VxUuidGen(coreId = coreId))
  uuidGen.io.incr := schedule_fire
  uuidGen.io.wid  := schedule_wid_w
  val instr_uuid  = uuidGen.io.uuid

  // -------------------------------------------------------------------------
  // Output queue (2-entry FIFO, registered output)
  // Packed as: {schedule_tmask, schedule_pc, schedule_wid, instr_uuid}
  // -------------------------------------------------------------------------

  val outBufW = NUM_THREADS + PC_BITS + NW_WIDTH + UUID_WIDTH

  val enq = Wire(Decoupled(UInt(outBufW.W)))
  enq.valid        := schedule_valid_w
  enq.bits         := Cat(schedule_tmask, schedule_pc, schedule_wid_w, instr_uuid)
  schedule_ready_w := enq.ready

  val deq = Queue(enq, 2)
  deq.ready         := io.schedule_ready
  io.schedule_valid := deq.valid

  val outData = deq.bits
  io.schedule_data.uuid  := outData(UUID_WIDTH - 1, 0)
  io.schedule_data.wid   := outData(UUID_WIDTH + NW_WIDTH - 1, UUID_WIDTH)
  io.schedule_data.PC    := outData(UUID_WIDTH + NW_WIDTH + PC_BITS - 1,
                                     UUID_WIDTH + NW_WIDTH)
  io.schedule_data.tmask := outData(UUID_WIDTH + NW_WIDTH + PC_BITS + NUM_THREADS - 1,
                                     UUID_WIDTH + NW_WIDTH + PC_BITS)

  // -------------------------------------------------------------------------
  // Barrier state
  // -------------------------------------------------------------------------

  val barrier_masks  = RegInit(VecInit(Seq.fill(NUM_BARRIERS)(0.U(NUM_WARPS.W))))
  val barrier_ctrs   = RegInit(VecInit(Seq.fill(NUM_BARRIERS)(0.U(NW_WIDTH.W))))

  // -------------------------------------------------------------------------
  // Wspawn state
  // -------------------------------------------------------------------------

  val wspawn_valid = RegInit(false.B)
  val wspawn_wmask = Reg(UInt(NUM_WARPS.W))
  val wspawn_pc    = Reg(UInt(PC_BITS.W))
  val wspawn_wid   = Reg(UInt(NW_WIDTH.W))

  // -------------------------------------------------------------------------
  // Active-warps population count (for is_single_warp check)
  // -------------------------------------------------------------------------

  val active_warps_cnt = PopCount(active_warps)
  val is_single_warp   = RegInit(true.B)

  // -------------------------------------------------------------------------
  // Combinational next-state (mirrors SV always @(*) with blocking assigns).
  //
  // The SV uses blocking assignments that accumulate bit-level changes to
  // stalled_warps_n etc.  In Chisel, Wire last-connect semantics prevent
  // self-reads.  We therefore build up the combinational result using a
  // series of Scala intermediate UInt values (immutable combinational chain)
  // that are threaded through each update stage.
  // -------------------------------------------------------------------------

  // curr_barrier_mask_p1: barrier mask with current warp's bit set
  val barrier_id_sel       = io.warp_ctl.barrier.id
  val curr_barrier_mask_p1 = barrier_masks(barrier_id_sel) | (1.U << io.warp_ctl.wid)

  // ---- Thread masks: per-warp, updated by TMC / split / join / wspawn ----
  val thread_masks_n = Wire(Vec(NUM_WARPS, UInt(NUM_THREADS.W)))
  for (i <- 0 until NUM_WARPS) thread_masks_n(i) := thread_masks(i)

  when (io.warp_ctl.valid && io.warp_ctl.tmc.valid) {
    thread_masks_n(io.warp_ctl.wid) := io.warp_ctl.tmc.tmask
  }
  when (io.warp_ctl.valid && io.warp_ctl.split.valid && io.warp_ctl.split.is_dvg) {
    thread_masks_n(io.warp_ctl.wid) := io.warp_ctl.split.then_tmask
  }
  when (join_valid && join_is_dvg) {
    thread_masks_n(join_wid) := join_tmask
  }
  for (i <- 0 until NUM_WARPS) {
    when (wspawn_valid && is_single_warp && wspawn_wmask(i)) {
      thread_masks_n(i) := 1.U   // thread 0 only
    }
  }

  // ---- Warp PCs: updated by branch / join / wspawn / schedule_if_fire ----
  val warp_pcs_n = Wire(Vec(NUM_WARPS, UInt(PC_BITS.W)))
  for (i <- 0 until NUM_WARPS) warp_pcs_n(i) := warp_pcs(i)

  when (join_valid && join_is_dvg && join_is_else) {
    warp_pcs_n(join_wid) := join_pc
  }
  for (i <- 0 until NUM_WARPS) {
    when (wspawn_valid && is_single_warp && wspawn_wmask(i)) {
      warp_pcs_n(i) := wspawn_pc
    }
  }
  for (i <- 0 until NUM_ALU_BLOCKS) {
    when (io.branch_ctl(i).valid && io.branch_ctl(i).taken) {
      warp_pcs_n(io.branch_ctl(i).wid) := io.branch_ctl(i).dest
    }
  }
  when (schedule_if_fire) {
    warp_pcs_n(io.schedule_data.wid) := io.schedule_data.PC + 4.U
  }

  // ---- Barrier masks & counters ----
  val barrier_masks_n = Wire(Vec(NUM_BARRIERS, UInt(NUM_WARPS.W)))
  val barrier_ctrs_n  = Wire(Vec(NUM_BARRIERS, UInt(NW_WIDTH.W)))
  for (i <- 0 until NUM_BARRIERS) barrier_masks_n(i) := barrier_masks(i)
  for (i <- 0 until NUM_BARRIERS) barrier_ctrs_n(i)  := barrier_ctrs(i)

  when (io.warp_ctl.valid && io.warp_ctl.barrier.valid && !io.warp_ctl.barrier.is_noop) {
    val bid = io.warp_ctl.barrier.id
    when (!io.warp_ctl.barrier.is_global &&
          barrier_ctrs(bid) === io.warp_ctl.barrier.size_m1) {
      barrier_ctrs_n(bid)  := 0.U
      barrier_masks_n(bid) := 0.U
    } .otherwise {
      barrier_ctrs_n(bid)  := barrier_ctrs(bid) + 1.U
      barrier_masks_n(bid) := curr_barrier_mask_p1
    }
  }

  // ---- Active warps: updated by TMC / wspawn ----
  // Build as accumulation over a Scala chain (no self-reads on Wire).
  val active_warps_tmc = Wire(UInt(NUM_WARPS.W))
  active_warps_tmc := active_warps
  when (io.warp_ctl.valid && io.warp_ctl.tmc.valid) {
    val wid = io.warp_ctl.wid
    active_warps_tmc := Mux(
      io.warp_ctl.tmc.tmask =/= 0.U,
      active_warps | (1.U << wid),
      active_warps & ~(1.U << wid)
    )
  }
  val active_warps_n = Wire(UInt(NUM_WARPS.W))
  when (wspawn_valid && is_single_warp) {
    active_warps_n := active_warps_tmc | wspawn_wmask
  } .otherwise {
    active_warps_n := active_warps_tmc
  }

  // ---- Stalled warps: multi-source bit manipulation ----
  //
  // The SV accumulates these updates (bit-level):
  //   1. Decode unlock:  clear bit[decode_wid]
  //   2. CSR unlock:     clear bit[csr_unlock_wid]
  //   3. Wspawn unlock:  clear bit[wspawn_wid]  (when wspawn fires)
  //   4. TMC unlock:     clear bit[warp_ctl.wid]
  //   5. Split unlock:   clear bit[warp_ctl.wid]
  //   6. Join unlock:    clear bit[join_wid]
  //   7. Barrier unlock: clear multiple bits (or just current wid for noop)
  //   8. Branch unlock:  clear bit[branch_wid]  (for each ALU block)
  //   9. Schedule stall: set   bit[schedule_wid]
  //
  // We build a set-clear bitmask: clear_mask OR-aggregates all clear events,
  // set_mask OR-aggregates all set events.  Then:
  //   stalled_warps_n = (stalled_warps & ~clear_mask) | set_mask
  //
  // This faithfully models the SV priority: since the SV performs all
  // updates in order, and some events can set+clear the same bit (e.g.
  // branch unlock then schedule stall for the same warp), the last
  // assignment wins.  We replicate that by applying set last.

  val stall_clear = Wire(UInt(NUM_WARPS.W))
  val stall_set   = Wire(UInt(NUM_WARPS.W))

  // Accumulate clear bits
  var sc = 0.U(NUM_WARPS.W)

  // (1) Decode unlock
  sc = sc | Mux(io.decode_sched.valid && io.decode_sched.unlock,
                1.U << io.decode_sched.wid, 0.U(NUM_WARPS.W))

  // (2) CSR unlock
  sc = sc | Mux(io.sched_csr.unlock_warp,
                1.U << io.sched_csr.unlock_wid, 0.U(NUM_WARPS.W))

  // (3) Wspawn unlock (spawning warp)
  sc = sc | Mux(wspawn_valid && is_single_warp,
                1.U << wspawn_wid, 0.U(NUM_WARPS.W))

  // (4) TMC unlock
  sc = sc | Mux(io.warp_ctl.valid && io.warp_ctl.tmc.valid,
                1.U << io.warp_ctl.wid, 0.U(NUM_WARPS.W))

  // (5) Split unlock
  sc = sc | Mux(io.warp_ctl.valid && io.warp_ctl.split.valid,
                1.U << io.warp_ctl.wid, 0.U(NUM_WARPS.W))

  // (6) Join unlock
  sc = sc | Mux(join_valid, 1.U << join_wid, 0.U(NUM_WARPS.W))

  // (7a) Barrier noop unlock
  sc = sc | Mux(io.warp_ctl.valid && io.warp_ctl.barrier.valid &&
                io.warp_ctl.barrier.is_noop,
                1.U << io.warp_ctl.wid, 0.U(NUM_WARPS.W))

  // (7b) Barrier all-arrived unlock: release barrier_masks[bid] + current warp
  val barrierAllArrived = io.warp_ctl.valid && io.warp_ctl.barrier.valid &&
                          !io.warp_ctl.barrier.is_noop &&
                          !io.warp_ctl.barrier.is_global &&
                          (barrier_ctrs(io.warp_ctl.barrier.id) === io.warp_ctl.barrier.size_m1)
  sc = sc | Mux(barrierAllArrived,
                barrier_masks(io.warp_ctl.barrier.id) | (1.U << io.warp_ctl.wid),
                0.U(NUM_WARPS.W))

  // (8) Branch unlock (per ALU block)
  for (i <- 0 until NUM_ALU_BLOCKS) {
    sc = sc | Mux(io.branch_ctl(i).valid,
                  1.U << io.branch_ctl(i).wid, 0.U(NUM_WARPS.W))
  }

  stall_clear := sc

  // Accumulate set bits: only the schedule stall
  stall_set := Mux(schedule_fire, 1.U << schedule_wid_w, 0.U(NUM_WARPS.W))

  // Final next-state: apply clears then sets (sets have priority as in SV)
  val stalled_warps_n = (stalled_warps & ~stall_clear) | stall_set

  // -------------------------------------------------------------------------
  // Sequential update
  // -------------------------------------------------------------------------

  active_warps  := active_warps_n
  stalled_warps := stalled_warps_n
  for (i <- 0 until NUM_WARPS)    thread_masks(i) := thread_masks_n(i)
  for (i <- 0 until NUM_WARPS)    warp_pcs(i)     := warp_pcs_n(i)
  for (i <- 0 until NUM_BARRIERS) barrier_masks(i) := barrier_masks_n(i)
  for (i <- 0 until NUM_BARRIERS) barrier_ctrs(i)  := barrier_ctrs_n(i)

  is_single_warp := (active_warps_cnt === 1.U)

  // Wspawn latch
  when (io.warp_ctl.valid && io.warp_ctl.wspawn.valid) {
    wspawn_valid := true.B
    wspawn_wmask := io.warp_ctl.wspawn.wmask
    wspawn_pc    := io.warp_ctl.wspawn.pc
    wspawn_wid   := io.warp_ctl.wid
  }
  when (wspawn_valid && is_single_warp) {
    wspawn_valid := false.B
  }

  // Cycle counter
  when (io.busy) {
    cycles := cycles + 1.U
  }

  // -------------------------------------------------------------------------
  // Pending instructions per warp
  // VX_pending_size #(.SIZE(4096), .ALM_EMPTY(1)) per warp
  // -------------------------------------------------------------------------

  // Compile-time wid → (isw, wis) helpers matching SV localparam logic
  def widToIswInt(wid: Int): Int =
    if (ISSUE_ISW_BITS != 0) wid & ((1 << ISSUE_ISW_W) - 1) else 0
  def widToWisInt(wid: Int): Int =
    if (ISSUE_WIS_BITS != 0) wid >> ISSUE_ISW_BITS else 0

  val pending_warp_empty     = Wire(Vec(NUM_WARPS, Bool()))
  val pending_warp_alm_empty = Wire(Vec(NUM_WARPS, Bool()))

  for (i <- 0 until NUM_WARPS) {
    val isw = widToIswInt(i)
    val wis = widToWisInt(i)

    val ps = Module(new VxPendingSize(size = 4096, almEmpty = 1))

    val issueValid = io.issue_sched(isw).valid
    val issueMatch: Bool =
      if (ISSUE_WIS_BITS != 0) io.issue_sched(isw).wis === wis.U(ISSUE_WIS_W.W)
      else true.B
    ps.io.incr := (issueValid && issueMatch).asUInt

    ps.io.decr := io.commit_sched.committed_warps(i).asUInt

    pending_warp_empty(i)     := ps.io.empty
    pending_warp_alm_empty(i) := ps.io.almEmpty
  }

  // CSR alm_empty mux
  io.sched_csr.alm_empty := pending_warp_alm_empty(io.sched_csr.alm_empty_wid)

  val no_pending_instr = pending_warp_empty.asUInt.andR

  // busy: 1-cycle registered delay (matches BUFFER_EX(1,1) macro)
  val busy_comb = (active_warps =/= 0.U) || !no_pending_instr
  io.busy := RegNext(busy_comb, false.B)

  // -------------------------------------------------------------------------
  // CSR exports
  // -------------------------------------------------------------------------

  io.sched_csr.cycles       := cycles
  io.sched_csr.active_warps := active_warps
  for (i <- 0 until NUM_WARPS) io.sched_csr.thread_masks(i) := thread_masks(i)

  // -------------------------------------------------------------------------
  // Performance counters
  // -------------------------------------------------------------------------

  val perf_idles_r  = RegInit(0.U(PERF_CTR_BITS.W))
  val perf_stalls_r = RegInit(0.U(PERF_CTR_BITS.W))

  perf_idles_r  := perf_idles_r  + (!schedule_valid_w).asUInt
  perf_stalls_r := perf_stalls_r + (io.schedule_valid && !io.schedule_ready).asUInt

  io.perf_idles  := perf_idles_r
  io.perf_stalls := perf_stalls_r
}
