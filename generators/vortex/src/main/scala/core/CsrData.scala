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
// Translated from VX_csr_data.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// CsrData — CSR register bank and read/write logic.
//
// Mirrors VX_csr_data.sv:
//   - Maintains mscratch and (when EXT_F_ENABLE) fcsr per-warp registers.
//   - Provides a combinational read path returning read_data_ro / read_data_rw.
//   - Provides a registered write path; most system CSRs are accepted and
//     silently discarded (reset-vector, PMP, etc.).
//   - Performance-counter CSRs route into the appropriate pipeline_perf /
//     sysmem_perf sub-fields.
//
// EXT_F_ENABLE = true (assumed default).
// PERF_ENABLE  = false by default (perf inputs present but not wired when false).
// XLEN         = 32 (assumed default, no XLEN_64).
// ---------------------------------------------------------------------------
class CsrData(
  coreId:     Int     = 0,
  instanceId: String  = "csr_data",
  perfEnable: Boolean = false
) extends Module {

  // -------------------------------------------------------------------------
  // CSR read helper for 64-bit counters in XLEN=32:
  //   addr   → returns counter[31:0]
  //   addr+0x80 → returns counter[63:32]
  // -------------------------------------------------------------------------
  private def csrRead64(addr: UInt, dst: UInt, src: UInt): Unit = {
    // covered in the switch below; helper kept for documentation
  }

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Base device configuration registers
    val base_dcrs = Input(new BaseDcrsBundle)

    // Performance counter inputs (present always, used when perfEnable)
    val sysmem_perf   = Input(new SysmemPerfBundle)
    val pipeline_perf = Input(new PipelinePerfBundle)

    // Commit CSR interface (instret counter)
    val commit_instret = Input(UInt(PERF_CTR_BITS.W))

    // FPU CSR interface (EXT_F_ENABLE assumed)
    val fpu_write_enable = Input(Vec(NUM_FPU_BLOCKS, Bool()))
    val fpu_write_wid    = Input(Vec(NUM_FPU_BLOCKS, UInt(NW_WIDTH.W)))
    val fpu_write_fflags = Input(Vec(NUM_FPU_BLOCKS, UInt(5.W)))  // FP_FLAGS_BITS = 5
    // FPU reads FRM for its current warp
    val fpu_read_wid     = Input(Vec(NUM_FPU_BLOCKS, UInt(NW_WIDTH.W)))
    val fpu_read_frm     = Output(Vec(NUM_FPU_BLOCKS, UInt(INST_FRM_BITS.W)))

    // Scheduler sideband
    val cycles        = Input(UInt(PERF_CTR_BITS.W))
    val active_warps  = Input(UInt(NUM_WARPS.W))
    val thread_masks  = Input(Vec(NUM_WARPS, UInt(NUM_THREADS.W)))

    // CSR read port
    val read_enable   = Input(Bool())
    val read_uuid     = Input(UInt(UUID_WIDTH.W))
    val read_wid      = Input(UInt(NW_WIDTH.W))
    val read_addr     = Input(UInt(VX_CSR_ADDR_BITS.W))
    val read_data_ro  = Output(UInt(XLEN.W))
    val read_data_rw  = Output(UInt(XLEN.W))

    // CSR write port
    val write_enable  = Input(Bool())
    val write_uuid    = Input(UInt(UUID_WIDTH.W))
    val write_wid     = Input(UInt(NW_WIDTH.W))
    val write_addr    = Input(UInt(VX_CSR_ADDR_BITS.W))
    val write_data    = Input(UInt(XLEN.W))
  })

  // -------------------------------------------------------------------------
  // FCSR register bank: [INST_FRM_BITS + FP_FLAGS_BITS - 1 : 0] per warp
  // FP_FLAGS_BITS = $bits(fflags_t) = 5  (NV|DZ|OF|UF|NX)
  // -------------------------------------------------------------------------
  val FP_FLAGS_BITS = 5
  val FCSR_WIDTH = INST_FRM_BITS + FP_FLAGS_BITS   // 3 + 5 = 8
  val fcsr = RegInit(VecInit(Seq.fill(NUM_WARPS)(0.U(FCSR_WIDTH.W))))

  // FPU flags accumulate via OR each cycle
  val fcsrNext = Wire(Vec(NUM_WARPS, UInt(FCSR_WIDTH.W)))
  fcsrNext := fcsr  // default: hold

  // Accumulate fpu write_fflags
  for (i <- 0 until NUM_FPU_BLOCKS) {
    when (io.fpu_write_enable(i)) {
      val wid = io.fpu_write_wid(i)
      fcsrNext(wid) := Cat(
        fcsr(wid)(FCSR_WIDTH - 1, FP_FLAGS_BITS),   // preserve FRM
        fcsr(wid)(FP_FLAGS_BITS - 1, 0) | io.fpu_write_fflags(i)
      )
    }
  }

  // Explicit CSR write to fflags/frm/fcsr fields
  when (io.write_enable) {
    switch (io.write_addr) {
      is (VX_CSR_FFLAGS.U) {
        fcsrNext(io.write_wid) := Cat(
          fcsr(io.write_wid)(FCSR_WIDTH - 1, FP_FLAGS_BITS),
          io.write_data(FP_FLAGS_BITS - 1, 0)
        )
      }
      is (VX_CSR_FRM.U) {
        fcsrNext(io.write_wid) := Cat(
          io.write_data(INST_FRM_BITS - 1, 0),
          fcsr(io.write_wid)(FP_FLAGS_BITS - 1, 0)
        )
      }
      is (VX_CSR_FCSR.U) {
        fcsrNext(io.write_wid) := io.write_data(FCSR_WIDTH - 1, 0)
      }
    }
  }

  fcsr := fcsrNext

  // FPU reads rounding mode
  for (i <- 0 until NUM_FPU_BLOCKS) {
    io.fpu_read_frm(i) := fcsr(io.fpu_read_wid(i))(FCSR_WIDTH - 1, FP_FLAGS_BITS)
  }

  // -------------------------------------------------------------------------
  // mscratch register (initialised from base_dcrs.startup_arg at reset)
  // -------------------------------------------------------------------------
  val mscratch = RegInit(0.U(XLEN.W))

  // On the first cycle after reset, load startup_arg.
  // The SV code initialises to base_dcrs.startup_arg in a reset-triggered
  // always block.  We mirror that with a reset register seeded from the input.
  val resetPipe = RegNext(true.B, false.B)  // goes high one cycle after reset
  when (!resetPipe) {
    mscratch := io.base_dcrs.startup_arg
  }

  when (io.write_enable) {
    switch (io.write_addr) {
      is (VX_CSR_MSCRATCH.U) { mscratch := io.write_data }
      // All other writable-but-discarded addresses: no action needed
    }
  }

  // -------------------------------------------------------------------------
  // CSR read — combinational
  // -------------------------------------------------------------------------
  val readDataRo  = Wire(UInt(XLEN.W))
  val readDataRw  = Wire(UInt(XLEN.W))
  readDataRo := 0.U
  readDataRw := 0.U

  // Vendor / arch / implementation IDs (from VortexConfigConstants)
  val vendorId        = VENDOR_ID
  val architectureId  = ARCHITECTURE_ID
  val implementationId= IMPLEMENTATION_ID
  // MISA: MXL=1 (XLEN=32), standard extension bits from MISA_STD
  val misaStdBits = MISA_STD

  val cycles64    = Cat(0.U((64 - PERF_CTR_BITS).W), io.cycles)
  val instret64   = Cat(0.U((64 - PERF_CTR_BITS).W), io.commit_instret)

  // Helper to return low or high 32 bits depending on addr offset from base
  def sel64(base: Int, src64: UInt): (Bool, UInt) = {
    val loMatch = io.read_addr === base.U
    val hiMatch = io.read_addr === (base + 0x80).U
    val data    = Mux(hiMatch, src64(63, 32), src64(31, 0))
    (loMatch || hiMatch, data)
  }

  // ---- Main address decode ----
  switch (io.read_addr) {
    is (VX_CSR_MVENDORID.U)      { readDataRo := vendorId.U(XLEN.W) }
    is (VX_CSR_MARCHID.U)        { readDataRo := architectureId.U(XLEN.W) }
    is (VX_CSR_MIMPID.U)         { readDataRo := implementationId.U(XLEN.W) }
    // MISA: MXL={0,1}=XLEN/16 encoded, plus MISA_STD extension bits
    is (VX_CSR_MISA.U)           { readDataRo := Cat(1.U(2.W), misaStdBits.U(30.W)) }

    is (VX_CSR_FFLAGS.U)         { readDataRw := fcsr(io.read_wid)(FP_FLAGS_BITS - 1, 0) }
    is (VX_CSR_FRM.U)            { readDataRw := fcsr(io.read_wid)(FCSR_WIDTH - 1, FP_FLAGS_BITS) }
    is (VX_CSR_FCSR.U)           { readDataRw := fcsr(io.read_wid) }

    is (VX_CSR_MSCRATCH.U)       { readDataRw := mscratch }

    is (VX_CSR_WARP_ID.U)        { readDataRo := io.read_wid }
    is (VX_CSR_CORE_ID.U)        { readDataRo := coreId.U(XLEN.W) }
    is (VX_CSR_ACTIVE_THREADS.U) { readDataRo := io.thread_masks(io.read_wid) }
    is (VX_CSR_ACTIVE_WARPS.U)   { readDataRo := io.active_warps }
    is (VX_CSR_NUM_THREADS.U)    { readDataRo := NUM_THREADS.U(XLEN.W) }
    is (VX_CSR_NUM_WARPS.U)      { readDataRo := NUM_WARPS.U(XLEN.W) }
    is (VX_CSR_NUM_CORES.U)      { readDataRo := (NUM_CORES * NUM_CLUSTERS).U(XLEN.W) }
    is (VX_CSR_LOCAL_MEM_BASE.U) { readDataRo := LMEM_BASE_ADDR.U(XLEN.W) }

    // mcycle: low half
    is (VX_CSR_MCYCLE.U)         { readDataRo := cycles64(31, 0) }
    // mcycle: high half (addr + 0x80)
    is ((VX_CSR_MCYCLE + 0x80).U){ readDataRo := cycles64(63, 32) }

    // minstret: low half
    is (VX_CSR_MINSTRET.U)       { readDataRo := instret64(31, 0) }
    is ((VX_CSR_MINSTRET + 0x80).U) { readDataRo := instret64(63, 32) }

    // Zero-returning system CSRs (accepted but not implemented)
    is (VX_CSR_SATP.U)     { readDataRo := 0.U }
    is (VX_CSR_MSTATUS.U)  { readDataRo := 0.U }
    is (VX_CSR_MNSTATUS.U) { readDataRo := 0.U }
    is (VX_CSR_MEDELEG.U)  { readDataRo := 0.U }
    is (VX_CSR_MIDELEG.U)  { readDataRo := 0.U }
    is (VX_CSR_MIE.U)      { readDataRo := 0.U }
    is (VX_CSR_MTVEC.U)    { readDataRo := 0.U }
    is (VX_CSR_MEPC.U)     { readDataRo := 0.U }
    is (VX_CSR_PMPCFG0.U)  { readDataRo := 0.U }
    is (VX_CSR_PMPADDR0.U) { readDataRo := 0.U }
  }

  // ---- Performance-counter MPM user range (class 1 / class 2) ----
  if (perfEnable) {
    // Only decoded when read_addr is in [VX_CSR_MPM_USER, VX_CSR_MPM_USER+32)
    // or the corresponding high-half range.
    val inMpmRange = (io.read_addr >= VX_CSR_MPM_USER.U) &&
                     (io.read_addr < (VX_CSR_MPM_USER + 32).U) ||
                     (io.read_addr >= VX_CSR_MPM_USER_H.U) &&
                     (io.read_addr < (VX_CSR_MPM_USER_H + 32).U)

    // Class 1 (core pipeline)
    def perf64(base: Int, src: UInt): UInt = {
      val src64 = Cat(0.U((64 - PERF_CTR_BITS).W), src)
      Mux(io.read_addr === (base + 0x80).U, src64(63, 32), src64(31, 0))
    }

    val mpmRo = Wire(UInt(XLEN.W))
    mpmRo := 0.U

    when (io.base_dcrs.mpm_class === VX_DCR_MPM_CLASS_CORE.U) {
      switch (io.read_addr) {
        is (VX_CSR_MPM_SCHED_ID.U)  { mpmRo := perf64(VX_CSR_MPM_SCHED_ID, io.pipeline_perf.sched.idles) }
        is ((VX_CSR_MPM_SCHED_ID+0x80).U) { mpmRo := perf64(VX_CSR_MPM_SCHED_ID, io.pipeline_perf.sched.idles) }
        is (VX_CSR_MPM_SCHED_ST.U)  { mpmRo := perf64(VX_CSR_MPM_SCHED_ST, io.pipeline_perf.sched.stalls) }
        is ((VX_CSR_MPM_SCHED_ST+0x80).U) { mpmRo := perf64(VX_CSR_MPM_SCHED_ST, io.pipeline_perf.sched.stalls) }
        is (VX_CSR_MPM_IBUF_ST.U)   { mpmRo := perf64(VX_CSR_MPM_IBUF_ST,  io.pipeline_perf.issue.ibf_stalls) }
        is ((VX_CSR_MPM_IBUF_ST+0x80).U)  { mpmRo := perf64(VX_CSR_MPM_IBUF_ST,  io.pipeline_perf.issue.ibf_stalls) }
        is (VX_CSR_MPM_SCRB_ST.U)   { mpmRo := perf64(VX_CSR_MPM_SCRB_ST,  io.pipeline_perf.issue.scb_stalls) }
        is ((VX_CSR_MPM_SCRB_ST+0x80).U)  { mpmRo := perf64(VX_CSR_MPM_SCRB_ST,  io.pipeline_perf.issue.scb_stalls) }
        is (VX_CSR_MPM_OPDS_ST.U)   { mpmRo := perf64(VX_CSR_MPM_OPDS_ST,  io.pipeline_perf.issue.opd_stalls) }
        is ((VX_CSR_MPM_OPDS_ST+0x80).U)  { mpmRo := perf64(VX_CSR_MPM_OPDS_ST,  io.pipeline_perf.issue.opd_stalls) }
        is (VX_CSR_MPM_SCRB_ALU.U)  { mpmRo := perf64(VX_CSR_MPM_SCRB_ALU, io.pipeline_perf.issue.units_uses(EX_ALU)) }
        is ((VX_CSR_MPM_SCRB_ALU+0x80).U) { mpmRo := perf64(VX_CSR_MPM_SCRB_ALU, io.pipeline_perf.issue.units_uses(EX_ALU)) }
        is (VX_CSR_MPM_SCRB_LSU.U)  { mpmRo := perf64(VX_CSR_MPM_SCRB_LSU, io.pipeline_perf.issue.units_uses(EX_LSU)) }
        is ((VX_CSR_MPM_SCRB_LSU+0x80).U) { mpmRo := perf64(VX_CSR_MPM_SCRB_LSU, io.pipeline_perf.issue.units_uses(EX_LSU)) }
        is (VX_CSR_MPM_SCRB_SFU.U)  { mpmRo := perf64(VX_CSR_MPM_SCRB_SFU, io.pipeline_perf.issue.units_uses(EX_SFU)) }
        is ((VX_CSR_MPM_SCRB_SFU+0x80).U) { mpmRo := perf64(VX_CSR_MPM_SCRB_SFU, io.pipeline_perf.issue.units_uses(EX_SFU)) }
        is (VX_CSR_MPM_SCRB_FPU.U)  { mpmRo := perf64(VX_CSR_MPM_SCRB_FPU, io.pipeline_perf.issue.units_uses(EX_FPU)) }
        is ((VX_CSR_MPM_SCRB_FPU+0x80).U) { mpmRo := perf64(VX_CSR_MPM_SCRB_FPU, io.pipeline_perf.issue.units_uses(EX_FPU)) }
        is (VX_CSR_MPM_SCRB_CSRS.U) { mpmRo := perf64(VX_CSR_MPM_SCRB_CSRS,io.pipeline_perf.issue.sfu_uses(SFU_CSRS)) }
        is ((VX_CSR_MPM_SCRB_CSRS+0x80).U){ mpmRo := perf64(VX_CSR_MPM_SCRB_CSRS,io.pipeline_perf.issue.sfu_uses(SFU_CSRS)) }
        is (VX_CSR_MPM_SCRB_WCTL.U) { mpmRo := perf64(VX_CSR_MPM_SCRB_WCTL,io.pipeline_perf.issue.sfu_uses(SFU_WCTL)) }
        is ((VX_CSR_MPM_SCRB_WCTL+0x80).U){ mpmRo := perf64(VX_CSR_MPM_SCRB_WCTL,io.pipeline_perf.issue.sfu_uses(SFU_WCTL)) }
        is (VX_CSR_MPM_IFETCHES.U)  { mpmRo := perf64(VX_CSR_MPM_IFETCHES,  io.pipeline_perf.ifetches) }
        is ((VX_CSR_MPM_IFETCHES+0x80).U) { mpmRo := perf64(VX_CSR_MPM_IFETCHES,  io.pipeline_perf.ifetches) }
        is (VX_CSR_MPM_LOADS.U)     { mpmRo := perf64(VX_CSR_MPM_LOADS,     io.pipeline_perf.loads) }
        is ((VX_CSR_MPM_LOADS+0x80).U)    { mpmRo := perf64(VX_CSR_MPM_LOADS,     io.pipeline_perf.loads) }
        is (VX_CSR_MPM_STORES.U)    { mpmRo := perf64(VX_CSR_MPM_STORES,    io.pipeline_perf.stores) }
        is ((VX_CSR_MPM_STORES+0x80).U)   { mpmRo := perf64(VX_CSR_MPM_STORES,    io.pipeline_perf.stores) }
        is (VX_CSR_MPM_IFETCH_LT.U) { mpmRo := perf64(VX_CSR_MPM_IFETCH_LT, io.pipeline_perf.ifetch_latency) }
        is ((VX_CSR_MPM_IFETCH_LT+0x80).U){ mpmRo := perf64(VX_CSR_MPM_IFETCH_LT, io.pipeline_perf.ifetch_latency) }
        is (VX_CSR_MPM_LOAD_LT.U)   { mpmRo := perf64(VX_CSR_MPM_LOAD_LT,  io.pipeline_perf.load_latency) }
        is ((VX_CSR_MPM_LOAD_LT+0x80).U)  { mpmRo := perf64(VX_CSR_MPM_LOAD_LT,  io.pipeline_perf.load_latency) }
      }
    }.elsewhen (io.base_dcrs.mpm_class === VX_DCR_MPM_CLASS_MEM.U) {
      switch (io.read_addr) {
        is (VX_CSR_MPM_ICACHE_READS.U)    { mpmRo := perf64(VX_CSR_MPM_ICACHE_READS,   io.sysmem_perf.icache.reads) }
        is ((VX_CSR_MPM_ICACHE_READS+0x80).U){ mpmRo := perf64(VX_CSR_MPM_ICACHE_READS,   io.sysmem_perf.icache.reads) }
        is (VX_CSR_MPM_ICACHE_MISS_R.U)   { mpmRo := perf64(VX_CSR_MPM_ICACHE_MISS_R,  io.sysmem_perf.icache.read_misses) }
        is ((VX_CSR_MPM_ICACHE_MISS_R+0x80).U){ mpmRo := perf64(VX_CSR_MPM_ICACHE_MISS_R,  io.sysmem_perf.icache.read_misses) }
        is (VX_CSR_MPM_ICACHE_MSHR_ST.U)  { mpmRo := perf64(VX_CSR_MPM_ICACHE_MSHR_ST, io.sysmem_perf.icache.mshr_stalls) }
        is ((VX_CSR_MPM_ICACHE_MSHR_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_ICACHE_MSHR_ST, io.sysmem_perf.icache.mshr_stalls) }
        is (VX_CSR_MPM_DCACHE_READS.U)    { mpmRo := perf64(VX_CSR_MPM_DCACHE_READS,   io.sysmem_perf.dcache.reads) }
        is ((VX_CSR_MPM_DCACHE_READS+0x80).U){ mpmRo := perf64(VX_CSR_MPM_DCACHE_READS,   io.sysmem_perf.dcache.reads) }
        is (VX_CSR_MPM_DCACHE_WRITES.U)   { mpmRo := perf64(VX_CSR_MPM_DCACHE_WRITES,  io.sysmem_perf.dcache.writes) }
        is ((VX_CSR_MPM_DCACHE_WRITES+0x80).U){ mpmRo := perf64(VX_CSR_MPM_DCACHE_WRITES,  io.sysmem_perf.dcache.writes) }
        is (VX_CSR_MPM_DCACHE_MISS_R.U)   { mpmRo := perf64(VX_CSR_MPM_DCACHE_MISS_R,  io.sysmem_perf.dcache.read_misses) }
        is ((VX_CSR_MPM_DCACHE_MISS_R+0x80).U){ mpmRo := perf64(VX_CSR_MPM_DCACHE_MISS_R,  io.sysmem_perf.dcache.read_misses) }
        is (VX_CSR_MPM_DCACHE_MISS_W.U)   { mpmRo := perf64(VX_CSR_MPM_DCACHE_MISS_W,  io.sysmem_perf.dcache.write_misses) }
        is ((VX_CSR_MPM_DCACHE_MISS_W+0x80).U){ mpmRo := perf64(VX_CSR_MPM_DCACHE_MISS_W,  io.sysmem_perf.dcache.write_misses) }
        is (VX_CSR_MPM_DCACHE_BANK_ST.U)  { mpmRo := perf64(VX_CSR_MPM_DCACHE_BANK_ST, io.sysmem_perf.dcache.bank_stalls) }
        is ((VX_CSR_MPM_DCACHE_BANK_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_DCACHE_BANK_ST, io.sysmem_perf.dcache.bank_stalls) }
        is (VX_CSR_MPM_DCACHE_MSHR_ST.U)  { mpmRo := perf64(VX_CSR_MPM_DCACHE_MSHR_ST, io.sysmem_perf.dcache.mshr_stalls) }
        is ((VX_CSR_MPM_DCACHE_MSHR_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_DCACHE_MSHR_ST, io.sysmem_perf.dcache.mshr_stalls) }
        is (VX_CSR_MPM_LMEM_READS.U)      { mpmRo := perf64(VX_CSR_MPM_LMEM_READS,    io.sysmem_perf.lmem.reads) }
        is ((VX_CSR_MPM_LMEM_READS+0x80).U)  { mpmRo := perf64(VX_CSR_MPM_LMEM_READS,    io.sysmem_perf.lmem.reads) }
        is (VX_CSR_MPM_LMEM_WRITES.U)     { mpmRo := perf64(VX_CSR_MPM_LMEM_WRITES,   io.sysmem_perf.lmem.writes) }
        is ((VX_CSR_MPM_LMEM_WRITES+0x80).U) { mpmRo := perf64(VX_CSR_MPM_LMEM_WRITES,   io.sysmem_perf.lmem.writes) }
        is (VX_CSR_MPM_LMEM_BANK_ST.U)    { mpmRo := perf64(VX_CSR_MPM_LMEM_BANK_ST, io.sysmem_perf.lmem.bank_stalls) }
        is ((VX_CSR_MPM_LMEM_BANK_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_LMEM_BANK_ST, io.sysmem_perf.lmem.bank_stalls) }
        is (VX_CSR_MPM_L2CACHE_READS.U)   { mpmRo := perf64(VX_CSR_MPM_L2CACHE_READS,  io.sysmem_perf.l2cache.reads) }
        is ((VX_CSR_MPM_L2CACHE_READS+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L2CACHE_READS,  io.sysmem_perf.l2cache.reads) }
        is (VX_CSR_MPM_L2CACHE_WRITES.U)  { mpmRo := perf64(VX_CSR_MPM_L2CACHE_WRITES, io.sysmem_perf.l2cache.writes) }
        is ((VX_CSR_MPM_L2CACHE_WRITES+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L2CACHE_WRITES, io.sysmem_perf.l2cache.writes) }
        is (VX_CSR_MPM_L2CACHE_MISS_R.U)  { mpmRo := perf64(VX_CSR_MPM_L2CACHE_MISS_R, io.sysmem_perf.l2cache.read_misses) }
        is ((VX_CSR_MPM_L2CACHE_MISS_R+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L2CACHE_MISS_R, io.sysmem_perf.l2cache.read_misses) }
        is (VX_CSR_MPM_L2CACHE_MISS_W.U)  { mpmRo := perf64(VX_CSR_MPM_L2CACHE_MISS_W, io.sysmem_perf.l2cache.write_misses) }
        is ((VX_CSR_MPM_L2CACHE_MISS_W+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L2CACHE_MISS_W, io.sysmem_perf.l2cache.write_misses) }
        is (VX_CSR_MPM_L2CACHE_BANK_ST.U) { mpmRo := perf64(VX_CSR_MPM_L2CACHE_BANK_ST,io.sysmem_perf.l2cache.bank_stalls) }
        is ((VX_CSR_MPM_L2CACHE_BANK_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L2CACHE_BANK_ST,io.sysmem_perf.l2cache.bank_stalls) }
        is (VX_CSR_MPM_L2CACHE_MSHR_ST.U) { mpmRo := perf64(VX_CSR_MPM_L2CACHE_MSHR_ST,io.sysmem_perf.l2cache.mshr_stalls) }
        is ((VX_CSR_MPM_L2CACHE_MSHR_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L2CACHE_MSHR_ST,io.sysmem_perf.l2cache.mshr_stalls) }
        is (VX_CSR_MPM_L3CACHE_READS.U)   { mpmRo := perf64(VX_CSR_MPM_L3CACHE_READS,  io.sysmem_perf.l3cache.reads) }
        is ((VX_CSR_MPM_L3CACHE_READS+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L3CACHE_READS,  io.sysmem_perf.l3cache.reads) }
        is (VX_CSR_MPM_L3CACHE_WRITES.U)  { mpmRo := perf64(VX_CSR_MPM_L3CACHE_WRITES, io.sysmem_perf.l3cache.writes) }
        is ((VX_CSR_MPM_L3CACHE_WRITES+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L3CACHE_WRITES, io.sysmem_perf.l3cache.writes) }
        is (VX_CSR_MPM_L3CACHE_MISS_R.U)  { mpmRo := perf64(VX_CSR_MPM_L3CACHE_MISS_R, io.sysmem_perf.l3cache.read_misses) }
        is ((VX_CSR_MPM_L3CACHE_MISS_R+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L3CACHE_MISS_R, io.sysmem_perf.l3cache.read_misses) }
        is (VX_CSR_MPM_L3CACHE_MISS_W.U)  { mpmRo := perf64(VX_CSR_MPM_L3CACHE_MISS_W, io.sysmem_perf.l3cache.write_misses) }
        is ((VX_CSR_MPM_L3CACHE_MISS_W+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L3CACHE_MISS_W, io.sysmem_perf.l3cache.write_misses) }
        is (VX_CSR_MPM_L3CACHE_BANK_ST.U) { mpmRo := perf64(VX_CSR_MPM_L3CACHE_BANK_ST,io.sysmem_perf.l3cache.bank_stalls) }
        is ((VX_CSR_MPM_L3CACHE_BANK_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L3CACHE_BANK_ST,io.sysmem_perf.l3cache.bank_stalls) }
        is (VX_CSR_MPM_L3CACHE_MSHR_ST.U) { mpmRo := perf64(VX_CSR_MPM_L3CACHE_MSHR_ST,io.sysmem_perf.l3cache.mshr_stalls) }
        is ((VX_CSR_MPM_L3CACHE_MSHR_ST+0x80).U){ mpmRo := perf64(VX_CSR_MPM_L3CACHE_MSHR_ST,io.sysmem_perf.l3cache.mshr_stalls) }
        is (VX_CSR_MPM_MEM_READS.U)       { mpmRo := perf64(VX_CSR_MPM_MEM_READS,     io.sysmem_perf.mem.reads) }
        is ((VX_CSR_MPM_MEM_READS+0x80).U)   { mpmRo := perf64(VX_CSR_MPM_MEM_READS,     io.sysmem_perf.mem.reads) }
        is (VX_CSR_MPM_MEM_WRITES.U)      { mpmRo := perf64(VX_CSR_MPM_MEM_WRITES,    io.sysmem_perf.mem.writes) }
        is ((VX_CSR_MPM_MEM_WRITES+0x80).U)  { mpmRo := perf64(VX_CSR_MPM_MEM_WRITES,    io.sysmem_perf.mem.writes) }
        is (VX_CSR_MPM_MEM_LT.U)          { mpmRo := perf64(VX_CSR_MPM_MEM_LT,       io.sysmem_perf.mem.latency) }
        is ((VX_CSR_MPM_MEM_LT+0x80).U)      { mpmRo := perf64(VX_CSR_MPM_MEM_LT,       io.sysmem_perf.mem.latency) }
        is (VX_CSR_MPM_COALESCER_MISS.U)  { mpmRo := perf64(VX_CSR_MPM_COALESCER_MISS,io.sysmem_perf.coalescer.misses) }
        is ((VX_CSR_MPM_COALESCER_MISS+0x80).U){ mpmRo := perf64(VX_CSR_MPM_COALESCER_MISS,io.sysmem_perf.coalescer.misses) }
      }
    }

    when (inMpmRange) {
      readDataRo := mpmRo
    }
  }

  io.read_data_ro := readDataRo
  io.read_data_rw := readDataRw
}
