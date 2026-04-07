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
// Translated from VX_csr_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// CsrUnit — CSR read/write and warp-unlock logic.
//
// Mirrors VX_csr_unit.sv:
//   - Stalls on FPU CSR accesses until all pending instructions for the
//     current warp have completed (via sched_csr alm_empty).
//   - Reads CSR data (RO or RW) from a CsrData sub-module.
//   - Computes the write-back value based on CSRRW / CSRRS / CSRRC op.
//   - Writes the result back to the register file via a 2-entry elastic
//     buffer → result_if output.
//   - Signals the scheduler to unlock the warp after FPU CSR access.
//
// Parameters:
//   coreId   – physical core identifier (CORE_ID in SV)
//   numLanes – number of SIMD lanes for this SFU block
// ---------------------------------------------------------------------------
class CsrUnit(
  coreId:     Int     = 0,
  numLanes:   Int     = NUM_SFU_LANES,
  instanceId: String  = "csr_unit",
  perfEnable: Boolean = false
) extends Module {

  private val PID_BITS  = log2Ceil(NUM_THREADS / numLanes)
  private val PID_WIDTH = up(PID_BITS)
  // DATAW for the elastic buffer
  private val DATAW =
    UUID_WIDTH + NW_WIDTH + numLanes + PC_BITS + NUM_REGS_BITS + 1 + (numLanes * XLEN) + PID_WIDTH + 1 + 1

  val NT_BITS_LOCAL = up(log2Ceil(NUM_THREADS))   // used for gtid calculation

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Perf inputs (present always, used when perfEnable=true)
    val sysmem_perf   = Input(new SysmemPerfBundle)
    val pipeline_perf = Input(new PipelinePerfBundle)

    // MPM class — direct from RoCC (same pattern as startup_addr)
    val mpm_class = Input(UInt(8.W))

    // FPU CSR interface (EXT_F_ENABLE assumed)
    val fpu_write_enable = Input(Vec(NUM_FPU_BLOCKS, Bool()))
    val fpu_write_wid    = Input(Vec(NUM_FPU_BLOCKS, UInt(NW_WIDTH.W)))
    val fpu_write_fflags = Input(Vec(NUM_FPU_BLOCKS, UInt(5.W)))  // FP_FLAGS_BITS=5
    val fpu_read_wid     = Input(Vec(NUM_FPU_BLOCKS, UInt(NW_WIDTH.W)))
    val fpu_read_frm     = Output(Vec(NUM_FPU_BLOCKS, UInt(INST_FRM_BITS.W)))

    // Commit CSR (instret counter)
    val commit_instret = Input(UInt(PERF_CTR_BITS.W))

    // Scheduler CSR sideband
    val cycles       = Input(UInt(PERF_CTR_BITS.W))
    val active_warps = Input(UInt(NUM_WARPS.W))
    val thread_masks = Input(Vec(NUM_WARPS, UInt(NUM_THREADS.W)))
    val alm_empty    = Input(Bool())
    // CSR unit drives wid back to scheduler for almEmpty query
    val alm_empty_wid = Output(UInt(NW_WIDTH.W))
    // Warp unlock after FPU CSR access
    val unlock_warp   = Output(Bool())
    val unlock_wid    = Output(UInt(NW_WIDTH.W))

    // Execute-stage input (slave)
    val execute_valid  = Input(Bool())
    val execute_ready  = Output(Bool())
    val ex_uuid        = Input(UInt(UUID_WIDTH.W))
    val ex_wid         = Input(UInt(NW_WIDTH.W))
    val ex_tmask       = Input(UInt(numLanes.W))
    val ex_PC          = Input(UInt(PC_BITS.W))
    val ex_op_type     = Input(UInt(INST_OP_BITS.W))
    val ex_wb          = Input(Bool())
    val ex_rd          = Input(UInt(NUM_REGS_BITS.W))
    val ex_rs1_data    = Input(Vec(numLanes, UInt(XLEN.W)))
    // op_args.csr fields
    val ex_csr_addr    = Input(UInt(VX_CSR_ADDR_BITS.W))
    val ex_csr_imm     = Input(UInt(5.W))    // RV_REGS_BITS = 5
    val ex_csr_use_imm = Input(Bool())
    val ex_pid         = Input(UInt(PID_WIDTH.W))
    val ex_sop         = Input(Bool())
    val ex_eop         = Input(Bool())

    // Result output (master)
    val result_valid  = Output(Bool())
    val result_ready  = Input(Bool())
    val res_uuid      = Output(UInt(UUID_WIDTH.W))
    val res_wid       = Output(UInt(NW_WIDTH.W))
    val res_tmask     = Output(UInt(numLanes.W))
    val res_PC        = Output(UInt(PC_BITS.W))
    val res_wb        = Output(Bool())
    val res_rd        = Output(UInt(NUM_REGS_BITS.W))
    val res_data      = Output(Vec(numLanes, UInt(XLEN.W)))
    val res_pid       = Output(UInt(PID_WIDTH.W))
    val res_sop       = Output(Bool())
    val res_eop       = Output(Bool())
  })

  // -------------------------------------------------------------------------
  // FPU CSR range check:  addr <= VX_CSR_FCSR (0x003)
  // -------------------------------------------------------------------------
  val isFpuCsr = io.ex_csr_addr <= VX_CSR_FCSR.U

  // -------------------------------------------------------------------------
  // Stall until all pending instructions for this warp complete (FPU CSR only)
  // -------------------------------------------------------------------------
  io.alm_empty_wid := io.ex_wid
  val noPendingInstr = io.alm_empty || !isFpuCsr

  val csrReqValid = io.execute_valid && noPendingInstr

  // -------------------------------------------------------------------------
  // CsrData sub-module
  // -------------------------------------------------------------------------
  val csrData = Module(new CsrData(
    coreId    = coreId,
    instanceId= s"${instanceId}-csr_data",
    perfEnable= perfEnable
  ))

  csrData.io.sysmem_perf    := io.sysmem_perf
  csrData.io.pipeline_perf  := io.pipeline_perf
  csrData.io.commit_instret := io.commit_instret
  csrData.io.mpm_class      := io.mpm_class
  csrData.io.cycles         := io.cycles
  csrData.io.active_warps   := io.active_warps
  csrData.io.thread_masks   := io.thread_masks

  csrData.io.fpu_write_enable := io.fpu_write_enable
  csrData.io.fpu_write_wid    := io.fpu_write_wid
  csrData.io.fpu_write_fflags := io.fpu_write_fflags
  csrData.io.fpu_read_wid     := io.fpu_read_wid
  io.fpu_read_frm             := csrData.io.fpu_read_frm

  // -------------------------------------------------------------------------
  // CSR read enable: only when we actually want to read (not thread/hartid)
  // The CSR data module handles the thread_id / mhartid special cases inline.
  // -------------------------------------------------------------------------
  val csrRdEnable = Wire(Bool())
  csrData.io.read_enable := csrReqValid && csrRdEnable
  csrData.io.read_uuid   := io.ex_uuid
  csrData.io.read_wid    := io.ex_wid
  csrData.io.read_addr   := io.ex_csr_addr

  // -------------------------------------------------------------------------
  // Thread-ID and hart-ID are computed locally (not in CsrData)
  // -------------------------------------------------------------------------
  val wtid = VecInit(Seq.tabulate(numLanes) { i =>
    if (PID_BITS != 0)
      (io.ex_pid * numLanes.U + i.U)(XLEN - 1, 0)
    else
      i.U(XLEN.W)
  })

  val gtid = VecInit(Seq.tabulate(numLanes) { i =>
    (coreId.U(XLEN.W) << (NW_BITS + NT_BITS_LOCAL)) |
    (io.ex_wid << NT_BITS_LOCAL.U) |
    wtid(i)
  })

  // Combine RO and RW read data
  val csrReadDataRo = csrData.io.read_data_ro
  val csrReadDataRw = csrData.io.read_data_rw

  // CSR read result selection:
  //   VX_CSR_THREAD_ID → per-lane warp-thread id
  //   VX_CSR_MHARTID   → per-lane global hart id
  //   default          → broadcast (ro | rw) to all lanes; assert csrRdEnable
  val isThreadId = io.ex_csr_addr === VX_CSR_THREAD_ID.U
  val isMhartId  = io.ex_csr_addr === VX_CSR_MHARTID.U
  csrRdEnable := !isThreadId && !isMhartId

  val broadcastVal = VecInit(Seq.fill(numLanes)(csrReadDataRo | csrReadDataRw))

  val csrReadData = Wire(Vec(numLanes, UInt(XLEN.W)))
  for (i <- 0 until numLanes) {
    csrReadData(i) := MuxCase(broadcastVal(i), Seq(
      isThreadId -> wtid(i),
      isMhartId  -> gtid(i)
    ))
  }

  // -------------------------------------------------------------------------
  // CSR write
  // -------------------------------------------------------------------------
  val csrWriteEnable = io.ex_op_type === INST_SFU_CSRRW.U

  // Source operand: immediate or rs1[0]
  val csrReqData = Mux(io.ex_csr_use_imm, io.ex_csr_imm.pad(XLEN), io.ex_rs1_data(0))
  val csrWrEnable = csrWriteEnable || csrReqData.orR

  // CSRRW: write_data = rs1/imm
  // CSRRS: write_data = rw | rs1/imm
  // CSRRC: write_data = rw & ~rs1/imm  (default)
  val csrWriteData = MuxCase(csrReadDataRw & ~csrReqData, Seq(
    (io.ex_op_type === INST_SFU_CSRRW.U) -> csrReqData,
    (io.ex_op_type === INST_SFU_CSRRS.U) -> (csrReadDataRw | csrReqData)
  ))

  csrData.io.write_enable := csrReqValid && csrWrEnable
  csrData.io.write_uuid   := io.ex_uuid
  csrData.io.write_wid    := io.ex_wid
  csrData.io.write_addr   := io.ex_csr_addr
  csrData.io.write_data   := csrWriteData

  // -------------------------------------------------------------------------
  // Warp unlock after FPU CSR access EOP
  // -------------------------------------------------------------------------
  val csrReqReady = Wire(Bool())
  io.unlock_warp := csrReqValid && csrReqReady && io.ex_eop && isFpuCsr
  io.unlock_wid  := io.ex_wid

  io.execute_ready := csrReqReady && noPendingInstr

  // -------------------------------------------------------------------------
  // 2-entry elastic result buffer
  // -------------------------------------------------------------------------
  val bufValid  = RegInit(false.B)
  val bufUuid   = Reg(UInt(UUID_WIDTH.W))
  val bufWid    = Reg(UInt(NW_WIDTH.W))
  val bufTmask  = Reg(UInt(numLanes.W))
  val bufPc     = Reg(UInt(PC_BITS.W))
  val bufRd     = Reg(UInt(NUM_REGS_BITS.W))
  val bufWb     = Reg(Bool())
  val bufData   = Reg(Vec(numLanes, UInt(XLEN.W)))
  val bufPid    = Reg(UInt(PID_WIDTH.W))
  val bufSop    = Reg(Bool())
  val bufEop    = Reg(Bool())

  csrReqReady := !bufValid || io.result_ready

  when (csrReqValid && csrReqReady) {
    bufValid := true.B
    bufUuid  := io.ex_uuid
    bufWid   := io.ex_wid
    bufTmask := io.ex_tmask
    bufPc    := io.ex_PC
    bufRd    := io.ex_rd
    bufWb    := io.ex_wb
    bufData  := csrReadData
    bufPid   := io.ex_pid
    bufSop   := io.ex_sop
    bufEop   := io.ex_eop
  }.elsewhen (io.result_ready && bufValid) {
    bufValid := false.B
  }

  io.result_valid := bufValid
  io.res_uuid     := bufUuid
  io.res_wid      := bufWid
  io.res_tmask    := bufTmask
  io.res_PC       := bufPc
  io.res_wb       := bufWb
  io.res_rd       := bufRd
  io.res_data     := bufData
  io.res_pid      := bufPid
  io.res_sop      := bufSop
  io.res_eop      := bufEop
}
