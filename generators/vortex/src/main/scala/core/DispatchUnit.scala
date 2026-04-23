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
// Translated from VX_dispatch_unit.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// DispatchUnit — per-execution-unit dispatch arbiter and SIMD splitter.
//
// Mirrors VX_dispatch_unit.sv:
//   - Accepts ISSUE_WIDTH dispatch_if streams (one per issue slot).
//   - Groups them into batches of BLOCK_SIZE (one block per execution-unit
//     lane group).  A priority arbiter selects which batch to service.
//   - If SIMD_WIDTH > NUM_LANES, each SIMD warp is split into NUM_PACKETS
//     sub-packets (via VX_nz_iterator), with sop/eop framing.
//   - Outputs BLOCK_SIZE execute_if streams carrying NUM_LANES-wide data.
//
// Parameters:
//   blockSize – BLOCK_SIZE: number of parallel execute slots
//   numLanes  – NUM_LANES:  SIMD lanes per execute slot
//   outBuf    – output buffer depth (maps to VX_elastic_buffer SIZE)
//
// The SIMD splitting (VX_nz_iterator) is modelled with a simple counter
// when SIMD_WIDTH != NUM_LANES.  The full fanout-enable register-pipeline
// optimisation from the SV is not modelled.
// ---------------------------------------------------------------------------
class DispatchUnit(
  blockSize: Int    = 1,
  numLanes:  Int    = NUM_ALU_LANES,
  outBuf:    Int    = 0
) extends Module {
  require(ISSUE_WIDTH % blockSize == 0, "ISSUE_WIDTH must be divisible by BLOCK_SIZE")
  require(SIMD_WIDTH  % numLanes  == 0, "SIMD_WIDTH must be divisible by NUM_LANES")

  private val BLOCK_SIZE_W = up(log2Ceil(blockSize))
  private val NUM_PACKETS  = SIMD_WIDTH / numLanes
  private val LPID_BITS    = log2Ceil(NUM_PACKETS)
  private val LPID_WIDTH   = up(LPID_BITS)
  private val GPID_BITS    = log2Ceil(NUM_THREADS / numLanes)
  private val GPID_WIDTH   = up(GPID_BITS)
  private val BATCH_COUNT  = ISSUE_WIDTH / blockSize
  private val BATCH_COUNT_W= up(log2Ceil(BATCH_COUNT))
  private val ISSUE_W      = up(log2Ceil(ISSUE_WIDTH))

  // DispatchBundle bit width (IN)
  // uuid + wis + sid + tmask(SIMD) + PC + op_type + op_args + wb + rd +
  // rs1(SIMD*XLEN) + rs2(SIMD*XLEN) + rs3(SIMD*XLEN) + sop + eop
  private val IN_DATAW =
    UUID_WIDTH + ISSUE_WIS_W + SIMD_IDX_W + SIMD_WIDTH +
    PC_BITS + INST_OP_BITS + INST_ARGS_BITS + 1 + NUM_REGS_BITS +
    (3 * SIMD_WIDTH * XLEN) + 1 + 1

  // ExecuteDataBundle bit width (OUT per block): replace SIMD_WIDTH lanes with numLanes
  private val OUT_DATAW =
    UUID_WIDTH + NW_WIDTH + numLanes +
    INST_OP_BITS + INST_ARGS_BITS + 1 + PC_BITS + NUM_REGS_BITS +
    (3 * numLanes * XLEN) + GPID_WIDTH + 1 + 1

  // -------------------------------------------------------------------------
  // I/O
  // -------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Dispatch inputs (slave) – one per ISSUE_WIDTH
    val dispatch_valid  = Input(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_ready  = Output(Vec(ISSUE_WIDTH, Bool()))
    val dispatch_data   = Input(Vec(ISSUE_WIDTH, new DispatchBundle))

    // Execute outputs (master) – one per BLOCK_SIZE
    val execute_valid   = Output(Vec(blockSize, Bool()))
    val execute_ready   = Input(Vec(blockSize, Bool()))
    val ex_uuid         = Output(Vec(blockSize, UInt(UUID_WIDTH.W)))
    val ex_wid          = Output(Vec(blockSize, UInt(NW_WIDTH.W)))
    val ex_tmask        = Output(Vec(blockSize, UInt(numLanes.W)))
    val ex_PC           = Output(Vec(blockSize, UInt(PC_BITS.W)))
    val ex_op_type      = Output(Vec(blockSize, UInt(INST_OP_BITS.W)))
    val ex_op_args      = Output(Vec(blockSize, new OpArgsBundle))
    val ex_wb           = Output(Vec(blockSize, Bool()))
    val ex_rd           = Output(Vec(blockSize, UInt(NUM_REGS_BITS.W)))
    val ex_rs1_data     = Output(Vec(blockSize, Vec(numLanes, UInt(XLEN.W))))
    val ex_rs2_data     = Output(Vec(blockSize, Vec(numLanes, UInt(XLEN.W))))
    val ex_rs3_data     = Output(Vec(blockSize, Vec(numLanes, UInt(XLEN.W))))
    val ex_pid          = Output(Vec(blockSize, UInt(GPID_WIDTH.W)))
    val ex_sop          = Output(Vec(blockSize, Bool()))
    val ex_eop          = Output(Vec(blockSize, Bool()))
  })

  // -------------------------------------------------------------------------
  // Batch index selection
  // -------------------------------------------------------------------------
  // batch_idx selects which group of BLOCK_SIZE dispatch slots to service.
  // -------------------------------------------------------------------------
  // Execute output bundle (one entry per block)
  // -------------------------------------------------------------------------
  class ExecBundle extends Bundle {
    val uuid     = UInt(UUID_WIDTH.W)
    val wid      = UInt(NW_WIDTH.W)
    val tmask    = UInt(numLanes.W)
    val PC       = UInt(PC_BITS.W)
    val op_type  = UInt(INST_OP_BITS.W)
    val op_args  = new OpArgsBundle
    val wb       = Bool()
    val rd       = UInt(NUM_REGS_BITS.W)
    val rs1_data = Vec(numLanes, UInt(XLEN.W))
    val rs2_data = Vec(numLanes, UInt(XLEN.W))
    val rs3_data = Vec(numLanes, UInt(XLEN.W))
    val pid      = UInt(GPID_WIDTH.W)
    val sop      = Bool()
    val eop      = Bool()
  }

  // -------------------------------------------------------------------------
  // Per-block wiring (declared before batchIdx so BATCH_COUNT>1 can reference)
  // -------------------------------------------------------------------------
  val blockReadyWire = Wire(Vec(blockSize, Bool()))
  val blockDoneWire  = Wire(Vec(blockSize, Bool()))

  val batchIdx = if (BATCH_COUNT == 1) {
    0.U(BATCH_COUNT_W.W)
  } else {
    val batchIdxReg = RegInit(0.U(BATCH_COUNT_W.W))
    val validBatches = VecInit(Seq.tabulate(BATCH_COUNT)(b =>
      VecInit(Seq.tabulate(blockSize)(s =>
        io.dispatch_valid(b * blockSize + s)
      )).asUInt.orR
    ))
    val batchDone = blockDoneWire.asUInt.andR

    // Priority arbiter for next batch
    val nextBatch = PriorityEncoder(validBatches.asUInt)
    when (batchDone) {
      batchIdxReg := nextBatch
    }
    batchIdxReg
  }

  val dispReadyOut = WireInit(VecInit(Seq.fill(ISSUE_WIDTH)(false.B)))

  for (b <- 0 until blockSize) {
    val issueIdx = (batchIdx * blockSize.U + b.U)(ISSUE_W - 1, 0)

    // Extract current dispatch slot data
    val dValid   = io.dispatch_valid(issueIdx)
    val dData    = io.dispatch_data(issueIdx)

    val dWis    = dData.wis
    val dSid    = dData.sid
    val dSop    = dData.sop
    val dEop    = dData.eop
    val dTmask  = dData.tmask
    val dRs1    = dData.rs1_data
    val dRs2    = dData.rs2_data
    val dRs3    = dData.rs3_data

    // -----------------------------------------------------------------------
    // SIMD sub-packet iteration
    // -----------------------------------------------------------------------
    val pidReg   = RegInit(0.U(LPID_WIDTH.W))
    val isSopP   = Wire(Bool())
    val isEopP   = Wire(Bool())
    val validP   = Wire(Bool())
    val readyP   = Wire(Bool())

    val selTmask = Wire(UInt(numLanes.W))
    val selRs1   = Wire(Vec(numLanes, UInt(XLEN.W)))
    val selRs2   = Wire(Vec(numLanes, UInt(XLEN.W)))
    val selRs3   = Wire(Vec(numLanes, UInt(XLEN.W)))

    if (NUM_PACKETS == 1 || SIMD_WIDTH == numLanes) {
      // Full SIMD: no splitting needed
      validP   := dValid
      readyP   := blockReadyWire(b)
      isSopP   := true.B
      isEopP   := true.B
      selTmask := dTmask.asUInt(numLanes - 1, 0)
      for (l <- 0 until numLanes) {
        selRs1(l) := dRs1(l)
        selRs2(l) := dRs2(l)
        selRs3(l) := dRs3(l)
      }
      blockDoneWire(b) := readyP || !dValid
    } else {
      // Partial SIMD: iterate over NUM_PACKETS sub-packets
      val fireP = validP && readyP

      isSopP := (pidReg === 0.U)
      isEopP := (pidReg === (NUM_PACKETS - 1).U)
      validP := dValid

      when (fireP) {
        when (isEopP) {
          pidReg := 0.U
        }.otherwise {
          pidReg := pidReg + 1.U
        }
      }

      // Select lanes for current packet
      selTmask := (dTmask.asUInt >> (pidReg * numLanes.U))(numLanes - 1, 0)
      for (l <- 0 until numLanes) {
        // Chisel doesn't support variable-index slice in Vec; use MuxLookup
        val laneIdx = pidReg * numLanes.U + l.U
        selRs1(l) := MuxLookup(laneIdx, 0.U)(
          Seq.tabulate(SIMD_WIDTH)(k => k.U -> dRs1(k))
        )
        selRs2(l) := MuxLookup(laneIdx, 0.U)(
          Seq.tabulate(SIMD_WIDTH)(k => k.U -> dRs2(k))
        )
        selRs3(l) := MuxLookup(laneIdx, 0.U)(
          Seq.tabulate(SIMD_WIDTH)(k => k.U -> dRs3(k))
        )
      }

      blockDoneWire(b) := (fireP && isEopP) || !dValid
      readyP := blockReadyWire(b)
    }

    // ISW (issue slot within batch)
    val isw: UInt = if (BATCH_COUNT == 1) {
      if (blockSize == 1) 0.U(ISSUE_W.W)
      else b.U(ISSUE_W.W)
    } else {
      if (blockSize == 1) batchIdx
      else Cat(batchIdx, b.U(BLOCK_SIZE_W.W))
    }

    val blockWid = wis_to_wid(dWis, isw)
    val warpPid  = (pidReg.pad(GPID_WIDTH) + (dSid * NUM_PACKETS.U).pad(GPID_WIDTH))(GPID_WIDTH - 1, 0)
    val warpSop  = isSopP && dSop
    val warpEop  = isEopP && dEop

    // -----------------------------------------------------------------------
    // Output buffer — Queue matches VX_elastic_buffer(SIZE=outBuf, OUT_REG=...)
    // -----------------------------------------------------------------------
    val enq = Wire(Decoupled(new ExecBundle))
    enq.valid          := validP
    enq.bits.uuid      := dData.uuid
    enq.bits.wid       := blockWid
    enq.bits.tmask     := selTmask
    enq.bits.PC        := dData.PC
    enq.bits.op_type   := dData.op_type
    enq.bits.op_args   := dData.op_args
    enq.bits.wb        := dData.wb
    enq.bits.rd        := dData.rd
    enq.bits.rs1_data  := selRs1
    enq.bits.rs2_data  := selRs2
    enq.bits.rs3_data  := selRs3
    enq.bits.pid       := warpPid
    enq.bits.sop       := warpSop
    enq.bits.eop       := warpEop

    blockReadyWire(b) := enq.ready

    val deq = Queue(enq, entries = math.max(1, outBuf), pipe = true)
    deq.ready := io.execute_ready(b)

    io.execute_valid(b) := deq.valid
    io.ex_uuid(b)       := deq.bits.uuid
    io.ex_wid(b)        := deq.bits.wid
    io.ex_tmask(b)      := deq.bits.tmask
    io.ex_PC(b)         := deq.bits.PC
    io.ex_op_type(b)    := deq.bits.op_type
    io.ex_op_args(b)    := deq.bits.op_args
    io.ex_wb(b)         := deq.bits.wb
    io.ex_rd(b)         := deq.bits.rd
    io.ex_rs1_data(b)   := deq.bits.rs1_data
    io.ex_rs2_data(b)   := deq.bits.rs2_data
    io.ex_rs3_data(b)   := deq.bits.rs3_data
    io.ex_pid(b)        := deq.bits.pid
    io.ex_sop(b)        := deq.bits.sop
    io.ex_eop(b)        := deq.bits.eop

    // Release the dispatch slot when all packets for this block have been sent
    dispReadyOut(issueIdx) := enq.ready && isEopP
  }

  io.dispatch_ready := dispReadyOut

}
