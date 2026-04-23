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
// Translated from VX_lsu_slice.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

// ---------------------------------------------------------------------------
// LsuSlice — single LSU lane slice.
//
// Mirrors VX_lsu_slice.sv:
//  - Computes full byte-address from rs1 + offset
//  - Classifies address as I/O or local-memory
//  - Tracks in-flight fence state
//  - Formats byte-enables and shift-rotated store data
//  - Instantiates MemScheduler (modelled as a blackbox placeholder here)
//  - Unpacks the response tag and sign/zero-extends load data
//  - Two result paths: load-response path and no-response (store/fence) path
//    merged with a 2-input priority arbiter
// ---------------------------------------------------------------------------
class LsuSlice(instanceId: String = "lsu_slice") extends Module {
  import VortexConfigConstants._
  import VortexGPUPkg._

  // --------------------------------------------------------------------------
  // Derived constants (match VX_lsu_slice.sv localparam)
  // --------------------------------------------------------------------------
  private val NUM_LANES   = NUM_LSU_LANES
  private val PID_BITS    = log2Ceil(NUM_THREADS / NUM_LANES)
  private val PID_WIDTH   = up(PID_BITS)
  private val LSUQ_SIZEW  = log2Ceil(LSUQ_IN_SIZE)
  private val REQ_ASHIFT  = log2Ceil(LSU_WORD_SIZE)
  private val MEM_ASHIFT  = log2Ceil(MEM_BLOCK_SIZE)
  private val MEM_ADDRW   = MEM_ADDR_WIDTH - MEM_ASHIFT

  // tag_id = wid + PC + wb + rd + op_type + align + pid + pkt_addr + fence
  private val TAG_ID_WIDTH =
    NW_WIDTH + PC_BITS + 1 + NUM_REGS_BITS + INST_LSU_BITS + (NUM_LANES * REQ_ASHIFT) + PID_WIDTH + LSUQ_SIZEW + 1
  private val TAG_WIDTH = UUID_WIDTH + TAG_ID_WIDTH

  // RSP_ARB_DATAW = uuid + wid + tmask + PC + wb + rd + data + pid + sop + eop
  private val RSP_ARB_DATAW =
    UUID_WIDTH + NW_WIDTH + NUM_LANES + PC_BITS + 1 + NUM_REGS_BITS + (NUM_LANES * XLEN) + PID_WIDTH + 1 + 1

  // --------------------------------------------------------------------------
  // I/O
  // --------------------------------------------------------------------------
  val io = IO(new Bundle {
    // Execution-stage input (slave)
    val execute_valid  = Input(Bool())
    val execute_ready  = Output(Bool())
    // execute data fields
    val ex_uuid        = Input(UInt(UUID_WIDTH.W))
    val ex_wid         = Input(UInt(NW_WIDTH.W))
    val ex_tmask       = Input(UInt(NUM_LANES.W))
    val ex_PC          = Input(UInt(PC_BITS.W))
    val ex_op_type     = Input(UInt(INST_LSU_BITS.W))
    val ex_wb          = Input(Bool())
    val ex_rd          = Input(UInt(NUM_REGS_BITS.W))
    val ex_rs1_data    = Input(Vec(NUM_LANES, UInt(XLEN.W)))
    val ex_rs2_data    = Input(Vec(NUM_LANES, UInt(XLEN.W)))
    val ex_lsu_offset  = Input(SInt(OFFSET_BITS.W))     // op_args.lsu.offset
    val ex_lsu_is_store= Input(Bool())                   // op_args.lsu.is_store
    val ex_pid         = Input(UInt(PID_WIDTH.W))
    val ex_sop         = Input(Bool())
    val ex_eop         = Input(Bool())

    // Result output (master)
    val result_valid   = Output(Bool())
    val result_ready   = Input(Bool())
    val res_uuid       = Output(UInt(UUID_WIDTH.W))
    val res_wid        = Output(UInt(NW_WIDTH.W))
    val res_tmask      = Output(UInt(NUM_LANES.W))
    val res_PC         = Output(UInt(PC_BITS.W))
    val res_wb         = Output(Bool())
    val res_rd         = Output(UInt(NUM_REGS_BITS.W))
    val res_data       = Output(Vec(NUM_LANES, UInt(XLEN.W)))
    val res_pid        = Output(UInt(PID_WIDTH.W))
    val res_sop        = Output(Bool())
    val res_eop        = Output(Bool())

    // LSU memory interface (master)
    val mem_req_valid  = Output(Bool())
    val mem_req_ready  = Input(Bool())
    val mem_req_mask   = Output(UInt(NUM_LANES.W))
    val mem_req_rw     = Output(Bool())
    val mem_req_addr   = Output(Vec(NUM_LANES, UInt(LSU_ADDR_WIDTH.W)))
    val mem_req_byteen = Output(Vec(NUM_LANES, UInt(LSU_WORD_SIZE.W)))
    val mem_req_data   = Output(Vec(NUM_LANES, UInt((LSU_WORD_SIZE * 8).W)))
    val mem_req_flags  = Output(Vec(NUM_LANES, UInt(MEM_FLAGS_WIDTH.W)))
    val mem_req_tag    = Output(UInt(TAG_WIDTH.W))

    val mem_rsp_valid  = Input(Bool())
    val mem_rsp_ready  = Output(Bool())
    val mem_rsp_mask   = Input(UInt(NUM_LANES.W))
    val mem_rsp_data   = Input(Vec(NUM_LANES, UInt((LSU_WORD_SIZE * 8).W)))
    val mem_rsp_tag    = Input(UInt(TAG_WIDTH.W))
    val mem_rsp_sop    = Input(Bool())
    val mem_rsp_eop    = Input(Bool())
  })

  // --------------------------------------------------------------------------
  // Full address = rs1 + sign-extended offset
  // --------------------------------------------------------------------------
  val fullAddr = VecInit(Seq.tabulate(NUM_LANES) { i =>
    (io.ex_rs1_data(i).asSInt + io.ex_lsu_offset).asUInt(XLEN - 1, 0)
  })

  // --------------------------------------------------------------------------
  // Fence detection
  // --------------------------------------------------------------------------
  val reqIsFence = inst_lsu_is_fence(io.ex_op_type)

  // --------------------------------------------------------------------------
  // Memory request flags (IO / local address classification)
  // --------------------------------------------------------------------------
  val IO_BASE    = IO_BASE_ADDR.toLong
  // IO_END_ADDR = USER_BASE_ADDR (0x00010000) from VX_config.vh
  val IO_END_VAL  = USER_BASE_ADDR
  val LMEM_BASE   = LMEM_BASE_ADDR
  val LMEM_END    = LMEM_BASE + (1L << LMEM_LOG_SIZE)

  val ioAddrStart  = (IO_BASE  >> MEM_ASHIFT).U(MEM_ADDRW.W)
  val ioAddrEnd    = (IO_END_VAL >> MEM_ASHIFT).U(MEM_ADDRW.W)
  val lmemAddrStart= (LMEM_BASE >> MEM_ASHIFT).U(MEM_ADDRW.W)
  val lmemAddrEnd  = (LMEM_END  >> MEM_ASHIFT).U(MEM_ADDRW.W)

  val memReqFlags = Wire(Vec(NUM_LANES, UInt(MEM_FLAGS_WIDTH.W)))
  for (i <- 0 until NUM_LANES) {
    val blockAddr = fullAddr(i)(MEM_ASHIFT + MEM_ADDRW - 1, MEM_ASHIFT)
    val isIO      = (blockAddr >= ioAddrStart)   && (blockAddr < ioAddrEnd)
    val isLmem    = (blockAddr >= lmemAddrStart) && (blockAddr < lmemAddrEnd)
    val flush     = reqIsFence
    memReqFlags(i) := Cat(isLmem, isIO, flush)
  }

  // --------------------------------------------------------------------------
  // Alignment (low REQ_ASHIFT bits of address, per lane)
  // --------------------------------------------------------------------------
  val reqAlign = VecInit(Seq.tabulate(NUM_LANES)(i => fullAddr(i)(REQ_ASHIFT - 1, 0)))

  // --------------------------------------------------------------------------
  // Memory request address (word-aligned)
  // --------------------------------------------------------------------------
  val memReqAddr = VecInit(Seq.tabulate(NUM_LANES)(i =>
    fullAddr(i)(MEM_ADDR_WIDTH - 1, REQ_ASHIFT)
  ))

  // --------------------------------------------------------------------------
  // Byte-enable generation
  // --------------------------------------------------------------------------
  val memReqByteen = Wire(Vec(NUM_LANES, UInt(LSU_WORD_SIZE.W)))
  for (i <- 0 until NUM_LANES) {
    val wsize = inst_lsu_wsize(io.ex_op_type)
    val byteen = Wire(UInt(LSU_WORD_SIZE.W))
    // XLEN=32 → LSU_WORD_SIZE=4 bytes, REQ_ASHIFT=2
    // wsize 0→byte, 1→halfword, default(2)→word
    byteen := MuxCase(Fill(LSU_WORD_SIZE, 1.U(1.W)), Seq(
      (wsize === 0.U) -> (1.U(LSU_WORD_SIZE.W) << reqAlign(i)),
      (wsize === 1.U) -> (3.U(LSU_WORD_SIZE.W) << Cat(reqAlign(i)(REQ_ASHIFT-1, 1), 0.U(1.W)))
    ))
    memReqByteen(i) := byteen
  }

  // --------------------------------------------------------------------------
  // Store data shift (rotate data to aligned position)
  // --------------------------------------------------------------------------
  val memReqData = Wire(Vec(NUM_LANES, UInt((LSU_WORD_SIZE * 8).W)))
  for (i <- 0 until NUM_LANES) {
    val shiftBytes = reqAlign(i)
    memReqData(i) := io.ex_rs2_data(i) << Cat(shiftBytes, 0.U(3.W))
  }

  // --------------------------------------------------------------------------
  // Packet counter for multi-packet tracking (PID_BITS != 0 case)
  // --------------------------------------------------------------------------
  // pkt_waddr: address allocated for current request
  // pkt_raddr: address of current response
  // For simplicity when PID_BITS==0, both are tied to 0.
  val pktWaddr = Wire(UInt(LSUQ_SIZEW.W))
  val pktRaddr = Wire(UInt(LSUQ_SIZEW.W))

  // Allocator state
  val pktAllocValid = RegInit(VecInit(Seq.fill(LSUQ_IN_SIZE)(false.B)))
  val pktAddrNext   = RegInit(0.U(LSUQ_SIZEW.W))

  // For PID_BITS==0, just tie to 0
  if (PID_BITS == 0) {
    pktWaddr := 0.U
    pktRaddr := 0.U
  } else {
    // Simple free-list allocator (equivalent to VX_allocator)
    // Allocate on read-request EOP fire, release on response EOP fire
    val memReqFire   = Wire(Bool())
    val memRspFire   = Wire(Bool())
    val memRspEopWire= Wire(Bool())

    // We use a small free-list counter (sufficient for LSUQ_IN_SIZE <= 4)
    val freeSlots    = RegInit(((1 << LSUQ_IN_SIZE) - 1).U(LSUQ_IN_SIZE.W))
    val allocIdx     = Wire(UInt(LSUQ_SIZEW.W))
    val freeIdx      = Wire(UInt(LSUQ_SIZEW.W))

    // Find lowest free slot
    allocIdx := PriorityEncoder(freeSlots)
    // Reconstruct raddr from tag
    // The pkt_raddr is embedded in the response tag; extract it
    // tag layout: { uuid, wid, PC, wb, rd, op_type, align[lanes], pid, pkt_addr, fence }
    val tagPktAddrOff = 1  // pkt_addr starts at bit 1 (right after fence bit at bit 0)
    freeIdx := io.mem_rsp_tag(tagPktAddrOff + LSUQ_SIZEW - 1, tagPktAddrOff)

    pktWaddr := allocIdx
    pktRaddr := freeIdx

    val rdFire     = memReqFire && !io.mem_req_rw
    val rdEopFire  = rdFire && io.ex_eop
    val rspEopPktFire = memRspFire && memRspEopWire

    when (rdEopFire) {
      freeSlots := freeSlots & ~(1.U(LSUQ_IN_SIZE.W) << allocIdx)
    }
    when (rspEopPktFire) {
      freeSlots := freeSlots | (1.U(LSUQ_IN_SIZE.W) << freeIdx)
    }

    // wire up
    memReqFire   := io.mem_req_valid && io.mem_req_ready
    memRspFire   := io.mem_rsp_valid && io.mem_rsp_ready
    memRspEopWire:= io.mem_rsp_eop
  }

  // --------------------------------------------------------------------------
  // Fence lock register
  // --------------------------------------------------------------------------
  val fenceLock = RegInit(false.B)

  val memReqFire = io.mem_req_valid && io.mem_req_ready
  val memRspFire = io.mem_rsp_valid && io.mem_rsp_ready

  // Unpack response tag to detect fence response
  // tag layout (MSB→LSB): { uuid, wid, PC, wb, rd, op_type, align, pid, pkt_raddr, fence }
  val rspTagFenceOff = 0
  val rspIsFence     = io.mem_rsp_tag(rspTagFenceOff)

  // Simple SOP/EOP tracking for response packets
  val memRspSopPkt: Bool = if (PID_BITS == 0) io.mem_rsp_sop else {
    // When PID_BITS != 0, sop_pkt comes from pkt_sop register
    // For structural simplicity we conservatively pass through mem_rsp_sop
    io.mem_rsp_sop
  }
  val memRspEopPkt: Bool = if (PID_BITS == 0) io.mem_rsp_eop else io.mem_rsp_eop

  when (memReqFire && reqIsFence && io.ex_eop) {
    fenceLock := true.B
  }
  when (memRspFire && rspIsFence && memRspEopPkt) {
    fenceLock := false.B
  }

  // --------------------------------------------------------------------------
  // No-response buffer enable: stores without writeback, or fence non-EOP packets
  // --------------------------------------------------------------------------
  val reqSkip        = reqIsFence && !io.ex_eop
  val noRspBufEnable = (io.mem_req_rw && !io.ex_wb) || reqSkip

  // Both buffers use lsu_res_t (ExeResBundle). Stores leave wb=false; rd/data
  // are don't-care and are never written to the register file.
  // Declared here so their .ready can be used in execute_ready below.
  val noRspEnq = Wire(Decoupled(new ExeResBundle(NUM_LANES)))
  val rspEnq   = Wire(Decoupled(new ExeResBundle(NUM_LANES)))

  // Gate mem_req_valid on no-response buffer space
  io.mem_req_valid := io.execute_valid && !reqSkip &&
    !(noRspBufEnable && !noRspEnq.ready) &&
    !fenceLock

  noRspEnq.valid       := io.execute_valid &&
    noRspBufEnable &&
    (reqSkip || io.mem_req_ready) &&
    !fenceLock
  noRspEnq.bits.uuid   := io.ex_uuid
  noRspEnq.bits.wid    := io.ex_wid
  noRspEnq.bits.tmask  := io.ex_tmask
  noRspEnq.bits.PC     := io.ex_PC
  noRspEnq.bits.wb     := false.B
  noRspEnq.bits.rd     := 0.U
  noRspEnq.bits.data   := DontCare
  noRspEnq.bits.pid    := io.ex_pid
  noRspEnq.bits.sop    := io.ex_sop
  noRspEnq.bits.eop    := io.ex_eop

  io.execute_ready := (io.mem_req_ready || reqSkip) &&
    !(noRspBufEnable && !noRspEnq.ready) &&
    !fenceLock

  // Wire request signals
  io.mem_req_mask   := io.ex_tmask
  io.mem_req_rw     := io.ex_lsu_is_store
  io.mem_req_addr   := memReqAddr
  io.mem_req_byteen := memReqByteen
  io.mem_req_data   := memReqData
  io.mem_req_flags  := memReqFlags

  // Pack request tag: { uuid, wid, PC, wb, rd, op_type, align, pid, pkt_waddr, fence }
  val alignBits = Wire(UInt((NUM_LANES * REQ_ASHIFT).W))
  alignBits := Cat(reqAlign.reverse)
  io.mem_req_tag := Cat(
    io.ex_uuid,
    io.ex_wid,
    io.ex_PC,
    io.ex_wb,
    io.ex_rd,
    io.ex_op_type,
    alignBits,
    io.ex_pid,
    pktWaddr,
    reqIsFence
  )

  // --------------------------------------------------------------------------
  // Response tag unpack
  // --------------------------------------------------------------------------
  // Layout (MSB→LSB):
  //   [TAG_WIDTH-1 : TAG_WIDTH-UUID_WIDTH]         = uuid
  //   [... : ...] wid, PC, wb, rd, op_type, align, pid, pkt_raddr, fence
  val rspTagWire = io.mem_rsp_tag
  var bitOff     = 0

  val rspIsFenceUnpk = rspTagWire(bitOff); bitOff += 1
  // pkt_raddr: LSUQ_SIZEW bits (consumed above for pktRaddr calculation)
  bitOff += LSUQ_SIZEW
  val rspPid     = rspTagWire(bitOff + PID_WIDTH - 1, bitOff); bitOff += PID_WIDTH
  val rspAlign   = VecInit(Seq.tabulate(NUM_LANES)(i =>
    rspTagWire(bitOff + (i + 1) * REQ_ASHIFT - 1, bitOff + i * REQ_ASHIFT)
  )); bitOff += NUM_LANES * REQ_ASHIFT
  val rspOpType  = rspTagWire(bitOff + INST_LSU_BITS - 1, bitOff); bitOff += INST_LSU_BITS
  val rspRd      = rspTagWire(bitOff + NUM_REGS_BITS - 1, bitOff); bitOff += NUM_REGS_BITS
  val rspWb      = rspTagWire(bitOff); bitOff += 1
  val rspPc      = rspTagWire(bitOff + PC_BITS - 1, bitOff); bitOff += PC_BITS
  val rspWid     = rspTagWire(bitOff + NW_WIDTH - 1, bitOff); bitOff += NW_WIDTH
  val rspUuid    = rspTagWire(bitOff + UUID_WIDTH - 1, bitOff)

  // --------------------------------------------------------------------------
  // Load response data formatting (sign/zero-extend)
  // --------------------------------------------------------------------------
  val rspData = Wire(Vec(NUM_LANES, UInt(XLEN.W)))
  for (i <- 0 until NUM_LANES) {
    // 32-bit path (XLEN=32 assumed)
    val raw32    = io.mem_rsp_data(i)(31, 0)
    val rspData16 = Mux(rspAlign(i)(1), raw32(31, 16), raw32(15, 0))
    val rspData8  = Mux(rspAlign(i)(0), rspData16(15, 8), rspData16(7, 0))

    val fmt = inst_lsu_fmt(rspOpType)
    rspData(i) := MuxLookup(fmt, 0.U)(Seq(
      LSU_FMT_B.U  -> rspData8.asSInt.pad(XLEN).asUInt,
      LSU_FMT_H.U  -> rspData16.asSInt.pad(XLEN).asUInt,
      LSU_FMT_BU.U -> rspData8.pad(XLEN),
      LSU_FMT_HU.U -> rspData16.pad(XLEN),
      LSU_FMT_W.U  -> raw32.asSInt.pad(XLEN).asUInt
    ))
  }

  // --------------------------------------------------------------------------
  // Response-path elastic buffer — Queue(2, pipe=true) matches VX_elastic_buffer
  // --------------------------------------------------------------------------
  rspEnq.valid      := io.mem_rsp_valid
  rspEnq.bits.uuid  := rspUuid
  rspEnq.bits.wid   := rspWid
  rspEnq.bits.tmask := io.mem_rsp_mask
  rspEnq.bits.PC    := rspPc
  rspEnq.bits.wb    := rspWb
  rspEnq.bits.rd    := rspRd
  rspEnq.bits.data  := rspData
  rspEnq.bits.pid   := rspPid
  rspEnq.bits.sop   := memRspSopPkt
  rspEnq.bits.eop   := memRspEopPkt
  io.mem_rsp_ready  := rspEnq.ready

  val rspDeq   = Queue(rspEnq,   entries = 2, pipe = true)
  val noRspDeq = Queue(noRspEnq, entries = 2, pipe = true)

  // --------------------------------------------------------------------------
  // 2-input priority arbiter (rsp has priority over noRsp)
  // --------------------------------------------------------------------------
  val arbRspPri   = rspDeq.valid
  val arbNoRspPri = noRspDeq.valid && !rspDeq.valid

  io.result_valid := arbRspPri || arbNoRspPri

  rspDeq.ready   := io.result_ready && arbRspPri
  noRspDeq.ready := io.result_ready && arbNoRspPri

  io.res_uuid  := Mux(arbRspPri, rspDeq.bits.uuid,  noRspDeq.bits.uuid)
  io.res_wid   := Mux(arbRspPri, rspDeq.bits.wid,   noRspDeq.bits.wid)
  io.res_tmask := Mux(arbRspPri, rspDeq.bits.tmask, noRspDeq.bits.tmask)
  io.res_PC    := Mux(arbRspPri, rspDeq.bits.PC,    noRspDeq.bits.PC)
  io.res_wb    := rspDeq.bits.wb   // noRsp always has wb=false; Mux not needed
  io.res_rd    := rspDeq.bits.rd
  io.res_data  := rspDeq.bits.data // noRsp data is DontCare; arbitration via wb
  io.res_pid   := Mux(arbRspPri, rspDeq.bits.pid,   noRspDeq.bits.pid)
  io.res_sop   := Mux(arbRspPri, rspDeq.bits.sop,   noRspDeq.bits.sop)
  io.res_eop   := Mux(arbRspPri, rspDeq.bits.eop,   noRspDeq.bits.eop)
}
