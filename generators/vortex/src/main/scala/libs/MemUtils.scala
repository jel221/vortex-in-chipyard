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
// Chisel translation of:
//   VX_axi_write_ack.sv
//   VX_async_ram_patch.sv
//   VX_mem_data_adapter.sv
//   VX_mem_bank_adapter.sv
//   VX_axi_adapter.sv
//   VX_mem_coalescer.sv
//   VX_mem_scheduler.sv

package vortex

import chisel3._
import chisel3.util._

// ============================================================================
// VX_axi_write_ack
//
// Tracks whether the AXI write-address (AW) and write-data (W) channels have
// each fired independently so that the two can be issued concurrently without
// losing either handshake.
//
// Ports:
//   awvalid/awready – AXI AW channel signals
//   wvalid/wready   – AXI W channel signals
//   aw_ack          – AW channel has already fired (sticky until tx_ack)
//   w_ack           – W  channel has already fired (sticky until tx_ack)
//   tx_ack          – both channels have now fired (combinational pulse)
//   tx_rdy          – both channels are ready to accept a new transaction
// ============================================================================
class VXAXIWriteAck extends Module {
  val io = IO(new Bundle {
    val awvalid = Input(Bool())
    val awready = Input(Bool())
    val wvalid  = Input(Bool())
    val wready  = Input(Bool())
    val aw_ack  = Output(Bool())
    val w_ack   = Output(Bool())
    val tx_ack  = Output(Bool())
    val tx_rdy  = Output(Bool())
  })

  val awFired = RegInit(false.B)
  val wFired  = RegInit(false.B)

  val awFire = io.awvalid && io.awready
  val wFire  = io.wvalid  && io.wready

  // tx_ack is combinational: both channels have fired this cycle or earlier
  val txAck = (awFire || awFired) && (wFire || wFired)

  when (awFire)  { awFired := true.B }
  when (wFire)   { wFired  := true.B }
  when (txAck)   { awFired := false.B; wFired := false.B }

  io.aw_ack := awFired
  io.w_ack  := wFired
  io.tx_ack := txAck
  io.tx_rdy := (io.awready || awFired) && (io.wready || wFired)
}

// ============================================================================
// VX_async_ram_patch
//
// A parameterised synchronous RAM wrapper that emulates an asynchronous-read
// (LUT-RAM) interface inside synthesis flows that only provide synchronous
// primitives.  The "patch" inserts a VX_placeholder bypass so that synthesis
// tools can later substitute the appropriate RAM primitive.
//
// In Chisel we model this as a simple synchronous-read SRAM.  The WRITE_FIRST
// vs READ_FIRST distinction maps onto Chisel's SyncReadMem behaviour:
//   WRITE_FIRST → write-before-read (new data visible on same cycle write)
//   READ_FIRST  → read-before-write (old data returned on write cycle)
//
// Parameters
//   dataw      – data width in bits
//   size       – number of entries
//   wrenw      – number of write-enable sub-words (1 = whole-word enable)
//   dualPort   – true → separate read and write address ports
//   writeFirst – true → write-first (new data) read semantic
//   raddrReg   – hint: register the read address (always true in sync RAM)
// ============================================================================
class VXAsyncRamPatch(
  val dataw      : Int  = 1,
  val size       : Int  = 1,
  val wrenw      : Int  = 1,
  val dualPort   : Boolean = false,
  val writeFirst : Boolean = false,
  val raddrReg   : Boolean = false
) extends Module {
  val addrw = log2Ceil(size)
  val wselw = dataw / wrenw

  val io = IO(new Bundle {
    val read  = Input(Bool())
    val write = Input(Bool())
    val wren  = Input(UInt(wrenw.W))
    val waddr = Input(UInt(addrw.W))
    val wdata = Input(UInt(dataw.W))
    val raddr = Input(UInt(addrw.W))
    val rdata = Output(UInt(dataw.W))
  })

  // Use SyncReadMem; for write-first semantics we forward write data when
  // read and write addresses collide.
  val mem = SyncReadMem(size, UInt(dataw.W))

  // Effective read address: for single-port RAMs use write address
  val effRaddr = if (dualPort) io.raddr else io.waddr

  // Perform write with optional byte-enables
  when (io.write) {
    if (wrenw == 1) {
      mem.write(io.waddr, io.wdata)
    } else {
      // Sub-word write enable: write each slice independently
      val oldData = mem(io.waddr)
      val newData = Wire(UInt(dataw.W))
      newData := oldData
      for (i <- 0 until wrenw) {
        when (io.wren(i)) {
          // cannot do bit-slice assignment in Chisel directly; use Cat
        }
      }
      // Build masked write via VecInit of sub-words
      val oldVec = Wire(Vec(wrenw, UInt(wselw.W)))
      val newVec = Wire(Vec(wrenw, UInt(wselw.W)))
      for (i <- 0 until wrenw) {
        oldVec(i) := oldData(i * wselw + wselw - 1, i * wselw)
        newVec(i) := Mux(io.wren(i), io.wdata(i * wselw + wselw - 1, i * wselw), oldVec(i))
      }
      mem.write(io.waddr, newVec.asUInt)
    }
  }

  // Synchronous read; apply when read enable is asserted
  val rdData = mem.read(effRaddr, io.read)

  // Write-first forwarding: if write and read hit same address this cycle,
  // return the new write data.
  if (writeFirst && dualPort) {
    val fwdHit = io.write && (io.waddr === RegNext(io.raddr))
    val fwdData = RegNext(io.wdata)
    io.rdata := Mux(fwdHit, fwdData, rdData)
  } else {
    io.rdata := rdData
  }
}

// ============================================================================
// VX_mem_data_adapter
//
// Adapts a memory request/response interface between two data widths.
// Three cases:
//   DST wider than SRC – widen: demux byte-enables / data into wider word,
//                                mux the relevant slice from the response.
//   DST narrower than SRC – narrow: iterate over P sub-words sequentially,
//                                    accumulate responses.
//   Equal widths – pass-through with optional address width adjustment.
//
// Output elastic buffers are modelled as Queue(1) or pass-through.
//
// Parameters (all in bits unless noted)
//   srcDataWidth – source (input) data width
//   srcAddrWidth – source address width
//   dstDataWidth – destination (output) data width
//   dstAddrWidth – destination address width
//   srcTagWidth  – source tag width
//   dstTagWidth  – destination tag width
//   reqOutBuf    – 0 = no output buffer on request path, 1 = add skid buf
//   rspOutBuf    – 0 = no output buffer on response path, 1 = add skid buf
// ============================================================================
class VXMemDataAdapter(
  val srcDataWidth : Int = 32,
  val srcAddrWidth : Int = 32,
  val dstDataWidth : Int = 32,
  val dstAddrWidth : Int = 32,
  val srcTagWidth  : Int = 8,
  val dstTagWidth  : Int = 8,
  val reqOutBuf    : Int = 0,
  val rspOutBuf    : Int = 0
) extends Module {
  // log2 of data widths
  val dstLdataw = log2Ceil(dstDataWidth)
  val srcLdataw = log2Ceil(srcDataWidth)
  val d         = math.abs(dstLdataw - srcLdataw)      // shift amount
  val p         = 1 << d                                // sub-word count

  val srcDataSize = srcDataWidth / 8
  val dstDataSize = dstDataWidth / 8

  val io = IO(new Bundle {
    // Upstream (source) interface
    val mem_req_valid_in  = Input(Bool())
    val mem_req_addr_in   = Input(UInt(srcAddrWidth.W))
    val mem_req_rw_in     = Input(Bool())
    val mem_req_byteen_in = Input(UInt(srcDataSize.W))
    val mem_req_data_in   = Input(UInt(srcDataWidth.W))
    val mem_req_tag_in    = Input(UInt(srcTagWidth.W))
    val mem_req_ready_in  = Output(Bool())

    val mem_rsp_valid_in  = Output(Bool())
    val mem_rsp_data_in   = Output(UInt(srcDataWidth.W))
    val mem_rsp_tag_in    = Output(UInt(srcTagWidth.W))
    val mem_rsp_ready_in  = Input(Bool())

    // Downstream (destination) interface
    val mem_req_valid_out  = Output(Bool())
    val mem_req_addr_out   = Output(UInt(dstAddrWidth.W))
    val mem_req_rw_out     = Output(Bool())
    val mem_req_byteen_out = Output(UInt(dstDataSize.W))
    val mem_req_data_out   = Output(UInt(dstDataWidth.W))
    val mem_req_tag_out    = Output(UInt(dstTagWidth.W))
    val mem_req_ready_out  = Input(Bool())

    val mem_rsp_valid_out  = Input(Bool())
    val mem_rsp_data_out   = Input(UInt(dstDataWidth.W))
    val mem_rsp_tag_out    = Input(UInt(dstTagWidth.W))
    val mem_rsp_ready_out  = Output(Bool())
  })

  // Internal wires before output buffers
  val reqValidOutW  = Wire(Bool())
  val reqAddrOutW   = Wire(UInt(dstAddrWidth.W))
  val reqRwOutW     = Wire(Bool())
  val reqByteenOutW = Wire(UInt(dstDataSize.W))
  val reqDataOutW   = Wire(UInt(dstDataWidth.W))
  val reqTagOutW    = Wire(UInt(dstTagWidth.W))
  val reqReadyOutW  = Wire(Bool())

  val rspValidInW  = Wire(Bool())
  val rspDataInW   = Wire(UInt(srcDataWidth.W))
  val rspTagInW    = Wire(UInt(srcTagWidth.W))
  val rspReadyInW  = Wire(Bool())

  // ---- Address width adjustment helper ------------------------------------
  def adaptAddr(src: UInt, srcW: Int, dstW: Int): UInt = {
    if (dstW < srcW)      src(dstW - 1, 0)
    else if (dstW > srcW) src.pad(dstW)
    else                  src
  }

  if (dstLdataw > srcLdataw) {
    // ---- Widen: SRC narrower, DST wider ------------------------------------
    // The D LSBs of the input address select which sub-word within the wider
    // destination word.
    val reqIdx = io.mem_req_addr_in(d - 1, 0)
    val rspIdx = io.mem_rsp_tag_out(d - 1, 0)

    val addrInQual = io.mem_req_addr_in(srcAddrWidth - 1, d)
    reqAddrOutW := adaptAddr(addrInQual, srcAddrWidth - d, dstAddrWidth)

    // Demux byte-enable and data into the wider word
    reqByteenOutW := (io.mem_req_byteen_in << (reqIdx * srcDataSize.U))(dstDataSize - 1, 0)
    reqDataOutW   := (io.mem_req_data_in   << (reqIdx * srcDataWidth.U))(dstDataWidth - 1, 0)

    reqValidOutW := io.mem_req_valid_in
    reqRwOutW    := io.mem_req_rw_in
    reqTagOutW   := Cat(io.mem_req_tag_in, reqIdx).pad(dstTagWidth)
    io.mem_req_ready_in := reqReadyOutW

    // Mux the response sub-word
    val rspDataVec = io.mem_rsp_data_out.asTypeOf(Vec(p, UInt(srcDataWidth.W)))
    rspValidInW := io.mem_rsp_valid_out
    rspDataInW  := rspDataVec(rspIdx)
    rspTagInW   := io.mem_rsp_tag_out(dstTagWidth - 1, d).head(srcTagWidth)
    io.mem_rsp_ready_out := rspReadyInW

  } else if (dstLdataw < srcLdataw) {
    // ---- Narrow: SRC wider, DST narrower -----------------------------------
    // Issue P sub-requests sequentially.  Accumulate P responses before
    // presenting the combined result back upstream.
    val reqCtr = RegInit(0.U(d.W))
    val rspCtr = RegInit(0.U(d.W))

    val reqDataVec   = io.mem_req_data_in.asTypeOf(Vec(p, UInt(dstDataWidth.W)))
    val reqByteenVec = io.mem_req_byteen_in.asTypeOf(Vec(p, UInt(dstDataSize.W)))

    val reqOutFire = reqValidOutW && io.mem_req_ready_out
    val rspInFire  = io.mem_rsp_valid_out && io.mem_rsp_ready_out

    // Accumulated response store
    val rspStore = RegInit(VecInit(Seq.fill(p)(0.U(dstDataWidth.W))))
    when (rspInFire) {
      rspStore(rspCtr) := io.mem_rsp_data_out
    }

    when (reqOutFire) { reqCtr := reqCtr + 1.U }
    when (rspInFire)  { rspCtr := rspCtr + 1.U }

    val addrInQual = Cat(io.mem_req_addr_in, reqCtr)
    reqAddrOutW   := adaptAddr(addrInQual, srcAddrWidth + d, dstAddrWidth)
    reqValidOutW  := io.mem_req_valid_in
    reqRwOutW     := io.mem_req_rw_in
    reqByteenOutW := reqByteenVec(reqCtr)
    reqDataOutW   := reqDataVec(reqCtr)
    reqTagOutW    := io.mem_req_tag_in.pad(dstTagWidth)
    // Only signal ready upstream when all P sub-requests have been sent
    io.mem_req_ready_in := reqReadyOutW && (reqCtr === (p - 1).U)

    // Only signal valid upstream when all P responses received
    rspValidInW := io.mem_rsp_valid_out && (rspCtr === (p - 1).U)
    rspDataInW  := rspStore.asUInt
    rspTagInW   := io.mem_rsp_tag_out.head(srcTagWidth)
    io.mem_rsp_ready_out := rspReadyInW

  } else {
    // ---- Same width: pass-through -----------------------------------------
    reqAddrOutW   := adaptAddr(io.mem_req_addr_in, srcAddrWidth, dstAddrWidth)
    reqValidOutW  := io.mem_req_valid_in
    reqRwOutW     := io.mem_req_rw_in
    reqByteenOutW := io.mem_req_byteen_in
    reqDataOutW   := io.mem_req_data_in
    reqTagOutW    := io.mem_req_tag_in.pad(dstTagWidth)
    io.mem_req_ready_in := reqReadyOutW

    rspValidInW := io.mem_rsp_valid_out
    rspDataInW  := io.mem_rsp_data_out
    rspTagInW   := io.mem_rsp_tag_out.head(srcTagWidth)
    io.mem_rsp_ready_out := rspReadyInW
  }

  // ---- Output buffers (1-entry skid or pass-through) ----------------------
  // Request output buffer
  if (reqOutBuf != 0) {
    val q = Module(new Queue(
      new Bundle {
        val rw     = Bool()
        val byteen = UInt(dstDataSize.W)
        val addr   = UInt(dstAddrWidth.W)
        val data   = UInt(dstDataWidth.W)
        val tag    = UInt(dstTagWidth.W)
      }, 1, pipe = true))
    q.io.enq.valid       := reqValidOutW
    q.io.enq.bits.rw     := reqRwOutW
    q.io.enq.bits.byteen := reqByteenOutW
    q.io.enq.bits.addr   := reqAddrOutW
    q.io.enq.bits.data   := reqDataOutW
    q.io.enq.bits.tag    := reqTagOutW
    reqReadyOutW             := q.io.enq.ready
    io.mem_req_valid_out     := q.io.deq.valid
    io.mem_req_rw_out        := q.io.deq.bits.rw
    io.mem_req_byteen_out    := q.io.deq.bits.byteen
    io.mem_req_addr_out      := q.io.deq.bits.addr
    io.mem_req_data_out      := q.io.deq.bits.data
    io.mem_req_tag_out       := q.io.deq.bits.tag
    q.io.deq.ready           := io.mem_req_ready_out
  } else {
    io.mem_req_valid_out  := reqValidOutW
    io.mem_req_rw_out     := reqRwOutW
    io.mem_req_byteen_out := reqByteenOutW
    io.mem_req_addr_out   := reqAddrOutW
    io.mem_req_data_out   := reqDataOutW
    io.mem_req_tag_out    := reqTagOutW
    reqReadyOutW          := io.mem_req_ready_out
  }

  // Response input buffer
  if (rspOutBuf != 0) {
    val q = Module(new Queue(
      new Bundle {
        val data = UInt(srcDataWidth.W)
        val tag  = UInt(srcTagWidth.W)
      }, 1, pipe = true))
    q.io.enq.valid      := rspValidInW
    q.io.enq.bits.data  := rspDataInW
    q.io.enq.bits.tag   := rspTagInW
    rspReadyInW            := q.io.enq.ready
    io.mem_rsp_valid_in    := q.io.deq.valid
    io.mem_rsp_data_in     := q.io.deq.bits.data
    io.mem_rsp_tag_in      := q.io.deq.bits.tag
    q.io.deq.ready         := io.mem_rsp_ready_in
  } else {
    io.mem_rsp_valid_in := rspValidInW
    io.mem_rsp_data_in  := rspDataInW
    io.mem_rsp_tag_in   := rspTagInW
    rspReadyInW         := io.mem_rsp_ready_in
  }
}

// ============================================================================
// VX_mem_bank_adapter
//
// Fan-out / arbitrate multiple input memory ports to multiple output banks.
// Uses a crossbar (VX_stream_xbar) for requests and another for responses.
// Bank selection can be interleaved or non-interleaved.  An optional tag
// buffer stores per-port read tags when the combined tag would exceed the
// output tag width.
//
// This Chisel translation keeps the structural intent and replaces the
// internal VX_stream_xbar calls with Chisel Queue + Mux logic for a
// single-bank, single-port baseline (the most common Vortex configuration).
// For NUM_PORTS_IN > 1 or NUM_BANKS_OUT > 1 the xbar is expressed as a
// round-robin arbiter using chisel3.util.RRArbiter / Demux.
//
// Parameters
//   dataWidth     – data width (bits)
//   addrWidthIn   – word-addressable input address width
//   addrWidthOut  – byte-addressable output address width
//   tagWidthIn    – input tag width
//   tagWidthOut   – output tag width
//   numPortsIn    – number of input ports
//   numBanksOut   – number of output banks
//   interleave    – bank selection: 0 = top-bit, 1 = low-bit interleaved
//   tagBufferSize – depth of per-port read tag buffers
// ============================================================================
class VXMemBankAdapterIO(
  val dataWidth    : Int,
  val addrWidthIn  : Int,
  val addrWidthOut : Int,
  val tagWidthIn   : Int,
  val tagWidthOut  : Int,
  val numPortsIn   : Int,
  val numBanksOut  : Int
) extends Bundle {
  val dataSize = dataWidth / 8

  // Input ports (flattened; index with explicit slicing in the module)
  val mem_req_valid_in  = Input(Vec(numPortsIn, Bool()))
  val mem_req_rw_in     = Input(Vec(numPortsIn, Bool()))
  val mem_req_byteen_in = Input(Vec(numPortsIn, UInt(dataSize.W)))
  val mem_req_addr_in   = Input(Vec(numPortsIn, UInt(addrWidthIn.W)))
  val mem_req_data_in   = Input(Vec(numPortsIn, UInt(dataWidth.W)))
  val mem_req_tag_in    = Input(Vec(numPortsIn, UInt(tagWidthIn.W)))
  val mem_req_ready_in  = Output(Vec(numPortsIn, Bool()))

  val mem_rsp_valid_in  = Output(Vec(numPortsIn, Bool()))
  val mem_rsp_data_in   = Output(Vec(numPortsIn, UInt(dataWidth.W)))
  val mem_rsp_tag_in    = Output(Vec(numPortsIn, UInt(tagWidthIn.W)))
  val mem_rsp_ready_in  = Input(Vec(numPortsIn, Bool()))

  // Output banks
  val mem_req_valid_out  = Output(Vec(numBanksOut, Bool()))
  val mem_req_rw_out     = Output(Vec(numBanksOut, Bool()))
  val mem_req_byteen_out = Output(Vec(numBanksOut, UInt(dataSize.W)))
  val mem_req_addr_out   = Output(Vec(numBanksOut, UInt(addrWidthOut.W)))
  val mem_req_data_out   = Output(Vec(numBanksOut, UInt(dataWidth.W)))
  val mem_req_tag_out    = Output(Vec(numBanksOut, UInt(tagWidthOut.W)))
  val mem_req_ready_out  = Input(Vec(numBanksOut, Bool()))

  val mem_rsp_valid_out  = Input(Vec(numBanksOut, Bool()))
  val mem_rsp_data_out   = Input(Vec(numBanksOut, UInt(dataWidth.W)))
  val mem_rsp_tag_out    = Input(Vec(numBanksOut, UInt(tagWidthOut.W)))
  val mem_rsp_ready_out  = Output(Vec(numBanksOut, Bool()))
}

class VXMemBankAdapter(
  val dataWidth    : Int     = 512,
  val addrWidthIn  : Int     = 26,
  val addrWidthOut : Int     = 32,
  val tagWidthIn   : Int     = 8,
  val tagWidthOut  : Int     = 8,
  val numPortsIn   : Int     = 1,
  val numBanksOut  : Int     = 1,
  val interleave   : Boolean = false,
  val tagBufferSize: Int     = 32,
  val arbiter      : String  = "R",
  val reqOutBuf    : Int     = 1,
  val rspOutBuf    : Int     = 1
) extends Module {
  val dataSize        = dataWidth / 8
  val bankSelBits     = log2Ceil(math.max(numBanksOut, 2))  // avoid log2(1)=0 issues
  val bankSelWidth    = if (numBanksOut > 1) bankSelBits else 1
  val dstAddrWidth    = addrWidthOut + (if (numBanksOut > 1) bankSelBits else 0)
  val bankAddrWidth   = dstAddrWidth - (if (numBanksOut > 1) bankSelBits else 0)
  val numPortsInBits  = log2Ceil(math.max(numPortsIn, 2))
  val tagBufAddrw     = log2Ceil(tagBufferSize)
  val neededTagWidth  = tagWidthIn + (if (numPortsIn > 1) numPortsInBits else 0)
  val readTagWidth    = if (neededTagWidth > tagWidthOut) tagBufAddrw else tagWidthIn
  val xbarTagWidth    = math.max(readTagWidth, tagWidthIn) // WRITE_TAG_WIDTH = tagWidthIn

  val io = IO(new VXMemBankAdapterIO(
    dataWidth, addrWidthIn, addrWidthOut,
    tagWidthIn, tagWidthOut, numPortsIn, numBanksOut))

  // ---- Bank selection -------------------------------------------------------
  // For each input port compute which bank the request targets and the
  // within-bank address.
  val reqBankSel  = Wire(Vec(numPortsIn, UInt(bankSelWidth.W)))
  val reqBankAddr = Wire(Vec(numPortsIn, UInt(bankAddrWidth.W)))

  for (i <- 0 until numPortsIn) {
    val addrExt = io.mem_req_addr_in(i).pad(dstAddrWidth)
    if (numBanksOut > 1) {
      if (interleave) {
        reqBankSel(i)  := addrExt(bankSelBits - 1, 0)
        reqBankAddr(i) := addrExt(bankSelBits + bankAddrWidth - 1, bankSelBits)
      } else {
        reqBankSel(i)  := addrExt(bankAddrWidth + bankSelBits - 1, bankAddrWidth)
        reqBankAddr(i) := addrExt(bankAddrWidth - 1, 0)
      }
    } else {
      reqBankSel(i)  := 0.U
      reqBankAddr(i) := addrExt(bankAddrWidth - 1, 0)
    }
  }

  // ---- Tag handling ---------------------------------------------------------
  // For reads: if the combined tag (input tag + port-select bits) exceeds the
  // output tag width we must buffer the original tag and use the buffer
  // address as the transaction tag.
  val rdReqTagReady = Wire(Vec(numPortsIn, Bool()))
  val rdReqTag      = Wire(Vec(numPortsIn, UInt(readTagWidth.W)))
  val rdRspTagIn    = Wire(Vec(numPortsIn, UInt(readTagWidth.W)))

  for (i <- 0 until numPortsIn) {
    if (neededTagWidth > tagWidthOut) {
      // Tag buffer: store original tag, use buffer write address as tag
      val tbuf = Module(new VxIndexBuffer(tagWidthIn, tagBufferSize))
      tbuf.io.acquireEn  := io.mem_req_valid_in(i) && !io.mem_req_rw_in(i) && io.mem_req_ready_in(i)
      tbuf.io.writeData  := io.mem_req_tag_in(i)
      tbuf.io.readAddr   := rdRspTagIn(i)
      tbuf.io.releaseEn  := io.mem_rsp_valid_in(i) && io.mem_rsp_ready_in(i)
      rdReqTagReady(i)   := !tbuf.io.full
      rdReqTag(i)        := tbuf.io.writeAddr
      io.mem_rsp_tag_in(i) := tbuf.io.readData
    } else {
      rdReqTagReady(i)     := true.B
      rdReqTag(i)          := io.mem_req_tag_in(i)
      io.mem_rsp_tag_in(i) := rdRspTagIn(i)
    }
  }

  // ---- Request crossbar (per-bank RR arbiter, per-port mux for ready) ------
  // Build per-port helper signals (tag readiness, tag value, bank addr)
  class ReqPayload extends Bundle {
    val rw     = Bool()
    val addr   = UInt(bankAddrWidth.W)
    val byteen = UInt(dataSize.W)
    val data   = UInt(dataWidth.W)
    val tag    = UInt(xbarTagWidth.W)
  }

  val portTagReady = Wire(Vec(numPortsIn, Bool()))
  val portTagValue = Wire(Vec(numPortsIn, UInt(xbarTagWidth.W)))
  for (i <- 0 until numPortsIn) {
    portTagReady(i) := io.mem_req_rw_in(i) || rdReqTagReady(i)
    portTagValue(i) := Mux(io.mem_req_rw_in(i),
      io.mem_req_tag_in(i).pad(xbarTagWidth),
      rdReqTag(i).pad(xbarTagWidth))
  }

  // Per-bank arbiters: collect per-input ready signals as a 2D matrix
  // arbInReady(b)(i) = arbiter b's ready for input i
  val arbInReady = Wire(Vec(numBanksOut, Vec(numPortsIn, Bool())))

  for (b <- 0 until numBanksOut) {
    val arb = Module(new RRArbiter(new ReqPayload(), numPortsIn))

    for (i <- 0 until numPortsIn) {
      val sel = if (numBanksOut > 1) (reqBankSel(i) === b.U) else true.B
      arb.io.in(i).valid        := io.mem_req_valid_in(i) && portTagReady(i) && sel
      arb.io.in(i).bits.rw     := io.mem_req_rw_in(i)
      arb.io.in(i).bits.addr   := reqBankAddr(i)
      arb.io.in(i).bits.byteen := io.mem_req_byteen_in(i)
      arb.io.in(i).bits.data   := io.mem_req_data_in(i)
      arb.io.in(i).bits.tag    := portTagValue(i)
      arbInReady(b)(i)          := arb.io.in(i).ready
    }

    val chosen = arb.io.chosen
    arb.io.out.ready := io.mem_req_ready_out(b)

    io.mem_req_valid_out(b)  := arb.io.out.valid
    io.mem_req_rw_out(b)     := arb.io.out.bits.rw
    io.mem_req_addr_out(b)   := arb.io.out.bits.addr.pad(addrWidthOut)
    io.mem_req_byteen_out(b) := arb.io.out.bits.byteen
    io.mem_req_data_out(b)   := arb.io.out.bits.data

    if (numPortsIn > 1) {
      io.mem_req_tag_out(b) := Cat(arb.io.out.bits.tag, chosen).pad(tagWidthOut)
    } else {
      io.mem_req_tag_out(b) := arb.io.out.bits.tag.pad(tagWidthOut)
    }
  }

  // Per-port ready: select the ready signal from the bank this port targets
  for (i <- 0 until numPortsIn) {
    val bankReadyVec = VecInit((0 until numBanksOut).map(b => arbInReady(b)(i)))
    val selectedReady =
      if (numBanksOut > 1) bankReadyVec(reqBankSel(i))
      else                 bankReadyVec(0)
    io.mem_req_ready_in(i) := selectedReady && portTagReady(i)
  }

  // ---- Response crossbar (per-port RR arbiter from banks) -------------------
  // rsp_ready_out is driven per bank; use OR-reduction when multiple ports
  // claim the same bank (only the selected port drives ready).
  val rspReadyContrib = Wire(Vec(numBanksOut, Vec(numPortsIn, Bool())))
  for (b <- 0 until numBanksOut) {
    for (i <- 0 until numPortsIn) { rspReadyContrib(b)(i) := false.B }
  }

  for (i <- 0 until numPortsIn) {
    val arb = Module(new RRArbiter(new Bundle {
      val data  = UInt(dataWidth.W)
      val rdTag = UInt(readTagWidth.W)
    }, numBanksOut))

    for (b <- 0 until numBanksOut) {
      val portSel =
        if (numPortsIn > 1) io.mem_rsp_tag_out(b)(numPortsInBits - 1, 0) === i.U
        else true.B
      arb.io.in(b).valid       := io.mem_rsp_valid_out(b) && portSel
      arb.io.in(b).bits.data   := io.mem_rsp_data_out(b)
      arb.io.in(b).bits.rdTag  :=
        io.mem_rsp_tag_out(b)(numPortsInBits + readTagWidth - 1, numPortsInBits)
      rspReadyContrib(b)(i)    := arb.io.in(b).ready
    }

    arb.io.out.ready       := io.mem_rsp_ready_in(i)
    io.mem_rsp_valid_in(i) := arb.io.out.valid
    io.mem_rsp_data_in(i)  := arb.io.out.bits.data
    rdRspTagIn(i)          := arb.io.out.bits.rdTag
  }

  // Combine per-port ready contributions for each bank
  for (b <- 0 until numBanksOut) {
    io.mem_rsp_ready_out(b) := rspReadyContrib(b).asUInt.orR
  }
}

// ============================================================================
// VX_axi_adapter
//
// Bridges Vortex's internal memory request/response protocol to AXI4.
// Structurally identical to VX_mem_bank_adapter but the output side is an
// AXI4 master port instead of a generic memory bank.
//
// Key differences from mem_bank_adapter:
//   - AXI address is byte-addressable; input is block-addressable.
//   - AW and W channels must both complete before the transaction is done;
//     VX_axi_write_ack handles this.
//   - Write responses (B channel) are accepted and ignored.
//   - Read responses come from the R channel (rid carries the tag).
//
// Parameters
//   dataWidth     – data width (bits)
//   addrWidthIn   – word-addressable input address width
//   addrWidthOut  – AXI byte-address width
//   tagWidthIn    – Vortex tag width
//   tagWidthOut   – AXI ID width
//   numPortsIn    – Vortex ports
//   numBanksOut   – number of AXI master ports
//   interleave    – bank interleave mode
//   tagBufferSize – read tag buffer depth
// ============================================================================

// AXI master port bundle for one bank
class AXIMasterPort(
  val dataWidth   : Int,
  val addrWidth   : Int,
  val tagWidth    : Int
) extends Bundle {
  val dataSize = dataWidth / 8
  // Write address channel
  val awvalid  = Output(Bool())
  val awready  = Input(Bool())
  val awaddr   = Output(UInt(addrWidth.W))
  val awid     = Output(UInt(tagWidth.W))
  val awlen    = Output(UInt(8.W))
  val awsize   = Output(UInt(3.W))
  val awburst  = Output(UInt(2.W))
  val awlock   = Output(UInt(2.W))
  val awcache  = Output(UInt(4.W))
  val awprot   = Output(UInt(3.W))
  val awqos    = Output(UInt(4.W))
  val awregion = Output(UInt(4.W))
  // Write data channel
  val wvalid   = Output(Bool())
  val wready   = Input(Bool())
  val wdata    = Output(UInt(dataWidth.W))
  val wstrb    = Output(UInt(dataSize.W))
  val wlast    = Output(Bool())
  // Write response channel
  val bvalid   = Input(Bool())
  val bready   = Output(Bool())
  val bid      = Input(UInt(tagWidth.W))
  val bresp    = Input(UInt(2.W))
  // Read address channel
  val arvalid  = Output(Bool())
  val arready  = Input(Bool())
  val araddr   = Output(UInt(addrWidth.W))
  val arid     = Output(UInt(tagWidth.W))
  val arlen    = Output(UInt(8.W))
  val arsize   = Output(UInt(3.W))
  val arburst  = Output(UInt(2.W))
  val arlock   = Output(UInt(2.W))
  val arcache  = Output(UInt(4.W))
  val arprot   = Output(UInt(3.W))
  val arqos    = Output(UInt(4.W))
  val arregion = Output(UInt(4.W))
  // Read response channel
  val rvalid   = Input(Bool())
  val rready   = Output(Bool())
  val rdata    = Input(UInt(dataWidth.W))
  val rlast    = Input(Bool())
  val rid      = Input(UInt(tagWidth.W))
  val rresp    = Input(UInt(2.W))
}

class VXAXIAdapter(
  val dataWidth    : Int     = 512,
  val addrWidthIn  : Int     = 26,
  val addrWidthOut : Int     = 32,
  val tagWidthIn   : Int     = 8,
  val tagWidthOut  : Int     = 8,
  val numPortsIn   : Int     = 1,
  val numBanksOut  : Int     = 1,
  val interleave   : Boolean = false,
  val tagBufferSize: Int     = 16,
  val arbiter      : String  = "R",
  val reqOutBuf    : Int     = 0,
  val rspOutBuf    : Int     = 0
) extends Module {
  val dataSize       = dataWidth / 8
  val log2DataSize   = log2Ceil(dataSize)
  val bankSelBits    = log2Ceil(math.max(numBanksOut, 2))
  val bankSelWidth   = if (numBanksOut > 1) bankSelBits else 1
  // Convert byte-addressable output to block-addressable input space
  val dstAddrWidth   = (addrWidthOut - log2DataSize) + (if (numBanksOut > 1) bankSelBits else 0)
  val bankAddrWidth  = dstAddrWidth - (if (numBanksOut > 1) bankSelBits else 0)
  val numPortsInBits = log2Ceil(math.max(numPortsIn, 2))
  val tagBufAddrw    = log2Ceil(tagBufferSize)
  val neededTagWidth = tagWidthIn + (if (numPortsIn > 1) numPortsInBits else 0)
  val readTagWidth   = if (neededTagWidth > tagWidthOut) tagBufAddrw else tagWidthIn
  val readFullTagWidth = readTagWidth + (if (numPortsIn > 1) numPortsInBits else 0)
  val writeTagWidth  = math.min(tagWidthIn, tagWidthOut)
  val xbarTagWidth   = math.max(readTagWidth, writeTagWidth)

  val io = IO(new Bundle {
    // Vortex interface
    val mem_req_valid  = Input(Vec(numPortsIn, Bool()))
    val mem_req_rw     = Input(Vec(numPortsIn, Bool()))
    val mem_req_byteen = Input(Vec(numPortsIn, UInt(dataSize.W)))
    val mem_req_addr   = Input(Vec(numPortsIn, UInt(addrWidthIn.W)))
    val mem_req_data   = Input(Vec(numPortsIn, UInt(dataWidth.W)))
    val mem_req_tag    = Input(Vec(numPortsIn, UInt(tagWidthIn.W)))
    val mem_req_ready  = Output(Vec(numPortsIn, Bool()))

    val mem_rsp_valid  = Output(Vec(numPortsIn, Bool()))
    val mem_rsp_data   = Output(Vec(numPortsIn, UInt(dataWidth.W)))
    val mem_rsp_tag    = Output(Vec(numPortsIn, UInt(tagWidthIn.W)))
    val mem_rsp_ready  = Input(Vec(numPortsIn, Bool()))

    // AXI master ports
    val axi = Vec(numBanksOut, new AXIMasterPort(dataWidth, addrWidthOut, tagWidthOut))
  })

  // ---- Bank / address selection -------------------------------------------
  val reqBankSel  = Wire(Vec(numPortsIn, UInt(bankSelWidth.W)))
  val reqBankAddr = Wire(Vec(numPortsIn, UInt(bankAddrWidth.W)))

  for (i <- 0 until numPortsIn) {
    val addrExt = io.mem_req_addr(i).pad(dstAddrWidth)
    if (numBanksOut > 1) {
      if (interleave) {
        reqBankSel(i)  := addrExt(bankSelBits - 1, 0)
        reqBankAddr(i) := addrExt(bankSelBits + bankAddrWidth - 1, bankSelBits)
      } else {
        reqBankSel(i)  := addrExt(bankAddrWidth + bankSelBits - 1, bankAddrWidth)
        reqBankAddr(i) := addrExt(bankAddrWidth - 1, 0)
      }
    } else {
      reqBankSel(i)  := 0.U
      reqBankAddr(i) := addrExt(bankAddrWidth - 1, 0)
    }
  }

  // ---- Tag handling (read requests) ---------------------------------------
  val rdReqTagReady = Wire(Vec(numPortsIn, Bool()))
  val rdReqTag      = Wire(Vec(numPortsIn, UInt(readTagWidth.W)))
  val rdRspTag      = Wire(Vec(numPortsIn, UInt(readTagWidth.W)))

  for (i <- 0 until numPortsIn) {
    if (neededTagWidth > tagWidthOut) {
      val tbuf = Module(new VxIndexBuffer(tagWidthIn, tagBufferSize))
      tbuf.io.acquireEn := io.mem_req_valid(i) && !io.mem_req_rw(i) && io.mem_req_ready(i)
      tbuf.io.writeData := io.mem_req_tag(i)
      tbuf.io.readAddr  := rdRspTag(i)
      tbuf.io.releaseEn := io.mem_rsp_valid(i) && io.mem_rsp_ready(i)
      rdReqTagReady(i)  := !tbuf.io.full
      rdReqTag(i)       := tbuf.io.writeAddr
      io.mem_rsp_tag(i) := tbuf.io.readData
    } else {
      rdReqTagReady(i)  := true.B
      rdReqTag(i)       := io.mem_req_tag(i)
      io.mem_rsp_tag(i) := rdRspTag(i)
    }
  }

  // ---- Per-bank AXI request generation ------------------------------------
  for (b <- 0 until numBanksOut) {
    val arb = Module(new RRArbiter(new Bundle {
      val rw     = Bool()
      val addr   = UInt(bankAddrWidth.W)
      val byteen = UInt(dataSize.W)
      val data   = UInt(dataWidth.W)
      val tag    = UInt(xbarTagWidth.W)
      val portId = UInt(numPortsInBits.W)
    }, numPortsIn))

    for (i <- 0 until numPortsIn) {
      val tagReady = io.mem_req_rw(i) || rdReqTagReady(i)
      val tagValue = Mux(io.mem_req_rw(i),
        io.mem_req_tag(i).pad(xbarTagWidth),
        rdReqTag(i).pad(xbarTagWidth))
      val sel = if (numBanksOut > 1) (reqBankSel(i) === b.U) else true.B
      arb.io.in(i).valid        := io.mem_req_valid(i) && tagReady && sel
      arb.io.in(i).bits.rw     := io.mem_req_rw(i)
      arb.io.in(i).bits.addr   := reqBankAddr(i)
      arb.io.in(i).bits.byteen := io.mem_req_byteen(i)
      arb.io.in(i).bits.data   := io.mem_req_data(i)
      arb.io.in(i).bits.tag    := tagValue
      arb.io.in(i).bits.portId := i.U
      io.mem_req_ready(i)      := arb.io.in(i).ready && tagReady
    }

    val xbarRw     = arb.io.out.bits.rw
    val xbarAddr   = arb.io.out.bits.addr
    val xbarByteen = arb.io.out.bits.byteen
    val xbarData   = arb.io.out.bits.data
    val xbarTag    = arb.io.out.bits.tag
    val xbarSel    = arb.io.out.bits.portId
    val xbarValid  = arb.io.out.valid

    // AXI write-ack handshake helper
    val writeAck = Module(new VXAXIWriteAck)
    writeAck.io.awvalid := io.axi(b).awvalid
    writeAck.io.awready := io.axi(b).awready
    writeAck.io.wvalid  := io.axi(b).wvalid
    writeAck.io.wready  := io.axi(b).wready

    arb.io.out.ready := Mux(xbarRw, writeAck.io.tx_rdy, io.axi(b).arready)

    // ---- AXI write address channel ----------------------------------------
    io.axi(b).awvalid  := xbarValid && xbarRw && !writeAck.io.aw_ack
    io.axi(b).awid     := xbarTag.pad(tagWidthOut)
    io.axi(b).awlen    := 0.U
    io.axi(b).awsize   := log2DataSize.U
    io.axi(b).awburst  := 0.U
    io.axi(b).awlock   := 0.U
    io.axi(b).awcache  := 0.U
    io.axi(b).awprot   := 0.U
    io.axi(b).awqos    := 0.U
    io.axi(b).awregion := 0.U

    // Convert block address to byte address
    if (interleave) {
      io.axi(b).awaddr := ((xbarAddr.pad(addrWidthOut) << (bankSelBits + log2DataSize).U) |
                           (b.U.pad(addrWidthOut) << log2DataSize.U))(addrWidthOut - 1, 0)
    } else {
      io.axi(b).awaddr := ((xbarAddr.pad(addrWidthOut) << log2DataSize.U) |
                           (b.U.pad(addrWidthOut) << (bankAddrWidth + log2DataSize).U))(addrWidthOut - 1, 0)
    }

    // ---- AXI write data channel -------------------------------------------
    io.axi(b).wvalid := xbarValid && xbarRw && !writeAck.io.w_ack
    io.axi(b).wdata  := xbarData
    io.axi(b).wstrb  := xbarByteen
    io.axi(b).wlast  := true.B

    // ---- AXI write response channel (accept and ignore) -------------------
    io.axi(b).bready := true.B

    // ---- AXI read address channel -----------------------------------------
    val xbarTagROut = if (numPortsIn > 1)
      Cat(xbarTag, xbarSel).pad(readFullTagWidth)
    else
      xbarTag.pad(readTagWidth)

    io.axi(b).arvalid  := xbarValid && !xbarRw
    io.axi(b).arid     := xbarTagROut.pad(tagWidthOut)
    io.axi(b).arlen    := 0.U
    io.axi(b).arsize   := log2DataSize.U
    io.axi(b).arburst  := 0.U
    io.axi(b).arlock   := 0.U
    io.axi(b).arcache  := 0.U
    io.axi(b).arprot   := 0.U
    io.axi(b).arqos    := 0.U
    io.axi(b).arregion := 0.U

    if (interleave) {
      io.axi(b).araddr := ((xbarAddr.pad(addrWidthOut) << (bankSelBits + log2DataSize).U) |
                           (b.U.pad(addrWidthOut) << log2DataSize.U))(addrWidthOut - 1, 0)
    } else {
      io.axi(b).araddr := ((xbarAddr.pad(addrWidthOut) << log2DataSize.U) |
                           (b.U.pad(addrWidthOut) << (bankAddrWidth + log2DataSize).U))(addrWidthOut - 1, 0)
    }
  }

  // ---- AXI read response → Vortex response ---------------------------------
  // Collect rready contributions per bank per port, then OR them.
  val rspRreadyContrib = Wire(Vec(numBanksOut, Vec(numPortsIn, Bool())))
  for (b <- 0 until numBanksOut) {
    for (i <- 0 until numPortsIn) { rspRreadyContrib(b)(i) := false.B }
  }

  for (i <- 0 until numPortsIn) {
    val arb = Module(new RRArbiter(new Bundle {
      val data  = UInt(dataWidth.W)
      val rdTag = UInt(readTagWidth.W)
    }, numBanksOut))

    for (b <- 0 until numBanksOut) {
      val portSel =
        if (numPortsIn > 1) io.axi(b).rid(numPortsInBits - 1, 0) === i.U
        else true.B
      arb.io.in(b).valid      := io.axi(b).rvalid && portSel
      arb.io.in(b).bits.data  := io.axi(b).rdata
      arb.io.in(b).bits.rdTag := io.axi(b).rid(numPortsInBits + readTagWidth - 1, numPortsInBits)
      rspRreadyContrib(b)(i)  := arb.io.in(b).ready
    }

    arb.io.out.ready      := io.mem_rsp_ready(i)
    io.mem_rsp_valid(i)   := arb.io.out.valid
    io.mem_rsp_data(i)    := arb.io.out.bits.data
    rdRspTag(i)           := arb.io.out.bits.rdTag
  }

  for (b <- 0 until numBanksOut) {
    io.axi(b).rready := rspRreadyContrib(b).asUInt.orR
  }
}

// ============================================================================
// VX_mem_coalescer
//
// Coalesces multiple narrow word-width requests from CORE_REQS lanes into
// fewer wide line-width requests.  Requests that share the same cache-line
// address are merged into a single output request.  The module operates as a
// two-state machine (WAIT / SEND) and uses an index buffer (ibuf) to track
// in-flight read tags so that responses can be un-merged back to the original
// requestors.
//
// Translation notes
//   - The VX_pipe_register and VX_index_buffer instances are replaced by
//     Chisel RegNext / SyncReadMem / VxIndexBuffer respectively.
//   - The VX_priority_encoder is replaced by PriorityEncoder.
//   - Parameterised 2-D packed arrays are flattened to Vec(Vec(...)).
//
// Parameters (all counts/widths unless noted)
//   numReqs       – number of input request lanes
//   addrWidth     – per-lane address width
//   flagsWidth    – optional flags width (0 = no flags)
//   dataInSize    – input data size in bytes
//   dataOutSize   – output (coalesced) data size in bytes
//   tagWidth      – input tag width
//   uuidWidth     – UUID sub-field width within tag (0 = no UUID)
//   queueSize     – in-flight request queue depth
// ============================================================================
class VXMemCoalescer(
  val numReqs    : Int = 4,
  val addrWidth  : Int = 32,
  val flagsWidth : Int = 0,
  val dataInSize : Int = 4,
  val dataOutSize: Int = 64,
  val tagWidth   : Int = 8,
  val uuidWidth  : Int = 0,
  val queueSize  : Int = 8
) extends Module {
  val dataInWidth  = dataInSize  * 8
  val dataOutWidth = dataOutSize * 8
  val dataRatio    = dataOutSize / dataInSize
  val dataRatioW   = log2Ceil(dataRatio)
  val outReqs      = numReqs / dataRatio
  val outAddrWidth = addrWidth - dataRatioW
  val queueAddrw   = log2Ceil(queueSize)
  val outTagWidth  = uuidWidth + queueAddrw
  val tagIdWidth   = tagWidth - uuidWidth
  val perfCtrBits  = log2Ceil(numReqs + 1)
  val flagsW       = if (flagsWidth > 0) flagsWidth else 1  // avoid 0-width

  val io = IO(new Bundle {
    val misses = Output(UInt(perfCtrBits.W))

    // Input request
    val in_req_valid  = Input(Bool())
    val in_req_rw     = Input(Bool())
    val in_req_mask   = Input(UInt(numReqs.W))
    val in_req_byteen = Input(Vec(numReqs, UInt(dataInSize.W)))
    val in_req_addr   = Input(Vec(numReqs, UInt(addrWidth.W)))
    val in_req_flags  = Input(Vec(numReqs, UInt(flagsW.W)))
    val in_req_data   = Input(Vec(numReqs, UInt(dataInWidth.W)))
    val in_req_tag    = Input(UInt(tagWidth.W))
    val in_req_ready  = Output(Bool())

    // Input response
    val in_rsp_valid  = Output(Bool())
    val in_rsp_mask   = Output(UInt(numReqs.W))
    val in_rsp_data   = Output(Vec(numReqs, UInt(dataInWidth.W)))
    val in_rsp_tag    = Output(UInt(tagWidth.W))
    val in_rsp_ready  = Input(Bool())

    // Output request
    val out_req_valid  = Output(Bool())
    val out_req_rw     = Output(Bool())
    val out_req_mask   = Output(UInt(outReqs.W))
    val out_req_byteen = Output(Vec(outReqs, UInt(dataOutSize.W)))
    val out_req_addr   = Output(Vec(outReqs, UInt(outAddrWidth.W)))
    val out_req_flags  = Output(Vec(outReqs, UInt(flagsW.W)))
    val out_req_data   = Output(Vec(outReqs, UInt(dataOutWidth.W)))
    val out_req_tag    = Output(UInt(outTagWidth.W))
    val out_req_ready  = Input(Bool())

    // Output response
    val out_rsp_valid  = Input(Bool())
    val out_rsp_mask   = Input(UInt(outReqs.W))
    val out_rsp_data   = Input(Vec(outReqs, UInt(dataOutWidth.W)))
    val out_rsp_tag    = Input(UInt(outTagWidth.W))
    val out_rsp_ready  = Output(Bool())
  })

  // ---- State machine -------------------------------------------------------
  val STATE_WAIT = false.B
  val STATE_SEND = true.B

  // Pipeline registers (VX_pipe_register equivalent)
  val stateR        = RegInit(false.B)   // false=WAIT, true=SEND
  val outReqValidR  = RegInit(false.B)
  val outReqRwR     = RegInit(false.B)
  val outReqMaskR   = RegInit(0.U(outReqs.W))
  val outReqAddrR   = RegInit(VecInit(Seq.fill(outReqs)(0.U(outAddrWidth.W))))
  val outReqFlagsR  = RegInit(VecInit(Seq.fill(outReqs)(0.U(flagsW.W))))
  val outReqByteenR = RegInit(VecInit(Seq.fill(outReqs)(0.U(dataOutSize.W))))
  val outReqDataR   = RegInit(VecInit(Seq.fill(outReqs)(0.U(dataOutWidth.W))))
  val outReqTagR    = RegInit(0.U(outTagWidth.W))
  val reqRemMaskR   = RegInit(((1L << numReqs) - 1).U(numReqs.W))  // init all 1s
  val batchValidR   = RegInit(0.U(outReqs.W))
  val seedAddrR     = RegInit(VecInit(Seq.fill(outReqs)(0.U(outAddrWidth.W))))
  val seedFlagsR    = RegInit(VecInit(Seq.fill(outReqs)(0.U(flagsW.W))))
  val addrMatchesR  = RegInit(0.U(numReqs.W))

  // ---- Index buffer for in-flight read tag storage -------------------------
  // ibuf_data: {tag_id, pmask, offset} for each in-flight request
  val ibufDataWidth = tagIdWidth + numReqs + (numReqs * dataRatioW)
  val ibuf   = Module(new VxIndexBuffer(ibufDataWidth, queueSize))
  val ibufFull  = ibuf.io.full
  val ibufWaddr = ibuf.io.writeAddr
  val ibufRaddr = io.out_rsp_tag(queueAddrw - 1, 0)

  // ---- Address offsets and seed generation --------------------------------
  // in_addr_offset: lower dataRatioW bits of each input address
  val inAddrOffset = VecInit(io.in_req_addr.map(_(dataRatioW - 1, 0)))

  // Per-outReqs seed selection (priority encoder over remaining mask)
  val batchValidN  = Wire(Vec(outReqs, Bool()))
  val seedAddrN    = Wire(Vec(outReqs, UInt(outAddrWidth.W)))
  val seedFlagsN   = Wire(Vec(outReqs, UInt(flagsW.W)))
  val addrMatchesN = Wire(UInt(numReqs.W))

  val addrMatchesNVec = Wire(Vec(numReqs, Bool()))

  for (i <- 0 until outReqs) {
    val batchMask = Wire(UInt(dataRatio.W))
    val batchMaskVec = Wire(Vec(dataRatio, Bool()))
    for (j <- 0 until dataRatio) {
      batchMaskVec(j) := io.in_req_mask(i * dataRatio + j) &&
                         reqRemMaskR(i * dataRatio + j)
    }
    batchMask := batchMaskVec.asUInt

    // Priority encoder: find lowest set bit
    val batchIdx = PriorityEncoder(batchMask)
    batchValidN(i) := batchMask.orR

    val addrBaseVec = VecInit((0 until dataRatio).map { j =>
      io.in_req_addr(dataRatio * i + j)(addrWidth - 1, dataRatioW)
    })
    val flagsVec = VecInit((0 until dataRatio).map { j =>
      io.in_req_flags(dataRatio * i + j)
    })

    seedAddrN(i)  := addrBaseVec(batchIdx)
    seedFlagsN(i) := flagsVec(batchIdx)

    for (j <- 0 until dataRatio) {
      addrMatchesNVec(i * dataRatio + j) := addrBaseVec(j) === seedAddrN(i)
    }
  }
  addrMatchesN := addrMatchesNVec.asUInt

  // ---- Current partial mask -----------------------------------------------
  val currentPmask = io.in_req_mask & addrMatchesR

  // ---- Data merge: byte-level merge of coalesced requests -----------------
  val reqByteenMerged = Wire(Vec(outReqs, UInt(dataOutSize.W)))
  val reqDataMerged   = Wire(Vec(outReqs, UInt(dataOutWidth.W)))

  for (i <- 0 until outReqs) {
    val finalByteen = Wire(UInt(dataOutSize.W))
    val finalData   = Wire(UInt(dataOutWidth.W))

    // Build per-offset accumulated values for each possible offset value
    val offsetByteen = Wire(Vec(dataRatio, UInt(dataInSize.W)))
    val offsetData   = Wire(Vec(dataRatio, UInt(dataInWidth.W)))
    for (pos <- 0 until dataRatio) {
      // OR together byte-enables from all lanes that map to this offset
      val beOr  = Wire(Vec(dataInSize, Bool()))
      val datMx = Wire(Vec(dataInSize, UInt(8.W)))
      for (b <- 0 until dataInSize) {
        beOr(b)  := false.B
        datMx(b) := 0.U
      }
      for (j <- 0 until dataRatio) {
        val globalIdx = i * dataRatio + j
        val active    = currentPmask(globalIdx)
        for (b <- 0 until dataInSize) {
          when (active && io.in_req_byteen(globalIdx)(b) &&
                inAddrOffset(globalIdx) === pos.U) {
            beOr(b)  := true.B
            datMx(b) := io.in_req_data(globalIdx)(b * 8 + 7, b * 8)
          }
        }
      }
      offsetByteen(pos) := beOr.asUInt
      offsetData(pos)   := datMx.asUInt
    }
    finalByteen := offsetByteen.asUInt
    finalData   := offsetData.asUInt

    reqByteenMerged(i) := finalByteen
    reqDataMerged(i)   := finalData
  }

  // ---- State machine combinational logic -----------------------------------
  val isLastBatch = !(io.in_req_mask & ~addrMatchesR & reqRemMaskR).orR
  val outReqFire  = outReqValidR && io.out_req_ready

  val outReqValidN  = Wire(Bool())
  val outReqRwN     = Wire(Bool())
  val outReqMaskN   = Wire(UInt(outReqs.W))
  val outReqAddrN   = Wire(Vec(outReqs, UInt(outAddrWidth.W)))
  val outReqFlagsN  = Wire(Vec(outReqs, UInt(flagsW.W)))
  val outReqByteenN = Wire(Vec(outReqs, UInt(dataOutSize.W)))
  val outReqDataN   = Wire(Vec(outReqs, UInt(dataOutWidth.W)))
  val outReqTagN    = Wire(UInt(outTagWidth.W))
  val reqRemMaskN   = Wire(UInt(numReqs.W))
  val stateN        = Wire(Bool())
  val inReqReadyN   = Wire(Bool())

  // Default: hold current values
  outReqValidN  := outReqValidR
  outReqRwN     := outReqRwR
  outReqMaskN   := outReqMaskR
  outReqAddrN   := outReqAddrR
  outReqFlagsN  := outReqFlagsR
  outReqByteenN := outReqByteenR
  outReqDataN   := outReqDataR
  outReqTagN    := outReqTagR
  reqRemMaskN   := reqRemMaskR
  stateN        := stateR
  inReqReadyN   := false.B

  when (!stateR) { // STATE_WAIT
    when (outReqFire) { outReqValidN := false.B }
    when (io.in_req_valid && !outReqValidN && !ibufFull) {
      stateN := true.B  // -> STATE_SEND
    }
  } .otherwise { // STATE_SEND
    stateN        := false.B  // -> STATE_WAIT
    outReqValidN  := true.B
    outReqMaskN   := batchValidR.asUInt
    outReqRwN     := io.in_req_rw
    for (i <- 0 until outReqs) {
      outReqAddrN(i)   := seedAddrR(i)
      outReqFlagsN(i)  := seedFlagsR(i)
      outReqByteenN(i) := reqByteenMerged(i)
      outReqDataN(i)   := reqDataMerged(i)
    }
    val uuidBits = if (uuidWidth > 0) io.in_req_tag(tagWidth - 1, tagWidth - uuidWidth) else 0.U
    outReqTagN  := Cat(uuidBits, ibufWaddr)
    reqRemMaskN := Mux(isLastBatch, ((1L << numReqs) - 1).U(numReqs.W), reqRemMaskR & ~currentPmask)
    inReqReadyN := isLastBatch
  }

  // ---- Pipeline register update (VX_pipe_register equivalent) -------------
  stateR       := stateN
  outReqValidR := outReqValidN
  outReqRwR    := outReqRwN
  outReqMaskR  := outReqMaskN
  for (i <- 0 until outReqs) {
    outReqAddrR(i)   := outReqAddrN(i)
    outReqFlagsR(i)  := outReqFlagsN(i)
    outReqByteenR(i) := outReqByteenN(i)
    outReqDataR(i)   := outReqDataN(i)
  }
  outReqTagR   := outReqTagN
  reqRemMaskR  := reqRemMaskN
  batchValidR  := batchValidN.asUInt
  for (i <- 0 until outReqs) {
    seedAddrR(i)  := seedAddrN(i)
    seedFlagsR(i) := seedFlagsN(i)
  }
  addrMatchesR := addrMatchesN

  // ---- Index buffer push/pop ----------------------------------------------
  val reqSent    = stateR  // pipeline stage: true when we just formed a request
  val outRspFire = io.out_rsp_valid && io.out_rsp_ready

  // Response end-of-packet tracking (async read needed for combinational eop logic)
  val rspRemMask  = Mem(queueSize, UInt(outReqs.W))
  val rspRemMaskN = rspRemMask.read(ibufRaddr) & ~io.out_rsp_mask
  val outRspEop   = !rspRemMaskN.orR

  when (outRspFire) {
    rspRemMask.write(ibufRaddr, rspRemMaskN)
  }

  val ibufPush = reqSent && !io.in_req_rw
  val ibufPop  = outRspFire && outRspEop

  // ibuf data packing: {tagId, pmask, offsets}
  val ibufDinTag    = io.in_req_tag(tagIdWidth - 1, 0)
  val ibufDinPmask  = currentPmask
  val ibufDinOffset = VecInit((0 until numReqs).map(k => inAddrOffset(k))).asUInt
  val ibufDin       = Cat(ibufDinTag, ibufDinPmask, ibufDinOffset)

  ibuf.io.acquireEn := ibufPush
  ibuf.io.writeData := ibufDin
  ibuf.io.readAddr  := ibufRaddr
  ibuf.io.releaseEn := ibufPop

  when (ibufPush) {
    rspRemMask.write(ibufWaddr, batchValidR)
  }

  val ibufDout       = ibuf.io.readData
  val offsetBits     = numReqs * dataRatioW
  val ibufDoutOffset = ibufDout(offsetBits - 1, 0)
  val ibufDoutPmask  = ibufDout(offsetBits + numReqs - 1, offsetBits)
  val ibufDoutTag    = ibufDout(ibufDataWidth - 1, offsetBits + numReqs)

  // ---- Un-merge responses --------------------------------------------------
  val inRspDataN = Wire(Vec(numReqs, UInt(dataInWidth.W)))
  val inRspMaskN = Wire(UInt(numReqs.W))
  val inRspMaskVec = Wire(Vec(numReqs, Bool()))

  for (i <- 0 until outReqs) {
    for (j <- 0 until dataRatio) {
      val r      = i * dataRatio + j
      val offLo  = r * dataRatioW
      val offset = ibufDoutOffset(offLo + dataRatioW - 1, offLo)
      val dataSlice = io.out_rsp_data(i).asTypeOf(Vec(dataRatio, UInt(dataInWidth.W)))
      inRspDataN(r) := dataSlice(offset)
      inRspMaskVec(r) := io.out_rsp_mask(i) && ibufDoutPmask(r)
    }
  }
  inRspMaskN := inRspMaskVec.asUInt

  // ---- Output assignments --------------------------------------------------
  io.out_req_valid  := outReqValidR
  io.out_req_rw     := outReqRwR
  io.out_req_mask   := outReqMaskR
  io.out_req_byteen := outReqByteenR
  io.out_req_addr   := outReqAddrR
  io.out_req_flags  := outReqFlagsR
  io.out_req_data   := outReqDataR
  io.out_req_tag    := outReqTagR

  io.in_req_ready   := inReqReadyN

  io.in_rsp_valid   := io.out_rsp_valid
  io.in_rsp_mask    := inRspMaskN
  io.in_rsp_data    := inRspDataN
  val uuidPart =
    if (uuidWidth > 0) io.out_rsp_tag(outTagWidth - 1, outTagWidth - uuidWidth)
    else 0.U
  io.in_rsp_tag     := Cat(uuidPart, ibufDoutTag)
  io.out_rsp_ready  := io.in_rsp_ready

  // ---- Performance counter: coalescing misses ------------------------------
  val partialTransfer = outReqFire && (reqRemMaskR =/= ((1L << numReqs) - 1).U(numReqs.W))
  val missesR = RegInit(0.U(perfCtrBits.W))
  missesR := missesR + partialTransfer.asUInt
  io.misses := missesR
}

// ============================================================================
// VX_mem_scheduler
//
// Top-level memory scheduler that chains together:
//   1. A core-side elastic request queue (FIFO with CORE_QUEUE_SIZE entries).
//   2. An optional coalescer (VX_mem_coalescer) when LINE_SIZE > WORD_SIZE.
//   3. A memory-side batch splitter when MERGED_REQS > MEM_CHANNELS.
//   4. An elastic output buffer on both the memory request and core response
//      paths.
//
// The index buffer (ibuf) maps memory response tags back to original core
// request tags so that out-of-order memory responses can be reassembled into
// in-order core responses.
//
// Translation notes
//   - VX_elastic_buffer → chisel3.util.Queue(size, flow=true)
//   - VX_index_buffer   → VxIndexBuffer (from Queues.scala)
//   - VX_mem_coalescer  → VXMemCoalescer (above)
//   - Batch-index counter replaces the MEM_BATCHES genvar loops.
//   - RSP_PARTIAL=0 full-reassembly path translated; RSP_PARTIAL=1 partial
//     path is also included.
//
// Parameters
//   coreReqs      – number of per-word request lanes from core
//   memChannels   – number of parallel memory channels
//   wordSize      – word size in bytes
//   lineSize      – cache line size in bytes
//   addrWidth     – word-granular address width
//   flagsWidth    – optional per-request flags width
//   tagWidth      – core request tag width
//   uuidWidth     – UUID sub-field width within tag
//   coreQueueSize – core request queue depth
//   memQueueSize  – memory (coalescer) queue depth
//   rspPartial    – 1 = deliver partial responses, 0 = wait for full line
//   coreOutBuf    – core response output buffer depth
//   memOutBuf     – memory request output buffer depth
// ============================================================================
class VXMemScheduler(
  val coreReqs      : Int = 4,
  val memChannels   : Int = 1,
  val wordSize      : Int = 4,
  val lineSize      : Int = 4,
  val addrWidth     : Int = 30,
  val flagsWidth    : Int = 0,
  val tagWidth      : Int = 8,
  val uuidWidth     : Int = 0,
  val coreQueueSize : Int = 8,
  val memQueueSize  : Int = 8,
  val rspPartial    : Int = 0,
  val coreOutBuf    : Int = 0,
  val memOutBuf     : Int = 0
) extends Module {
  val wordWidth       = wordSize  * 8
  val lineWidth       = lineSize  * 8
  val coalesceEnable  = (coreReqs > 1) && (lineSize != wordSize)
  val perLineReqs     = lineSize  / wordSize
  val mergedReqs      = coreReqs  / perLineReqs
  val memBatches      = (mergedReqs + memChannels - 1) / memChannels
  val memBatchBits    = log2Ceil(math.max(memBatches, 2))
  val memQueueAddrw   = log2Ceil(if (coalesceEnable) memQueueSize else coreQueueSize)
  val memAddrWidth    = addrWidth - log2Ceil(perLineReqs)
  val memTagWidth     = uuidWidth + memQueueAddrw + memBatchBits
  val coreQueueAddrw  = log2Ceil(coreQueueSize)
  val tagIdWidth      = tagWidth - uuidWidth
  val reqqTagWidth    = uuidWidth + coreQueueAddrw
  val mergedTagWidth  = uuidWidth + memQueueAddrw
  val coreChannels    = if (coalesceEnable) coreReqs else memChannels
  val coreBatches     = if (coalesceEnable) 1 else memBatches
  val batchSelWidth   = log2Ceil(math.max(memBatches, 2))
  val flagsW          = if (flagsWidth > 0) flagsWidth else 1

  val io = IO(new Bundle {
    // Core request
    val core_req_valid  = Input(Bool())
    val core_req_rw     = Input(Bool())
    val core_req_mask   = Input(UInt(coreReqs.W))
    val core_req_byteen = Input(Vec(coreReqs, UInt(wordSize.W)))
    val core_req_addr   = Input(Vec(coreReqs, UInt(addrWidth.W)))
    val core_req_flags  = Input(Vec(coreReqs, UInt(flagsW.W)))
    val core_req_data   = Input(Vec(coreReqs, UInt(wordWidth.W)))
    val core_req_tag    = Input(UInt(tagWidth.W))
    val core_req_ready  = Output(Bool())

    // Core request queue status
    val req_queue_empty     = Output(Bool())
    val req_queue_rw_notify = Output(Bool())

    // Core response
    val core_rsp_valid  = Output(Bool())
    val core_rsp_mask   = Output(UInt(coreReqs.W))
    val core_rsp_data   = Output(Vec(coreReqs, UInt(wordWidth.W)))
    val core_rsp_tag    = Output(UInt(tagWidth.W))
    val core_rsp_sop    = Output(Bool())
    val core_rsp_eop    = Output(Bool())
    val core_rsp_ready  = Input(Bool())

    // Memory request
    val mem_req_valid  = Output(Bool())
    val mem_req_rw     = Output(Bool())
    val mem_req_mask   = Output(UInt(memChannels.W))
    val mem_req_byteen = Output(Vec(memChannels, UInt(lineSize.W)))
    val mem_req_addr   = Output(Vec(memChannels, UInt(memAddrWidth.W)))
    val mem_req_flags  = Output(Vec(memChannels, UInt(flagsW.W)))
    val mem_req_data   = Output(Vec(memChannels, UInt(lineWidth.W)))
    val mem_req_tag    = Output(UInt(memTagWidth.W))
    val mem_req_ready  = Input(Bool())

    // Memory response
    val mem_rsp_valid  = Input(Bool())
    val mem_rsp_mask   = Input(UInt(memChannels.W))
    val mem_rsp_data   = Input(Vec(memChannels, UInt(lineWidth.W)))
    val mem_rsp_tag    = Input(UInt(memTagWidth.W))
    val mem_rsp_ready  = Output(Bool())
  })

  // ---- Index buffer for tag remapping ------------------------------------
  val ibuf = Module(new VxIndexBuffer(tagIdWidth, coreQueueSize))

  val ibufWaddr = ibuf.io.writeAddr
  val ibufFull  = ibuf.io.full
  val ibufEmpty = ibuf.io.empty

  // ---- Request queue (VX_elastic_buffer) ----------------------------------
  // Bundles the per-lane request fields together for queuing
  class ReqqBundle extends Bundle {
    val rw     = Bool()
    val mask   = UInt(coreReqs.W)
    val byteen = Vec(coreReqs, UInt(wordSize.W))
    val addr   = Vec(coreReqs, UInt(addrWidth.W))
    val flags  = Vec(coreReqs, UInt(flagsW.W))
    val data   = Vec(coreReqs, UInt(wordWidth.W))
    val tag    = UInt(reqqTagWidth.W)
  }

  val ibufReady = io.core_req_rw || !ibufFull
  val reqqQ     = Module(new Queue(new ReqqBundle, coreQueueSize, flow = true))

  val reqqTagU = Wire(UInt(reqqTagWidth.W))
  if (uuidWidth != 0) {
    reqqTagU := Cat(io.core_req_tag(tagWidth - 1, tagWidth - uuidWidth), ibufWaddr)
  } else {
    reqqTagU := ibufWaddr
  }

  reqqQ.io.enq.valid          := io.core_req_valid && ibufReady
  reqqQ.io.enq.bits.rw        := io.core_req_rw
  reqqQ.io.enq.bits.mask      := io.core_req_mask
  reqqQ.io.enq.bits.byteen    := io.core_req_byteen
  reqqQ.io.enq.bits.addr      := io.core_req_addr
  reqqQ.io.enq.bits.flags     := io.core_req_flags
  reqqQ.io.enq.bits.data      := io.core_req_data
  reqqQ.io.enq.bits.tag       := reqqTagU

  val reqqReadyIn  = reqqQ.io.enq.ready
  io.core_req_ready := reqqReadyIn && ibufReady

  val reqqValid  = reqqQ.io.deq.valid
  val reqqRw     = reqqQ.io.deq.bits.rw
  val reqqMask   = reqqQ.io.deq.bits.mask
  val reqqByteen = reqqQ.io.deq.bits.byteen
  val reqqAddr   = reqqQ.io.deq.bits.addr
  val reqqFlags  = reqqQ.io.deq.bits.flags
  val reqqData   = reqqQ.io.deq.bits.data
  val reqqTag    = reqqQ.io.deq.bits.tag

  io.req_queue_rw_notify := reqqValid && reqqQ.io.deq.ready && reqqRw
  io.req_queue_empty     := !reqqValid && ibufEmpty

  // ---- Index buffer acquire -----------------------------------------------
  val coreReqFire = io.core_req_valid && io.core_req_ready
  ibuf.io.acquireEn := coreReqFire && !io.core_req_rw
  ibuf.io.writeData := io.core_req_tag(tagIdWidth - 1, 0)

  // ---- Coalescer or passthrough -------------------------------------------
  // Signals after optional coalescing stage
  val reqValidS   = Wire(Bool())
  val reqMaskS    = Wire(UInt(mergedReqs.W))
  val reqRwS      = Wire(Bool())
  val reqByteenS  = Wire(Vec(mergedReqs, UInt(lineSize.W)))
  val reqAddrS    = Wire(Vec(mergedReqs, UInt(memAddrWidth.W)))
  val reqFlagsS   = Wire(Vec(mergedReqs, UInt(flagsW.W)))
  val reqDataS    = Wire(Vec(mergedReqs, UInt(lineWidth.W)))
  val reqTagS     = Wire(UInt(mergedTagWidth.W))
  val reqReadyS   = Wire(Bool())

  val memRspValidS = Wire(Bool())
  val memRspMaskS  = Wire(UInt(coreChannels.W))
  val memRspDataS  = Wire(Vec(coreChannels, UInt(wordWidth.W)))
  val memRspTagS   = Wire(UInt(memTagWidth.W))
  val memRspReadyS = Wire(Bool())

  if (coalesceEnable) {
    val coal = Module(new VXMemCoalescer(
      numReqs     = coreReqs,
      addrWidth   = addrWidth,
      flagsWidth  = flagsWidth,
      dataInSize  = wordSize,
      dataOutSize = lineSize,
      tagWidth    = reqqTagWidth,
      uuidWidth   = uuidWidth,
      queueSize   = memQueueSize
    ))
    coal.io.in_req_valid  := reqqValid
    coal.io.in_req_rw     := reqqRw
    coal.io.in_req_mask   := reqqMask
    coal.io.in_req_byteen := reqqByteen
    coal.io.in_req_addr   := reqqAddr
    coal.io.in_req_flags  := reqqFlags
    coal.io.in_req_data   := reqqData
    coal.io.in_req_tag    := reqqTag
    reqqQ.io.deq.ready    := coal.io.in_req_ready

    reqValidS   := coal.io.out_req_valid
    reqMaskS    := coal.io.out_req_mask
    reqRwS      := coal.io.out_req_rw
    reqByteenS  := coal.io.out_req_byteen
    reqAddrS    := coal.io.out_req_addr
    reqFlagsS   := coal.io.out_req_flags
    reqDataS    := coal.io.out_req_data
    reqTagS     := coal.io.out_req_tag
    coal.io.out_req_ready := reqReadyS

    coal.io.out_rsp_valid := io.mem_rsp_valid
    coal.io.out_rsp_mask  := io.mem_rsp_mask
    coal.io.out_rsp_data  := io.mem_rsp_data
    coal.io.out_rsp_tag   := io.mem_rsp_tag
    io.mem_rsp_ready      := coal.io.out_rsp_ready

    memRspValidS := coal.io.in_rsp_valid
    memRspMaskS  := coal.io.in_rsp_mask
    memRspDataS  := coal.io.in_rsp_data.asTypeOf(Vec(coreChannels, UInt(wordWidth.W)))
    memRspTagS   := coal.io.in_rsp_tag.pad(memTagWidth)
    coal.io.in_rsp_ready := memRspReadyS

  } else {
    // Passthrough: reqq outputs map directly to merged request signals
    // (mergedReqs == memChannels / coreBatches here)
    reqValidS  := reqqValid
    reqMaskS   := reqqMask
    reqRwS     := reqqRw
    reqByteenS := reqqByteen.asTypeOf(Vec(mergedReqs, UInt(lineSize.W)))
    reqAddrS   := reqqAddr.asTypeOf(Vec(mergedReqs, UInt(memAddrWidth.W)))
    reqFlagsS  := reqqFlags.asTypeOf(Vec(mergedReqs, UInt(flagsW.W)))
    reqDataS   := reqqData.asTypeOf(Vec(mergedReqs, UInt(lineWidth.W)))
    reqTagS    := reqqTag.pad(mergedTagWidth)
    reqqQ.io.deq.ready := reqReadyS

    memRspValidS := io.mem_rsp_valid
    memRspMaskS  := io.mem_rsp_mask
    memRspDataS  := io.mem_rsp_data.asTypeOf(Vec(coreChannels, UInt(wordWidth.W)))
    memRspTagS   := io.mem_rsp_tag
    io.mem_rsp_ready := memRspReadyS
  }

  // ---- ibuf read-address: derived from response tag -----------------------
  val coreBatchBits = log2Ceil(math.max(coreBatches, 2))
  val ibufRaddrSig  = memRspTagS(coreBatchBits + coreQueueAddrw - 1, coreBatchBits)
  ibuf.io.readAddr  := ibufRaddrSig

  // ---- Batch splitting (when MERGED_REQS > MEM_CHANNELS) ------------------
  // Build per-batch slices of the merged request
  val memReqMaskB   = Wire(Vec(memBatches, UInt(memChannels.W)))
  val memReqByteenB = Wire(Vec(memBatches, Vec(memChannels, UInt(lineSize.W))))
  val memReqAddrB   = Wire(Vec(memBatches, Vec(memChannels, UInt(memAddrWidth.W))))
  val memReqFlagsB  = Wire(Vec(memBatches, Vec(memChannels, UInt(flagsW.W))))
  val memReqDataB   = Wire(Vec(memBatches, Vec(memChannels, UInt(lineWidth.W))))

  for (i <- 0 until memBatches) {
    val maskVec = Wire(Vec(memChannels, Bool()))
    for (j <- 0 until memChannels) {
      val r = i * memChannels + j
      if (r < mergedReqs) {
        maskVec(j)          := reqMaskS(r)
        memReqByteenB(i)(j) := reqByteenS(r)
        memReqAddrB(i)(j)   := reqAddrS(r)
        memReqFlagsB(i)(j)  := reqFlagsS(r)
        memReqDataB(i)(j)   := reqDataS(r)
      } else {
        maskVec(j)          := false.B
        memReqByteenB(i)(j) := 0.U
        memReqAddrB(i)(j)   := 0.U
        memReqFlagsB(i)(j)  := 0.U
        memReqDataB(i)(j)   := 0.U
      }
    }
    memReqMaskB(i) := maskVec.asUInt
  }

  val memReqValidS  = Wire(Bool())
  val memReqMaskS   = Wire(UInt(memChannels.W))
  val memReqRwS     = Wire(Bool())
  val memReqByteenS = Wire(Vec(memChannels, UInt(lineSize.W)))
  val memReqAddrS   = Wire(Vec(memChannels, UInt(memAddrWidth.W)))
  val memReqFlagsS  = Wire(Vec(memChannels, UInt(flagsW.W)))
  val memReqDataS   = Wire(Vec(memChannels, UInt(lineWidth.W)))
  val memReqTagS    = Wire(UInt(memTagWidth.W))
  val memReqReadyS  = Wire(Bool())

  val reqBatchIdx = Wire(UInt(batchSelWidth.W))
  val reqSentAll  = Wire(Bool())

  if (memBatches != 1) {
    val reqBatchIdxR = RegInit(0.U(memBatchBits.W))

    // Find last valid batch (highest index with non-zero mask)
    val reqBatchValids = VecInit((0 until memBatches).map(i => memReqMaskB(i).orR))
    val reqBatchIdxLast = Wire(UInt(memBatchBits.W))
    reqBatchIdxLast := (memBatches - 1).U
    for (i <- (0 until memBatches).reverse) {
      when (reqBatchValids(i)) { reqBatchIdxLast := i.U }
    }

    val isDegenerateBatch = !memReqMaskB(reqBatchIdxR).orR
    val memReqValidB = reqValidS && !isDegenerateBatch
    val memReqReadyB = memReqReadyS || isDegenerateBatch

    when (reqValidS && memReqReadyB) {
      when (reqSentAll) {
        reqBatchIdxR := 0.U
      } .otherwise {
        reqBatchIdxR := reqBatchIdxR + 1.U
      }
    }

    memReqValidS := memReqValidB
    reqBatchIdx  := reqBatchIdxR
    reqSentAll   := memReqReadyB && (reqBatchIdxR === reqBatchIdxLast)
    memReqTagS   := Cat(reqTagS, reqBatchIdxR)

  } else {
    memReqValidS := reqValidS
    reqBatchIdx  := 0.U
    reqSentAll   := memReqReadyS
    memReqTagS   := reqTagS.pad(memTagWidth)
  }

  reqReadyS      := reqSentAll
  memReqMaskS   := memReqMaskB(reqBatchIdx)
  memReqRwS     := reqRwS
  memReqByteenS := memReqByteenB(reqBatchIdx)
  memReqAddrS   := memReqAddrB(reqBatchIdx)
  memReqFlagsS  := memReqFlagsB(reqBatchIdx)
  memReqDataS   := memReqDataB(reqBatchIdx)

  // ---- Memory request output buffer ----------------------------------------
  class MemReqBundle extends Bundle {
    val mask   = UInt(memChannels.W)
    val rw     = Bool()
    val byteen = Vec(memChannels, UInt(lineSize.W))
    val addr   = Vec(memChannels, UInt(memAddrWidth.W))
    val flags  = Vec(memChannels, UInt(flagsW.W))
    val data   = Vec(memChannels, UInt(lineWidth.W))
    val tag    = UInt(memTagWidth.W)
  }

  val memReqBuf = Module(new Queue(new MemReqBundle,
    if (memOutBuf > 0) memOutBuf else 1, flow = memOutBuf == 0))

  memReqBuf.io.enq.valid        := memReqValidS
  memReqBuf.io.enq.bits.mask    := memReqMaskS
  memReqBuf.io.enq.bits.rw      := memReqRwS
  memReqBuf.io.enq.bits.byteen  := memReqByteenS
  memReqBuf.io.enq.bits.addr    := memReqAddrS
  memReqBuf.io.enq.bits.flags   := memReqFlagsS
  memReqBuf.io.enq.bits.data    := memReqDataS
  memReqBuf.io.enq.bits.tag     := memReqTagS
  memReqReadyS                   := memReqBuf.io.enq.ready

  io.mem_req_valid  := memReqBuf.io.deq.valid
  io.mem_req_mask   := memReqBuf.io.deq.bits.mask
  io.mem_req_rw     := memReqBuf.io.deq.bits.rw
  io.mem_req_byteen := memReqBuf.io.deq.bits.byteen
  io.mem_req_addr   := memReqBuf.io.deq.bits.addr
  io.mem_req_flags  := memReqBuf.io.deq.bits.flags
  io.mem_req_data   := memReqBuf.io.deq.bits.data
  io.mem_req_tag    := memReqBuf.io.deq.bits.tag
  memReqBuf.io.deq.ready := io.mem_req_ready

  // ---- Memory response → core response ------------------------------------
  val rspBatchIdx = Wire(UInt(batchSelWidth.W))
  if (coreBatches > 1) {
    rspBatchIdx := memRspTagS(coreBatchBits - 1, 0)
  } else {
    rspBatchIdx := 0.U
  }

  val crspValid = Wire(Bool())
  val crspMask  = Wire(UInt(coreReqs.W))
  val crspData  = Wire(Vec(coreReqs, UInt(wordWidth.W)))
  val crspSop   = Wire(Bool())
  val crspEop   = Wire(Bool())
  val crspReady = Wire(Bool())

  if (coreReqs == 1) {
    crspValid := memRspValidS
    crspMask  := memRspMaskS
    crspSop   := true.B
    crspEop   := true.B
    crspData  := memRspDataS.asTypeOf(Vec(coreReqs, UInt(wordWidth.W)))
    memRspReadyS := crspReady
  } else {
    val rspRemMask = Mem(coreQueueSize, UInt(coreReqs.W))  // async read for combinational eop
    val currMaskVec = Wire(Vec(coreReqs, Bool()))
    for (r <- 0 until coreReqs) {
      val i = r / coreChannels
      val j = r % coreChannels
      currMaskVec(r) := (i.U === rspBatchIdx) && memRspMaskS(j)
    }
    val currMask    = currMaskVec.asUInt
    val ibufRaddr   = ibufRaddrSig  // ibuf read address, set above
    val rspRemMaskN = rspRemMask.read(ibufRaddr) & ~currMask
    val rspComplete = !rspRemMaskN.orR

    val memRspFireS = memRspValidS && memRspReadyS

    when (ibuf.io.acquireEn) {
      rspRemMask.write(ibufWaddr, io.core_req_mask)
    }
    when (memRspFireS) {
      rspRemMask.write(ibufRaddr, rspRemMaskN)
    }

    if (rspPartial != 0) {
      // Partial response path: forward each sub-response immediately
      val rspSopR = RegInit(VecInit(Seq.fill(coreQueueSize)(true.B)))
      when (ibuf.io.acquireEn)  { rspSopR(ibufWaddr) := true.B }
      when (memRspFireS)        { rspSopR(ibufRaddr) := false.B }

      crspValid := memRspValidS
      crspMask  := currMask
      crspSop   := rspSopR(ibufRaddr)

      for (r <- 0 until coreReqs) {
        val j = r % coreChannels
        crspData(r) := memRspDataS(j)
      }
      memRspReadyS := crspReady
    } else {
      // Full reassembly path: accumulate all sub-responses then deliver
      val rspOrigMask = Mem(coreQueueSize, UInt(coreReqs.W))  // async read
      when (ibuf.io.acquireEn) {
        rspOrigMask.write(ibufWaddr, io.core_req_mask)
      }

      // Per-channel, per-batch response store
      val rspStore = Array.fill(coreChannels)(
        Array.fill(coreBatches)(
          SyncReadMem(coreQueueSize, UInt(wordWidth.W))
        )
      )

      for (ch <- 0 until coreChannels) {
        for (bat <- 0 until coreBatches) {
          val wren = memRspFireS &&
            (bat.U === rspBatchIdx) &&
            (if (coreChannels == 1) true.B else memRspMaskS(ch))
          when (wren) {
            rspStore(ch)(bat).write(ibufRaddr, memRspDataS(ch))
          }
        }
      }

      for (r <- 0 until coreReqs) {
        val i = r / coreChannels
        val j = r % coreChannels
        crspData(r) := rspStore(j)(i).read(ibufRaddr)
      }

      crspValid := memRspValidS && rspComplete
      crspMask  := rspOrigMask.read(ibufRaddr)
      crspSop   := true.B
      memRspReadyS := crspReady || !rspComplete
    }

    crspEop := rspComplete
  }

  // ibuf release on end-of-packet
  ibuf.io.releaseEn := crspValid && crspReady && crspEop

  // Reconstruct original tag from ibuf
  val crspTag = Wire(UInt(tagWidth.W))
  if (uuidWidth != 0) {
    crspTag := Cat(
      memRspTagS(memTagWidth - 1, memTagWidth - uuidWidth),
      ibuf.io.readData)
  } else {
    crspTag := ibuf.io.readData
  }

  // ---- Core response output buffer ----------------------------------------
  class CoreRspBundle extends Bundle {
    val mask = UInt(coreReqs.W)
    val sop  = Bool()
    val eop  = Bool()
    val data = Vec(coreReqs, UInt(wordWidth.W))
    val tag  = UInt(tagWidth.W)
  }

  val rspBuf = Module(new Queue(new CoreRspBundle,
    if (coreOutBuf > 0) coreOutBuf else 1, flow = coreOutBuf == 0))

  rspBuf.io.enq.valid       := crspValid
  rspBuf.io.enq.bits.mask   := crspMask
  rspBuf.io.enq.bits.sop    := crspSop
  rspBuf.io.enq.bits.eop    := crspEop
  rspBuf.io.enq.bits.data   := crspData
  rspBuf.io.enq.bits.tag    := crspTag
  crspReady                  := rspBuf.io.enq.ready

  io.core_rsp_valid := rspBuf.io.deq.valid
  io.core_rsp_mask  := rspBuf.io.deq.bits.mask
  io.core_rsp_sop   := rspBuf.io.deq.bits.sop
  io.core_rsp_eop   := rspBuf.io.deq.bits.eop
  io.core_rsp_data  := rspBuf.io.deq.bits.data
  io.core_rsp_tag   := rspBuf.io.deq.bits.tag
  rspBuf.io.deq.ready := io.core_rsp_ready
}
