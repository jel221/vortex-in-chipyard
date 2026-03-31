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

// Chisel translations of:
//   VX_lsu_mem_if.sv   – already translated in MemBusBundle.scala (LsuMemBusBundle)
//   VX_lsu_adapter.sv  – scatter/gather between per-warp LSU lanes and per-lane MemBus ports
//   VX_lsu_mem_arb.sv  – arbitrated N-input → M-output LSU memory bus
//   VX_lmem_switch.sv  – split LSU requests to global / local memory based on flag

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// LsuAdapter
//
// Translation of VX_lsu_adapter.sv.
//
// Adapts a single VX_lsu_mem_if (multi-lane, masked request) into an array
// of per-lane VX_mem_bus_if channels.
//
// The SV module uses VX_stream_unpack to scatter a masked multi-lane request
// into individual per-lane requests, and VX_stream_pack to re-assemble
// per-lane responses into a masked multi-lane response.
//
// In Chisel we implement the same logic directly:
//   • Request scatter: for each lane, if mask[lane] is set and the overall
//     request is accepted, fire that lane's mem bus request.
//   • Response pack: use a round-robin arbiter over the per-lane response
//     channels; accumulate a mask of arrived responses and emit the packed
//     response once all lanes for a given tag have responded.
//
// For simplicity (and faithfulness to the SV VX_stream_unpack / VX_stream_pack
// behaviour which handles one tag at a time) we implement a single-transaction-
// in-flight model: the adapter stalls new requests until all pending lane
// responses have been collected.
//
// Parameters
//   numLanes     – NUM_LANES
//   dataSize     – DATA_SIZE (bytes)
//   addrWidth    – REQ_ADDR_WIDTH = MEM_ADDR_WIDTH - log2(DATA_SIZE)
//   flagsWidth   – MEM_FLAGS_WIDTH
//   tagWidth     – TAG_WIDTH (of the LSU interface)
//   tagSelBits   – TAG_SEL_BITS (used by VX_stream_pack; kept for reference)
//   uuidWidth    – UUID sub-field width
// ---------------------------------------------------------------------------

class LsuAdapter(
    numLanes:   Int,
    dataSize:   Int,
    addrWidth:  Int,
    flagsWidth: Int = 3,
    tagWidth:   Int = 1,
    tagSelBits: Int = 0,
    uuidWidth:  Int = 0
) extends Module {
  val io = IO(new Bundle {
    val lsuMem = Flipped(new LsuMemBusBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
    val memBus = Vec(numLanes, new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
  })

  // -----------------------------------------------------------------------
  // Request scatter path
  //
  // A masked multi-lane request is scattered to numLanes individual requests.
  // All active (masked) lanes must be accepted in the same cycle.
  // We hold the request until all active lanes have been granted.
  // -----------------------------------------------------------------------

  // Track which lanes have been dispatched for the current request.
  val dispatched = RegInit(0.U(numLanes.W))

  // The request is "in progress" if lsuMem.req.valid is high.
  // A lane is active if its mask bit is set AND it has not yet been dispatched.
  val activeReq   = io.lsuMem.req.valid
  val maskBits    = io.lsuMem.req.bits.mask
  val pendingLanes = maskBits & ~dispatched  // lanes still to be sent

  // For each lane: fire the mem bus request if this lane is pending.
  val laneAccepted = Wire(Vec(numLanes, Bool()))
  for (i <- 0 until numLanes) {
    io.memBus(i).req.valid        := activeReq && pendingLanes(i)
    io.memBus(i).req.bits.rw     := io.lsuMem.req.bits.rw
    io.memBus(i).req.bits.addr   := io.lsuMem.req.bits.addr(i)
    io.memBus(i).req.bits.data   := io.lsuMem.req.bits.data(i)
    io.memBus(i).req.bits.byteen := io.lsuMem.req.bits.byteen(i)
    io.memBus(i).req.bits.flags  := io.lsuMem.req.bits.flags(i)
    io.memBus(i).req.bits.tag    := io.lsuMem.req.bits.tag
    laneAccepted(i)               := io.memBus(i).req.valid && io.memBus(i).req.ready
  }

  // Accumulate dispatched lanes across cycles.
  val newDispatched = dispatched | laneAccepted.asUInt
  // All active lanes have been dispatched when newDispatched covers maskBits.
  val allDispatched = (newDispatched & maskBits) === maskBits

  when (activeReq) {
    when (allDispatched) {
      dispatched := 0.U  // reset for next request
    } .otherwise {
      dispatched := newDispatched
    }
  } .otherwise {
    dispatched := 0.U
  }

  // Signal ready to the LSU once all active lanes have been accepted.
  io.lsuMem.req.ready := activeReq && allDispatched

  // -----------------------------------------------------------------------
  // Response pack path
  //
  // Per-lane responses (each with the same tag) are collected and emitted as
  // a single packed response with a mask indicating which lanes responded.
  //
  // We collect responses into a buffer, accumulating the mask.  Once we
  // have at least one valid response ready to forward, we emit it.
  // For simplicity (VX_stream_pack with TAG_SEL_BITS == 0) we hold pending
  // responses until they can be forwarded.
  // -----------------------------------------------------------------------

  // Pending (buffered) response accumulation.
  val rspPending    = RegInit(false.B)
  val rspMask       = RegInit(0.U(numLanes.W))
  val rspData       = Reg(Vec(numLanes, UInt((dataSize * 8).W)))
  val rspTag        = Reg(new MemBusTagBundle(tagWidth, uuidWidth))

  // Accept per-lane responses when there is no pending packed response blocking.
  val rspBlocked = rspPending && !io.lsuMem.rsp.ready

  for (i <- 0 until numLanes) {
    io.memBus(i).rsp.ready := !rspBlocked
    when (io.memBus(i).rsp.valid && io.memBus(i).rsp.ready) {
      rspMask   := rspMask | (1.U << i.U)
      rspData(i):= io.memBus(i).rsp.bits.data
      rspTag    := io.memBus(i).rsp.bits.tag
      rspPending:= true.B
    }
  }

  // Emit the packed response when at least one lane has responded.
  io.lsuMem.rsp.valid      := rspPending
  io.lsuMem.rsp.bits.mask  := rspMask
  io.lsuMem.rsp.bits.data  := rspData
  io.lsuMem.rsp.bits.tag   := rspTag

  // Clear when the downstream accepts.
  when (io.lsuMem.rsp.valid && io.lsuMem.rsp.ready) {
    rspPending := false.B
    rspMask    := 0.U
  }
}

// ---------------------------------------------------------------------------
// LsuMemArb
//
// Translation of VX_lsu_mem_arb.sv.
//
// Arbitrates NUM_INPUTS LSU memory bus interfaces down to NUM_OUTPUTS using
// round-robin arbitration.  When NUM_INPUTS > NUM_OUTPUTS selector bits are
// embedded in the tag (at tagSelIdx) for response routing, exactly mirroring
// the VX_mem_arb pattern.
//
// Parameters
//   numInputs  – NUM_INPUTS
//   numOutputs – NUM_OUTPUTS
//   numLanes   – NUM_LANES
//   dataSize   – DATA_SIZE (bytes)
//   addrWidth  – ADDR_WIDTH
//   flagsWidth – FLAGS_WIDTH
//   tagWidth   – TAG_WIDTH (input ports)
//   tagSelIdx  – TAG_SEL_IDX: insertion position for selector bits
//   uuidWidth  – UUID sub-field width
// ---------------------------------------------------------------------------

class LsuMemArb(
    numInputs:  Int,
    numOutputs: Int,
    numLanes:   Int,
    dataSize:   Int,
    addrWidth:  Int,
    flagsWidth: Int = 3,
    tagWidth:   Int = 1,
    tagSelIdx:  Int = 0,
    uuidWidth:  Int = 0
) extends Module {
  require(numInputs  >= 1)
  require(numOutputs >= 1)

  val logNumReqs: Int =
    if (numInputs > numOutputs)
      math.max(1, log2Ceil(math.ceil(numInputs.toDouble / numOutputs).toInt))
    else 0

  val outTagWidth: Int = if (numInputs > numOutputs) tagWidth + logNumReqs else tagWidth

  val io = IO(new Bundle {
    val busIn  = Flipped(Vec(numInputs,  new LsuMemBusBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth,    uuidWidth)))
    val busOut =         Vec(numOutputs, new LsuMemBusBundle(numLanes, dataSize, addrWidth, flagsWidth, outTagWidth, uuidWidth))
  })

  // -----------------------------------------------------------------------
  // Request arbitration: numInputs → numOutputs
  // -----------------------------------------------------------------------
  for (outIdx <- 0 until numOutputs) {
    val arb = Module(new RRArbiter(
      new LsuMemReqBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth),
      numInputs
    ))
    for (inIdx <- 0 until numInputs) {
      arb.io.in(inIdx).valid := io.busIn(inIdx).req.valid
      arb.io.in(inIdx).bits  := io.busIn(inIdx).req.bits
    }

    io.busOut(outIdx).req.valid         := arb.io.out.valid
    arb.io.out.ready                    := io.busOut(outIdx).req.ready

    // Copy fields; embed selector bits into tag if needed.
    io.busOut(outIdx).req.bits.mask    := arb.io.out.bits.mask
    io.busOut(outIdx).req.bits.rw      := arb.io.out.bits.rw
    io.busOut(outIdx).req.bits.addr    := arb.io.out.bits.addr
    io.busOut(outIdx).req.bits.data    := arb.io.out.bits.data
    io.busOut(outIdx).req.bits.byteen  := arb.io.out.bits.byteen
    io.busOut(outIdx).req.bits.flags   := arb.io.out.bits.flags

    if (numInputs > numOutputs) {
      val chosenBits = arb.io.chosen.asUInt
      val inTag      = arb.io.out.bits.tag.asUInt
      val outTag     = WireDefault(0.U(outTagWidth.W))
      if (tagSelIdx == 0) {
        outTag := Cat(inTag, chosenBits(logNumReqs - 1, 0))
      } else if (tagSelIdx >= tagWidth) {
        outTag := Cat(chosenBits(logNumReqs - 1, 0), inTag)
      } else {
        outTag := Cat(
          inTag(tagWidth - 1, tagSelIdx),
          chosenBits(logNumReqs - 1, 0),
          inTag(tagSelIdx - 1, 0)
        )
      }
      io.busOut(outIdx).req.bits.tag := outTag.asTypeOf(io.busOut(outIdx).req.bits.tag)
    } else {
      io.busOut(outIdx).req.bits.tag := arb.io.out.bits.tag
    }

    for (inIdx <- 0 until numInputs) {
      io.busIn(inIdx).req.ready := arb.io.in(inIdx).ready
    }
  }

  // -----------------------------------------------------------------------
  // Response path
  // -----------------------------------------------------------------------
  if (numInputs > numOutputs) {
    for (inIdx <- 0 until numInputs) {
      val valids = Wire(Vec(numOutputs, Bool()))
      val datas  = Wire(Vec(numOutputs, new LsuMemRspBundle(numLanes, dataSize, tagWidth, uuidWidth)))
      for (outIdx <- 0 until numOutputs) {
        val rspTag  = io.busOut(outIdx).rsp.bits.tag.asUInt
        val selBits: UInt =
          if (tagSelIdx == 0)          rspTag(logNumReqs - 1, 0)
          else if (tagSelIdx >= tagWidth) rspTag(outTagWidth - 1, outTagWidth - logNumReqs)
          else                          rspTag(tagSelIdx + logNumReqs - 1, tagSelIdx)

        val strippedTag: UInt =
          if (tagSelIdx == 0)
            rspTag(outTagWidth - 1, logNumReqs)
          else if (tagSelIdx >= tagWidth)
            rspTag(tagWidth - 1, 0)
          else
            Cat(rspTag(outTagWidth - 1, tagSelIdx + logNumReqs), rspTag(tagSelIdx - 1, 0))

        valids(outIdx) := io.busOut(outIdx).rsp.valid && (selBits === inIdx.U)
        datas(outIdx) := {
          val w = Wire(new LsuMemRspBundle(numLanes, dataSize, tagWidth, uuidWidth))
          w.mask := io.busOut(outIdx).rsp.bits.mask
          w.data := io.busOut(outIdx).rsp.bits.data
          w.tag  := strippedTag.asTypeOf(w.tag)
          w
        }
      }
      val anyValid = valids.reduce(_ || _)
      io.busIn(inIdx).rsp.valid := anyValid
      io.busIn(inIdx).rsp.bits  := MuxCase(datas(0), (0 until numOutputs).map(j => valids(j) -> datas(j)))
      for (outIdx <- 0 until numOutputs) {
        io.busOut(outIdx).rsp.ready := io.busIn(inIdx).rsp.ready && valids(outIdx)
      }
    }
  } else {
    val rspArb = Module(new RRArbiter(
      new LsuMemRspBundle(numLanes, dataSize, tagWidth, uuidWidth),
      numOutputs
    ))
    for (outIdx <- 0 until numOutputs) {
      rspArb.io.in(outIdx).valid  := io.busOut(outIdx).rsp.valid
      rspArb.io.in(outIdx).bits   := io.busOut(outIdx).rsp.bits
      io.busOut(outIdx).rsp.ready := rspArb.io.in(outIdx).ready
    }
    for (inIdx <- 0 until numInputs) {
      io.busIn(inIdx).rsp.valid := rspArb.io.out.valid
      io.busIn(inIdx).rsp.bits  := rspArb.io.out.bits
    }
    rspArb.io.out.ready := io.busIn.map(_.rsp.ready).reduce(_ || _)
  }
}

// ---------------------------------------------------------------------------
// LmemSwitch
//
// Translation of VX_lmem_switch.sv.
//
// Splits a single LSU memory request stream into two:
//   • globalOut – requests whose per-lane flags have MEM_REQ_FLAG_LOCAL == 0
//   • localOut  – requests whose per-lane flags have MEM_REQ_FLAG_LOCAL == 1
//
// The split is done per-lane: the mask forwarded to each output has the
// appropriate lanes zeroed.  A request may produce traffic on both outputs
// simultaneously when some lanes target global and others local memory.
//
// Responses from both outputs are arbitrated (round-robin) back to the input.
//
// Parameters
//   numLanes    – `NUM_LSU_LANES
//   dataSize    – LSU_WORD_SIZE (bytes)
//   addrWidth   – LSU_ADDR_WIDTH
//   flagsWidth  – MEM_FLAGS_WIDTH
//   tagWidth    – LSU_TAG_WIDTH
//   uuidWidth   – UUID sub-field width
//   localFlagBit – bit index of MEM_REQ_FLAG_LOCAL within flags (default 0)
// ---------------------------------------------------------------------------

class LmemSwitch(
    numLanes:    Int,
    dataSize:    Int,
    addrWidth:   Int,
    flagsWidth:  Int = 3,
    tagWidth:    Int = 1,
    uuidWidth:   Int = 0,
    localFlagBit: Int = 0
) extends Module {
  val io = IO(new Bundle {
    val lsuIn    = Flipped(new LsuMemBusBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
    val globalOut =        new LsuMemBusBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth)
    val localOut  =        new LsuMemBusBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth)
  })

  // Per-lane: is this lane targeting local memory?
  val isAddrLocalMask = VecInit((0 until numLanes).map { i =>
    io.lsuIn.req.bits.flags(i)(localFlagBit)
  })

  // Is any masked lane going global / local?
  val isAddrGlobal = (io.lsuIn.req.bits.mask & ~isAddrLocalMask.asUInt).orR
  val isAddrLocal  = (io.lsuIn.req.bits.mask &  isAddrLocalMask.asUInt).orR

  // -----------------------------------------------------------------------
  // Global output request: forward with local-lane bits cleared from mask
  // -----------------------------------------------------------------------
  // Use a queue (elastic buffer) to decouple, matching the SV VX_elastic_buffer.
  val globalBuf = Module(new Queue(
    new LsuMemReqBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth),
    entries = 2, pipe = false, flow = false
  ))
  globalBuf.io.enq.valid       := io.lsuIn.req.valid && isAddrGlobal
  globalBuf.io.enq.bits        := io.lsuIn.req.bits
  globalBuf.io.enq.bits.mask   := io.lsuIn.req.bits.mask & ~isAddrLocalMask.asUInt
  io.globalOut.req.valid       := globalBuf.io.deq.valid
  io.globalOut.req.bits        := globalBuf.io.deq.bits
  globalBuf.io.deq.ready       := io.globalOut.req.ready

  // -----------------------------------------------------------------------
  // Local output request: forward with global-lane bits cleared from mask
  // -----------------------------------------------------------------------
  val localBuf = Module(new Queue(
    new LsuMemReqBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth),
    entries = 2, pipe = false, flow = false
  ))
  localBuf.io.enq.valid        := io.lsuIn.req.valid && isAddrLocal
  localBuf.io.enq.bits         := io.lsuIn.req.bits
  localBuf.io.enq.bits.mask    := io.lsuIn.req.bits.mask &  isAddrLocalMask.asUInt
  io.localOut.req.valid        := localBuf.io.deq.valid
  io.localOut.req.bits         := localBuf.io.deq.bits
  localBuf.io.deq.ready        := io.localOut.req.ready

  // -----------------------------------------------------------------------
  // Input ready: accept when at least one of the two output buffers can take
  // the request (matching the SV combinational ready logic).
  // -----------------------------------------------------------------------
  io.lsuIn.req.ready := (globalBuf.io.enq.ready && isAddrGlobal) ||
                        (localBuf.io.enq.ready  && isAddrLocal)

  // -----------------------------------------------------------------------
  // Response: round-robin arbitration of global + local responses
  // -----------------------------------------------------------------------
  val rspArb = Module(new RRArbiter(
    new LsuMemRspBundle(numLanes, dataSize, tagWidth, uuidWidth),
    2
  ))
  rspArb.io.in(0).valid   := io.globalOut.rsp.valid
  rspArb.io.in(0).bits    := io.globalOut.rsp.bits
  io.globalOut.rsp.ready  := rspArb.io.in(0).ready

  rspArb.io.in(1).valid   := io.localOut.rsp.valid
  rspArb.io.in(1).bits    := io.localOut.rsp.bits
  io.localOut.rsp.ready   := rspArb.io.in(1).ready

  io.lsuIn.rsp.valid      := rspArb.io.out.valid
  io.lsuIn.rsp.bits       := rspArb.io.out.bits
  rspArb.io.out.ready     := io.lsuIn.rsp.ready
}
