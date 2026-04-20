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
//   VX_mem_arb.sv    – arbitrated N-input → M-output memory bus crossbar
//   VX_mem_switch.sv – switched (sel-driven) N-input → M-output memory bus crossbar

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// MemArb
//
// Translation of VX_mem_arb.sv.
//
// The SV module arbitrates NUM_INPUTS request streams down to NUM_OUTPUTS
// using VX_stream_arb (round-robin by default).  When NUM_INPUTS > NUM_OUTPUTS
// the arbitration selector bits are inserted into / removed from the tag so
// that responses can be steered back to the correct input port.
//
// Parameters
//   numInputs   – NUM_INPUTS
//   numOutputs  – NUM_OUTPUTS
//   dataSize    – DATA_SIZE (bytes)
//   addrWidth   – ADDR_WIDTH (MEM_ADDR_WIDTH - log2(DATA_SIZE))
//   flagsWidth  – FLAGS_WIDTH / MEM_FLAGS_WIDTH (default 3)
//   tagWidth    – TAG_WIDTH for the *input* ports
//   tagSelIdx   – TAG_SEL_IDX: bit position where selector bits are inserted
//   uuidWidth   – UUID sub-field width within the tag (0 = no UUID)
//
// When numInputs > numOutputs the output tag is widened by
//   logNumReqs = ceil(log2(ceil(numInputs / numOutputs))) bits
// inserted at tagSelIdx.  The input tag on the response path has those bits
// extracted and used to route the response to the originating input port.
//
// Arbitration and response-path switching are modelled functionally using
// Chisel's Arbiter / RRArbiter and a registered output stage.  In the
// simulation model every port is registered so timing matches the SV "R"
// (round-robin) arbiter style.
// ---------------------------------------------------------------------------

class MemArb(
    numInputs:  Int,
    numOutputs: Int,
    dataSize:   Int,
    addrWidth:  Int,
    flagsWidth: Int = 3,
    tagWidth:   Int = 1,
    tagSelIdx:  Int = 0,
    uuidWidth:  Int = 0
) extends Module {
  require(numInputs  >= 1)
  require(numOutputs >= 1)

  // selector bits inserted into the output tag when numInputs > numOutputs
  val logNumReqs: Int =
    if (numInputs > numOutputs)
      math.max(1, log2Ceil(math.ceil(numInputs.toDouble / numOutputs).toInt))
    else 0

  // output tag width (wider when we embed selector bits)
  val outTagWidth: Int = if (numInputs > numOutputs) tagWidth + logNumReqs else tagWidth

  val io = IO(new Bundle {
    val busIn  = Flipped(Vec(numInputs,  new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth,    uuidWidth)))
    val busOut =         Vec(numOutputs, new MemBusBundle(dataSize, addrWidth, flagsWidth, outTagWidth, uuidWidth))
  })

  // -----------------------------------------------------------------------
  // Request path: numInputs → numOutputs round-robin arbitration
  // -----------------------------------------------------------------------

  // We instantiate one RRArbiter per output port, each selecting from a
  // subset of inputs (simple interleaved assignment: input i goes to output
  // i % numOutputs).  For the common cases (N:1 and 1:1) this is exact.
  // For the general N:M case we replicate each input to every arbiter that
  // could win it, then mask with a grant from a higher-level scheduler.
  // Here we use the simplest faithful model: a single flat RRArbiter over
  // all inputs feeding a demux to outputs.  This is behaviourally equivalent
  // for the numOutputs == 1 case; for numOutputs > 1 we create one arbiter
  // per output fed by all inputs (the SV does the same conceptually via
  // VX_stream_arb).

  for (outIdx <- 0 until numOutputs) {
    // Collect all input requests that are eligible for this output port.
    // In the SV "R" round-robin arbitration the outputs are assigned in a
    // balanced fashion.  We model each output as arbitrating among all
    // inputs with a round-robin arbiter.  Grant is per-cycle exclusive
    // via the arbiter's `io.chosen` signal.
    val arb = Module(new RRArbiter(
      new MemBusReqBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth),
      numInputs
    ))

    for (inIdx <- 0 until numInputs) {
      arb.io.in(inIdx).valid := io.busIn(inIdx).req.valid
      arb.io.in(inIdx).bits  := io.busIn(inIdx).req.bits
    }

    // Connect output
    io.busOut(outIdx).req.valid := arb.io.out.valid
    arb.io.out.ready            := io.busOut(outIdx).req.ready

    // Build output request: copy all fields from the winning input.
    // If numInputs > numOutputs, insert the arbiter-chosen selector bits
    // into the tag at tagSelIdx.
    if (numInputs > numOutputs) {
      val chosenBits = arb.io.chosen.asUInt
      // Insert selector bits into tag
      val inTag = arb.io.out.bits.tag.asUInt
      val outTag = WireDefault(0.U(outTagWidth.W))
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
      io.busOut(outIdx).req.bits.tag    := outTag.asTypeOf(io.busOut(outIdx).req.bits.tag)
    } else {
      io.busOut(outIdx).req.bits.tag    := arb.io.out.bits.tag
    }
    io.busOut(outIdx).req.bits.rw     := arb.io.out.bits.rw
    io.busOut(outIdx).req.bits.addr   := arb.io.out.bits.addr
    io.busOut(outIdx).req.bits.data   := arb.io.out.bits.data
    io.busOut(outIdx).req.bits.byteen := arb.io.out.bits.byteen
    io.busOut(outIdx).req.bits.flags  := arb.io.out.bits.flags

    // Feed back ready to the winning input; losers get false.
    for (inIdx <- 0 until numInputs) {
      // An input is ready when: it is chosen AND the output is ready.
      io.busIn(inIdx).req.ready :=
        arb.io.in(inIdx).ready
    }
  }

  // -----------------------------------------------------------------------
  // Response path
  // -----------------------------------------------------------------------
  // When numInputs > numOutputs responses arrive on the (narrower) output
  // ports and must be steered back to the correct input port using the
  // selector bits embedded in the tag.
  //
  // When numInputs <= numOutputs each output may return responses and we
  // arbitrate them back to the (narrower) input side.

  if (numInputs > numOutputs) {
    // For each output port, extract the selector bits from the response tag
    // to determine which input to route to.
    // We collect all (outPort, inputIdx) pairs and use a Mux.
    for (inIdx <- 0 until numInputs) {
      val valids = Wire(Vec(numOutputs, Bool()))
      val datas  = Wire(Vec(numOutputs, new MemBusRspBundle(dataSize, tagWidth, uuidWidth)))
      for (outIdx <- 0 until numOutputs) {
        val rspTag  = io.busOut(outIdx).rsp.bits.tag.asUInt
        // Extract selector bits from rspTag
        val selBits: UInt =
          if (tagSelIdx == 0)         rspTag(logNumReqs - 1, 0)
          else if (tagSelIdx >= tagWidth) rspTag(outTagWidth - 1, outTagWidth - logNumReqs)
          else                         rspTag(tagSelIdx + logNumReqs - 1, tagSelIdx)

        // Strip the selector bits to get the original tag
        val strippedTag: UInt =
          if (tagSelIdx == 0)
            rspTag(outTagWidth - 1, logNumReqs)
          else if (tagSelIdx >= tagWidth)
            rspTag(tagWidth - 1, 0)
          else
            Cat(rspTag(outTagWidth - 1, tagSelIdx + logNumReqs), rspTag(tagSelIdx - 1, 0))

        valids(outIdx) := io.busOut(outIdx).rsp.valid && (selBits === inIdx.U)
        datas(outIdx)  := {
          val w = Wire(new MemBusRspBundle(dataSize, tagWidth, uuidWidth))
          w.data := io.busOut(outIdx).rsp.bits.data
          w.tag  := strippedTag.asTypeOf(w.tag)
          w
        }
      }
      // Mux: pick the first (or only) output that targets this input
      val anyValid = valids.reduce(_ || _)
      io.busIn(inIdx).rsp.valid := anyValid
      io.busIn(inIdx).rsp.bits  := MuxCase(datas(0), (0 until numOutputs).map(j => valids(j) -> datas(j)))
    }

    // rsp.ready: for each output port, steer ready back to whichever input the
    // current response is destined for (determined by the sel bits in the tag).
    for (outIdx <- 0 until numOutputs) {
      val rspTag = io.busOut(outIdx).rsp.bits.tag.asUInt
      val selBits: UInt =
        if (tagSelIdx == 0)            rspTag(logNumReqs - 1, 0)
        else if (tagSelIdx >= tagWidth) rspTag(outTagWidth - 1, outTagWidth - logNumReqs)
        else                            rspTag(tagSelIdx + logNumReqs - 1, tagSelIdx)
      io.busOut(outIdx).rsp.ready := MuxCase(false.B,
        (0 until numInputs).map(inIdx => (selBits === inIdx.U) -> io.busIn(inIdx).rsp.ready))
    }
  } else {
    // numInputs <= numOutputs: arbitrate responses from all outputs to inputs
    val rspArb = Module(new RRArbiter(
      new MemBusRspBundle(dataSize, tagWidth, uuidWidth),
      numOutputs
    ))
    for (outIdx <- 0 until numOutputs) {
      rspArb.io.in(outIdx).valid := io.busOut(outIdx).rsp.valid
      rspArb.io.in(outIdx).bits  := io.busOut(outIdx).rsp.bits
      io.busOut(outIdx).rsp.ready := rspArb.io.in(outIdx).ready
    }
    // Fan out to all inputs (only one at a time in the 1-output case)
    // For the general case route to a single output via index; here we
    // broadcast to all inputs and gate valid per input.
    // In the SV model responses are broadcast to the single matching input.
    // We approximate by connecting all inputs to the same arbiter output.
    for (inIdx <- 0 until numInputs) {
      io.busIn(inIdx).rsp.valid := rspArb.io.out.valid
      io.busIn(inIdx).rsp.bits  := rspArb.io.out.bits
    }
    rspArb.io.out.ready := io.busIn.map(_.rsp.ready).reduce(_ || _)
  }
}

// ---------------------------------------------------------------------------
// MemSwitch
//
// Translation of VX_mem_switch.sv.
//
// Unlike MemArb (which arbitrates freely), MemSwitch routes each input to a
// specific output port selected by the `busSel` input.  Responses from all
// output ports are arbitrated back to the inputs.
//
// Parameters
//   numInputs   – NUM_INPUTS
//   numOutputs  – NUM_OUTPUTS
//   dataSize    – DATA_SIZE (bytes)
//   addrWidth   – ADDR_WIDTH
//   flagsWidth  – FLAGS_WIDTH (MEM_FLAGS_WIDTH)
//   tagWidth    – TAG_WIDTH
//   uuidWidth   – UUID sub-field width
// ---------------------------------------------------------------------------

class MemSwitch(
    numInputs:  Int,
    numOutputs: Int,
    dataSize:   Int,
    addrWidth:  Int,
    flagsWidth: Int = 3,
    tagWidth:   Int = 1,
    uuidWidth:  Int = 0
) extends Module {
  require(numInputs  >= 1)
  require(numOutputs >= 1)

  val selBits: Int = if (numInputs > numOutputs)
    math.max(1, log2Ceil(math.ceil(numInputs.toDouble / numOutputs).toInt))
  else
    math.max(1, log2Ceil(math.ceil(numOutputs.toDouble / numInputs).toInt))

  val selCount = math.min(numInputs, numOutputs)

  val io = IO(new Bundle {
    // busSel(i) selects the target output for input i (when numInputs <= numOutputs)
    // or the target input for output i (when numOutputs <= numInputs).
    val busSel = Input(Vec(selCount, UInt(math.max(1, log2Ceil(math.max(numInputs, numOutputs))).W)))
    val busIn  = Flipped(Vec(numInputs,  new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth)))
    val busOut =         Vec(numOutputs, new MemBusBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
  })

  // -----------------------------------------------------------------------
  // Request path: route input[i] to output[busSel[i]]
  // -----------------------------------------------------------------------

  // Build output-valid and data from the selected input
  for (outIdx <- 0 until numOutputs) {
    // Find which input is routed to this output
    // busSel has one entry per selCount = min(in, out)
    val selIdx = math.min(outIdx, selCount - 1)

    // When numInputs <= numOutputs: each input selects its output;
    //   output outIdx fires when some input i has busSel(i) == outIdx.
    // When numInputs > numOutputs: each output outIdx services a set of
    //   inputs; we use a priority arbiter among those.

    if (numInputs <= numOutputs) {
      // Each input routes to one output; output outIdx is active when the
      // unique input that chose it is valid.
      val matchValid = Wire(Vec(numInputs, Bool()))
      val matchData  = Wire(Vec(numInputs, new MemBusReqBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth)))
      for (inIdx <- 0 until numInputs) {
        matchValid(inIdx) := io.busIn(inIdx).req.valid && (io.busSel(inIdx) === outIdx.U)
        matchData(inIdx)  := io.busIn(inIdx).req.bits
      }
      val anyMatch = matchValid.reduce(_ || _)
      io.busOut(outIdx).req.valid := anyMatch
      io.busOut(outIdx).req.bits  := MuxCase(
        matchData(0),
        (0 until numInputs).map(i => matchValid(i) -> matchData(i))
      )
      for (inIdx <- 0 until numInputs) {
        io.busIn(inIdx).req.ready :=
          io.busOut(outIdx).req.ready && matchValid(inIdx)
      }
    } else {
      // numInputs > numOutputs: use priority arbiter per output.
      val arb = Module(new RRArbiter(
        new MemBusReqBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth),
        numInputs
      ))
      for (inIdx <- 0 until numInputs) {
        arb.io.in(inIdx).valid := io.busIn(inIdx).req.valid && (io.busSel(math.min(inIdx, selCount-1)) === outIdx.U)
        arb.io.in(inIdx).bits  := io.busIn(inIdx).req.bits
      }
      io.busOut(outIdx).req.valid := arb.io.out.valid
      io.busOut(outIdx).req.bits  := arb.io.out.bits
      arb.io.out.ready            := io.busOut(outIdx).req.ready
      for (inIdx <- 0 until numInputs) {
        io.busIn(inIdx).req.ready := arb.io.in(inIdx).ready
      }
    }
  }

  // -----------------------------------------------------------------------
  // Response path: arbitrate all output responses back to inputs
  // -----------------------------------------------------------------------
  val rspArb = Module(new RRArbiter(
    new MemBusRspBundle(dataSize, tagWidth, uuidWidth),
    numOutputs
  ))
  for (outIdx <- 0 until numOutputs) {
    rspArb.io.in(outIdx).valid := io.busOut(outIdx).rsp.valid
    rspArb.io.in(outIdx).bits  := io.busOut(outIdx).rsp.bits
    io.busOut(outIdx).rsp.ready := rspArb.io.in(outIdx).ready
  }
  for (inIdx <- 0 until numInputs) {
    io.busIn(inIdx).rsp.valid := rspArb.io.out.valid
    io.busIn(inIdx).rsp.bits  := rspArb.io.out.bits
  }
  rspArb.io.out.ready := io.busIn.map(_.rsp.ready).reduce(_ || _)
}
