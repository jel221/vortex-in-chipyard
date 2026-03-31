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

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// VxElasticBuffer
//
// A valid/ready elastic buffer with configurable depth.
//   size == 0  → pure wire pass-through
//   size >= 1  → a skid buffer (size=1) or FIFO (size>1) that absorbs
//                backpressure so the upstream never stalls when the
//                downstream is momentarily unready.
//
// outReg != 0 adds an extra pipeline register on the output path.
// ---------------------------------------------------------------------------
class VxElasticBuffer(
  val dataw  : Int = 1,
  val size   : Int = 1,
  val outReg : Int = 0
) extends Module {

  val io = IO(new Bundle {
    val validIn  = Input(Bool())
    val readyIn  = Output(Bool())
    val dataIn   = Input(UInt(dataw.W))
    val validOut = Output(Bool())
    val readyOut = Input(Bool())
    val dataOut  = Output(UInt(dataw.W))
  })

  if (size == 0) {
    // Pass-through: no storage at all
    io.validOut := io.validIn
    io.dataOut  := io.dataIn
    io.readyIn  := io.readyOut

  } else {
    // ------------------------------------------------------------------
    // Skid / FIFO implementation.
    //
    // We keep a small FIFO of depth `size`.  The FIFO decouples the
    // upstream handshake from the downstream handshake.
    //
    // For the common case (size == 1) this degenerates to the classic
    // two-register skid buffer (primary register + overflow register).
    // ------------------------------------------------------------------

    val depth = math.max(size, 1)
    val addrW = log2Ceil(depth + 1)

    // Storage: circular buffer
    val mem    = Reg(Vec(depth, UInt(dataw.W)))
    val wrPtr  = RegInit(0.U(addrW.W))
    val rdPtr  = RegInit(0.U(addrW.W))
    val count  = RegInit(0.U(addrW.W))

    val full  = count === depth.U
    val empty = count === 0.U

    val push = io.validIn  && io.readyIn
    val pop  = io.validOut && io.readyOut

    io.readyIn := !full

    when (push && !pop) {
      count := count + 1.U
    }.elsewhen (!push && pop) {
      count := count - 1.U
    }

    when (push) {
      mem(wrPtr) := io.dataIn
      wrPtr := Mux(wrPtr === (depth - 1).U, 0.U, wrPtr + 1.U)
    }
    when (pop) {
      rdPtr := Mux(rdPtr === (depth - 1).U, 0.U, rdPtr + 1.U)
    }

    if (outReg != 0) {
      // Add an output pipeline register: once we take something from the
      // FIFO we hold it in a register and present it downstream.
      val outValid = RegInit(false.B)
      val outData  = Reg(UInt(dataw.W))

      val canLoad = !outValid || io.readyOut

      when (canLoad && !empty) {
        outValid := true.B
        outData  := mem(rdPtr)
      }.elsewhen (io.readyOut) {
        outValid := false.B
      }

      io.validOut := outValid
      io.dataOut  := outData
    } else {
      io.validOut := !empty
      io.dataOut  := mem(rdPtr)
    }
  }
}

// ---------------------------------------------------------------------------
// VxStreamArb
//
// Multi-input / multi-output stream arbiter.
//
// When numInputs > numOutputs:
//   Each output is fed by an arbiter that selects from (numInputs/numOutputs)
//   candidate inputs.  selOut(o) carries the index of the selected input
//   within its slice of the input space.
//
// When numInputs < numOutputs:
//   Each input is broadcast to (numOutputs/numInputs) candidate outputs.
//   An arbiter selects which output slice is active.
//
// When numInputs == numOutputs:
//   Pure pass-through (per-lane elastic buffer if outBuf != 0).
//
// Parameters
//   arbiter  one of "P" (priority), "R" (round-robin), "M" (matrix), "C" (cyclic)
//   sticky   hold the grant while the granted request stays asserted
//   outBuf   > 0 inserts an elastic buffer on every output
// ---------------------------------------------------------------------------
class VxStreamArb(
  val numInputs  : Int    = 1,
  val numOutputs : Int    = 1,
  val dataw      : Int    = 1,
  val sticky     : Int    = 0,
  val arbiter    : String = "R",
  val outBuf     : Int    = 0
) extends Module {

  // Derived geometry
  val numReqs    : Int = if (numInputs > numOutputs)
                           (numInputs  + numOutputs - 1) / numOutputs
                         else
                           (numOutputs + numInputs  - 1) / numInputs
  val selCount   : Int = math.min(numInputs, numOutputs)
  val logNumReqs : Int = log2Ceil(math.max(numReqs, 2))
  val numReqsW   : Int = math.max(1, logNumReqs)

  val io = IO(new Bundle {
    val validIn  = Input(Vec(numInputs,  Bool()))
    val dataIn   = Input(Vec(numInputs,  UInt(dataw.W)))
    val readyIn  = Output(Vec(numInputs, Bool()))

    val validOut = Output(Vec(numOutputs, Bool()))
    val dataOut  = Output(Vec(numOutputs, UInt(dataw.W)))
    val readyOut = Input(Vec(numOutputs,  Bool()))

    // selOut(i) = index within the request group that was selected
    val selOut   = Output(Vec(selCount, UInt(numReqsW.W)))
  })

  if (numInputs > numOutputs) {
    // ----------------------------------------------------------------
    // Fan-in: multiple inputs → fewer outputs.
    // For each output port we have numReqs candidate inputs, indexed
    // as: input[r * numOutputs + o] feeds output[o] with request r.
    // ----------------------------------------------------------------

    // Arbiter over numReqs request groups (one per output group)
    val arb = Module(new VxGenericArbiter(
      numReqs = numReqs,
      arbType = arbiter,
      sticky  = sticky
    ))

    // Build request vector: group r is active if any output in that group
    // has a valid input
    val arbRequests = Wire(UInt(numReqs.W))
    val reqBits = Wire(Vec(numReqs, Bool()))
    for (r <- 0 until numReqs) {
      val anyValid = (0 until numOutputs).map { o =>
        val i = r * numOutputs + o
        if (i < numInputs) io.validIn(i) else false.B
      }.reduce(_ || _)
      reqBits(r) := anyValid
    }
    arbRequests := reqBits.asUInt

    arb.io.requests := arbRequests

    val arbIndex  = arb.io.grantIndex
    val arbOnehot = arb.io.grantOnehot
    val arbValid  = arb.io.grantValid

    // Per-output wire buses before the output buffers
    val validOutW = Wire(Vec(numOutputs, Bool()))
    val dataOutW  = Wire(Vec(numOutputs, UInt(dataw.W)))
    val readyOutW = Wire(Vec(numOutputs, Bool()))

    for (o <- 0 until numOutputs) {
      // Collect per-request valid/data for this output
      val validInW = Wire(Vec(numReqs, Bool()))
      val dataInW  = Wire(Vec(numReqs, UInt(dataw.W)))
      for (r <- 0 until numReqs) {
        val i = r * numOutputs + o
        if (i < numInputs) {
          validInW(r) := io.validIn(i)
          dataInW(r)  := io.dataIn(i)
        } else {
          validInW(r) := false.B
          dataInW(r)  := 0.U(dataw.W)
        }
      }
      // Valid on this output: arbiter has a valid grant and that group has
      // a valid input for this output
      if (numOutputs == 1) {
        validOutW(o) := arbValid
      } else {
        validOutW(o) := (validInW.asUInt & arbOnehot).orR
      }
      dataOutW(o) := dataInW(arbIndex)
    }

    // ready_in: input i is ready when the arbiter selects its group and
    // its output is ready
    for (i <- 0 until numInputs) {
      val o = i % numOutputs
      val r = i / numOutputs
      io.readyIn(i) := readyOutW(o) && arbOnehot(r)
    }

    // Arbiter fires when any output buffer is ready to accept
    arb.io.grantReady := readyOutW.asUInt.orR

    // Output buffers
    for (o <- 0 until numOutputs) {
      if (outBuf > 0) {
        val buf = Module(new VxElasticBuffer(dataw = logNumReqs + dataw, size = outBuf))
        buf.io.validIn := validOutW(o)
        buf.io.dataIn  := Cat(arbIndex, dataOutW(o))
        readyOutW(o)   := buf.io.readyIn
        val outCat      = buf.io.dataOut
        io.validOut(o) := buf.io.validOut
        io.dataOut(o)  := outCat(dataw - 1, 0)
        io.selOut(o)   := outCat(logNumReqs + dataw - 1, dataw)
        buf.io.readyOut := io.readyOut(o)
      } else {
        io.validOut(o) := validOutW(o)
        io.dataOut(o)  := dataOutW(o)
        io.selOut(o)   := arbIndex
        readyOutW(o)   := io.readyOut(o)
      }
    }

  } else if (numInputs < numOutputs) {
    // ----------------------------------------------------------------
    // Fan-out: fewer inputs → multiple outputs.
    // Output[r * numInputs + i] is driven by input[i] when the arbiter
    // selects group r.
    // ----------------------------------------------------------------

    val arb = Module(new VxGenericArbiter(
      numReqs = numReqs,
      arbType = arbiter,
      sticky  = sticky
    ))

    // Requests: group r is ready when any output in that group is ready
    val reqBits = Wire(Vec(numReqs, Bool()))
    for (r <- 0 until numReqs) {
      val anyReady = (0 until numInputs).map { i =>
        val o = r * numInputs + i
        if (o < numOutputs) io.readyOut(o) else false.B
      }.reduce(_ || _)
      reqBits(r) := anyReady
    }
    arb.io.requests := reqBits.asUInt

    val arbIndex  = arb.io.grantIndex
    val arbOnehot = arb.io.grantOnehot
    val arbValid  = arb.io.grantValid

    val validOutW = Wire(Vec(numOutputs, Bool()))
    val dataOutW  = Wire(Vec(numOutputs, UInt(dataw.W)))
    val readyOutW = Wire(Vec(numOutputs, Bool()))

    for (o <- 0 until numOutputs) {
      val i = o % numInputs
      val r = o / numInputs
      validOutW(o) := io.validIn(i) && arbOnehot(r)
      dataOutW(o)  := io.dataIn(i)
    }

    for (i <- 0 until numInputs) {
      val readyBits = Wire(Vec(numReqs, Bool()))
      for (r <- 0 until numReqs) {
        val o = r * numInputs + i
        readyBits(r) := readyOutW(o)
      }
      if (numInputs == 1) {
        io.readyIn(i) := arbValid
      } else {
        io.readyIn(i) := (readyBits.asUInt & arbOnehot).orR
      }
    }

    // Arbiter fires when any valid input is present
    arb.io.grantReady := io.validIn.asUInt.orR

    for (o <- 0 until numOutputs) {
      if (outBuf > 0) {
        val buf = Module(new VxElasticBuffer(dataw = dataw, size = outBuf))
        buf.io.validIn  := validOutW(o)
        buf.io.dataIn   := dataOutW(o)
        readyOutW(o)    := buf.io.readyIn
        io.validOut(o)  := buf.io.validOut
        io.dataOut(o)   := buf.io.dataOut
        buf.io.readyOut := io.readyOut(o)
      } else {
        io.validOut(o) := validOutW(o)
        io.dataOut(o)  := dataOutW(o)
        readyOutW(o)   := io.readyOut(o)
      }
    }

    for (i <- 0 until selCount) {
      io.selOut(i) := arbIndex
    }

  } else {
    // ----------------------------------------------------------------
    // Pass-through: numInputs == numOutputs
    // ----------------------------------------------------------------
    for (o <- 0 until numOutputs) {
      if (outBuf > 0) {
        val buf = Module(new VxElasticBuffer(dataw = dataw, size = outBuf))
        buf.io.validIn  := io.validIn(o)
        buf.io.dataIn   := io.dataIn(o)
        io.readyIn(o)   := buf.io.readyIn
        io.validOut(o)  := buf.io.validOut
        io.dataOut(o)   := buf.io.dataOut
        buf.io.readyOut := io.readyOut(o)
      } else {
        io.validOut(o) := io.validIn(o)
        io.dataOut(o)  := io.dataIn(o)
        io.readyIn(o)  := io.readyOut(o)
      }
      io.selOut(o) := 0.U
    }
  }
}

// ---------------------------------------------------------------------------
// VxStreamSwitch
//
// Steered switch: unlike VxStreamArb the steering is explicit (sel_in)
// rather than arbitrated.  The caller provides the routing index.
//
// When numInputs > numOutputs:
//   sel_in(o) selects which of the numReqs candidate inputs is routed to
//   output o.  Input i is ready when sel_in(i % numOutputs) == i / numOutputs
//   and the corresponding output is ready.
//
// When numOutputs > numInputs:
//   sel_in(i) selects which of the numReqs candidate outputs receives
//   input i.
//
// When numInputs == numOutputs:
//   Pass-through; sel_in is ignored.
// ---------------------------------------------------------------------------
class VxStreamSwitch(
  val numInputs  : Int = 1,
  val numOutputs : Int = 1,
  val dataw      : Int = 1,
  val outBuf     : Int = 0
) extends Module {

  val numReqs   : Int = if (numInputs > numOutputs)
                          (numInputs  + numOutputs - 1) / numOutputs
                        else
                          (numOutputs + numInputs  - 1) / numInputs
  val selCount  : Int = math.min(numInputs, numOutputs)
  val logNumReqs: Int = math.max(1, log2Ceil(math.max(numReqs, 2)))

  val io = IO(new Bundle {
    val selIn    = Input(Vec(selCount, UInt(logNumReqs.W)))

    val validIn  = Input(Vec(numInputs,  Bool()))
    val dataIn   = Input(Vec(numInputs,  UInt(dataw.W)))
    val readyIn  = Output(Vec(numInputs, Bool()))

    val validOut = Output(Vec(numOutputs, Bool()))
    val dataOut  = Output(Vec(numOutputs, UInt(dataw.W)))
    val readyOut = Input(Vec(numOutputs,  Bool()))
  })

  // Internal wires before elastic buffers
  val validOutW = Wire(Vec(numOutputs, Bool()))
  val dataOutW  = Wire(Vec(numOutputs, UInt(dataw.W)))
  val readyOutW = Wire(Vec(numOutputs, Bool()))

  if (numInputs > numOutputs) {
    // sel_in(o) steers output o to input (sel_in(o) * numOutputs + o)
    for (o <- 0 until numOutputs) {
      // Collect per-request valid/data for this output
      val validInW = Wire(Vec(numReqs, Bool()))
      val dataInW  = Wire(Vec(numReqs, UInt(dataw.W)))
      for (r <- 0 until numReqs) {
        val i = r * numOutputs + o
        if (i < numInputs) {
          validInW(r) := io.validIn(i)
          dataInW(r)  := io.dataIn(i)
        } else {
          validInW(r) := false.B
          dataInW(r)  := 0.U(dataw.W)
        }
      }
      validOutW(o) := validInW(io.selIn(o))
      dataOutW(o)  := dataInW(io.selIn(o))
    }

    // ready_in: for each input i, it is ready when the output it belongs
    // to has selected it
    for (i <- 0 until numInputs) {
      val o = i % numOutputs
      val r = i / numOutputs
      io.readyIn(i) := readyOutW(o) && (io.selIn(o) === r.U)
    }

  } else if (numOutputs > numInputs) {
    // sel_in(i) steers input i to output (sel_in(i) * numInputs + i)
    // Use a Mux-tree per output so there is a single driver per element.
    for (o <- 0 until numOutputs) {
      // Determine which input feeds this output (if any)
      val i_of_o = o % numInputs
      val r_of_o = o / numInputs
      validOutW(o) := io.validIn(i_of_o) && (io.selIn(i_of_o) === r_of_o.U)
      dataOutW(o)  := io.dataIn(i_of_o)
    }
    for (i <- 0 until numInputs) {
      // ready_in(i): the selected output is ready
      val readyBits = Wire(Vec(numReqs, Bool()))
      for (r <- 0 until numReqs) {
        val o = r * numInputs + i
        readyBits(r) := (if (o < numOutputs) readyOutW(o) else false.B)
      }
      io.readyIn(i) := readyBits(io.selIn(i))
    }

  } else {
    // Pass-through
    for (i <- 0 until numOutputs) {
      validOutW(i) := io.validIn(i)
      dataOutW(i)  := io.dataIn(i)
      io.readyIn(i) := readyOutW(i)
    }
  }

  // Elastic output buffers
  for (o <- 0 until numOutputs) {
    if (outBuf > 0) {
      val buf = Module(new VxElasticBuffer(dataw = dataw, size = outBuf))
      buf.io.validIn  := validOutW(o)
      buf.io.dataIn   := dataOutW(o)
      readyOutW(o)    := buf.io.readyIn
      io.validOut(o)  := buf.io.validOut
      io.dataOut(o)   := buf.io.dataOut
      buf.io.readyOut := io.readyOut(o)
    } else {
      io.validOut(o) := validOutW(o)
      io.dataOut(o)  := dataOutW(o)
      readyOutW(o)   := io.readyOut(o)
    }
  }
}

// ---------------------------------------------------------------------------
// VxStreamXbar
//
// Crossbar switch: each input carries an output-select field (sel_in).
// Multiple inputs may target the same output; an arbiter resolves
// conflicts per output.
//
// The sel_out output of each arbitrated port reports which input won.
//
// collisions counts the number of input pairs per cycle that target the
// same output simultaneously.
// ---------------------------------------------------------------------------
class VxStreamXbar(
  val numInputs   : Int    = 4,
  val numOutputs  : Int    = 4,
  val dataw       : Int    = 4,
  val arbiter     : String = "R",
  val outBuf      : Int    = 0,
  val perfCtrBits : Int    = 8
) extends Module {

  val inWidth  = log2Ceil(math.max(numInputs,  2))
  val outWidth = log2Ceil(math.max(numOutputs, 2))

  val io = IO(new Bundle {
    val validIn   = Input(Vec(numInputs,  Bool()))
    val dataIn    = Input(Vec(numInputs,  UInt(dataw.W)))
    val selIn     = Input(Vec(numInputs,  UInt(outWidth.W)))
    val readyIn   = Output(Vec(numInputs, Bool()))

    val validOut  = Output(Vec(numOutputs, Bool()))
    val dataOut   = Output(Vec(numOutputs, UInt(dataw.W)))
    val selOut    = Output(Vec(numOutputs, UInt(inWidth.W)))
    val readyOut  = Input(Vec(numOutputs,  Bool()))

    val collisions = Output(UInt(perfCtrBits.W))
  })

  if (numInputs == 1 && numOutputs == 1) {
    // Trivial pass-through
    val buf = Module(new VxElasticBuffer(dataw = dataw, size = math.max(outBuf, 0)))
    buf.io.validIn  := io.validIn(0)
    buf.io.dataIn   := io.dataIn(0)
    io.readyIn(0)   := buf.io.readyIn
    io.validOut(0)  := buf.io.validOut
    io.dataOut(0)   := buf.io.dataOut
    io.selOut(0)    := 0.U
    buf.io.readyOut := io.readyOut(0)

  } else if (numInputs == 1) {
    // Single input demuxed to numOutputs
    // Output o is valid when selIn(0) == o
    val validOutW = Wire(Vec(numOutputs, Bool()))
    val readyOutW = Wire(Vec(numOutputs, Bool()))

    for (o <- 0 until numOutputs) {
      validOutW(o) := io.validIn(0) && (io.selIn(0) === o.U)
    }
    io.readyIn(0) := readyOutW(io.selIn(0))

    for (o <- 0 until numOutputs) {
      if (outBuf > 0) {
        val buf = Module(new VxElasticBuffer(dataw = dataw, size = outBuf))
        buf.io.validIn  := validOutW(o)
        buf.io.dataIn   := io.dataIn(0)
        readyOutW(o)    := buf.io.readyIn
        io.validOut(o)  := buf.io.validOut
        io.dataOut(o)   := buf.io.dataOut
        buf.io.readyOut := io.readyOut(o)
      } else {
        io.validOut(o) := validOutW(o)
        io.dataOut(o)  := io.dataIn(0)
        readyOutW(o)   := io.readyOut(o)
      }
      io.selOut(o) := 0.U
    }

  } else if (numOutputs == 1) {
    // Multiple inputs arbitrated to single output
    val arb = Module(new VxStreamArb(
      numInputs  = numInputs,
      numOutputs = 1,
      dataw      = dataw,
      arbiter    = arbiter,
      outBuf     = outBuf
    ))
    arb.io.validIn  := io.validIn
    arb.io.dataIn   := io.dataIn
    io.readyIn      := arb.io.readyIn
    io.validOut(0)  := arb.io.validOut(0)
    io.dataOut(0)   := arb.io.dataOut(0)
    io.selOut(0)    := arb.io.selOut(0)
    arb.io.readyOut(0) := io.readyOut(0)

  } else {
    // General case: numInputs > 1, numOutputs > 1
    //
    // per_output_valid_in(i)(o): input i targets output o
    // per_output_valid_in_w(o)(i): transposed – for output o, which inputs
    //                               target it
    val perOutValidIn  = Wire(Vec(numInputs,  Vec(numOutputs, Bool())))
    val perOutValidInW = Wire(Vec(numOutputs, Vec(numInputs,  Bool())))
    val perOutReadyIn  = Wire(Vec(numOutputs, Vec(numInputs,  Bool())))
    val perOutReadyInW = Wire(Vec(numInputs,  Vec(numOutputs, Bool())))

    // Demux each input's valid onto the output it selects
    for (i <- 0 until numInputs) {
      for (o <- 0 until numOutputs) {
        perOutValidIn(i)(o) := io.validIn(i) && (io.selIn(i) === o.U)
      }
    }

    // Transpose
    for (i <- 0 until numInputs) {
      for (o <- 0 until numOutputs) {
        perOutValidInW(o)(i) := perOutValidIn(i)(o)
        perOutReadyInW(i)(o) := perOutReadyIn(o)(i)
      }
    }

    // One arbiter per output
    for (o <- 0 until numOutputs) {
      val arb = Module(new VxStreamArb(
        numInputs  = numInputs,
        numOutputs = 1,
        dataw      = dataw,
        arbiter    = arbiter,
        outBuf     = outBuf
      ))
      arb.io.validIn := perOutValidInW(o)
      arb.io.dataIn  := io.dataIn
      for (i <- 0 until numInputs) {
        perOutReadyIn(o)(i) := arb.io.readyIn(i)
      }
      io.validOut(o) := arb.io.validOut(0)
      io.dataOut(o)  := arb.io.dataOut(0)
      io.selOut(o)   := arb.io.selOut(0)
      arb.io.readyOut(0) := io.readyOut(o)
    }

    // Ready-in: input i is ready if any of its targeted outputs accepted it
    for (i <- 0 until numInputs) {
      io.readyIn(i) := perOutReadyInW(i).asUInt.orR
    }
  }

  // -------------------------------------------------------------------
  // Collision counter
  // We count input pairs that simultaneously target the same output and
  // at least one of the pair is ready.  We accumulate the per-cycle count.
  // -------------------------------------------------------------------
  val perCycleCollision  = Wire(Vec(numInputs, Bool()))
  val perCycleCollisionR = RegNext(perCycleCollision)
  val collisionCount     = PopCount(perCycleCollisionR)
  val collisionsR        = RegInit(0.U(perfCtrBits.W))

  for (i <- 0 until numInputs) {
    val collides = Wire(Vec(numInputs, Bool()))
    for (j <- 0 until numInputs) {
      if (j > i) {
        collides(j) := io.validIn(i) && io.validIn(j) &&
                       (io.selIn(i) === io.selIn(j)) &&
                       (io.readyIn(i) || io.readyIn(j))
      } else {
        collides(j) := false.B
      }
    }
    perCycleCollision(i) := collides.asUInt.orR
  }

  collisionsR := collisionsR + collisionCount
  io.collisions := collisionsR
}

// ---------------------------------------------------------------------------
// VxStreamPack
//
// Packs multiple valid requests that share the same tag-select bits into a
// single output beat.  An arbiter picks the "lead" request; all other
// requests whose tag-select bits match the lead are folded into the same
// output word (mask_out indicates which lanes are present).
//
// Parameters
//   numReqs     number of input request lanes
//   dataWidth   per-lane data width
//   tagWidth    tag field width (full)
//   tagSelBits  LSBs of the tag that must match for packing (0 = pack all)
//   arbiter     arbitration policy for picking the lead request
//   outBuf      elastic buffer depth on the output
// ---------------------------------------------------------------------------
class VxStreamPack(
  val numReqs    : Int    = 1,
  val dataWidth  : Int    = 1,
  val tagWidth   : Int    = 1,
  val tagSelBits : Int    = 0,
  val arbiter    : String = "P",
  val outBuf     : Int    = 0
) extends Module {

  val io = IO(new Bundle {
    val validIn  = Input(Vec(numReqs,  Bool()))
    val dataIn   = Input(Vec(numReqs,  UInt(dataWidth.W)))
    val tagIn    = Input(Vec(numReqs,  UInt(tagWidth.W)))
    val readyIn  = Output(Vec(numReqs, Bool()))

    val validOut = Output(Bool())
    val maskOut  = Output(UInt(numReqs.W))
    val dataOut  = Output(Vec(numReqs, UInt(dataWidth.W)))
    val tagOut   = Output(UInt(tagWidth.W))
    val readyOut = Input(Bool())
  })

  if (numReqs == 1) {
    // Trivial pass-through
    io.validOut  := io.validIn(0)
    io.maskOut   := 1.U
    io.dataOut(0) := io.dataIn(0)
    io.tagOut    := io.tagIn(0)
    io.readyIn(0) := io.readyOut

  } else {
    val logNumReqs = log2Ceil(numReqs)

    val arb = Module(new VxGenericArbiter(
      numReqs  = numReqs,
      arbType  = arbiter,
      sticky   = 0
    ))
    arb.io.requests := io.validIn.asUInt

    // Grant: the lead request index
    val grantIndex  = arb.io.grantIndex
    val grantValid  = arb.io.grantValid
    val grantReady  = Wire(Bool())
    arb.io.grantReady := grantReady

    // Tag-select field of the lead request
    val tagSel = io.tagIn(grantIndex)

    // Determine which requests match the lead's tag-select bits
    val tagMatches = Wire(Vec(numReqs, Bool()))
    for (i <- 0 until numReqs) {
      if (tagSelBits == 0) {
        tagMatches(i) := true.B
      } else {
        tagMatches(i) := io.tagIn(i)(tagSelBits - 1, 0) === tagSel(tagSelBits - 1, 0)
      }
    }

    // ready_in: a lane is ready when the arbiter fires and it matches
    for (i <- 0 until numReqs) {
      io.readyIn(i) := grantReady && tagMatches(i)
    }

    val maskSel = io.validIn.asUInt & tagMatches.asUInt

    // Pack into one payload word and feed through an elastic buffer
    val internalDataW = numReqs + tagWidth + numReqs * dataWidth

    val buf = Module(new VxElasticBuffer(dataw = internalDataW, size = math.max(outBuf, 0)))
    buf.io.validIn := grantValid
    buf.io.dataIn  := Cat(maskSel,
                          tagSel,
                          VecInit(io.dataIn.reverse).asUInt)
    grantReady     := buf.io.readyIn

    buf.io.readyOut := io.readyOut
    io.validOut     := buf.io.validOut

    val outCat = buf.io.dataOut
    // Layout: [numReqs*dataWidth-1 : 0] = dataOut lanes
    //         [numReqs*dataWidth + tagWidth - 1 : numReqs*dataWidth] = tag
    //         [numReqs*dataWidth + tagWidth + numReqs - 1 : numReqs*dataWidth + tagWidth] = mask
    val dataFlat = outCat(numReqs * dataWidth - 1, 0)
    val tagFlat  = outCat(numReqs * dataWidth + tagWidth - 1, numReqs * dataWidth)
    val maskFlat = outCat(internalDataW - 1, numReqs * dataWidth + tagWidth)

    io.tagOut  := tagFlat
    io.maskOut := maskFlat
    for (i <- 0 until numReqs) {
      io.dataOut(i) := dataFlat(dataWidth * (i + 1) - 1, dataWidth * i)
    }
  }
}

// ---------------------------------------------------------------------------
// VxStreamUnpack
//
// Unpacks a single input beat (with mask) into individual per-lane outputs.
// Each set lane in mask_in generates an independent output handshake.
// The module tracks which lanes have already been accepted (rem_mask) and
// only advances to the next input beat when all requested lanes have fired.
// ---------------------------------------------------------------------------
class VxStreamUnpack(
  val numReqs   : Int = 1,
  val dataWidth : Int = 1,
  val tagWidth  : Int = 1,
  val outBuf    : Int = 0
) extends Module {

  val io = IO(new Bundle {
    val validIn  = Input(Bool())
    val maskIn   = Input(UInt(numReqs.W))
    val dataIn   = Input(Vec(numReqs, UInt(dataWidth.W)))
    val tagIn    = Input(UInt(tagWidth.W))
    val readyIn  = Output(Bool())

    val validOut = Output(Vec(numReqs, Bool()))
    val dataOut  = Output(Vec(numReqs, UInt(dataWidth.W)))
    val tagOut   = Output(Vec(numReqs, UInt(tagWidth.W)))
    val readyOut = Input(Vec(numReqs,  Bool()))
  })

  if (numReqs == 1) {
    // Trivial pass-through
    io.validOut(0)  := io.validIn
    io.dataOut(0)   := io.dataIn(0)
    io.tagOut(0)    := io.tagIn
    io.readyIn      := io.readyOut(0)

  } else {
    // rem_mask_r: bits still to be sent for the current input beat
    val remMaskR = RegInit(((1 << numReqs) - 1).U(numReqs.W))

    // Per-lane elastic buffers and their ready signals
    val readyOutW = Wire(Vec(numReqs, Bool()))

    // A lane fires (pops from rem_mask) when its elastic buffer accepts
    val remMaskN = remMaskR & ~readyOutW.asUInt
    val sentAll  = (io.maskIn & remMaskN) === 0.U

    when (io.validIn) {
      remMaskR := Mux(sentAll, ((1 << numReqs) - 1).U, remMaskN)
    }

    io.readyIn := sentAll

    for (i <- 0 until numReqs) {
      val buf = Module(new VxElasticBuffer(dataw = dataWidth + tagWidth, size = math.max(outBuf, 0)))
      buf.io.validIn  := io.validIn && io.maskIn(i) && remMaskR(i)
      buf.io.dataIn   := Cat(io.dataIn(i), io.tagIn)
      readyOutW(i)    := buf.io.readyIn
      buf.io.readyOut := io.readyOut(i)

      io.validOut(i) := buf.io.validOut
      // Layout: tagWidth LSBs = tag, upper bits = data
      io.tagOut(i)   := buf.io.dataOut(tagWidth - 1, 0)
      io.dataOut(i)  := buf.io.dataOut(dataWidth + tagWidth - 1, tagWidth)
    }
  }
}

// ---------------------------------------------------------------------------
// VxStreamOmega
//
// Multi-stage Omega (perfect-shuffle) interconnect network.
//
// For networks smaller than or equal to RADIX a plain crossbar is used.
// For larger networks the topology is:
//   - NUM_STAGES = log_RADIX(N_INPUTS) stages
//   - Each stage contains N_INPUTS / RADIX switches of size RADIX×RADIX
//   - Each switch is a VxStreamXbar
//   - Inputs are connected to stage-0 switches via a bit-reversal permutation
//   - Adjacent stages are wired via a perfect-shuffle permutation
//   - The outputs of the last stage drive the module outputs
//
// The routing tag carried with each flit is the destination output address
// encoded in base-RADIX.  Each stage peels off the most-significant RADIX_LG
// bits of the destination to steer the flit through that stage's switch.
//
// collisions counts backpressure/collision events inside the network.
// ---------------------------------------------------------------------------
class VxStreamOmega(
  val numInputs   : Int    = 4,
  val numOutputs  : Int    = 4,
  val radix       : Int    = 2,
  val dataw       : Int    = 4,
  val arbiter     : String = "R",
  val outBuf      : Int    = 0,
  val perfCtrBits : Int    = 32
) extends Module {

  val inWidth  = log2Ceil(math.max(numInputs,  2))
  val outWidth = log2Ceil(math.max(numOutputs, 2))

  val io = IO(new Bundle {
    val validIn   = Input(Vec(numInputs,  Bool()))
    val dataIn    = Input(Vec(numInputs,  UInt(dataw.W)))
    val selIn     = Input(Vec(numInputs,  UInt(outWidth.W)))
    val readyIn   = Output(Vec(numInputs, Bool()))

    val validOut  = Output(Vec(numOutputs, Bool()))
    val dataOut   = Output(Vec(numOutputs, UInt(dataw.W)))
    val selOut    = Output(Vec(numOutputs, UInt(inWidth.W)))
    val readyOut  = Input(Vec(numOutputs,  Bool()))

    val collisions = Output(UInt(perfCtrBits.W))
  })

  require(radix >= 2 && (radix & (radix - 1)) == 0,
    "VxStreamOmega: radix must be a power of 2")

  if (numInputs <= radix && numOutputs <= radix) {
    // Fall back to a plain crossbar for small networks
    val xbar = Module(new VxStreamXbar(
      numInputs   = numInputs,
      numOutputs  = numOutputs,
      dataw       = dataw,
      arbiter     = arbiter,
      outBuf      = outBuf,
      perfCtrBits = perfCtrBits
    ))
    xbar.io.validIn  := io.validIn
    xbar.io.dataIn   := io.dataIn
    xbar.io.selIn    := io.selIn
    io.readyIn       := xbar.io.readyIn
    io.validOut      := xbar.io.validOut
    io.dataOut       := xbar.io.dataOut
    io.selOut        := xbar.io.selOut
    xbar.io.readyOut := io.readyOut
    io.collisions    := xbar.io.collisions

  } else {
    // Full Omega network
    val radixLg    = log2Ceil(radix)
    val nInputsM   = math.max(numInputs, numOutputs)
    // Number of base-radix digits needed to address nInputsM entries
    val nInputsLg  = (log2Ceil(math.max(nInputsM, 2)) + radixLg - 1) / radixLg
    val nInputs    = math.pow(radix, nInputsLg).toInt   // padded network size
    val numStages  = nInputsLg                           // = log_radix(nInputs)
    val numSwitches = nInputs / radix

    // Internal payload: sel_in tag (for routing) + data + sel_out (source id)
    val tagInW  = nInputsLg  // routing tag width (base-radix digits)
    val payloadW = tagInW + dataw + inWidth  // total flit width

    // Network wires: [stage][switch][port]
    // valid, ready, and payload (Cat of routing-tag, data, source-id)
    val swValidIn  = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, Bool()))))
    val swDataIn   = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, UInt(payloadW.W)))))
    val swSelIn    = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, UInt(radixLg.W)))))
    val swReadyIn  = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, Bool()))))
    val swValidOut = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, Bool()))))
    val swDataOut  = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, UInt(payloadW.W)))))
    val swReadyOut = Wire(Vec(numStages, Vec(numSwitches, Vec(radix, Bool()))))

    // Helper: bit-reversal left-rotate by 1 on nInputsLg bits
    def bitRotLeft1(x: Int, bits: Int): Int =
      ((x << 1) | (x >> (bits - 1))) & ((1 << bits) - 1)

    // ---------------------------------------------------------------
    // Connect inputs to stage 0 via bit-reversal permutation
    // ---------------------------------------------------------------
    for (i <- 0 until nInputs) {
      val dstIdx  = bitRotLeft1(i, nInputsLg)
      val sw      = dstIdx / radix
      val port    = dstIdx % radix
      if (i < numInputs) {
        swValidIn(0)(sw)(port) := io.validIn(i)
        // payload = Cat(routing-tag, data, source-id)
        swDataIn(0)(sw)(port)  := Cat(io.selIn(i), io.dataIn(i), i.U(inWidth.W))
        io.readyIn(i)          := swReadyIn(0)(sw)(port)
      } else {
        swValidIn(0)(sw)(port) := false.B
        swDataIn(0)(sw)(port)  := 0.U
      }
    }

    // ---------------------------------------------------------------
    // Routing: each port's steering is the top RADIX_LG bits of the
    // routing tag for that stage (peeled from MSB downward).
    // The routing tag occupies the top nInputsLg bits of the payload.
    // ---------------------------------------------------------------
    for (stage <- 0 until numStages) {
      for (sw <- 0 until numSwitches) {
        for (port <- 0 until radix) {
          val tagMsb = payloadW - 1
          val tagLsb = dataw + inWidth
          val stageShift = (numStages - 1 - stage) * radixLg
          swSelIn(stage)(sw)(port) :=
            swDataIn(stage)(sw)(port)(tagMsb, tagLsb)(stageShift + radixLg - 1, stageShift)
        }
      }
    }

    // ---------------------------------------------------------------
    // Connect adjacent stages via perfect-shuffle permutation
    // ---------------------------------------------------------------
    for (stage <- 0 until numStages - 1) {
      for (sw <- 0 until numSwitches) {
        for (port <- 0 until radix) {
          val lane    = sw * radix + port
          val dstLane = bitRotLeft1(lane, nInputsLg)
          val dstSw   = dstLane / radix
          val dstPort = dstLane % radix
          swValidIn(stage + 1)(dstSw)(dstPort) := swValidOut(stage)(sw)(port)
          swDataIn(stage + 1)(dstSw)(dstPort)  := swDataOut(stage)(sw)(port)
          swReadyOut(stage)(sw)(port)           := swReadyIn(stage + 1)(dstSw)(dstPort)
        }
      }
    }

    // ---------------------------------------------------------------
    // Instantiate switches
    // ---------------------------------------------------------------
    for (sw <- 0 until numSwitches) {
      for (stage <- 0 until numStages) {
        val xbar = Module(new VxStreamXbar(
          numInputs   = radix,
          numOutputs  = radix,
          dataw       = payloadW,
          arbiter     = arbiter,
          outBuf      = outBuf,
          perfCtrBits = perfCtrBits
        ))
        xbar.io.validIn  := swValidIn(stage)(sw)
        xbar.io.dataIn   := swDataIn(stage)(sw)
        xbar.io.selIn    := swSelIn(stage)(sw)
        for (p <- 0 until radix) {
          swReadyIn(stage)(sw)(p)  := xbar.io.readyIn(p)
          swValidOut(stage)(sw)(p) := xbar.io.validOut(p)
          swDataOut(stage)(sw)(p)  := xbar.io.dataOut(p)
        }
        xbar.io.readyOut := swReadyOut(stage)(sw)
        // selOut and collisions from individual switches are unused here;
        // we aggregate collisions separately below.
      }
    }

    // ---------------------------------------------------------------
    // Connect last-stage outputs
    // ---------------------------------------------------------------
    for (i <- 0 until nInputs) {
      val sw   = i / radix
      val port = i % radix
      if (i < numOutputs) {
        io.validOut(i)                          := swValidOut(numStages - 1)(sw)(port)
        io.dataOut(i)                           := swDataOut(numStages - 1)(sw)(port)(dataw + inWidth - 1, inWidth)
        io.selOut(i)                            := swDataOut(numStages - 1)(sw)(port)(inWidth - 1, 0)
        swReadyOut(numStages - 1)(sw)(port)     := io.readyOut(i)
      } else {
        swReadyOut(numStages - 1)(sw)(port) := false.B
      }
    }

    // ---------------------------------------------------------------
    // Collision counter: count per-cycle conflict events across all
    // switch input ports, then accumulate.
    // ---------------------------------------------------------------
    val totalPorts = numStages * numSwitches * radix
    val perCycleColl  = Wire(Vec(totalPorts, Bool()))
    val perCycleCollR = RegNext(perCycleColl)
    val collCount     = PopCount(perCycleCollR)
    val collisionsR   = RegInit(0.U(perfCtrBits.W))

    var portIdx = 0
    for (stage <- 0 until numStages) {
      for (sw <- 0 until numSwitches) {
        for (portA <- 0 until radix) {
          var collBit: Bool = false.B
          for (portB <- portA + 1 until radix) {
            val conflict = swValidIn(stage)(sw)(portA) &&
                           swValidIn(stage)(sw)(portB) &&
                           (swSelIn(stage)(sw)(portA) === swSelIn(stage)(sw)(portB)) &&
                           (swReadyIn(stage)(sw)(portA) || swReadyIn(stage)(sw)(portB))
            collBit = collBit || conflict
          }
          perCycleColl(portIdx) := collBit
          portIdx += 1
        }
      }
    }

    collisionsR    := collisionsR + collCount
    io.collisions  := collisionsR
  }
}

// ---------------------------------------------------------------------------
// VxStreamXpoint
//
// Crosspoint switch: a purely wired (no arbitration) routing fabric.
//
// Two operating modes controlled by outDriven:
//
//   outDriven = false (default, input-driven):
//     Each input i drives output sel_in(i).  The caller must ensure no two
//     inputs share the same destination; behaviour is undefined otherwise.
//     ready_in(i) = ready_out_w(sel_in(i))
//
//   outDriven = true (output-driven):
//     Each output o is fed from input sel_in(o).
//     ready_in(i) = OR of all ready_out_w(o) where sel_in(o) == i
// ---------------------------------------------------------------------------
class VxStreamXpoint(
  val numInputs  : Int = 1,
  val numOutputs : Int = 1,
  val dataw      : Int = 1,
  val outDriven  : Int = 0,
  val outBuf     : Int = 0
) extends Module {

  val selSrc  = if (outDriven != 0) numOutputs else numInputs
  val selDst  = if (outDriven != 0) numInputs  else numOutputs
  val selW    = log2Ceil(math.max(selDst, 2))

  val io = IO(new Bundle {
    val selIn    = Input(Vec(selSrc, UInt(selW.W)))

    val validIn  = Input(Vec(numInputs,  Bool()))
    val dataIn   = Input(Vec(numInputs,  UInt(dataw.W)))
    val readyIn  = Output(Vec(numInputs, Bool()))

    val validOut = Output(Vec(numOutputs, Bool()))
    val dataOut  = Output(Vec(numOutputs, UInt(dataw.W)))
    val readyOut = Input(Vec(numOutputs,  Bool()))
  })

  // Pre-buffer combinational wires
  val validOutW = Wire(Vec(numOutputs, Bool()))
  val dataOutW  = Wire(Vec(numOutputs, UInt(dataw.W)))
  val readyOutW = Wire(Vec(numOutputs, Bool()))

  if (outDriven != 0) {
    // Output-driven: output o grabs from input sel_in(o)
    for (o <- 0 until numOutputs) {
      validOutW(o) := io.validIn(io.selIn(o))
      dataOutW(o)  := io.dataIn(io.selIn(o))
    }
    // ready_in(i) = OR of readyOutW(o) for all o that select input i
    for (i <- 0 until numInputs) {
      val bits = (0 until numOutputs).map { o =>
        Mux(io.selIn(o) === i.U, readyOutW(o), false.B)
      }
      io.readyIn(i) := bits.reduce(_ || _)
    }

  } else {
    // Input-driven: input i drives output sel_in(i).
    // Default: outputs are invalid; overridden by the input loops below.
    for (o <- 0 until numOutputs) {
      validOutW(o) := false.B
      dataOutW(o)  := 0.U(dataw.W)
    }
    for (i <- 0 until numInputs) {
      // Chisel last-connect: when valid and sel matches, update the output.
      when (io.validIn(i)) {
        for (o <- 0 until numOutputs) {
          when (io.selIn(i) === o.U) {
            validOutW(o) := true.B
            dataOutW(o)  := io.dataIn(i)
          }
        }
      }
      io.readyIn(i) := readyOutW(io.selIn(i))
    }
  }

  // Elastic output buffers
  for (o <- 0 until numOutputs) {
    if (outBuf > 0) {
      val buf = Module(new VxElasticBuffer(dataw = dataw, size = outBuf))
      buf.io.validIn  := validOutW(o)
      buf.io.dataIn   := dataOutW(o)
      readyOutW(o)    := buf.io.readyIn
      io.validOut(o)  := buf.io.validOut
      io.dataOut(o)   := buf.io.dataOut
      buf.io.readyOut := io.readyOut(o)
    } else {
      io.validOut(o) := validOutW(o)
      io.dataOut(o)  := dataOutW(o)
      readyOutW(o)   := io.readyOut(o)
    }
  }
}

// ---------------------------------------------------------------------------
// VxPeSerializer
//
// Processing-element serializer: fans a wide input across NUM_PES processing
// elements, collects their outputs, and reassembles the full output vector.
//
// When NUM_LANES == NUM_PES the module is a simple latency pipeline:
//   - PE inputs are registered behind PE_REG pipeline stages.
//   - A LATENCY-deep valid/tag shift register tracks when PE outputs are ready.
//   - Results are passed through an output elastic buffer.
//
// When NUM_LANES > NUM_PES (NUM_LANES must be a multiple of NUM_PES):
//   - Input is serialized into BATCH_SIZE = NUM_LANES/NUM_PES time-slices.
//   - batch_in_idx advances each cycle valid_in is asserted.
//   - PE inputs are selected from the current input slice.
//   - PE outputs are accumulated into data_out_r.
//   - Output is valid once all BATCH_SIZE result slices have been captured.
// ---------------------------------------------------------------------------
class VxPeSerializer(
  val numLanes     : Int = 1,
  val numPes       : Int = 1,
  val latency      : Int = 1,
  val dataInWidth  : Int = 1,
  val dataOutWidth : Int = 1,
  val tagWidth     : Int = 0,
  val peReg        : Int = 0,
  val outBuf       : Int = 0
) extends Module {

  require(numLanes % numPes == 0,
    "VxPeSerializer: numLanes must be a multiple of numPes")

  val tagW = math.max(tagWidth, 1)   // avoid zero-width wires

  val io = IO(new Bundle {
    val validIn  = Input(Bool())
    val dataIn   = Input(Vec(numLanes, UInt(dataInWidth.W)))
    val tagIn    = if (tagWidth > 0) Some(Input(UInt(tagWidth.W)))  else None
    val readyIn  = Output(Bool())

    val peEnable  = Output(Bool())
    val peDataOut = Output(Vec(numPes, UInt(dataInWidth.W)))
    val peDataIn  = Input(Vec(numPes,  UInt(dataOutWidth.W)))

    val validOut = Output(Bool())
    val dataOut  = Output(Vec(numLanes, UInt(dataOutWidth.W)))
    val tagOut   = if (tagWidth > 0) Some(Output(UInt(tagWidth.W))) else None
    val readyOut = Input(Bool())
  })

  // ----- valid/tag shift register through PE latency ----- //
  val enable    = Wire(Bool())
  val peValidIn = Wire(Bool())
  val peTagIn   = if (tagWidth > 0) Wire(UInt(tagWidth.W)) else Wire(UInt(1.W))

  val depth = peReg + latency
  if (depth == 0) {
    peValidIn := io.validIn
    if (tagWidth > 0) peTagIn := io.tagIn.get else peTagIn := 0.U
  } else {
    // Shift register for valid (with reset) and tag (no reset)
    val validPipe = RegInit(VecInit(Seq.fill(depth)(false.B)))
    val tagPipe   = Reg(Vec(depth, UInt(tagW.W)))
    when (enable) {
      validPipe(0) := io.validIn
      tagPipe(0)   := (if (tagWidth > 0) io.tagIn.get else 0.U)
      for (i <- 1 until depth) {
        validPipe(i) := validPipe(i - 1)
        tagPipe(i)   := tagPipe(i - 1)
      }
    }.otherwise {
      // on back-pressure, reset the valid bits but keep data
      for (i <- 0 until depth) validPipe(i) := false.B
    }
    peValidIn := validPipe(depth - 1)
    peTagIn   := tagPipe(depth - 1)
  }

  // ----- PE data pipeline register ----- //
  val peDataOutW = Wire(Vec(numPes, UInt(dataInWidth.W)))  // before pe-reg

  if (peReg == 0) {
    io.peDataOut := peDataOutW
  } else {
    val pipe = RegInit(VecInit(Seq.fill(peReg)(VecInit(Seq.fill(numPes)(0.U(dataInWidth.W))))))
    when (enable) {
      pipe(0) := peDataOutW
      for (i <- 1 until peReg) pipe(i) := pipe(i - 1)
    }
    io.peDataOut := pipe(peReg - 1)
  }

  io.peEnable := enable

  // ----- valid/ready before output buffer ----- //
  val validOutU = Wire(Bool())
  val dataOutU  = Wire(Vec(numLanes, UInt(dataOutWidth.W)))
  val tagOutU   = Wire(UInt(tagW.W))
  val readyOutU = Wire(Bool())

  if (numLanes == numPes) {
    // ---------------------------------------------------------------
    // Pass-through: no serialization needed
    // ---------------------------------------------------------------
    for (i <- 0 until numPes) peDataOutW(i) := io.dataIn(i)

    enable    := readyOutU || !peValidIn
    io.readyIn := enable

    validOutU := peValidIn
    for (i <- 0 until numLanes) dataOutU(i) := io.peDataIn(i)
    tagOutU   := peTagIn

  } else {
    // ---------------------------------------------------------------
    // Serialized path
    // ---------------------------------------------------------------
    val batchSize  = numLanes / numPes
    val batchSizeW = log2Ceil(batchSize)

    val batchInIdx  = RegInit(0.U(batchSizeW.W))
    val batchOutIdx = RegInit(0.U(batchSizeW.W))
    val batchInDone  = RegInit(false.B)
    val batchOutDone = RegInit(false.B)

    // PE input: slice [batchInIdx * numPes, (batchInIdx+1)*numPes)
    for (i <- 0 until numPes) {
      // Use a Mux tree over all possible batch indices
      val candidates = (0 until batchSize).map { b =>
        io.dataIn(b * numPes + i)
      }
      peDataOutW(i) := VecInit(candidates)(batchInIdx)
    }

    when (enable) {
      // Advance input pointer
      when (io.validIn) {
        batchInIdx  := Mux(batchInIdx === (batchSize - 1).U, 0.U, batchInIdx + 1.U)
        batchInDone := batchInIdx === (batchSize - 2).U
      }
      // Advance output pointer
      when (peValidIn) {
        batchOutIdx  := Mux(batchOutIdx === (batchSize - 1).U, 0.U, batchOutIdx + 1.U)
        batchOutDone := batchOutIdx === (batchSize - 2).U
      }
    }

    // Accumulate PE results into output register
    val dataOutR = RegInit(VecInit(Seq.fill(batchSize)(0.U((numPes * dataOutWidth).W))))
    when (peValidIn) {
      dataOutR(batchOutIdx) := VecInit(io.peDataIn.reverse).asUInt
    }

    enable    := readyOutU || !validOutU
    io.readyIn := enable && batchInDone

    validOutU := batchOutDone
    // Flatten the batch output register into per-lane signals
    for (b <- 0 until batchSize) {
      for (p <- 0 until numPes) {
        dataOutU(b * numPes + p) :=
          dataOutR(b)(dataOutWidth * (p + 1) - 1, dataOutWidth * p)
      }
    }
    tagOutU := peTagIn
  }

  // ----- Output elastic buffer ----- //
  val outDataW = numLanes * dataOutWidth + tagW
  val buf = Module(new VxElasticBuffer(dataw = outDataW, size = math.max(outBuf, 0)))
  buf.io.validIn  := validOutU
  buf.io.dataIn   := Cat(VecInit(dataOutU.reverse).asUInt, tagOutU)
  readyOutU       := buf.io.readyIn
  buf.io.readyOut := io.readyOut

  io.validOut := buf.io.validOut
  val outCat = buf.io.dataOut
  val outTagBits  = outCat(tagW - 1, 0)
  val outDataBits = outCat(outDataW - 1, tagW)
  for (i <- 0 until numLanes) {
    io.dataOut(i) := outDataBits(dataOutWidth * (i + 1) - 1, dataOutWidth * i)
  }
  if (tagWidth > 0) io.tagOut.get := outTagBits
}
