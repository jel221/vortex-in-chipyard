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

// Priority arbiter: grants to the lowest-index active request.
// STICKY: hold the grant while the granted request remains asserted.
class VxPriorityArbiter(
  val numReqs : Int = 1,
  val sticky  : Int = 0
) extends Module {
  val logNumReqs = log2Up(numReqs)

  val io = IO(new Bundle {
    val requests    = Input(UInt(numReqs.W))
    val grantIndex  = Output(UInt(logNumReqs.W))
    val grantOnehot = Output(UInt(numReqs.W))
    val grantValid  = Output(Bool())
    val grantReady  = Input(Bool())
  })

  if (numReqs == 1) {
    io.grantIndex  := 0.U
    io.grantOnehot := io.requests
    io.grantValid  := io.requests(0)
  } else {
    val prevGrant = RegInit(0.U(numReqs.W))
    when(io.grantValid && io.grantReady) {
      prevGrant := io.grantOnehot
    }

    val retainGrant = if (sticky != 0) (prevGrant & io.requests).orR else false.B
    val requestsW   = Mux(retainGrant, prevGrant, io.requests)

    // Priority encode: lowest set bit
    val enc = Module(new VxPriorityEncoder(n = numReqs))
    enc.io.dataIn := requestsW
    io.grantIndex  := enc.io.indexOut
    io.grantOnehot := enc.io.onehotOut

    val grantValidW = enc.io.validOut
    io.grantValid  := (if (sticky != 0) io.requests.orR else grantValidW)
  }
}

// Round-robin arbiter (model-1: dual-mask implementation)
class VxRrArbiter(
  val numReqs : Int = 1,
  val sticky  : Int = 0
) extends Module {
  val logNumReqs = log2Up(numReqs)

  val io = IO(new Bundle {
    val requests    = Input(UInt(numReqs.W))
    val grantIndex  = Output(UInt(logNumReqs.W))
    val grantOnehot = Output(UInt(numReqs.W))
    val grantValid  = Output(Bool())
    val grantReady  = Input(Bool())
  })

  if (numReqs == 1) {
    io.grantIndex  := 0.U
    io.grantOnehot := io.requests
    io.grantValid  := io.requests(0)
  } else {
    // reqs_mask tracks which requests are still eligible in this round
    val reqsMask = RegInit(((1 << numReqs) - 1).U(numReqs.W))

    val maskedReqs = io.requests & reqsMask

    // priority prefix for masked requests
    val maskedPriReqs = Wire(Vec(numReqs, Bool()))
    maskedPriReqs(0) := false.B
    for (i <- 1 until numReqs) {
      maskedPriReqs(i) := maskedPriReqs(i - 1) || maskedReqs(i - 1)
    }

    // priority prefix for unmasked requests
    val unmaskedPriReqs = Wire(Vec(numReqs, Bool()))
    unmaskedPriReqs(0) := false.B
    for (i <- 1 until numReqs) {
      unmaskedPriReqs(i) := unmaskedPriReqs(i - 1) || io.requests(i - 1)
    }

    val grantMasked   = maskedReqs  & ~maskedPriReqs.asUInt
    val grantUnmasked = io.requests & ~unmaskedPriReqs.asUInt

    val hasMaskedReqs   = maskedReqs.orR
    val hasUnmaskedReqs = io.requests.orR

    val prevGrant = RegInit(0.U(numReqs.W))
    when(io.grantValid && io.grantReady) {
      prevGrant := io.grantOnehot
    }

    val retainGrant = if (sticky != 0) (prevGrant & io.requests).orR else false.B

    val grant  = Mux(hasMaskedReqs, grantMasked, grantUnmasked)
    val grantW = Mux(retainGrant, prevGrant, grant)

    io.grantOnehot := grantW

    when(io.grantValid && io.grantReady && !retainGrant) {
      when(hasMaskedReqs) {
        reqsMask := maskedPriReqs.asUInt
      }.elsewhen(hasUnmaskedReqs) {
        reqsMask := unmaskedPriReqs.asUInt
      }
    }

    val enc = Module(new VxOnehotEncoder(n = numReqs))
    enc.io.dataIn := grantW
    io.grantIndex := enc.io.dataOut

    val grantValidW = enc.io.validOut
    io.grantValid  := (if (sticky != 0) io.requests.orR else grantValidW)
  }
}

// Cyclic arbiter: advances a pointer each grant, falls back to priority encoder
class VxCyclicArbiter(
  val numReqs : Int = 1,
  val sticky  : Int = 0
) extends Module {
  val logNumReqs = log2Up(numReqs)

  val io = IO(new Bundle {
    val requests    = Input(UInt(numReqs.W))
    val grantIndex  = Output(UInt(logNumReqs.W))
    val grantOnehot = Output(UInt(numReqs.W))
    val grantValid  = Output(Bool())
    val grantReady  = Input(Bool())
  })

  if (numReqs == 1) {
    io.grantIndex  := 0.U
    io.grantOnehot := io.requests
    io.grantValid  := io.requests(0)
  } else {
    val isPow2 = (1 << logNumReqs) == numReqs

    val grantIndexR = RegInit(0.U(logNumReqs.W))

    val prevGrant = RegInit(0.U(numReqs.W))
    when(io.grantValid && io.grantReady) {
      prevGrant := io.grantOnehot
    }

    val retainGrant = if (sticky != 0) (prevGrant & io.requests).orR else false.B
    val requestsW   = Mux(retainGrant, prevGrant, io.requests)

    // Advance pointer
    when(io.grantValid && io.grantReady && !retainGrant) {
      if (!isPow2) {
        when(io.grantIndex === (numReqs - 1).U) {
          grantIndexR := 0.U
        }.otherwise {
          grantIndexR := io.grantIndex + 1.U
        }
      } else {
        grantIndexR := io.grantIndex + 1.U
      }
    }

    // Priority encoder fallback
    val enc = Module(new VxPriorityEncoder(n = numReqs))
    enc.io.dataIn := requestsW
    val grantIndexUm  = enc.io.indexOut
    val grantOnehotUm = enc.io.onehotOut
    val grantValidW   = enc.io.validOut

    val grantOnehotW = UIntToOH(grantIndexR)

    val isHit = io.requests(grantIndexR) && !retainGrant

    io.grantIndex  := Mux(isHit, grantIndexR,  grantIndexUm)
    io.grantOnehot := Mux(isHit, grantOnehotW, grantOnehotUm)
    io.grantValid  := (if (sticky != 0) io.requests.orR else grantValidW)
  }
}

// Matrix arbiter: uses a priority matrix for fair round-robin arbitration
class VxMatrixArbiter(
  val numReqs : Int = 1,
  val sticky  : Int = 0
) extends Module {
  val logNumReqs = log2Up(numReqs)

  val io = IO(new Bundle {
    val requests    = Input(UInt(numReqs.W))
    val grantIndex  = Output(UInt(logNumReqs.W))
    val grantOnehot = Output(UInt(numReqs.W))
    val grantValid  = Output(Bool())
    val grantReady  = Input(Bool())
  })

  if (numReqs == 1) {
    io.grantIndex  := 0.U
    io.grantOnehot := io.requests
    io.grantValid  := io.requests(0)
  } else {
    // state(r)(c) for r < c: tracks whether r has lower priority than c
    // Stored as a 2D array of registers (upper triangle only)
    val state = Array.tabulate(numReqs, numReqs) { (r, c) =>
      if (r < c) Some(RegInit(false.B)) else None
    }

    val prevGrant = RegInit(0.U(numReqs.W))
    when(io.grantValid && io.grantReady) {
      prevGrant := io.grantOnehot
    }

    val retainGrant = if (sticky != 0) (prevGrant & io.requests).orR else false.B

    // pri(r)(c): is there a higher-priority requester c competing with r?
    val pri = Wire(Vec(numReqs, Vec(numReqs, Bool())))
    for (r <- 0 until numReqs) {
      for (c <- 0 until numReqs) {
        if (r > c) {
          // state[c][r] exists (c < r)
          pri(r)(c) := io.requests(c) && state(c)(r).get
        } else if (r < c) {
          pri(r)(c) := io.requests(c) && !state(r)(c).get
        } else {
          pri(r)(c) := false.B
        }
      }
    }

    val grant = Wire(UInt(numReqs.W))
    grant := VecInit(
      (0 until numReqs).map { r =>
        io.requests(r) && !pri(r).asUInt.orR
      }
    ).asUInt

    val grantW = Mux(retainGrant, prevGrant, grant)

    // Update state registers
    for (r <- 0 until numReqs) {
      for (c <- r + 1 until numReqs) {
        when(io.grantValid && io.grantReady && !retainGrant) {
          state(r)(c).get := (state(r)(c).get || grant(c)) && !grant(r)
        }
      }
    }

    io.grantOnehot := grantW

    val enc = Module(new VxOnehotEncoder(n = numReqs))
    enc.io.dataIn := grantW
    io.grantIndex := enc.io.dataOut

    val grantValidW = enc.io.validOut
    io.grantValid  := (if (sticky != 0) io.requests.orR else grantValidW)
  }
}

// Generic arbiter: selects between priority (P), round-robin (R), matrix (M), or cyclic (C)
class VxGenericArbiter(
  val numReqs  : Int    = 1,
  val arbType  : String = "P",  // "P", "R", "M", or "C"
  val sticky   : Int    = 0
) extends Module {
  val logNumReqs = log2Up(numReqs)

  val io = IO(new Bundle {
    val requests    = Input(UInt(numReqs.W))
    val grantIndex  = Output(UInt(logNumReqs.W))
    val grantOnehot = Output(UInt(numReqs.W))
    val grantValid  = Output(Bool())
    val grantReady  = Input(Bool())
  })

  arbType match {
    case "P" =>
      val arb = Module(new VxPriorityArbiter(numReqs = numReqs, sticky = sticky))
      arb.io.requests   := io.requests
      arb.io.grantReady := io.grantReady
      io.grantIndex     := arb.io.grantIndex
      io.grantOnehot    := arb.io.grantOnehot
      io.grantValid     := arb.io.grantValid

    case "R" =>
      val arb = Module(new VxRrArbiter(numReqs = numReqs, sticky = sticky))
      arb.io.requests   := io.requests
      arb.io.grantReady := io.grantReady
      io.grantIndex     := arb.io.grantIndex
      io.grantOnehot    := arb.io.grantOnehot
      io.grantValid     := arb.io.grantValid

    case "M" =>
      val arb = Module(new VxMatrixArbiter(numReqs = numReqs, sticky = sticky))
      arb.io.requests   := io.requests
      arb.io.grantReady := io.grantReady
      io.grantIndex     := arb.io.grantIndex
      io.grantOnehot    := arb.io.grantOnehot
      io.grantValid     := arb.io.grantValid

    case "C" | _ =>
      val arb = Module(new VxCyclicArbiter(numReqs = numReqs, sticky = sticky))
      arb.io.requests   := io.requests
      arb.io.grantReady := io.grantReady
      io.grantIndex     := arb.io.grantIndex
      io.grantOnehot    := arb.io.grantOnehot
      io.grantValid     := arb.io.grantValid
  }
}
