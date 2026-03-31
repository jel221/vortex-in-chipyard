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

// Popcount: count the number of set bits in an N-bit input
class VxPopcount(val n: Int = 1) extends Module {
  val m = log2Ceil(n + 1)

  val io = IO(new Bundle {
    val dataIn  = Input(UInt(n.W))
    val dataOut = Output(UInt(m.W))
  })

  // Use Chisel's built-in popcount
  io.dataOut := PopCount(io.dataIn)
}

// Find-first: binary tree to find first (or last) valid entry
// REVERSE=false -> first valid (lowest index), REVERSE=true -> last valid (highest index)
class VxFindFirst(
  val n       : Int     = 1,
  val dataw   : Int     = 1,
  val reverse : Boolean = false
) extends Module {
  val io = IO(new Bundle {
    val dataIn  = Input(Vec(n, UInt(dataw.W)))
    val validIn = Input(UInt(n.W))
    val dataOut = Output(UInt(dataw.W))
    val validOut= Output(Bool())
  })

  if (n == 1) {
    io.dataOut  := io.dataIn(0)
    io.validOut := io.validIn(0)
  } else {
    // Build a binary tree over the inputs
    // Each node holds (valid: Bool, data: UInt)
    // We select the "first" valid (lowest or highest index depending on reverse)
    def treeReduce(pairs: Seq[(Bool, UInt)]): (Bool, UInt) = {
      if (pairs.length == 1) {
        pairs.head
      } else {
        val half   = (pairs.length + 1) / 2
        val left   = treeReduce(pairs.take(half))
        val right  = treeReduce(pairs.drop(half))
        // For REVERSE=false (find first/lowest): prefer left (lower index)
        // For REVERSE=true  (find last/highest): prefer right (higher index)
        val (lv, ld) = left
        val (rv, rd) = right
        val outV = lv || rv
        val outD = if (!reverse) {
          Mux(lv, ld, rd)   // prefer left (lower)
        } else {
          Mux(rv, rd, ld)   // prefer right (higher)
        }
        (outV, outD)
      }
    }

    val pairs: Seq[(Bool, UInt)] = (0 until n).map { i =>
      (io.validIn(i).asBool, io.dataIn(i))
    }

    val (v, d) = treeReduce(pairs)
    io.validOut := v
    io.dataOut  := d
  }
}

// Leading-zero count (or trailing-zero count when REVERSE=1)
// REVERSE=0: count leading zeros (from MSB)
// REVERSE=1: count trailing zeros / find first set from LSB
class VxLzc(
  val n       : Int     = 2,
  val reverse : Boolean = false   // false=leading zeros, true=trailing zeros
) extends Module {
  val logn = log2Up(n)

  val io = IO(new Bundle {
    val dataIn  = Input(UInt(n.W))
    val dataOut = Output(UInt(logn.W))
    val validOut= Output(Bool())
  })

  if (n == 1) {
    io.dataOut  := 0.U
    io.validOut := io.dataIn(0)
  } else {
    // Build index array
    val ff = Module(new VxFindFirst(n = n, dataw = logn, reverse = !reverse))
    for (i <- 0 until n) {
      ff.io.dataIn(i) := (if (reverse) i.U(logn.W) else (n - 1 - i).U(logn.W))
    }
    ff.io.validIn := io.dataIn
    io.dataOut    := ff.io.dataOut
    io.validOut   := ff.io.validOut
  }
}

// Priority encoder: find lowest-set-bit (REVERSE=false) or highest-set-bit (REVERSE=true)
class VxPriorityEncoder(
  val n       : Int     = 1,
  val reverse : Boolean = false   // false=LSB, true=MSB
) extends Module {
  val ln = log2Up(n)

  val io = IO(new Bundle {
    val dataIn   = Input(UInt(n.W))
    val onehotOut= Output(UInt(n.W))
    val indexOut = Output(UInt(ln.W))
    val validOut = Output(Bool())
  })

  if (n == 1) {
    io.onehotOut := io.dataIn
    io.indexOut  := 0.U
    io.validOut  := io.dataIn(0)
  } else if (!reverse) {
    // LSB priority: isolate lowest set bit
    // onehot = data & (-data)  i.e. data & (~data + 1)
    val neg = (~io.dataIn).asUInt + 1.U
    io.onehotOut := io.dataIn & neg

    // index via trailing-zero count
    val lzc = Module(new VxLzc(n = n, reverse = true))
    lzc.io.dataIn := io.dataIn
    io.indexOut   := lzc.io.dataOut
    io.validOut   := lzc.io.validOut
  } else {
    // MSB priority: isolate highest set bit
    // Use prefix OR then find last set bit
    val higherPri = Wire(Vec(n, Bool()))
    higherPri(n - 1) := false.B
    for (i <- n - 2 to 0 by -1) {
      higherPri(i) := higherPri(i + 1) || io.dataIn(i + 1)
    }
    io.onehotOut := io.dataIn & ~higherPri.asUInt

    val ff = Module(new VxFindFirst(n = n, dataw = ln, reverse = true))
    ff.io.validIn := io.dataIn
    for (i <- 0 until n) ff.io.dataIn(i) := i.U(ln.W)
    io.indexOut  := ff.io.dataOut
    io.validOut  := ff.io.validOut
  }
}

// Reduce tree: recursively combine N inputs with a binary operator
// op: "+", "^", "&", "|"
class VxReduceTree(
  val inW  : Int    = 1,
  val outW : Int    = 1,
  val n    : Int    = 1,
  val op   : String = "+"
) extends Module {
  val io = IO(new Bundle {
    val dataIn  = Input(Vec(n, UInt(inW.W)))
    val dataOut = Output(UInt(outW.W))
  })

  def reduce(inputs: Seq[UInt]): UInt = {
    if (inputs.length == 1) {
      inputs.head.pad(outW)
    } else {
      val half  = inputs.length / 2
      val outA  = reduce(inputs.take(half))
      val outB  = reduce(inputs.drop(half))
      op match {
        case "+"  => outA + outB
        case "^"  => outA ^ outB
        case "&"  => outA & outB
        case "|"  => outA | outB
        case _    => throw new IllegalArgumentException(s"Unknown op: $op")
      }
    }
  }

  io.dataOut := reduce(io.dataIn.toSeq)
}

// Kogge-Stone parallel prefix scan
// op: "^" (XOR), "&" (AND), "|" (OR)
// reverse=false: lo-to-hi inclusive prefix scan
class VxScan(
  val n       : Int     = 1,
  val op      : String  = "^",
  val reverse : Boolean = false
) extends Module {
  val io = IO(new Bundle {
    val dataIn  = Input(UInt(n.W))
    val dataOut = Output(UInt(n.W))
  })

  if (n == 1) {
    io.dataOut := io.dataIn
  } else {
    val logn = log2Ceil(n)

    // Compute inclusive prefix scan using a simple sequential approach
    // (Kogge-Stone is synthesis-optimized; for Chisel we use a direct combinational approach)
    val t = Wire(Vec(n, Bool()))

    // Convert input to bit vector (reverse if needed)
    val inBits = (0 until n).map { i =>
      if (!reverse) io.dataIn(i).asBool else io.dataIn(n - 1 - i).asBool
    }

    // Build prefix array: t(0) = inBits(0), t(i) = t(i-1) op inBits(i)
    t(0) := inBits(0)
    for (i <- 1 until n) {
      t(i) := (op match {
        case "^" => t(i - 1) ^ inBits(i)
        case "&" => t(i - 1) & inBits(i)
        case "|" => t(i - 1) | inBits(i)
        case _   => throw new IllegalArgumentException(s"Unknown op: $op")
      }).asBool
    }

    // Output with reversal
    val outBits = Wire(Vec(n, Bool()))
    for (i <- 0 until n) {
      outBits(if (!reverse) i else n - 1 - i) := t(i)
    }
    io.dataOut := outBits.asUInt
  }
}

// One-hot encoder: converts a one-hot input to a binary index
class VxOnehotEncoder(
  val n       : Int     = 1,
  val reverse : Boolean = false
) extends Module {
  val ln = log2Up(n)

  val io = IO(new Bundle {
    val dataIn  = Input(UInt(n.W))
    val dataOut = Output(UInt(ln.W))
    val validOut= Output(Bool())
  })

  if (n == 1) {
    io.dataOut  := 0.U
    io.validOut := io.dataIn(0)
  } else if (n == 2) {
    io.dataOut  := (if (!reverse) io.dataIn(1) else io.dataIn(0)).asUInt
    io.validOut := io.dataIn.orR
  } else {
    // For each output bit j, it is 1 if any input i where bit j of i is set
    val idx = Wire(Vec(ln, Bool()))
    for (j <- 0 until ln) {
      val bits = (0 until n).map { i =>
        if (reverse) ((n - 1 - i) >> j & 1) != 0 else (i >> j & 1) != 0
      }
      idx(j) := VecInit(
        (0 until n).filter(i => bits(i)).map(i => io.dataIn(i).asBool)
      ).asUInt.orR
    }
    io.dataOut  := idx.asUInt
    io.validOut := io.dataIn.orR
  }
}

// One-hot multiplexer: select one of N inputs using a one-hot select
class VxOnehotMux(
  val dataw : Int = 1,
  val n     : Int = 1
) extends Module {
  val io = IO(new Bundle {
    val dataIn  = Input(Vec(n, UInt(dataw.W)))
    val selIn   = Input(UInt(n.W))
    val dataOut = Output(UInt(dataw.W))
  })

  if (n == 1) {
    io.dataOut := io.dataIn(0)
  } else {
    // OR-of-masked implementation: data_out[b] = OR over i of (sel[i] & data_in[i][b])
    val masked = Wire(Vec(n, UInt(dataw.W)))
    for (i <- 0 until n) {
      masked(i) := Mux(io.selIn(i), io.dataIn(i), 0.U(dataw.W))
    }
    io.dataOut := masked.reduce(_ | _)
  }
}

// One-hot shift: outer product of two one-hot vectors
// data_out[i*N + j] = data_in1[i] & data_in0[j]
class VxOnehotShift(
  val n : Int = 1,
  val m : Int = 1
) extends Module {
  val io = IO(new Bundle {
    val dataIn0 = Input(UInt(n.W))
    val dataIn1 = Input(UInt(m.W))
    val dataOut = Output(UInt((n * m).W))
  })

  val bits = Wire(Vec(n * m, Bool()))
  for (i <- 0 until m) {
    for (j <- 0 until n) {
      bits(i * n + j) := io.dataIn1(i) && io.dataIn0(j)
    }
  }
  io.dataOut := bits.asUInt
}
