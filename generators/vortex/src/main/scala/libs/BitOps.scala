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

// Bit concatenation: {left_in, right_in}
// If L==0, output is right_in; if R==0, output is left_in.
class VxBitsConcat(
  val l : Int = 1,
  val r : Int = 1
) extends Module {
  val lEff = if (l == 0) 1 else l
  val rEff = if (r == 0) 1 else r

  val io = IO(new Bundle {
    val leftIn  = Input(UInt(lEff.W))
    val rightIn = Input(UInt(rEff.W))
    val dataOut = Output(UInt((l + r).W))
  })

  if (l == 0) {
    io.dataOut := io.rightIn
  } else if (r == 0) {
    io.dataOut := io.leftIn
  } else {
    io.dataOut := Cat(io.leftIn, io.rightIn)
  }
}

// Bit insertion: insert S bits (ins_in) into data_in at position POS
// data_out = {data_in[N-1:POS], ins_in, data_in[POS-1:0]}
class VxBitsInsert(
  val n   : Int = 1,
  val s   : Int = 1,
  val pos : Int = 0
) extends Module {
  val sEff = if (s == 0) 1 else s

  val io = IO(new Bundle {
    val dataIn  = Input(UInt(n.W))
    val insIn   = Input(UInt(sEff.W))
    val dataOut = Output(UInt((n + s).W))
  })

  if (s == 0) {
    io.dataOut := io.dataIn
  } else if (pos == 0) {
    io.dataOut := Cat(io.dataIn, io.insIn)
  } else if (pos == n) {
    io.dataOut := Cat(io.insIn, io.dataIn)
  } else {
    io.dataOut := Cat(io.dataIn(n - 1, pos), io.insIn, io.dataIn(pos - 1, 0))
  }
}

// Bit removal: remove S bits at position POS from data_in
// sel_out = data_in[POS +: S], data_out = remaining bits
class VxBitsRemove(
  val n   : Int = 2,
  val s   : Int = 1,
  val pos : Int = 0
) extends Module {
  val sEff  = if (s == 0) 1 else s
  val outW  = if (n - s <= 0) 1 else n - s

  val io = IO(new Bundle {
    val dataIn  = Input(UInt(n.W))
    val selOut  = Output(UInt(sEff.W))
    val dataOut = Output(UInt(outW.W))
  })

  if (s == 0) {
    io.selOut  := 0.U
    io.dataOut := io.dataIn
  } else if (pos == 0) {
    io.selOut  := io.dataIn(s - 1, 0)
    io.dataOut := io.dataIn(n - 1, s)
  } else if (pos + s == n) {
    io.selOut  := io.dataIn(n - 1, pos)
    io.dataOut := io.dataIn(pos - 1, 0)
  } else {
    io.selOut  := io.dataIn(pos + s - 1, pos)
    io.dataOut := Cat(io.dataIn(n - 1, pos + s), io.dataIn(pos - 1, 0))
  }
}

// Shift register with configurable depth, taps, reset, and enable
class VxShiftRegister(
  val dataw     : Int = 1,
  val resetw    : Int = 0,
  val depth     : Int = 1,
  val numTaps   : Int = 1,
  val tapStart  : Int = -1,   // default: depth-1
  val tapStride : Int = 1,
  val initValue : Int = 0
) extends Module {
  val ts   = if (tapStart == -1) depth - 1 else tapStart
  val rw   = resetw

  val io = IO(new Bundle {
    val enable  = Input(Bool())
    val dataIn  = Input(UInt(dataw.W))
    val dataOut = Output(Vec(numTaps, UInt(dataw.W)))
  })

  if (depth == 0) {
    // Pass-through
    for (t <- 0 until numTaps) io.dataOut(t) := io.dataIn
  } else {
    val pipe = Reg(Vec(depth, UInt(dataw.W)))

    for (i <- 0 until depth) {
      val next = if (i == 0) io.dataIn else pipe(i - 1)
      if (rw == dataw) {
        // Full reset
        when(reset.asBool) {
          pipe(i) := initValue.U(dataw.W)
        }.elsewhen(io.enable) {
          pipe(i) := next
        }
      } else if (rw != 0) {
        // Partial reset: top rw bits reset, bottom bits no reset
        when(reset.asBool) {
          pipe(i) := Cat(initValue.U(rw.W), pipe(i)(dataw - rw - 1, 0))
        }.elsewhen(io.enable) {
          pipe(i) := next
        }
      } else {
        // No reset
        when(io.enable) {
          pipe(i) := next
        }
      }
    }

    for (t <- 0 until numTaps) {
      io.dataOut(t) := pipe(t * tapStride + ts)
    }
  }
}

// Non-zero iterator: iterates over non-zero entries in a stream of N elements
// Outputs the first non-zero entry not yet sent, tracking start/end of stream.
class VxNzIterator(
  val dataw    : Int = 8,
  val keyw     : Int = -1,  // default: dataw
  val n        : Int = 4,
  val outReg   : Int = 0
) extends Module {
  val kw       = if (keyw == -1) dataw else keyw
  val lpidWidth = log2Up(n)

  val io = IO(new Bundle {
    val validIn  = Input(Bool())
    val dataIn   = Input(Vec(n, UInt(dataw.W)))
    val next     = Input(Bool())
    val validOut = Output(Bool())
    val dataOut  = Output(UInt(dataw.W))
    val pid      = Output(UInt(lpidWidth.W))
    val sop      = Output(Bool())
    val eop      = Output(Bool())
  })

  if (n == 1) {
    io.validOut := io.validIn
    io.dataOut  := io.dataIn(0)
    io.pid      := 0.U
    io.sop      := true.B
    io.eop      := true.B
  } else {
    // Track which entries have already been sent
    val sentMaskP = RegInit(0.U(n.W))
    val isFirstP  = RegInit(true.B)

    // Determine which entries are valid (non-zero key)
    val packetValids = VecInit((0 until n).map { i =>
      io.dataIn(i)(kw - 1, 0).orR
    })

    // Find first unsent valid entry
    val ffFirst = Module(new VxFindFirst(n = n, dataw = lpidWidth, reverse = false))
    ffFirst.io.validIn := (packetValids.asUInt & ~sentMaskP)
    for (i <- 0 until n) ffFirst.io.dataIn(i) := i.U(lpidWidth.W)
    val startP = ffFirst.io.dataOut

    // Find last valid entry (for eop detection)
    val ffLast = Module(new VxFindFirst(n = n, dataw = lpidWidth, reverse = true))
    ffLast.io.validIn := packetValids.asUInt
    for (i <- 0 until n) ffLast.io.dataIn(i) := i.U(lpidWidth.W)
    val endP = ffLast.io.dataOut

    val isLastP = startP === endP
    val enable  = io.validIn && (!io.validOut || io.next)

    val validOutR = RegInit(false.B)
    val dataOutR  = Reg(UInt(dataw.W))
    val pidR      = Reg(UInt(lpidWidth.W))
    val sopR      = RegInit(false.B)
    val eopR      = RegInit(false.B)

    when(reset.asBool || (enable && (isLastP || eopR))) {
      sentMaskP := 0.U
      isFirstP  := true.B
    }.elsewhen(enable) {
      sentMaskP := sentMaskP | (1.U << startP)
      isFirstP  := false.B
    }

    when(reset.asBool || (enable && eopR)) {
      validOutR := false.B
    }.elsewhen(enable) {
      validOutR := io.validIn
      dataOutR  := io.dataIn(startP)
      pidR      := startP
      sopR      := isFirstP
      eopR      := isLastP
    }

    io.validOut := validOutR
    io.dataOut  := dataOutR
    io.pid      := pidR
    io.sop      := sopR
    io.eop      := eopR
  }
}

// Matrix transpose: data_out[j][i] = data_in[i][j]
class VxTranspose(
  val dataw : Int = 1,
  val n     : Int = 1,
  val m     : Int = 1
) extends Module {
  val io = IO(new Bundle {
    // data_in: N rows of M elements
    val dataIn  = Input(Vec(n, Vec(m, UInt(dataw.W))))
    // data_out: M rows of N elements
    val dataOut = Output(Vec(m, Vec(n, UInt(dataw.W))))
  })

  for (i <- 0 until n) {
    for (j <- 0 until m) {
      io.dataOut(j)(i) := io.dataIn(i)(j)
    }
  }
}
