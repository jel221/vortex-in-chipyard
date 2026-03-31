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

// Combinational / pipelined multiplier.
// LATENCY=0: purely combinational.
// LATENCY>0: result is registered through a shift register pipeline.
class VxMultiplier(
  val aWidth  : Int = 1,
  val bWidth  : Int = -1,   // default: aWidth
  val rWidth  : Int = -1,   // default: aWidth + bWidth
  val isSigned: Boolean = false,
  val latency : Int = 0
) extends Module {
  val bW = if (bWidth == -1) aWidth else bWidth
  val rW = if (rWidth == -1) aWidth + bW else rWidth

  val io = IO(new Bundle {
    val enable = Input(Bool())
    val dataa  = Input(UInt(aWidth.W))
    val datab  = Input(UInt(bW.W))
    val result = Output(UInt(rW.W))
  })

  // Compute product
  val prodW = Wire(UInt(rW.W))
  if (isSigned) {
    prodW := (io.dataa.asSInt * io.datab.asSInt).asUInt(rW - 1, 0)
  } else {
    prodW := (io.dataa * io.datab)(rW - 1, 0)
  }

  // Pipeline stages
  if (latency == 0) {
    io.result := prodW
  } else {
    val pipe = Seq.fill(latency)(Reg(UInt(rW.W)))
    pipe.zipWithIndex.foreach { case (reg, i) =>
      when(io.enable) {
        reg := (if (i == 0) prodW else pipe(i - 1))
      }
    }
    io.result := pipe(latency - 1)
  }
}

// Iterative (serial) multiplier.
// Processes one bit of operand A per clock cycle.
// LANES: number of parallel multipliers sharing the same control.
class VxSerialMul(
  val aWidth  : Int     = 32,
  val bWidth  : Int     = -1,   // default: aWidth
  val rWidth  : Int     = -1,   // default: aWidth + bWidth
  val isSigned: Boolean = false,
  val lanes   : Int     = 1
) extends Module {
  val bW     = if (bWidth == -1) aWidth else bWidth
  val rW     = if (rWidth == -1) aWidth + bW else rWidth
  val xWidth = if (isSigned) math.max(aWidth, bW) else aWidth
  val yWidth = if (isSigned) math.max(aWidth, bW) else bW
  val pWidth = xWidth + yWidth
  val cntrw  = log2Ceil(xWidth)

  val io = IO(new Bundle {
    val strobe = Input(Bool())
    val busy   = Output(Bool())
    val dataa  = Input(Vec(lanes, UInt(aWidth.W)))
    val datab  = Input(Vec(lanes, UInt(bWidth.max(1).W)))
    val result = Output(Vec(lanes, UInt(rW.W)))
  })

  val cntr   = Reg(UInt(cntrw.W))
  val busyR  = RegInit(false.B)

  when(!reset.asBool) {
    when(io.strobe) {
      busyR := true.B
    }
    when(busyR && cntr === 0.U) {
      busyR := false.B
    }
  }

  cntr := cntr - 1.U
  when(io.strobe) {
    cntr := (xWidth - 1).U
  }

  io.busy := busyR

  for (i <- 0 until lanes) {
    val a = Reg(UInt(xWidth.W))
    val b = Reg(UInt(yWidth.W))
    val p = Reg(UInt(pWidth.W))

    val axb = Mux(b(0), a, 0.U(xWidth.W))

    when(io.strobe) {
      if (isSigned) {
        a := io.dataa(i).asSInt.pad(xWidth).asUInt
        b := io.datab(i).asSInt.pad(yWidth).asUInt
      } else {
        a := io.dataa(i)
        b := io.datab(i)
      }
      p := 0.U
    }.elsewhen(busyR) {
      b := b >> 1
      // Shift lower bits of p right by 1
      p := Cat(
        // Upper part: p[pWidth-1 : yWidth-1] + shift logic
        (Cat(0.U(1.W), p(pWidth - 1, yWidth)) + Cat(0.U(1.W), axb))(xWidth, 0),
        p(yWidth - 2, 0),
        0.U(1.W)
      )
    }

    if (isSigned) {
      // Two's complement correction for signed multiply
      io.result(i) := (p + Cat(1.U(1.W), 0.U((xWidth - 2).W), 1.U(1.W), 0.U(yWidth.W)))(rW - 1, 0)
    } else {
      io.result(i) := p(rW - 1, 0)
    }
  }
}

// Combinational / pipelined divider.
// Computes quotient and remainder. LATENCY=0 is purely combinational.
class VxDivider(
  val nWidth  : Int     = 1,
  val dWidth  : Int     = 1,
  val qWidth  : Int     = 1,
  val rWidth  : Int     = 1,
  val nSigned : Boolean = false,
  val dSigned : Boolean = false,
  val latency : Int     = 0
) extends Module {
  val io = IO(new Bundle {
    val enable    = Input(Bool())
    val numer     = Input(UInt(nWidth.W))
    val denom     = Input(UInt(dWidth.W))
    val quotient  = Output(UInt(qWidth.W))
    val remainder = Output(UInt(rWidth.W))
  })

  // Combinational divide
  val quotUnqual  = Wire(UInt(nWidth.W))
  val remUnqual   = Wire(UInt(dWidth.W))

  if (nSigned && dSigned) {
    quotUnqual  := (io.numer.asSInt / io.denom.asSInt).asUInt
    remUnqual   := (io.numer.asSInt % io.denom.asSInt).asUInt
  } else if (nSigned) {
    quotUnqual  := (io.numer.asSInt / io.denom.zext).asUInt
    remUnqual   := (io.numer.asSInt % io.denom.zext).asUInt
  } else if (dSigned) {
    quotUnqual  := (io.numer.zext / io.denom.asSInt).asUInt
    remUnqual   := (io.numer.zext % io.denom.asSInt).asUInt
  } else {
    quotUnqual  := io.numer / io.denom
    remUnqual   := io.numer % io.denom
  }

  if (latency == 0) {
    io.quotient  := quotUnqual(qWidth - 1, 0)
    io.remainder := remUnqual(rWidth - 1, 0)
  } else {
    val qPipe = Seq.fill(latency)(Reg(UInt(nWidth.W)))
    val rPipe = Seq.fill(latency)(Reg(UInt(dWidth.W)))
    qPipe.zipWithIndex.foreach { case (reg, i) =>
      when(io.enable) {
        reg := (if (i == 0) quotUnqual else qPipe(i - 1))
      }
    }
    rPipe.zipWithIndex.foreach { case (reg, i) =>
      when(io.enable) {
        reg := (if (i == 0) remUnqual else rPipe(i - 1))
      }
    }
    io.quotient  := qPipe(latency - 1)(qWidth - 1, 0)
    io.remainder := rPipe(latency - 1)(rWidth - 1, 0)
  }
}

// Iterative (serial) divider using a non-restoring algorithm.
// LANES: number of parallel dividers sharing the same control.
class VxSerialDiv(
  val widthn : Int = 32,
  val widthd : Int = 32,
  val widthq : Int = 32,
  val widthr : Int = 32,
  val lanes  : Int = 1
) extends Module {
  val minND = math.min(widthn, widthd)
  val cntrw = log2Ceil(widthn)

  val io = IO(new Bundle {
    val strobe    = Input(Bool())
    val busy      = Output(Bool())
    val isSigned  = Input(Bool())
    val numer     = Input(Vec(lanes, UInt(widthn.W)))
    val denom     = Input(Vec(lanes, UInt(widthd.W)))
    val quotient  = Output(Vec(lanes, UInt(widthq.W)))
    val remainder = Output(Vec(lanes, UInt(widthr.W)))
  })

  val cntr  = Reg(UInt(cntrw.W))
  val busyR = RegInit(false.B)

  when(!reset.asBool) {
    when(io.strobe) {
      busyR := true.B
    }
    when(io.busy && cntr === 0.U) {
      busyR := false.B
    }
  }
  cntr := cntr - 1.U
  when(io.strobe) {
    cntr := (widthn - 1).U
  }

  io.busy := busyR

  for (i <- 0 until lanes) {
    // working: widthn + minND + 1 bits
    val workW  = widthn + minND + 1
    val working = Reg(UInt(workW.W))
    val denomR  = Reg(UInt(widthd.W))
    val invQuot = Reg(Bool())
    val invRem  = Reg(Bool())

    val negateNumer = io.isSigned && io.numer(i)(widthn - 1)
    val negateDenom = io.isSigned && io.denom(i)(widthd - 1)

    val numerQual = Mux(negateNumer, (-io.numer(i).asSInt).asUInt, io.numer(i))
    val denomQual = Mux(negateDenom, (-io.denom(i).asSInt).asUInt, io.denom(i))

    // sub_result = upper (widthd+1) bits of working minus denomR
    val subResult = Cat(0.U(1.W), working(widthn + minND, widthn)) - Cat(0.U(1.W), denomR)

    when(io.strobe) {
      working  := Cat(0.U(widthd.W), numerQual, 0.U(1.W))
      denomR   := denomQual
      invQuot  := (io.denom(i) =/= 0.U) && io.isSigned && (io.numer(i)(widthn - 1) ^ io.denom(i)(widthd - 1))
      invRem   := io.isSigned && io.numer(i)(widthn - 1)
    }.elsewhen(busyR) {
      when(subResult(widthd)) {
        // sub borrow: shift without updating
        working := Cat(working(workW - 2, 0), 0.U(1.W))
      }.otherwise {
        // sub no borrow: store sub result
        working := Cat(subResult(widthd - 1, 0), working(widthn - 1, 0), 1.U(1.W))
      }
    }

    val q = working(widthq - 1, 0)
    val r = working(widthn + widthr, widthn + 1)

    io.quotient(i)  := Mux(invQuot, (-q.asSInt).asUInt, q)
    io.remainder(i) := Mux(invRem,  (-r.asSInt).asUInt, r)
  }
}
