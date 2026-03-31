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

// N-to-1 multiplexer: select one of N DATAW-bit inputs using a binary index
class VxMux(
  val dataw : Int = 1,
  val n     : Int = 1
) extends Module {
  val ln = log2Up(n)

  val io = IO(new Bundle {
    val dataIn  = Input(Vec(n, UInt(dataw.W)))
    val selIn   = Input(UInt(ln.W))
    val dataOut = Output(UInt(dataw.W))
  })

  if (n > 1) {
    io.dataOut := io.dataIn(io.selIn)
  } else {
    io.dataOut := io.dataIn(0)
  }
}

// 1-to-N demultiplexer: route a DATAW-bit input to one of N outputs
// All outputs are zero except the selected one.
class VxDemux(
  val dataw : Int = 1,
  val n     : Int = 1
) extends Module {
  val ln = log2Up(n)

  val io = IO(new Bundle {
    val selIn   = Input(UInt(ln.W))
    val dataIn  = Input(UInt(dataw.W))
    val dataOut = Output(Vec(n, UInt(dataw.W)))
  })

  if (n > 1) {
    for (i <- 0 until n) {
      io.dataOut(i) := Mux(io.selIn === i.U, io.dataIn, 0.U(dataw.W))
    }
  } else {
    io.dataOut(0) := io.dataIn
  }
}
