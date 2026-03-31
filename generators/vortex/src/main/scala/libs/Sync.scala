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

// Reset relay: re-registers the reset signal to reduce fanout.
// When N is large relative to maxFanout, inserts a register stage.
// maxFanout <= 0 disables relay (pass-through).
class VxResetRelay(
  val n          : Int = 1,
  val maxFanout  : Int = 0    // 0 = pass-through always
) extends Module {
  val io = IO(new Bundle {
    val resetO = Output(UInt(n.W))
  })

  // Determine whether relay is needed
  val needRelay = maxFanout > 0 && n > (maxFanout + maxFanout / 2)

  if (needRelay) {
    val f = math.max(maxFanout, 1)
    val r = (n + f - 1) / f  // number of relay registers (ceiling division)
    val resetR = RegNext(reset, init = true.B)
    val bits = Wire(Vec(n, Bool()))
    for (i <- 0 until n) {
      bits(i) := resetR
    }
    io.resetO := bits.asUInt
  } else {
    io.resetO := Fill(n, reset.asUInt)
  }
}

// Ticket lock: issues sequential ticket IDs for ordered mutex acquisition.
// acquire_id is the ID assigned on acquire; release_id is the next to be served.
class VxTicketLock(
  val n : Int = 2
) extends Module {
  val logn = log2Up(n)

  val io = IO(new Bundle {
    val aquireEn  = Input(Bool())
    val releaseEn = Input(Bool())
    val acquireId = Output(UInt(logn.W))
    val releaseId = Output(UInt(logn.W))
    val full      = Output(Bool())
    val empty     = Output(Bool())
  })

  val rdCtrR = RegInit(0.U(logn.W))
  val wrCtrR = RegInit(0.U(logn.W))

  // Pending size tracks occupancy
  val pendSize = Module(new VxPendingSize(size = n))
  pendSize.io.incr := io.aquireEn
  pendSize.io.decr := io.releaseEn
  io.full  := pendSize.io.full
  io.empty := pendSize.io.empty

  when(!io.full && io.aquireEn) {
    wrCtrR := wrCtrR + 1.U
  }
  when(!io.empty && io.releaseEn) {
    rdCtrR := rdCtrR + 1.U
  }

  io.acquireId := wrCtrR
  io.releaseId := rdCtrR
}
