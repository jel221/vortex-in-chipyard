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

// Pending size tracker: tracks the number of in-flight items,
// with full/empty/alm_full/alm_empty status flags.
// Mirrors VX_pending_size.sv:
//   incrw==1 && decrw==1 → g_single_step (flag-based, no explicit counter)
//   otherwise            → g_wide_step   (signed-arithmetic counter)
class VxPendingSize(
  val size     : Int = 1,
  val incrw    : Int = 1,
  val decrw    : Int = 1,
  val almFull  : Int = -1,  // default: size-1
  val almEmpty : Int = 1
) extends Module {
  val af    = if (almFull == -1) size - 1 else almFull
  val sizew = log2Ceil(size + 1)

  val io = IO(new Bundle {
    val incr     = Input(UInt(incrw.W))
    val decr     = Input(UInt(decrw.W))
    val empty    = Output(Bool())
    val almEmpty = Output(Bool())
    val full     = Output(Bool())
    val almFull  = Output(Bool())
    val size     = Output(UInt(sizew.W))
  })

  if (size == 1) {
    // g_size_eq1
    val sizeR = RegInit(false.B)
    when(io.incr.asBool && !io.decr.asBool) {
      sizeR := true.B
    }.elsewhen(!io.incr.asBool && io.decr.asBool) {
      sizeR := false.B
    }
    io.empty    := !sizeR
    io.full     := sizeR
    io.almEmpty := true.B
    io.almFull  := true.B
    io.size     := sizeR.asUInt

  } else if (incrw != 1 || decrw != 1) {
    // g_wide_step: signed delta = incr - decr; size_n = size_r + delta.
    // Mirrors SV: registers full/empty/alm_ from size_n each cycle.
    val sizeR     = RegInit(0.U(sizew.W))
    val emptyR    = RegInit(true.B)
    val fullR     = RegInit(false.B)
    val almEmptyR = RegInit(true.B)
    val almFullR  = RegInit(false.B)

    val delta  = io.incr.zext - io.decr.zext          // SInt subtraction
    val sizeN  = sizeR.zext + delta                    // SInt addition

    assert(sizeN >= 0.S,      "VxPendingSize: counter underflow")
    assert(sizeN <= size.S,   "VxPendingSize: counter overflow")

    sizeR     := sizeN.asUInt
    emptyR    := (sizeN === 0.S)
    fullR     := (sizeN === size.S)
    almEmptyR := (sizeN <= almEmpty.S)
    almFullR  := (sizeN >= af.S)

    io.empty    := emptyR
    io.full     := fullR
    io.almEmpty := almEmptyR
    io.almFull  := almFullR
    io.size     := sizeR

  } else {
    // g_single_step: incrw==1, decrw==1 — flag-based, no wide counter needed.
    val addrw     = log2Up(size)
    val usedR     = RegInit(0.U(addrw.W))
    val emptyR    = RegInit(true.B)
    val fullR     = RegInit(false.B)
    val almEmptyR = RegInit(true.B)
    val almFullR  = RegInit(false.B)

    val incrB = io.incr.asBool
    val decrB = io.decr.asBool

    val isAlmEmpty  = usedR === almEmpty.U
    val isAlmEmptyN = usedR === (almEmpty + 1).U
    val isAlmFull   = usedR === af.U
    val isAlmFullN  = usedR === (af - 1).U
    val isEmptyN    = usedR === 1.U
    val isFullN     = usedR === (size - 1).U

    when(incrB && !decrB) {
      emptyR := false.B
      when(isFullN)    { fullR    := true.B  }
      when(isAlmEmpty) { almEmptyR := false.B }
      when(isAlmFullN) { almFullR  := true.B  }
      usedR := usedR + 1.U
    }.elsewhen(!incrB && decrB) {
      fullR := false.B
      when(isEmptyN)    { emptyR   := true.B  }
      when(isAlmFull)   { almFullR  := false.B }
      when(isAlmEmptyN) { almEmptyR := true.B  }
      usedR := usedR - 1.U
    }

    io.empty    := emptyR
    io.full     := fullR
    io.almEmpty := almEmptyR
    io.almFull  := almFullR

    if (sizew > addrw) {
      io.size := Cat(fullR, usedR)
    } else {
      io.size := usedR
    }
  }
}

// FIFO queue backed by a dual-port RAM.
// Depth must be a power of 2. OUT_REG adds an output pipeline register.
class VxFifoQueue(
  val dataw    : Int = 32,
  val depth    : Int = 32,
  val almFull  : Int = -1,  // default: depth-1
  val almEmpty : Int = 1,
  val outReg   : Int = 0
) extends Module {
  require(isPow2(depth), "depth must be a power of 2")
  val af    = if (almFull == -1) depth - 1 else almFull
  val addrw = log2Ceil(depth)
  val sizew = log2Ceil(depth + 1)

  val io = IO(new Bundle {
    val push     = Input(Bool())
    val pop      = Input(Bool())
    val dataIn   = Input(UInt(dataw.W))
    val dataOut  = Output(UInt(dataw.W))
    val empty    = Output(Bool())
    val almEmpty = Output(Bool())
    val full     = Output(Bool())
    val almFull  = Output(Bool())
    val size     = Output(UInt(sizew.W))
  })

  val pendSize = Module(new VxPendingSize(
    size     = depth,
    almFull  = af,
    almEmpty = almEmpty
  ))
  pendSize.io.incr := io.push
  pendSize.io.decr := io.pop
  io.empty    := pendSize.io.empty
  io.almEmpty := pendSize.io.almEmpty
  io.full     := pendSize.io.full
  io.almFull  := pendSize.io.almFull
  io.size     := pendSize.io.size

  if (depth == 1) {
    val headR = RegEnable(io.dataIn, io.push)
    io.dataOut := headR
  } else {
    val rdPtrR = RegInit(0.U(addrw.W))
    val wrPtrR = RegInit(0.U(addrw.W))

    when(io.push) { wrPtrR := wrPtrR + 1.U }
    when(io.pop)  { rdPtrR := rdPtrR + 1.U }

    // Dual-port RAM: write on push, read continuously from read pointer
    val dpRam = Module(new VxDpRam(dataw = dataw, size = depth, rdwMode = "W"))
    dpRam.io.read  := true.B
    dpRam.io.write := io.push
    dpRam.io.wren  := 1.U(1.W)
    dpRam.io.waddr := wrPtrR
    dpRam.io.wdata := io.dataIn
    dpRam.io.raddr := rdPtrR
    val dataOutW = dpRam.io.rdata

    if (outReg != 0) {
      val dataOutR = Reg(UInt(dataw.W))
      val goingEmpty = pendSize.io.almEmpty  // approximation: alm_empty when size == 1
      val bypass = io.push && (io.empty || (goingEmpty && io.pop))
      when(bypass) {
        dataOutR := io.dataIn
      }.elsewhen(io.pop) {
        dataOutR := dataOutW
      }
      io.dataOut := dataOutR
    } else {
      io.dataOut := dataOutW
    }
  }
}

// Allocator: manages a free-list of SIZE slots.
// acquire_en gets the next free slot address; release_en frees a slot.
class VxAllocator(
  val size : Int = 1
) extends Module {
  val addrw = log2Up(size)

  val io = IO(new Bundle {
    val acquireEn   = Input(Bool())
    val acquireAddr = Output(UInt(addrw.W))
    val releaseEn   = Input(Bool())
    val releaseAddr = Input(UInt(addrw.W))
    val empty       = Output(Bool())
    val full        = Output(Bool())
  })

  val freeSlots   = RegInit(((1 << size) - 1).U(size.W))
  val acquireAddrR = RegInit(0.U(addrw.W))
  val emptyR      = RegInit(true.B)
  val fullR       = RegInit(false.B)

  // Next free slots value
  val freeSlotsN = Wire(UInt(size.W))
  freeSlotsN := freeSlots
  // Apply release first, then acquire
  val freeSlotsAfterRelease = Wire(UInt(size.W))
  freeSlotsAfterRelease := freeSlots
  when(io.releaseEn) {
    freeSlotsAfterRelease := freeSlots | (1.U << io.releaseAddr)
  }
  freeSlotsN := Mux(io.acquireEn, freeSlotsAfterRelease & ~(1.U << acquireAddrR), freeSlotsAfterRelease)

  // Priority encode free slots to find next free index
  val enc = Module(new VxPriorityEncoder(n = size))
  enc.io.dataIn := freeSlotsN
  val freeIndex = enc.io.indexOut
  val freeValid = enc.io.validOut

  when(io.acquireEn || (io.releaseEn && fullR)) {
    acquireAddrR := freeIndex
  }

  freeSlots := freeSlotsN
  emptyR    := freeSlotsN.andR
  fullR     := !freeValid

  io.acquireAddr := acquireAddrR
  io.empty       := emptyR
  io.full        := fullR
}

// Index buffer: combines an allocator with a dual-port RAM.
// Used for tracking in-flight requests by index.
class VxIndexBuffer(
  val dataw  : Int = 1,
  val size   : Int = 1
) extends Module {
  val addrw = log2Up(size)

  val io = IO(new Bundle {
    val writeAddr  = Output(UInt(addrw.W))
    val writeData  = Input(UInt(dataw.W))
    val acquireEn  = Input(Bool())
    val readAddr   = Input(UInt(addrw.W))
    val readData   = Output(UInt(dataw.W))
    val releaseEn  = Input(Bool())
    val empty      = Output(Bool())
    val full       = Output(Bool())
  })

  val alloc = Module(new VxAllocator(size = size))
  alloc.io.acquireEn   := io.acquireEn
  alloc.io.releaseEn   := io.releaseEn
  alloc.io.releaseAddr := io.readAddr
  io.writeAddr         := alloc.io.acquireAddr
  io.empty             := alloc.io.empty
  io.full              := alloc.io.full

  val ram = Module(new VxDpRam(dataw = dataw, size = size, rdwMode = "W"))
  ram.io.read  := true.B
  ram.io.write := io.acquireEn
  ram.io.wren  := 1.U(1.W)
  ram.io.waddr := alloc.io.acquireAddr
  ram.io.wdata := io.writeData
  ram.io.raddr := io.readAddr
  io.readData  := ram.io.rdata
}

// Index queue: a FIFO where entries can be selectively invalidated by index.
// Auto-removes the head when it has been invalidated (pop by index).
class VxIndexQueue(
  val dataw : Int = 1,
  val size  : Int = 1
) extends Module {
  val addrw = log2Up(size)

  val io = IO(new Bundle {
    val writeData  = Input(UInt(dataw.W))
    val writeAddr  = Output(UInt(addrw.W))
    val push       = Input(Bool())
    val pop        = Input(Bool())
    val full       = Output(Bool())
    val empty      = Output(Bool())
    val readAddr   = Input(UInt(addrw.W))
    val readData   = Output(UInt(dataw.W))
  })

  val entries = Mem(size, UInt(dataw.W))
  val valid   = RegInit(0.U(size.W))

  val rdPtr = RegInit(0.U((addrw + 1).W))
  val wrPtr = RegInit(0.U((addrw + 1).W))

  val rdA = rdPtr(addrw - 1, 0)
  val wrA = wrPtr(addrw - 1, 0)

  io.empty := rdPtr === wrPtr
  io.full  := (wrA === rdA) && (wrPtr(addrw) =/= rdPtr(addrw))

  val enqueue = io.push
  val dequeue = !io.empty && !valid(rdA)  // auto-remove when head is invalid

  when(enqueue) {
    valid   := valid | (1.U << wrA)
    wrPtr   := wrPtr + 1.U
    entries.write(wrA, io.writeData)
  }
  when(dequeue) {
    rdPtr := rdPtr + 1.U
  }
  when(io.pop) {
    valid := valid & ~(1.U << io.readAddr)
  }

  io.writeAddr := wrA
  io.readData  := entries.read(io.readAddr)
}
