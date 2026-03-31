// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_ipdom_stack.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Immediate Post-DOMinator (IPDOM) stack for warp divergence handling.
 *  Mirrors VX_ipdom_stack.sv.
 *
 *  @param width  Data width per entry half (WIDTH parameter)
 *  @param depth  Stack depth (DEPTH parameter)
 */
class VxIpdomStack(
  val width: Int = 1,
  val depth: Int = 1
) extends Module {
  val addrw   = math.max(1, log2Ceil(math.max(depth, 2)))
  val bramSz  = depth * NUM_WARPS

  val io = IO(new Bundle {
    val wid    = Input(UInt(NW_WIDTH.W))
    val d0     = Input(UInt(width.W))
    val d1     = Input(UInt(width.W))
    val rdPtr  = Input(UInt(addrw.W))
    val push   = Input(Bool())
    val pop    = Input(Bool())
    val qVal   = Output(UInt(width.W))
    val qIdx   = Output(Bool())
    val wrPtr  = Output(Vec(NUM_WARPS, UInt(addrw.W)))
    val empty  = Output(Bool())
    val full   = Output(Bool())
  })

  // Per-warp pointer / status registers
  val wrPtrR = RegInit(VecInit(Seq.fill(NUM_WARPS)(0.U(addrw.W))))
  val emptyR = RegInit(VecInit(Seq.fill(NUM_WARPS)(true.B)))
  val fullR  = RegInit(VecInit(Seq.fill(NUM_WARPS)(false.B)))

  for (i <- 0 until NUM_WARPS) {
    val pushS = io.push && (io.wid === i.U)
    val popS  = io.pop  && (io.wid === i.U)

    when (pushS) {
      wrPtrR(i) := wrPtrR(i) + 1.U
      emptyR(i) := false.B
      fullR(i)  := (wrPtrR(i) === (depth - 1).U)
    }.elsewhen (popS) {
      // pop decrements by q_idx (1=primary, 0=secondary)
      wrPtrR(i) := wrPtrR(i) - io.qIdx.asUInt
      emptyR(i) := (io.rdPtr === 0.U) && io.qIdx
      fullR(i)  := false.B
    }
  }

  // BRAM address computation
  val bramAddrW = math.max(1, log2Ceil(math.max(bramSz, 2)))

  val waddr = Wire(UInt(bramAddrW.W))
  val raddr = Wire(UInt(bramAddrW.W))

  if (depth > 1 && NUM_WARPS > 1) {
    waddr := Mux(io.push,
      Cat(wrPtrR(io.wid), io.wid),
      Cat(io.rdPtr, io.wid))
    raddr := Cat(io.rdPtr, io.wid)
  } else if (depth > 1) {
    waddr := Mux(io.push, wrPtrR(0), io.rdPtr)
    raddr := io.rdPtr
  } else if (NUM_WARPS > 1) {
    waddr := io.wid
    raddr := 0.U
  } else {
    waddr := 0.U
    raddr := 0.U
  }

  // Dual-port BRAM: stores {q_idx(1), d1(width), d0(width)}
  val bramDataW = 1 + width * 2
  val mem = SyncReadMem(bramSz, UInt(bramDataW.W))

  // Read (registered)
  val rdData = mem.read(raddr, io.pop)

  val q0   = rdData(width - 1, 0)
  val q1   = rdData(2 * width - 1, width)
  val qIdx = rdData(bramDataW - 1)

  // Write: on push store {0, d1, d0}; on pop store {1, q1, q0}
  when (io.push || io.pop) {
    val wdata = Mux(io.push,
      Cat(false.B, io.d1, io.d0),
      Cat(true.B,  q1,    q0))
    mem.write(waddr, wdata)
  }

  io.qIdx := qIdx
  io.qVal  := Mux(qIdx, q0, q1)
  io.wrPtr := wrPtrR
  io.empty := emptyR(io.wid)
  io.full  := fullR(io.wid)
}
