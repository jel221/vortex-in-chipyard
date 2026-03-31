// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
// Translated from VX_split_join.sv

package vortex

import chisel3._
import chisel3.util._

import VortexGPUPkg._
import VortexConfigConstants._

/** Split/Join handler for warp divergence.
 *  Mirrors VX_split_join.sv.
 *
 *  @param outReg pipeline register depth on the join output
 */
class VxSplitJoin(val outReg: Int = 0) extends Module {
  val io = IO(new Bundle {
    // inputs
    val valid    = Input(Bool())
    val wid      = Input(UInt(NW_WIDTH.W))
    val split    = Input(new SplitBundle(NUM_THREADS, PC_BITS))
    val sjoin    = Input(new JoinBundle(DV_STACK_SIZEW))
    val stackWid = Input(UInt(NW_WIDTH.W))
    // outputs
    val joinValid  = Output(Bool())
    val joinIsDvg  = Output(Bool())
    val joinIsElse = Output(Bool())
    val joinWid    = Output(UInt(NW_WIDTH.W))
    val joinTmask  = Output(UInt(NUM_THREADS.W))
    val joinPc     = Output(UInt(PC_BITS.W))
    val stackPtr   = Output(UInt(DV_STACK_SIZEW.W))
  })

  if (NT_BITS != 0) {
    val splitValid = io.valid && io.split.valid
    val sjoinValid = io.valid && io.sjoin.valid

    // IPDOM stack
    val ipdom = Module(new VxIpdomStack(
      width = NUM_THREADS + PC_BITS,
      depth = DV_STACK_SIZE
    ))

    // d0 = {then_tmask | else_tmask, 0}  (rejoin mask, no PC)
    // d1 = {else_tmask, next_pc}          (else branch target)
    val d0 = Cat(io.split.then_tmask | io.split.else_tmask, 0.U(PC_BITS.W))
    val d1 = Cat(io.split.else_tmask, io.split.next_pc)

    val sjoinIsDvg = io.sjoin.stack_ptr =/= ipdom.io.wrPtr(io.wid)

    ipdom.io.wid   := io.wid
    ipdom.io.d0    := d0
    ipdom.io.d1    := d1
    ipdom.io.push  := splitValid && io.split.is_dvg
    ipdom.io.pop   := sjoinValid && sjoinIsDvg
    ipdom.io.rdPtr := io.sjoin.stack_ptr

    val ipdomTmask = ipdom.io.qVal(NUM_THREADS + PC_BITS - 1, PC_BITS)
    val ipdomPc    = ipdom.io.qVal(PC_BITS - 1, 0)
    val ipdomIdx   = ipdom.io.qIdx

    // Pipeline register (depth = outReg)
    val joinValidR  = RegInit(false.B)
    val joinWidR    = Reg(UInt(NW_WIDTH.W))
    val joinIsDvgR  = Reg(Bool())
    val joinIsElseR = Reg(Bool())
    val joinTmaskR  = Reg(UInt(NUM_THREADS.W))
    val joinPcR     = Reg(UInt(PC_BITS.W))

    if (outReg > 0) {
      joinValidR  := sjoinValid
      joinWidR    := io.wid
      joinIsDvgR  := sjoinIsDvg
      joinIsElseR := !ipdomIdx
      joinTmaskR  := ipdomTmask
      joinPcR     := ipdomPc
    }

    io.joinValid  := (if (outReg > 0) joinValidR  else sjoinValid)
    io.joinWid    := (if (outReg > 0) joinWidR    else io.wid)
    io.joinIsDvg  := (if (outReg > 0) joinIsDvgR  else sjoinIsDvg)
    io.joinIsElse := (if (outReg > 0) joinIsElseR else !ipdomIdx)
    io.joinTmask  := (if (outReg > 0) joinTmaskR  else ipdomTmask)
    io.joinPc     := (if (outReg > 0) joinPcR     else ipdomPc)
    io.stackPtr   := ipdom.io.wrPtr(io.stackWid)

  } else {
    // NT_BITS == 0: single-thread, no divergence
    val sjoinValid = io.valid && io.sjoin.valid

    io.joinValid  := sjoinValid
    io.joinWid    := io.wid
    io.joinIsDvg  := false.B
    io.joinIsElse := false.B
    io.joinTmask  := 1.U
    io.joinPc     := 0.U(PC_BITS.W)
    io.stackPtr   := 0.U(DV_STACK_SIZEW.W)
  }
}
