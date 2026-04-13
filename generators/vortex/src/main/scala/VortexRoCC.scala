// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
//
// Rocket Chip RoCC accelerator wrapper for the Vortex GPGPU Chisel translation.
//
// RoCC command (CUSTOM0 = 0x0B):
//   rs1 = startup address  → wired directly to GPU (bypasses DCR)
//   rs2 = startup argument → written to DCR 0x003 (VX_DCR_BASE_STARTUP_ARG0)
//
// startup_addr is registered and held on gpu.io.startup_addr throughout
// execution; no DCR write is needed for it.  After writing the arg DCR, the
// GPU is released from reset and begins execution.  The RoCC busy signal is
// held high until the GPU reports it has finished (gpu.io.busy goes low).
// The GPU is re-asserted into reset so it is ready for the next invocation.

package vortex

import chisel3._
import chisel3.util._
import freechips.rocketchip.tile._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import org.chipsalliance.cde.config._
import org.chipsalliance.diplomacy.lazymodule._

// ---------------------------------------------------------------------------
// LazyRoCC wrapper
// ---------------------------------------------------------------------------
class VortexRoCC(opcodes: OpcodeSet)(implicit p: Parameters)
    extends LazyRoCC(opcodes) {

  // L3 memory-bus tag breakdown (matches VortexTopParams in VortexTop.scala):
  //   L3_MSHR_ADR_W = log2Ceil(L3_MSHR_SIZE) = 4   (L3_MSHR_SIZE = 16)
  //   L3_BANK_SEL   = 1                              (L3_NUM_BANKS = 1)
  //   tagValueBits  = 5  →  numSrcIds = 32
  private val tagValueBits = log2Ceil(VortexConfigConstants.L3_MSHR_SIZE) + 1
  private val numSrcIds    = 1 << tagValueBits

  // One TileLink client node per L3 memory port
  val memNodes = Seq.tabulate(VortexGPUPkg.L3_MEM_PORTS) { i =>
    TLClientNode(Seq(TLMasterPortParameters.v1(
      clients = Seq(TLMasterParameters.v1(
        name     = s"vortex-mem-$i",
        sourceId = IdRange(0, numSrcIds)
      ))
    )))
  }

  // Connect every memory node to the system bus via atlNode.
  // TLWidthWidget widens the inner (VortexRoCCModule-facing) TL data bus to
  // L3_LINE_SIZE bytes so that tl.d.bits.data is the full 512-bit cache line.
  // It also handles splitting wide A-channel Puts into narrow beats going out.
  memNodes.foreach { node =>
    atlNode := TLBuffer() := TLWidthWidget(VortexConfigConstants.L3_LINE_SIZE) := node
  }

  override lazy val module = new VortexRoCCModule(this)
}

// ---------------------------------------------------------------------------
// Module implementation
// ---------------------------------------------------------------------------
class VortexRoCCModule(outer: VortexRoCC)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) with HasCoreParameters {

  import VortexConfigConstants._
  import VortexGPUPkg._

  private val logLineSize  = log2Ceil(L3_LINE_SIZE)
  private val tagValueBits = log2Ceil(L3_MSHR_SIZE) + 1   // L3_MSHR_ADR_W + L3_BANK_SEL
  private val numSrcIds    = 1 << tagValueBits

  // -------------------------------------------------------------------------
  // GPU instance — held in reset until startup_addr is latched and gpuReset deasserted
  // -------------------------------------------------------------------------
  val gpuReset = RegInit(true.B)
  val gpu = withReset(gpuReset || reset.asBool) { Module(new VortexTop) }

  val sIdle :: sStart :: sReset :: sBusy :: Nil = Enum(4)
  val state        = RegInit(sIdle)
  val startup_addr = Reg(UInt(32.W))   // startup address (rs1) — wired directly
  val mpm_class    = RegInit(0.U(8.W)) // MPM class, not implemented yet

  // Both held stable for the entire execution
  gpu.io.startup_addr := startup_addr
  gpu.io.mpm_class    := mpm_class

  io.cmd.ready := state === sIdle
  
  // This is somewhat similar to the code in rtlsim vortex.cpp
  switch (state) {
    is (sIdle) {
      when (io.cmd.valid) {
        startup_addr := io.cmd.bits.rs1
        state        := sReset
      }
    }
    is (sReset) {
        gpuReset := false.B
        state   := sStart
    }
    // Needs 1 cycle to issue till it asserts busy
    is (sStart) {
        state := sBusy
    }
    is (sBusy) {
      when (!gpu.io.busy) {
        gpuReset := true.B
        state    := sIdle
      }
    }
  }

  io.busy      := state =/= sIdle
  io.interrupt := false.B // TODO: To fully implement, need the GPU to interrupt

  // No output
  io.resp.valid := false.B
  io.resp.bits  := DontCare

  // Tie off HellaCacheIO (memory is accessed via TL nodes instead)
  // TODO: Maybe a program-defined mechanism to exclusively access control (not data) bytes
  io.mem.req.valid          := false.B
  io.mem.req.bits           := DontCare
  io.mem.s1_kill            := false.B
  io.mem.s2_kill            := false.B
  io.mem.keep_clock_enabled := false.B

  // -------------------------------------------------------------------------
  // MemBus → TileLink adapters (one per L3 memory port)
  // -------------------------------------------------------------------------
  for (mp <- 0 until L3_MEM_PORTS) {
    val memBus     = gpu.io.mem_bus(mp)
    val (tl, edge) = outer.memNodes(mp).out(0)

    // Free-list: one bit per source ID, true = free
    val srcFree  = RegInit(VecInit(Seq.fill(numSrcIds)(true.B)))
    val allocOH  = PriorityEncoderOH(srcFree.asUInt)
    val allocId  = OHToUInt(allocOH)
    val canAlloc = srcFree.asUInt.orR

    // Per-source-ID tables to recover original Vortex tag on response
    val tagValTable = Reg(Vec(numSrcIds, UInt(tagValueBits.W)))
    val uuidTable   = Reg(Vec(numSrcIds, UInt(UUID_WIDTH.W)))

    // Convert Vortex word address to TL byte address
    val reqByteAddr = Cat(memBus.req.bits.addr, 0.U(logLineSize.W))

    val (_, getA) = edge.Get(
      fromSource = allocId,
      toAddress  = reqByteAddr,
      lgSize     = logLineSize.U
    )
    val (_, putA) = edge.Put(
      fromSource = allocId,
      toAddress  = reqByteAddr,
      lgSize     = logLineSize.U,
      data       = memBus.req.bits.data,
      mask       = memBus.req.bits.byteen
    )

    tl.a.valid       := memBus.req.valid && canAlloc && !gpuReset
    tl.a.bits        := Mux(memBus.req.bits.rw, putA, getA)
    memBus.req.ready := tl.a.ready && canAlloc

    // Allocate source ID and save original tag when request fires
    when (memBus.req.valid && tl.a.ready && canAlloc) {
      srcFree(allocId)     := false.B
      tagValTable(allocId) := memBus.req.bits.tag.value
      uuidTable(allocId)   := memBus.req.bits.tag.uuid
    }

    tl.d.ready                := memBus.rsp.ready && !gpuReset
    memBus.rsp.valid          := tl.d.valid
    memBus.rsp.bits.data      := tl.d.bits.data
    memBus.rsp.bits.tag.value := tagValTable(tl.d.bits.source)
    memBus.rsp.bits.tag.uuid  := uuidTable(tl.d.bits.source)

    // Free source ID on response
    when (tl.d.valid && tl.d.ready) {
      srcFree(tl.d.bits.source) := true.B
    }

    // Tie off probe / release / grant-ack channels
    tl.b.ready := true.B
    tl.c.valid := false.B
    tl.c.bits  := DontCare
    tl.e.valid := false.B
    tl.e.bits  := DontCare
  }
}
