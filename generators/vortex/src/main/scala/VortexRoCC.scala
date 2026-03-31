// Copyright © 2019-2023
// Licensed under the Apache License, Version 2.0
//
// Rocket Chip RoCC accelerator wrapper for the Vortex GPGPU Chisel translation.
//
// RoCC command (CUSTOM0 = 0x0B):
//   rs1 = startup address  → written to DCR 0x001 (VX_DCR_BASE_STARTUP_ADDR0)
//   rs2 = startup argument → written to DCR 0x003 (VX_DCR_BASE_STARTUP_ARG0)
//
// After writing the two startup DCRs, the GPU is released from reset and
// begins execution.  The RoCC busy signal is held high until the GPU reports
// it has finished (gpu.io.busy goes low).  The GPU is re-asserted into reset
// so it is ready for the next invocation.

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
  val memNodes = Seq.tabulate(VortexConfigConstants.L3_MEM_PORTS) { i =>
    TLClientNode(Seq(TLMasterPortParameters.v1(
      clients = Seq(TLMasterParameters.v1(
        name     = s"vortex-mem-$i",
        sourceId = IdRange(0, numSrcIds)
      ))
    )))
  }

  // Connect every memory node to the system bus via atlNode
  memNodes.foreach { node => atlNode := TLBuffer() := node }

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
  // GPU instance — held in reset until startup DCRs have been written
  // -------------------------------------------------------------------------
  val gpuReset = RegInit(true.B)
  val gpu = withReset(gpuReset || reset.asBool) { Module(new VortexTop) }

  // DCR bus defaults (driven by state machine below)
  gpu.io.dcr_write_valid := false.B
  gpu.io.dcr_write_addr  := 0.U
  gpu.io.dcr_write_data  := 0.U

  // -------------------------------------------------------------------------
  // State machine: capture RoCC args → write DCRs → release reset → wait
  // -------------------------------------------------------------------------
  val sIdle :: sWriteAddr :: sWriteArg :: sStart :: sBusy :: Nil = Enum(5)
  val state   = RegInit(sIdle)
  val addrReg = Reg(UInt(32.W))   // startup address  (rs1[31:0])
  val argReg  = Reg(UInt(32.W))   // startup argument (rs2[31:0])

  io.cmd.ready := state === sIdle

  switch (state) {
    is (sIdle) {
      when (io.cmd.valid) {
        addrReg := io.cmd.bits.rs1(31, 0)
        argReg  := io.cmd.bits.rs2(31, 0)
        state   := sWriteAddr
      }
    }
    is (sWriteAddr) {
      gpu.io.dcr_write_valid := true.B
      gpu.io.dcr_write_addr  := VX_DCR_BASE_STARTUP_ADDR0.U
      gpu.io.dcr_write_data  := addrReg
      state := sWriteArg
    }
    is (sWriteArg) {
      gpu.io.dcr_write_valid := true.B
      gpu.io.dcr_write_addr  := VX_DCR_BASE_STARTUP_ARG0.U
      gpu.io.dcr_write_data  := argReg
      state := sStart
    }
    is (sStart) {
      gpuReset := false.B
      state    := sBusy
    }
    is (sBusy) {
      when (!gpu.io.busy) {
        gpuReset := true.B
        state    := sIdle
      }
    }
  }

  io.busy      := state =/= sIdle
  io.interrupt := false.B

  // No RoCC response needed
  io.resp.valid := false.B
  io.resp.bits  := DontCare

  // Tie off HellaCacheIO (memory is accessed via TL nodes instead)
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

    // Per-port UUID save table: entry = tag.value index (tagValueBits wide)
    val uuidTable = Reg(Vec(numSrcIds, UInt(UUID_WIDTH.W)))

    // Convert Vortex word address to TL byte address
    val reqByteAddr = Cat(memBus.req.bits.addr, 0.U(logLineSize.W))

    val (_, getA) = edge.Get(
      fromSource = memBus.req.bits.tag.value,
      toAddress  = reqByteAddr,
      lgSize     = logLineSize.U
    )
    val (_, putA) = edge.Put(
      fromSource = memBus.req.bits.tag.value,
      toAddress  = reqByteAddr,
      lgSize     = logLineSize.U,
      data       = memBus.req.bits.data,
      mask       = memBus.req.bits.byteen
    )

    tl.a.valid       := memBus.req.valid
    tl.a.bits        := Mux(memBus.req.bits.rw, putA, getA)
    memBus.req.ready := tl.a.ready

    // Save UUID when the request fires; return it with the response
    when (memBus.req.valid && tl.a.ready) {
      uuidTable(memBus.req.bits.tag.value) := memBus.req.bits.tag.uuid
    }

    tl.d.ready                := memBus.rsp.ready
    memBus.rsp.valid          := tl.d.valid
    memBus.rsp.bits.data      := tl.d.bits.data
    memBus.rsp.bits.tag.value := tl.d.bits.source
    memBus.rsp.bits.tag.uuid  := uuidTable(tl.d.bits.source)

    // Tie off probe / release / grant-ack channels
    tl.b.ready := true.B
    tl.c.valid := false.B
    tl.c.bits  := DontCare
    tl.e.valid := false.B
    tl.e.bits  := DontCare
  }
}
