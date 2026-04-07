// Vortex GPGPU RoCC accelerator.
//
// Uses RISC-V CUSTOM0 opcode space (opcode 0x0B).
// Instruction encoding (funct7 field dispatch):
//   funct  0  DEV_CAPS     : rs1=caps_id            → rd=value
//   funct  1  MEM_ALLOC    : rs1=size, rs2=flags    → rd=handle
//   funct  2  MEM_FREE     : rs1=handle
//   funct  3  MEM_ADDRESS  : rs1=handle             → rd=device_addr
//   funct  4  MEM_INFO     : rs1=0→free / 1→used   → rd=bytes
//   funct  5  COPY_TO_DEV  : rs1=handle, rs2=host_ptr  (size from SET_XFER_SIZE)
//   funct  6  COPY_FROM_DEV: rs1=host_ptr, rs2=handle (size from SET_XFER_SIZE)
//   funct  7  START        : rs1=kernel_addr, rs2=args_addr
//                            Writes DCR startup regs then releases GPU reset.
//   funct  8  READY_WAIT   : rs1=timeout_ms        → rd=0(done)/1(timeout)
//   funct  9  DCR_READ     : rs1=addr              → rd=value  (stub→0)
//   funct 10  DCR_WRITE    : rs1=addr, rs2=value
//   funct 11  MPM_QUERY    : rs1=addr, rs2=core_id → rd=counter (stub→0)
//   funct 12  SET_XFER_SIZE: rs1=size

package vortex

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import org.chipsalliance.diplomacy.lazymodule._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._

// ---------------------------------------------------------------------------
// Funct-field constants
// ---------------------------------------------------------------------------
object VortexFunct {
  val DEV_CAPS      = 0.U(7.W)
  val MEM_ALLOC     = 1.U(7.W)
  val MEM_FREE      = 2.U(7.W)
  val MEM_ADDRESS   = 3.U(7.W)
  val MEM_INFO      = 4.U(7.W)
  val COPY_TO_DEV   = 5.U(7.W)
  val COPY_FROM_DEV = 6.U(7.W)
  val START         = 7.U(7.W)
  val READY_WAIT    = 8.U(7.W)
  val DCR_READ      = 9.U(7.W)
  val DCR_WRITE     = 10.U(7.W)
  val MPM_QUERY     = 11.U(7.W)
  val SET_XFER_SIZE = 12.U(7.W)
}

// ---------------------------------------------------------------------------
// vx_dev_caps capability IDs (matches vortex.h)
// ---------------------------------------------------------------------------
object VortexCapId {
  val CACHE_LINE_SIZE = 0
  val NUM_THREADS     = 1
  val NUM_WARPS       = 2
  val NUM_CORES       = 3
  val NUM_CLUSTERS    = 4
  val NUM_BARRIERS    = 4   // alias kept for API compat (same slot as clusters)
  val LOCAL_MEM_SIZE  = 5
  val KERNEL_BUF_SIZE = 6
  val NUM_REGS        = 7
  val ISA             = 8
}

// ---------------------------------------------------------------------------
// LazyRoCC wrapper
// ---------------------------------------------------------------------------
class VortexRoCC(opcodes: OpcodeSet)(implicit p: Parameters)
    extends LazyRoCC(opcodes) {

  override lazy val module = new VortexRoCCModule(this)
}

// ---------------------------------------------------------------------------
// Module implementation
// ---------------------------------------------------------------------------
class VortexRoCCModule(outer: VortexRoCC)(implicit p: Parameters)
    extends LazyRoCCModuleImp(outer) with HasCoreParameters {




  val gpu = Module(new VortexBlackBox(vp))

  // GPU stays in reset until START is issued; re-enters reset after
  // READY_WAIT completes so successive kernel launches work correctly.
  val gpuReset = RegInit(true.B)
  gpu.io.clk   := clock
  gpu.io.reset := reset.asBool || gpuReset

  // ------------------------------------------------------------------
  // Wire AXI4 master port to the BlackBox
  // ------------------------------------------------------------------
  val (axi, _) = outer.axi4MasterNode.out(0)

  // Write address channel
  axi.aw.valid         := gpu.io.m_axi_awvalid
  gpu.io.m_axi_awready := axi.aw.ready
  axi.aw.bits.addr     := gpu.io.m_axi_awaddr
  axi.aw.bits.id       := gpu.io.m_axi_awid
  axi.aw.bits.len      := gpu.io.m_axi_awlen
  axi.aw.bits.size     := gpu.io.m_axi_awsize
  axi.aw.bits.burst    := gpu.io.m_axi_awburst
  axi.aw.bits.lock     := gpu.io.m_axi_awlock(0)  // AXI4 uses 1-bit lock
  axi.aw.bits.cache    := gpu.io.m_axi_awcache
  axi.aw.bits.prot     := gpu.io.m_axi_awprot
  axi.aw.bits.qos      := gpu.io.m_axi_awqos

  // Write data channel
  axi.w.valid          := gpu.io.m_axi_wvalid
  gpu.io.m_axi_wready  := axi.w.ready
  axi.w.bits.data      := gpu.io.m_axi_wdata
  axi.w.bits.strb      := gpu.io.m_axi_wstrb
  axi.w.bits.last      := gpu.io.m_axi_wlast

  // Write response channel
  gpu.io.m_axi_bvalid  := axi.b.valid
  axi.b.ready          := gpu.io.m_axi_bready
  gpu.io.m_axi_bid     := axi.b.bits.id
  gpu.io.m_axi_bresp   := axi.b.bits.resp

  // Read address channel
  axi.ar.valid         := gpu.io.m_axi_arvalid
  gpu.io.m_axi_arready := axi.ar.ready
  axi.ar.bits.addr     := gpu.io.m_axi_araddr
  axi.ar.bits.id       := gpu.io.m_axi_arid
  axi.ar.bits.len      := gpu.io.m_axi_arlen
  axi.ar.bits.size     := gpu.io.m_axi_arsize
  axi.ar.bits.burst    := gpu.io.m_axi_arburst
  axi.ar.bits.lock     := gpu.io.m_axi_arlock(0)
  axi.ar.bits.cache    := gpu.io.m_axi_arcache
  axi.ar.bits.prot     := gpu.io.m_axi_arprot
  axi.ar.bits.qos      := gpu.io.m_axi_arqos

  // Read data channel
  gpu.io.m_axi_rvalid  := axi.r.valid
  axi.r.ready          := gpu.io.m_axi_rready
  gpu.io.m_axi_rdata   := axi.r.bits.data
  gpu.io.m_axi_rlast   := axi.r.bits.last
  gpu.io.m_axi_rid     := axi.r.bits.id
  gpu.io.m_axi_rresp   := axi.r.bits.resp

  // ------------------------------------------------------------------
  // DCR write register (one-cycle pulse to GPU)
  // ------------------------------------------------------------------
  val dcrValid = RegInit(false.B)
  val dcrAddr  = Reg(UInt(12.W))
  val dcrData  = Reg(UInt(32.W))

  gpu.io.dcr_wr_valid := dcrValid
  gpu.io.dcr_wr_addr  := dcrAddr
  gpu.io.dcr_wr_data  := dcrData

  // Auto-clear after one cycle; individual blocks below may override this.
  when (dcrValid) { dcrValid := false.B }

  // ------------------------------------------------------------------
  // Memory allocation table
  // ------------------------------------------------------------------
  val NUM_ALLOC_SLOTS = 16
  // Stub device-memory window above 4 GB (Vortex device address space)
  val DEV_MEM_BASE = "h_1_0000_0000".U(64.W)
  val DEV_MEM_SIZE = "h_1000_0000".U(64.W)  // 256 MB

  val allocValid = RegInit(VecInit(Seq.fill(NUM_ALLOC_SLOTS)(false.B)))
  val allocBase  = Reg(Vec(NUM_ALLOC_SLOTS, UInt(64.W)))
  val allocSize  = Reg(Vec(NUM_ALLOC_SLOTS, UInt(64.W)))
  val devMemNext = RegInit(DEV_MEM_BASE)

  // ------------------------------------------------------------------
  // DMA transfer-size latch
  // ------------------------------------------------------------------
  val xferSize = RegInit(0.U(64.W))

  // ------------------------------------------------------------------
  // GPU execution state
  // ------------------------------------------------------------------
  val gpuRunning = RegInit(false.B)

  // ------------------------------------------------------------------
  // START state machine: write 4 DCR startup regs, then release GPU reset.
  //
  // Vortex startup registers (from VX_types.vh):
  //   0x001  VX_DCR_BASE_STARTUP_ADDR0  – kernel PC bits [31:0]
  //   0x002  VX_DCR_BASE_STARTUP_ADDR1  – kernel PC bits [63:32]
  //   0x003  VX_DCR_BASE_STARTUP_ARG0   – args pointer bits [31:0]
  //   0x004  VX_DCR_BASE_STARTUP_ARG1   – args pointer bits [63:32]
  //
  // The START RoCC command supplies rs1=kernel_addr, rs2=args_addr.
  // ------------------------------------------------------------------
  val s_cmd :: s_dcr1 :: s_dcr2 :: s_dcr3 :: s_dcr4 :: s_launch :: Nil = Enum(6)
  val startState = RegInit(s_cmd)
  val startAddr  = Reg(UInt(xLen.W))
  val startArg   = Reg(UInt(xLen.W))

  // DCR state machine runs concurrently; its assignments come after the
  // default dcrValid := false.B, so they take priority (last-wins in Chisel).
  switch (startState) {
    is (s_dcr1) {
      dcrValid   := true.B
      dcrAddr    := 0x001.U
      dcrData    := startAddr(31, 0)
      startState := s_dcr2
    }
    is (s_dcr2) {
      dcrValid   := true.B
      dcrAddr    := 0x002.U
      dcrData    := (if (xLen > 32) startAddr(63, 32) else 0.U(32.W))
      startState := s_dcr3
    }
    is (s_dcr3) {
      dcrValid   := true.B
      dcrAddr    := 0x003.U
      dcrData    := startArg(31, 0)
      startState := s_dcr4
    }
    is (s_dcr4) {
      dcrValid   := true.B
      dcrAddr    := 0x004.U
      dcrData    := (if (xLen > 32) startArg(63, 32) else 0.U(32.W))
      startState := s_launch
    }
    is (s_launch) {
      gpuReset   := false.B   // release GPU reset → execution begins
      gpuRunning := true.B
      startState := s_cmd
    }
  }

  // ------------------------------------------------------------------
  // DMA countdown state machine (stub: ~1 cycle per 64-B cache line)
  // ------------------------------------------------------------------
  val s_idle :: s_dma :: Nil = Enum(2)
  val dmaState  = RegInit(s_idle)
  val dmaCycles = RegInit(0.U(32.W))
  val dmaRd     = Reg(UInt(5.W))
  val dmaHasRd  = Reg(Bool())

  // ------------------------------------------------------------------
  // Incoming command queue
  // ------------------------------------------------------------------
  val cmd   = Queue(io.cmd, 2)
  val funct = cmd.bits.inst.funct
  val rs1   = cmd.bits.rs1
  val rs2   = cmd.bits.rs2
  val rd    = cmd.bits.inst.rd
  val hasRd = cmd.bits.inst.xd

  // ------------------------------------------------------------------
  // Response queue
  // ------------------------------------------------------------------
  val respQ = Module(new Queue(new RoCCResponse, 4))
  io.resp <> respQ.io.deq

  cmd.ready          := false.B
  respQ.io.enq.valid := false.B
  respQ.io.enq.bits  := DontCare

  val respValid = WireDefault(false.B)
  val respData  = WireDefault(0.U(xLen.W))
  val respRd    = WireDefault(rd)

  // ------------------------------------------------------------------
  // Command dispatch – only when idle (no START sequence or DMA in flight)
  // ------------------------------------------------------------------
  when (startState === s_cmd && dmaState === s_idle && cmd.valid) {
    switch (funct) {

      // --- vx_dev_caps ---
      is (VortexFunct.DEV_CAPS) {
        val capVal = MuxLookup(rs1(7, 0), 0.U(xLen.W))(Seq(
          VortexCapId.CACHE_LINE_SIZE.U -> 64.U(xLen.W),
          VortexCapId.NUM_THREADS.U     -> vp.numThreads.U(xLen.W),
          VortexCapId.NUM_WARPS.U       -> vp.numWarps.U(xLen.W),
          VortexCapId.NUM_CORES.U       -> vp.numCores.U(xLen.W),
          VortexCapId.NUM_CLUSTERS.U    -> vp.numClusters.U(xLen.W),
          VortexCapId.LOCAL_MEM_SIZE.U  -> 49152.U(xLen.W),
          VortexCapId.KERNEL_BUF_SIZE.U -> 4096.U(xLen.W),
          VortexCapId.NUM_REGS.U        -> 32.U(xLen.W),
          VortexCapId.ISA.U             -> 0.U(xLen.W)
        ))
        respData  := capVal
        respValid := hasRd
        cmd.ready := !hasRd || respQ.io.enq.ready
      }

      // --- vx_mem_alloc ---
      is (VortexFunct.MEM_ALLOC) {
        val freeSlots = VecInit(allocValid.map(!_))
        val freeSlot  = PriorityEncoder(freeSlots)
        val hasFree   = freeSlots.asUInt.orR
        when (hasFree && (!hasRd || respQ.io.enq.ready)) {
          allocValid(freeSlot) := true.B
          allocBase(freeSlot)  := devMemNext
          allocSize(freeSlot)  := rs1
          devMemNext           := devMemNext + rs1
          respData  := freeSlot
          respValid := hasRd
          cmd.ready := true.B
        }
      }

      // --- vx_mem_free ---
      is (VortexFunct.MEM_FREE) {
        val slot = rs1(log2Ceil(NUM_ALLOC_SLOTS) - 1, 0)
        allocValid(slot) := false.B
        cmd.ready        := true.B
      }

      // --- vx_mem_address ---
      is (VortexFunct.MEM_ADDRESS) {
        val slot = rs1(log2Ceil(NUM_ALLOC_SLOTS) - 1, 0)
        respData  := allocBase(slot)
        respValid := hasRd
        cmd.ready := !hasRd || respQ.io.enq.ready
      }

      // --- vx_mem_info  (rs1=0→free, rs1=1→used) ---
      is (VortexFunct.MEM_INFO) {
        val used = devMemNext - DEV_MEM_BASE
        val free = DEV_MEM_SIZE - used
        respData  := Mux(rs1(0), used, free)
        respValid := hasRd
        cmd.ready := !hasRd || respQ.io.enq.ready
      }

      // --- set transfer size for next DMA ---
      is (VortexFunct.SET_XFER_SIZE) {
        xferSize  := rs1
        cmd.ready := true.B
      }

      // --- vx_copy_to_dev (stub DMA) ---
      is (VortexFunct.COPY_TO_DEV) {
        dmaCycles := (xferSize >> 6) + 4.U
        dmaRd     := rd
        dmaHasRd  := hasRd
        dmaState  := s_dma
        cmd.ready := true.B
      }

      // --- vx_copy_from_dev (stub DMA) ---
      is (VortexFunct.COPY_FROM_DEV) {
        dmaCycles := (xferSize >> 6) + 4.U
        dmaRd     := rd
        dmaHasRd  := hasRd
        dmaState  := s_dma
        cmd.ready := true.B
      }

      // --- vx_start ---
      // Captures kernel/args addresses, then the START state machine writes
      // DCR startup registers and releases the GPU reset.
      is (VortexFunct.START) {
        startAddr  := rs1
        startArg   := rs2
        gpuReset   := true.B    // hold reset while writing DCRs
        startState := s_dcr1
        cmd.ready  := true.B
      }

      // --- vx_ready_wait ---
      // Polls gpu.io.busy; blocks until the GPU finishes execution.
      // Returns 0 (done) when busy de-asserts.  No timeout implemented.
      is (VortexFunct.READY_WAIT) {
        when (!gpu.io.busy || !gpuRunning) {
          gpuReset   := true.B    // re-assert reset for clean next launch
          gpuRunning := false.B
          respData   := 0.U       // 0 = done (no timeout)
          respValid  := hasRd
          cmd.ready  := !hasRd || respQ.io.enq.ready
        }
      }

      // --- vx_dcr_read (stub – always returns 0) ---
      is (VortexFunct.DCR_READ) {
        respData  := 0.U
        respValid := hasRd
        cmd.ready := !hasRd || respQ.io.enq.ready
      }

      // --- vx_dcr_write – direct write to Vortex DCR register ---
      is (VortexFunct.DCR_WRITE) {
        dcrValid  := true.B
        dcrAddr   := rs1(11, 0)
        dcrData   := rs2(31, 0)
        cmd.ready := true.B
      }

      // --- vx_mpm_query (stub – returns 0) ---
      is (VortexFunct.MPM_QUERY) {
        respData  := 0.U
        respValid := hasRd
        cmd.ready := !hasRd || respQ.io.enq.ready
      }
    }
  }

  // ------------------------------------------------------------------
  // DMA countdown (busy state)
  // ------------------------------------------------------------------
  when (dmaState === s_dma) {
    when (dmaCycles > 0.U) {
      dmaCycles := dmaCycles - 1.U
    } .otherwise {
      when (!dmaHasRd || respQ.io.enq.ready) {
        dmaState := s_idle
      }
      respData  := 0.U
      respValid := dmaHasRd
      respRd    := dmaRd
    }
  }

  // ------------------------------------------------------------------
  // Response queue drive
  // ------------------------------------------------------------------
  respQ.io.enq.valid    := respValid
  respQ.io.enq.bits.rd   := respRd
  respQ.io.enq.bits.data := respData

  // ------------------------------------------------------------------
  // Housekeeping
  // ------------------------------------------------------------------
  io.busy      := gpuRunning || (dmaState === s_dma) ||
                  cmd.valid  || (startState =/= s_cmd)
  io.interrupt := false.B

  // Host-side DMA (COPY_TO/FROM_DEV) uses a countdown stub.
  // A real implementation would use io.mem (HellaCacheIO) for host-side
  // transfers and the GPU's own AXI port for device-side transfers.
  io.mem.req.valid := false.B
}
