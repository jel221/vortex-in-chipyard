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

// Chisel translations of:
//   VX_local_mem.sv     – banked scratchpad memory with crossbar dispatch
//   VX_local_mem_top.sv – flattened-port wrapper around VX_local_mem

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// LocalMem
//
// Translation of VX_local_mem.sv.
//
// A banked scratchpad SRAM shared across NUM_REQS request ports.  Requests
// are dispatched to NUM_BANKS banks via a crossbar (VX_stream_xbar in SV,
// implemented here as a priority arbitration crossbar).  Each bank holds a
// single-port SRAM (VX_sp_ram with OUT_REG=1).
//
// Key SV behaviour preserved:
//   • Bank selection: addr[BANK_SEL_BITS-1:0] (low bits) → bank index.
//   • Bank address:   addr[BANK_SEL_BITS + BANK_ADDR_WIDTH - 1 : BANK_SEL_BITS].
//   • Read-during-write hazard: if the previous cycle wrote the same bank
//     address that is now being read, the read is stalled (per_bank_req_ready
//     deasserted; valid is also suppressed for that read).
//   • Write response is dropped (writes do not produce a response).
//   • The SRAM output is registered (OUT_REG=1); a VX_pipe_buffer (2-entry
//     elastic buffer) holds the {req_selector, tag} metadata while the data
//     propagates through the register stage.
//   • Responses are gathered back across a crossbar (priority) keyed on the
//     per_bank_req_idx (which input port originated the request).
//
// Parameters
//   size       – SIZE (bytes); default 1024*16*8
//   numReqs    – NUM_REQS (number of access ports)
//   numBanks   – NUM_BANKS
//   wordSize   – WORD_SIZE (bytes); default XLEN/8 = 4
//   tagWidth   – TAG_WIDTH
//   addrWidth  – ADDR_WIDTH = CLOG2(SIZE) = BANK_ADDR_WIDTH + CLOG2(NUM_BANKS)
//   flagsWidth – MEM_FLAGS_WIDTH (default 3; flags are accepted but ignored)
//   uuidWidth  – UUID sub-field width (default 0)
// ---------------------------------------------------------------------------

class LocalMem(
    size:       Int = 1024 * 16 * 8,
    numReqs:    Int = 4,
    numBanks:   Int = 4,
    wordSize:   Int = 4,
    tagWidth:   Int = 16,
    flagsWidth: Int = 3,
    uuidWidth:  Int = 0
) extends Module {
  val wordWidth      = wordSize * 8
  val numWords       = size / wordSize
  val wordsPerBank   = numWords / numBanks
  val bankAddrWidth  = log2Ceil(wordsPerBank)
  val bankSelBits    = if (numBanks > 1) log2Ceil(numBanks) else 0
  val addrWidth      = bankAddrWidth + math.max(bankSelBits, 0)
  val reqSelBits     = if (numReqs > 1) log2Ceil(numReqs) else 1  // UP(CLOG2(NUM_REQS))
  val reqSelWidth    = math.max(1, reqSelBits)

  val io = IO(new Bundle {
    val memBus = Flipped(Vec(numReqs, new MemBusBundle(wordSize, addrWidth, flagsWidth, tagWidth, uuidWidth)))
  })

  // -----------------------------------------------------------------------
  // Bank selection and address extraction
  // -----------------------------------------------------------------------

  // req_bank_idx[i] = addr[BANK_SEL_BITS-1:0]
  val reqBankIdx = Wire(Vec(numReqs, UInt(math.max(1, bankSelBits).W)))
  for (i <- 0 until numReqs) {
    if (numBanks > 1) reqBankIdx(i) := io.memBus(i).req.bits.addr(bankSelBits - 1, 0)
    else              reqBankIdx(i) := 0.U
  }

  // req_bank_addr[i] = addr[BANK_SEL_BITS + BANK_ADDR_WIDTH - 1 : BANK_SEL_BITS]
  val reqBankAddr = Wire(Vec(numReqs, UInt(bankAddrWidth.W)))
  for (i <- 0 until numReqs) {
    reqBankAddr(i) := io.memBus(i).req.bits.addr(bankSelBits + bankAddrWidth - 1, bankSelBits)
  }

  // -----------------------------------------------------------------------
  // Crossbar: dispatch requests from NUM_REQS ports to NUM_BANKS banks
  //
  // Model: for each bank, use a priority arbiter among the requests that
  // target that bank.  Each request can only go to one bank (req_bank_idx).
  // We record `per_bank_req_idx` (which input port won) for routing responses.
  //
  // SV uses VX_stream_xbar with ARBITER="P" and OUT_BUF=3 (registered FIFO).
  // We model the registered output with a Queue(entries=2).
  // -----------------------------------------------------------------------

  // Per-bank request state (registered output buffer).
  case class BankReqBundle() extends Bundle {
    val rw     = Bool()
    val addr   = UInt(bankAddrWidth.W)
    val data   = UInt(wordWidth.W)
    val byteen = UInt(wordSize.W)
    val tag    = UInt(tagWidth.W)
    val reqIdx = UInt(reqSelWidth.W)  // which input port sent this request
  }

  // For each bank: priority arbiter over all inputs targeting that bank.
  val bankReqQueues = Seq.fill(numBanks)(
    Module(new Queue(new BankReqBundle(), entries = 2, pipe = false, flow = false))
  )

  // Track which input gets ready from each bank (for req.ready back-pressure).
  val reqReadyFromBank = Wire(Vec(numReqs, Vec(numBanks, Bool())))
  for (i <- 0 until numReqs) for (b <- 0 until numBanks) reqReadyFromBank(i)(b) := false.B

  for (b <- 0 until numBanks) {
    // Priority arbiter: input 0 has highest priority (matches SV "P" arbiter).
    // We find the first valid request targeting this bank.
    val candidates = (0 until numReqs).map { i =>
      io.memBus(i).req.valid && (reqBankIdx(i) === b.U)
    }
    val winner    = PriorityEncoder(VecInit(candidates))
    val anyValid  = candidates.reduce(_ || _)

    val enqBundle = Wire(new BankReqBundle())
    enqBundle.rw     := io.memBus(winner).req.bits.rw
    enqBundle.addr   := reqBankAddr(winner)
    enqBundle.data   := io.memBus(winner).req.bits.data
    enqBundle.byteen := io.memBus(winner).req.bits.byteen
    enqBundle.tag    := io.memBus(winner).req.bits.tag.asUInt
    enqBundle.reqIdx := winner

    bankReqQueues(b).io.enq.valid := anyValid
    bankReqQueues(b).io.enq.bits  := enqBundle

    // Feed back ready: the winning input for this bank is ready when the bank
    // queue can accept.
    for (i <- 0 until numReqs) {
      reqReadyFromBank(i)(b) := anyValid && (winner === i.U) && bankReqQueues(b).io.enq.ready
    }
  }

  // Combine ready signals: input i is ready if it was served by any bank.
  for (i <- 0 until numReqs) {
    io.memBus(i).req.ready := reqReadyFromBank(i).reduce(_ || _)
  }

  // -----------------------------------------------------------------------
  // Per-bank SRAM + read-during-write hazard + pipe buffer
  // -----------------------------------------------------------------------

  case class BankRspBundle() extends Bundle {
    val data   = UInt(wordWidth.W)
    val tag    = UInt(tagWidth.W)
    val reqIdx = UInt(reqSelWidth.W)
  }

  val bankRspValids = Wire(Vec(numBanks, Bool()))
  val bankRspBundles = Wire(Vec(numBanks, new BankRspBundle()))
  val bankRspReadys  = Wire(Vec(numBanks, Bool()))

  for (b <- 0 until numBanks) {
    val deq = bankReqQueues(b).io.deq

    // Read-during-write hazard registers.
    val lastWrValid = RegInit(false.B)
    val lastWrAddr  = Reg(UInt(bankAddrWidth.W))

    val isRdwHazard = lastWrValid && !deq.bits.rw && (deq.bits.addr === lastWrAddr)

    // SRAM (single-port, registered output).
    val mem = SyncReadMem(wordsPerBank, Vec(wordSize, UInt(8.W)))

    // bank_rsp_ready comes from the pipe buffer downstream.
    val bankRspReady = Wire(Bool())

    // bank_req_ready (to the queue): writes pass through freely; reads stall on
    // hazard or when the response pipe buffer is full.
    val bankReqReady = (bankRspReady || deq.bits.rw) && !isRdwHazard
    deq.ready := bankReqReady

    // Perform the SRAM operation.
    val rdata = Wire(Vec(wordSize, UInt(8.W)))
    rdata := DontCare

    when (deq.valid && bankReqReady) {
      when (deq.bits.rw) {
        // Write with byte-enable.
        val wdataVec = VecInit((0 until wordSize).map(j =>
          deq.bits.data(j * 8 + 7, j * 8)
        ))
        mem.write(deq.bits.addr, wdataVec, (0 until wordSize).map(j =>
          deq.bits.byteen(j)
        ).toSeq)
      } .otherwise {
        // Read (registered output: the read data is available next cycle).
        val rd = mem.read(deq.bits.addr, !deq.bits.rw)
        rdata := rd
      }
    }

    // Update hazard registers.
    lastWrValid := deq.valid && bankReqReady && deq.bits.rw
    lastWrAddr  := deq.bits.addr

    // Valid read response: generated the cycle after a successful read (SV
    // assigns bank_rsp_valid = req_valid && ~req_rw && ~rdw_hazard; this is
    // combinational in SV because the RAM output is registered separately,
    // but here we produce the valid one cycle later via RegNext to match
    // OUT_REG=1 in VX_sp_ram).
    val bankRspValidNext = RegNext(deq.valid && bankReqReady && !deq.bits.rw && !isRdwHazard, false.B)

    // Pipe buffer: holds {reqIdx, tag} for one cycle while SRAM data propagates.
    // Corresponds to VX_pipe_buffer in SV (2-entry elastic buffer).
    val pipeBuf = Module(new Queue(new BankRspBundle(), entries = 2, pipe = true, flow = false))
    pipeBuf.io.enq.valid     := bankRspValidNext
    pipeBuf.io.enq.bits.data := Cat(rdata.reverse)
    pipeBuf.io.enq.bits.tag  := RegNext(deq.bits.tag)
    pipeBuf.io.enq.bits.reqIdx := RegNext(deq.bits.reqIdx)

    bankRspReady         := pipeBuf.io.enq.ready
    bankRspValids(b)     := pipeBuf.io.deq.valid
    bankRspBundles(b)    := pipeBuf.io.deq.bits
    bankRspReadys(b)     := pipeBuf.io.deq.ready
    pipeBuf.io.deq.ready := bankRspReadys(b)
  }

  // -----------------------------------------------------------------------
  // Response crossbar: route bank responses back to originating input ports
  //
  // Mirrors VX_stream_xbar with NUM_INPUTS=NUM_BANKS, NUM_OUTPUTS=NUM_REQS,
  // ARBITER="P" (priority), OUT_BUF=OUT_BUF.
  // -----------------------------------------------------------------------

  for (i <- 0 until numReqs) {
    // Find the first bank that has a valid response targeting input i.
    val bankCandidates = (0 until numBanks).map { b =>
      bankRspValids(b) && (bankRspBundles(b).reqIdx === i.U)
    }
    val anyBankRsp  = bankCandidates.reduce(_ || _)
    val winnerBank  = PriorityEncoder(VecInit(bankCandidates))

    io.memBus(i).rsp.valid      := anyBankRsp
    io.memBus(i).rsp.bits.data  := bankRspBundles(winnerBank).data
    io.memBus(i).rsp.bits.tag   :=
      bankRspBundles(winnerBank).tag.asTypeOf(io.memBus(i).rsp.bits.tag)

    // Feed ready back to the winning bank.
    for (b <- 0 until numBanks) {
      bankRspReadys(b) :=
        bankCandidates(b) && (winnerBank === b.U) && io.memBus(i).rsp.ready
    }
  }
}

// ---------------------------------------------------------------------------
// LocalMemTop
//
// Translation of VX_local_mem_top.sv.
//
// A thin flattened-port wrapper around LocalMem.  The SV module converts the
// individual array ports (mem_req_valid, mem_req_rw, …) into VX_mem_bus_if
// and instantiates VX_local_mem with OUT_BUF=3.
//
// Parameters match VX_local_mem_top.sv.  addrWidth is derived from the size
// and bank geometry (as in the SV).
// ---------------------------------------------------------------------------

class LocalMemTop(
    size:     Int = 1024 * 16 * 8,
    numReqs:  Int = 4,
    numBanks: Int = 4,
    wordSize: Int = 4,   // XLEN/8
    tagWidth: Int = 16,
    flagsWidth: Int = 3,
    uuidWidth:  Int = 0
) extends Module {
  // Derive address width as in SV: BANK_ADDR_WIDTH + CLOG2(NUM_BANKS)
  private val numWords       = size / wordSize
  private val wordsPerBank   = numWords / numBanks
  private val bankAddrWidth  = log2Ceil(wordsPerBank)
  private val bankSelBits    = if (numBanks > 1) log2Ceil(numBanks) else 0
  val addrWidth              = bankAddrWidth + bankSelBits

  val io = IO(new Bundle {
    // Core request
    val memReqValid  = Input(Vec(numReqs, Bool()))
    val memReqRw     = Input(Vec(numReqs, Bool()))
    val memReqByteen = Input(Vec(numReqs, UInt(wordSize.W)))
    val memReqAddr   = Input(Vec(numReqs, UInt(addrWidth.W)))
    val memReqFlags  = Input(Vec(numReqs, UInt(flagsWidth.W)))
    val memReqData   = Input(Vec(numReqs, UInt((wordSize * 8).W)))
    val memReqTag    = Input(Vec(numReqs, UInt(tagWidth.W)))
    val memReqReady  = Output(Vec(numReqs, Bool()))
    // Core response
    val memRspValid  = Output(Vec(numReqs, Bool()))
    val memRspData   = Output(Vec(numReqs, UInt((wordSize * 8).W)))
    val memRspTag    = Output(Vec(numReqs, UInt(tagWidth.W)))
    val memRspReady  = Input(Vec(numReqs, Bool()))
  })

  val lmem = Module(new LocalMem(
    size       = size,
    numReqs    = numReqs,
    numBanks   = numBanks,
    wordSize   = wordSize,
    tagWidth   = tagWidth,
    flagsWidth = flagsWidth,
    uuidWidth  = uuidWidth
  ))

  for (i <- 0 until numReqs) {
    // Request wiring
    lmem.io.memBus(i).req.valid          := io.memReqValid(i)
    lmem.io.memBus(i).req.bits.rw        := io.memReqRw(i)
    lmem.io.memBus(i).req.bits.byteen    := io.memReqByteen(i)
    lmem.io.memBus(i).req.bits.addr      := io.memReqAddr(i)
    lmem.io.memBus(i).req.bits.flags     := io.memReqFlags(i)
    lmem.io.memBus(i).req.bits.data      := io.memReqData(i)
    // Tag: wrap flat UInt into the MemBusTagBundle structure.
    lmem.io.memBus(i).req.bits.tag       := io.memReqTag(i).asTypeOf(lmem.io.memBus(i).req.bits.tag)
    io.memReqReady(i)                    := lmem.io.memBus(i).req.ready

    // Response wiring
    io.memRspValid(i)                    := lmem.io.memBus(i).rsp.valid
    io.memRspData(i)                     := lmem.io.memBus(i).rsp.bits.data
    io.memRspTag(i)                      := lmem.io.memBus(i).rsp.bits.tag.asUInt
    lmem.io.memBus(i).rsp.ready          := io.memRspReady(i)
  }
}
