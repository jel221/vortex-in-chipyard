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

// Chisel translations of Vortex SV buffer primitives:
//   hw/rtl/libs/VX_pipe_register.sv
//   hw/rtl/libs/VX_pipe_buffer.sv
//   hw/rtl/libs/VX_bypass_buffer.sv
//   hw/rtl/libs/VX_toggle_buffer.sv
//   hw/rtl/libs/VX_skid_buffer.sv
//   hw/rtl/libs/VX_elastic_buffer.sv
//   hw/rtl/libs/VX_stream_buffer.sv
//   hw/rtl/libs/VX_elastic_adapter.sv

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// VX_pipe_register
//
// A shift-register pipeline with configurable depth.  The `resetw` parameter
// controls how many MSBs are subject to synchronous reset:
//   resetw == 0     -> no reset (data-path only)
//   resetw == dataw -> full reset to initValue
//   0 < resetw < dataw -> partial reset: upper resetw bits reset to
//                         initValue(resetw-1,0); lower bits are un-reset
//
// When depth == 0 the module is a pure pass-through.
// The enable input gates all register updates.
// ---------------------------------------------------------------------------
class VXPipeRegister(
  val dataw:     Int,
  val resetw:    Int  = 0,
  val depth:     Int  = 1,
  val initValue: UInt = 0.U
) extends Module {
  require(dataw  >= 1, "dataw must be >= 1")
  require(resetw >= 0 && resetw <= dataw, "resetw must be in [0, dataw]")
  require(depth  >= 0, "depth must be >= 0")

  val io = IO(new Bundle {
    val enable   = Input(Bool())
    val data_in  = Input(UInt(dataw.W))
    val data_out = Output(UInt(dataw.W))
  })

  if (depth == 0) {
    io.data_out := io.data_in
  } else {
    // Build the pipeline as individual stage registers.
    // For each stage we may need to split the storage into a reset portion
    // (upper resetw bits) and a non-reset portion (lower dataw-resetw bits)
    // because Chisel does not natively support partial-reset registers.

    if (resetw == 0) {
      // No reset on any bit: simple data-path shift register.
      val pipe = Seq.fill(depth)(Reg(UInt(dataw.W)))
      for (i <- 0 until depth) {
        val stageIn = if (i == 0) io.data_in else pipe(i - 1)
        when(io.enable) { pipe(i) := stageIn }
      }
      io.data_out := pipe(depth - 1)

    } else if (resetw == dataw) {
      // Full reset: every bit resets to initValue.
      val resetVal = initValue(dataw - 1, 0)
      val pipe = Seq.fill(depth)(RegInit(resetVal))
      for (i <- 0 until depth) {
        val stageIn = if (i == 0) io.data_in else pipe(i - 1)
        when(io.enable) { pipe(i) := stageIn }
      }
      io.data_out := pipe(depth - 1)

    } else {
      // Partial reset: upper `resetw` bits are reset; lower bits are not.
      // We model each stage as a pair of registers.
      val hiWidth = resetw
      val loWidth = dataw - resetw
      val hiResetVal = initValue(resetw - 1, 0)

      val hiPipe = Seq.fill(depth)(RegInit(hiResetVal))
      val loPipe = Seq.fill(depth)(Reg(UInt(loWidth.W)))

      for (i <- 0 until depth) {
        val hiIn = if (i == 0) io.data_in(dataw - 1, loWidth)
                   else hiPipe(i - 1)
        val loIn = if (i == 0) io.data_in(loWidth - 1, 0)
                   else loPipe(i - 1)
        when(io.enable) {
          hiPipe(i) := hiIn
          loPipe(i) := loIn
        }
      }
      io.data_out := Cat(hiPipe(depth - 1), loPipe(depth - 1))
    }
  }
}

// ---------------------------------------------------------------------------
// VX_pipe_buffer
//
// Pipelined elastic buffer: full bandwidth, registered output, single register
// per stage.  `ready_in` and `ready_out` are coupled through the chain.
//
// Depth == 0: pure pass-through (no registers, no backpressure).
// ---------------------------------------------------------------------------
class VXPipeBuffer(
  val dataw:  Int,
  val resetw: Int = 0,
  val depth:  Int = 1
) extends Module {
  require(dataw >= 1)
  require(depth >= 0)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val data_in   = Input(UInt(dataw.W))
    val data_out  = Output(UInt(dataw.W))
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
  })

  if (depth == 0) {
    io.ready_in  := io.ready_out
    io.valid_out := io.valid_in
    io.data_out  := io.data_in
  } else {
    // Each stage holds (valid, data).
    // ready(i) = ready(i+1) || !valid(i+1)
    // On every cycle, if ready(i) is high the stage latches the upstream value.
    val validRegs = Seq.fill(depth)(RegInit(false.B))
    val dataRegs  = Seq.fill(depth)(Reg(UInt(dataw.W)))

    // Compute ready signals from downstream to upstream.
    // ready(depth) = ready_out; ready(i) = ready(i+1) || !validRegs(i)
    val readySig = Wire(Vec(depth + 1, Bool()))
    readySig(depth) := io.ready_out
    for (i <- (0 until depth).reverse) {
      readySig(i) := readySig(i + 1) || !validRegs(i)
    }

    // Stage inputs
    val validSrc = Wire(Vec(depth, Bool()))
    val dataSrc  = Wire(Vec(depth, UInt(dataw.W)))
    validSrc(0) := io.valid_in
    dataSrc(0)  := io.data_in
    for (i <- 1 until depth) {
      validSrc(i) := validRegs(i - 1)
      dataSrc(i)  := dataRegs(i - 1)
    }

    for (i <- 0 until depth) {
      when(readySig(i)) {
        validRegs(i) := validSrc(i)
        dataRegs(i)  := dataSrc(i)
      }
    }

    io.ready_in  := readySig(0)
    io.valid_out := validRegs(depth - 1)
    io.data_out  := dataRegs(depth - 1)
  }
}

// ---------------------------------------------------------------------------
// VX_bypass_buffer
//
// Bypass elastic buffer: full bandwidth, combinational output, one register.
//
// When has_data is set, data_out is the registered value; otherwise data_out
// is driven directly from data_in (bypass).  ready_in and ready_out are
// coupled (ready_in = ready_out || !has_data).
//
// passthru == true: pure combinational pass-through.
// ---------------------------------------------------------------------------
class VXBypassBuffer(
  val dataw:    Int,
  val passthru: Boolean = false
) extends Module {
  require(dataw >= 1)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val data_in   = Input(UInt(dataw.W))
    val data_out  = Output(UInt(dataw.W))
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
  })

  if (passthru) {
    io.ready_in  := io.ready_out
    io.valid_out := io.valid_in
    io.data_out  := io.data_in
  } else {
    val buffer   = Reg(UInt(dataw.W))
    val has_data = RegInit(false.B)

    // has_data update
    when(io.ready_out) {
      has_data := false.B
    }.elsewhen(!has_data) {
      has_data := io.valid_in
    }

    // buffer latches data_in whenever the slot is free
    when(!has_data) {
      buffer := io.data_in
    }

    io.ready_in  := io.ready_out || !has_data
    io.data_out  := Mux(has_data, buffer, io.data_in)
    io.valid_out := io.valid_in || has_data
  }
}

// ---------------------------------------------------------------------------
// VX_toggle_buffer
//
// Toggle elastic buffer: half bandwidth, registered output, one register.
// ready_in and ready_out are decoupled (ready_in = !has_data).
// A new item can only enter after the current one has been consumed.
//
// passthru == true: pure combinational pass-through.
// ---------------------------------------------------------------------------
class VXToggleBuffer(
  val dataw:    Int,
  val passthru: Boolean = false
) extends Module {
  require(dataw >= 1)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val data_in   = Input(UInt(dataw.W))
    val data_out  = Output(UInt(dataw.W))
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
  })

  if (passthru) {
    io.ready_in  := io.ready_out
    io.valid_out := io.valid_in
    io.data_out  := io.data_in
  } else {
    val buffer   = Reg(UInt(dataw.W))
    val has_data = RegInit(false.B)

    when(!has_data) {
      has_data := io.valid_in
    }.elsewhen(io.ready_out) {
      has_data := false.B
    }

    when(!has_data) {
      buffer := io.data_in
    }

    io.ready_in  := !has_data
    io.valid_out := has_data
    io.data_out  := buffer
  }
}

// ---------------------------------------------------------------------------
// VX_stream_buffer
//
// Stream elastic buffer: full bandwidth, two-register storage, decoupled
// ready_in / ready_out.  When outReg is true data_out is fully registered
// (one extra cycle of latency); otherwise data_out is combinational from the
// primary slot.
//
// passthru == true: pure combinational pass-through.
//
// Internal state:
//   valid_in_r  – '1' means the primary slot is free to accept new data
//                 (matches SV ready_in = valid_in_r).  Reset to 1.
//   valid_out_r – output valid register.  Reset to 0.
//   data_out_r  – primary data register.
//   buffer_r    – secondary (backup) data register.
// ---------------------------------------------------------------------------
class VXStreamBuffer(
  val dataw:    Int,
  val outReg:   Boolean = false,
  val passthru: Boolean = false
) extends Module {
  require(dataw >= 1)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val data_in   = Input(UInt(dataw.W))
    val data_out  = Output(UInt(dataw.W))
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
  })

  if (passthru) {
    io.ready_in  := io.ready_out
    io.valid_out := io.valid_in
    io.data_out  := io.data_in
  } else {
    val data_out_r  = Reg(UInt(dataw.W))
    val buffer_r    = Reg(UInt(dataw.W))
    val valid_out_r = RegInit(false.B)
    // valid_in_r: 1 = primary slot is free (ready to accept).  Reset to 1.
    val valid_in_r  = RegInit(true.B)

    val fire_in  = io.valid_in && io.ready_in   // item accepted from upstream
    val flow_out = io.ready_out || !valid_out_r  // downstream can accept

    // Update valid_in_r: becomes flow_out whenever valid_in or flow_out is true
    when(io.valid_in || flow_out) {
      valid_in_r := flow_out
    }

    // Update valid_out_r
    when(flow_out) {
      valid_out_r := io.valid_in || !valid_in_r
    }

    if (outReg) {
      // Registered output path:
      //   buffer_r   = capture incoming data on fire_in
      //   data_out_r = mux between data_in (if valid_in_r) or buffer_r,
      //                loaded on flow_out
      when(fire_in) {
        buffer_r := io.data_in
      }
      when(flow_out) {
        data_out_r := Mux(valid_in_r, io.data_in, buffer_r)
      }
      io.data_out := data_out_r
    } else {
      // Combinational output path:
      //   data_out_r = newest incoming word (updated on fire_in)
      //   buffer_r   = previous data_out_r (shifted on fire_in)
      //   data_out   = mux: valid_in_r ? data_out_r : buffer_r
      when(fire_in) {
        data_out_r := io.data_in
        buffer_r   := data_out_r
      }
      io.data_out := Mux(valid_in_r, data_out_r, buffer_r)
    }

    io.valid_out := valid_out_r
    io.ready_in  := valid_in_r
  }
}

// ---------------------------------------------------------------------------
// VX_skid_buffer
//
// Skid buffer that dispatches to:
//   passthru == true -> pure combinational pass-through
//   halfBw   == true -> VXToggleBuffer  (half bandwidth, decoupled ready)
//   otherwise        -> VXStreamBuffer  (full bandwidth, decoupled ready)
//
// Parameters
//   dataw    – data width
//   passthru – combinational pass-through
//   halfBw   – select half-bandwidth toggle buffer
//   outReg   – forwarded to VXStreamBuffer (registered output)
// ---------------------------------------------------------------------------
class VXSkidBuffer(
  val dataw:    Int,
  val passthru: Boolean = false,
  val halfBw:   Boolean = false,
  val outReg:   Boolean = false
) extends Module {
  require(dataw >= 1)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val data_in   = Input(UInt(dataw.W))
    val data_out  = Output(UInt(dataw.W))
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
  })

  if (passthru) {
    io.valid_out := io.valid_in
    io.data_out  := io.data_in
    io.ready_in  := io.ready_out
  } else if (halfBw) {
    val buf = Module(new VXToggleBuffer(dataw))
    buf.io.valid_in  := io.valid_in
    buf.io.data_in   := io.data_in
    io.ready_in      := buf.io.ready_in
    io.valid_out     := buf.io.valid_out
    io.data_out      := buf.io.data_out
    buf.io.ready_out := io.ready_out
  } else {
    val buf = Module(new VXStreamBuffer(dataw, outReg = outReg))
    buf.io.valid_in  := io.valid_in
    buf.io.data_in   := io.data_in
    io.ready_in      := buf.io.ready_in
    io.valid_out     := buf.io.valid_out
    io.data_out      := buf.io.data_out
    buf.io.ready_out := io.ready_out
  }
}

// ---------------------------------------------------------------------------
// VX_elastic_buffer
//
// Elastic buffer that selects an implementation based on size:
//   size == 0                    -> pass-through
//   size == 1                    -> VXPipeBuffer (depth = max(outReg, 1))
//   size == 2 && !lutRam         -> VXStreamBuffer + optional VXPipeBuffer tail
//   size >= 3 || lutRam == true  -> VxFifoQueue  + optional VXPipeBuffer tail
//
// The outReg parameter selects output pipeline registers:
//   outReg == 0 -> no output register
//   outReg == 1 -> one output register absorbed into the stream/fifo stage
//   outReg >  1 -> (outReg-1) additional VXPipeBuffer stages appended
//
// Parameters
//   dataw  – data width
//   size   – number of entries  (0 = pass-through)
//   outReg – output pipeline depth
//   lutRam – request LUT-RAM in the underlying fifo (forces fifo path)
// ---------------------------------------------------------------------------
class VXElasticBuffer(
  val dataw:  Int,
  val size:   Int     = 1,
  val outReg: Int     = 0,
  val lutRam: Boolean = false
) extends Module {
  require(dataw  >= 1)
  require(size   >= 0)
  require(outReg >= 0)

  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val data_in   = Input(UInt(dataw.W))
    val data_out  = Output(UInt(dataw.W))
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
  })

  if (size == 0) {
    // Pass-through
    io.valid_out := io.valid_in
    io.data_out  := io.data_in
    io.ready_in  := io.ready_out

  } else if (size == 1) {
    val pipeDepth = math.max(outReg, 1)
    val buf = Module(new VXPipeBuffer(dataw, depth = pipeDepth))
    buf.io.valid_in  := io.valid_in
    buf.io.data_in   := io.data_in
    io.ready_in      := buf.io.ready_in
    io.valid_out     := buf.io.valid_out
    io.data_out      := buf.io.data_out
    buf.io.ready_out := io.ready_out

  } else if (size == 2 && !lutRam) {
    // Stream buffer absorbs outReg == 1; further stages go into a pipe buffer.
    val streamOutReg = (outReg == 1)
    val stream = Module(new VXStreamBuffer(dataw, outReg = streamOutReg))
    stream.io.valid_in := io.valid_in
    stream.io.data_in  := io.data_in
    io.ready_in        := stream.io.ready_in

    val tailDepth = if (outReg > 1) outReg - 1 else 0
    if (tailDepth == 0) {
      io.valid_out        := stream.io.valid_out
      io.data_out         := stream.io.data_out
      stream.io.ready_out := io.ready_out
    } else {
      val tail = Module(new VXPipeBuffer(dataw, depth = tailDepth))
      tail.io.valid_in    := stream.io.valid_out
      tail.io.data_in     := stream.io.data_out
      stream.io.ready_out := tail.io.ready_in
      io.valid_out        := tail.io.valid_out
      io.data_out         := tail.io.data_out
      tail.io.ready_out   := io.ready_out
    }

  } else {
    // FIFO path (size >= 3 or lutRam requested).
    // VxFifoQueue requires depth to be a power of 2; round up.
    val fifoDepth = {
      var d = 1
      while (d < size) d <<= 1
      d
    }

    val fifo = Module(new VxFifoQueue(
      dataw  = dataw,
      depth  = fifoDepth,
      outReg = if (outReg == 1) 1 else 0
    ))

    fifo.io.push   := io.valid_in && io.ready_in
    fifo.io.dataIn := io.data_in
    io.ready_in    := !fifo.io.full

    val validOutT = !fifo.io.empty

    val tailDepth = if (outReg > 1) outReg - 1 else 0
    if (tailDepth == 0) {
      fifo.io.pop  := validOutT && io.ready_out
      io.valid_out := validOutT
      io.data_out  := fifo.io.dataOut
    } else {
      val tail = Module(new VXPipeBuffer(dataw, depth = tailDepth))
      fifo.io.pop       := validOutT && tail.io.ready_in
      tail.io.valid_in  := validOutT
      tail.io.data_in   := fifo.io.dataOut
      io.valid_out      := tail.io.valid_out
      io.data_out       := tail.io.data_out
      tail.io.ready_out := io.ready_out
    }
  }
}

// ---------------------------------------------------------------------------
// VX_elastic_adapter
//
// Decouples a valid/ready handshake from a potentially multi-cycle busy
// operation.  Accepts one item at a time from the upstream interface (fires
// strobe), holds the "loaded" state until the downstream consumer is ready
// and not busy, then presents valid_out.
//
// Ports
//   valid_in  / ready_in  – upstream handshake
//   valid_out / ready_out – downstream handshake
//   busy   – asserted by the consumer while an operation is in flight
//   strobe – one-cycle pulse when a new item is accepted (push event)
// ---------------------------------------------------------------------------
class VXElasticAdapter extends Module {
  val io = IO(new Bundle {
    val valid_in  = Input(Bool())
    val ready_in  = Output(Bool())
    val ready_out = Input(Bool())
    val valid_out = Output(Bool())
    val busy      = Input(Bool())
    val strobe    = Output(Bool())
  })

  val loaded = RegInit(false.B)

  val push = io.valid_in && io.ready_in   // accepting a new item
  val pop  = io.valid_out && io.ready_out // downstream consumed it

  when(push) {
    loaded := true.B
  }
  when(pop) {
    loaded := false.B
  }

  io.ready_in  := !loaded
  io.valid_out := loaded && !io.busy
  io.strobe    := push
}
