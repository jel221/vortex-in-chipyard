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

// Chisel translation of VX_cache_flush.sv

package vortex

import chisel3._
import chisel3.util._

/**
 * CacheFlush – per-bank flush / init sequencer.
 *
 * Mirrors VX_cache_flush.sv exactly.
 *
 * On reset the FSM starts in STATE_INIT and pulses through every line to
 * invalidate the tag RAM.  After that it idles waiting for flush_begin.
 *
 * Counter layout:
 *   [CS_LINE_SEL_BITS-1 : 0]           → flush_line
 *   [CS_LINE_SEL_BITS + CS_WAY_SEL_BITS - 1 : CS_LINE_SEL_BITS] → flush_way
 *   (way bits only present when WRITEBACK && NUM_WAYS > 1)
 */
class CacheFlush(p: CacheParams, bankId: Int = 0) extends Module {
  private val lineSelBits = p.lineSelBits
  private val waySelBits  = p.waySelBits
  private val waySelWidth = p.waySelWidth
  private val numWays     = p.numWays

  // Counter covers lines + (optionally) ways; ensure at least 1 bit
  private val ctrWidth = math.max(1, lineSelBits + (if (p.writeback != 0) waySelBits else 0))

  val io = IO(new Bundle {
    val flush_begin = Input(Bool())
    val flush_end   = Output(Bool())
    val flush_init  = Output(Bool())
    val flush_valid = Output(Bool())
    val flush_line  = Output(UInt(lineSelBits.W))
    val flush_way   = Output(UInt(waySelWidth.W))
    val flush_ready = Input(Bool())
    val mshr_empty  = Input(Bool())
    val bank_empty  = Input(Bool())
  })

  // ---- FSM states ----------------------------------------------------------
  val STATE_IDLE  = 0.U(3.W)
  val STATE_INIT  = 1.U(3.W)
  val STATE_WAIT1 = 2.U(3.W)
  val STATE_FLUSH = 3.U(3.W)
  val STATE_WAIT2 = 4.U(3.W)
  val STATE_DONE  = 5.U(3.W)

  // Start in STATE_INIT to invalidate the tag RAM on power-up
  val state   = RegInit(STATE_INIT)
  val counter = RegInit(0.U(ctrWidth.W))

  // ---- next-state logic ----------------------------------------------------
  val state_n = WireDefault(state)

  switch (state) {
    is (STATE_IDLE) {
      when (io.flush_begin) {
        state_n := STATE_WAIT1
      }
    }
    is (STATE_INIT) {
      when (counter === ((1 << lineSelBits) - 1).U) {
        state_n := STATE_IDLE
      }
    }
    is (STATE_WAIT1) {
      when (io.mshr_empty) {
        state_n := STATE_FLUSH
      }
    }
    is (STATE_FLUSH) {
      when (counter === ((1 << ctrWidth) - 1).U && io.flush_ready) {
        state_n := (if (bankId == 0) STATE_DONE else STATE_WAIT2)
      }
    }
    is (STATE_WAIT2) {
      when (io.bank_empty) {
        state_n := STATE_DONE
      }
    }
    is (STATE_DONE) {
      state_n := STATE_IDLE
    }
  }

  // ---- counter update ------------------------------------------------------
  when (reset.asBool) {
    state   := STATE_INIT
    counter := 0.U
  } .otherwise {
    state := state_n
    when (state =/= STATE_IDLE) {
      when ((state === STATE_INIT) ||
            ((state === STATE_FLUSH) && io.flush_ready)) {
        counter := counter + 1.U
      }
    } .otherwise {
      counter := 0.U
    }
  }

  // ---- outputs -------------------------------------------------------------
  io.flush_end   := (state === STATE_DONE)
  io.flush_init  := (state === STATE_INIT)
  io.flush_valid := (state === STATE_FLUSH)
  io.flush_line  := (if (lineSelBits > 0) counter(lineSelBits - 1, 0) else 0.U)

  if (p.writeback != 0 && numWays > 1) {
    io.flush_way := counter(lineSelBits + waySelBits - 1, lineSelBits)
  } else {
    io.flush_way := 0.U
  }
}
