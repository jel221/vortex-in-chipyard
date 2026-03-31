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

// Single-port RAM
// RDW_MODE: "W" = write-first, "R" = read-first, "N" = no-change
class VxSpRam(
  val dataw    : Int    = 1,
  val size     : Int    = 1,
  val wrenw    : Int    = 1,
  val outReg   : Int    = 0,
  val rdwMode  : String = "W"  // "W", "R", or "N"
) extends Module {
  val addrw = log2Ceil(size)
  val wselw = dataw / wrenw

  val io = IO(new Bundle {
    val read  = Input(Bool())
    val write = Input(Bool())
    val wren  = Input(UInt(wrenw.W))
    val addr  = Input(UInt(addrw.W))
    val wdata = Input(UInt(dataw.W))
    val rdata = Output(UInt(dataw.W))
  })

  val mem = SyncReadMem(size, UInt(dataw.W))

  // Write logic with byte-enable
  when(io.write) {
    if (wrenw == 1) {
      mem.write(io.addr, io.wdata)
    } else {
      // Masked write: read-modify-write
      val old   = mem.read(io.addr, true.B)
      val wdata_n = Wire(UInt(dataw.W))
      val merged = Wire(UInt(dataw.W))
      wdata_n := old
      val bits = Wire(Vec(dataw, Bool()))
      for (b <- 0 until dataw) bits(b) := old(b)
      for (i <- 0 until wrenw) {
        when(io.wren(i)) {
          for (b <- 0 until wselw) {
            bits(i * wselw + b) := io.wdata(i * wselw + b)
          }
        }
      }
      merged := bits.asUInt
      mem.write(io.addr, merged)
    }
  }

  if (outReg != 0) {
    // Registered output
    rdwMode match {
      case "W" =>
        // write-first: register the address, output from mem
        val addr_r = RegEnable(io.addr, io.read)
        io.rdata := mem.read(addr_r, true.B)
      case "R" =>
        // read-first: read at current address then register
        val rdata_r = RegEnable(mem.read(io.addr, true.B), io.read)
        io.rdata := rdata_r
      case "N" | _ =>
        // no-change: only update read register when not writing
        val rdata_r = Reg(UInt(dataw.W))
        when(io.read && !io.write) {
          rdata_r := mem.read(io.addr, true.B)
        }
        io.rdata := rdata_r
    }
  } else {
    // Asynchronous read (combinational)
    rdwMode match {
      case "W" =>
        // write-first: just read; SyncReadMem in read-during-write gives new data
        io.rdata := mem.read(io.addr, true.B)
      case _ =>
        // read-first / no-change: combinational read
        io.rdata := mem.read(io.addr, true.B)
    }
  }
}

// Dual-port RAM (separate read and write addresses)
// RDW_MODE: "W" = write-first, "R" = read-first
class VxDpRam(
  val dataw   : Int    = 1,
  val size    : Int    = 1,
  val wrenw   : Int    = 1,
  val outReg  : Int    = 0,
  val rdwMode : String = "W"  // "W" or "R"
) extends Module {
  val addrw = log2Ceil(size)
  val wselw = dataw / wrenw

  val io = IO(new Bundle {
    val read  = Input(Bool())
    val write = Input(Bool())
    val wren  = Input(UInt(wrenw.W))
    val waddr = Input(UInt(addrw.W))
    val wdata = Input(UInt(dataw.W))
    val raddr = Input(UInt(addrw.W))
    val rdata = Output(UInt(dataw.W))
  })

  val mem = SyncReadMem(size, UInt(dataw.W))

  // Write logic
  when(io.write) {
    if (wrenw == 1) {
      mem.write(io.waddr, io.wdata)
    } else {
      val old   = mem.read(io.waddr, true.B)
      val bits  = Wire(Vec(dataw, Bool()))
      for (b <- 0 until dataw) bits(b) := old(b)
      for (i <- 0 until wrenw) {
        when(io.wren(i)) {
          for (b <- 0 until wselw) {
            bits(i * wselw + b) := io.wdata(i * wselw + b)
          }
        }
      }
      mem.write(io.waddr, bits.asUInt)
    }
  }

  if (outReg != 0) {
    rdwMode match {
      case "W" =>
        // write-first with output register: register the read address
        val raddr_r = RegEnable(io.raddr, io.read)
        io.rdata := mem.read(raddr_r, true.B)
      case "R" | _ =>
        // read-first: register the read data
        val rdata_r = RegEnable(mem.read(io.raddr, true.B), io.read)
        io.rdata := rdata_r
    }
  } else {
    // Combinational (async) read
    io.rdata := mem.read(io.raddr, true.B)
  }
}
