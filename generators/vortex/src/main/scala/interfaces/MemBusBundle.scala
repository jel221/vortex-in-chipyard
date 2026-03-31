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
//   VX_mem_bus_if.sv
//   VX_lsu_mem_if.sv
//   VX_gbar_bus_if.sv

package vortex

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// VX_mem_bus_if
//
// SV parameters:
//   DATA_SIZE    – word size in bytes  (default 1)
//   FLAGS_WIDTH  – flag bits           (default MEM_FLAGS_WIDTH, typically 3)
//   TAG_WIDTH    – total tag bits      (default 1)
//   ADDR_WIDTH   – derived from MEM_ADDR_WIDTH - log2(DATA_SIZE)
//
// The SV interface splits the tag into a UUID sub-field and a value sub-field.
// In Chisel we keep that split visible via MemBusTagBundle, but callers that
// do not need the split can use the flat UInt view of the whole tag.
// ---------------------------------------------------------------------------

/** Tag bundle: uuid (debug) + opaque value field.
 *
 *  @param tagWidth   total tag width in bits
 *  @param uuidWidth  width of the uuid sub-field (set to 0 to suppress uuid)
 */
class MemBusTagBundle(val tagWidth: Int, val uuidWidth: Int) extends Bundle {
  require(tagWidth >= uuidWidth, "tagWidth must be >= uuidWidth")
  val uuid  = UInt(uuidWidth.W)
  // value occupies the remaining bits; width is at least 0
  val value = UInt(math.max(tagWidth - uuidWidth, 0).W)
}

/** Request payload for VX_mem_bus_if. */
class MemBusReqBundle(
    val dataSize:   Int,  // bytes
    val addrWidth:  Int,
    val flagsWidth: Int,
    val tagWidth:   Int,
    val uuidWidth:  Int
) extends Bundle {
  val rw     = Bool()                          // 1 = write, 0 = read
  val addr   = UInt(addrWidth.W)
  val data   = UInt((dataSize * 8).W)
  val byteen = UInt(dataSize.W)
  val flags  = UInt(flagsWidth.W)
  val tag    = new MemBusTagBundle(tagWidth, uuidWidth)
}

/** Response payload for VX_mem_bus_if. */
class MemBusRspBundle(
    val dataSize:  Int,
    val tagWidth:  Int,
    val uuidWidth: Int
) extends Bundle {
  val data = UInt((dataSize * 8).W)
  val tag  = new MemBusTagBundle(tagWidth, uuidWidth)
}

/** Full VX_mem_bus_if translated as a master-side Bundle.
 *
 *  From the master modport:
 *    - req channel : master drives valid/data, slave drives ready  → Decoupled
 *    - rsp channel : slave drives valid/data, master drives ready  → Flipped(Decoupled)
 *
 *  To instantiate a slave port use `Flipped(new MemBusBundle(...))`.
 *
 *  @param dataSize    word size in bytes
 *  @param addrWidth   address width (MEM_ADDR_WIDTH - log2(dataSize))
 *  @param flagsWidth  memory request flag bits (MEM_FLAGS_WIDTH)
 *  @param tagWidth    total tag width
 *  @param uuidWidth   UUID sub-field width within the tag (0 when NDEBUG)
 */
class MemBusBundle(
    dataSize:   Int,
    addrWidth:  Int,
    flagsWidth: Int = 3,
    tagWidth:   Int = 1,
    uuidWidth:  Int = 0
) extends Bundle {
  val req = Decoupled(new MemBusReqBundle(dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
  val rsp = Flipped(Decoupled(new MemBusRspBundle(dataSize, tagWidth, uuidWidth)))
}

// ---------------------------------------------------------------------------
// VX_lsu_mem_if
//
// Like VX_mem_bus_if but every data field is replicated NUM_LANES times, and
// there is an additional per-lane 'mask' field on both req and rsp.
// ---------------------------------------------------------------------------

/** Request payload for VX_lsu_mem_if (per-lane vectors). */
class LsuMemReqBundle(
    val numLanes:   Int,
    val dataSize:   Int,
    val addrWidth:  Int,
    val flagsWidth: Int,
    val tagWidth:   Int,
    val uuidWidth:  Int
) extends Bundle {
  val mask   = UInt(numLanes.W)
  val rw     = Bool()
  val addr   = Vec(numLanes, UInt(addrWidth.W))
  val data   = Vec(numLanes, UInt((dataSize * 8).W))
  val byteen = Vec(numLanes, UInt(dataSize.W))
  val flags  = Vec(numLanes, UInt(flagsWidth.W))
  val tag    = new MemBusTagBundle(tagWidth, uuidWidth)
}

/** Response payload for VX_lsu_mem_if. */
class LsuMemRspBundle(
    val numLanes:  Int,
    val dataSize:  Int,
    val tagWidth:  Int,
    val uuidWidth: Int
) extends Bundle {
  val mask = UInt(numLanes.W)
  val data = Vec(numLanes, UInt((dataSize * 8).W))
  val tag  = new MemBusTagBundle(tagWidth, uuidWidth)
}

/** Full VX_lsu_mem_if translated as a master-side Bundle.
 *
 *  To instantiate a slave port use `Flipped(new LsuMemBusBundle(...))`.
 */
class LsuMemBusBundle(
    numLanes:   Int,
    dataSize:   Int,
    addrWidth:  Int,
    flagsWidth: Int = 3,
    tagWidth:   Int = 1,
    uuidWidth:  Int = 0
) extends Bundle {
  val req = Decoupled(new LsuMemReqBundle(numLanes, dataSize, addrWidth, flagsWidth, tagWidth, uuidWidth))
  val rsp = Flipped(Decoupled(new LsuMemRspBundle(numLanes, dataSize, tagWidth, uuidWidth)))
}

// ---------------------------------------------------------------------------
// VX_gbar_bus_if
//
// Global barrier bus.  Parameters NB_WIDTH and NC_WIDTH come from the GPU
// config (log2 of NUM_BARRIERS and NUM_CORES respectively).  The response
// channel has no ready signal in the SV original (it is a simple valid/data
// push), so we model it as a ValidIO rather than DecoupledIO.
// ---------------------------------------------------------------------------

/** Request payload for VX_gbar_bus_if.
 *
 *  @param nbWidth  NB_WIDTH = UP(CLOG2(NUM_BARRIERS))
 *  @param ncWidth  NC_WIDTH = UP(CLOG2(NUM_CORES))
 */
class GbarReqBundle(val nbWidth: Int, val ncWidth: Int) extends Bundle {
  val id      = UInt(nbWidth.W)  // barrier id
  val size_m1 = UInt(ncWidth.W)  // number of participating cores minus 1
  val core_id = UInt(ncWidth.W)  // this core's id
}

/** Response payload for VX_gbar_bus_if. */
class GbarRspBundle(val nbWidth: Int) extends Bundle {
  val id = UInt(nbWidth.W)
}

/** Full VX_gbar_bus_if translated as a master-side Bundle.
 *
 *  The SV slave modport drives rsp_valid/rsp_data without a corresponding
 *  rsp_ready (no back-pressure on responses), so the rsp channel is a
 *  ValidIO (Chisel `Valid`).
 *
 *  To instantiate a slave port use `Flipped(new GbarBusBundle(...))`.
 *
 *  @param nbWidth  NB_WIDTH = UP(CLOG2(NUM_BARRIERS))
 *  @param ncWidth  NC_WIDTH = UP(CLOG2(NUM_CORES))
 */
class GbarBusBundle(nbWidth: Int, ncWidth: Int) extends Bundle {
  val req = Decoupled(new GbarReqBundle(nbWidth, ncWidth))
  // rsp has no ready – slave pushes with valid only
  val rsp = Flipped(Valid(new GbarRspBundle(nbWidth)))
}
