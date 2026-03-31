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

// Chisel translation of:
//   VX_dcr_bus_if.sv

package vortex

import chisel3._

// ---------------------------------------------------------------------------
// VX_dcr_bus_if
//
// Device Configuration Register (DCR) bus.
// The SV interface carries three unidirectional wires (master → slave):
//   write_valid : Bool
//   write_addr  : UInt(VX_DCR_ADDR_WIDTH.W)  = UInt(12.W)
//   write_data  : UInt(VX_DCR_DATA_WIDTH.W)  = UInt(32.W)
//
// There is no ready or response channel – writes are posted (fire-and-forget).
//
// Translation notes:
//   - VX_DCR_ADDR_WIDTH = VX_DCR_ADDR_BITS = 12
//   - VX_DCR_DATA_WIDTH = 32 (hardcoded in the SV package)
//
// The Bundle is defined from the *master* perspective (all fields are Output
// when used as-is).  Wrap with Flipped() to get the slave perspective.
// ---------------------------------------------------------------------------

/** VX_dcr_bus_if translated as a master-side Bundle.
 *
 *  @param addrWidth  VX_DCR_ADDR_WIDTH (default 12, matches VX_DCR_ADDR_BITS)
 *  @param dataWidth  VX_DCR_DATA_WIDTH (default 32)
 *
 *  Usage:
 *  {{{
 *    // master port
 *    val dcr = Output(new DcrBusBundle())
 *
 *    // slave port
 *    val dcr = Input(new DcrBusBundle())
 *    // or equivalently:
 *    val dcr = Flipped(new DcrBusBundle())
 *  }}}
 */
class DcrBusBundle(
    val addrWidth: Int = 12,
    val dataWidth: Int = 32
) extends Bundle {
  val write_valid = Bool()
  val write_addr  = UInt(addrWidth.W)
  val write_data  = UInt(dataWidth.W)
}
