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

// Chisel translation of VX_cache_define.vh

package vortex

import chisel3._
import chisel3.util._

/** Replacement policy selectors (CS_REPL_*) */
object CacheReplPolicy {
  val RANDOM = 0
  val FIFO   = 1
  val PLRU   = 2
}

/**
 * CacheParams captures all the structural parameters that drive the
 * address-field arithmetic defined in VX_cache_define.vh.
 *
 * All bit-width fields follow the convention:
 *   xxxBits  = log2 count (may be 0)
 *   xxxWidth = max(1, xxxBits)   ("UP" in SV)
 */
case class CacheParams(
  cacheSize:   Int = 1024,   // bytes
  lineSize:    Int = 16,     // bytes
  numBanks:    Int = 1,
  numWays:     Int = 1,
  wordSize:    Int = 4,      // bytes
  memAddrWidth: Int = 32,    // MEM_ADDR_WIDTH (global constant)
  writeEnable: Int = 1,
  writeback:   Int = 0,
  dirtyBytes:  Int = 0,
  replPolicy:  Int = CacheReplPolicy.FIFO
) {
  // ---- widths (bits) -------------------------------------------------------
  val wordWidth: Int = wordSize * 8
  val lineWidth: Int = lineSize * 8

  val bankSize: Int = cacheSize / numBanks

  def up(n: Int): Int = math.max(1, n)

  // words per line
  val wordsPerLine: Int = lineSize / wordSize

  // lines per bank (per way)
  val linesPerBank: Int = bankSize / (lineSize * numWays)

  // address field widths
  val wordAddrWidth: Int  = memAddrWidth - log2Ceil(wordSize)
  val memAddrWidthCS: Int = memAddrWidth - log2Ceil(lineSize)
  val lineAddrWidth: Int  = memAddrWidthCS - log2Ceil(numBanks)

  // word-select field
  val wordSelBits:      Int = log2Ceil(wordsPerLine)
  val wordSelAddrStart: Int = 0
  val wordSelAddrEnd:   Int = wordSelAddrStart + wordSelBits - 1

  // bank-select field
  val bankSelBits:      Int = log2Ceil(numBanks)
  val bankSelAddrStart: Int = 1 + wordSelAddrEnd
  val bankSelAddrEnd:   Int = bankSelAddrStart + bankSelBits - 1

  // line-select field
  val lineSelBits:      Int = log2Ceil(linesPerBank)
  val lineSelAddrStart: Int = 1 + bankSelAddrEnd
  val lineSelAddrEnd:   Int = lineSelAddrStart + lineSelBits - 1

  // tag-select field
  val tagSelBits:      Int = wordAddrWidth - 1 - lineSelAddrEnd
  val tagSelAddrStart: Int = 1 + lineSelAddrEnd
  val tagSelAddrEnd:   Int = wordAddrWidth - 1

  // derived "UP" widths
  val reqSelBits:    Int = log2Ceil(1) // placeholder; filled by NUM_REQS at bank level
  val waySelBits:    Int = log2Ceil(numWays)
  val waySelWidth:   Int = up(waySelBits)
  val wordSelWidth:  Int = up(wordSelBits)
  val lineSelWidth:  Int = up(lineSelBits)  // == lineSelBits when > 0

  // Tag width inside the tag RAM: valid [+ dirty] + tag
  val tagRamWidth: Int = 1 + writeback + tagSelBits
}
