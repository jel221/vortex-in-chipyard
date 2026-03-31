package vortex

import org.chipsalliance.cde.config._

// Configuration parameters for the Vortex GPGPU RoCC accelerator.
// GPU topology params (numThreads etc.) are used to answer DEV_CAPS queries.
// AXI params control the BlackBox interface and TileLink bridge sizing.
case class VortexParams(
  numThreads:   Int = 4,   // NUM_THREADS (must match VX_config.vh)
  numWarps:     Int = 4,   // NUM_WARPS
  numCores:     Int = 1,   // NUM_CORES
  numClusters:  Int = 1,   // NUM_CLUSTERS
  // AXI interface sizing for VortexAXIWrapper.  axiDataWidth=64 matches
  // Chipyard's 64-bit system bus; Vortex_axi uses VX_mem_data_adapter
  // internally to convert 512-bit cache lines to this width.
  axiDataWidth: Int = 64,  // AXI data width in bits (must be power-of-2, ≤512)
  axiAddrWidth: Int = 32,  // AXI address width in bits (MEM_ADDR_WIDTH)
  axiIdWidth:   Int = 8    // AXI ID width in bits; 2^N max outstanding txns
)

case object VortexParamsKey extends Field[VortexParams](VortexParams())
