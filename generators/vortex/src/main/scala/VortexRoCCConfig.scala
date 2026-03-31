package vortex

import org.chipsalliance.cde.config._
import org.chipsalliance.diplomacy.lazymodule._
import freechips.rocketchip.tile._

// Attach the Vortex GPGPU as a RoCC accelerator on custom-0 (opcode 0x0B).
// Optionally override VortexParamsKey to tune GPU topology and AXI widths.
//
// Example:
//   class MyConfig extends Config(
//     new vortex.WithVortexRoCC ++
//     new freechips.rocketchip.rocket.WithNSmallCores(1) ++
//     new chipyard.config.AbstractConfig)
class WithVortexRoCC extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC, site) ++ Seq(
    (p: Parameters) => LazyModule(new VortexRoCC(OpcodeSet.custom0)(p))
  )
})

// Example: 4-core Vortex with 4 threads/warps per core
class WithVortexRoCCLarge extends Config((site, here, up) => {
  case VortexParamsKey => VortexParams(
    numThreads  = 4,
    numWarps    = 4,
    numCores    = 4,
    numClusters = 1
  )
  case BuildRoCC => up(BuildRoCC, site) ++ Seq(
    (p: Parameters) => LazyModule(new VortexRoCC(OpcodeSet.custom0)(p))
  )
})
