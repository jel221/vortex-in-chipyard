package chipyard

import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.prci.{AsynchronousCrossing}
import freechips.rocketchip.subsystem.{InCluster, SubsystemBankedCoherenceKey, BankedCoherenceParams}

class VortexRocketConfig extends Config(
  new vortex.WithVortexRoCC ++
  new freechips.rocketchip.rocket.WithCFlushEnabled ++
  new freechips.rocketchip.rocket.WithNSmallCores(1) ++
  // Replace InclusiveCache (L2) with the default broadcast manager so the
  // memory bus exposes supportsPutPartial for the GPU's uncached TL writes.
  new Config((site, here, up) => {
    case SubsystemBankedCoherenceKey => BankedCoherenceParams()
  }) ++
  new chipyard.config.AbstractConfig)