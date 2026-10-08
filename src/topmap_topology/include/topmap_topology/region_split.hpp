#pragma once

#include <vector>

#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/region_types.hpp"

namespace topmap {

// T14: split grown regions at navigation bottlenecks (R3), concave
// reflex-vertex chords (R5) and ramp/flat boundaries (R2), in that order.
// Regions a bottleneck already separated are not re-cut by the concave pass.
// Every output region stays edge-connected (R20).
class RegionSplitter {
 public:
  Result<std::vector<SurfaceRegion>> split(const TriangleMesh& mesh,
                                           const std::vector<SurfaceRegion>& regions,
                                           const RegionParams& params) const;
  Result<std::vector<SurfaceRegion>> splitBottlenecks(const TriangleMesh& mesh,
                                                      const std::vector<SurfaceRegion>& regions,
                                                      const RegionParams& params) const;
  Result<std::vector<SurfaceRegion>> splitConcave(const TriangleMesh& mesh,
                                                  const std::vector<SurfaceRegion>& regions,
                                                  const RegionParams& params) const;
  Result<std::vector<SurfaceRegion>> splitRamps(const TriangleMesh& mesh,
                                                const std::vector<SurfaceRegion>& regions,
                                                const RegionParams& params) const;
};

}  // namespace topmap
