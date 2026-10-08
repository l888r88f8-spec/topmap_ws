#pragma once

#include <vector>

#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/region_types.hpp"

namespace topmap {

// T15: extract Portal candidates from the shared mesh edges of adjacent
// Regions. Edges that touch the same region pair are linked at their shared
// endpoints into Boundary Chains; one region pair may own several independent
// chains (parallel portals), and each chain yields one Portal.
//
// `raw_width` is the arc length of the chain - basic candidate geometry only.
// It is deliberately NOT the traversal width: the chain can meander along a
// watershed boundary, so T16 recomputes `valid_width` from the mesh clearance
// field.
class PortalExtractor {
 public:
  Result<std::vector<Portal>> extract(const TriangleMesh& mesh,
                                      const std::vector<SurfaceRegion>& regions,
                                      const PortalParams& params) const;
};

}  // namespace topmap
