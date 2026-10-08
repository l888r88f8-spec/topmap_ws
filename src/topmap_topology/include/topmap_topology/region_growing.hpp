#pragma once

#include <vector>

#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/region_types.hpp"

namespace topmap {

// A4.7: the join score between two adjacent triangles. Each difference term is
// normalised by the corresponding TraversabilityParams limit so the weights act
// on dimensionless [0, 1] quantities and `region_score_threshold` is
// dimensionless as well (R1).
float regionJoinScore(const TriangleNode& a, const TriangleNode& b, const RegionParams& p);

// Traversability-constrained region growing (T13). Seeds are the unassigned
// traversable triangles, taken by descending area then ascending id, and
// expanded best-first while the join score stays below the threshold, the edge
// is traversable and the region stays slope/height homogeneous. Placeability
// (min_clearance >= required_clearance) is deliberately NOT applied here: it is
// T16's and the path validator's hard constraint, and eroding the Region early
// would corrupt T16's region-union width field.
class RegionGrower {
 public:
  Result<std::vector<SurfaceRegion>> grow(const TriangleMesh& mesh,
                                          const RegionParams& params) const;
};

}  // namespace topmap
