#pragma once

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"
#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// T21: project an arbitrary 3D start/goal point onto the traversable,
// robot-placeable surface and return the Region that owns the nearest triangle.
//
// R6 normal consistency: a candidate triangle must be roughly horizontal
// (angle(normal, +Z) <= plan.max_project_normal_diff_deg) and the query point
// must lie on its front face, so a point under an upper floor is never snapped
// through the ceiling onto that floor. The only negative tolerance allowed is
// plan.max_step_height, expressed through TraversabilityParams.
class RegionLocator {
 public:
  Result<RegionId> locate(const TriangleMesh& mesh, const TopologyGraph& graph,
                          const Eigen::Vector3f& point, const PlanParams& params) const;
};

}  // namespace topmap
