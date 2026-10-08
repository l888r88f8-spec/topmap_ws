#pragma once

#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_surface/mesh_types.hpp"

namespace topmap {

// T24: segmented surface A* that reaches every input waypoint in order. Each
// adjacent waypoint pair is searched independently over the corridor triangles
// and the per-segment centroid polylines are concatenated (adjacent duplicates
// removed), so the returned path contains the start, every intermediate
// waypoint (or its exact projection) and the goal in the given order.
//
// R10: the search is a hard clearance constraint. A node may only be expanded
// when it is inside `corridor.triangles`, isRobotPlaceable(trav), shares a
// traversable edge and passes isEdgeTraversable; from/to are mapped onto the
// nearest placeable, front-facing corridor triangle whose normal is within
// trav.max_normal_diff_rad() of +Z.
class SurfaceAStar {
 public:
  Result<std::vector<Eigen::Vector3f>> search(const TriangleMesh& mesh, const Corridor& corridor,
                                              const std::vector<Eigen::Vector3f>& waypoints,
                                              const TraversabilityParams& trav,
                                              int* expanded_nodes = nullptr) const;
};

}  // namespace topmap
