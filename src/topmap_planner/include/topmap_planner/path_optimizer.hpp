#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/region_types.hpp"

namespace topmap {

// T25: path post-processing for the hierarchical planner:
//   shortcut -> funnel -> curvature-limited Laplacian smoothing.
//
// Every stage is a pure function of its inputs and deterministic. All public
// entry points are total: `shortcut`, `funnel` and `smooth` cannot return a
// Status, so malformed parameters/inputs make them return the input unchanged
// with `removed == 0` (shortcut). Only `optimize` reports failures.
class PathOptimizer {
 public:
  // Frozen A4.8 signature. `Corridor` stores only PortalId, so this overload
  // cannot recover Portal geometry and therefore runs shortcut -> smooth with
  // the funnel stage as an explicit deterministic no-op. Use the ordered-portal
  // overload below (T27, when the TopologyGraph is available) for the complete
  // shortcut -> funnel -> smooth chain.
  Result<std::vector<Eigen::Vector3f>> optimize(const std::vector<Eigen::Vector3f>& raw,
                                                const TriangleMesh& mesh, const Corridor& corridor,
                                                const PlanParams& params, const RobotParams& robot,
                                                const TraversabilityParams& trav) const;

  // T27 compatibility overload: `ordered_portals` are the Portals on the
  // topology path in traversal order, which lets the funnel stage run for real.
  Result<std::vector<Eigen::Vector3f>> optimize(const std::vector<Eigen::Vector3f>& raw,
                                                const TriangleMesh& mesh, const Corridor& corridor,
                                                const std::vector<Portal>& ordered_portals,
                                                const PlanParams& params, const RobotParams& robot,
                                                const TraversabilityParams& trav) const;

  // Greedy shortcut. From every anchor i it retries the farthest reachable j,
  // sampling (p_i, p_j) with an interval <= params.check_step including both
  // endpoints; each sample must map to the nearest corridor triangle within
  // params.max_project_radius, that triangle must be robot-placeable and
  // front-facing, and consecutive distinct sample triangles must share a
  // traversable edge / isEdgeTraversable inside one component. On success
  // `removed` grows by exactly j - i - 1. Invalid parameters/input: returns the
  // input unchanged and sets *removed = 0.
  std::vector<Eigen::Vector3f> shortcut(const std::vector<Eigen::Vector3f>& pts,
                                        const TriangleMesh& mesh, const Corridor& corridor,
                                        const PlanParams& params, const TraversabilityParams& trav,
                                        std::size_t* removed) const;

  // Simple Stupid Funnel over the ordered Portals. The Portal left/right labels
  // are treated as geometry only: the actual left/right side is derived from the
  // direction of travel, so reversed endpoints are handled. Start and goal come
  // from pts.front()/pts.back() and are kept fixed; 3D z of the emitted portal
  // endpoints is preserved. An empty portal list is a deterministic no-op, and
  // the output always has at least two points.
  std::vector<Eigen::Vector3f> funnel(const std::vector<Eigen::Vector3f>& pts,
                                      const std::vector<Portal>& ordered_portals) const;

  // iterations of 0.5-weight Laplacian smoothing with fixed endpoints. A
  // candidate interior point is only accepted when it projects to a
  // robot-placeable mesh triangle and both adjacent segments stay safe; a point
  // whose triple exceeds robot.max_path_curvature is reverted (never deleted,
  // so a high curvature is never hidden by dropping points). Invalid
  // parameters/input: returns the input unchanged.
  std::vector<Eigen::Vector3f> smooth(const std::vector<Eigen::Vector3f>& pts,
                                      const TriangleMesh& mesh, const PlanParams& params,
                                      const RobotParams& robot,
                                      const TraversabilityParams& trav) const;
};

}  // namespace topmap
