#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// T26: post-planning safety re-check. This is the last line of defence before a
// path is handed to the robot, so it recomputes every criterion from the mesh
// and the raw polyline instead of trusting the upstream stages:
//   * every sample of the resampled path must project onto a robot-placeable,
//     front-facing triangle (clearance, slope, roughness, curvature, area);
//   * every crossed shared edge must be edge-traversable (normal difference,
//     height jump, optional slope-difference);
//   * the path triple curvature must stay within robot.max_path_curvature;
//   * the crossed-Portal sequence must be verifiable against the topology.
class SafetyValidator {
 public:
  // Frozen A4.8 signature. Fills a structured report; malformed inputs (empty
  // mesh, fewer than two points, non-finite point, non-positive check_step,
  // invalid traversability parameters) yield `valid == false` rather than
  // throwing. `violating_indices` holds the deduplicated input-vertex indices
  // responsible for every failure.
  PathValidationReport validate(const std::vector<Eigen::Vector3f>& path, const TriangleMesh& mesh,
                                const TraversabilityParams& params, const RobotParams& robot,
                                float check_step) const;

  // T33: batch per-segment safety check used by Method A's graph builder. The
  // spatial index is built once and reused for every segment, so checking the
  // whole candidate edge set is O(sum of sampled lengths), never
  // O(edges * mesh). For each segment the result is true iff every resampled
  // sample projects to a robot-placeable, front-facing triangle and every
  // consecutive pair of projections is joined by a traversable shared edge:
  // exactly the placeability/clearance/step part of validate(), without the
  // whole-path slope and curvature tests. Invalid arguments (non-positive
  // check_step, empty mesh, non-finite endpoint, invalid traversability) yield
  // kInvalidArgument.
  Result<std::vector<char>> validateSegments(
      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& segments,
      const TriangleMesh& mesh, const TraversabilityParams& params, float check_step) const;

  // Nearest baked-traversable triangle to `p` by true point-triangle distance.
  // Ties go to the smallest TriangleId. On success `*distance` (when non-null)
  // receives the true 3D distance. A non-finite query, a negative/non-finite
  // max_distance, an empty mesh or a nearest distance above max_distance all
  // return kInvalidArgument.
  Result<TriangleId> locatePoint(const TriangleMesh& mesh, const Eigen::Vector3f& p,
                                 float max_distance, float* distance = nullptr) const;

  // R6 oriented variant: additionally requires the triangle normal to be within
  // max_normal_diff_rad of +Z and `p` to lie on the triangle front side. This is
  // what stops a query from snapping onto the underside of a higher floor.
  Result<TriangleId> locatePointOriented(const TriangleMesh& mesh, const Eigen::Vector3f& p,
                                         float max_distance, float max_normal_diff_rad,
                                         float* distance = nullptr) const;

  // R13: real geometric Portal crossing detection for acceptance metric A-03.
  // The Portal boundary chain is a shared-edge polyline; a path segment counts
  // as crossing a Portal when it intersects that chain geometry. The returned
  // ids preserve first-crossing order and contain no duplicates, so a parallel
  // Portal pair on the same Region pair is disambiguated by geometry rather than
  // by the two endpoint Regions alone.
  static std::vector<PortalId> crossedPortals(const std::vector<Eigen::Vector3f>& path,
                                              const TriangleMesh& mesh, const TopologyGraph& graph);

  // Exact sequence comparison against crossedPortals(); repeats in `expected`
  // never match because crossedPortals() is duplicate-free.
  static bool portalOrderMatches(const std::vector<Eigen::Vector3f>& path, const TriangleMesh& mesh,
                                 const TopologyGraph& graph, const std::vector<PortalId>& expected);
};

}  // namespace topmap
