#pragma once

#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"

namespace topmap {

// R10: the spec's Traversable = C_slope /\ C_roughness /\ C_support is split in
// two levels:
//   isTriangleTraversable  = C_slope /\ C_roughness /\ C_curvature /\ C_area
//   hasRequiredClearance   = min_clearance >= required_clearance
//   isRobotPlaceable       = the conjunction of both
// Surface A* and Region growing must require both so clearance is a hard search
// constraint rather than an after-the-fact check.
class TraversabilityEvaluator {
 public:
  static bool isTriangleTraversable(const TriangleNode& t, const TraversabilityParams& p);
  static bool hasRequiredClearance(const TriangleNode& t, const TraversabilityParams& p);
  static bool isRobotPlaceable(const TriangleNode& t, const TraversabilityParams& p);
  static bool isEdgeTraversable(const TriangleNode& a, const TriangleNode& b,
                                const TraversabilityParams& p);
  // max centroid -> other-plane distance, in metres.
  static float heightJump(const TriangleNode& a, const TriangleNode& b);
  // Fills traversable, min_clearance and the per-edge traversable flags.
  static void apply(TriangleMesh& mesh, const TraversabilityParams& p);
};

}  // namespace topmap
