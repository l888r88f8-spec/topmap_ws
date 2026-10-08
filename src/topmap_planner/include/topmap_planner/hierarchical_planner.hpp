#pragma once

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// T27: the end-to-end hierarchical planner. It chains the frozen stages in the
// order mandated by the design spec and the implementation plan:
//
//   RegionLocator -> TopologyAStar -> CorridorBuilder -> segmented SurfaceAStar
//   -> PathOptimizer -> SafetyValidator -> PlanResult
//
// Each stage has a documented fallback:
//   * a failed corridor build is retried with one extra expansion ring; if it
//     still fails and plan.fallback_full_mesh is set, the search runs over the
//     full robot-placeable mesh;
//   * a failed surface search is retried over the full placeable mesh when the
//     fallback is enabled and the current corridor is not already full;
//   * a failed optimization keeps the raw T24 discrete path;
//   * a rejected optimized path is re-validated on the raw path before the plan
//     is declared unsafe.
//
// Every returned PlanStats field describes the path that is actually returned.
class HierarchicalPlanner {
 public:
  // R1: clearance and curvature checks need robot/traversability params, so the
  // planner receives the complete configuration.
  explicit HierarchicalPlanner(TopmapConfig config);

  Result<PlanResult> plan(const TriangleMesh& mesh, const TopologyGraph& graph,
                          const PlanRequest& request) const;

 private:
  const TopmapConfig config_;
};

}  // namespace topmap
