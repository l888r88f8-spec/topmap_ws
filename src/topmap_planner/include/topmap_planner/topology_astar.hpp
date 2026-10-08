#pragma once

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// T22: A* over the explicit Region-Portal bipartite graph. A search state is
// either a Region or a Portal, so the reconstructed sequence strictly
// alternates Region, Portal, Region, ... .
//
// Entering a Portal from a Region costs distance(region.centroid,
// portal.center); entering a Region from a Portal costs
// portal.traversal_cost + distance(portal.center, region.centroid). The
// heuristic is the straight-line distance from the state's position (Region:
// centroid, Portal: center) to the goal Region centroid, scaled by `weight`;
// weight == 1 is an admissible A*.
class TopologyAStar {
 public:
  Result<TopologyPath> search(const TopologyGraph& graph, RegionId start, RegionId goal,
                              float weight = 1.0f, int* expanded_nodes = nullptr) const;
};

}  // namespace topmap
