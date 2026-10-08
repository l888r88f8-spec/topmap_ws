#pragma once

#include <utility>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// A4.7 CompressionStats. `rejected_by_invariant` and `rejected_by_regime` break
// the rejection count down by cause so an empty compression can be diagnosed;
// the cost gate only contributes to `merges_rejected`.
//   rejected_by_invariant + rejected_by_regime <= merges_rejected
struct CompressionStats {
  int regions_before = 0, regions_after = 0, portals_before = 0, portals_after = 0;
  int merges_applied = 0, merges_rejected = 0;
  int rejected_by_invariant = 0, rejected_by_regime = 0;
};

// T19: topology-preserving compression. Degree-2 regions that do not damage the
// graph's reachability, loops or unique cross-layer connections are contracted
// into a neighbour, cheapest merge first. With `preserve_topology` (default) the
// reachability signature is captured before and after every attempt and the
// contraction is rolled back exactly if it changes.
class TopologyCompressor {
 public:
  Result<std::pair<TopologyGraph, CompressionStats>> compress(
      const TopologyGraph& graph, const CompressionParams& params,
      const RegionParams& region_params) const;
};

}  // namespace topmap
