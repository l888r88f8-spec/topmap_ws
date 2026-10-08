#pragma once

#include <string>
#include <vector>

#include "topmap_core/params.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// A4.7: topology invariants used by the compressor (T19) to decide whether a
// contraction preserves the graph's reachability, loops and key connections.
class TopologyInvariants {
 public:
  static GraphSignature capture(const TopologyGraph& graph);

  // R11: asymmetric comparison (a = before contraction, b = after).
  //   1. connected_components must match.
  //   2. every origin id must reach the same component (core reachability).
  //   3. cycle_rank must be preserved.
  //   4. b must not introduce a bridge between origins that were not already
  //      linked by a's critical backbone (merge-invariant form of "no new
  //      critical connection").
  //   5. every parallel origin pair of a that disappears in b must have both
  //      endpoints merged into one super Region.
  static bool sameConnectivity(const GraphSignature& a, const GraphSignature& b,
                               std::string* reason);

  static std::vector<PortalId> bridgePortals(const TopologyGraph& graph);
  static std::vector<RegionId> articulationRegions(const TopologyGraph& graph);
  // R12: contraction safety, not vertex deletion. Returns regions that must not
  // be contracted: degree != 2, two portals leading to the same neighbour, or a
  // portal that is a unique cross-layer connection.
  static std::vector<RegionId> nonContractibleRegions(const TopologyGraph& graph);
  // Cross-layer (disjoint [min_z,max_z]) portals whose removal disconnects their
  // two endpoint regions, i.e. the only connection between the two height bands.
  static std::vector<PortalId> uniqueCrossLayerPortals(const TopologyGraph& graph);
};

// R12-b: two regions belong to the same slope regime when both are flat or both
// are ramps. The is_ramp flag set by T14 takes priority; regions that were not
// split still fall back to mean_slope vs params.ramp_slope_deg.
bool sameSlopeRegime(const SurfaceRegion& a, const SurfaceRegion& b, const RegionParams& params);

}  // namespace topmap
