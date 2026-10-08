#pragma once

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"
#include "topmap_topology/region_types.hpp"

namespace topmap {

// R11: the topology signature is defined on the reachability of the *original*
// regions, not on the current (possibly compressed) region ids. Capture is
// implemented by TopologyInvariants (T18); the struct lives here because
// TopologyGraph::setCriticalFlags consumes it and T17 must compile on its own.
struct GraphSignature {
  int region_count = 0;
  int connected_components = 0;
  // (origin_id, component_rep) with component_rep = smallest origin id of the
  // component. Sorted by origin_id. Reachability is "same component_rep".
  std::vector<std::pair<RegionId, RegionId>> origin_component_of;
  // (origin_id, current_region_id) sorted by origin_id. Region ids are only
  // meaningful inside one signature; sameConnectivity uses this to check that a
  // vanished parallel origin pair has been merged into one super Region.
  std::vector<std::pair<RegionId, RegionId>> origin_region_of;
  // Bridges expressed as (min origin of endpoint A, min origin of endpoint B),
  // each pair ordered internally, the list sorted and deduped.
  std::vector<std::pair<RegionId, RegionId>> critical_origin_pairs;
  // Region pairs carrying >= 2 portals, mapped to origin minima, sorted/deduped.
  std::vector<std::pair<RegionId, RegionId>> parallel_origin_pairs;
  int cycle_rank = 0;  // |portals| - |regions| + connected_components
};

// A4.7: the Region-Portal-Region bipartite graph. Regions and Portals are
// stored verbatim and indexed by their (sparse) ids; every returned id list is
// deterministically ascending. A build or replace validates the whole input
// before committing, so a rejected input never leaves a partially updated graph.
class TopologyGraph {
 public:
  static Result<TopologyGraph> build(const std::vector<SurfaceRegion>& regions,
                                     const std::vector<Portal>& portals);

  const std::vector<SurfaceRegion>& regions() const { return regions_; }
  const std::vector<Portal>& portals() const { return portals_; }
  const SurfaceRegion* region(RegionId id) const;
  const Portal* portal(PortalId id) const;
  bool hasRegion(RegionId id) const;
  std::vector<PortalId> portalsOf(RegionId id) const;
  std::vector<RegionId> neighborRegions(RegionId id) const;
  Result<RegionId> otherRegion(PortalId id, RegionId from) const;
  std::size_t portalCountBetween(RegionId a, RegionId b) const;
  Result<RegionId> regionOfTriangle(TriangleId id) const;
  int connectedComponents() const { return components_; }

  // Writes the signature/bridge verdict back onto the stored regions and
  // portals: a portal is critical iff its origin pair is a critical (bridge)
  // origin pair; a region is critical iff it carries at least one such portal.
  void setCriticalFlags(const GraphSignature& sig);

  // Rebuilds the graph from scratch through the same validation as build(). On
  // any validation failure the receiver is left completely unchanged. Origins
  // are inherited from an old region with the same id when one exists (so a
  // compressor can merge origins deliberately through setOriginRegions); brand
  // new ids default to themselves.
  void replaceRegionsPortals(std::vector<SurfaceRegion> regions, std::vector<Portal> portals);

  // R11 origin provenance. build() sets every region's origin to itself; the
  // compressor later folds absorbed regions into their super Region.
  const std::vector<RegionId>& originRegions(RegionId id) const;
  void setOriginRegions(RegionId id, std::vector<RegionId> origins);
  std::vector<RegionId> allOriginIds() const;
  Result<RegionId> currentRegionOfOrigin(RegionId origin_id) const;

 private:
  // Validates regions/portals and, on success, populates a fresh graph. The
  // optional previous graph supplies origin inheritance for replace().
  static Result<TopologyGraph> make(std::vector<SurfaceRegion> regions,
                                    std::vector<Portal> portals,
                                    const TopologyGraph* previous);

  std::vector<SurfaceRegion> regions_;
  std::vector<Portal> portals_;
  std::unordered_map<RegionId, std::size_t> region_index_;
  std::unordered_map<PortalId, std::size_t> portal_index_;
  std::unordered_map<TriangleId, RegionId> triangle_region_;
  std::vector<std::vector<PortalId>> region_portals_;    // per region index
  std::vector<std::vector<RegionId>> region_neighbors_;  // per region index, sorted/unique
  int components_ = 0;
  std::vector<std::vector<RegionId>> origin_regions_;    // per region index
  std::unordered_map<RegionId, RegionId> origin_to_current_;
};

}  // namespace topmap
