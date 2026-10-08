#include "topmap_topology/topology_invariants.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace topmap {
namespace {

constexpr float kPi = 3.14159265358979323846f;

struct TopologyCuts {
  std::vector<PortalId> bridges;          // ascending
  std::vector<RegionId> articulation;     // ascending
};

// Tarjan over the region graph. Edges (portals) are identified by id so that
// parallel edges are handled correctly: only the exact edge used to enter a
// vertex is skipped, so a second parallel edge lowers low[] and prevents a
// false bridge / articulation verdict.
TopologyCuts ComputeCuts(const TopologyGraph& graph) {
  const std::vector<SurfaceRegion>& regions = graph.regions();
  const std::vector<Portal>& portals = graph.portals();
  const std::size_t n = regions.size();

  std::unordered_map<RegionId, std::size_t> index;
  for (std::size_t i = 0; i < n; ++i) index.emplace(regions[i].id, i);

  std::vector<std::vector<std::pair<std::size_t, PortalId>>> adjacency(n);
  for (const Portal& portal : portals) {
    const std::size_t a = index.at(portal.region_a);
    const std::size_t b = index.at(portal.region_b);
    adjacency[a].push_back({b, portal.id});
    adjacency[b].push_back({a, portal.id});
  }

  std::vector<int> discovery(n, -1);
  std::vector<int> low(n, 0);
  std::vector<bool> is_articulation(n, false);
  int timer = 0;
  TopologyCuts cuts;

  std::function<void(std::size_t, PortalId)> dfs = [&](std::size_t u, PortalId parent_edge) {
    discovery[u] = low[u] = timer++;
    int children = 0;
    for (const auto& edge : adjacency[u]) {
      const std::size_t v = edge.first;
      const PortalId edge_id = edge.second;
      if (edge_id == parent_edge) continue;
      if (discovery[v] == -1) {
        ++children;
        dfs(v, edge_id);
        low[u] = std::min(low[u], low[v]);
        if (parent_edge != kInvalidId && low[v] >= discovery[u]) is_articulation[u] = true;
        if (low[v] > discovery[u]) cuts.bridges.push_back(edge_id);
      } else {
        low[u] = std::min(low[u], discovery[v]);
      }
    }
    if (parent_edge == kInvalidId && children > 1) is_articulation[u] = true;
  };

  for (std::size_t i = 0; i < n; ++i)
    if (discovery[i] == -1) dfs(i, kInvalidId);

  for (std::size_t i = 0; i < n; ++i)
    if (is_articulation[i]) cuts.articulation.push_back(regions[i].id);

  std::sort(cuts.bridges.begin(), cuts.bridges.end());
  std::sort(cuts.articulation.begin(), cuts.articulation.end());
  return cuts;
}

RegionId MinOrigin(const TopologyGraph& graph, RegionId region) {
  const std::vector<RegionId>& origins = graph.originRegions(region);
  return origins.empty() ? region : origins.front();
}

template <typename T>
void SortUnique(std::vector<T>& values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

bool Contains(const std::vector<std::pair<RegionId, RegionId>>& sorted,
              const std::pair<RegionId, RegionId>& value) {
  return std::binary_search(sorted.begin(), sorted.end(), value);
}

bool LookupOriginRegion(const std::vector<std::pair<RegionId, RegionId>>& sorted, RegionId origin,
                        RegionId* region) {
  const auto it = std::lower_bound(
      sorted.begin(), sorted.end(), std::make_pair(origin, std::numeric_limits<RegionId>::min()),
      [](const std::pair<RegionId, RegionId>& lhs, const std::pair<RegionId, RegionId>& rhs) {
        return lhs.first < rhs.first;
      });
  if (it == sorted.end() || it->first != origin) return false;
  *region = it->second;
  return true;
}

}  // namespace

std::vector<PortalId> TopologyInvariants::bridgePortals(const TopologyGraph& graph) {
  return ComputeCuts(graph).bridges;
}

std::vector<RegionId> TopologyInvariants::articulationRegions(const TopologyGraph& graph) {
  return ComputeCuts(graph).articulation;
}

std::vector<PortalId> TopologyInvariants::uniqueCrossLayerPortals(const TopologyGraph& graph) {
  std::vector<PortalId> unique;
  for (const PortalId portal_id : ComputeCuts(graph).bridges) {
    const Portal& portal = *graph.portal(portal_id);
    const SurfaceRegion& a = *graph.region(portal.region_a);
    const SurfaceRegion& b = *graph.region(portal.region_b);
    const bool overlap = a.min_z <= b.max_z && b.min_z <= a.max_z;
    if (!overlap) unique.push_back(portal_id);  // a bridge between disjoint height bands
  }
  return unique;
}

std::vector<RegionId> TopologyInvariants::nonContractibleRegions(const TopologyGraph& graph) {
  const std::vector<PortalId> cross = uniqueCrossLayerPortals(graph);
  const std::unordered_set<PortalId> cross_set(cross.begin(), cross.end());

  std::vector<RegionId> blocked;
  for (const SurfaceRegion& region : graph.regions()) {
    const std::vector<PortalId> portals = graph.portalsOf(region.id);
    bool bad = portals.size() != 2;
    if (!bad && graph.neighborRegions(region.id).size() != 2)
      bad = true;  // both portals lead to the same neighbour -> contraction kills a loop
    if (!bad) {
      for (const PortalId portal_id : portals) {
        if (cross_set.count(portal_id) != 0) {
          bad = true;
          break;
        }
      }
    }
    if (bad) blocked.push_back(region.id);
  }
  std::sort(blocked.begin(), blocked.end());
  return blocked;
}

GraphSignature TopologyInvariants::capture(const TopologyGraph& graph) {
  GraphSignature sig;
  const std::vector<SurfaceRegion>& regions = graph.regions();
  const std::vector<Portal>& portals = graph.portals();
  sig.region_count = static_cast<int>(regions.size());
  sig.connected_components = graph.connectedComponents();

  std::unordered_map<RegionId, std::size_t> index;
  for (std::size_t i = 0; i < regions.size(); ++i) index.emplace(regions[i].id, i);

  // Connected components over current regions, mirroring the graph's own DSU.
  std::vector<std::size_t> parent(regions.size());
  std::iota(parent.begin(), parent.end(), 0);
  std::function<std::size_t(std::size_t)> find = [&](std::size_t x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };
  for (const Portal& portal : portals) {
    const std::size_t a = find(index.at(portal.region_a));
    const std::size_t b = find(index.at(portal.region_b));
    if (a != b) parent[b] = a;
  }

  // Canonical component representative = smallest origin id in the component.
  std::unordered_map<std::size_t, RegionId> component_min_origin;
  for (const RegionId origin : graph.allOriginIds()) {
    const RegionId current = graph.currentRegionOfOrigin(origin).value();
    const std::size_t root = find(index.at(current));
    const auto it = component_min_origin.find(root);
    if (it == component_min_origin.end() || origin < it->second) component_min_origin[root] = origin;
  }
  for (const RegionId origin : graph.allOriginIds()) {
    const RegionId current = graph.currentRegionOfOrigin(origin).value();
    const std::size_t root = find(index.at(current));
    sig.origin_component_of.push_back({origin, component_min_origin[root]});
    sig.origin_region_of.push_back({origin, current});
  }

  // Bridges, expressed as origin minima so they survive a merge/renumber.
  for (const PortalId portal_id : ComputeCuts(graph).bridges) {
    const Portal& portal = *graph.portal(portal_id);
    RegionId a = MinOrigin(graph, portal.region_a);
    RegionId b = MinOrigin(graph, portal.region_b);
    if (a > b) std::swap(a, b);
    sig.critical_origin_pairs.push_back({a, b});
  }
  SortUnique(sig.critical_origin_pairs);

  // Region pairs carrying parallel portals, again mapped to origin minima.
  std::vector<std::pair<RegionId, RegionId>> region_pairs;
  for (const SurfaceRegion& region : regions) {
    for (const RegionId neighbor : graph.neighborRegions(region.id)) {
      if (region.id < neighbor && graph.portalCountBetween(region.id, neighbor) >= 2)
        region_pairs.push_back({region.id, neighbor});
    }
  }
  SortUnique(region_pairs);
  for (const auto& pair : region_pairs) {
    RegionId a = MinOrigin(graph, pair.first);
    RegionId b = MinOrigin(graph, pair.second);
    if (a > b) std::swap(a, b);
    sig.parallel_origin_pairs.push_back({a, b});
  }
  SortUnique(sig.parallel_origin_pairs);

  sig.cycle_rank = static_cast<int>(portals.size()) - static_cast<int>(regions.size()) +
                   sig.connected_components;
  return sig;
}

bool TopologyInvariants::sameConnectivity(const GraphSignature& a, const GraphSignature& b,
                                          std::string* reason) {
  const auto fail = [reason](const char* message) {
    if (reason != nullptr) *reason = message;
    return false;
  };

  if (a.origin_component_of != b.origin_component_of)
    return fail("origin reachability changed");
  if (a.connected_components != b.connected_components)
    return fail("component count changed");
  if (a.cycle_rank != b.cycle_rank) return fail("cycle rank changed");

  // Rule 4 (merge-invariant): a bridge in b may only join origins that a's
  // critical backbone already linked. This rejects genuinely new critical
  // connections while accepting a bridge that was relabelled when its endpoint
  // regions merged (e.g. (0,1)+(1,2) -> (0,2)).
  {
    std::unordered_map<RegionId, RegionId> parent;
    std::function<RegionId(RegionId)> find = [&](RegionId x) -> RegionId {
      const auto it = parent.find(x);
      if (it == parent.end() || it->second == x) return x;
      const RegionId root = find(it->second);
      it->second = root;
      return root;
    };
    const auto add = [&parent](RegionId x) { parent.emplace(x, x); };
    const auto unite = [&](RegionId x, RegionId y) {
      add(x);
      add(y);
      const RegionId rx = find(x);
      const RegionId ry = find(y);
      if (rx != ry) parent[ry] = rx;
    };
    for (const auto& pair : a.critical_origin_pairs) unite(pair.first, pair.second);
    for (const auto& pair : b.critical_origin_pairs) {
      add(pair.first);
      add(pair.second);
      if (find(pair.first) != find(pair.second))
        return fail("new critical connection introduced");
    }
  }

  // Rule 5: a parallel origin pair of a that vanished in b is only legal when
  // both origins now live in one super Region.
  for (const auto& pair : a.parallel_origin_pairs) {
    if (Contains(b.parallel_origin_pairs, pair)) continue;
    RegionId region_a = kInvalidId;
    RegionId region_b = kInvalidId;
    const bool found_a = LookupOriginRegion(b.origin_region_of, pair.first, &region_a);
    const bool found_b = LookupOriginRegion(b.origin_region_of, pair.second, &region_b);
    if (!found_a || !found_b || region_a != region_b)
      return fail("parallel connection lost without merge");
  }
  return true;
}

bool sameSlopeRegime(const SurfaceRegion& a, const SurfaceRegion& b, const RegionParams& params) {
  const float threshold = params.ramp_slope_deg * kPi / 180.0f;
  const auto is_ramp_regime = [threshold](const SurfaceRegion& region) {
    return region.is_ramp || region.mean_slope > threshold;
  };
  return is_ramp_regime(a) == is_ramp_regime(b);
}

}  // namespace topmap
