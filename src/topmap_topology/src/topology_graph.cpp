#include "topmap_topology/topology_graph.hpp"

#include <algorithm>
#include <numeric>
#include <string>

namespace topmap {
namespace {

// Disjoint-set union over region indices, used only for the component count.
class DisjointSet {
 public:
  explicit DisjointSet(std::size_t n) : parent_(n) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  std::size_t find(std::size_t x) {
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];
      x = parent_[x];
    }
    return x;
  }
  void unite(std::size_t a, std::size_t b) {
    a = find(a);
    b = find(b);
    if (a != b) parent_[b] = a;
  }

 private:
  std::vector<std::size_t> parent_;
};

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

void SortUnique(std::vector<RegionId>& values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

}  // namespace

Result<TopologyGraph> TopologyGraph::build(const std::vector<SurfaceRegion>& regions,
                                           const std::vector<Portal>& portals) {
  return make(regions, portals, nullptr);
}

Result<TopologyGraph> TopologyGraph::make(std::vector<SurfaceRegion> regions,
                                          std::vector<Portal> portals,
                                          const TopologyGraph* previous) {
  TopologyGraph graph;
  std::unordered_map<RegionId, std::size_t> region_index;

  // Region ids must be valid and unique. Ids are sparse, so vector position is
  // never used as the identity.
  for (std::size_t i = 0; i < regions.size(); ++i) {
    const RegionId id = regions[i].id;
    if (id < 0) return Invalid("region has invalid id " + std::to_string(id));
    if (!region_index.emplace(id, i).second)
      return Invalid("duplicate region id " + std::to_string(id));
  }

  // Every triangle belongs to at most one region.
  for (const SurfaceRegion& region : regions) {
    for (const TriangleId triangle : region.triangle_ids) {
      if (triangle < 0)
        return Invalid("region " + std::to_string(region.id) + " has invalid triangle id " +
                       std::to_string(triangle));
      if (!graph.triangle_region_.emplace(triangle, region.id).second)
        return Invalid("triangle " + std::to_string(triangle) + " is owned by multiple regions");
    }
  }

  // Portal ids must be valid and unique; both endpoints must exist, differ and
  // be present in the region set.
  for (std::size_t i = 0; i < portals.size(); ++i) {
    const Portal& portal = portals[i];
    if (portal.id < 0) return Invalid("portal has invalid id " + std::to_string(portal.id));
    if (!graph.portal_index_.emplace(portal.id, i).second)
      return Invalid("duplicate portal id " + std::to_string(portal.id));
    if (portal.region_a < 0 || portal.region_b < 0)
      return Invalid("portal " + std::to_string(portal.id) + " references an invalid region id");
    if (portal.region_a == portal.region_b)
      return Invalid("portal " + std::to_string(portal.id) + " is a self loop on region " +
                     std::to_string(portal.region_a));
    if (region_index.find(portal.region_a) == region_index.end() ||
        region_index.find(portal.region_b) == region_index.end())
      return Invalid("portal " + std::to_string(portal.id) + " references a missing region");
  }

  // All checks passed: commit the payload and build the derived indices.
  graph.regions_ = std::move(regions);
  graph.portals_ = std::move(portals);
  graph.region_index_ = std::move(region_index);

  graph.region_portals_.assign(graph.regions_.size(), {});
  for (const Portal& portal : graph.portals_) {
    graph.region_portals_[graph.region_index_.at(portal.region_a)].push_back(portal.id);
    graph.region_portals_[graph.region_index_.at(portal.region_b)].push_back(portal.id);
  }
  for (std::vector<PortalId>& list : graph.region_portals_) SortUnique(list);

  graph.region_neighbors_.assign(graph.regions_.size(), {});
  for (std::size_t i = 0; i < graph.regions_.size(); ++i) {
    const RegionId id = graph.regions_[i].id;
    for (const PortalId portal_id : graph.region_portals_[i]) {
      const Portal& portal = *graph.portal(portal_id);
      graph.region_neighbors_[i].push_back(portal.region_a == id ? portal.region_b : portal.region_a);
    }
    SortUnique(graph.region_neighbors_[i]);
  }

  DisjointSet components(graph.regions_.size());
  for (const Portal& portal : graph.portals_) {
    components.unite(graph.region_index_.at(portal.region_a), graph.region_index_.at(portal.region_b));
  }
  graph.components_ = 0;
  for (std::size_t i = 0; i < graph.regions_.size(); ++i)
    if (components.find(i) == i) ++graph.components_;

  // R11: origin provenance. build() makes every region its own origin; replace()
  // inherits the origin list of any region id that already existed so a
  // compressor can explicitly fold origins through setOriginRegions.
  graph.origin_regions_.assign(graph.regions_.size(), {});
  for (std::size_t i = 0; i < graph.regions_.size(); ++i) {
    const RegionId id = graph.regions_[i].id;
    std::vector<RegionId> origins;
    if (previous != nullptr) {
      const auto it = previous->region_index_.find(id);
      if (it != previous->region_index_.end()) origins = previous->origin_regions_[it->second];
    }
    if (origins.empty()) origins = {id};
    SortUnique(origins);
    for (const RegionId origin : origins) {
      if (origin < 0) continue;
      if (graph.origin_to_current_.find(origin) != graph.origin_to_current_.end()) continue;
      graph.origin_to_current_.emplace(origin, id);
      graph.origin_regions_[i].push_back(origin);
    }
  }

  return graph;
}

const SurfaceRegion* TopologyGraph::region(RegionId id) const {
  const auto it = region_index_.find(id);
  return it == region_index_.end() ? nullptr : &regions_[it->second];
}

const Portal* TopologyGraph::portal(PortalId id) const {
  const auto it = portal_index_.find(id);
  return it == portal_index_.end() ? nullptr : &portals_[it->second];
}

bool TopologyGraph::hasRegion(RegionId id) const {
  return region_index_.find(id) != region_index_.end();
}

std::vector<PortalId> TopologyGraph::portalsOf(RegionId id) const {
  const auto it = region_index_.find(id);
  return it == region_index_.end() ? std::vector<PortalId>{} : region_portals_[it->second];
}

std::vector<RegionId> TopologyGraph::neighborRegions(RegionId id) const {
  const auto it = region_index_.find(id);
  return it == region_index_.end() ? std::vector<RegionId>{} : region_neighbors_[it->second];
}

Result<RegionId> TopologyGraph::otherRegion(PortalId id, RegionId from) const {
  const Portal* p = portal(id);
  if (p == nullptr) return Invalid("unknown portal id " + std::to_string(id));
  if (p->region_a == from) return p->region_b;
  if (p->region_b == from) return p->region_a;
  return Invalid("region " + std::to_string(from) + " is not incident to portal " +
                 std::to_string(id));
}

std::size_t TopologyGraph::portalCountBetween(RegionId a, RegionId b) const {
  if (a == b) return 0;
  const auto it = region_index_.find(a);
  if (it == region_index_.end()) return 0;
  std::size_t count = 0;
  for (const PortalId portal_id : region_portals_[it->second]) {
    const Portal& p = *portal(portal_id);
    if ((p.region_a == a && p.region_b == b) || (p.region_a == b && p.region_b == a)) ++count;
  }
  return count;
}

Result<RegionId> TopologyGraph::regionOfTriangle(TriangleId id) const {
  const auto it = triangle_region_.find(id);
  if (it == triangle_region_.end())
    return Invalid("triangle " + std::to_string(id) + " is not owned by any region");
  return it->second;
}

void TopologyGraph::setCriticalFlags(const GraphSignature& sig) {
  for (SurfaceRegion& region : regions_) region.is_topology_critical = false;
  for (Portal& portal : portals_) portal.is_critical = false;

  const auto min_origin = [this](RegionId id) -> RegionId {
    const auto it = region_index_.find(id);
    if (it == region_index_.end() || origin_regions_[it->second].empty()) return id;
    return origin_regions_[it->second].front();
  };

  for (Portal& portal : portals_) {
    std::pair<RegionId, RegionId> pair{min_origin(portal.region_a), min_origin(portal.region_b)};
    if (pair.first > pair.second) std::swap(pair.first, pair.second);
    if (std::binary_search(sig.critical_origin_pairs.begin(), sig.critical_origin_pairs.end(), pair))
      portal.is_critical = true;
  }

  for (std::size_t i = 0; i < regions_.size(); ++i) {
    for (const PortalId portal_id : region_portals_[i]) {
      if (portal(portal_id)->is_critical) {
        regions_[i].is_topology_critical = true;
        break;
      }
    }
  }
}

void TopologyGraph::replaceRegionsPortals(std::vector<SurfaceRegion> regions,
                                          std::vector<Portal> portals) {
  Result<TopologyGraph> rebuilt = make(std::move(regions), std::move(portals), this);
  if (!rebuilt.ok()) return;  // Transactional: a rejected replacement changes nothing.
  *this = rebuilt.take();
}

const std::vector<RegionId>& TopologyGraph::originRegions(RegionId id) const {
  static const std::vector<RegionId> kEmpty;
  const auto it = region_index_.find(id);
  return it == region_index_.end() ? kEmpty : origin_regions_[it->second];
}

void TopologyGraph::setOriginRegions(RegionId id, std::vector<RegionId> origins) {
  const auto it = region_index_.find(id);
  if (it == region_index_.end()) return;

  std::vector<RegionId> cleaned;
  for (const RegionId origin : origins)
    if (origin >= 0) cleaned.push_back(origin);
  SortUnique(cleaned);

  // An origin may belong to exactly one current region. On any conflict the
  // whole update is rejected so the reverse index can never go stale.
  for (const RegionId origin : cleaned) {
    const auto current = origin_to_current_.find(origin);
    if (current != origin_to_current_.end() && current->second != id) return;
  }

  const std::size_t index = it->second;
  for (const RegionId origin : origin_regions_[index]) origin_to_current_.erase(origin);
  origin_regions_[index] = cleaned;
  for (const RegionId origin : cleaned) origin_to_current_[origin] = id;
}

std::vector<RegionId> TopologyGraph::allOriginIds() const {
  std::vector<RegionId> ids;
  for (const std::vector<RegionId>& list : origin_regions_)
    ids.insert(ids.end(), list.begin(), list.end());
  SortUnique(ids);
  return ids;
}

Result<RegionId> TopologyGraph::currentRegionOfOrigin(RegionId origin_id) const {
  const auto it = origin_to_current_.find(origin_id);
  if (it == origin_to_current_.end())
    return Invalid("unknown origin region id " + std::to_string(origin_id));
  return it->second;
}

}  // namespace topmap
