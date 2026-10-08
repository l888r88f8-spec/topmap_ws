#include "topmap_topology/portal_extractor.hpp"

#include <algorithm>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace topmap {
namespace {

struct RawEdge {
  PointId a = kInvalidId;
  PointId b = kInvalidId;
};

// One Boundary Chain, emitted as an ordered vertex sequence. Consecutive
// vertices are always joined by one of the input shared edges, so
// polylineLength(verts) is exactly the shared boundary length and no two
// non-adjacent branch tips are ever placed next to each other.
struct Chain {
  std::vector<PointId> verts;
  bool closed = false;
};

// Deterministic path decomposition of the shared-edge graph. The plan requires
// each chain vertex to have degree <= 2: when the interface branches (a vertex
// with three or more shared edges) the walk consumes incident edges in
// ascending (neighbour id, edge index) order from each growing end, so the two
// smallest-id branches form the through-path and the leftover edges seed new
// chains. This is scene-independent and reproducible.
std::vector<Chain> buildChains(const std::vector<RawEdge>& edges) {
  const int edge_total = static_cast<int>(edges.size());
  std::map<PointId, std::vector<std::pair<PointId, int>>> incident;
  for (int i = 0; i < edge_total; ++i) {
    incident[edges[i].a].emplace_back(edges[i].b, i);
    incident[edges[i].b].emplace_back(edges[i].a, i);
  }
  for (auto& entry : incident) {
    std::sort(entry.second.begin(), entry.second.end());
  }

  // Global edge order: (smaller endpoint, larger endpoint, edge index).
  std::vector<int> order(static_cast<std::size_t>(edge_total));
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&edges](int i, int j) {
    const auto key = [&edges](int e) {
      return std::make_tuple(std::min(edges[e].a, edges[e].b), std::max(edges[e].a, edges[e].b), e);
    };
    return key(i) < key(j);
  });

  std::vector<char> used(static_cast<std::size_t>(edge_total), 0);
  std::vector<Chain> chains;
  for (int seed : order) {
    if (used[seed]) continue;
    used[seed] = 1;
    Chain chain;
    chain.verts = {edges[seed].a, edges[seed].b};

    const auto already_on_path = [&chain](PointId v) {
      return std::find(chain.verts.begin(), chain.verts.end(), v) != chain.verts.end();
    };
    const auto extend = [&](bool front) {
      while (true) {
        const PointId end = front ? chain.verts.front() : chain.verts.back();
        const PointId other = front ? chain.verts.back() : chain.verts.front();
        int pick = -1;
        PointId pick_neighbour = kInvalidId;
        for (const auto& candidate : incident[end]) {
          const PointId neighbour = candidate.first;
          const int edge_id = candidate.second;
          if (used[edge_id]) continue;
          // Only the opposite chain end may repeat, and only to close a ring.
          if (neighbour != other && already_on_path(neighbour)) continue;
          if (pick < 0 || neighbour < pick_neighbour ||
              (neighbour == pick_neighbour && edge_id < pick)) {
            pick = edge_id;
            pick_neighbour = neighbour;
          }
        }
        if (pick < 0) break;
        used[pick] = 1;
        if (front) {
          chain.verts.insert(chain.verts.begin(), pick_neighbour);
        } else {
          chain.verts.push_back(pick_neighbour);
        }
        if (pick_neighbour == other) {
          chain.closed = true;
          break;
        }
      }
    };
    extend(false);
    extend(true);
    chains.push_back(std::move(chain));
  }
  return chains;
}

}  // namespace

Result<std::vector<Portal>> PortalExtractor::extract(
    const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
    const PortalParams& params) const {
  if (params.boundary_min_edges < 1) {
    return Status::Error(ErrorCode::kInvalidArgument, "portal.boundary_min_edges must be >= 1");
  }
  if (mesh.size() == 0) {
    return Status::Error(ErrorCode::kMeshEmpty, "cannot extract portals from an empty mesh");
  }
  if (regions.empty()) {
    return Status::Error(ErrorCode::kRegionEmpty, "cannot extract portals without regions");
  }

  // Region ownership. A triangle may belong to exactly one Region.
  std::vector<RegionId> triangle_region(static_cast<std::size_t>(mesh.size()), kInvalidId);
  std::set<RegionId> region_ids;
  std::size_t owned_triangles = 0;
  for (const SurfaceRegion& region : regions) {
    const RegionId id = region.id;
    if (id < 0) {
      return Status::Error(ErrorCode::kInvalidArgument, "region has a negative id");
    }
    if (!region_ids.insert(id).second) {
      return Status::Error(ErrorCode::kInvalidArgument, "duplicate RegionId " + std::to_string(id));
    }
    for (TriangleId t : region.triangle_ids) {
      if (t < 0 || t >= static_cast<TriangleId>(mesh.size())) {
        return Status::Error(ErrorCode::kInvalidArgument,
                             "region references an out-of-range triangle id");
      }
      if (triangle_region[t] != kInvalidId) {
        return Status::Error(ErrorCode::kInvalidArgument,
                             "triangle belongs to more than one region");
      }
      triangle_region[t] = id;
      ++owned_triangles;
    }
  }
  if (owned_triangles == 0) {
    return Status::Error(ErrorCode::kRegionEmpty, "regions contain no triangles");
  }

  // Shared mesh edges between different Regions, grouped by the (min, max)
  // RegionId pair. Each undirected edge is recorded once by requiring t < u.
  std::map<std::pair<RegionId, RegionId>, std::vector<RawEdge>> by_pair;
  for (TriangleId t = 0; t < static_cast<TriangleId>(mesh.size()); ++t) {
    const RegionId ra = triangle_region[t];
    if (ra == kInvalidId) continue;
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      const TriangleId u = e.neighbor;
      if (u == kInvalidId || u < t) continue;
      const RegionId rb = triangle_region[u];
      if (rb == kInvalidId || rb == ra) continue;
      if (e.v0 == kInvalidId || e.v1 == kInvalidId || e.v0 == e.v1) continue;
      const RegionId a = std::min(ra, rb);
      const RegionId b = std::max(ra, rb);
      by_pair[{a, b}].push_back(RawEdge{e.v0, e.v1});
    }
  }
  if (by_pair.empty()) {
    return Status::Error(ErrorCode::kPortalExtractionFailed,
                         "adjacent regions share no mesh edges");
  }

  std::vector<Portal> portals;
  for (const auto& entry : by_pair) {
    for (Chain& chain : buildChains(entry.second)) {
      const int edge_count = static_cast<int>(chain.verts.size()) - 1;
      if (edge_count < params.boundary_min_edges) continue;
      // raw_width is the polyline length of the ordered chain - the boundary
      // geometry only. T16 recomputes the traversal width from the mesh.
      float length = 0.0f;
      for (std::size_t k = 1; k < chain.verts.size(); ++k) {
        length += (mesh.vertices()[chain.verts[k]] - mesh.vertices()[chain.verts[k - 1]]).norm();
      }
      Portal portal;
      portal.region_a = entry.first.first;
      portal.region_b = entry.first.second;
      portal.boundary_chain = std::move(chain.verts);
      portal.raw_width = length;
      portal.left = mesh.vertices()[portal.boundary_chain.front()];
      portal.right = mesh.vertices()[portal.boundary_chain.back()];
      Eigen::Vector3f center = Eigen::Vector3f::Zero();
      for (PointId v : portal.boundary_chain) center += mesh.vertices()[v];
      center /= static_cast<float>(portal.boundary_chain.size());
      portal.center = center;
      portals.push_back(std::move(portal));
    }
  }

  if (portals.empty()) {
    return Status::Error(ErrorCode::kPortalExtractionFailed,
                         "no boundary chain reached portal.boundary_min_edges");
  }

  // Stable, scene-independent ordering: region pair, then the left endpoint and
  // finally the full chain, so ids are reproducible.
  std::sort(portals.begin(), portals.end(), [](const Portal& x, const Portal& y) {
    return std::make_tuple(x.region_a, x.region_b, x.left.x(), x.left.y(), x.left.z(),
                           x.right.x(), x.right.y(), x.right.z(), x.raw_width,
                           x.boundary_chain) <
           std::make_tuple(y.region_a, y.region_b, y.left.x(), y.left.y(), y.left.z(),
                           y.right.x(), y.right.y(), y.right.z(), y.raw_width,
                           y.boundary_chain);
  });
  for (std::size_t i = 0; i < portals.size(); ++i) {
    portals[i].id = static_cast<PortalId>(i);
  }
  return portals;
}

}  // namespace topmap
