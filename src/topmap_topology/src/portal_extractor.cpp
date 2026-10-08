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

// One endpoint-connected Boundary Chain. `verts` is a deterministic traversal
// order of the component's vertices and `edges` holds each shared edge exactly
// once, so `raw_width` never double counts a pinch that two sub-branches share.
struct Chain {
  std::vector<PointId> verts;
  std::vector<RawEdge> edges;
  bool closed = false;
};

// Chains are the connected components of the shared-edge graph under "shares an
// endpoint". This keeps a pinched opening as a single portal (R15: kLoop = 8)
// while still keeping genuinely disjoint passages of the same Region pair as
// separate portals.
std::vector<Chain> buildChains(const std::vector<RawEdge>& edges) {
  const int edge_total = static_cast<int>(edges.size());
  std::vector<int> parent(static_cast<std::size_t>(edge_total));
  std::iota(parent.begin(), parent.end(), 0);
  const auto find = [&parent](int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };
  const auto unite = [&find, &parent](int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) parent[a] = b;
  };

  std::map<PointId, int> first_edge_at;
  for (int i = 0; i < edge_total; ++i) {
    for (PointId v : {edges[i].a, edges[i].b}) {
      const auto it = first_edge_at.find(v);
      if (it == first_edge_at.end()) {
        first_edge_at.emplace(v, i);
      } else {
        unite(i, it->second);
      }
    }
  }

  std::map<int, std::vector<int>> groups;
  for (int i = 0; i < edge_total; ++i) groups[find(i)].push_back(i);

  std::vector<Chain> chains;
  chains.reserve(groups.size());
  for (const auto& group : groups) {
    std::map<PointId, std::vector<int>> incident;
    for (int i : group.second) {
      incident[edges[i].a].push_back(i);
      incident[edges[i].b].push_back(i);
    }

    // Prefer a degree-1 endpoint as the traversal seed so a simple path is
    // emitted end to end; a pure ring has none and starts at the smallest id.
    PointId start = incident.begin()->first;
    bool has_open_end = false;
    for (const auto& entry : incident) {
      if (entry.second.size() == 1u) {
        start = entry.first;
        has_open_end = true;
        break;
      }
    }
    if (!has_open_end) {
      for (const auto& entry : incident) {
        start = std::min(start, entry.first);
      }
    }

    bool closed = true;
    for (const auto& entry : incident) {
      if (entry.second.size() != 2u) {
        closed = false;
        break;
      }
    }

    std::vector<char> used(static_cast<std::size_t>(edge_total), 0);
    Chain chain;
    chain.verts.push_back(start);
    for (int i : group.second) chain.edges.push_back(edges[i]);
    chain.closed = closed;

    std::vector<PointId> stack{start};
    while (!stack.empty()) {
      const PointId v = stack.back();
      int best_edge = -1;
      PointId best_other = kInvalidId;
      for (int i : incident[v]) {
        if (used[i]) continue;
        const PointId other = (edges[i].a == v) ? edges[i].b : edges[i].a;
        if (best_edge < 0 || other < best_other || (other == best_other && i < best_edge)) {
          best_edge = i;
          best_other = other;
        }
      }
      if (best_edge < 0) {
        stack.pop_back();
        continue;
      }
      used[best_edge] = 1;
      chain.verts.push_back(best_other);
      stack.push_back(best_other);
    }
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
      if (static_cast<int>(chain.edges.size()) < params.boundary_min_edges) continue;
      float length = 0.0f;
      for (const RawEdge& e : chain.edges) {
        length += (mesh.vertices()[e.b] - mesh.vertices()[e.a]).norm();
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
