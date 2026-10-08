#include "topmap_topology/region_split.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float toRad(float deg) { return deg * kPi / 180.0f; }

// Small union-find used for the width-field merge tree and for grouping the
// watershed boundary into individual passages.
struct Dsu {
  std::vector<int> parent;
  void reset(int n) {
    parent.resize(n);
    for (int i = 0; i < n; ++i) parent[i] = i;
  }
  int find(int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) parent[a] = b;
  }
};

SurfaceRegion makeRegionStats(RegionId id, const TriangleMesh& mesh,
                              const std::vector<TriangleId>& ids, float ramp_slope_rad) {
  SurfaceRegion region;
  region.id = id;
  region.triangle_ids = ids;
  double area = 0.0, sum_slope = 0.0, sum_slope2 = 0.0, sum_rough = 0.0;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  float max_slope = 0.0f;
  for (TriangleId t : ids) {
    const TriangleNode& n = mesh.triangle(t);
    area += n.area;
    centroid += n.area * n.centroid.cast<double>();
    sum_slope += n.area * n.slope;
    sum_slope2 += n.area * n.slope * n.slope;
    sum_rough += n.area * n.roughness;
    min_z = std::min(min_z, n.centroid.z());
    max_z = std::max(max_z, n.centroid.z());
    max_slope = std::max(max_slope, n.slope);
  }
  if (area > 0.0) {
    region.centroid = (centroid / area).cast<float>();
    region.mean_slope = static_cast<float>(sum_slope / area);
    region.roughness = static_cast<float>(sum_rough / area);
    const double mean = sum_slope / area;
    const double var = sum_slope2 / area - mean * mean;
    region.slope_variance = static_cast<float>(var > 0.0 ? var : 0.0);
  }
  region.area = static_cast<float>(area);
  region.min_z = min_z;
  region.max_z = max_z;
  region.max_slope = max_slope;
  region.is_ramp = region.mean_slope > ramp_slope_rad;
  return region;
}

// Split a region into shared-edge connected components. When `blocked` is
// non-null, edges whose (v0, v1) key appears in it are treated as removed.
std::vector<std::vector<TriangleId>> edgeComponents(
    const TriangleMesh& mesh, const std::vector<TriangleId>& ids,
    const std::set<std::pair<PointId, PointId>>* blocked) {
  std::vector<char> inreg(mesh.size(), 0);
  for (TriangleId t : ids) inreg[t] = 1;
  std::vector<char> seen(mesh.size(), 0);
  std::vector<std::vector<TriangleId>> comps;
  for (TriangleId seed : ids) {
    if (seen[seed]) continue;
    seen[seed] = 1;
    std::vector<TriangleId> stack{seed};
    std::vector<TriangleId> comp;
    while (!stack.empty()) {
      const TriangleId t = stack.back();
      stack.pop_back();
      comp.push_back(t);
      for (const TriangleEdge& e : mesh.triangle(t).edges) {
        const TriangleId u = e.neighbor;
        if (u == kInvalidId || !inreg[u] || seen[u]) continue;
        if (blocked != nullptr) {
          const std::pair<PointId, PointId> key{std::min(e.v0, e.v1), std::max(e.v0, e.v1)};
          if (blocked->count(key) != 0) continue;
        }
        seen[u] = 1;
        stack.push_back(u);
      }
    }
    std::sort(comp.begin(), comp.end());
    comps.push_back(std::move(comp));
  }
  std::sort(comps.begin(), comps.end(),
            [](const std::vector<TriangleId>& a, const std::vector<TriangleId>& b) {
              return a.front() < b.front();
            });
  return comps;
}

// Multi-source Dijkstra over the region: every seed owns the triangles closest
// to it (geodesic-centroid metric). Returns a per-triangle owner index; -1
// outside the region.
void multiSourceAssign(const TriangleMesh& mesh, const std::vector<char>& inreg,
                       const std::vector<TriangleId>& seeds, std::vector<int>& owner) {
  const float inf = std::numeric_limits<float>::infinity();
  owner.assign(mesh.size(), -1);
  std::vector<float> dist(mesh.size(), inf);
  using Item = std::pair<float, TriangleId>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const TriangleId s = seeds[i];
    if (s == kInvalidId || s >= static_cast<TriangleId>(mesh.size()) || !inreg[s]) continue;
    if (owner[s] != -1) continue;
    dist[s] = 0.0f;
    owner[s] = static_cast<int>(i);
    pq.push({0.0f, s});
  }
  while (!pq.empty()) {
    const Item item = pq.top();
    pq.pop();
    const float d = item.first;
    const TriangleId t = item.second;
    if (d > dist[t]) continue;
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      const TriangleId u = e.neighbor;
      if (u == kInvalidId || !inreg[u]) continue;
      const float nd = d + (mesh.triangle(t).centroid - mesh.triangle(u).centroid).norm();
      if (nd < dist[u]) {
        dist[u] = nd;
        owner[u] = owner[t];
        pq.push({nd, u});
      }
    }
  }
  for (std::size_t i = 0; i < inreg.size(); ++i) {
    if (inreg[i] != 0 && owner[i] == -1) owner[i] = 0;
  }
}

struct WidthLeaf {
  TriangleId rep = kInvalidId;
  float peak = 0.0f;
  float saddle = std::numeric_limits<float>::infinity();
  float area = 0.0f;
};

// Persistent local maxima of the width field. Processing the region in
// descending width and unioning components yields a merge tree; a leaf is a
// navigable "room" if it either never merges or merges through a saddle much
// narrower than its own peak. The criterion is purely relative to the width
// field of this region, so no scene size or absolute coordinate is used.
std::vector<TriangleId> significantCores(const TriangleMesh& mesh,
                                         const std::vector<TriangleId>& ids,
                                         const std::vector<float>& width,
                                         const RegionParams& params,
                                         std::vector<WidthLeaf>* out_leaves) {
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<TriangleId> order = ids;
  std::sort(order.begin(), order.end(), [&](TriangleId a, TriangleId b) {
    if (width[a] != width[b]) return width[a] > width[b];
    return a < b;
  });

  std::vector<int> parent(mesh.size(), -1);
  for (TriangleId t : ids) parent[t] = t;
  std::vector<float> comp_peak(mesh.size(), 0.0f);
  std::vector<int> comp_leaf(mesh.size(), -1);
  std::vector<char> active(mesh.size(), 0);
  std::vector<WidthLeaf> leaves;

  const auto find = [&](int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };

  for (TriangleId t : order) {
    active[t] = 1;
    int roots[3];
    int nroots = 0;
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      const TriangleId u = e.neighbor;
      if (u == kInvalidId || !active[u]) continue;
      const int r = find(u);
      bool duplicate = false;
      for (int k = 0; k < nroots; ++k) {
        if (roots[k] == r) duplicate = true;
      }
      if (!duplicate) roots[nroots++] = r;
    }
    if (nroots == 0) {
      const int leaf = static_cast<int>(leaves.size());
      leaves.push_back(WidthLeaf{t, width[t], inf, mesh.triangle(t).area});
      comp_peak[t] = width[t];
      comp_leaf[t] = leaf;
      parent[t] = t;
    } else {
      int survivor = roots[0];
      for (int k = 1; k < nroots; ++k) {
        const float pk = comp_peak[roots[k]];
        const float ps = comp_peak[survivor];
        if (pk > ps || (pk == ps &&
                        leaves[comp_leaf[roots[k]]].rep < leaves[comp_leaf[survivor]].rep)) {
          survivor = roots[k];
        }
      }
      const int sleaf = comp_leaf[survivor];
      leaves[sleaf].area += mesh.triangle(t).area;
      for (int k = 0; k < nroots; ++k) {
        const int r = roots[k];
        if (r == survivor) continue;
        const int rleaf = comp_leaf[r];
        leaves[rleaf].saddle = width[t];
        leaves[sleaf].area += leaves[rleaf].area;
        parent[r] = survivor;
      }
      parent[t] = survivor;
      comp_peak[survivor] = std::max(comp_peak[survivor], width[t]);
      comp_leaf[survivor] = sleaf;
    }
  }

  std::vector<TriangleId> cores;
  for (const WidthLeaf& leaf : leaves) {
    const bool significant =
        leaf.area >= params.min_side_area &&
        (leaf.saddle == inf ||
         (leaf.saddle < params.bottleneck_ratio * leaf.peak &&
          leaf.saddle < params.bottleneck_abs_ceiling));
    if (significant) cores.push_back(leaf.rep);
  }
  if (out_leaves != nullptr) *out_leaves = leaves;
  return cores;
}

// Split one grown region at its navigation bottlenecks. Rooms are the
// persistent maxima of the region's own boundary-distance field; a passage is a
// connected group of watershed boundaries whose narrowest crossing is a
// significant bottleneck. Rooms separated by a wide boundary are the same room
// and are re-merged, so a convex region with one maximum stays intact.
std::vector<std::vector<TriangleId>> splitBottlenecksOne(const TriangleMesh& mesh,
                                                         const std::vector<TriangleId>& ids,
                                                         const RegionParams& params) {
  std::vector<char> inreg(mesh.size(), 0);
  for (TriangleId t : ids) inreg[t] = 1;

  const std::vector<float> rbd = regionBoundaryDistance(mesh, ids);
  std::vector<float> width(mesh.size(), 0.0f);
  std::vector<float> samples;
  samples.reserve(ids.size());
  for (TriangleId t : ids) {
    width[t] = 2.0f * rbd[t];
    samples.push_back(width[t]);
  }
  std::sort(samples.begin(), samples.end());
  const float median = samples[samples.size() / 2];
  const float threshold =
      std::min(params.bottleneck_ratio * median, params.bottleneck_abs_ceiling);

  const std::vector<TriangleId> cores = significantCores(mesh, ids, width, params, nullptr);
  if (cores.size() <= 1) return {ids};

  std::vector<int> room_owner;
  multiSourceAssign(mesh, inreg, cores, room_owner);

  std::vector<std::pair<TriangleId, TriangleId>> bedges;
  for (TriangleId t : ids) {
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      const TriangleId u = e.neighbor;
      if (u == kInvalidId || !inreg[u] || u < t) continue;
      if (room_owner[t] != room_owner[u]) bedges.push_back({t, u});
    }
  }
  if (bedges.empty()) return {ids};

  const int ne = static_cast<int>(bedges.size());
  Dsu edges;
  edges.reset(ne);
  // Group boundary edges that meet at a shared vertex. A boundary edge is
  // represented by the vertices of its two adjacent triangles; edges touching
  // the same vertex belong to the same passage. An inverted vertex -> first
  // incident edge table merges them in O(E alpha(E)) instead of testing every
  // edge pair.
  std::vector<int> vertex_edge(mesh.vertices().size(), -1);
  for (int i = 0; i < ne; ++i) {
    const auto mergeTriangle = [&](TriangleId t) {
      for (PointId v : mesh.triangle(t).vertex_ids) {
        const int prev = vertex_edge[v];
        if (prev < 0) {
          vertex_edge[v] = i;
        } else {
          edges.unite(prev, i);
        }
      }
    };
    mergeTriangle(bedges[i].first);
    mergeTriangle(bedges[i].second);
  }

  std::map<int, float> group_min;
  std::map<int, TriangleId> group_rep;
  for (int i = 0; i < ne; ++i) {
    const int g = edges.find(i);
    const float w = width[bedges[i].first] + width[bedges[i].second];
    const auto it = group_min.find(g);
    if (it == group_min.end() || w < it->second) {
      group_min[g] = w;
      group_rep[g] = bedges[i].first;
    }
  }

  std::set<int> bottleneck_groups;
  for (const auto& kv : group_min) {
    if (kv.second < threshold && kv.second < params.bottleneck_abs_ceiling) {
      bottleneck_groups.insert(kv.first);
    }
  }

  Dsu cells;
  cells.reset(static_cast<int>(cores.size()));
  for (int i = 0; i < ne; ++i) {
    const int g = edges.find(i);
    if (bottleneck_groups.count(g) != 0) continue;
    const int a = room_owner[bedges[i].first];
    const int b = room_owner[bedges[i].second];
    if (a >= 0 && b >= 0) cells.unite(a, b);
  }

  std::vector<TriangleId> seeds;
  std::set<int> emitted;
  for (int i = 0; i < static_cast<int>(cores.size()); ++i) {
    const int root = cells.find(i);
    if (emitted.insert(root).second) seeds.push_back(cores[i]);
  }
  for (int g : bottleneck_groups) seeds.push_back(group_rep[g]);
  std::sort(seeds.begin(), seeds.end());
  seeds.erase(std::unique(seeds.begin(), seeds.end()), seeds.end());

  if (seeds.size() <= 1) return {ids};

  std::vector<int> final_owner;
  multiSourceAssign(mesh, inreg, seeds, final_owner);
  std::map<int, std::vector<TriangleId>> groups;
  for (TriangleId t : ids) groups[final_owner[t]].push_back(t);

  std::vector<std::vector<TriangleId>> out;
  out.reserve(groups.size());
  for (auto& kv : groups) out.push_back(std::move(kv.second));
  return out;
}

// ---------------------------------------------------------------------------
// Concave (medial-axis branch) splitting.
//
// A region is only split when its boundary turns sharply inward (a reflex
// corner). For each such corner a cut is shot through the region along the
// inward angle bisector; the cut is materialised by blocking the shared edges
// it crosses. The arm length and angle thresholds come from the RegionParams.
// Convex regions (rooms, corridors, flat floors) have no reflex corner and are
// returned unchanged.
// ---------------------------------------------------------------------------

struct V2 {
  float x = 0.0f;
  float y = 0.0f;
};

V2 sub(V2 a, V2 b) { return V2{a.x - b.x, a.y - b.y}; }
float cross2(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
float dot2(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
float norm2(V2 a) { return std::sqrt(dot2(a, a)); }
V2 normalized(V2 a) {
  const float n = norm2(a);
  if (!(n > 1e-9f)) return V2{0.0f, 0.0f};
  return V2{a.x / n, a.y / n};
}

bool segmentsCross(V2 p1, V2 p2, V2 q1, V2 q2) {
  const auto orient = [](V2 a, V2 b, V2 c) {
    const float v = cross2(sub(b, a), sub(c, a));
    if (v > 1e-9f) return 1;
    if (v < -1e-9f) return -1;
    return 0;
  };
  const int o1 = orient(p1, p2, q1);
  const int o2 = orient(p1, p2, q2);
  const int o3 = orient(q1, q2, p1);
  const int o4 = orient(q1, q2, p2);
  return o1 * o2 < 0 && o3 * o4 < 0;
}

double hullArea(std::vector<V2> points) {
  const int n = static_cast<int>(points.size());
  if (n < 3) return 0.0;
  std::sort(points.begin(), points.end(),
            [](const V2& a, const V2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  std::vector<V2> hull;
  for (int i = 0; i < n; ++i) {
    while (hull.size() >= 2u &&
           cross2(sub(hull.back(), hull[hull.size() - 2u]), sub(points[i], hull[hull.size() - 2u])) <=
               0.0f) {
      hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  const std::size_t lower = hull.size() + 1u;
  for (int i = n - 2; i >= 0; --i) {
    while (hull.size() >= lower &&
           cross2(sub(hull.back(), hull[hull.size() - 2u]), sub(points[i], hull[hull.size() - 2u])) <=
               0.0f) {
      hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  if (hull.size() > 1u) hull.pop_back();
  double area = 0.0;
  const std::size_t m = hull.size();
  for (std::size_t i = 0; i < m; ++i) {
    const V2 a = hull[i];
    const V2 b = hull[(i + 1) % m];
    area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
  }
  return std::fabs(area) * 0.5;
}

// Alternative reflex-corner bisector split. Retained as an independent
// implementation; the active concave pass uses the medial-axis split below.
[[maybe_unused]] std::vector<std::vector<TriangleId>> splitConcaveBisectorOne(
    const TriangleMesh& mesh, const std::vector<TriangleId>& ids, const RegionParams& params) {
  if (ids.size() < 4u) return {ids};

  std::vector<char> inreg(mesh.size(), 0);
  for (TriangleId t : ids) inreg[t] = 1;

  // Region boundary as undirected vertex pairs.
  std::set<std::pair<PointId, PointId>> boundary;
  for (TriangleId t : ids) {
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      if (e.neighbor != kInvalidId && inreg[e.neighbor]) continue;
      boundary.insert({std::min(e.v0, e.v1), std::max(e.v0, e.v1)});
    }
  }
  if (boundary.size() < 4u) return {ids};

  // Best-fit plane of the region for a stable 2D projection.
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double total_area = 0.0;
  for (TriangleId t : ids) {
    const TriangleNode& n = mesh.triangle(t);
    normal += n.normal.cast<double>() * n.area;
    center += n.centroid.cast<double>() * n.area;
    total_area += n.area;
  }
  if (!(total_area > 0.0)) return {ids};
  if (normal.norm() < 1e-9) normal = Eigen::Vector3d::UnitZ();
  normal.normalize();
  center /= total_area;
  Eigen::Vector3d u = std::fabs(normal.x()) < 0.9 ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
  u = (u - normal * normal.dot(u)).normalized();
  const Eigen::Vector3d v = normal.cross(u);
  const auto project = [&](PointId id) {
    const Eigen::Vector3d q = mesh.vertices()[id].cast<double>() - center;
    return V2{static_cast<float>(q.dot(u)), static_cast<float>(q.dot(v))};
  };

  std::map<PointId, std::vector<PointId>> adjacency;
  for (const auto& e : boundary) {
    adjacency[e.first].push_back(e.second);
    adjacency[e.second].push_back(e.first);
  }

  // Convexity gate: only a region that is visibly non-convex (a limb shape)
  // can carry a meaningful reflex corner. A room or corridor is close to its
  // own convex hull and is left untouched, which keeps the earlier bottleneck
  // partition stable.
  double region_area = 0.0;
  for (TriangleId t : ids) region_area += mesh.triangle(t).area;
  std::vector<V2> boundary_points;
  boundary_points.reserve(boundary.size());
  for (const auto& e : boundary) boundary_points.push_back(project(e.first));
  const double hull = hullArea(boundary_points);
  if (!(region_area > 0.0) || hull < 1.25 * region_area) return {ids};

  // Walk the boundary graph into closed loops, consuming each undirected
  // boundary edge exactly once. Marking edges (rather than only the vertex we
  // came from) makes the walk terminate where several boundary loops meet at a
  // single pinched vertex.
  using BoundaryEdge = std::pair<PointId, PointId>;
  std::set<BoundaryEdge> used_edges;
  std::vector<std::vector<PointId>> loops;
  for (const BoundaryEdge& seed : boundary) {
    if (used_edges.count(seed) != 0) continue;
    const PointId start = seed.first;
    std::vector<PointId> loop;
    PointId current = start;
    while (true) {
      loop.push_back(current);
      PointId next = kInvalidId;
      for (PointId candidate : adjacency[current]) {
        const BoundaryEdge key{std::min(current, candidate), std::max(current, candidate)};
        if (used_edges.count(key) == 0) {
          used_edges.insert(key);
          next = candidate;
          break;
        }
      }
      if (next == kInvalidId || next == start) break;
      current = next;
    }
    if (loop.size() >= 4u) loops.push_back(std::move(loop));
  }
  if (loops.empty()) return {ids};

  const float arm = std::max(0.05f, params.concavity_branch_min_arm_length);
  const float angle_threshold = params.concavity_branch_angle_deg;
  std::vector<std::pair<V2, V2>> cuts;

  for (std::vector<PointId>& loop : loops) {
    const int n = static_cast<int>(loop.size());
    std::vector<V2> p(n);
    for (int i = 0; i < n; ++i) p[i] = project(loop[i]);
    double area2 = 0.0;
    for (int i = 0; i < n; ++i) {
      const V2 a = p[i];
      const V2 b = p[(i + 1) % n];
      area2 += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    if (area2 < 0.0) {
      std::reverse(loop.begin(), loop.end());
      for (int i = 0; i < n; ++i) p[i] = project(loop[i]);
    }

    std::vector<float> arc(n + 1, 0.0f);
    for (int i = 0; i < n; ++i) arc[i + 1] = arc[i] + norm2(sub(p[(i + 1) % n], p[i]));
    const float total = arc[n];
    if (!(total > 1e-6f)) continue;
    const auto atArc = [&](float target) {
      float x = std::fmod(target, total);
      if (x < 0.0f) x += total;
      int i = 0;
      while (i < n && arc[i + 1] < x) ++i;
      if (i >= n) i = n - 1;
      const float seg = arc[i + 1] - arc[i];
      const float f = seg > 1e-12f ? (x - arc[i]) / seg : 0.0f;
      return V2{p[i].x + (p[(i + 1) % n].x - p[i].x) * f,
                p[i].y + (p[(i + 1) % n].y - p[i].y) * f};
    };

    struct Candidate {
      int index;
      float turn;
    };
    std::vector<Candidate> candidates;
    for (int i = 0; i < n; ++i) {
      const V2 back = atArc(arc[i] - arm);
      const V2 forward = atArc(arc[i] + arm);
      const V2 a = sub(p[i], back);
      const V2 b = sub(forward, p[i]);
      if (norm2(a) < 1e-6f || norm2(b) < 1e-6f) continue;
      const float turn = std::atan2(cross2(a, b), dot2(a, b)) * 180.0f / kPi;
      if (turn < -angle_threshold) candidates.push_back(Candidate{i, turn});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& x, const Candidate& y) { return x.turn < y.turn; });
    std::vector<char> suppressed(n, 0);
    std::vector<Candidate> kept;
    for (const Candidate& c : candidates) {
      if (suppressed[c.index]) continue;
      kept.push_back(c);
      for (int j = 0; j < n; ++j) {
        float d = std::fabs(arc[j] - arc[c.index]);
        d = std::min(d, total - d);
        if (d < arm) suppressed[j] = 1;
      }
    }

    for (const Candidate& c : kept) {
      const int i = c.index;
      const V2 a = normalized(sub(p[i], atArc(arc[i] - 1e-3f)));
      const V2 b = normalized(sub(atArc(arc[i] + 1e-3f), p[i]));
      const V2 na{-a.y, a.x};
      const V2 nb{-b.y, b.x};
      V2 direction = normalized(V2{na.x + nb.x, na.y + nb.y});
      if (norm2(direction) < 1e-9f) continue;
      const V2 origin = p[i];
      float best = std::numeric_limits<float>::infinity();
      for (const auto& e : boundary) {
        if (e.first == loop[i] || e.second == loop[i]) continue;
        const V2 r = project(e.first);
        const V2 s = project(e.second);
        const V2 rs = sub(s, r);
        const float den = cross2(direction, rs);
        if (std::fabs(den) < 1e-12f) continue;
        const V2 rp = sub(r, origin);
        const float t = cross2(rp, rs) / den;
        const float w = cross2(rp, direction) / den;
        if (t > 1e-4f && w >= -1e-4f && w <= 1.0f + 1e-4f && t < best) best = t;
      }
      if (best < std::numeric_limits<float>::infinity() && best > 1e-3f) {
        cuts.push_back({origin, V2{origin.x + direction.x * best, origin.y + direction.y * best}});
      }
    }
  }
  if (cuts.empty()) return {ids};

  std::set<std::pair<PointId, PointId>> blocked;
  for (const auto& cut : cuts) {
    for (TriangleId t : ids) {
      for (const TriangleEdge& e : mesh.triangle(t).edges) {
        const TriangleId u = e.neighbor;
        if (u == kInvalidId || !inreg[u] || u < t) continue;
        if (segmentsCross(cut.first, cut.second, project(e.v0), project(e.v1))) {
          blocked.insert({std::min(e.v0, e.v1), std::max(e.v0, e.v1)});
        }
      }
    }
  }
  if (blocked.empty()) return {ids};
  return edgeComponents(mesh, ids, &blocked);
}

// ---------------------------------------------------------------------------
// Concave (reflex-vertex chord) splitting.
//
// The region boundary is extracted as ordered closed loops and projected onto
// the region's principal plane. Mesh stair-stepping is suppressed by measuring
// the boundary turn over an arc of concavity_branch_min_arm_length and keeping
// only the strongest reflex vertex of each cluster. At a significant reflex
// vertex the two boundary tangents (the incoming edge continued, the outgoing
// edge reversed) are extended into the region; the one that enters the interior
// is followed to its nearest crossing of a non-adjacent boundary edge, yielding
// a chord from the reflex vertex to the opposite wall. Blocking the
// triangle-dual edges a chord truly separates cuts the region into blocks.
// Blocks smaller than min_side_area are folded into the adjacent main block
// they share the longest boundary with (or the nearest main block when they
// touch none), so the fine cuts around a thin limb leave no slivers.
// ---------------------------------------------------------------------------

bool pointInPolygon(const std::vector<V2>& poly, V2 p) {
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const V2 a = poly[i];
    const V2 b = poly[j];
    if (((a.y > p.y) != (b.y > p.y)) &&
        (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
      inside = !inside;
    }
  }
  return inside;
}

int orient2(V2 a, V2 b, V2 c) {
  const float v = cross2(sub(b, a), sub(c, a));
  if (v > 1e-9f) return 1;
  if (v < -1e-9f) return -1;
  return 0;
}

bool onSegment(V2 a, V2 b, V2 p) {
  return std::min(a.x, b.x) - 1e-6f <= p.x && p.x <= std::max(a.x, b.x) + 1e-6f &&
         std::min(a.y, b.y) - 1e-6f <= p.y && p.y <= std::max(a.y, b.y) + 1e-6f &&
         orient2(a, b, p) == 0;
}

// Crossing or touching: a chord endpoint that merely grazes a dual edge still
// has to block it, otherwise the cut leaks through the shared vertex.
bool segmentsIntersectOrTouch(V2 p1, V2 p2, V2 q1, V2 q2) {
  const int o1 = orient2(p1, p2, q1);
  const int o2 = orient2(p1, p2, q2);
  const int o3 = orient2(q1, q2, p1);
  const int o4 = orient2(q1, q2, p2);
  if (o1 != o2 && o3 != o4) return true;
  return onSegment(p1, p2, q1) || onSegment(p1, p2, q2) ||
         onSegment(q1, q2, p1) || onSegment(q1, q2, p2);
}

// Fold every block smaller than min_area into an adjacent main block, choosing
// the neighbour with the longest shared boundary (ties resolved by id). A
// fragment whose only neighbours are fragments is attached to the nearest main
// block instead.
std::vector<std::vector<TriangleId>> mergeFragments(
    const TriangleMesh& mesh, const std::vector<TriangleId>& ids,
    std::vector<std::vector<TriangleId>> groups, float min_area) {
  if (groups.size() <= 1u) return groups;
  std::vector<int> owner(mesh.size(), -1);
  for (int g = 0; g < static_cast<int>(groups.size()); ++g) {
    for (TriangleId t : groups[g]) owner[t] = g;
  }
  std::vector<double> mass(groups.size(), 0.0);
  std::vector<Eigen::Vector3d> moment(groups.size(), Eigen::Vector3d::Zero());
  for (int g = 0; g < static_cast<int>(groups.size()); ++g) {
    for (TriangleId t : groups[g]) {
      const double a = mesh.triangle(t).area;
      mass[g] += a;
      moment[g] += a * mesh.triangle(t).centroid.cast<double>();
    }
  }
  std::map<std::pair<int, int>, double> shared;
  for (TriangleId t : ids) {
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      const TriangleId u = e.neighbor;
      if (u == kInvalidId || owner[u] < 0 || owner[u] == owner[t]) continue;
      const int a = std::min(owner[t], owner[u]);
      const int b = std::max(owner[t], owner[u]);
      shared[{a, b}] += (mesh.vertices()[e.v0] - mesh.vertices()[e.v1]).norm();
    }
  }
  Dsu dsu;
  dsu.reset(static_cast<int>(groups.size()));
  std::vector<int> order(groups.size());
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = static_cast<int>(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    if (mass[a] != mass[b]) return mass[a] < mass[b];
    return a < b;
  });
  for (int f : order) {
    const int rf = dsu.find(f);
    if (rf != f || mass[rf] >= min_area) continue;
    std::map<int, double> neighbour;
    for (const auto& kv : shared) {
      if (kv.first.first != f && kv.first.second != f) continue;
      const int other = kv.first.first == f ? kv.first.second : kv.first.first;
      const int ro = dsu.find(other);
      if (ro == rf) continue;
      neighbour[ro] += kv.second;
    }
    int target = -1;
    double best = -1.0;
    for (const auto& kv : neighbour) {
      if (mass[kv.first] < min_area) continue;
      const bool better = kv.second > best + 1e-9 ||
                          (std::fabs(kv.second - best) <= 1e-9 &&
                           (target < 0 || kv.first < target));
      if (better) {
        best = kv.second;
        target = kv.first;
      }
    }
    if (target < 0) {
      double best_dist = std::numeric_limits<double>::infinity();
      Eigen::Vector3d fc = Eigen::Vector3d::Zero();
      if (mass[rf] > 0.0) fc = moment[rf] / mass[rf];
      for (int r = 0; r < static_cast<int>(groups.size()); ++r) {
        if (dsu.find(r) != r || r == rf || mass[r] < min_area) continue;
        const double dist = (fc - moment[r] / mass[r]).norm();
        if (dist < best_dist) {
          best_dist = dist;
          target = r;
        }
      }
    }
    if (target < 0) continue;
    dsu.unite(rf, target);
    const int root = dsu.find(rf);
    mass[root] = mass[rf] + mass[target];
    moment[root] = moment[rf] + moment[target];
  }
  std::map<int, std::vector<TriangleId>> merged;
  for (int g = 0; g < static_cast<int>(groups.size()); ++g) {
    std::vector<TriangleId>& dst = merged[dsu.find(g)];
    dst.insert(dst.end(), groups[g].begin(), groups[g].end());
  }
  std::vector<std::vector<TriangleId>> out;
  out.reserve(merged.size());
  for (auto& kv : merged) {
    std::sort(kv.second.begin(), kv.second.end());
    out.push_back(std::move(kv.second));
  }
  return out;
}

std::vector<std::vector<TriangleId>> splitConcaveOne(
    const TriangleMesh& mesh, const std::vector<TriangleId>& ids,
    const RegionParams& params) {
  if (ids.size() < 8u) return {ids};

  std::vector<char> inreg(mesh.size(), 0);
  for (TriangleId t : ids) inreg[t] = 1;

  std::set<std::pair<PointId, PointId>> boundary;
  for (TriangleId t : ids) {
    for (const TriangleEdge& e : mesh.triangle(t).edges) {
      if (e.neighbor != kInvalidId && inreg[e.neighbor]) continue;
      boundary.insert({std::min(e.v0, e.v1), std::max(e.v0, e.v1)});
    }
  }
  if (boundary.size() < 4u) return {ids};

  // Best-fit plane of the region for a stable 2D projection.
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double total_area = 0.0;
  for (TriangleId t : ids) {
    const TriangleNode& n = mesh.triangle(t);
    normal += n.normal.cast<double>() * n.area;
    center += n.centroid.cast<double>() * n.area;
    total_area += n.area;
  }
  if (!(total_area > 0.0)) return {ids};
  if (normal.norm() < 1e-9) normal = Eigen::Vector3d::UnitZ();
  normal.normalize();
  center /= total_area;
  Eigen::Vector3d u = std::fabs(normal.x()) < 0.9 ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
  u = (u - normal * normal.dot(u)).normalized();
  const Eigen::Vector3d v = normal.cross(u);
  const auto projectVec = [&](const Eigen::Vector3f& point) {
    const Eigen::Vector3d q = point.cast<double>() - center;
    return V2{static_cast<float>(q.dot(u)), static_cast<float>(q.dot(v))};
  };
  const auto project = [&](PointId id) { return projectVec(mesh.vertices()[id]); };

  std::map<PointId, std::vector<PointId>> adjacency;
  for (const auto& e : boundary) {
    adjacency[e.first].push_back(e.second);
    adjacency[e.second].push_back(e.first);
  }

  const float arm = std::max(0.05f, params.concavity_branch_min_arm_length);
  const float angle_threshold = params.concavity_branch_angle_deg;
  const float eps = 1e-3f;
  std::vector<std::pair<V2, V2>> chords;

  // Walk the boundary graph into closed loops, consuming every undirected edge
  // exactly once. Marking edges (not just the previous vertex) terminates a
  // walk where several loops meet at one pinched vertex.
  using BoundaryEdge = std::pair<PointId, PointId>;
  std::set<BoundaryEdge> used_edges;
  for (const BoundaryEdge& seed : boundary) {
    if (used_edges.count(seed) != 0) continue;
    const PointId start = seed.first;
    std::vector<PointId> loop;
    PointId current = start;
    while (true) {
      loop.push_back(current);
      PointId next = kInvalidId;
      for (PointId candidate : adjacency[current]) {
        const BoundaryEdge key{std::min(current, candidate), std::max(current, candidate)};
        if (used_edges.count(key) == 0) {
          used_edges.insert(key);
          next = candidate;
          break;
        }
      }
      if (next == kInvalidId || next == start) break;
      current = next;
    }
    if (loop.size() < 4u) continue;

    const int n = static_cast<int>(loop.size());
    std::vector<V2> p(n);
    for (int i = 0; i < n; ++i) p[i] = project(loop[i]);
    double signed_area2 = 0.0;
    for (int i = 0; i < n; ++i) {
      signed_area2 += static_cast<double>(p[i].x) * p[(i + 1) % n].y -
                      static_cast<double>(p[(i + 1) % n].x) * p[i].y;
    }
    if (signed_area2 < 0.0) {
      std::reverse(loop.begin(), loop.end());
      for (int i = 0; i < n; ++i) p[i] = project(loop[i]);
    }

    std::vector<float> arc(n + 1, 0.0f);
    for (int i = 0; i < n; ++i) arc[i + 1] = arc[i] + norm2(sub(p[(i + 1) % n], p[i]));
    const float total = arc[n];
    if (!(total > 1e-6f)) continue;
    const auto atArc = [&](float target) {
      float x = std::fmod(target, total);
      if (x < 0.0f) x += total;
      int i = 0;
      while (i < n && arc[i + 1] < x) ++i;
      if (i >= n) i = n - 1;
      const float seg = arc[i + 1] - arc[i];
      const float f = seg > 1e-12f ? (x - arc[i]) / seg : 0.0f;
      return V2{p[i].x + (p[(i + 1) % n].x - p[i].x) * f,
                p[i].y + (p[(i + 1) % n].y - p[i].y) * f};
    };

    struct Candidate {
      int index;
      float turn;
    };
    std::vector<Candidate> candidates;
    for (int i = 0; i < n; ++i) {
      const V2 back = atArc(arc[i] - arm);
      const V2 forward = atArc(arc[i] + arm);
      const V2 a = sub(p[i], back);
      const V2 b = sub(forward, p[i]);
      if (norm2(a) < 1e-6f || norm2(b) < 1e-6f) continue;
      const float turn = std::atan2(cross2(a, b), dot2(a, b)) * 180.0f / kPi;
      if (turn <= -angle_threshold) candidates.push_back(Candidate{i, turn});
    }
    // Keep the strongest reflex corner of each arc-length cluster so the mesh
    // staircase contributes one cut, not one per step.
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& x, const Candidate& y) {
                if (x.turn != y.turn) return x.turn < y.turn;
                return x.index < y.index;
              });
    std::vector<char> suppressed(n, 0);
    std::vector<Candidate> kept;
    for (const Candidate& c : candidates) {
      if (suppressed[c.index]) continue;
      kept.push_back(c);
      for (int j = 0; j < n; ++j) {
        float d = std::fabs(arc[j] - arc[c.index]);
        d = std::min(d, total - d);
        if (d < arm) suppressed[j] = 1;
      }
    }

    for (const Candidate& c : kept) {
      const int i = c.index;
      const V2 a = normalized(sub(atArc(arc[i] - arm), atArc(arc[i] - 2.0f * arm)));
      const V2 b = normalized(sub(atArc(arc[i] + 2.0f * arm), atArc(arc[i] + arm)));
      const V2 directions[2] = {a, V2{-b.x, -b.y}};
      for (const V2& dir : directions) {
        if (norm2(dir) < 1e-9f) continue;
        const V2 epsilon_point{p[i].x + dir.x * eps, p[i].y + dir.y * eps};
        if (!pointInPolygon(p, epsilon_point)) continue;
        float best = std::numeric_limits<float>::infinity();
        for (int e = 0; e < n; ++e) {
          if (e == i || e == (i + n - 1) % n) continue;  // incident edges
          const V2 r = p[e];
          const V2 s = p[(e + 1) % n];
          const V2 rs = sub(s, r);
          const float den = cross2(dir, rs);
          if (std::fabs(den) < 1e-12f) continue;
          const V2 rp = sub(r, p[i]);
          const float t = cross2(rp, rs) / den;
          const float w = cross2(rp, dir) / den;
          if (t > 2.0f * eps && w >= -1e-4f && w <= 1.0f + 1e-4f && t < best) best = t;
        }
        if (best < std::numeric_limits<float>::infinity()) {
          chords.push_back({p[i], V2{p[i].x + dir.x * best, p[i].y + dir.y * best}});
        }
      }
    }
  }
  if (chords.empty()) return {ids};

  // Drop chords that are near-parallel and lie on the same support line.
  std::vector<std::pair<V2, V2>> unique;
  for (const auto& chord : chords) {
    const V2 d = normalized(sub(chord.second, chord.first));
    bool duplicate = false;
    for (const auto& other : unique) {
      const V2 od = normalized(sub(other.second, other.first));
      if (std::fabs(cross2(d, od)) > 0.05f) continue;
      const float da = std::fabs(cross2(sub(chord.first, other.first), od));
      const float db = std::fabs(cross2(sub(chord.second, other.first), od));
      if (da < 0.05f && db < 0.05f) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) unique.push_back(chord);
  }
  if (unique.empty()) return {ids};

  // Block the dual-graph edges each chord truly separates.
  std::set<std::pair<PointId, PointId>> blocked;
  for (const auto& chord : unique) {
    for (TriangleId t : ids) {
      for (const TriangleEdge& e : mesh.triangle(t).edges) {
        const TriangleId other = e.neighbor;
        if (other == kInvalidId || !inreg[other] || other < t) continue;
        const V2 ca = projectVec(mesh.triangle(t).centroid);
        const V2 cb = projectVec(mesh.triangle(other).centroid);
        if (segmentsIntersectOrTouch(chord.first, chord.second, ca, cb)) {
          blocked.insert({std::min(e.v0, e.v1), std::max(e.v0, e.v1)});
        }
      }
    }
  }
  if (blocked.empty()) return {ids};

  std::vector<std::vector<TriangleId>> blocks = edgeComponents(mesh, ids, &blocked);
  return mergeFragments(mesh, ids, std::move(blocks), params.min_side_area);
}

std::vector<SurfaceRegion> normalize(const TriangleMesh& mesh,
                                     std::vector<std::vector<TriangleId>> groups,
                                     float ramp_slope_rad) {
  std::vector<SurfaceRegion> out;
  for (std::vector<TriangleId>& ids : groups) {
    std::sort(ids.begin(), ids.end());
    for (std::vector<TriangleId>& comp : edgeComponents(mesh, ids, nullptr)) {
      if (comp.empty()) continue;
      out.push_back(makeRegionStats(static_cast<RegionId>(out.size()), mesh, comp, ramp_slope_rad));
    }
  }
  return out;
}

}  // namespace

Result<std::vector<SurfaceRegion>> RegionSplitter::split(
    const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
    const RegionParams& params) const {
  const float ramp_slope_rad = toRad(params.ramp_slope_deg);
  std::vector<std::vector<TriangleId>> groups;
  for (const SurfaceRegion& region : regions) {
    if (region.triangle_ids.empty()) continue;
    // Bottlenecks run first and alone. A region a bottleneck already separated
    // must not have its artificial boundaries re-cut by the concave pass, so
    // only originals that survive the bottleneck as one piece are decomposed at
    // their reflex corners.
    std::vector<std::vector<TriangleId>> pieces =
        splitBottlenecksOne(mesh, region.triangle_ids, params);
    if (pieces.size() > 1u) {
      for (std::vector<TriangleId>& piece : pieces) groups.push_back(std::move(piece));
    } else {
      for (std::vector<TriangleId>& piece : splitConcaveOne(mesh, region.triangle_ids, params)) {
        groups.push_back(std::move(piece));
      }
    }
  }
  if (groups.empty()) {
    return Status::Error(ErrorCode::kRegionSplitFailed, "region split produced no regions");
  }
  std::vector<SurfaceRegion> concave_regions = normalize(mesh, std::move(groups), ramp_slope_rad);
  auto ramps = splitRamps(mesh, concave_regions, params);
  if (!ramps.ok()) return ramps;
  if (ramps.value().empty()) {
    return Status::Error(ErrorCode::kRegionSplitFailed, "region split produced no regions");
  }
  return ramps;
}

Result<std::vector<SurfaceRegion>> RegionSplitter::splitBottlenecks(
    const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
    const RegionParams& params) const {
  std::vector<std::vector<TriangleId>> groups;
  for (const SurfaceRegion& region : regions) {
    if (region.triangle_ids.empty()) continue;
    for (std::vector<TriangleId>& g : splitBottlenecksOne(mesh, region.triangle_ids, params)) {
      groups.push_back(std::move(g));
    }
  }
  if (groups.empty()) {
    return Status::Error(ErrorCode::kRegionSplitFailed, "bottleneck split produced no regions");
  }
  return normalize(mesh, std::move(groups), toRad(params.ramp_slope_deg));
}

Result<std::vector<SurfaceRegion>> RegionSplitter::splitConcave(
    const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
    const RegionParams& params) const {
  std::vector<std::vector<TriangleId>> groups;
  for (const SurfaceRegion& region : regions) {
    if (region.triangle_ids.empty()) continue;
    for (std::vector<TriangleId>& g : splitConcaveOne(mesh, region.triangle_ids, params)) {
      groups.push_back(std::move(g));
    }
  }
  if (groups.empty()) {
    return Status::Error(ErrorCode::kRegionSplitFailed, "concave split produced no regions");
  }
  return normalize(mesh, std::move(groups), toRad(params.ramp_slope_deg));
}

Result<std::vector<SurfaceRegion>> RegionSplitter::splitRamps(
    const TriangleMesh& mesh, const std::vector<SurfaceRegion>& regions,
    const RegionParams& params) const {
  const float ramp_slope_rad = toRad(params.ramp_slope_deg);
  std::vector<std::vector<TriangleId>> groups;
  for (const SurfaceRegion& region : regions) {
    if (region.triangle_ids.empty()) continue;
    // Split a mixed region at its flat/ramp boundaries using the triangle slope
    // itself (not the region mean), otherwise a platform and its ramp could
    // never be separated.
    std::vector<char> inreg(mesh.size(), 0);
    for (TriangleId t : region.triangle_ids) inreg[t] = 1;
    std::vector<char> seen(mesh.size(), 0);
    for (TriangleId seed : region.triangle_ids) {
      if (seen[seed]) continue;
      seen[seed] = 1;
      const bool seed_ramp = mesh.triangle(seed).slope > ramp_slope_rad;
      std::vector<TriangleId> component{seed};
      std::vector<TriangleId> stack{seed};
      while (!stack.empty()) {
        const TriangleId t = stack.back();
        stack.pop_back();
        for (const TriangleEdge& e : mesh.triangle(t).edges) {
          const TriangleId u = e.neighbor;
          if (u == kInvalidId || !inreg[u] || seen[u]) continue;
          if ((mesh.triangle(u).slope > ramp_slope_rad) != seed_ramp) continue;
          seen[u] = 1;
          component.push_back(u);
          stack.push_back(u);
        }
      }
      std::sort(component.begin(), component.end());
      groups.push_back(std::move(component));
    }
  }
  if (groups.empty()) {
    return Status::Error(ErrorCode::kRegionSplitFailed, "ramp split produced no regions");
  }
  return normalize(mesh, std::move(groups), ramp_slope_rad);
}

}  // namespace topmap
