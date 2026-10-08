#include "topmap_surface/traversability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "topmap_core/geometry.hpp"

namespace topmap {
namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

// Multi-source Dijkstra over the shared-edge adjacency graph, restricted to
// `allowed` triangles. `dist` is seeded by the caller and updated in place.
void relaxDistances(const TriangleMesh& mesh, const std::vector<char>& allowed,
                    std::vector<float>& dist) {
  using Item = std::pair<float, TriangleId>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    if (allowed[id] && dist[id] < kInf) queue.push({dist[id], id});
  }
  while (!queue.empty()) {
    const Item item = queue.top();
    queue.pop();
    const TriangleId id = item.second;
    if (item.first > dist[id]) continue;  // stale entry
    for (const TriangleEdge& e : mesh.triangle(id).edges) {
      if (e.neighbor == kInvalidId || !allowed[e.neighbor]) continue;
      const float step = (mesh.triangle(id).centroid - mesh.triangle(e.neighbor).centroid).norm();
      const float candidate = item.first + step;
      if (candidate < dist[e.neighbor]) {
        dist[e.neighbor] = candidate;
        queue.push({candidate, e.neighbor});
      }
    }
  }
}

float distanceToEdge(const TriangleMesh& mesh, const TriangleNode& t, const TriangleEdge& e) {
  const std::vector<Eigen::Vector3f>& vertices = mesh.vertices();
  if (e.v0 == kInvalidId || e.v1 == kInvalidId) return kInf;
  if (e.v0 < 0 || e.v1 < 0 || e.v0 >= static_cast<PointId>(vertices.size()) ||
      e.v1 >= static_cast<PointId>(vertices.size())) {
    return kInf;
  }
  return segmentDistance(t.centroid, vertices[e.v0], vertices[e.v1]);
}

}  // namespace

bool TraversabilityEvaluator::isTriangleTraversable(const TriangleNode& t,
                                                    const TraversabilityParams& p) {
  return t.slope <= p.max_slope_rad() && t.roughness <= p.max_roughness &&
         t.curvature <= p.max_surface_curvature && t.area >= p.min_triangle_area;
}

bool TraversabilityEvaluator::hasRequiredClearance(const TriangleNode& t,
                                                   const TraversabilityParams& p) {
  return t.min_clearance >= p.required_clearance;
}

bool TraversabilityEvaluator::isRobotPlaceable(const TriangleNode& t,
                                               const TraversabilityParams& p) {
  return isTriangleTraversable(t, p) && hasRequiredClearance(t, p);
}

float TraversabilityEvaluator::heightJump(const TriangleNode& a, const TriangleNode& b) {
  return std::max(std::fabs(pointPlaneDistance(a.centroid, b.normal, b.centroid)),
                  std::fabs(pointPlaneDistance(b.centroid, a.normal, a.centroid)));
}

bool TraversabilityEvaluator::isEdgeTraversable(const TriangleNode& a, const TriangleNode& b,
                                                const TraversabilityParams& p) {
  if (angleBetween(a.normal, b.normal) > p.max_normal_diff_rad()) return false;
  if (heightJump(a, b) > p.max_step_height) return false;
  // R1-b: flat <-> ramp transition exemption. The normal-difference and step
  // checks above already constrain geometric continuity, so a slope-difference
  // violation on its own does not break a continuous transition.
  if (std::fabs(a.slope - b.slope) > p.max_slope_diff_rad() && !p.allow_slope_transition) {
    return false;
  }
  return true;
}

void TraversabilityEvaluator::apply(TriangleMesh& mesh, const TraversabilityParams& p) {
  std::vector<TriangleNode>& triangles = mesh.triangles();
  for (TriangleNode& t : triangles) t.traversable = isTriangleTraversable(t, p);
  for (TriangleNode& t : triangles) {
    for (TriangleEdge& e : t.edges) {
      if (e.neighbor == kInvalidId) continue;
      e.traversable = t.traversable && triangles[e.neighbor].traversable &&
                      isEdgeTraversable(t, triangles[e.neighbor], p);
    }
  }

  // R8: min_clearance is the single definition "geodesic distance to the
  // nearest non-traversable triangle or mesh boundary". The source set must
  // contain BOTH (a) every non-traversable triangle, and (b) every traversable
  // triangle that owns a boundary edge. Omitting (b) makes a fully traversable
  // plane report infinite clearance.
  const std::vector<char> allowed(triangles.size(), 1);
  std::vector<float> dist(triangles.size(), kInf);
  for (const TriangleNode& t : triangles) {
    if (!t.traversable) {
      dist[t.id] = 0.0f;
      continue;
    }
    float best = kInf;
    for (const TriangleEdge& e : t.edges) {
      if (e.neighbor != kInvalidId) continue;  // interior edge: not a mesh boundary
      best = std::min(best, distanceToEdge(mesh, t, e));
    }
    dist[t.id] = best;
  }
  relaxDistances(mesh, allowed, dist);
  for (TriangleNode& t : triangles) t.min_clearance = dist[t.id];
}

// R8: geodesic distance from each triangle's centroid to the boundary of the
// given Region. This is intentionally a different quantity from
// TriangleNode.min_clearance: it depends on the Region partition. Region
// triangles not part of `region_triangles` get 0.
std::vector<float> regionBoundaryDistance(const TriangleMesh& mesh,
                                          const std::vector<TriangleId>& region_triangles) {
  const std::size_t count = mesh.size();
  std::vector<float> result(count, 0.0f);
  std::vector<char> allowed(count, 0);
  for (TriangleId id : region_triangles) {
    if (id >= 0 && id < static_cast<TriangleId>(count)) allowed[id] = 1;
  }

  std::vector<float> dist(count, kInf);
  for (TriangleId id : region_triangles) {
    if (id < 0 || id >= static_cast<TriangleId>(count)) continue;
    const TriangleNode& t = mesh.triangle(id);
    float best = kInf;
    for (const TriangleEdge& e : t.edges) {
      if (e.neighbor != kInvalidId && allowed[e.neighbor]) continue;  // interior to the Region
      best = std::min(best, distanceToEdge(mesh, t, e));
    }
    dist[id] = best;
  }
  relaxDistances(mesh, allowed, dist);
  for (TriangleId id : region_triangles) {
    if (id < 0 || id >= static_cast<TriangleId>(count)) continue;
    result[id] = dist[id] < kInf ? dist[id] : 0.0f;
  }
  return result;
}

}  // namespace topmap
