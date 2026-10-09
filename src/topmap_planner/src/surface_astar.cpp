#include "topmap_planner/surface_astar.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/geometry.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
// Adjacent duplicate vertices closer than this are collapsed when the per
// segment polylines are concatenated.
constexpr float kDuplicateEpsilon = 1e-6f;

bool IsFinite(float value) { return std::isfinite(value); }

bool IsFinite(const Eigen::Vector3f& point) {
  return IsFinite(point.x()) && IsFinite(point.y()) && IsFinite(point.z());
}

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

Status SurfaceFailure(std::string message) {
  return Status::Error(ErrorCode::kSurfaceSearchFailed, std::move(message));
}

// Bias graph search away from the hard clearance boundary. The multiplier is
// always >= 1, so the Euclidean heuristic remains admissible. Safety is still
// enforced independently by isRobotPlaceable; this term only chooses a more
// useful centerline among otherwise traversable alternatives.
float ClearanceCostMultiplier(float clearance, const TraversabilityParams& trav) {
  if (!(trav.required_clearance > 0.0f)) return 1.0f;
  const float floor = std::max(0.1f * trav.required_clearance, 1e-3f);
  const float margin = std::max(clearance - trav.required_clearance, floor);
  return 1.0f + trav.required_clearance / margin;
}

// R10/R6: every TraversabilityParams field that the placement and edge checks
// consume must be finite and non-negative. A negative clearance or a NaN step
// height silently turns the hard constraint into a no-op (or a no-edge graph),
// so reject it structurally before touching the mesh.
Status ValidateTraversabilityParams(const TraversabilityParams& trav) {
  struct Field {
    float value;
    const char* name;
  };
  const Field fields[] = {
      {trav.max_slope_deg, "max_slope_deg"},
      {trav.max_normal_diff_deg, "max_normal_diff_deg"},
      {trav.max_slope_diff_deg, "max_slope_diff_deg"},
      {trav.max_step_height, "max_step_height"},
      {trav.max_roughness, "max_roughness"},
      {trav.max_surface_curvature, "max_surface_curvature"},
      {trav.min_triangle_area, "min_triangle_area"},
      {trav.required_clearance, "required_clearance"},
  };
  for (const Field& field : fields) {
    if (!IsFinite(field.value) || field.value < 0.0f) {
      return Invalid(std::string("traversability.") + field.name +
                     " must be finite and non-negative");
    }
  }
  return Status::Ok();
}

// Closest point on the triangle (a,b,c) to p, clamped to the triangle. Used to
// keep the exact waypoint projection rather than a bare centroid so the portal
// center order survives concatenation.
Eigen::Vector3f ClosestPointOnTriangle(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                                       const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f ab = b - a, ac = c - a, ap = p - a;
  const float d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) return a;

  const Eigen::Vector3f bp = p - b;
  const float d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) return b;

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const Eigen::Vector3f cp = p - c;
  const float d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) return c;

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float w = d2 / (d2 - d6);
    return a + w * ac;
  }

  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + w * (c - b);
  }

  const float denom = 1.0f / (va + vb + vc);
  const float v = vb * denom, w = vc * denom;
  return a + ab * v + ac * w;
}

Eigen::Vector3f ClosestPointOnTriangle(const TriangleMesh& mesh, TriangleId id,
                                       const Eigen::Vector3f& p) {
  const TriangleNode& triangle = mesh.triangle(id);
  return ClosestPointOnTriangle(p, mesh.vertices()[triangle.vertex_ids[0]],
                                mesh.vertices()[triangle.vertex_ids[1]],
                                mesh.vertices()[triangle.vertex_ids[2]]);
}

// T24-internal locator (R6 + R10). It is deliberately self-contained: T24 must
// compile before T26 introduces SafetyValidator, so no SafetyValidator symbol is
// referenced here. Candidates must be inside the corridor triangle set, be
// robot-placeable, be roughly horizontal and face the query point.
Result<TriangleId> LocateInCorridor(const TriangleMesh& mesh, const std::vector<char>& in_corridor,
                                    const Eigen::Vector3f& point, const TraversabilityParams& trav,
                                    float max_distance) {
  const float max_normal_rad = trav.max_normal_diff_rad();
  const float back_face_tolerance = trav.max_step_height;
  TriangleId best = kInvalidId;
  float best_distance = kInf;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    if (!in_corridor[static_cast<std::size_t>(id)]) continue;
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) continue;
    if (angleBetween(triangle.normal, Eigen::Vector3f::UnitZ()) > max_normal_rad) continue;
    if (triangle.normal.dot(point - triangle.centroid) < -back_face_tolerance) continue;
    const float distance =
        pointTriangleDistance(point, mesh.vertices()[triangle.vertex_ids[0]],
                              mesh.vertices()[triangle.vertex_ids[1]],
                              mesh.vertices()[triangle.vertex_ids[2]]);
    // Ascending ids plus a strict `<` make ties deterministic (smallest id wins).
    if (best == kInvalidId || distance < best_distance) {
      best = id;
      best_distance = distance;
    }
  }
  if (best == kInvalidId || best_distance > max_distance) {
    return SurfaceFailure("waypoint cannot be projected into the corridor");
  }
  return best;
}

struct QueueNode {
  float f = 0.0f;
  float g = 0.0f;
  TriangleId id = kInvalidId;
};

struct QueueGreater {
  bool operator()(const QueueNode& a, const QueueNode& b) const {
    // Deterministic tie-break: f, then g, then TriangleId.
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g > b.g;
    return a.id > b.id;
  }
};

Result<std::vector<TriangleId>> SearchSegment(const TriangleMesh& mesh,
                                              const std::vector<char>& in_corridor, TriangleId from,
                                              TriangleId to, const TraversabilityParams& trav,
                                              float heuristic_scale,
                                              std::vector<char>* expanded_seen,
                                              int* expanded_nodes) {
  const std::size_t count = mesh.size();
  const auto in_bounds = [count](TriangleId id) {
    return id >= 0 && id < static_cast<TriangleId>(count);
  };

  // LocateInCorridor already returns a placeable triangle, but SearchSegment
  // never trusts its inputs: from/to must be in range, inside the corridor and
  // themselves robot-placeable, otherwise the seed itself could sit on a
  // clearance-violating triangle (R10).
  if (!in_bounds(from) || !in_bounds(to)) {
    return Invalid("surface A* segment endpoint is out of range");
  }
  if (!in_corridor[static_cast<std::size_t>(from)] ||
      !in_corridor[static_cast<std::size_t>(to)]) {
    return SurfaceFailure("surface A* segment endpoint is outside the corridor");
  }
  if (!TraversabilityEvaluator::isRobotPlaceable(mesh.triangle(from), trav) ||
      !TraversabilityEvaluator::isRobotPlaceable(mesh.triangle(to), trav)) {
    return SurfaceFailure("surface A* segment endpoint is not robot-placeable");
  }
  if (from == to) return std::vector<TriangleId>{from};

  std::vector<float> g(count, kInf);
  std::vector<TriangleId> parent(count, kInvalidId);
  std::vector<char> closed(count, 0);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueGreater> open;

  const auto heuristic = [&](TriangleId id) {
    return heuristic_scale *
           (mesh.triangle(id).centroid - mesh.triangle(to).centroid).norm();
  };

  g[static_cast<std::size_t>(from)] = 0.0f;
  open.push(QueueNode{heuristic(from), 0.0f, from});

  while (!open.empty()) {
    const QueueNode node = open.top();
    open.pop();
    if (closed[static_cast<std::size_t>(node.id)]) continue;
    closed[static_cast<std::size_t>(node.id)] = 1;
    if (expanded_nodes != nullptr &&
        (expanded_seen == nullptr || !(*expanded_seen)[static_cast<std::size_t>(node.id)])) {
      ++(*expanded_nodes);
    }
    if (expanded_seen != nullptr) {
      (*expanded_seen)[static_cast<std::size_t>(node.id)] = 1;
    }
    if (node.id == to) break;

    const TriangleNode& current = mesh.triangle(node.id);
    for (const TriangleEdge& edge : current.edges) {
      const TriangleId next_id = edge.neighbor;
      // Bounds first: a malformed neighbor must never index in_corridor/mesh.
      if (!in_bounds(next_id)) continue;
      if (!in_corridor[static_cast<std::size_t>(next_id)]) continue;
      if (!edge.traversable) continue;
      const TriangleNode& next = mesh.triangle(next_id);
      if (!TraversabilityEvaluator::isRobotPlaceable(next, trav)) continue;
      if (!TraversabilityEvaluator::isEdgeTraversable(current, next, trav)) continue;

      const float geometric_step = (current.centroid - next.centroid).norm();
      const float clearance = std::min(current.min_clearance, next.min_clearance);
      const float step =
          geometric_step * ClearanceCostMultiplier(clearance, trav);
      // A non-finite edge cost must never enter the queue: NaN comparisons make
      // the relaxation and the queue order undefined.
      if (!std::isfinite(step)) continue;
      const float candidate = g[static_cast<std::size_t>(node.id)] + step;
      if (!std::isfinite(candidate)) continue;
      if (candidate < g[static_cast<std::size_t>(next_id)]) {
        g[static_cast<std::size_t>(next_id)] = candidate;
        parent[static_cast<std::size_t>(next_id)] = node.id;
        open.push(QueueNode{candidate + heuristic(next_id), candidate, next_id});
      }
    }
  }

  if (g[static_cast<std::size_t>(to)] == kInf) {
    return SurfaceFailure("no traversable corridor path between adjacent waypoints");
  }

  std::vector<TriangleId> segment;
  for (TriangleId id = to; id != kInvalidId; id = parent[static_cast<std::size_t>(id)]) {
    segment.push_back(id);
  }
  std::reverse(segment.begin(), segment.end());
  return segment;
}

}  // namespace

Result<std::vector<Eigen::Vector3f>> SurfaceAStar::search(
    const TriangleMesh& mesh, const Corridor& corridor,
    const std::vector<Eigen::Vector3f>& waypoints, const TraversabilityParams& trav,
    int* expanded_nodes) const {
  if (expanded_nodes != nullptr) *expanded_nodes = 0;

  if (waypoints.size() < 2u) {
    return Invalid("surface A* needs at least two waypoints");
  }
  for (const Eigen::Vector3f& waypoint : waypoints) {
    if (!IsFinite(waypoint)) return Invalid("surface A* waypoint is not finite");
  }
  const Status trav_status = ValidateTraversabilityParams(trav);
  if (!trav_status.ok()) return trav_status;
  if (mesh.size() == 0) return SurfaceFailure("cannot search an empty mesh");
  if (corridor.triangles.empty()) return SurfaceFailure("corridor has no triangles");

  const std::size_t count = mesh.size();
  std::vector<char> in_corridor(count, 0);
  std::size_t valid_triangles = 0;
  for (std::size_t index = 0; index < corridor.triangles.size(); ++index) {
    const TriangleId id = corridor.triangles[index];
    // A corridor id outside the mesh is malformed input, not a no-op: silently
    // skipping it could hide a caller bug and let a search succeed on a
    // truncated corridor.
    if (id < 0 || id >= static_cast<TriangleId>(count)) {
      return Invalid("corridor.triangles[" + std::to_string(index) +
                     "] is out of range: " + std::to_string(id));
    }
    if (!in_corridor[static_cast<std::size_t>(id)]) {
      in_corridor[static_cast<std::size_t>(id)] = 1;
      ++valid_triangles;
    }
  }
  if (valid_triangles == 0u) return SurfaceFailure("corridor has no valid triangles");

  float heuristic_scale = kInf;
  for (const TriangleId id : corridor.triangles) {
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) continue;
    heuristic_scale =
        std::min(heuristic_scale, ClearanceCostMultiplier(triangle.min_clearance, trav));
  }
  if (!IsFinite(heuristic_scale) || heuristic_scale < 1.0f) heuristic_scale = 1.0f;

  // Map every waypoint before searching so a bad waypoint fails fast and
  // deterministically.
  //
  // Frozen-signature deviation (reported, not hidden): search() only receives
  // TraversabilityParams, so the projection radius has to come from the default
  // PlanParams{}.max_project_radius (0.30 m) instead of the planner's
  // configured value, and the normal gate comes from trav.max_normal_diff_rad()
  // rather than plan.max_project_normal_diff_deg. T27 must thread PlanParams (or
  // the radius) through once this signature is unfrozen.
  const float max_project_distance = PlanParams{}.max_project_radius;
  std::vector<TriangleId> nodes(waypoints.size(), kInvalidId);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    auto located = LocateInCorridor(mesh, in_corridor, waypoints[i], trav, max_project_distance);
    if (!located.ok()) {
      return SurfaceFailure("waypoint " + std::to_string(i) + " (" +
                            std::to_string(waypoints[i].x()) + ", " +
                            std::to_string(waypoints[i].y()) + ", " +
                            std::to_string(waypoints[i].z()) + ") cannot be projected: " +
                            located.status().message);
    }
    nodes[i] = located.value();
  }

  std::vector<Eigen::Vector3f> path;
  // A hierarchical request searches one segment per ordered Portal. Count a
  // triangle once for the whole request so expanded_nodes measures unique
  // search work rather than charging the same node again at every waypoint.
  std::vector<char> expanded_seen(mesh.size(), 0);
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    auto segment = SearchSegment(mesh, in_corridor, nodes[i], nodes[i + 1], trav, heuristic_scale,
                                 &expanded_seen, expanded_nodes);
    if (!segment.ok()) return segment.status();

    std::vector<Eigen::Vector3f> segment_points;
    if (segment.value().size() == 1u) {
      segment_points.push_back(ClosestPointOnTriangle(mesh, nodes[i], waypoints[i]));
      segment_points.push_back(ClosestPointOnTriangle(mesh, nodes[i + 1], waypoints[i + 1]));
    } else {
      segment_points.reserve(segment.value().size());
      for (const TriangleId id : segment.value()) {
        segment_points.push_back(mesh.triangle(id).centroid);
      }
      segment_points.front() = ClosestPointOnTriangle(mesh, nodes[i], waypoints[i]);
      segment_points.back() = ClosestPointOnTriangle(mesh, nodes[i + 1], waypoints[i + 1]);
    }

    // Concatenate the per-segment polylines without ever dropping a waypoint
    // boundary. For every later segment the first point is the projection of
    // the waypoint already emitted as the previous segment's last point, so
    // exactly that one junction duplicate is skipped. Interior duplicates are
    // collapsed, but the segment's last point (the next waypoint projection) is
    // always kept: two distinct waypoints that happen to project to the same
    // point must keep their order in the output.
    std::size_t begin = 0;
    if (i > 0 && !path.empty() &&
        (path.back() - segment_points.front()).norm() <= kDuplicateEpsilon) {
      begin = 1;
    }
    for (std::size_t j = begin; j < segment_points.size(); ++j) {
      const bool is_segment_end = (j + 1 == segment_points.size());
      if (!is_segment_end && !path.empty() &&
          (path.back() - segment_points[j]).norm() <= kDuplicateEpsilon) {
        continue;
      }
      path.push_back(segment_points[j]);
    }
  }

  if (path.size() < 2u) return SurfaceFailure("surface search produced a degenerate path");
  return path;
}

}  // namespace topmap
