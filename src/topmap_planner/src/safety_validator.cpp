#include "topmap_planner/safety_validator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/geometry.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kPi = 3.14159265358979323846f;
// Adjacent samples closer than this collapse when a polyline is deduplicated.
constexpr float kDuplicateEpsilon = 1e-6f;
// Tolerances that absorb float round-off on exactly-at-threshold paths. They are
// far below any physically meaningful margin.
constexpr float kSlopeEpsilonDeg = 1e-3f;
constexpr float kCurvatureEpsilon = 1e-3f;
// The frozen validate() signature removed PlanParams, so the projection radius
// cannot be threaded through. Use the same default SurfaceAStar falls back to
// (documented deviation) instead of inventing a second constant.
const float kProjectionTolerance = PlanParams{}.max_project_radius;
// Boundary-chain crossing tolerance, in metres. A crossing is a geometric
// intersection; this only absorbs float round-off and the second-order miss of a
// centroid chord over a shallow fold.
constexpr double kFenceTolerance = 0.05;

bool IsFinite(float value) { return std::isfinite(value); }

bool IsFinite(const Eigen::Vector3f& point) {
  return IsFinite(point.x()) && IsFinite(point.y()) && IsFinite(point.z());
}

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

// Mirrors SurfaceAStar/PathOptimizer: a negative or NaN traversability field
// silently disables a hard constraint, so reject it structurally.
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

float TriangleDistance(const TriangleMesh& mesh, TriangleId id, const Eigen::Vector3f& p) {
  const TriangleNode& triangle = mesh.triangle(id);
  return pointTriangleDistance(p, mesh.vertices()[triangle.vertex_ids[0]],
                               mesh.vertices()[triangle.vertex_ids[1]],
                               mesh.vertices()[triangle.vertex_ids[2]]);
}

// Nearest triangle of any kind, true 3D distance, smallest id wins ties.
bool NearestAnyTriangle(const TriangleMesh& mesh, const Eigen::Vector3f& p, TriangleId* located,
                        float* distance) {
  TriangleId best = kInvalidId;
  float best_distance = kInf;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const float d = TriangleDistance(mesh, id, p);
    if (!IsFinite(d)) continue;
    if (best == kInvalidId || d < best_distance) {
      best = id;
      best_distance = d;
    }
  }
  if (best == kInvalidId) return false;
  if (located != nullptr) *located = best;
  if (distance != nullptr) *distance = best_distance;
  return true;
}

// Nearest robot-placeable, front-facing triangle within `tolerance`.
bool NearestPlaceableTriangle(const TriangleMesh& mesh, const TraversabilityParams& trav,
                              const Eigen::Vector3f& p, float tolerance, TriangleId* located,
                              float* distance) {
  const float back_face_tolerance = trav.max_step_height;
  TriangleId best = kInvalidId;
  float best_distance = kInf;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) continue;
    if (triangle.normal.dot(p - triangle.centroid) < -back_face_tolerance) continue;
    const float d = TriangleDistance(mesh, id, p);
    if (!IsFinite(d) || d > tolerance) continue;
    if (best == kInvalidId || d < best_distance) {
      best = id;
      best_distance = d;
    }
  }
  if (best == kInvalidId) return false;
  if (located != nullptr) *located = best;
  if (distance != nullptr) *distance = best_distance;
  return true;
}

bool SharesTraversableEdge(const TriangleMesh& mesh, const TraversabilityParams& trav,
                           TriangleId a, TriangleId b) {
  if (a == b) return true;
  if (mesh.componentId(a) != mesh.componentId(b)) return false;
  const TriangleNode& ta = mesh.triangle(a);
  const TriangleNode& tb = mesh.triangle(b);
  for (const TriangleEdge& edge : ta.edges) {
    if (edge.neighbor != b) continue;
    return edge.traversable && TraversabilityEvaluator::isEdgeTraversable(ta, tb, trav);
  }
  return false;
}

// Projects `point` onto a placeable triangle, resolving the shared-edge/shared-
// vertex ambiguity against the previously located sample so a legitimate corner
// touch is not misread as a non-traversable jump.
bool LocatePlaceable(const TriangleMesh& mesh, const TraversabilityParams& trav,
                     const Eigen::Vector3f& point, TriangleId previous, float tolerance,
                     TriangleId* located) {
  TriangleId nearest = kInvalidId;
  if (!NearestPlaceableTriangle(mesh, trav, point, tolerance, &nearest, nullptr)) return false;
  if (previous == kInvalidId || nearest == previous ||
      SharesTraversableEdge(mesh, trav, previous, nearest)) {
    *located = nearest;
    return true;
  }
  TriangleId connected = kInvalidId;
  float connected_distance = kInf;
  for (const TriangleEdge& edge : mesh.triangle(previous).edges) {
    const TriangleId id = edge.neighbor;
    if (id == kInvalidId) continue;
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) continue;
    if (triangle.normal.dot(point - triangle.centroid) < -trav.max_step_height) continue;
    const float distance = TriangleDistance(mesh, id, point);
    if (!IsFinite(distance) || distance > tolerance) continue;
    if (connected == kInvalidId || distance < connected_distance) {
      connected = id;
      connected_distance = distance;
    }
  }
  if (connected == kInvalidId) return false;
  *located = connected;
  return true;
}

// kappa = 2*sin(angle at B)/|AC| == 2*|BA x BC| / (|BA|*|BC|*|AC|).
float CurvatureAt(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f ba = a - b, bc = c - b;
  const float lab = ba.norm(), lbc = bc.norm(), lac = (c - a).norm();
  if (lab <= kDuplicateEpsilon || lbc <= kDuplicateEpsilon || lac <= kDuplicateEpsilon) {
    return 0.0f;
  }
  return 2.0f * ba.cross(bc).norm() / (lab * lbc * lac);
}

// A sample plus the index of the input vertex it is attributed to.
struct Sample {
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  std::size_t source = 0;
};

std::vector<Sample> Resample(const std::vector<Eigen::Vector3f>& path, float check_step) {
  std::vector<Sample> samples;
  samples.push_back(Sample{path.front(), 0});
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3f a = path[i];
    const Eigen::Vector3f b = path[i + 1];
    const float length = (b - a).norm();
    int steps = static_cast<int>(std::ceil(length / check_step));
    if (steps < 1) steps = 1;
    for (int s = 1; s < steps; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(steps);
      samples.push_back(Sample{a + (b - a) * t, i});
    }
    // The segment end carries its own vertex index so a bad vertex is reported
    // as that vertex, not as the segment before it.
    samples.push_back(Sample{b, i + 1});
  }
  return samples;
}

std::uint64_t EdgeKey(PointId a, PointId b) {
  const std::uint32_t low = static_cast<std::uint32_t>(std::min(a, b));
  const std::uint32_t high = static_cast<std::uint32_t>(std::max(a, b));
  return (static_cast<std::uint64_t>(low) << 32) | static_cast<std::uint64_t>(high);
}

// Average upward normal of the mesh triangles incident to an undirected edge.
// Boundary-chain edges are shared mesh edges, so this recovers the local surface
// tangent plane without needing the Region partition.
class EdgeNormals {
 public:
  explicit EdgeNormals(const TriangleMesh& mesh) {
    for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
      const TriangleNode& triangle = mesh.triangle(id);
      for (int k = 0; k < 3; ++k) {
        const PointId u = triangle.vertex_ids[k];
        const PointId v = triangle.vertex_ids[(k + 1) % 3];
        if (!Valid(u) || !Valid(v)) continue;
        // Eigen's fixed-size default constructor does not zero its storage, so
        // operator[] would make the first `+=` read indeterminate memory. Seed
        // the accumulator explicitly.
        sums_.try_emplace(EdgeKey(u, v), Eigen::Vector3d::Zero())
            .first->second += triangle.normal.cast<double>();
      }
    }
  }

  Eigen::Vector3d Normal(PointId u, PointId v) const {
    const auto it = sums_.find(EdgeKey(u, v));
    if (it == sums_.end() || it->second.squaredNorm() < 1e-12) {
      return Eigen::Vector3d::UnitZ();
    }
    return it->second.normalized();
  }

 private:
  static bool Valid(PointId id) { return id >= 0; }
  std::unordered_map<std::uint64_t, Eigen::Vector3d> sums_;
};

// Tests whether segment [a,b] passes through the local "fence" spanned by edge
// [c,d]: the path must cross the edge's tangent plane (sign flip), the crossing
// point must lie within the edge extent, and it must be within `tolerance` of the
// edge itself. This is a true geometric intersection with the shared-edge
// geometry, so parallel Portals on the same Region pair are told apart.
bool CrossesFence(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                  const Eigen::Vector3d& d, const Eigen::Vector3d& edge_normal, double* s_out) {
  if (!a.allFinite() || !b.allFinite() || !c.allFinite() || !d.allFinite()) return false;
  const Eigen::Vector3d edge = d - c;
  const double length = edge.norm();
  if (!(length > 1e-9)) return false;
  const Eigen::Vector3d direction = edge / length;
  Eigen::Vector3d normal = edge_normal;
  if (!normal.allFinite() || normal.squaredNorm() < 1e-12) normal = Eigen::Vector3d::UnitZ();
  normal.normalize();

  const Eigen::Vector3d fence_normal = normal.cross(direction);
  if (!fence_normal.allFinite() || fence_normal.squaredNorm() < 1e-12) return false;
  const Eigen::Vector3d m = fence_normal.normalized();

  const double fa = (a - c).dot(m);
  const double fb = (b - c).dot(m);
  // A path vertex may land exactly on the boundary chain (e.g. the planner
  // routes through a Portal centre), so an endpoint on the fence counts: both
  // adjacent segments then report the same Portal and the caller de-duplicates.
  if (fa > 0.0 && fb > 0.0) return false;
  if (fa < 0.0 && fb < 0.0) return false;
  if (fa == 0.0 && fb == 0.0) return false;

  const double s = fa / (fa - fb);
  if (s < 0.0 || s > 1.0) return false;
  const Eigen::Vector3d crossing = a + (b - a) * s;
  const double along = (crossing - c).dot(direction);
  if (along < -kFenceTolerance || along > length + kFenceTolerance) return false;
  const double offset = ((crossing - c) - along * direction).norm();
  if (offset > kFenceTolerance) return false;

  if (s_out != nullptr) *s_out = s;
  return true;
}

}  // namespace

PathValidationReport SafetyValidator::validate(const std::vector<Eigen::Vector3f>& path,
                                               const TriangleMesh& mesh,
                                               const TraversabilityParams& params,
                                               const RobotParams& robot, float check_step) const {
  PathValidationReport report;
  const std::size_t count = path.size();

  bool invalid = false;
  if (!IsFinite(check_step) || !(check_step > 0.0f)) invalid = true;
  if (!IsFinite(robot.max_path_curvature) || !(robot.max_path_curvature > 0.0f)) invalid = true;
  if (!ValidateTraversabilityParams(params).ok()) invalid = true;
  if (mesh.size() == 0) invalid = true;
  if (count < 2u) invalid = true;
  for (std::size_t i = 0; i < count; ++i) {
    if (!IsFinite(path[i])) {
      invalid = true;
      report.violating_indices.push_back(i);
    }
  }
  if (invalid) {
    if (report.violating_indices.empty()) {
      for (std::size_t i = 0; i < count; ++i) report.violating_indices.push_back(i);
    }
    std::sort(report.violating_indices.begin(), report.violating_indices.end());
    report.violating_indices.erase(
        std::unique(report.violating_indices.begin(), report.violating_indices.end()),
        report.violating_indices.end());
    report.valid = false;
    return report;
  }

  const std::vector<Sample> samples = Resample(path, check_step);
  std::vector<TriangleId> located(samples.size(), kInvalidId);

  auto record = [&report](std::size_t source) { report.violating_indices.push_back(source); };

  report.min_clearance = kInf;
  bool any_located = false;
  TriangleId previous = kInvalidId;
  for (std::size_t k = 0; k < samples.size(); ++k) {
    const Eigen::Vector3f& point = samples[k].point;
    TriangleId nearest = kInvalidId;
    float nearest_distance = kInf;
    if (!NearestAnyTriangle(mesh, point, &nearest, &nearest_distance) ||
        nearest_distance > kProjectionTolerance) {
      record(samples[k].source);
      previous = kInvalidId;
      continue;
    }

    const TriangleNode& nearest_triangle = mesh.triangle(nearest);
    report.min_clearance = std::min(report.min_clearance, nearest_triangle.min_clearance);
    any_located = true;
    if (!TraversabilityEvaluator::isRobotPlaceable(nearest_triangle, params) ||
        nearest_triangle.normal.dot(point - nearest_triangle.centroid) < -params.max_step_height) {
      record(samples[k].source);
    }

    TriangleId current = kInvalidId;
    if (!LocatePlaceable(mesh, params, point, previous, kProjectionTolerance, &current)) {
      record(samples[k].source);
      previous = kInvalidId;
      continue;
    }
    located[k] = current;
    previous = current;
  }

  // Crossed shared edges must be traversable (normal difference, height jump and
  // optional slope difference) and feed the reported max_step.
  for (std::size_t k = 0; k + 1 < samples.size(); ++k) {
    const TriangleId a = located[k];
    const TriangleId b = located[k + 1];
    if (a == kInvalidId || b == kInvalidId || a == b) continue;
    const TriangleNode& ta = mesh.triangle(a);
    const TriangleNode& tb = mesh.triangle(b);
    bool adjacent = false;
    for (const TriangleEdge& edge : ta.edges) {
      if (edge.neighbor == b) {
        adjacent = true;
        break;
      }
    }
    if (!adjacent) {
      record(samples[k].source);
      continue;
    }
    report.max_step =
        std::max(report.max_step, TraversabilityEvaluator::heightJump(ta, tb));
    if (!TraversabilityEvaluator::isEdgeTraversable(ta, tb, params)) {
      record(samples[k].source);
    }
  }

  // Path slope over every resampled sub-segment.
  for (std::size_t k = 0; k + 1 < samples.size(); ++k) {
    const Eigen::Vector3f delta = samples[k + 1].point - samples[k].point;
    const float horizontal = std::hypot(delta.x(), delta.y());
    const float slope_deg =
        std::atan2(std::fabs(delta.z()), horizontal) * 180.0f / kPi;
    report.max_slope_deg = std::max(report.max_slope_deg, slope_deg);
    if (slope_deg > params.max_slope_deg + kSlopeEpsilonDeg) {
      record(samples[k].source);
    }
  }

  // Path curvature over every interior sample triple.
  for (std::size_t k = 1; k + 1 < samples.size(); ++k) {
    const float kappa = CurvatureAt(samples[k - 1].point, samples[k].point, samples[k + 1].point);
    report.max_curvature = std::max(report.max_curvature, kappa);
    if (kappa > robot.max_path_curvature + kCurvatureEpsilon) {
      record(samples[k].source);
    }
  }

  if (!any_located) report.min_clearance = 0.0f;

  std::sort(report.violating_indices.begin(), report.violating_indices.end());
  report.violating_indices.erase(
      std::unique(report.violating_indices.begin(), report.violating_indices.end()),
      report.violating_indices.end());
  report.valid = report.violating_indices.empty();
  return report;
}

Result<TriangleId> SafetyValidator::locatePoint(const TriangleMesh& mesh, const Eigen::Vector3f& p,
                                                float max_distance, float* distance) const {
  if (!IsFinite(p)) return Invalid("locatePoint query point is not finite");
  if (!IsFinite(max_distance) || max_distance < 0.0f) {
    return Invalid("locatePoint max_distance must be finite and non-negative");
  }
  if (mesh.size() == 0) return Invalid("locatePoint called on an empty mesh");

  TriangleId best = kInvalidId;
  float best_distance = kInf;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    if (!mesh.triangle(id).traversable) continue;
    const float d = TriangleDistance(mesh, id, p);
    if (!IsFinite(d)) continue;
    // Ascending ids plus a strict `<` make ties deterministic (smallest id wins).
    if (best == kInvalidId || d < best_distance) {
      best = id;
      best_distance = d;
    }
  }
  if (best == kInvalidId) {
    return Invalid("locatePoint found no traversable triangle");
  }
  if (best_distance > max_distance) {
    return Invalid("locatePoint nearest traversable triangle is beyond max_distance");
  }
  if (distance != nullptr) *distance = best_distance;
  return best;
}

Result<TriangleId> SafetyValidator::locatePointOriented(const TriangleMesh& mesh,
                                                        const Eigen::Vector3f& p, float max_distance,
                                                        float max_normal_diff_rad,
                                                        float* distance) const {
  if (!IsFinite(p)) return Invalid("locatePointOriented query point is not finite");
  if (!IsFinite(max_distance) || max_distance < 0.0f) {
    return Invalid("locatePointOriented max_distance must be finite and non-negative");
  }
  if (!IsFinite(max_normal_diff_rad) || max_normal_diff_rad < 0.0f) {
    return Invalid("locatePointOriented max_normal_diff_rad must be finite and non-negative");
  }
  if (mesh.size() == 0) return Invalid("locatePointOriented called on an empty mesh");

  TriangleId best = kInvalidId;
  float best_distance = kInf;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const TriangleNode& triangle = mesh.triangle(id);
    if (!triangle.traversable) continue;
    if (angleBetween(triangle.normal, Eigen::Vector3f::UnitZ()) > max_normal_diff_rad) continue;
    // R6: only the front face may accept the query, so a point under an upper
    // floor cannot snap through its back face.
    if (triangle.normal.dot(p - triangle.centroid) < 0.0f) continue;
    const float d = TriangleDistance(mesh, id, p);
    if (!IsFinite(d)) continue;
    if (best == kInvalidId || d < best_distance) {
      best = id;
      best_distance = d;
    }
  }
  if (best == kInvalidId) {
    return Invalid("locatePointOriented found no oriented traversable triangle");
  }
  if (best_distance > max_distance) {
    return Invalid("locatePointOriented nearest triangle is beyond max_distance");
  }
  if (distance != nullptr) *distance = best_distance;
  return best;
}

std::vector<PortalId> SafetyValidator::crossedPortals(const std::vector<Eigen::Vector3f>& path,
                                                      const TriangleMesh& mesh,
                                                      const TopologyGraph& graph) {
  std::vector<PortalId> crossings;
  if (path.size() < 2u || mesh.size() == 0 || graph.portals().empty()) return crossings;
  for (const Eigen::Vector3f& point : path) {
    if (!IsFinite(point)) return {};
  }

  const std::vector<Eigen::Vector3f>& vertices = mesh.vertices();
  const std::size_t vertex_count = vertices.size();
  const EdgeNormals edge_normals(mesh);

  struct Hit {
    double s = 0.0;
    PortalId id = kInvalidId;
  };

  std::unordered_set<PortalId> seen;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d a = path[i].cast<double>();
    const Eigen::Vector3d b = path[i + 1].cast<double>();
    std::vector<Hit> hits;
    for (const Portal& portal : graph.portals()) {
      double best_s = std::numeric_limits<double>::infinity();
      const std::vector<PointId>& chain = portal.boundary_chain;
      for (std::size_t k = 0; k + 1 < chain.size(); ++k) {
        const PointId u = chain[k];
        const PointId v = chain[k + 1];
        if (u < 0 || v < 0 || static_cast<std::size_t>(u) >= vertex_count ||
            static_cast<std::size_t>(v) >= vertex_count) {
          continue;
        }
        double s = 0.0;
        if (CrossesFence(a, b, vertices[static_cast<std::size_t>(u)].cast<double>(),
                         vertices[static_cast<std::size_t>(v)].cast<double>(),
                         edge_normals.Normal(u, v), &s)) {
          best_s = std::min(best_s, s);
        }
      }
      if (best_s < std::numeric_limits<double>::infinity()) {
        hits.push_back(Hit{best_s, portal.id});
      }
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& left, const Hit& right) {
      if (left.s != right.s) return left.s < right.s;
      return left.id < right.id;
    });
    for (const Hit& hit : hits) {
      if (seen.insert(hit.id).second) crossings.push_back(hit.id);
    }
  }
  return crossings;
}

bool SafetyValidator::portalOrderMatches(const std::vector<Eigen::Vector3f>& path,
                                         const TriangleMesh& mesh, const TopologyGraph& graph,
                                         const std::vector<PortalId>& expected) {
  return crossedPortals(path, mesh, graph) == expected;
}

}  // namespace topmap
