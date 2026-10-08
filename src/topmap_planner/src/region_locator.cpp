#include "topmap_planner/region_locator.hpp"

#include <cmath>
#include <string>

#include <Eigen/Core>

#include "topmap_core/geometry.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

constexpr float kPi = 3.14159265358979323846f;

bool IsFinite(float value) { return std::isfinite(value); }

bool IsFinite(const Eigen::Vector3f& point) {
  return IsFinite(point.x()) && IsFinite(point.y()) && IsFinite(point.z());
}

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

}  // namespace

Result<RegionId> RegionLocator::locate(const TriangleMesh& mesh, const TopologyGraph& graph,
                                       const Eigen::Vector3f& point,
                                       const PlanParams& params) const {
  if (!IsFinite(point)) return Invalid("region locator query point is not finite");
  if (!IsFinite(params.max_project_radius) || params.max_project_radius < 0.0f) {
    return Invalid("plan.max_project_radius must be finite and non-negative");
  }
  if (!IsFinite(params.max_project_normal_diff_deg) ||
      params.max_project_normal_diff_deg < 0.0f) {
    return Invalid("plan.max_project_normal_diff_deg must be finite and non-negative");
  }
  if (mesh.size() == 0) {
    return Status::Error(ErrorCode::kMeshEmpty, "cannot locate a region on an empty mesh");
  }
  if (graph.regions().empty()) {
    return Invalid("cannot locate a region on an empty topology graph");
  }

  // R10: placeability is the hard surface constraint (traversable plus
  // clearance); the default TraversabilityParams is the frozen contract because
  // locate() only receives PlanParams.
  const TraversabilityParams traversability;
  const float max_normal_rad = params.max_project_normal_diff_deg * kPi / 180.0f;
  const float back_face_tolerance = traversability.max_step_height;

  TriangleId best_id = kInvalidId;
  float best_distance = 0.0f;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const TriangleNode& triangle = mesh.triangle(id);
    if (!triangle.traversable) continue;
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, traversability)) continue;
    if (angleBetween(triangle.normal, Eigen::Vector3f::UnitZ()) > max_normal_rad) continue;
    // The point must be on the front face; a step-height of tolerance keeps a
    // start that is nominally on the surface from being rejected.
    if (triangle.normal.dot(point - triangle.centroid) < -back_face_tolerance) continue;

    const Eigen::Vector3f& a = mesh.vertices()[triangle.vertex_ids[0]];
    const Eigen::Vector3f& b = mesh.vertices()[triangle.vertex_ids[1]];
    const Eigen::Vector3f& c = mesh.vertices()[triangle.vertex_ids[2]];
    const float distance = pointTriangleDistance(point, a, b, c);
    // Ascending iteration plus a strict `<` makes ties deterministic: the
    // smallest TriangleId wins.
    if (best_id == kInvalidId || distance < best_distance) {
      best_id = id;
      best_distance = distance;
    }
  }

  if (best_id == kInvalidId || best_distance > params.max_project_radius) {
    return Status::Error(ErrorCode::kStartNotOnSurface,
                         "query point is not within max_project_radius of the surface");
  }

  auto region = graph.regionOfTriangle(best_id);
  if (!region.ok()) {
    return Status::Error(ErrorCode::kStartNotOnSurface,
                         "nearest triangle is not owned by any region");
  }
  return region.value();
}

}  // namespace topmap
