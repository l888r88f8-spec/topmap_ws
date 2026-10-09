#include "topmap_planner/safety_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
// Lower bound for the uniform-grid cell size so a degenerate mesh cannot divide
// by zero; normal meshes derive a much larger cell from their edge lengths.
constexpr float kGridMinCell = 1e-3f;

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

// True 3D nearest-triangle search accelerated by an XY uniform grid that is
// built once per public call and reused by every sample, instead of rescanning
// the mesh per sample. The cell size is the median triangle edge length of the
// mesh, so it is derived from the resolution rather than any scene constant.
// Each triangle is registered in every cell its XY AABB overlaps; a query then
// expands square rings and stops as soon as no unvisited cell can hold a closer
// triangle. Candidates are chosen by the exact point-triangle distance with the
// smallest TriangleId winning ties, so the result is identical to the
// ascending-id full scan this replaces. Every triangle (not only the traversable
// ones) is indexed, which preserves cross-layer safety: the nearest triangle of
// any kind still decides placeability and clearance.
class MeshGrid {
 public:
  bool Build(const TriangleMesh& mesh) {
    mesh_ = &mesh;
    cell_size_ = 0.0f;
    origin_.setZero();
    nx_ = 0;
    ny_ = 0;
    entries_.clear();
    cell_begin_.clear();
    if (mesh.size() == 0) return false;

    std::vector<float> edges;
    edges.reserve(mesh.size() * 3u);
    float min_x = kInf, min_y = kInf;
    float max_x = -kInf, max_y = -kInf;
    for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
      const TriangleNode& t = mesh.triangle(id);
      const Eigen::Vector3f& a = mesh.vertices()[t.vertex_ids[0]];
      const Eigen::Vector3f& b = mesh.vertices()[t.vertex_ids[1]];
      const Eigen::Vector3f& c = mesh.vertices()[t.vertex_ids[2]];
      edges.push_back((b - a).norm());
      edges.push_back((c - b).norm());
      edges.push_back((a - c).norm());
      min_x = std::min(min_x, std::min(a.x(), std::min(b.x(), c.x())));
      max_x = std::max(max_x, std::max(a.x(), std::max(b.x(), c.x())));
      min_y = std::min(min_y, std::min(a.y(), std::min(b.y(), c.y())));
      max_y = std::max(max_y, std::max(a.y(), std::max(b.y(), c.y())));
    }
    if (edges.empty()) return false;
    if (!IsFinite(min_x) || !IsFinite(max_x) || !IsFinite(min_y) || !IsFinite(max_y)) {
      return false;
    }

    const std::size_t mid = edges.size() / 2u;
    std::nth_element(edges.begin(), edges.begin() + static_cast<std::ptrdiff_t>(mid), edges.end());
    float cell = edges[mid];
    // Generic memory guard: cap the cell count at a small multiple of the
    // triangle count even for meshes with one huge spatial extent.
    const double span_x = static_cast<double>(max_x) - static_cast<double>(min_x);
    const double span_y = static_cast<double>(max_y) - static_cast<double>(min_y);
    const double area = span_x * span_y;
    const double max_cells = 4.0 * static_cast<double>(mesh.size()) + 1024.0;
    if (area > 0.0) {
      const double min_cell = std::sqrt(area / max_cells);
      if (min_cell > static_cast<double>(cell)) cell = static_cast<float>(min_cell);
    }
    if (!IsFinite(cell) || cell < kGridMinCell) cell = kGridMinCell;
    cell_size_ = cell;
    origin_ = Eigen::Vector2f(min_x, min_y);

    nx_ = static_cast<int>(std::floor(span_x / static_cast<double>(cell_size_))) + 1;
    ny_ = static_cast<int>(std::floor(span_y / static_cast<double>(cell_size_))) + 1;
    if (nx_ < 1) nx_ = 1;
    if (ny_ < 1) ny_ = 1;

    struct Range {
      int ix0 = 0, ix1 = 0, iy0 = 0, iy1 = 0;
    };
    std::vector<Range> ranges(mesh.size());
    for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
      const TriangleNode& t = mesh.triangle(id);
      const Eigen::Vector3f& a = mesh.vertices()[t.vertex_ids[0]];
      const Eigen::Vector3f& b = mesh.vertices()[t.vertex_ids[1]];
      const Eigen::Vector3f& c = mesh.vertices()[t.vertex_ids[2]];
      const float tri_min_x = std::min(a.x(), std::min(b.x(), c.x()));
      const float tri_max_x = std::max(a.x(), std::max(b.x(), c.x()));
      const float tri_min_y = std::min(a.y(), std::min(b.y(), c.y()));
      const float tri_max_y = std::max(a.y(), std::max(b.y(), c.y()));
      Range& r = ranges[static_cast<std::size_t>(id)];
      r.ix0 = CellX(tri_min_x);
      r.ix1 = CellX(tri_max_x);
      r.iy0 = CellY(tri_min_y);
      r.iy1 = CellY(tri_max_y);
    }

    const std::size_t num_cells =
        static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    cell_begin_.assign(num_cells + 1u, 0);
    for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
      const Range& r = ranges[static_cast<std::size_t>(id)];
      for (int iy = r.iy0; iy <= r.iy1; ++iy) {
        for (int ix = r.ix0; ix <= r.ix1; ++ix) {
          ++cell_begin_[static_cast<std::size_t>(iy) * static_cast<std::size_t>(nx_) +
                        static_cast<std::size_t>(ix) + 1u];
        }
      }
    }
    for (std::size_t i = 1u; i <= num_cells; ++i) cell_begin_[i] += cell_begin_[i - 1u];
    entries_.resize(static_cast<std::size_t>(cell_begin_[num_cells]));
    std::vector<int> cursor(cell_begin_.begin(), cell_begin_.end() - 1);
    for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
      const Range& r = ranges[static_cast<std::size_t>(id)];
      for (int iy = r.iy0; iy <= r.iy1; ++iy) {
        for (int ix = r.ix0; ix <= r.ix1; ++ix) {
          const std::size_t c = static_cast<std::size_t>(iy) * static_cast<std::size_t>(nx_) +
                                static_cast<std::size_t>(ix);
          entries_[static_cast<std::size_t>(cursor[c]++)] = id;
        }
      }
    }
    return true;
  }

  bool NearestWithin(const Eigen::Vector3f& point, float tolerance, TriangleId* located,
                     float* distance) const {
    return NearestAccept(point, [](const TriangleMesh&, TriangleId, const Eigen::Vector3f&) {
      return true;
    }, tolerance, located, distance);
  }

  // Exact nearest robot-placeable, front-facing triangle within `tolerance`.
  bool NearestPlaceable(const Eigen::Vector3f& point, const TraversabilityParams& trav,
                        float tolerance, TriangleId* located, float* distance) const {
    const float back_face_tolerance = trav.max_step_height;
    return NearestAccept(
        point,
        [&](const TriangleMesh& mesh, TriangleId id, const Eigen::Vector3f& p) {
          const TriangleNode& triangle = mesh.triangle(id);
          if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) return false;
          return triangle.normal.dot(p - triangle.centroid) >= -back_face_tolerance;
        },
        tolerance, located, distance);
  }

 private:
  int CellX(float x) const {
    if (!(x > -kInf)) return 0;  // NaN guard; callers validate finiteness.
    const float cell = (x - origin_.x()) / cell_size_;
    if (cell <= 0.0f) return 0;
    if (cell >= static_cast<float>(nx_ - 1)) return nx_ - 1;
    return static_cast<int>(cell);
  }
  int CellY(float y) const {
    if (!(y > -kInf)) return 0;
    const float cell = (y - origin_.y()) / cell_size_;
    if (cell <= 0.0f) return 0;
    if (cell >= static_cast<float>(ny_ - 1)) return ny_ - 1;
    return static_cast<int>(cell);
  }

  // `accept` is applied only to in-range candidates and after the distance
  // filter, exactly like the ascending-id scans it replaces. `max_distance` may
  // be +inf for an unrestricted nearest query.
  template <typename Accept>
  bool NearestAccept(const Eigen::Vector3f& point, const Accept& accept, float max_distance,
                     TriangleId* located, float* distance) const {
    // Degenerate mesh bounds (Build failed) fall back to the exact linear scan
    // so behaviour never depends on the grid being available.
    if (empty()) {
      if (mesh_ == nullptr) return false;
      TriangleId best = kInvalidId;
      float best_distance = kInf;
      for (TriangleId id = 0; id < static_cast<TriangleId>(mesh_->size()); ++id) {
        if (!accept(*mesh_, id, point)) continue;
        const float d = TriangleDistance(*mesh_, id, point);
        if (!IsFinite(d) || d > max_distance) continue;
        if (best == kInvalidId || d < best_distance || (d == best_distance && id < best)) {
          best = id;
          best_distance = d;
        }
      }
      if (best == kInvalidId) return false;
      if (located != nullptr) *located = best;
      if (distance != nullptr) *distance = best_distance;
      return true;
    }

    const int cx = CellX(point.x());
    const int cy = CellY(point.y());
    const int max_ring = std::max(nx_, ny_);
    TriangleId best = kInvalidId;
    float best_distance = kInf;
    for (int r = 0; r <= max_ring; ++r) {
      const int x0 = std::max(0, cx - r);
      const int x1 = std::min(nx_ - 1, cx + r);
      const int y0 = std::max(0, cy - r);
      const int y1 = std::min(ny_ - 1, cy + r);
      if (x0 > x1 || y0 > y1) break;
      for (int y = y0; y <= y1; ++y) {
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(nx_);
        for (int x = x0; x <= x1; ++x) {
          const std::size_t c = row + static_cast<std::size_t>(x);
          for (int k = cell_begin_[c]; k < cell_begin_[c + 1u]; ++k) {
            const TriangleId id = entries_[static_cast<std::size_t>(k)];
            if (!accept(*mesh_, id, point)) continue;
            const float d = TriangleDistance(*mesh_, id, point);
            if (!IsFinite(d) || d > max_distance) continue;
            if (best == kInvalidId || d < best_distance || (d == best_distance && id < best)) {
              best = id;
              best_distance = d;
            }
          }
        }
      }
      // After searching the square of radius r every unvisited cell is at XY
      // distance >= r * cell_size, hence at true 3D distance at least that.
      if (best != kInvalidId) {
        const float bound = static_cast<float>(r) * cell_size_;
        if (best_distance < bound || (r == 0 && best_distance <= 0.0f)) break;
      } else if (IsFinite(max_distance) &&
                 static_cast<float>(r) * cell_size_ > max_distance) {
        // No closer cell can hold a candidate within the tolerance either.
        break;
      }
      if (x0 == 0 && x1 == nx_ - 1 && y0 == 0 && y1 == ny_ - 1) break;
    }
    if (best == kInvalidId) return false;
    if (located != nullptr) *located = best;
    if (distance != nullptr) *distance = best_distance;
    return true;
  }

  bool empty() const { return cell_size_ <= 0.0f || nx_ <= 0 || ny_ <= 0; }

  const TriangleMesh* mesh_ = nullptr;
  float cell_size_ = 0.0f;
  Eigen::Vector2f origin_ = Eigen::Vector2f::Zero();
  int nx_ = 0;
  int ny_ = 0;
  std::vector<TriangleId> entries_;  // CSR payload
  std::vector<int> cell_begin_;      // CSR offsets, size nx*ny + 1
};

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

// Consecutive samples may straddle a mesh vertex. The triangles containing the
// two samples can then be separated by several triangles in the local vertex
// fan even though the short path segment is continuous. Search only triangles
// within the projection tolerance of the current sample, and require every
// crossed edge and intermediate triangle to remain traversable.
bool ConnectedNearPoint(const TriangleMesh& mesh, const TraversabilityParams& trav,
                        TriangleId a, TriangleId b, const Eigen::Vector3f& point,
                        float tolerance) {
  if (SharesTraversableEdge(mesh, trav, a, b)) return true;
  if (a < 0 || b < 0 || mesh.componentId(a) != mesh.componentId(b)) return false;
  std::vector<char> visited(mesh.size(), 0);
  std::vector<TriangleId> queue;
  queue.push_back(a);
  visited[static_cast<std::size_t>(a)] = 1;
  for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
    const TriangleId current = queue[cursor];
    for (const TriangleEdge& edge : mesh.triangle(current).edges) {
      const TriangleId next = edge.neighbor;
      if (next == kInvalidId || visited[static_cast<std::size_t>(next)] || !edge.traversable) {
        continue;
      }
      const TriangleNode& next_triangle = mesh.triangle(next);
      if (!TraversabilityEvaluator::isRobotPlaceable(next_triangle, trav) ||
          !TraversabilityEvaluator::isEdgeTraversable(mesh.triangle(current), next_triangle,
                                                      trav) ||
          TriangleDistance(mesh, next, point) > tolerance) {
        continue;
      }
      if (next == b) return true;
      visited[static_cast<std::size_t>(next)] = 1;
      queue.push_back(next);
    }
  }
  return false;
}

// Projects `point` onto a placeable triangle, resolving the shared-edge/shared-
// vertex ambiguity against the previously located sample so a legitimate corner
// touch is not misread as a non-traversable jump.
bool LocatePlaceable(const MeshGrid& grid, const TriangleMesh& mesh,
                     const TraversabilityParams& trav, const Eigen::Vector3f& point,
                     TriangleId previous, float tolerance, TriangleId* located) {
  TriangleId nearest = kInvalidId;
  if (!grid.NearestWithin(point, tolerance, &nearest, nullptr)) return false;
  const TriangleNode& nearest_triangle = mesh.triangle(nearest);
  if (!TraversabilityEvaluator::isRobotPlaceable(nearest_triangle, trav) ||
      nearest_triangle.normal.dot(point - nearest_triangle.centroid) < -trav.max_step_height) {
    return false;
  }
  if (previous == kInvalidId || nearest == previous ||
      ConnectedNearPoint(mesh, trav, previous, nearest, point, tolerance)) {
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

// Tests whether segment [a,b] passes through the vertical curtain spanned by the
// shared edge [c,d], evaluated on the XY projection. The edge's local surface
// normal is deliberately not used: on a ramp the averaged normal tilts the fence
// plane, so a chord above the boundary would miss the crossing, yet a robot
// crossing the ramp genuinely traverses the boundary chain. The sign test keeps
// a path running parallel to (or along) the chain from being reported, and the
// `along` window keeps the hit on the actual edge extent, so parallel Portals on
// the same Region pair remain distinguishable by their boundary geometry.
bool CrossesCurtain(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c,
                    const Eigen::Vector3d& d, double* s_out) {
  if (!a.allFinite() || !b.allFinite() || !c.allFinite() || !d.allFinite()) return false;
  const Eigen::Vector2d a2(a.x(), a.y());
  const Eigen::Vector2d b2(b.x(), b.y());
  const Eigen::Vector2d c2(c.x(), c.y());
  const Eigen::Vector2d d2(d.x(), d.y());
  const Eigen::Vector2d edge = d2 - c2;
  const double length = edge.norm();
  if (!(length > 1e-9)) return false;
  const Eigen::Vector2d direction = edge / length;
  const Eigen::Vector2d normal(-direction.y(), direction.x());

  const double fa = (a2 - c2).dot(normal);
  const double fb = (b2 - c2).dot(normal);
  // A path vertex may land exactly on the boundary chain (e.g. the planner
  // routes through a Portal centre), so an endpoint on the curtain counts: both
  // adjacent segments then report the same Portal and the caller de-duplicates.
  if (fa > 0.0 && fb > 0.0) return false;
  if (fa < 0.0 && fb < 0.0) return false;
  if (fa == 0.0 && fb == 0.0) return false;

  const double s = fa / (fa - fb);
  if (s < 0.0 || s > 1.0) return false;
  const Eigen::Vector2d crossing = a2 + (b2 - a2) * s;
  const double along = (crossing - c2).dot(direction);
  if (along < -kFenceTolerance || along > length + kFenceTolerance) return false;

  if (s_out != nullptr) *s_out = s;
  return true;
}

// Samples segment [a,b] at an interval <= check_step and requires every sample
// to project onto a robot-placeable, front-facing triangle with traversable
// edge transitions. This mirrors the per-sample body of validate(), so a graph
// edge that passes here cannot later fail validate()'s clearance/placeability
// gate.
bool SegmentSampledSafe(const MeshGrid& grid, const TriangleMesh& mesh,
                        const TraversabilityParams& params, const Eigen::Vector3f& a,
                        const Eigen::Vector3f& b, float check_step) {
  const float length = (b - a).norm();
  if (!IsFinite(length)) return false;
  int steps = static_cast<int>(std::ceil(length / check_step));
  if (steps < 1) steps = 1;
  TriangleId previous = kInvalidId;
  for (int s = 0; s <= steps; ++s) {
    const float t = static_cast<float>(s) / static_cast<float>(steps);
    const Eigen::Vector3f sample = a + (b - a) * t;

    TriangleId nearest = kInvalidId;
    float nearest_distance = kInf;
    if (!grid.NearestWithin(sample, kProjectionTolerance, &nearest, &nearest_distance)) {
      return false;
    }
    const TriangleNode& nearest_triangle = mesh.triangle(nearest);
    if (!TraversabilityEvaluator::isRobotPlaceable(nearest_triangle, params) ||
        nearest_triangle.normal.dot(sample - nearest_triangle.centroid) < -params.max_step_height) {
      return false;
    }

    TriangleId current = kInvalidId;
    if (!LocatePlaceable(grid, mesh, params, sample, previous, kProjectionTolerance, &current)) {
      return false;
    }
    if (previous != kInvalidId && current != previous &&
        !ConnectedNearPoint(mesh, params, previous, current, sample, kProjectionTolerance)) {
      return false;
    }
    previous = current;
  }
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

  // One spatial index for the whole call; every sample reuses it instead of
  // rescanning the mesh.
  MeshGrid grid;
  grid.Build(mesh);

  auto record = [&report](std::size_t source) { report.violating_indices.push_back(source); };

  report.min_clearance = kInf;
  bool any_located = false;
  TriangleId previous = kInvalidId;
  for (std::size_t k = 0; k < samples.size(); ++k) {
    const Eigen::Vector3f& point = samples[k].point;
    TriangleId nearest = kInvalidId;
    float nearest_distance = kInf;
    if (!grid.NearestWithin(point, kProjectionTolerance, &nearest, &nearest_distance)) {
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
    if (!LocatePlaceable(grid, mesh, params, point, previous, kProjectionTolerance, &current)) {
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
    if (!ConnectedNearPoint(mesh, params, a, b, samples[k + 1].point,
                            kProjectionTolerance)) {
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

  // Use the same geometric-vertex triples as PathOptimizer. Interpolated
  // samples on each straight segment are for collision/clearance checks; using
  // them as curvature neighbours would make the reported curvature depend on
  // check_step and artificially amplify every polyline corner.
  for (std::size_t k = 1; k + 1 < path.size(); ++k) {
    const float kappa = CurvatureAt(path[k - 1], path[k], path[k + 1]);
    report.max_curvature = std::max(report.max_curvature, kappa);
    if (kappa > robot.max_path_curvature + kCurvatureEpsilon) {
      record(k);
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

Result<std::vector<char>> SafetyValidator::validateSegments(
    const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& segments,
    const TriangleMesh& mesh, const TraversabilityParams& params, float check_step) const {
  if (!IsFinite(check_step) || !(check_step > 0.0f)) {
    return Invalid("validateSegments check_step must be finite and positive");
  }
  if (!ValidateTraversabilityParams(params).ok()) {
    return Invalid("validateSegments received invalid traversability parameters");
  }
  if (mesh.size() == 0) return Invalid("validateSegments called on an empty mesh");
  for (const std::pair<Eigen::Vector3f, Eigen::Vector3f>& segment : segments) {
    if (!IsFinite(segment.first) || !IsFinite(segment.second)) {
      return Invalid("validateSegments segment endpoint is not finite");
    }
  }

  // One spatial index for the whole batch; every segment reuses it.
  MeshGrid grid;
  grid.Build(mesh);
  std::vector<char> safe;
  safe.reserve(segments.size());
  for (const std::pair<Eigen::Vector3f, Eigen::Vector3f>& segment : segments) {
    safe.push_back(
        SegmentSampledSafe(grid, mesh, params, segment.first, segment.second, check_step) ? 1 : 0);
  }
  return safe;
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
        if (CrossesCurtain(a, b, vertices[static_cast<std::size_t>(u)].cast<double>(),
                           vertices[static_cast<std::size_t>(v)].cast<double>(), &s)) {
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
