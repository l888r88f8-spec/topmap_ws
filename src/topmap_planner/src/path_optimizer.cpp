#include "topmap_planner/path_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "topmap_core/geometry.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
// Adjacent vertices closer than this collapse when a stage rebuilds a polyline.
constexpr float kDuplicateEpsilon = 1e-6f;
// Squared point-distance epsilon used by the funnel's degenerate-cone tests.
constexpr float kPointEpsilonSq = 1e-12f;
// 2D area epsilon for the funnel's orientation tests.
constexpr double kAreaEps = 1e-9;
// Lower bound for the uniform-grid cell size so a degenerate mesh cannot divide
// by zero; normal meshes derive a much larger cell from their edge lengths.
constexpr float kGridMinCell = 1e-3f;

// T34 deterministic circular corner filleting.
constexpr float kPiF = 3.14159265358979323846f;
// Radius margin: R = (1 + margin) / max_path_curvature keeps the arc's own
// discrete curvature (1 / R) strictly below the limit in the presence of float
// rounding, so a fillet can never trade feasibility for a sharper turn.
constexpr float kFilletRadiusMargin = 0.05f;
// Deflection epsilon: below this the corner is collinear and needs no arc.
constexpr float kFilletMinTurn = 1e-4f;
// A U-turn needs R * tan(pi / 2) -> infinity, so it is never filleted.
constexpr float kFilletMaxTurn = kPiF - 1e-4f;
// Round-off slack for the per-segment tangent budget: an exact fit
// (left + right == length) is legal, so only a genuine overlap is a conflict.
constexpr float kFilletSegmentSlackScale = 16.0f;
// Deterministic upper bound on the emitted polyline so a pathological input can
// never explode the output.
constexpr std::size_t kFilletMaxPoints = 512u;

bool IsFinite(float value) { return std::isfinite(value); }

bool IsFinite(const Eigen::Vector3f& point) {
  return IsFinite(point.x()) && IsFinite(point.y()) && IsFinite(point.z());
}

Status Invalid(std::string message) {
  return Status::Error(ErrorCode::kInvalidArgument, std::move(message));
}

Status OptimizeFailure(std::string message) {
  return Status::Error(ErrorCode::kPathOptimizationFailed, std::move(message));
}

// R1: parameters consumed by the optimizer must be finite and physically
// meaningful; a zero/NaN check_step would make sampling degenerate and a
// non-positive curvature limit would reject every path.
Status ValidatePlanParams(const PlanParams& params) {
  if (!IsFinite(params.check_step) || !(params.check_step > 0.0f)) {
    return Invalid("plan.check_step must be finite and positive");
  }
  if (!IsFinite(params.max_project_radius) || !(params.max_project_radius > 0.0f)) {
    return Invalid("plan.max_project_radius must be finite and positive");
  }
  if (params.smoothing_iterations < 0) {
    return Invalid("plan.smoothing_iterations must be non-negative");
  }
  return Status::Ok();
}

Status ValidateRobotParams(const RobotParams& robot) {
  if (!IsFinite(robot.max_path_curvature) || !(robot.max_path_curvature > 0.0f)) {
    return Invalid("robot.max_path_curvature must be finite and positive");
  }
  return Status::Ok();
}

// Mirrors SurfaceAStar's structural check: a negative clearance or NaN slope
// silently disables the hard safety constraint.
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

// Everything a segment query needs. `allowed` marks the triangles a sample may
// map to (the corridor for shortcut/optimize, the whole mesh for smooth) and
// `grid` is the reusable spatial index over exactly those triangles, built once
// per public call so segment sampling never rescans the whole mesh.
struct SafetyContext {
  const TriangleMesh* mesh = nullptr;
  const std::vector<char>* allowed = nullptr;
  const TraversabilityParams* trav = nullptr;
  float check_step = 0.05f;
  float project_tolerance = 0.30f;
  const class MeshGrid* grid = nullptr;
};

float TriangleDistance(const TriangleMesh& mesh, TriangleId id, const Eigen::Vector3f& p) {
  const TriangleNode& triangle = mesh.triangle(id);
  return pointTriangleDistance(p, mesh.vertices()[triangle.vertex_ids[0]],
                               mesh.vertices()[triangle.vertex_ids[1]],
                               mesh.vertices()[triangle.vertex_ids[2]]);
}

// XY uniform grid over the allowed triangles, shared by every sample of a call.
// The cell size is the median triangle edge length of the indexed set, so it is
// derived from the mesh resolution rather than any scene constant. Each triangle
// is registered in every cell its XY AABB overlaps; a point query then visits
// its own cell plus a ring of neighbours wide enough that any triangle within
// `project_tolerance` of the point lies in a visited cell. The nearest candidate
// is chosen by the true 3D point-triangle distance, so the result is identical to
// a full scan (smallest id wins exact ties).
class MeshGrid {
 public:
  bool Build(const TriangleMesh& mesh, const std::vector<char>& allowed, float tolerance) {
    // The full mesh is indexed so a query always sees the true nearest triangle
    // of any kind. `allowed` (the corridor) is enforced at accept time instead:
    // SafetyValidator rejects a sample whose nearest mesh triangle is not
    // placeable, and the optimizer must not accept a farther placeable triangle
    // that the validator would never see.
    (void)allowed;
    mesh_ = &mesh;
    cell_size_ = 0.0f;
    origin_.setZero();
    nx_ = 0;
    ny_ = 0;
    rings_ = 1;
    entries_.clear();
    cell_begin_.clear();

    // Median edge length over the whole mesh; an empty mesh yields an empty grid
    // and every projection fails exactly like a full scan.
    std::vector<float> edges;
    edges.reserve(mesh.size() * 3u);
    float min_x = kInf, min_y = kInf;
    float max_x = -kInf, max_y = -kInf;
    std::size_t indexed = 0;
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
      ++indexed;
    }
    if (indexed == 0u || edges.empty()) return false;
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
    const double max_cells = 4.0 * static_cast<double>(indexed) + 1024.0;
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

    // Rasterise each triangle's XY AABB into its covering cell range.
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

    // +1 cell margin absorbs the float floor of the query point.
    int rings = static_cast<int>(std::ceil(tolerance / cell_size_)) + 1;
    if (rings < 1) rings = 1;
    rings_ = rings;
    return true;
  }

  bool empty() const { return cell_size_ <= 0.0f || nx_ <= 0 || ny_ <= 0; }

  // True nearest allowed triangle to `point`; ties go to the smallest id like
  // the ascending-id full scan it replaces. Rings expand outward and stop as
  // soon as no unvisited cell can hold a closer triangle: after searching the
  // square of radius r, every unvisited cell is at XY distance >= r*cell, so a
  // best distance strictly below that bound is already the global nearest.
  bool NearestWithin(const Eigen::Vector3f& point, TriangleId* located, float* distance) const {
    if (empty()) return false;
    const int cx = CellX(point.x());
    const int cy = CellY(point.y());
    TriangleId best = kInvalidId;
    float best_distance = kInf;
    for (int r = 0; r <= rings_; ++r) {
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
            const float d = TriangleDistance(*mesh_, id, point);
            if (best == kInvalidId || d < best_distance ||
                (d == best_distance && id < best)) {
              best = id;
              best_distance = d;
            }
          }
        }
      }
      if (best != kInvalidId) {
        const float bound = static_cast<float>(r) * cell_size_;
        // Strict `<` keeps smallest-id tie-breaking exact; r == 0 only stops for
        // a point lying exactly on a triangle (its whole cell was searched).
        if (best_distance < bound || (r == 0 && best_distance <= 0.0f)) break;
      }
      if (x0 == 0 && x1 == nx_ - 1 && y0 == 0 && y1 == ny_ - 1) break;
    }
    if (best == kInvalidId) return false;
    if (located != nullptr) *located = best;
    if (distance != nullptr) *distance = best_distance;
    return true;
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

  const TriangleMesh* mesh_ = nullptr;
  float cell_size_ = 0.0f;
  Eigen::Vector2f origin_ = Eigen::Vector2f::Zero();
  int nx_ = 0;
  int ny_ = 0;
  int rings_ = 1;
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

bool ConnectedNearPoint(const SafetyContext& ctx, TriangleId a, TriangleId b,
                        const Eigen::Vector3f& point) {
  const TriangleMesh& mesh = *ctx.mesh;
  const TraversabilityParams& trav = *ctx.trav;
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
      if (next == kInvalidId || visited[static_cast<std::size_t>(next)] || !edge.traversable ||
          !(*ctx.allowed)[static_cast<std::size_t>(next)]) {
        continue;
      }
      const TriangleNode& next_triangle = mesh.triangle(next);
      if (!TraversabilityEvaluator::isRobotPlaceable(next_triangle, trav) ||
          !TraversabilityEvaluator::isEdgeTraversable(mesh.triangle(current), next_triangle,
                                                      trav) ||
          TriangleDistance(mesh, next, point) > ctx.project_tolerance) {
        continue;
      }
      if (next == b) return true;
      visited[static_cast<std::size_t>(next)] = 1;
      queue.push_back(next);
    }
  }
  return false;
}

// Locates the nearest mesh triangle (smallest id wins ties) within
// `project_tolerance` using the reusable grid instead of a full mesh scan, and
// accepts only when that triangle is inside the allowed corridor, robot-
// placeable and front-facing. The nearest triangle itself must be placeable: a
// nearby boundary (low clearance) triangle is a rejection, not a reason to snap
// to a farther placeable one. This mirrors SafetyValidator's per-sample gate and
// keeps a sample from hopping across layers/components.
bool NearestPlaceable(const SafetyContext& ctx, const Eigen::Vector3f& point,
                      TriangleId* located) {
  const TriangleMesh& mesh = *ctx.mesh;
  const TraversabilityParams& trav = *ctx.trav;
  TriangleId best = kInvalidId;
  float best_distance = kInf;
  if (ctx.grid == nullptr || !ctx.grid->NearestWithin(point, &best, &best_distance)) {
    return false;
  }
  if (best == kInvalidId || !std::isfinite(best_distance) ||
      best_distance > ctx.project_tolerance) {
    return false;
  }
  // The grid indexes the full mesh so it sees the same nearest triangle as
  // SafetyValidator. A nearest triangle outside the corridor (or non-placeable)
  // is a rejection rather than a reason to snap to a farther placeable one.
  if (ctx.allowed == nullptr || best >= static_cast<TriangleId>(ctx.allowed->size()) ||
      !(*ctx.allowed)[static_cast<std::size_t>(best)]) {
    return false;
  }
  const TriangleNode& triangle = mesh.triangle(best);
  if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) return false;
  // R6: reject the back face so a point under an upper floor cannot snap through
  // the ceiling; max_step_height is the only allowed negative tolerance.
  if (triangle.normal.dot(point - triangle.centroid) < -trav.max_step_height) return false;
  if (located != nullptr) *located = best;
  return true;
}

// Segment sampling projection. A sample that lies on a shared mesh edge/vertex
// has several equidistant triangles; the smallest-id one may only touch the
// previous sample's triangle at a vertex. Such a vertex-only touch is not a
// traversable transition and must not break an otherwise valid segment, so when
// the global nearest is placeable but not edge-adjacent to the previous sample
// we fall back to the nearest placeable edge-neighbour of the previous sample.
bool LocatePlaceable(const SafetyContext& ctx, const Eigen::Vector3f& point,
                     TriangleId previous, TriangleId* located) {
  TriangleId nearest = kInvalidId;
  if (!NearestPlaceable(ctx, point, &nearest)) return false;
  if (previous == kInvalidId || nearest == previous ||
      ConnectedNearPoint(ctx, previous, nearest, point)) {
    *located = nearest;
    return true;
  }
  TriangleId connected = kInvalidId;
  float connected_distance = kInf;
  for (const TriangleEdge& edge : ctx.mesh->triangle(previous).edges) {
    const TriangleId id = edge.neighbor;
    if (id == kInvalidId || !(*ctx.allowed)[static_cast<std::size_t>(id)]) continue;
    if (!TraversabilityEvaluator::isRobotPlaceable(ctx.mesh->triangle(id), *ctx.trav)) continue;
    const float distance = TriangleDistance(*ctx.mesh, id, point);
    if (!std::isfinite(distance) || distance > ctx.project_tolerance) continue;
    if (connected == kInvalidId || distance < connected_distance) {
      connected = id;
      connected_distance = distance;
    }
  }
  if (connected == kInvalidId) return false;
  *located = connected;
  return true;
}

// Samples [a, b] with an interval <= check_step including both endpoints and
// requires every sample to be placeable with traversable edge transitions.
bool SegmentSafe(const SafetyContext& ctx, const Eigen::Vector3f& a,
                 const Eigen::Vector3f& b) {
  const float length = (b - a).norm();
  if (!IsFinite(length)) return false;
  int steps = static_cast<int>(std::ceil(length / ctx.check_step));
  if (steps < 1) steps = 1;
  TriangleId previous = kInvalidId;
  for (int s = 0; s <= steps; ++s) {
    const float t = static_cast<float>(s) / static_cast<float>(steps);
    const Eigen::Vector3f sample = a + (b - a) * t;
    TriangleId current = kInvalidId;
    if (!LocatePlaceable(ctx, sample, previous, &current)) return false;
    if (previous != kInvalidId && current != previous &&
        !ConnectedNearPoint(ctx, previous, current, sample)) {
      return false;
    }
    previous = current;
  }
  return true;
}

// kappa = 2*sin(angle at B)/|AC| == 2*|BA x BC| / (|BA|*|BC|*|AC|).
float CurvatureAt(const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                  const Eigen::Vector3f& c) {
  const Eigen::Vector3f ba = a - b, bc = c - b;
  const float lab = ba.norm(), lbc = bc.norm(), lac = (c - a).norm();
  if (lab <= kDuplicateEpsilon || lbc <= kDuplicateEpsilon || lac <= kDuplicateEpsilon) {
    return 0.0f;
  }
  return 2.0f * ba.cross(bc).norm() / (lab * lbc * lac);
}

float MaxCurvature(const std::vector<Eigen::Vector3f>& points) {
  float maximum = 0.0f;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    maximum = std::max(maximum, CurvatureAt(points[i - 1], points[i], points[i + 1]));
  }
  return maximum;
}

// Greedy visibility simplification used by optimize(). Unlike the public
// shortcut primitive, each chosen chord must also keep the previous retained
// corner within the robot curvature limit. Once a corner is retained this
// makes its curvature final, so later choices cannot invalidate it.
std::vector<Eigen::Vector3f> CurvatureAwareShortcut(
    const std::vector<Eigen::Vector3f>& points, const SafetyContext& ctx,
    float max_curvature) {
  if (points.size() < 2u) return points;
  std::vector<Eigen::Vector3f> out;
  out.reserve(points.size());
  out.push_back(points.front());
  std::size_t i = 0;
  while (i + 1u < points.size()) {
    std::size_t selected = i + 1u;
    bool found = false;
    for (std::size_t j = points.size() - 1u; j > i; --j) {
      if (!SegmentSafe(ctx, points[i], points[j])) continue;
      if (out.size() >= 2u &&
          CurvatureAt(out[out.size() - 2u], points[i], points[j]) > max_curvature) {
        continue;
      }
      selected = j;
      found = true;
      break;
    }
    if (!found && !SegmentSafe(ctx, points[i], points[selected])) return points;
    out.push_back(points[selected]);
    i = selected;
  }
  return out;
}

bool CrossesPortalSequence(const std::vector<Eigen::Vector3f>& path,
                           const std::vector<Portal>& portals) {
  if (portals.empty()) return true;
  if (path.size() < 2u) return false;
  const auto cross2d = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
    return a.x() * b.y() - a.y() * b.x();
  };
  std::size_t segment = 0;
  float previous_t = -1.0f;
  for (const Portal& portal : portals) {
    bool found = false;
    const Eigen::Vector2f c = portal.left.head<2>();
    const Eigen::Vector2f d = portal.right.head<2>();
    while (segment + 1u < path.size()) {
      const Eigen::Vector2f a = path[segment].head<2>();
      const Eigen::Vector2f b = path[segment + 1u].head<2>();
      const Eigen::Vector2f r = b - a;
      const Eigen::Vector2f s = d - c;
      const float denominator = cross2d(r, s);
      if (std::fabs(denominator) > 1e-8f) {
        const float t = cross2d(c - a, s) / denominator;
        const float u = cross2d(c - a, r) / denominator;
        if (t >= -1e-5f && t <= 1.0f + 1e-5f && u >= -1e-5f && u <= 1.0f + 1e-5f &&
            (previous_t < 0.0f || t + 1e-5f >= previous_t)) {
          previous_t = t;
          found = true;
          break;
        }
      }
      ++segment;
      previous_t = -1.0f;
    }
    if (!found) return false;
  }
  return true;
}

std::vector<Eigen::Vector3f> CurvatureConstrainedFallback(
    const std::vector<Eigen::Vector3f>& raw, const SafetyContext& ctx,
    float max_curvature) {
  if (raw.size() < 2u) return {};

  // Endpoints are mandatory; intermediate samples are candidates for the
  // curvature-constrained visibility path.
  std::vector<char> raw_mandatory(raw.size(), 0);
  raw_mandatory.front() = 1;
  raw_mandatory.back() = 1;

  // Keep roughly one candidate every 0.2 m. The fallback
  // remains small enough for a second-order DAG search while preserving the
  // geometry needed to route around corners.
  const float spacing = std::max(4.0f * ctx.check_step, 0.20f);
  std::vector<std::size_t> indices;
  indices.push_back(0u);
  float accumulated = 0.0f;
  for (std::size_t i = 1; i < raw.size(); ++i) {
    accumulated += (raw[i] - raw[i - 1u]).norm();
    if (raw_mandatory[i] || accumulated >= spacing || i + 1u == raw.size()) {
      if (indices.back() != i) indices.push_back(i);
      accumulated = 0.0f;
    }
  }
  if (indices.size() < 2u) return {};

  const std::size_t n = indices.size();
  std::vector<Eigen::Vector3f> points;
  std::vector<char> mandatory(n, 0);
  points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    points.push_back(raw[indices[i]]);
    mandatory[i] = raw_mandatory[indices[i]];
  }

  std::vector<char> safe(n * n, 0);
  for (std::size_t i = 0; i + 1u < n; ++i) {
    for (std::size_t j = i + 1u; j < n; ++j) {
      bool skips_anchor = false;
      for (std::size_t k = i + 1u; k < j; ++k) {
        if (mandatory[k]) {
          skips_anchor = true;
          break;
        }
      }
      if (skips_anchor) break;
      safe[i * n + j] = SegmentSafe(ctx, points[i], points[j]) ? 1 : 0;
    }
  }

  const float inf = kInf;
  std::vector<float> cost(n * n, inf);
  std::vector<int> parent(n * n, -1);
  for (std::size_t j = 1; j < n; ++j) {
    if (safe[j]) cost[j] = (points[j] - points[0]).norm();
  }
  for (std::size_t j = 1; j + 1u < n; ++j) {
    for (std::size_t i = 0; i < j; ++i) {
      const std::size_t state = i * n + j;
      if (!IsFinite(cost[state])) continue;
      for (std::size_t k = j + 1u; k < n; ++k) {
        if (!safe[j * n + k]) continue;
        if (CurvatureAt(points[i], points[j], points[k]) > max_curvature) continue;
        const float candidate = cost[state] + (points[k] - points[j]).norm();
        const std::size_t next = j * n + k;
        if (candidate < cost[next]) {
          cost[next] = candidate;
          parent[next] = static_cast<int>(i);
        }
      }
    }
  }

  std::size_t previous = n;
  float best_cost = inf;
  for (std::size_t i = 0; i + 1u < n; ++i) {
    const float candidate = cost[i * n + (n - 1u)];
    if (candidate < best_cost) {
      best_cost = candidate;
      previous = i;
    }
  }
  if (previous == n) return {};

  std::vector<std::size_t> reversed{n - 1u, previous};
  std::size_t current = n - 1u;
  while (previous != 0u) {
    const int before = parent[previous * n + current];
    if (before < 0) return {};
    current = previous;
    previous = static_cast<std::size_t>(before);
    reversed.push_back(previous);
  }
  std::reverse(reversed.begin(), reversed.end());
  std::vector<Eigen::Vector3f> result;
  result.reserve(reversed.size());
  for (const std::size_t index : reversed) result.push_back(points[index]);
  return result;
}

// T34: collapses consecutive points closer than kDuplicateEpsilon. The exact
// first and last input points are preserved (the last wins over a duplicate of
// it), so a degenerate input can never leave a fillet with an undefined corner
// direction and the output never contains adjacent duplicates.
std::vector<Eigen::Vector3f> CollapseDuplicates(const std::vector<Eigen::Vector3f>& pts) {
  std::vector<Eigen::Vector3f> out;
  out.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const bool last = (i + 1u == pts.size());
    if (!out.empty() && (pts[i] - out.back()).squaredNorm() <=
                             kDuplicateEpsilon * kDuplicateEpsilon) {
      if (last) out.back() = pts[i];
      continue;
    }
    out.push_back(pts[i]);
  }
  return out;
}

void AppendUnique(std::vector<Eigen::Vector3f>* out, const Eigen::Vector3f& point) {
  if (!out->empty() &&
      (point - out->back()).squaredNorm() <= kDuplicateEpsilon * kDuplicateEpsilon) {
    return;
  }
  out->push_back(point);
}

// Builds the allowed mask from the corridor. Returns false on any out-of-range
// id so a malformed corridor can never silently truncate the search.
bool BuildAllowed(const TriangleMesh& mesh, const Corridor& corridor,
                  std::vector<char>* allowed) {
  allowed->assign(mesh.size(), 0);
  for (const TriangleId id : corridor.triangles) {
    if (id < 0 || id >= static_cast<TriangleId>(mesh.size())) return false;
    (*allowed)[static_cast<std::size_t>(id)] = 1;
  }
  return true;
}

// T25 core shared by both optimize overloads. `ordered_portals == nullptr`
// means the frozen signature (funnel deterministically skipped).
Result<std::vector<Eigen::Vector3f>> OptimizeImpl(
    const std::vector<Eigen::Vector3f>& raw, const TriangleMesh& mesh, const Corridor& corridor,
    const std::vector<Portal>* ordered_portals, const PlanParams& params, const RobotParams& robot,
    const TraversabilityParams& trav) {
  if (raw.size() < 2u) return Invalid("path optimization needs at least two points");
  for (const Eigen::Vector3f& point : raw) {
    if (!IsFinite(point)) return Invalid("path optimization point is not finite");
  }
  Status status = ValidatePlanParams(params);
  if (!status.ok()) return status;
  status = ValidateRobotParams(robot);
  if (!status.ok()) return status;
  status = ValidateTraversabilityParams(trav);
  if (!status.ok()) return status;
  if (mesh.size() == 0) return OptimizeFailure("cannot optimize on an empty mesh");

  std::vector<char> allowed;
  if (!BuildAllowed(mesh, corridor, &allowed)) {
    return Invalid("corridor.triangles contains an out-of-range id");
  }
  std::size_t valid_triangles = 0;
  for (const char in_corridor : allowed) {
    if (in_corridor) ++valid_triangles;
  }
  if (valid_triangles == 0u) return OptimizeFailure("corridor has no valid triangles");

  const PathOptimizer optimizer;
  std::vector<Eigen::Vector3f> path = raw;
  MeshGrid grid;
  if (!grid.Build(mesh, allowed, params.max_project_radius)) {
    return OptimizeFailure("corridor has no triangles to index");
  }
  const SafetyContext ctx{&mesh, &allowed, &trav, params.check_step, params.max_project_radius,
                          &grid};
  const auto segments_safe = [&ctx](const std::vector<Eigen::Vector3f>& candidate) {
    if (candidate.size() < 2u) return false;
    for (std::size_t i = 0; i + 1 < candidate.size(); ++i) {
      if (!SegmentSafe(ctx, candidate[i], candidate[i + 1])) return false;
    }
    return true;
  };

  if (params.shortcut_enabled) {
    path = CurvatureAwareShortcut(path, ctx, robot.max_path_curvature);
  }
  if (path.size() < 2u) return OptimizeFailure("shortcut produced fewer than two points");

  // Frozen-signature contract: without Portal geometry the funnel stage is a
  // deterministic no-op. The overload below supplies the ordered Portals. A
  // 2D funnel can cut across a 3D slope transition even when its XY projection
  // is valid, so accept it only when every resulting segment stays on the
  // placeable corridor; otherwise retain the last safe shortcut path.
  if (params.funnel_enabled && ordered_portals != nullptr) {
    std::vector<Eigen::Vector3f> funnelled = optimizer.funnel(path, *ordered_portals);
    if (segments_safe(funnelled)) path = std::move(funnelled);
  }
  if (path.size() < 2u) return OptimizeFailure("funnel produced fewer than two points");

  if (params.smoothing_enabled) {
    path = optimizer.smooth(path, mesh, params, robot, trav);
    // Smoothing can leave several nearly-collinear samples around a bend. A
    // second safety-constrained shortcut pass removes those samples before
    // curvature is evaluated or circular fillets compete for the same short
    // segment. This is still the shortcut -> funnel -> smooth pipeline: the
    // extra pass is cleanup of smoothing output and uses the identical
    // corridor/safety predicate as the primary shortcut stage.
    if (params.shortcut_enabled) {
      std::vector<Eigen::Vector3f> cleaned =
          CurvatureAwareShortcut(path, ctx, robot.max_path_curvature);
      const bool portal_order_ok =
          ordered_portals == nullptr || CrossesPortalSequence(cleaned, *ordered_portals);
      if (portal_order_ok && MaxCurvature(cleaned) <= robot.max_path_curvature) {
        path = std::move(cleaned);
      }
    }
  }
  if (path.size() < 2u) return OptimizeFailure("smoothing produced fewer than two points");

  // T34: round the corners that are still too sharp with curvature-feasible
  // circular arcs. Smoothing removes gentle noise first; a corner that cannot be
  // rounded with a radius at or above the curvature limit is left sharp, so the
  // authoritative curvature gate below rejects it rather than hiding it.
  path = optimizer.fillet(path, mesh, corridor, params, robot, trav);
  if (path.size() < 2u) return OptimizeFailure("filleting produced fewer than two points");

  // Internal full-path safety re-check: sampling + placeability + traversable
  // edges + curvature. If the local pipeline gets trapped by a corner, solve a
  // sparse second-order path problem over the original safe polyline. The
  // hierarchical planner independently verifies Portal crossing order.
  bool safe = segments_safe(path);
  float maximum_curvature = MaxCurvature(path);
  const bool fallback_enabled =
      params.shortcut_enabled || params.funnel_enabled || params.smoothing_enabled;
  if (fallback_enabled && (!safe || maximum_curvature > robot.max_path_curvature)) {
    std::vector<Eigen::Vector3f> fallback =
        CurvatureConstrainedFallback(raw, ctx, robot.max_path_curvature);
    const bool portal_order_ok =
        ordered_portals == nullptr || CrossesPortalSequence(fallback, *ordered_portals);
    if (!fallback.empty() && portal_order_ok && segments_safe(fallback) &&
        MaxCurvature(fallback) <= robot.max_path_curvature) {
      path = std::move(fallback);
      safe = true;
      maximum_curvature = MaxCurvature(path);
    }
  }
  if (!safe) {
    return OptimizeFailure("optimized path fails the internal safety check");
  }
  if (maximum_curvature > robot.max_path_curvature) {
    return OptimizeFailure("optimized path exceeds robot.max_path_curvature");
  }
  return path;
}

}  // namespace

Result<std::vector<Eigen::Vector3f>> PathOptimizer::optimize(
    const std::vector<Eigen::Vector3f>& raw, const TriangleMesh& mesh, const Corridor& corridor,
    const PlanParams& params, const RobotParams& robot,
    const TraversabilityParams& trav) const {
  return OptimizeImpl(raw, mesh, corridor, nullptr, params, robot, trav);
}

Result<std::vector<Eigen::Vector3f>> PathOptimizer::optimize(
    const std::vector<Eigen::Vector3f>& raw, const TriangleMesh& mesh, const Corridor& corridor,
    const std::vector<Portal>& ordered_portals, const PlanParams& params,
    const RobotParams& robot, const TraversabilityParams& trav) const {
  return OptimizeImpl(raw, mesh, corridor, &ordered_portals, params, robot, trav);
}

std::vector<Eigen::Vector3f> PathOptimizer::shortcut(
    const std::vector<Eigen::Vector3f>& pts, const TriangleMesh& mesh, const Corridor& corridor,
    const PlanParams& params, const TraversabilityParams& trav, std::size_t* removed) const {
  if (removed != nullptr) *removed = 0;
  if (pts.size() < 2u) return pts;
  if (!ValidatePlanParams(params).ok() || !ValidateTraversabilityParams(trav).ok()) return pts;
  if (mesh.size() == 0) return pts;
  for (const Eigen::Vector3f& point : pts) {
    if (!IsFinite(point)) return pts;
  }
  std::vector<char> allowed;
  if (!BuildAllowed(mesh, corridor, &allowed)) return pts;

  MeshGrid grid;
  grid.Build(mesh, allowed, params.max_project_radius);
  const SafetyContext ctx{&mesh, &allowed, &trav, params.check_step, params.max_project_radius,
                          &grid};
  std::vector<Eigen::Vector3f> out;
  out.reserve(pts.size());
  out.push_back(pts.front());
  std::size_t i = 0;
  while (i + 1u < pts.size()) {
    std::size_t j = pts.size() - 1u;
    bool found = false;
    while (j > i + 1u) {
      if (SegmentSafe(ctx, pts[i], pts[j])) {
        found = true;
        break;
      }
      --j;
    }
    if (found) {
      if (removed != nullptr) *removed += (j - i - 1u);
      out.push_back(pts[j]);
      i = j;
    } else {
      out.push_back(pts[i + 1u]);
      ++i;
    }
  }
  return out;
}

std::vector<Eigen::Vector3f> PathOptimizer::funnel(
    const std::vector<Eigen::Vector3f>& pts,
    const std::vector<Portal>& ordered_portals) const {
  if (pts.size() < 2u) return pts;
  if (ordered_portals.empty()) return pts;
  const Eigen::Vector3f start = pts.front();
  const Eigen::Vector3f goal = pts.back();
  if (!IsFinite(start) || !IsFinite(goal)) return pts;
  for (const Portal& portal : ordered_portals) {
    if (!IsFinite(portal.left) || !IsFinite(portal.right) || !IsFinite(portal.center)) {
      return pts;
    }
  }

  // Build travel-oriented left/right gates. The stored Portal labels have no
  // guaranteed relation to the direction of travel, so each endpoint is
  // classified by the local forward direction (previous -> next gate center).
  struct Gate {
    Eigen::Vector3f left = Eigen::Vector3f::Zero();
    Eigen::Vector3f right = Eigen::Vector3f::Zero();
  };
  std::vector<Gate> gates;
  gates.reserve(ordered_portals.size() + 1u);
  for (std::size_t i = 0; i < ordered_portals.size(); ++i) {
    const Portal& portal = ordered_portals[i];
    const Eigen::Vector3f previous =
        (i == 0) ? start : ordered_portals[i - 1].center;
    const Eigen::Vector3f next =
        (i + 1 == ordered_portals.size()) ? goal : ordered_portals[i + 1].center;
    Eigen::Vector3f forward = next - previous;
    if (forward.head<2>().squaredNorm() <= kPointEpsilonSq) forward = portal.center - previous;
    if (forward.head<2>().squaredNorm() <= kPointEpsilonSq) forward = goal - start;
    if (forward.head<2>().squaredNorm() <= kPointEpsilonSq) forward = Eigen::Vector3f::UnitX();
    const auto cross_forward = [&](const Eigen::Vector3f& endpoint) {
      return static_cast<double>(forward.x()) *
                 (static_cast<double>(endpoint.y()) - portal.center.y()) -
             static_cast<double>(forward.y()) *
                 (static_cast<double>(endpoint.x()) - portal.center.x());
    };
    Gate gate;
    // The endpoint with the larger signed cross is to the left of the travel
    // direction; ties fall back to the stored order deterministically.
    if (cross_forward(portal.left) >= cross_forward(portal.right)) {
      gate.left = portal.left;
      gate.right = portal.right;
    } else {
      gate.left = portal.right;
      gate.right = portal.left;
    }
    gates.push_back(gate);
  }
  // A degenerate goal gate lets the funnel detect that the goal lies outside
  // the current cone and collapse to the correct boundary point.
  gates.push_back(Gate{goal, goal});

  const auto cross2d = [](const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                          const Eigen::Vector3f& c) {
    return (static_cast<double>(b.x()) - a.x()) * (static_cast<double>(c.y()) - a.y()) -
           (static_cast<double>(b.y()) - a.y()) * (static_cast<double>(c.x()) - a.x());
  };
  const auto same = [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return (a - b).squaredNorm() <= kPointEpsilonSq;
  };

  std::vector<Eigen::Vector3f> out;
  out.push_back(start);
  Eigen::Vector3f apex = start, left = start, right = start;
  std::size_t apex_index = 0, left_index = 0, right_index = 0;
  for (std::size_t i = 0; i < gates.size(); ++i) {
    const Eigen::Vector3f new_left = gates[i].left;
    const Eigen::Vector3f new_right = gates[i].right;

    // A fresh cone after the start / after a collapse: both boundaries come
    // from the same gate, so no crossing test applies.
    if (same(apex, left) && same(apex, right)) {
      right = new_right;
      right_index = i;
      left = new_left;
      left_index = i;
      continue;
    }

    bool restarted = false;
    // Right boundary: crossing the left ray collapses the funnel onto the left
    // vertex; otherwise tighten inward only.
    if (!same(apex, left) && cross2d(apex, left, new_right) > kAreaEps) {
      out.push_back(left);
      apex = left;
      apex_index = left_index;
      left = apex;
      right = apex;
      left_index = right_index = apex_index;
      i = apex_index;
      restarted = true;
    } else if (same(apex, right) || cross2d(apex, right, new_right) >= -kAreaEps) {
      right = new_right;
      right_index = i;
    }
    if (restarted) continue;  // for-loop ++i restarts just after the apex

    // Left boundary: crossing the right ray collapses onto the right vertex.
    if (!same(apex, right) && cross2d(apex, right, new_left) < -kAreaEps) {
      out.push_back(right);
      apex = right;
      apex_index = right_index;
      left = apex;
      right = apex;
      left_index = right_index = apex_index;
      i = apex_index;
      continue;
    }
    if (same(apex, left) || cross2d(apex, left, new_left) <= kAreaEps) {
      left = new_left;
      left_index = i;
    }
  }
  out.push_back(goal);

  // Collapse consecutive duplicates but never drop below two points.
  std::vector<Eigen::Vector3f> deduped;
  deduped.reserve(out.size());
  for (const Eigen::Vector3f& point : out) {
    if (deduped.empty() ||
        (point - deduped.back()).squaredNorm() > kDuplicateEpsilon * kDuplicateEpsilon) {
      deduped.push_back(point);
    }
  }
  if (deduped.size() < 2u) {
    deduped.clear();
    deduped.push_back(start);
    deduped.push_back(goal);
  }
  return deduped;
}

std::vector<Eigen::Vector3f> PathOptimizer::smooth(const std::vector<Eigen::Vector3f>& pts,
                                                   const TriangleMesh& mesh,
                                                   const PlanParams& params,
                                                   const RobotParams& robot,
                                                   const TraversabilityParams& trav) const {
  if (pts.size() < 3u) return pts;
  if (!ValidatePlanParams(params).ok() || !ValidateRobotParams(robot).ok() ||
      !ValidateTraversabilityParams(trav).ok()) {
    return pts;
  }
  if (mesh.size() == 0) return pts;
  for (const Eigen::Vector3f& point : pts) {
    if (!IsFinite(point)) return pts;
  }
  if (params.smoothing_iterations <= 0) return pts;

  const std::vector<char> allowed(mesh.size(), 1);
  MeshGrid grid;
  grid.Build(mesh, allowed, params.max_project_radius);
  const SafetyContext ctx{&mesh, &allowed, &trav, params.check_step, params.max_project_radius,
                          &grid};

  std::vector<Eigen::Vector3f> current = pts;
  for (int iteration = 0; iteration < params.smoothing_iterations; ++iteration) {
    std::vector<Eigen::Vector3f> next = current;
    for (std::size_t i = 1; i + 1 < current.size(); ++i) {
      // 0.5-weight Laplacian: p_i += 0.5 * (avg(neighbours) - p_i).
      const Eigen::Vector3f neighbour_average = 0.5f * (current[i - 1] + current[i + 1]);
      const Eigen::Vector3f candidate = 0.5f * current[i] + 0.5f * neighbour_average;
      TriangleId located = kInvalidId;
      if (!NearestPlaceable(ctx, candidate, &located)) continue;
      if (!SegmentSafe(ctx, current[i - 1], candidate)) continue;
      if (!SegmentSafe(ctx, candidate, current[i + 1])) continue;
      next[i] = candidate;
    }

    // Curvature gate: revert (to the previous iteration) every point whose
    // triple exceeds the limit. Repeated to a fixed point so reverting one point
    // cannot leave a new violation behind. Points are never deleted.
    int guard = 0;
    const int max_guard = static_cast<int>(current.size()) + 1;
    bool changed = true;
    while (changed && guard++ < max_guard) {
      changed = false;
      for (std::size_t i = 1; i + 1 < next.size(); ++i) {
        if (CurvatureAt(next[i - 1], next[i], next[i + 1]) > robot.max_path_curvature &&
            next[i] != current[i]) {
          next[i] = current[i];
          changed = true;
        }
      }
    }
    current = std::move(next);
  }
  return current;
}

std::vector<Eigen::Vector3f> PathOptimizer::fillet(const std::vector<Eigen::Vector3f>& pts,
                                                   const TriangleMesh& mesh,
                                                   const Corridor& corridor,
                                                   const PlanParams& params,
                                                   const RobotParams& robot,
                                                   const TraversabilityParams& trav) const {
  if (pts.size() < 3u) return pts;
  if (!ValidatePlanParams(params).ok() || !ValidateRobotParams(robot).ok() ||
      !ValidateTraversabilityParams(trav).ok()) {
    return pts;
  }
  if (mesh.size() == 0) return pts;
  for (const Eigen::Vector3f& point : pts) {
    if (!IsFinite(point)) return pts;
  }

  std::vector<Eigen::Vector3f> path = CollapseDuplicates(pts);
  if (path.size() < 3u) return path;

  std::vector<char> allowed;
  if (!BuildAllowed(mesh, corridor, &allowed)) return pts;
  MeshGrid grid;
  if (!grid.Build(mesh, allowed, params.max_project_radius)) return path;
  const SafetyContext ctx{&mesh, &allowed, &trav, params.check_step, params.max_project_radius,
                          &grid};

  // Radius is at or above the curvature limit; the small margin keeps the
  // sampled arc's discrete curvature strictly inside robot.max_path_curvature.
  const float radius = (1.0f + kFilletRadiusMargin) / robot.max_path_curvature;
  if (!IsFinite(radius) || !(radius > 0.0f)) return path;

  const std::size_t n = path.size();
  std::vector<Eigen::Vector3f> direction(n - 1u);
  std::vector<float> length(n - 1u, 0.0f);
  for (std::size_t s = 0; s + 1u < n; ++s) {
    direction[s] = path[s + 1u] - path[s];
    length[s] = direction[s].norm();
    if (!IsFinite(length[s]) || length[s] <= kDuplicateEpsilon) return path;
    direction[s] /= length[s];
  }

  // Desired tangent distance R * tan(phi / 2) for every violating bend. A
  // corner already inside the curvature limit is left exactly as it is; only a
  // genuine bend that exceeds the limit is rounded.
  std::vector<float> turn(n, 0.0f);
  std::vector<float> tangent(n, 0.0f);
  for (std::size_t i = 1; i + 1u < n; ++i) {
    if (CurvatureAt(path[i - 1u], path[i], path[i + 1u]) <= robot.max_path_curvature) continue;
    const float dot = clampf(direction[i - 1u].dot(direction[i]), -1.0f, 1.0f);
    const float cross = direction[i - 1u].cross(direction[i]).norm();
    const float phi = std::atan2(cross, dot);
    // Collinear corners need no arc; a U-turn needs an infinite tangent.
    if (phi <= kFilletMinTurn || phi >= kFilletMaxTurn) continue;
    const float t = radius * std::tan(0.5f * phi);
    if (!IsFinite(t) || t <= 0.0f) continue;
    turn[i] = phi;
    tangent[i] = t;
  }

  // Neighbouring fillets on the same segment must not overlap. A tangent budget
  // that exactly fills the segment is legal (the two tangent points coincide, so
  // no straight section remains); only a genuine overlap is a conflict. Nothing
  // shrinks the radius; the corner that cannot fit is left sharp instead. The
  // smaller tangent is dropped first (ties drop the right corner) so the sharper
  // neighbour keeps its room; a corner whose tangent alone exceeds its segment is
  // dropped too. Each pass clears at least one tangent, so this terminates.
  bool changed = true;
  int guard = 0;
  const int max_guard = static_cast<int>(n) + 1;
  while (changed && guard++ < max_guard) {
    changed = false;
    for (std::size_t s = 0; s + 1u < n; ++s) {
      const float left = (s >= 1u) ? tangent[s] : 0.0f;
      const float right = (s + 2u < n) ? tangent[s + 1u] : 0.0f;
      // Absorb float round-off so an exact tangent fit is not mistaken for an
      // overlap. The slack is a few ULPs of the segment length.
      const float slack = kFilletSegmentSlackScale *
                          std::numeric_limits<float>::epsilon() * std::max(length[s], 1.0f);
      if (left + right <= length[s] + slack) continue;
      const bool has_left = left > 0.0f;
      const bool has_right = right > 0.0f;
      if (has_left && has_right && left < right) {
        tangent[s] = 0.0f;
        turn[s] = 0.0f;
      } else if (has_right) {
        tangent[s + 1u] = 0.0f;
        turn[s + 1u] = 0.0f;
      } else if (has_left) {
        tangent[s] = 0.0f;
        turn[s] = 0.0f;
      }
      changed = true;
    }
  }

  // Angular step whose chord is at most plan.check_step. When check_step exceeds
  // the diameter the arc is sampled at least twice so a very tight (but legal)
  // arc is never collapsed to a single straight chord.
  const float arc_angle_step =
      2.0f * std::asin(clampf(params.check_step / (2.0f * radius), 0.0f, 1.0f));

  std::vector<Eigen::Vector3f> out;
  out.reserve(std::min<std::size_t>(kFilletMaxPoints, n * 4u));
  AppendUnique(&out, path.front());
  for (std::size_t i = 1; i + 1u < n; ++i) {
    if (!(tangent[i] > 0.0f)) {
      AppendUnique(&out, path[i]);
      continue;
    }
    const Eigen::Vector3f u = direction[i - 1u];
    const Eigen::Vector3f v = direction[i];
    const float t = tangent[i];
    Eigen::Vector3f normal = u.cross(v);
    const float normal_length = normal.norm();
    if (!IsFinite(normal_length) || normal_length <= kDuplicateEpsilon) {
      AppendUnique(&out, path[i]);
      continue;
    }
    normal /= normal_length;

    const Eigen::Vector3f t1 = path[i] - t * u;
    const Eigen::Vector3f t2 = path[i] + t * v;
    const Eigen::Vector3f center = t1 + radius * normal.cross(u);
    const float sweep = turn[i];

    int samples = 1;
    if (arc_angle_step > 1e-6f) {
      samples = static_cast<int>(std::ceil(sweep / arc_angle_step));
    }
    if (samples < 2) samples = 2;
    // Deterministic output bound: an arc that would exceed it is skipped rather
    // than sampled coarser than plan.check_step.
    if (out.size() + static_cast<std::size_t>(samples) + 2u > kFilletMaxPoints) {
      AppendUnique(&out, path[i]);
      continue;
    }

    // e1 = (t1 - center) / radius, e2 = u; (e1, e2, normal) is right-handed and
    // the arc turns by +sweep about `normal`, so its start tangent is u.
    const Eigen::Vector3f e1 = -(normal.cross(u));
    std::vector<Eigen::Vector3f> arc;
    arc.reserve(static_cast<std::size_t>(samples) + 1u);
    arc.push_back(t1);
    for (int k = 1; k < samples; ++k) {
      const float angle = sweep * static_cast<float>(k) / static_cast<float>(samples);
      arc.push_back(center + radius * (std::cos(angle) * e1 + std::sin(angle) * u));
    }
    arc.push_back(t2);

    // Every new straight and arc segment must stay on the placeable corridor
    // with traversable transitions; otherwise the corner stays sharp.
    bool safe = SegmentSafe(ctx, out.back(), arc.front());
    for (std::size_t k = 0; safe && k + 1u < arc.size(); ++k) {
      safe = SegmentSafe(ctx, arc[k], arc[k + 1u]);
    }
    if (safe) safe = SegmentSafe(ctx, arc.back(), path[i + 1u]);
    if (!safe) {
      AppendUnique(&out, path[i]);
      continue;
    }
    for (const Eigen::Vector3f& point : arc) AppendUnique(&out, point);
  }
  AppendUnique(&out, path.back());
  if (out.size() < 2u) return path;
  return out;
}

}  // namespace topmap
