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
  if (!IsFinite(params.path_surface_tolerance) || !(params.path_surface_tolerance > 0.0f)) {
    return Invalid("plan.path_surface_tolerance must be finite and positive");
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
  // Radius used only to resolve shared-vertex/fan ambiguity between two samples
  // whose own nearest triangles are already within project_tolerance. Never used
  // to accept a sample that is farther than project_tolerance from the surface.
  float connect_tolerance = 0.30f;
};

float TriangleDistance(const TriangleMesh& mesh, TriangleId id, const Eigen::Vector3f& p) {
  const TriangleNode& triangle = mesh.triangle(id);
  return pointTriangleDistance(p, mesh.vertices()[triangle.vertex_ids[0]],
                               mesh.vertices()[triangle.vertex_ids[1]],
                               mesh.vertices()[triangle.vertex_ids[2]]);
}

// Per-axis XY AABB expansion required by ContainsXy's relative edge epsilon.
//
// ContainsXy accepts the union of two barycentric half-space branches:
//   { lambda_i >= -delta } and { lambda_i <= +delta },
// where delta = epsilon / |2*area| and epsilon = 1e-6 * max(1, |2*area|). The
// old epsilon / min_edge bound ignored the fact that a point can be accepted
// just beyond a very acute vertex while staying within epsilon of both incident
// edge lines; for a wedge with width W that overhang is up to 2*delta*W.
// The same bound covers both branches: the second branch is only non-empty when
// |2*area| <= 3*epsilon (delta >= 1/3), and its accepted points also stay within
// 2*delta*W of the AABB. This is deliberately conservative and proven from the
// barycentric constraints, not from the shortest edge.
struct XyMargin {
  float x = 0.0f;
  float y = 0.0f;
  bool pathological = false;
};

XyMargin XyContainmentMargin(const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                             const Eigen::Vector3f& c) {
  const Eigen::Vector2d a2(a.x(), a.y());
  const Eigen::Vector2d b2(b.x(), b.y());
  const Eigen::Vector2d c2(c.x(), c.y());
  const double area2 = (b2 - a2).x() * (c2 - a2).y() - (b2 - a2).y() * (c2 - a2).x();
  if (!std::isfinite(area2) || std::fabs(area2) <= 1e-12) return XyMargin{};
  const double epsilon = 1e-6 * std::max(1.0, std::fabs(area2));
  const double delta = epsilon / std::fabs(area2);
  const double width_x = std::max(a2.x(), std::max(b2.x(), c2.x())) -
                         std::min(a2.x(), std::min(b2.x(), c2.x()));
  const double width_y = std::max(a2.y(), std::max(b2.y(), c2.y())) -
                         std::min(a2.y(), std::min(b2.y(), c2.y()));
  // Small slack absorbs float rounding when converting the expanded AABB back
  // to integer cell coordinates.
  const double margin_x = 2.0 * delta * width_x + 1e-6;
  const double margin_y = 2.0 * delta * width_y + 1e-6;
  XyMargin margin;
  if (!std::isfinite(margin_x) || !std::isfinite(margin_y) || margin_x > 1e30 ||
      margin_y > 1e30) {
    // A near-degenerate triangle can make the proven bound unbounded. The grid
    // then falls back to a complete per-query scan instead of allocating an
    // enormous cell range or silently dropping the triangle.
    margin.pathological = true;
    return margin;
  }
  margin.x = static_cast<float>(margin_x);
  margin.y = static_cast<float>(margin_y);
  return margin;
}

// XY uniform grid over the allowed triangles, shared by every sample of a call.
// The cell size is the median triangle edge length of the indexed set, so it is
// derived from the mesh resolution rather than any scene constant. Each triangle
// is registered in every cell its exact XY AABB overlaps; a nearest query then
// visits its own cell plus a ring of neighbours wide enough that any triangle
// within `project_tolerance` of the point lies in a visited cell. The nearest
// candidate is chosen by the true 3D point-triangle distance, so the result is
// identical to a full scan (smallest id wins exact ties). XY containment queries
// additionally inspect the 3x3 neighborhood for ContainsXy's small exterior
// epsilon, with larger-margin triangles kept in a separate overflow list.
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
    full_scan_ = false;
    cell_size_ = 0.0f;
    origin_.setZero();
    nx_ = 0;
    ny_ = 0;
    rings_ = 1;
    entries_.clear();
    cell_begin_.clear();
    xy_overflow_.clear();

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

    // Rasterise each triangle's exact XY AABB into its covering cell range for
    // nearest-point queries. The registration count is bounded before allocating
    // anything; a pathological exact AABB (a single face spanning a huge number
    // of cells) switches the whole grid to a complete per-query scan rather than
    // exhausting memory or dropping a triangle.
    struct Range {
      int ix0 = 0, ix1 = 0, iy0 = 0, iy1 = 0;
    };
    const std::size_t registration_limit =
        std::max<std::size_t>(64u * indexed, 1u << 20);
    double registrations = 0.0;
    bool full_scan = false;
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
      registrations +=
          static_cast<double>(r.ix1 - r.ix0 + 1) * static_cast<double>(r.iy1 - r.iy0 + 1);
      if (registrations > static_cast<double>(registration_limit)) {
        full_scan = true;
        break;
      }
      // ContainsXy may accept points just outside the exact XY AABB. The proven
      // margin is normally at most one cell, so ForEachCandidateAtXy can find
      // those triangles by querying the 3x3 cell neighborhood. Only triangles
      // whose proven margin is larger (or unbounded) need the small overflow
      // list; they stay registered by their exact AABB for nearest queries.
      const XyMargin margin = XyContainmentMargin(a, b, c);
      if (margin.pathological || margin.x > cell_size_ || margin.y > cell_size_) {
        xy_overflow_.push_back(id);
      }
    }
    if (full_scan) {
      full_scan_ = true;
      cell_size_ = 0.0f;
      nx_ = 0;
      ny_ = 0;
      entries_.clear();
      cell_begin_.clear();
      xy_overflow_.clear();
      return true;
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

  bool empty() const {
    return mesh_ == nullptr || (!full_scan_ && (cell_size_ <= 0.0f || nx_ <= 0 || ny_ <= 0));
  }

  // True nearest allowed triangle to `point`; ties go to the smallest id like
  // the ascending-id full scan it replaces. Rings expand outward and stop as
  // soon as no unvisited cell can hold a closer triangle: after searching the
  // square of radius r, every unvisited cell is at XY distance >= r*cell, so a
  // best distance strictly below that bound is already the global nearest.
  bool NearestWithin(const Eigen::Vector3f& point, TriangleId* located, float* distance) const {
    if (empty()) return false;
    if (full_scan_) {
      TriangleId best = kInvalidId;
      float best_distance = kInf;
      for (TriangleId id = 0; id < static_cast<TriangleId>(mesh_->size()); ++id) {
        const float d = TriangleDistance(*mesh_, id, point);
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

  // Enumerates a complete superset of the triangles ContainsXy can accept at
  // `point`. Normal triangles are indexed by their exact XY AABB; because their
  // proven containment margin is at most one cell, the 3x3 cell neighborhood
  // around the query is sufficient. Triangles with a larger or pathological
  // margin are always tested from the small overflow list. IDs are collected,
  // sorted and de-duplicated so the caller sees the same ascending-id order as
  // the full scan this replaces.
  template <typename Visitor>
  void ForEachCandidateAtXy(const Eigen::Vector3f& point, Visitor&& visitor) const {
    if (empty()) return;
    if (full_scan_) {
      for (TriangleId id = 0; id < static_cast<TriangleId>(mesh_->size()); ++id) {
        visitor(id);
      }
      return;
    }
    std::vector<TriangleId> candidates;
    candidates.reserve(xy_overflow_.size() + 32u);
    const int cx = CellX(point.x());
    const int cy = CellY(point.y());
    for (int dy = -1; dy <= 1; ++dy) {
      const int y = cy + dy;
      if (y < 0 || y >= ny_) continue;
      const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(nx_);
      for (int dx = -1; dx <= 1; ++dx) {
        const int x = cx + dx;
        if (x < 0 || x >= nx_) continue;
        const std::size_t c = row + static_cast<std::size_t>(x);
        for (int k = cell_begin_[c]; k < cell_begin_[c + 1u]; ++k) {
          candidates.push_back(entries_[static_cast<std::size_t>(k)]);
        }
      }
    }
    candidates.insert(candidates.end(), xy_overflow_.begin(), xy_overflow_.end());
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    for (const TriangleId id : candidates) visitor(id);
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
  bool full_scan_ = false;
  float cell_size_ = 0.0f;
  Eigen::Vector2f origin_ = Eigen::Vector2f::Zero();
  int nx_ = 0;
  int ny_ = 0;
  int rings_ = 1;
  std::vector<TriangleId> entries_;  // CSR payload
  std::vector<int> cell_begin_;      // CSR offsets, size nx*ny + 1
  // Triangles whose proven XY containment margin exceeds one cell. They remain
  // in the exact-AABB nearest index above and are additionally tested for every
  // XY containment query.
  std::vector<TriangleId> xy_overflow_;
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
          TriangleDistance(mesh, next, point) > ctx.connect_tolerance) {
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
    if (!std::isfinite(distance) || distance > ctx.connect_tolerance) continue;
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

// Ground-steering curvature: the yaw curvature the vehicle actually has to
// steer, measured in the XY travel plane. Following a real mesh crease produces
// a 3D pitch corner (flat <-> ramp) that must not be reported as a steering
// violation. A genuine left/right turn still has the same 2*sin(angle)/|AC|
// value as the 3D formula because both triangles are horizontal projections of
// the incident headings. A purely vertical segment has no steering direction and
// contributes zero.
float CurvatureAt(const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                  const Eigen::Vector3f& c) {
  const Eigen::Vector2f ba(a.x() - b.x(), a.y() - b.y());
  const Eigen::Vector2f bc(c.x() - b.x(), c.y() - b.y());
  const Eigen::Vector2f ca(c.x() - a.x(), c.y() - a.y());
  const float lab = ba.norm(), lbc = bc.norm(), lac = ca.norm();
  if (lab <= kDuplicateEpsilon || lbc <= kDuplicateEpsilon || lac <= kDuplicateEpsilon) {
    return 0.0f;
  }
  return 2.0f * std::fabs(ba.x() * bc.y() - ba.y() * bc.x()) / (lab * lbc * lac);
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
    float max_curvature, const std::vector<char>* mandatory = nullptr) {
  if (points.size() < 2u) return points;
  // Prefix counts make the "does this chord skip a mandatory crease?" test O(1)
  // instead of rescanning every interior index for every candidate j. When no
  // mandatory mask is supplied, the test is skipped entirely.
  std::vector<std::size_t> mandatory_prefix;
  if (mandatory != nullptr) {
    mandatory_prefix.resize(points.size() + 1u, 0u);
    for (std::size_t k = 0; k < points.size(); ++k) {
      mandatory_prefix[k + 1u] =
          mandatory_prefix[k] +
          ((k < mandatory->size() && (*mandatory)[k] != 0) ? 1u : 0u);
    }
  }
  std::vector<Eigen::Vector3f> out;
  out.reserve(points.size());
  out.push_back(points.front());
  std::size_t i = 0;
  while (i + 1u < points.size()) {
    std::size_t selected = i + 1u;
    bool found = false;
    for (std::size_t j = points.size() - 1u; j > i; --j) {
      // A shortcut may never skip a mandatory terrain crease. The crease point
      // itself may be a destination, so only indices strictly between i and j
      // are checked. Prefix counts answer that in O(1).
      if (!mandatory_prefix.empty() &&
          mandatory_prefix[j] > mandatory_prefix[i + 1u]) {
        continue;
      }
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

float Cross2d(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
  return a.x() * b.y() - a.y() * b.x();
}

// True when the XY projection of `p` lies inside triangle (a,b,c). The small
// relative epsilon only absorbs float round-off at shared edges; it is not a
// lateral snapping tolerance, so a sample over a hole is never accepted.
bool ContainsXy(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                const Eigen::Vector3f& c) {
  const Eigen::Vector2f p2(p.x(), p.y());
  const Eigen::Vector2f a2(a.x(), a.y());
  const Eigen::Vector2f b2(b.x(), b.y());
  const Eigen::Vector2f c2(c.x(), c.y());
  const float area = Cross2d(b2 - a2, c2 - a2);
  if (!IsFinite(area) || std::fabs(area) <= 1e-12f) return false;
  const float epsilon = 1e-6f * std::max(1.0f, std::fabs(area));
  const float d1 = Cross2d(b2 - a2, p2 - a2);
  const float d2 = Cross2d(c2 - b2, p2 - b2);
  const float d3 = Cross2d(a2 - c2, p2 - c2);
  const bool outside_negative = d1 < -epsilon || d2 < -epsilon || d3 < -epsilon;
  const bool outside_positive = d1 > epsilon || d2 > epsilon || d3 > epsilon;
  return !(outside_negative && outside_positive);
}

// Vertical projection of `p` onto the plane of triangle (a,b,c): the returned
// point keeps p.x/p.y exactly and only resolves the surface height. A near-
// vertical triangle has no single-valued height and is rejected.
bool VerticalSurfaceZ(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                      const Eigen::Vector3f& b, const Eigen::Vector3f& c, float* z) {
  const Eigen::Vector3f normal = (b - a).cross(c - a);
  const float nz = normal.z();
  if (!IsFinite(nz) || std::fabs(nz) <= 1e-9f) return false;
  const float value = a.z() - (normal.x() * (p.x() - a.x()) + normal.y() * (p.y() - a.y())) / nz;
  if (!IsFinite(value)) return false;
  *z = value;
  return true;
}

// Locates the surface height at `sample`'s XY inside the allowed corridor. The
// query never moves laterally, so a chord over a hole cannot be "repaired" by
// snapping to the nearest boundary triangle. Continuity with `previous` keeps
// overlapping floors/components from being exchanged: only the same triangle or
// a traversably edge-adjacent one is accepted.
bool LocateSurfaceAtXy(const MeshGrid& grid, const TriangleMesh& mesh,
                       const std::vector<char>& allowed, const TraversabilityParams& trav,
                       const Eigen::Vector3f& sample, TriangleId previous,
                       float max_search_radius, TriangleId* located,
                       Eigen::Vector3f* surface_point, int required_component = -1) {
  TriangleId best = kInvalidId;
  float best_distance = kInf;
  int best_continuity = 3;
  Eigen::Vector3f best_point = Eigen::Vector3f::Zero();
  // The grid enumerates a complete superset of the containing triangles; every
  // acceptance, continuity, distance and tie rule below is unchanged from the
  // full ascending-id scan it replaces.
  grid.ForEachCandidateAtXy(sample, [&](const TriangleId id) {
    if (!allowed[static_cast<std::size_t>(id)]) return;
    if (required_component >= 0 && mesh.componentId(id) != required_component) return;
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) return;
    const Eigen::Vector3f& a = mesh.vertices()[triangle.vertex_ids[0]];
    const Eigen::Vector3f& b = mesh.vertices()[triangle.vertex_ids[1]];
    const Eigen::Vector3f& c = mesh.vertices()[triangle.vertex_ids[2]];
    if (!ContainsXy(sample, a, b, c)) return;
    if (triangle.normal.dot(sample - triangle.centroid) < -trav.max_step_height) return;
    float z = 0.0f;
    if (!VerticalSurfaceZ(sample, a, b, c, &z)) return;
    const Eigen::Vector3f point(sample.x(), sample.y(), z);
    if (triangle.normal.dot(point - triangle.centroid) < -trav.max_step_height) return;
    const float distance = std::fabs(z - sample.z());
    if (!IsFinite(distance) || distance > max_search_radius) return;

    int continuity = 0;
    if (previous != kInvalidId) {
      if (id == previous) {
        continuity = 0;
      } else if (SharesTraversableEdge(mesh, trav, previous, id)) {
        continuity = 1;
      } else {
        // A sample can straddle a mesh vertex where the two containing
        // triangles are separated by a local fan of triangles. Accept that only
        // when the fan is traversable and sits near the same sample, exactly
        // like the optimizer's own sample-projection logic. This does not permit
        // a hole or layer jump because the query XY must still be contained by
        // the candidate triangle (checked above).
        const SafetyContext local{&mesh, &allowed, &trav, 0.05f, max_search_radius, nullptr};
        if (!ConnectedNearPoint(local, previous, id, point)) return;
        continuity = 2;
      }
    }
    if (best == kInvalidId || continuity < best_continuity ||
        (continuity == best_continuity &&
         (distance < best_distance || (distance == best_distance && id < best)))) {
      best = id;
      best_continuity = continuity;
      best_distance = distance;
      best_point = point;
    }
  });
  if (best == kInvalidId) return false;
  if (located != nullptr) *located = best;
  if (surface_point != nullptr) *surface_point = best_point;
  return true;
}

// Sampling interval for the surface lift. The bound keeps the chord sag across
// the steepest allowed slope below path_surface_tolerance; check_step is never
// exceeded.
float SurfaceLiftStep(const PlanParams& params, const TraversabilityParams& trav) {
  const float slope_tan =
      std::max(std::tan(std::min(trav.max_slope_rad(), kPiF * 0.5f - 1e-3f)), 1e-3f);
  const float tolerance_step = 2.0f * params.path_surface_tolerance / slope_tan;
  float step = std::min(params.check_step, tolerance_step);
  if (!IsFinite(step) || step < 1e-3f) step = 1e-3f;
  return step;
}

// Converts a guide polyline into both the dense per-triangle surface path and
// the sparse path that keeps only the guide's own lifted vertices. Both outputs
// need the same sampling sequence and continuity state, so they are collected in
// one traversal: the dense path is what guarantees strict adherence across a
// slope change, while the sparse path preserves the caller's guide geometry
// (including a required Portal transition) whenever its chords are already
// within tolerance. Sampling, duplicate removal and continuity are byte-for-byte
// the previous dense and sparse passes, just not repeated.
bool LiftGuideToSurface(const std::vector<Eigen::Vector3f>& guide, const TriangleMesh& mesh,
                        const std::vector<char>& allowed, const TraversabilityParams& trav,
                        const PlanParams& params, const MeshGrid& grid,
                        std::vector<Eigen::Vector3f>* dense,
                        std::vector<Eigen::Vector3f>* sparse) {
  dense->clear();
  sparse->clear();
  if (guide.size() < 2u) return false;
  const float step = SurfaceLiftStep(params, trav);
  const float duplicate_sq = kDuplicateEpsilon * kDuplicateEpsilon;
  TriangleId previous = kInvalidId;

  for (std::size_t i = 0; i + 1u < guide.size(); ++i) {
    const Eigen::Vector3f& a = guide[i];
    const Eigen::Vector3f& b = guide[i + 1u];
    const float length = (b - a).norm();
    if (!IsFinite(length)) return false;
    int samples = static_cast<int>(std::ceil(length / step));
    if (samples < 1) samples = 1;
    const int first = (i == 0u) ? 0 : 1;
    for (int s = first; s <= samples; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(samples);
      TriangleId located = kInvalidId;
      Eigen::Vector3f point = Eigen::Vector3f::Zero();
      if (!LocateSurfaceAtXy(grid, mesh, allowed, trav, a + (b - a) * t, previous,
                             params.max_project_radius, &located, &point)) {
        return false;
      }
      previous = located;
      if (dense->empty() || (point - dense->back()).squaredNorm() > duplicate_sq) {
        dense->push_back(point);
      }
      if ((s == samples || (i == 0u && s == 0)) &&
          (sparse->empty() || (point - sparse->back()).squaredNorm() > duplicate_sq)) {
        sparse->push_back(point);
      }
    }
  }
  return dense->size() >= 2u;
}

// Terrain crease edges are the shared mesh edges whose two adjacent, allowed,
// placeable triangles have different face planes. The threshold only rejects
// numerical plane noise and coplanar triangulation diagonals; a real slope
// transition is orders of magnitude larger.
constexpr float kCreaseNormalCrossEpsilon = 1e-3f;  // sin(~0.057 deg)
constexpr double kCreaseParameterEpsilon = 1e-6;
constexpr std::size_t kMaxCreaseHitsPerSegment = 64u;
constexpr std::size_t kMaxCreaseCandidatePoints = 2048u;

struct CreaseEdge {
  Eigen::Vector3f a = Eigen::Vector3f::Zero();
  Eigen::Vector3f b = Eigen::Vector3f::Zero();
};

struct CreaseHit {
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  double t = 0.0;
};

// Collects only edges the candidate may legally cross: both triangles in the
// corridor, robot-placeable, same component/layer, shared edge traversable, and
// the two face planes genuinely different. Boundary edges are ignored.
void CollectCreaseEdges(const TriangleMesh& mesh, const std::vector<char>& allowed,
                        const TraversabilityParams& trav, int component,
                        std::vector<CreaseEdge>* edges) {
  edges->clear();
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    if (!allowed[static_cast<std::size_t>(id)]) continue;
    if (mesh.componentId(id) != component) continue;
    const TriangleNode& triangle = mesh.triangle(id);
    if (!TraversabilityEvaluator::isRobotPlaceable(triangle, trav)) continue;
    for (const TriangleEdge& edge : triangle.edges) {
      if (edge.neighbor == kInvalidId || edge.neighbor < id) continue;
      if (edge.neighbor >= static_cast<TriangleId>(allowed.size()) ||
          !allowed[static_cast<std::size_t>(edge.neighbor)]) {
        continue;
      }
      if (mesh.componentId(edge.neighbor) != component) continue;
      const TriangleNode& neighbour = mesh.triangle(edge.neighbor);
      if (!TraversabilityEvaluator::isRobotPlaceable(neighbour, trav)) continue;
      if (!edge.traversable ||
          !TraversabilityEvaluator::isEdgeTraversable(triangle, neighbour, trav)) {
        continue;
      }
      if (triangle.normal.cross(neighbour.normal).norm() <= kCreaseNormalCrossEpsilon) {
        continue;
      }
      CreaseEdge crease;
      crease.a = mesh.vertices()[edge.v0];
      crease.b = mesh.vertices()[edge.v1];
      edges->push_back(crease);
    }
  }
}

// Intersects the guide segment's XY projection with one real edge segment. The
// emitted point is interpolated from the actual mesh edge endpoints, so it lies
// on the crease to float round-off. A guide that runs along a crease is
// ambiguous (there is no single crossing); the caller then uses the old dense
// fallback instead of guessing.
bool IntersectCrease(const Eigen::Vector3f& from, const Eigen::Vector3f& to,
                     const CreaseEdge& edge, CreaseHit* hit, bool* ambiguous) {
  const Eigen::Vector2d p(from.x(), from.y());
  const Eigen::Vector2d r(to.x() - from.x(), to.y() - from.y());
  const Eigen::Vector2d e0(edge.a.x(), edge.a.y());
  const Eigen::Vector2d s(edge.b.x() - edge.a.x(), edge.b.y() - edge.a.y());
  const double r_length = r.norm();
  const double s_length = s.norm();
  if (!(r_length > 0.0) || !(s_length > 0.0)) return false;
  const double denominator = r.x() * s.y() - r.y() * s.x();
  const double parallel_epsilon = 1e-12 * std::max(1.0, r_length * s_length);
  if (!std::isfinite(denominator) || std::fabs(denominator) <= parallel_epsilon) {
    const Eigen::Vector2d offset = e0 - p;
    const double line_offset = offset.x() * r.y() - offset.y() * r.x();
    if (std::fabs(line_offset) <= 1e-6 * std::max(1.0, r_length)) {
      *ambiguous = true;
    }
    return false;
  }
  const Eigen::Vector2d offset = e0 - p;
  const double t = (offset.x() * s.y() - offset.y() * s.x()) / denominator;
  const double u = (offset.x() * r.y() - offset.y() * r.x()) / denominator;
  if (t < -kCreaseParameterEpsilon || t > 1.0 + kCreaseParameterEpsilon ||
      u < -kCreaseParameterEpsilon || u > 1.0 + kCreaseParameterEpsilon) {
    return false;
  }
  const double clamped_t = std::clamp(t, 0.0, 1.0);
  const double clamped_u = std::clamp(u, 0.0, 1.0);
  const Eigen::Vector3d a(edge.a.x(), edge.a.y(), edge.a.z());
  const Eigen::Vector3d b(edge.b.x(), edge.b.y(), edge.b.z());
  const Eigen::Vector3d point = a + clamped_u * (b - a);
  if (!point.allFinite()) return false;
  hit->point = point.cast<float>();
  hit->t = clamped_t;
  return true;
}

// Appends in path order while preserving the exact crease point. If a projected
// guide vertex is a near-duplicate, the crease point replaces it and stays
// marked mandatory for later simplification.
void AppendCandidatePoint(std::vector<Eigen::Vector3f>* points, std::vector<char>* mandatory,
                          const Eigen::Vector3f& point, bool crease) {
  if (!points->empty() &&
      (point - points->back()).squaredNorm() <= kDuplicateEpsilon * kDuplicateEpsilon) {
    if (crease) {
      points->back() = point;
      (*mandatory)[mandatory->size() - 1u] = 1;
    }
    return;
  }
  points->push_back(point);
  mandatory->push_back(crease ? 1 : 0);
}

// Fast candidate: the guide's own surface-projected vertices plus the exact
// intersections of each guide segment with real terrain crease edges. It never
// samples triangle interiors or coplanar grid diagonals. Guide vertices are
// projected within the already-selected component (never across a layer or
// component); if a projection, crease intersection, or count is ambiguous, the
// caller keeps the existing dense lift as the fallback rather than dropping the
// candidate silently.
bool BuildCreasePreservingPath(const std::vector<Eigen::Vector3f>& guide,
                               const TriangleMesh& mesh, const std::vector<char>& allowed,
                               const TraversabilityParams& trav, const PlanParams& params,
                               const MeshGrid& grid, std::vector<Eigen::Vector3f>* out,
                               std::vector<char>* mandatory) {
  out->clear();
  mandatory->clear();
  if (guide.size() < 2u) return false;

  TriangleId start_triangle = kInvalidId;
  Eigen::Vector3f start_point = Eigen::Vector3f::Zero();
  if (!LocateSurfaceAtXy(grid, mesh, allowed, trav, guide.front(), kInvalidId,
                         params.max_project_radius, &start_triangle, &start_point)) {
    return false;
  }
  const int component = mesh.componentId(start_triangle);
  std::vector<CreaseEdge> creases;
  CollectCreaseEdges(mesh, allowed, trav, component, &creases);
  AppendCandidatePoint(out, mandatory, start_point, false);

  for (std::size_t i = 0; i + 1u < guide.size(); ++i) {
    std::vector<CreaseHit> hits;
    bool ambiguous = false;
    for (const CreaseEdge& edge : creases) {
      CreaseHit hit;
      if (!IntersectCrease(guide[i], guide[i + 1u], edge, &hit, &ambiguous)) continue;
      hits.push_back(hit);
      if (hits.size() > kMaxCreaseHitsPerSegment) return false;
    }
    if (ambiguous) return false;
    std::sort(hits.begin(), hits.end(), [](const CreaseHit& lhs, const CreaseHit& rhs) {
      if (lhs.t != rhs.t) return lhs.t < rhs.t;
      if (lhs.point.x() != rhs.point.x()) return lhs.point.x() < rhs.point.x();
      if (lhs.point.y() != rhs.point.y()) return lhs.point.y() < rhs.point.y();
      return lhs.point.z() < rhs.point.z();
    });

    TriangleId end_triangle = kInvalidId;
    Eigen::Vector3f end_point = Eigen::Vector3f::Zero();
    if (!LocateSurfaceAtXy(grid, mesh, allowed, trav, guide[i + 1u], kInvalidId,
                           params.max_project_radius, &end_triangle, &end_point,
                           component)) {
      return false;
    }
    for (const CreaseHit& hit : hits) {
      AppendCandidatePoint(out, mandatory, hit.point, true);
    }
    AppendCandidatePoint(out, mandatory, end_point, false);
    if (out->size() > kMaxCreaseCandidatePoints) return false;
  }
  return out->size() >= 2u && mandatory->size() == out->size();
}

bool PathSurfaceSafe(const std::vector<Eigen::Vector3f>& path, const SafetyContext& ctx,
                     float max_curvature) {
  if (path.size() < 2u) return false;
  for (std::size_t i = 0; i + 1u < path.size(); ++i) {
    if (!SegmentSafe(ctx, path[i], path[i + 1u])) return false;
  }
  return MaxCurvature(path) <= max_curvature;
}

// Final production stage: lift the candidate onto the actual corridor mesh and
// simplify it with the strict surface tolerance, never the 0.30 m endpoint
// radius. Both the dense lift and the simplified result must pass the complete
// segment test (interior samples, placeability, traversable transitions) and the
// ground-steering curvature limit.
bool BuildSurfaceConstrainedPath(const std::vector<Eigen::Vector3f>& guide,
                                 const TriangleMesh& mesh, const std::vector<char>& allowed,
                                 const PlanParams& params, const RobotParams& robot,
                                 const TraversabilityParams& trav,
                                 const std::vector<Portal>* ordered_portals,
                                 std::vector<Eigen::Vector3f>* out) {
  // One grid serves both the surface lift's XY containment queries and the
  // strict segment checks below. It is built before the lift so sampling never
  // falls back to a whole-mesh scan.
  MeshGrid grid;
  if (!grid.Build(mesh, allowed, params.path_surface_tolerance)) return false;
  const float connect_tolerance =
      std::min(params.max_project_radius, std::max(params.path_surface_tolerance,
                                                  2.0f * params.check_step));
  const SafetyContext ctx{&mesh, &allowed, &trav, params.check_step,
                          params.path_surface_tolerance, &grid, connect_tolerance};
  const auto portal_order_ok = [ordered_portals](const std::vector<Eigen::Vector3f>& path) {
    return ordered_portals == nullptr || CrossesPortalSequence(path, *ordered_portals);
  };
  // The fast crease candidate is only accepted after the existing strict lift
  // succeeds on that exact candidate. This keeps the original fine XY
  // containment/layer coverage and SurfaceLiftStep sampling: an endpoint chord
  // over a small hole cannot pass merely because both endpoints are on the mesh.
  const auto lifts_to_surface = [&](const std::vector<Eigen::Vector3f>& candidate) {
    std::vector<Eigen::Vector3f> temporary_dense;
    std::vector<Eigen::Vector3f> temporary_sparse;
    return LiftGuideToSurface(candidate, mesh, allowed, trav, params, grid,
                              &temporary_dense, &temporary_sparse);
  };

  // Fast path for the common flat -> ramp -> platform case: project the guide's
  // own vertices and insert the exact intersections with real terrain crease
  // edges. Simplification is allowed but can never skip a crease point. If any
  // part of this candidate is ambiguous or unsafe, fall through to the existing
  // dense lift unchanged; a candidate is never silently dropped or accepted on
  // a looser tolerance.
  std::vector<Eigen::Vector3f> crease_path;
  std::vector<char> crease_mandatory;
  if (BuildCreasePreservingPath(guide, mesh, allowed, trav, params, grid, &crease_path,
                                &crease_mandatory)) {
    if (params.shortcut_enabled && crease_path.size() > 2u) {
      std::vector<Eigen::Vector3f> simplified =
          CurvatureAwareShortcut(crease_path, ctx, robot.max_path_curvature,
                                 &crease_mandatory);
      if (lifts_to_surface(simplified) &&
          PathSurfaceSafe(simplified, ctx, robot.max_path_curvature) &&
          portal_order_ok(simplified)) {
        *out = std::move(simplified);
        return true;
      }
    }
    if (lifts_to_surface(crease_path) &&
        PathSurfaceSafe(crease_path, ctx, robot.max_path_curvature) &&
        portal_order_ok(crease_path)) {
      *out = std::move(crease_path);
      return true;
    }
  }

  std::vector<Eigen::Vector3f> dense;
  std::vector<Eigen::Vector3f> sparse;
  if (!LiftGuideToSurface(guide, mesh, allowed, trav, params, grid, &dense, &sparse)) {
    return false;
  }
  const bool sparse_ok = sparse.size() >= 2u;
  // Prefer the shortest path that passes every strict segment/Portal check.
  // Shortcutting remains configurable. A sparse lift of the original guide is
  // preferred over a fully dense lift when its straight segments are already
  // within tolerance: this keeps a feasible Portal corner without inventing a
  // sample-dependent curvature violation, while PathSurfaceSafe still checks
  // every segment interior. The dense path is the fallback for a guide segment
  // that bridges a slope change or otherwise needs intermediate surface points.
  if (params.shortcut_enabled) {
    std::vector<Eigen::Vector3f> simplified =
        CurvatureAwareShortcut(dense, ctx, robot.max_path_curvature);
    if (PathSurfaceSafe(simplified, ctx, robot.max_path_curvature) &&
        portal_order_ok(simplified)) {
      *out = std::move(simplified);
      return true;
    }
  }
  if (sparse_ok && PathSurfaceSafe(sparse, ctx, robot.max_path_curvature) &&
      portal_order_ok(sparse)) {
    *out = std::move(sparse);
    return true;
  }
  if (PathSurfaceSafe(dense, ctx, robot.max_path_curvature) && portal_order_ok(dense)) {
    *out = std::move(dense);
    return true;
  }
  return false;
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

  // Final production stage: every returned segment must hug the real corridor
  // mesh. Shortcut/funnel/smoothing are still free to move the guide, but their
  // floating chords are now lifted onto the mesh and re-simplified under the
  // strict path_surface_tolerance (never the 0.30 m endpoint radius). If the
  // optimized guide cannot be lifted safely, the original surface A* polyline is
  // lifted instead; a path is never accepted on the old 0.30 m tolerance.
  const bool fallback_enabled =
      params.shortcut_enabled || params.funnel_enabled || params.smoothing_enabled;
  std::vector<Eigen::Vector3f> surface_path;
  bool safe = BuildSurfaceConstrainedPath(path, mesh, allowed, params, robot, trav,
                                          ordered_portals, &surface_path);
  if (!safe && fallback_enabled) {
    // Prefer a curvature-constrained visibility path that is already restricted
    // to the production surface tolerance. The regular fallback below keeps the
    // wider endpoint-projection radius for slope transitions, but on branching
    // scenes that radius lets a loose chord shortcut XY outside the corridor.
    SafetyContext strict_ctx = ctx;
    strict_ctx.project_tolerance = params.path_surface_tolerance;
    strict_ctx.connect_tolerance =
        std::min(params.max_project_radius,
                 std::max(params.path_surface_tolerance, 2.0f * params.check_step));
    // The final surface lift samples finer than check_step wherever the slope
    // bound requires it. Match that interval here so an edge cannot hide a
    // clearance violation between two coarse visibility samples.
    strict_ctx.check_step = std::min(params.check_step, SurfaceLiftStep(params, trav));
    std::vector<Eigen::Vector3f> strict_fallback =
        CurvatureConstrainedFallback(raw, strict_ctx, robot.max_path_curvature);
    const bool strict_portal_order_ok =
        ordered_portals == nullptr ||
        CrossesPortalSequence(strict_fallback, *ordered_portals);
    if (!strict_fallback.empty() && strict_portal_order_ok) {
      safe = BuildSurfaceConstrainedPath(strict_fallback, mesh, allowed, params, robot, trav,
                                         ordered_portals, &surface_path);
    }
  }
  if (!safe && fallback_enabled) {
    // A second-order fallback over the original polyline, still lifted onto the
    // real surface before it can become the result.
    std::vector<Eigen::Vector3f> fallback =
        CurvatureConstrainedFallback(raw, ctx, robot.max_path_curvature);
    const bool portal_order_ok =
        ordered_portals == nullptr || CrossesPortalSequence(fallback, *ordered_portals);
    if (!fallback.empty() && portal_order_ok) {
      safe = BuildSurfaceConstrainedPath(fallback, mesh, allowed, params, robot, trav,
                                         ordered_portals, &surface_path);
    }
  }
  if (!safe) {
    // Last attempt: lift the untouched surface A* polyline. This is still the
    // production optimizer's result path, not a bypass that returns the raw
    // floating guide.
    safe = BuildSurfaceConstrainedPath(raw, mesh, allowed, params, robot, trav,
                                       ordered_portals, &surface_path);
  }
  if (!safe) {
    return OptimizeFailure("optimized path cannot be constrained to the surface");
  }
  path = std::move(surface_path);
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
