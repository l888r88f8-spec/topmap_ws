#include "topmap_topology/region_growing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "topmap_core/geometry.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap {
namespace {

// The frozen A4.7 `grow` signature carries RegionParams only. Placeability and
// score normalisation therefore use the A4.4 default TraversabilityParams,
// which is the contract for a mesh processed by the default evaluator.
const TraversabilityParams& defaultTraversability() {
  static const TraversabilityParams kDefault{};
  return kDefault;
}

float normalizedJoinScore(const TriangleNode& a, const TriangleNode& b, const RegionParams& p,
                          const TraversabilityParams& tp) {
  const float dn = angleBetween(a.normal, b.normal) / std::max(tp.max_normal_diff_rad(), 1e-6f);
  const float ds = std::fabs(a.slope - b.slope) / std::max(tp.max_slope_diff_rad(), 1e-6f);
  const float dr = std::fabs(a.roughness - b.roughness) / std::max(tp.max_roughness, 1e-6f);
  const float dh =
      TraversabilityEvaluator::heightJump(a, b) / std::max(tp.max_step_height, 1e-6f);
  return p.normal_weight * dn + p.slope_weight * ds + p.roughness_weight * dr +
         p.height_weight * dh;
}

// Incremental area-weighted statistics used both for the homogeneity checks and
// for the final SurfaceRegion fields.
struct Accumulator {
  double area = 0.0;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double sum_slope = 0.0;
  double sum_slope2 = 0.0;
  double sum_roughness = 0.0;
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  float max_slope = 0.0f;
  bool has_flat = false;
  bool has_ramp = false;
};

void accumulate(Accumulator& acc, const TriangleNode& t, float slope_regime_rad) {
  const double a = static_cast<double>(t.area);
  acc.area += a;
  acc.centroid += a * t.centroid.cast<double>();
  acc.sum_slope += a * t.slope;
  acc.sum_slope2 += a * t.slope * t.slope;
  acc.sum_roughness += a * t.roughness;
  acc.min_z = std::min(acc.min_z, t.centroid.z());
  acc.max_z = std::max(acc.max_z, t.centroid.z());
  acc.max_slope = std::max(acc.max_slope, t.slope);
  if (t.slope > slope_regime_rad) {
    acc.has_ramp = true;
  } else {
    acc.has_flat = true;
  }
}

double slopeVariance(const Accumulator& acc) {
  if (!(acc.area > 0.0)) return 0.0;
  const double mean = acc.sum_slope / acc.area;
  const double variance = acc.sum_slope2 / acc.area - mean * mean;
  return variance > 0.0 ? variance : 0.0;
}

SurfaceRegion makeRegion(RegionId id, const TriangleMesh& mesh,
                         const std::vector<TriangleId>& triangle_ids, float slope_regime_rad,
                         float ramp_slope_rad) {
  Accumulator acc;
  for (TriangleId t : triangle_ids) {
    accumulate(acc, mesh.triangle(t), slope_regime_rad);
  }
  SurfaceRegion region;
  region.id = id;
  region.triangle_ids = triangle_ids;
  if (acc.area > 0.0) {
    region.centroid = (acc.centroid / acc.area).cast<float>();
    region.mean_slope = static_cast<float>(acc.sum_slope / acc.area);
    region.roughness = static_cast<float>(acc.sum_roughness / acc.area);
  }
  region.area = static_cast<float>(acc.area);
  region.min_z = acc.min_z;
  region.max_z = acc.max_z;
  region.max_slope = acc.max_slope;
  region.slope_variance = static_cast<float>(slopeVariance(acc));
  region.is_ramp = region.mean_slope > ramp_slope_rad;
  return region;
}

}  // namespace

float regionJoinScore(const TriangleNode& a, const TriangleNode& b, const RegionParams& p) {
  return normalizedJoinScore(a, b, p, defaultTraversability());
}

Result<std::vector<SurfaceRegion>> RegionGrower::grow(const TriangleMesh& mesh,
                                                      const RegionParams& params) const {
  const TraversabilityParams& tp = defaultTraversability();
  const float slope_regime_rad = params.slope_regime_deg * 3.14159265358979323846f / 180.0f;
  const float ramp_slope_rad = params.ramp_slope_deg * 3.14159265358979323846f / 180.0f;

  const TriangleId count = static_cast<TriangleId>(mesh.size());
  std::vector<char> placeable(count, 0);
  for (TriangleId id = 0; id < count; ++id) {
    placeable[id] =
        TraversabilityEvaluator::isRobotPlaceable(mesh.triangle(id), tp) ? 1 : 0;
  }
  std::vector<char> assigned(count, 0);

  // SR1: descending area, ascending id for determinism.
  std::vector<TriangleId> seed_order;
  seed_order.reserve(count);
  for (TriangleId id = 0; id < count; ++id) {
    if (placeable[id]) seed_order.push_back(id);
  }
  std::sort(seed_order.begin(), seed_order.end(), [&mesh](TriangleId a, TriangleId b) {
    const float area_a = mesh.triangle(a).area;
    const float area_b = mesh.triangle(b).area;
    if (area_a > area_b) return true;
    if (area_a < area_b) return false;
    return a < b;
  });

  using Item = std::pair<float, TriangleId>;
  std::vector<std::vector<TriangleId>> grown;

  for (TriangleId seed : seed_order) {
    if (assigned[seed]) continue;
    assigned[seed] = 1;
    std::vector<TriangleId> region{seed};
    Accumulator acc;
    accumulate(acc, mesh.triangle(seed), slope_regime_rad);

    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> frontier;
    const auto push_neighbors = [&](TriangleId from) {
      for (const TriangleEdge& e : mesh.triangle(from).edges) {
        const TriangleId to = e.neighbor;
        if (to == kInvalidId || assigned[to] || !placeable[to] || !e.traversable) continue;
        frontier.push({normalizedJoinScore(mesh.triangle(from), mesh.triangle(to), params, tp),
                       to});
      }
    };
    push_neighbors(seed);

    while (!frontier.empty()) {
      const Item item = frontier.top();
      frontier.pop();
      const TriangleId candidate = item.second;
      if (assigned[candidate] || !placeable[candidate]) continue;
      const TriangleNode& t = mesh.triangle(candidate);
      if (item.first >= params.region_score_threshold) continue;

      // R2: the height-span constraint only guards a change of slope regime, so
      // a long ramp is never fragmented by it.
      const bool crosses_regime =
          (acc.has_flat && t.slope > slope_regime_rad) || (acc.has_ramp && t.slope <= slope_regime_rad);
      const bool check_height = !params.height_span_across_regime_only || crosses_regime;
      const float new_min_z = std::min(acc.min_z, t.centroid.z());
      const float new_max_z = std::max(acc.max_z, t.centroid.z());
      if (check_height && (new_max_z - new_min_z) > params.max_height_span) continue;

      Accumulator next = acc;
      accumulate(next, t, slope_regime_rad);
      if (slopeVariance(next) > params.max_slope_variance) continue;

      assigned[candidate] = 1;
      region.push_back(candidate);
      acc = next;
      push_neighbors(candidate);
    }
    grown.push_back(std::move(region));
  }

  std::vector<SurfaceRegion> regions;
  for (std::vector<TriangleId>& ids : grown) {
    if (static_cast<int>(ids.size()) < params.min_region_triangles) continue;
    SurfaceRegion region = makeRegion(static_cast<RegionId>(regions.size()), mesh, ids,
                                      slope_regime_rad, ramp_slope_rad);
    if (region.area < params.min_region_area) continue;
    std::sort(region.triangle_ids.begin(), region.triangle_ids.end());
    regions.push_back(std::move(region));
  }

  if (regions.empty()) {
    return Status::Error(ErrorCode::kRegionGrowingFailed,
                         "no robot-placeable region met the size thresholds");
  }
  return regions;
}

}  // namespace topmap
