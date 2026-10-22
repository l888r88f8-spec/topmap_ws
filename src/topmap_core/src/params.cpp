#include "topmap_core/params.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <set>
#include <string>

namespace topmap {
namespace {

constexpr float kPi = 3.14159265358979323846f;

Result<float> req(const YAML::Node& n, const std::string& path) {
  if (!n || !n.IsScalar()) return Status::Error(ErrorCode::kInvalidArgument, "missing key " + path);
  try {
    return n.as<float>();
  } catch (const YAML::Exception& e) {
    return Status::Error(ErrorCode::kInvalidArgument, path + ": " + e.what());
  }
}

Result<int> reqInt(const YAML::Node& n, const std::string& path) {
  auto f = req(n, path);
  if (!f.ok()) return f.status();
  return static_cast<int>(std::lround(f.value()));
}

Result<bool> reqBool(const YAML::Node& n, const std::string& path) {
  if (!n || !n.IsScalar()) return Status::Error(ErrorCode::kInvalidArgument, "missing key " + path);
  try {
    return n.as<bool>();
  } catch (const YAML::Exception& e) {
    return Status::Error(ErrorCode::kInvalidArgument, path + ": " + e.what());
  }
}

// A topmap section must be a mapping; a scalar/sequence node would make the
// later operator[] calls throw YAML::BadSubscript instead of returning a
// structured error naming the offending path.
Status requireMap(const YAML::Node& n, const std::string& path) {
  if (!n) return Status::Error(ErrorCode::kInvalidArgument, "missing key " + path);
  if (!n.IsMap()) return Status::Error(ErrorCode::kInvalidArgument, path + ": expected mapping");
  return Status::Ok();
}

// R1: reject unknown keys (including removed derived keys) so a stale config
// cannot silently appear to take effect.
Status rejectUnknown(const YAML::Node& section, const std::string& prefix,
                     const std::set<std::string>& allowed) {
  if (!section || !section.IsMap()) return Status::Ok();
  for (const auto& kv : section) {
    std::string key;
    try {
      key = kv.first.as<std::string>();
    } catch (const YAML::Exception& e) {
      return Status::Error(ErrorCode::kInvalidArgument, prefix + ": " + e.what());
    }
    if (allowed.find(key) == allowed.end())
      return Status::Error(ErrorCode::kInvalidArgument, "unknown key " + prefix + "." + key);
  }
  return Status::Ok();
}

#define REQ(dst, node, key, path) { auto r_ = req((node)[key], path "." key); if (!r_.ok()) return r_.status(); dst = r_.value(); }
#define REQ_I(dst, node, key, path) { auto r_ = reqInt((node)[key], path "." key); if (!r_.ok()) return r_.status(); dst = r_.value(); }
#define REQ_B(dst, node, key, path) { auto r_ = reqBool((node)[key], path "." key); if (!r_.ok()) return r_.status(); dst = r_.value(); }

}  // namespace

float RobotParams::max_slope_rad() const { return max_slope_deg * kPi / 180.0f; }
float RobotParams::required_clearance() const { return 0.5f * width + safety_distance; }
float RobotParams::required_width() const { return width + 2.0f * safety_distance; }
float TraversabilityParams::max_slope_rad() const { return max_slope_deg * kPi / 180.0f; }
float TraversabilityParams::max_normal_diff_rad() const { return max_normal_diff_deg * kPi / 180.0f; }
float TraversabilityParams::max_slope_diff_rad() const { return max_slope_diff_deg * kPi / 180.0f; }

namespace {
Result<TopmapConfig> loadTopmapConfigImpl(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    return Status::Error(ErrorCode::kInvalidArgument, e.what());
  }
  if (!root.IsMap())
    return Status::Error(ErrorCode::kInvalidArgument, "topmap: expected mapping");
  YAML::Node t = root["topmap"];
  if (auto s = requireMap(t, "topmap"); !s.ok()) return s;
  TopmapConfig c;
  YAML::Node n;

  n = t["robot"];
  if (auto s = requireMap(n, "topmap.robot"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.robot",
      {"width", "length", "max_slope_deg", "max_step_height", "safety_distance",
       "max_path_curvature"}); !s.ok()) return s;
  REQ(c.robot.width, n, "width", "topmap.robot");
  REQ(c.robot.length, n, "length", "topmap.robot");
  REQ(c.robot.max_slope_deg, n, "max_slope_deg", "topmap.robot");
  REQ(c.robot.max_step_height, n, "max_step_height", "topmap.robot");
  REQ(c.robot.safety_distance, n, "safety_distance", "topmap.robot");
  REQ(c.robot.max_path_curvature, n, "max_path_curvature", "topmap.robot");

  n = t["map"];
  if (auto s = requireMap(n, "topmap.map"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.map",
      {"voxel_size", "outlier_k", "outlier_std_ratio", "normal_search_radius", "max_edge_length",
       "max_normal_deviation_deg", "prefer_layered_triangulation", "keep_raw_cloud"}); !s.ok()) return s;
  REQ(c.map.voxel_size, n, "voxel_size", "topmap.map");
  REQ_I(c.map.outlier_k, n, "outlier_k", "topmap.map");
  REQ(c.map.outlier_std_ratio, n, "outlier_std_ratio", "topmap.map");
  REQ(c.map.normal_search_radius, n, "normal_search_radius", "topmap.map");
  REQ(c.map.max_edge_length, n, "max_edge_length", "topmap.map");
  REQ(c.map.max_normal_deviation_deg, n, "max_normal_deviation_deg", "topmap.map");
  REQ_B(c.map.prefer_layered_triangulation, n, "prefer_layered_triangulation", "topmap.map");
  REQ_B(c.map.keep_raw_cloud, n, "keep_raw_cloud", "topmap.map");

  n = t["traversability"];
  if (auto s = requireMap(n, "topmap.traversability"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.traversability",
      {"max_normal_diff_deg", "max_slope_diff_deg", "allow_slope_transition", "max_roughness",
       "max_surface_curvature", "min_triangle_area"}); !s.ok()) return s;
  REQ(c.traversability.max_normal_diff_deg, n, "max_normal_diff_deg", "topmap.traversability");
  REQ(c.traversability.max_slope_diff_deg, n, "max_slope_diff_deg", "topmap.traversability");
  REQ_B(c.traversability.allow_slope_transition, n, "allow_slope_transition", "topmap.traversability");
  REQ(c.traversability.max_roughness, n, "max_roughness", "topmap.traversability");
  REQ(c.traversability.max_surface_curvature, n, "max_surface_curvature", "topmap.traversability");
  REQ(c.traversability.min_triangle_area, n, "min_triangle_area", "topmap.traversability");

  n = t["region"];
  if (auto s = requireMap(n, "topmap.region"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.region",
      {"normal_weight", "slope_weight", "roughness_weight", "height_weight", "region_score_threshold",
       "max_height_span", "slope_regime_deg", "height_span_across_regime_only", "max_slope_variance",
       "min_region_area", "min_region_triangles", "bottleneck_ratio", "bottleneck_abs_ceiling",
       "min_side_area", "concavity_branch_min_arm_length", "concavity_branch_angle_deg",
       "ramp_slope_deg"}); !s.ok()) return s;
  REQ(c.region.normal_weight, n, "normal_weight", "topmap.region");
  REQ(c.region.slope_weight, n, "slope_weight", "topmap.region");
  REQ(c.region.roughness_weight, n, "roughness_weight", "topmap.region");
  REQ(c.region.height_weight, n, "height_weight", "topmap.region");
  REQ(c.region.region_score_threshold, n, "region_score_threshold", "topmap.region");
  REQ(c.region.max_height_span, n, "max_height_span", "topmap.region");
  REQ(c.region.slope_regime_deg, n, "slope_regime_deg", "topmap.region");
  REQ_B(c.region.height_span_across_regime_only, n, "height_span_across_regime_only", "topmap.region");
  REQ(c.region.max_slope_variance, n, "max_slope_variance", "topmap.region");
  REQ(c.region.min_region_area, n, "min_region_area", "topmap.region");
  REQ_I(c.region.min_region_triangles, n, "min_region_triangles", "topmap.region");
  REQ(c.region.bottleneck_ratio, n, "bottleneck_ratio", "topmap.region");
  REQ(c.region.bottleneck_abs_ceiling, n, "bottleneck_abs_ceiling", "topmap.region");
  REQ(c.region.min_side_area, n, "min_side_area", "topmap.region");
  REQ(c.region.concavity_branch_min_arm_length, n, "concavity_branch_min_arm_length", "topmap.region");
  REQ(c.region.concavity_branch_angle_deg, n, "concavity_branch_angle_deg", "topmap.region");
  REQ(c.region.ramp_slope_deg, n, "ramp_slope_deg", "topmap.region");

  n = t["portal"];
  if (auto s = requireMap(n, "topmap.portal"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.portal",
      {"boundary_min_edges", "width_margin"}); !s.ok()) return s;
  REQ_I(c.portal.boundary_min_edges, n, "boundary_min_edges", "topmap.portal");
  REQ(c.portal.width_margin, n, "width_margin", "topmap.portal");

  n = t["compression"];
  if (auto s = requireMap(n, "topmap.compression"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.compression",
      {"max_merge_cost", "preserve_topology", "forbid_cross_regime_merge"}); !s.ok()) return s;
  REQ(c.compression.max_merge_cost, n, "max_merge_cost", "topmap.compression");
  REQ_B(c.compression.preserve_topology, n, "preserve_topology", "topmap.compression");
  REQ_B(c.compression.forbid_cross_regime_merge, n, "forbid_cross_regime_merge", "topmap.compression");

  n = t["plan"];
  if (auto s = requireMap(n, "topmap.plan"); !s.ok()) return s;
  if (auto s = rejectUnknown(n, "topmap.plan",
      {"max_project_radius", "path_surface_tolerance", "max_project_normal_diff_deg",
       "corridor_expansion_rings", "fallback_full_mesh", "shortcut_enabled", "funnel_enabled",
       "smoothing_enabled", "smoothing_iterations", "check_step", "topology_weight"}); !s.ok())
    return s;
  REQ(c.plan.max_project_radius, n, "max_project_radius", "topmap.plan");
  // Backward-compatible incremental key: an older config that omits it keeps the
  // safe default instead of being rejected, while an explicit value is read and
  // range-checked like every other planning parameter.
  if (n["path_surface_tolerance"]) {
    REQ(c.plan.path_surface_tolerance, n, "path_surface_tolerance", "topmap.plan");
  }
  REQ(c.plan.max_project_normal_diff_deg, n, "max_project_normal_diff_deg", "topmap.plan");
  REQ_I(c.plan.corridor_expansion_rings, n, "corridor_expansion_rings", "topmap.plan");
  REQ_B(c.plan.fallback_full_mesh, n, "fallback_full_mesh", "topmap.plan");
  REQ_B(c.plan.shortcut_enabled, n, "shortcut_enabled", "topmap.plan");
  REQ_B(c.plan.funnel_enabled, n, "funnel_enabled", "topmap.plan");
  REQ_B(c.plan.smoothing_enabled, n, "smoothing_enabled", "topmap.plan");
  REQ_I(c.plan.smoothing_iterations, n, "smoothing_iterations", "topmap.plan");
  REQ(c.plan.check_step, n, "check_step", "topmap.plan");
  REQ(c.plan.topology_weight, n, "topology_weight", "topmap.plan");

  // R1: backfill derived quantities; the single source of truth is robot.
  c.traversability.max_slope_deg = c.robot.max_slope_deg;
  c.traversability.max_step_height = c.robot.max_step_height;
  c.traversability.required_clearance = c.robot.required_clearance();

  // Spec section 8: reject out-of-range values.
  if (c.robot.width <= 0.0f || c.robot.length <= 0.0f || c.robot.safety_distance < 0.0f ||
      c.robot.max_slope_deg <= 0.0f || c.robot.max_slope_deg >= 90.0f ||
      c.robot.max_step_height < 0.0f || c.robot.max_path_curvature <= 0.0f)
    return Status::Error(ErrorCode::kInvalidArgument, "topmap.robot: value out of range");
  if (c.map.voxel_size <= 0.0f || c.map.max_edge_length <= 0.0f)
    return Status::Error(ErrorCode::kInvalidArgument, "topmap.map: value out of range");
  if (c.region.bottleneck_ratio <= 0.0f || c.region.bottleneck_ratio >= 1.0f)
    return Status::Error(ErrorCode::kInvalidArgument, "topmap.region.bottleneck_ratio must be in (0,1)");
  if (c.region.region_score_threshold < 0.0f || c.region.region_score_threshold > 1.0f)
    return Status::Error(ErrorCode::kInvalidArgument,
                         "topmap.region.region_score_threshold must be in [0,1]");  // R1 normalized
  if (c.plan.check_step <= 0.0f)
    return Status::Error(ErrorCode::kInvalidArgument, "topmap.plan.check_step must be positive");
  if (!std::isfinite(c.plan.path_surface_tolerance) || c.plan.path_surface_tolerance <= 0.0f)
    return Status::Error(ErrorCode::kInvalidArgument,
                         "topmap.plan.path_surface_tolerance must be finite and positive");
  return c;
}
}  // namespace

Result<TopmapConfig> loadTopmapConfig(const std::string& path) {
  // Final boundary: no YAML::Exception may escape into the caller.
  try {
    return loadTopmapConfigImpl(path);
  } catch (const YAML::Exception& e) {
    return Status::Error(ErrorCode::kInvalidArgument, e.what());
  }
}

Result<RobotParams> loadRobotParams(const std::string& path) {
  auto c = loadTopmapConfig(path);
  if (!c.ok()) return c.status();
  return c.value().robot;
}

}  // namespace topmap
