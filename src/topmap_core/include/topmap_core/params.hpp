#pragma once
#include <string>

#include "topmap_core/error.hpp"

namespace topmap {

struct RobotParams {
  float width = 0.60f;  float length = 0.80f;  float max_slope_deg = 20.0f;
  float max_step_height = 0.05f; float safety_distance = 0.15f;
  float max_path_curvature = 0.25f;          // 1/m, only path-curvature yardstick
  float max_slope_rad() const;
  // Single clearance yardstick: robot inscribed radius + safety distance.
  float required_clearance() const;          // 0.5*width + safety_distance = 0.45
  // Minimum free width the robot may pass through.
  float required_width() const;              // width + 2*safety_distance = 0.90
};

struct MapBuildParams {
  float voxel_size = 0.10f;      int outlier_k = 20;       float outlier_std_ratio = 1.0f;
  float normal_search_radius = 0.30f; float max_edge_length = 0.30f;
  float max_normal_deviation_deg = 35.0f; bool prefer_layered_triangulation = false;
  bool  keep_raw_cloud = true;   // R4: keep raw-resolution cloud for step/width rechecks
};

struct TraversabilityParams {
  float max_slope_deg = 20.0f;      float max_normal_diff_deg = 30.0f;
  float max_slope_diff_deg = 15.0f; float max_step_height = 0.05f;
  // R1: flat<->ramp transition exemption. When the slope difference exceeds
  // max_slope_diff_deg but the normal difference and the height jump are both
  // within limits, the transition is still considered continuous.
  bool  allow_slope_transition = true;
  float max_roughness = 0.03f;
  float max_surface_curvature = 0.05f;       // 1/m, surface curvature (not path)
  float min_triangle_area = 0.0025f;
  float required_clearance = 0.45f;          // derived from robot, consistency-checked
  float max_slope_rad() const; float max_normal_diff_rad() const; float max_slope_diff_rad() const;
};

struct RegionParams {
  // R1: weights act on differences normalized to [0,1], threshold is dimensionless.
  float normal_weight = 1.0f, slope_weight = 0.5f, roughness_weight = 0.5f, height_weight = 1.0f;
  float region_score_threshold = 0.60f;
  // R2: height-span constraint only applies across slope regimes, so long ramps
  // are not fragmented.
  float max_height_span = 0.30f;
  float slope_regime_deg = 8.0f;             // above this slope => "ramp regime"
  bool  height_span_across_regime_only = true;
  float max_slope_variance = 0.02f;
  float min_region_area = 0.50f;
  int   min_region_triangles = 20;
  // R3: bottleneck split uses a relative criterion (independent of robot size).
  float bottleneck_ratio = 0.50f;            // w < ratio * median region width => candidate
  float bottleneck_abs_ceiling = 2.00f;      // and w must be below this absolute ceiling
  float min_side_area = 0.50f;
  // R5: concave split uses a medial-axis branching criterion.
  float concavity_branch_min_arm_length = 1.00f;
  float concavity_branch_angle_deg = 60.0f;
  float ramp_slope_deg = 8.0f;
};

struct PortalParams {
  int   boundary_min_edges = 2;   // minimum shared edges forming a chain
  float width_margin = 0.0f;
};

struct CompressionParams {
  float max_merge_cost = 0.25f;   bool preserve_topology = true;
  // R12-b: hard-forbid cross-regime merging. When true, a flat region and a ramp
  // region may not merge even if the geometric cost is below max_merge_cost, so
  // the is_ramp semantics established by T14 are preserved.
  bool forbid_cross_regime_merge = true;
};

struct PlanParams {
  float max_project_radius = 0.30f;
  // Strict adherence of the *returned* path to the traversable surface. This is
  // deliberately independent of max_project_radius, which is only the endpoint /
  // waypoint localization search radius. Every interior sample of every emitted
  // segment must lie within this distance of the real mesh.
  float path_surface_tolerance = 0.005f;
  float max_project_normal_diff_deg = 45.0f;  // R6: normal consistency on projection
  int corridor_expansion_rings = 1;
  bool fallback_full_mesh = true;   bool shortcut_enabled = true;
  bool funnel_enabled = true;       bool smoothing_enabled = true;
  int smoothing_iterations = 20;
  float check_step = 0.05f;         float topology_weight = 1.0f;
  // Path curvature limit is unified to robot.max_path_curvature; no separate
  // curvature_limit key. R29: test-only degradation injection, not part of the
  // YAML contract; the production path keeps it false.
  bool force_corridor_failure = false;
};

struct TopmapConfig {
  RobotParams robot; MapBuildParams map; TraversabilityParams traversability;
  RegionParams region; PortalParams portal; CompressionParams compression; PlanParams plan;
};

Result<TopmapConfig> loadTopmapConfig(const std::string& yaml_path);
Result<RobotParams> loadRobotParams(const std::string& yaml_path);

}  // namespace topmap
