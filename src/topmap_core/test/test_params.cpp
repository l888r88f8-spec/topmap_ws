#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "topmap_core/params.hpp"

namespace {

std::string readFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string writeTemp(const std::string& name, const std::string& body) {
  const std::string path = ::testing::TempDir() + name;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << body;
  return path;
}

// Replace a top-level section mapping with a scalar while leaving the rest of
// the document untouched, to exercise the shape-validation boundary.
std::string scalarizeSection(const std::string& yaml, const std::string& section) {
  std::istringstream in(yaml);
  std::ostringstream out;
  const std::string header = "  " + section + ":";
  std::string line;
  bool skipping = false;
  while (std::getline(in, line)) {
    if (!skipping && line == header) {
      out << header << " 3.14\n";
      skipping = true;
      continue;
    }
    if (skipping) {
      if (line.rfind("    ", 0) == 0) continue;  // still inside the section body
      skipping = false;
    }
    out << line << "\n";
  }
  return out.str();
}

}  // namespace

TEST(ParamsTest, LoadsAllSections) {
  // topmap_valid.yaml deliberately uses legal non-default values in every
  // section so this test proves the mapping is read rather than inherited from
  // the struct defaults.
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok()) << cfg.status().message;
  const auto& c = cfg.value();
  EXPECT_FLOAT_EQ(c.robot.width, 0.72f);
  EXPECT_FLOAT_EQ(c.robot.length, 0.95f);
  EXPECT_FLOAT_EQ(c.robot.max_slope_deg, 25.0f);
  EXPECT_FLOAT_EQ(c.robot.max_step_height, 0.08f);
  EXPECT_FLOAT_EQ(c.robot.safety_distance, 0.20f);
  EXPECT_FLOAT_EQ(c.robot.max_path_curvature, 0.35f);
  EXPECT_FLOAT_EQ(c.map.voxel_size, 0.12f);
  EXPECT_EQ(c.map.outlier_k, 15);
  EXPECT_FLOAT_EQ(c.map.outlier_std_ratio, 1.5f);
  EXPECT_FLOAT_EQ(c.map.normal_search_radius, 0.35f);
  EXPECT_FLOAT_EQ(c.map.max_edge_length, 0.40f);
  EXPECT_FLOAT_EQ(c.map.max_normal_deviation_deg, 40.0f);
  EXPECT_TRUE(c.map.prefer_layered_triangulation);
  EXPECT_FALSE(c.map.keep_raw_cloud);
  EXPECT_FLOAT_EQ(c.traversability.max_normal_diff_deg, 35.0f);
  EXPECT_FLOAT_EQ(c.traversability.max_slope_diff_deg, 12.0f);
  EXPECT_FALSE(c.traversability.allow_slope_transition);
  EXPECT_FLOAT_EQ(c.traversability.max_roughness, 0.04f);
  EXPECT_FLOAT_EQ(c.traversability.max_surface_curvature, 0.08f);
  EXPECT_FLOAT_EQ(c.traversability.min_triangle_area, 0.0030f);
  EXPECT_FLOAT_EQ(c.region.normal_weight, 1.2f);
  EXPECT_FLOAT_EQ(c.region.slope_weight, 0.6f);
  EXPECT_FLOAT_EQ(c.region.roughness_weight, 0.4f);
  EXPECT_FLOAT_EQ(c.region.height_weight, 0.9f);
  EXPECT_FLOAT_EQ(c.region.region_score_threshold, 0.65f);
  EXPECT_FLOAT_EQ(c.region.max_height_span, 0.35f);
  EXPECT_FLOAT_EQ(c.region.slope_regime_deg, 10.0f);
  EXPECT_FALSE(c.region.height_span_across_regime_only);
  EXPECT_FLOAT_EQ(c.region.max_slope_variance, 0.03f);
  EXPECT_FLOAT_EQ(c.region.min_region_area, 0.60f);
  EXPECT_EQ(c.region.min_region_triangles, 25);
  EXPECT_FLOAT_EQ(c.region.bottleneck_ratio, 0.40f);
  EXPECT_FLOAT_EQ(c.region.bottleneck_abs_ceiling, 2.50f);
  EXPECT_FLOAT_EQ(c.region.min_side_area, 0.55f);
  EXPECT_FLOAT_EQ(c.region.concavity_branch_min_arm_length, 1.20f);
  EXPECT_FLOAT_EQ(c.region.concavity_branch_angle_deg, 55.0f);
  EXPECT_FLOAT_EQ(c.region.ramp_slope_deg, 9.0f);
  EXPECT_EQ(c.portal.boundary_min_edges, 3);
  EXPECT_FLOAT_EQ(c.portal.width_margin, 0.05f);
  EXPECT_FLOAT_EQ(c.compression.max_merge_cost, 0.30f);
  EXPECT_FALSE(c.compression.preserve_topology);
  EXPECT_FALSE(c.compression.forbid_cross_regime_merge);  // R12-b: value comes from YAML
  EXPECT_FLOAT_EQ(c.plan.max_project_radius, 0.35f);
  EXPECT_FLOAT_EQ(c.plan.max_project_normal_diff_deg, 50.0f);
  EXPECT_EQ(c.plan.corridor_expansion_rings, 2);
  EXPECT_FALSE(c.plan.fallback_full_mesh);
  EXPECT_FALSE(c.plan.shortcut_enabled);
  EXPECT_FALSE(c.plan.funnel_enabled);
  EXPECT_FALSE(c.plan.smoothing_enabled);
  EXPECT_EQ(c.plan.smoothing_iterations, 30);
  EXPECT_FLOAT_EQ(c.plan.check_step, 0.10f);
  EXPECT_FLOAT_EQ(c.plan.topology_weight, 1.5f);
}

TEST(ParamsTest, DerivedQuantitiesAreComputedNotRead) {
  // R1: derived quantities do not appear in YAML, they are computed from robot
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok()) << cfg.status().message;
  const auto& c = cfg.value();
  EXPECT_NEAR(c.robot.required_clearance(), 0.56f, 1e-6f);  // 0.5*0.72 + 0.20
  EXPECT_NEAR(c.robot.required_width(), 1.12f, 1e-6f);      // 0.72 + 2*0.20
  EXPECT_FLOAT_EQ(c.traversability.required_clearance, c.robot.required_clearance());
  EXPECT_FLOAT_EQ(c.traversability.max_slope_deg, c.robot.max_slope_deg);
  EXPECT_FLOAT_EQ(c.traversability.max_step_height, c.robot.max_step_height);
}

TEST(ParamsTest, MissingKeyFailsWithKeyPath) {
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_missing.yaml");
  ASSERT_FALSE(cfg.ok());
  EXPECT_EQ(cfg.status().code, topmap::ErrorCode::kInvalidArgument);
  EXPECT_NE(cfg.status().message.find("topmap.region.bottleneck_ratio"), std::string::npos)
      << cfg.status().message;
}

TEST(ParamsTest, RemovedKeysAreRejectedAsUnknown) {
  // R1: a YAML still carrying a removed derived key must error rather than be
  // silently ignored, otherwise old config files look like they still apply.
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_legacy.yaml");
  ASSERT_FALSE(cfg.ok());
  EXPECT_EQ(cfg.status().code, topmap::ErrorCode::kInvalidArgument);
  EXPECT_NE(cfg.status().message.find("unknown key"), std::string::npos) << cfg.status().message;
}

TEST(ParamsTest, OutOfRangeValueIsRejected) {
  // Spec section 8: an out-of-range value refuses to start.
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_invalid_range.yaml");
  ASSERT_FALSE(cfg.ok());
  EXPECT_EQ(cfg.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(ParamsTest, CurvatureParamsAreDistinct) {
  // R1: path curvature and surface curvature are two different quantities.
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok());
  EXPECT_FLOAT_EQ(cfg.value().robot.max_path_curvature, 0.35f);
  EXPECT_FLOAT_EQ(cfg.value().traversability.max_surface_curvature, 0.08f);
}

TEST(ParamsTest, ScalarOrSequenceNodesReturnStructuredErrorWithoutThrowing) {
  // A scalar/sequence where a mapping is expected must not leak a
  // YAML::BadSubscript; it must become kInvalidArgument naming the section.
  const std::string valid = readFile("test/testdata/topmap_valid.yaml");
  ASSERT_FALSE(valid.empty());

  struct ShapeCase {
    std::string name;
    std::string body;
    std::string path;
  };
  std::vector<ShapeCase> cases = {
      {"topmap_root_scalar.yaml", "topmap: 3.14\n", "topmap"},
      {"topmap_root_sequence.yaml", "topmap: [1, 2, 3]\n", "topmap"},
  };
  for (const char* section : {"robot", "map", "traversability", "region",
                              "portal", "compression", "plan"}) {
    cases.push_back({"topmap_section_" + std::string(section) + ".yaml",
                     scalarizeSection(valid, section),
                     "topmap." + std::string(section)});
  }

  for (const auto& c : cases) {
    const std::string path = writeTemp(c.name, c.body);
    topmap::Result<topmap::TopmapConfig> cfg;
    ASSERT_NO_THROW(cfg = topmap::loadTopmapConfig(path)) << c.name;
    ASSERT_FALSE(cfg.ok()) << c.name;
    EXPECT_EQ(cfg.status().code, topmap::ErrorCode::kInvalidArgument) << c.name;
    EXPECT_NE(cfg.status().message.find(c.path), std::string::npos)
        << c.name << " message=" << cfg.status().message;
  }
}
