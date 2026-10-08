#include <gtest/gtest.h>

#include "topmap_core/params.hpp"

TEST(ParamsTest, LoadsAllSections) {
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok()) << cfg.status().message;
  EXPECT_FLOAT_EQ(cfg.value().robot.width, 0.60f);
  EXPECT_FLOAT_EQ(cfg.value().robot.max_slope_deg, 20.0f);
  EXPECT_FLOAT_EQ(cfg.value().region.bottleneck_ratio, 0.50f);
  EXPECT_TRUE(cfg.value().plan.fallback_full_mesh);
  EXPECT_TRUE(cfg.value().traversability.allow_slope_transition);
  EXPECT_TRUE(cfg.value().map.keep_raw_cloud);
  EXPECT_TRUE(cfg.value().compression.forbid_cross_regime_merge);  // R12-b
}

TEST(ParamsTest, DerivedQuantitiesAreComputedNotRead) {
  // R1: derived quantities do not appear in YAML, they are computed from robot
  auto cfg = topmap::loadTopmapConfig("test/testdata/topmap_valid.yaml");
  ASSERT_TRUE(cfg.ok()) << cfg.status().message;
  const auto& c = cfg.value();
  EXPECT_NEAR(c.robot.required_clearance(), 0.45f, 1e-6f);  // 0.5*0.60 + 0.15
  EXPECT_NEAR(c.robot.required_width(), 0.90f, 1e-6f);      // 0.60 + 2*0.15
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
  EXPECT_FLOAT_EQ(cfg.value().robot.max_path_curvature, 0.25f);
  EXPECT_FLOAT_EQ(cfg.value().traversability.max_surface_curvature, 0.05f);
}
