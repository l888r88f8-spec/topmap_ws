#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "topmap_surface/surface_features.hpp"

namespace {

pcl::PointCloud<pcl::PointXYZ> MakeGrid(float slope_deg, float noise) {
  pcl::PointCloud<pcl::PointXYZ> c;
  const float t = std::tan(slope_deg * M_PI / 180.0f);
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      c.push_back(pcl::PointXYZ(i * 0.05f, j * 0.05f, t * i * 0.05f + noise));
    }
  }
  return c;
}

pcl::PointCloud<pcl::PointXYZ> MakeRoughGrid(float amplitude) {
  pcl::PointCloud<pcl::PointXYZ> c;
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      const float z = amplitude * std::sin(1.7f * i) * std::cos(2.3f * j);
      c.push_back(pcl::PointXYZ(i * 0.05f, j * 0.05f, z));
    }
  }
  return c;
}

std::size_t CenterIndex() { return 10u * 21u + 10u; }

}  // namespace

TEST(SurfaceFeatureTest, FlatPlaneNormalsAreUp) {
  auto c = MakeGrid(0.0f, 0.0f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  ASSERT_FALSE(f.value().empty());
  EXPECT_GT(f.value()[0].normal.z(), 0.99f);
  EXPECT_LT(f.value()[0].slope, 0.02f);
}

TEST(SurfaceFeatureTest, TiltedPlaneSlopeMatches) {
  auto c = MakeGrid(20.0f, 0.0f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  EXPECT_NEAR(f.value()[0].slope * 180.0f / M_PI, 20.0f, 1.0f);
}

TEST(SurfaceFeatureTest, EmptyCloudIsRejected) {
  pcl::PointCloud<pcl::PointXYZ> c;
  topmap::MapBuildParams p;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_FALSE(f.ok());
  EXPECT_EQ(f.status().code, topmap::ErrorCode::kPointCloudTooSmall);
}

TEST(SurfaceFeatureTest, TooSmallCloudIsRejected) {
  pcl::PointCloud<pcl::PointXYZ> c;
  c.push_back(pcl::PointXYZ(0, 0, 0));
  c.push_back(pcl::PointXYZ(1, 0, 0));
  topmap::MapBuildParams p;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_FALSE(f.ok());
  EXPECT_EQ(f.status().code, topmap::ErrorCode::kPointCloudTooSmall);
}

TEST(SurfaceFeatureTest, NonPositiveSearchRadiusIsRejected) {
  auto c = MakeGrid(0.0f, 0.0f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.0f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_FALSE(f.ok());
  EXPECT_EQ(f.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(SurfaceFeatureTest, NonFiniteCoordinatesAreRejected) {
  auto c = MakeGrid(0.0f, 0.0f);
  c[0].x = std::numeric_limits<float>::quiet_NaN();
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_FALSE(f.ok());
  EXPECT_EQ(f.status().code, topmap::ErrorCode::kPointCloudInvalidCoordinates);
}

TEST(SurfaceFeatureTest, KeepsOneFeaturePerInputPoint) {
  auto c = MakeGrid(10.0f, 0.0f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.20f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  ASSERT_EQ(f.value().size(), c.size());
  for (std::size_t i = 0; i < c.size(); ++i) {
    EXPECT_FLOAT_EQ(f.value()[i].position.x(), c[i].x);
    EXPECT_FLOAT_EQ(f.value()[i].position.y(), c[i].y);
    EXPECT_FLOAT_EQ(f.value()[i].position.z(), c[i].z);
  }
}

TEST(SurfaceFeatureTest, IsolatedPointsFallBackToUnitZ) {
  pcl::PointCloud<pcl::PointXYZ> c;
  c.push_back(pcl::PointXYZ(0, 0, 0));
  c.push_back(pcl::PointXYZ(10, 0, 0));
  c.push_back(pcl::PointXYZ(0, 10, 0));
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.01f;  // no point has a neighbourhood
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  ASSERT_EQ(f.value().size(), 3u);
  for (const auto& feature : f.value()) {
    EXPECT_NEAR(feature.normal.z(), 1.0f, 1e-6f);
    EXPECT_NEAR(feature.slope, 0.0f, 1e-6f);
    EXPECT_FLOAT_EQ(feature.roughness, 0.0f);
  }
}

TEST(SurfaceFeatureTest, FlatPlaneHasNearZeroRoughness) {
  auto c = MakeGrid(0.0f, 0.0f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  EXPECT_LT(f.value()[CenterIndex()].roughness, 1e-3f);
}

TEST(SurfaceFeatureTest, NoisyPlaneHasPositiveRoughness) {
  auto c = MakeRoughGrid(0.01f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  EXPECT_GT(f.value()[CenterIndex()].roughness, 1e-3f);
}

TEST(SurfaceFeatureTest, CurvatureIsFiniteAndNonNegative) {
  auto c = MakeGrid(0.0f, 0.0f);
  topmap::MapBuildParams p;
  p.normal_search_radius = 0.15f;
  auto f = topmap::SurfaceFeatureEstimator::estimate(c, p);
  ASSERT_TRUE(f.ok());
  for (const auto& feature : f.value()) {
    EXPECT_TRUE(std::isfinite(feature.curvature));
    EXPECT_GE(feature.curvature, 0.0f);
  }
}
