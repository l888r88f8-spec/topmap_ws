#include <gtest/gtest.h>

#include <cstdio>
#include <limits>
#include <string>

#include <pcl/io/pcd_io.h>

#include "topmap_surface/point_cloud_processor.hpp"

namespace {

pcl::PointCloud<pcl::PointXYZ> Grid(int n, float step) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      cloud.push_back(pcl::PointXYZ(i * step, j * step, 0.0f));
    }
  }
  return cloud;
}

}  // namespace

TEST(PointCloudProcessorTest, RemovesNanAndDownsamples) {
  pcl::PointCloud<pcl::PointXYZ> in;
  in.push_back(pcl::PointXYZ(0.00f, 0, 0));
  in.push_back(pcl::PointXYZ(0.05f, 0, 0));
  in.push_back(pcl::PointXYZ(1.00f, 1, 0));
  in.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  topmap::MapBuildParams p;
  p.voxel_size = 0.10f;
  p.outlier_k = 0;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().size(), 2u);
}

TEST(PointCloudProcessorTest, EmptyInputIsRejected) {
  pcl::PointCloud<pcl::PointXYZ> in;
  topmap::MapBuildParams p;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_FALSE(out.ok());
  EXPECT_EQ(out.status().code, topmap::ErrorCode::kPointCloudEmpty);
}

TEST(PointCloudProcessorTest, AllNanInputIsRejectedAsEmpty) {
  pcl::PointCloud<pcl::PointXYZ> in;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  in.push_back(pcl::PointXYZ(nan, 0, 0));
  in.push_back(pcl::PointXYZ(0, nan, 0));
  in.push_back(pcl::PointXYZ(0, 0, nan));
  topmap::MapBuildParams p;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_FALSE(out.ok());
  EXPECT_EQ(out.status().code, topmap::ErrorCode::kPointCloudEmpty);
}

TEST(PointCloudProcessorTest, InfiniteCoordinatesAreRejected) {
  pcl::PointCloud<pcl::PointXYZ> in = Grid(2, 0.10f);  // 4 valid points
  const float inf = std::numeric_limits<float>::infinity();
  in.push_back(pcl::PointXYZ(inf, 0, 0));
  topmap::MapBuildParams p;
  p.voxel_size = 0.0f;
  p.outlier_k = 0;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_FALSE(out.ok());
  EXPECT_EQ(out.status().code, topmap::ErrorCode::kPointCloudInvalidCoordinates);
}

TEST(PointCloudProcessorTest, TooFewPointsAreRejected) {
  pcl::PointCloud<pcl::PointXYZ> in;
  in.push_back(pcl::PointXYZ(0, 0, 0));
  in.push_back(pcl::PointXYZ(1, 0, 0));
  topmap::MapBuildParams p;
  p.voxel_size = 0.0f;
  p.outlier_k = 0;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_FALSE(out.ok());
  EXPECT_EQ(out.status().code, topmap::ErrorCode::kPointCloudTooSmall);
}

TEST(PointCloudProcessorTest, StatisticalOutlierRemovalDropsIsolatedPoint) {
  pcl::PointCloud<pcl::PointXYZ> in = Grid(10, 0.05f);  // 100 points
  in.push_back(pcl::PointXYZ(50.0f, 50.0f, 50.0f));      // isolated outlier
  topmap::MapBuildParams p;
  p.voxel_size = 0.0f;
  p.outlier_k = 20;
  p.outlier_std_ratio = 1.0f;
  auto out = topmap::PointCloudProcessor::process(in, p);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_GE(out.value().size(), 95u);
  EXPECT_LT(out.value().size(), 101u);
  for (const auto& pt : out.value().points) {
    EXPECT_LT(pt.x, 10.0f);
  }
}

TEST(PointCloudProcessorTest, LoadsPcdFromDisk) {
  const pcl::PointCloud<pcl::PointXYZ> cloud = Grid(4, 0.10f);
  const std::string path = "t06_point_cloud_roundtrip.pcd";
  ASSERT_EQ(pcl::io::savePCDFileBinary(path, cloud), 0);
  auto out = topmap::PointCloudProcessor::loadPCD(path);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().size(), cloud.size());
  std::remove(path.c_str());
}

TEST(PointCloudProcessorTest, MissingPcdFileIsRejected) {
  auto out = topmap::PointCloudProcessor::loadPCD("/nonexistent/topmap/does_not_exist.pcd");
  ASSERT_FALSE(out.ok());
  EXPECT_EQ(out.status().code, topmap::ErrorCode::kInvalidArgument);
}
