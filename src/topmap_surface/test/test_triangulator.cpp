#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "topmap_surface/triangulator.hpp"

namespace {

std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>> GridPoints(
    int n, float step, float z, float tilt_deg = 0.0f) {
  std::vector<Eigen::Vector3f> p, nn;
  const float t = std::tan(tilt_deg * M_PI / 180.0f);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      p.emplace_back(i * step, j * step, z + t * i * step);
    }
  }
  nn.assign(p.size(), Eigen::Vector3f(0, 0, 1));
  return {p, nn};
}

}  // namespace

TEST(TriangulatorTest, LayeredGridProducesExpectedTriangleCount) {
  auto [p, n] = GridPoints(6, 0.10f, 0.0f);
  topmap::MeshBuildParams bp;
  bp.prefer_layered = true;
  bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().mode, topmap::TriangulationMode::kLayered2p5D);
  EXPECT_EQ(r.value().mesh.faces.size(), 2u * 5u * 5u);
  EXPECT_EQ(r.value().discarded_triangles, 0);
}

TEST(TriangulatorTest, OverlappingFloorsStaySeparated) {
  auto [p1, n1] = GridPoints(5, 0.10f, 0.0f);
  auto [p2, n2] = GridPoints(5, 0.10f, 3.0f);
  std::vector<Eigen::Vector3f> p = p1, n = n1;
  p.insert(p.end(), p2.begin(), p2.end());
  n.insert(n.end(), n2.begin(), n2.end());
  topmap::MeshBuildParams bp;
  bp.prefer_layered = true;
  bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().component_count, 2);
  EXPECT_EQ(r.value().mode, topmap::TriangulationMode::kLayered2p5D);
  for (const auto& f : r.value().mesh.faces) {
    const float z0 = p[f[0]].z(), z1 = p[f[1]].z(), z2 = p[f[2]].z();
    EXPECT_NEAR(std::max({z0, z1, z2}) - std::min({z0, z1, z2}), 0.0f, 0.01f);
  }
}

TEST(TriangulatorTest, AutoModeUsesCgal3DOnDenseSurface) {
  auto [p, n] = GridPoints(6, 0.10f, 0.0f);
  topmap::MeshBuildParams bp;  // prefer_layered == false -> AFSR first
  bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().mode, topmap::TriangulationMode::kDelaunay3DSurface);
  EXPECT_GE(r.value().mesh.faces.size(), p.size());
}

TEST(TriangulatorTest, FallsBackToLayeredWhenCgalProducesTooFewTriangles) {
  std::vector<Eigen::Vector3f> p = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  std::vector<Eigen::Vector3f> n(p.size(), Eigen::Vector3f(0, 0, 1));
  topmap::MeshBuildParams bp;  // AFSR yields 1 facet < 3 points
  bp.max_edge_length = 2.0f;
  bp.component_radius = 2.0f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().mode, topmap::TriangulationMode::kLayered2p5D);
  EXPECT_EQ(r.value().mesh.faces.size(), 1u);
}

TEST(TriangulatorTest, ExplicitLayeredFailureUsesLayeredErrorCode) {
  std::vector<Eigen::Vector3f> p = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  std::vector<Eigen::Vector3f> n(p.size(), Eigen::Vector3f(0, 0, 1));
  topmap::MeshBuildParams bp;
  bp.prefer_layered = true;
  bp.component_radius = 10.0f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kLayeredTriangulationFailed);
}

TEST(TriangulatorTest, AutoBothModesFailUsesTriangulationErrorCode) {
  std::vector<Eigen::Vector3f> p = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  std::vector<Eigen::Vector3f> n(p.size(), Eigen::Vector3f(0, 0, 1));
  topmap::MeshBuildParams bp;
  bp.component_radius = 10.0f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kTriangulationFailed);
}

TEST(TriangulatorTest, MaxEdgeLengthAndDiscardRatioRejectLayeredMesh) {
  auto [p, n] = GridPoints(6, 0.10f, 0.0f);
  topmap::MeshBuildParams bp;
  bp.prefer_layered = true;
  bp.max_edge_length = 0.11f;  // every right-triangle hypotenuse (0.141) is filtered
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kLayeredTriangulationFailed);
}

TEST(TriangulatorTest, EmptyInputRejected) {
  std::vector<Eigen::Vector3f> p, n;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, {});
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kPointCloudEmpty);
}

TEST(TriangulatorTest, TooSmallInputRejected) {
  std::vector<Eigen::Vector3f> p = {{0, 0, 0}, {1, 0, 0}};
  std::vector<Eigen::Vector3f> n(p.size(), Eigen::Vector3f(0, 0, 1));
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, {});
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kPointCloudTooSmall);
}

TEST(TriangulatorTest, NormalsSizeMismatchRejected) {
  std::vector<Eigen::Vector3f> p = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  std::vector<Eigen::Vector3f> n = {{0, 0, 1}};
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, {});
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TriangulatorTest, FeatureVectorsPassThroughToMesh) {
  auto [p, n] = GridPoints(5, 0.10f, 0.0f);
  std::vector<float> roughness(p.size());
  for (std::size_t i = 0; i < p.size(); ++i) roughness[i] = static_cast<float>(i);
  std::vector<float> curvature(p.size(), 0.25f);
  topmap::MeshBuildParams bp;
  bp.prefer_layered = true;
  bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp, &roughness, &curvature);
  ASSERT_TRUE(r.ok()) << r.status().message;
  ASSERT_EQ(r.value().mesh.vertex_roughness.size(), p.size());
  ASSERT_EQ(r.value().mesh.vertex_curvature.size(), p.size());
  for (std::size_t i = 0; i < p.size(); ++i) {
    EXPECT_FLOAT_EQ(r.value().mesh.vertices[i].x(), p[i].x());
    EXPECT_FLOAT_EQ(r.value().mesh.vertex_roughness[i], roughness[i]);
    EXPECT_FLOAT_EQ(r.value().mesh.vertex_curvature[i], curvature[i]);
  }
}

TEST(TriangulatorTest, MismatchedFeatureLengthRejected) {
  auto [p, n] = GridPoints(4, 0.10f, 0.0f);
  std::vector<float> bad(p.size() - 1, 0.0f);
  topmap::MeshBuildParams bp;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp, &bad, nullptr);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TriangulatorTest, TiltedGridIsReconstructedIn3DMode) {
  auto [p, n] = GridPoints(6, 0.10f, 0.0f, 20.0f);
  topmap::MeshBuildParams bp;
  bp.max_edge_length = 0.25f;
  auto r = topmap::SurfaceTriangulator::triangulate(p, n, bp);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_FALSE(r.value().mesh.faces.empty());
}
