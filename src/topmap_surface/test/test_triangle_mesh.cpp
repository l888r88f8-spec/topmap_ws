#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "topmap_surface/mesh_types.hpp"

namespace {

topmap::MeshData TwoTriangles() {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}, {0, 2, 3}};
  return d;
}

topmap::MeshData TwoOverlappingLayers() {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 3}, {1, 0, 3}, {0, 1, 3}};
  d.faces = {{0, 1, 2}, {3, 4, 5}};
  return d;
}

// A 2x3 vertex strip split into four triangles:
//   T0 {0,1,4}  T1 {0,4,3}  T2 {1,2,5}  T3 {1,5,4}
// Adjacency: T0-{T1,T3}, T1-{T0}, T2-{T3}, T3-{T2,T0}.
topmap::MeshData TriangleStrip() {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 1, 0}, {1, 1, 0}, {2, 1, 0}};
  d.faces = {{0, 1, 4}, {0, 4, 3}, {1, 2, 5}, {1, 5, 4}};
  return d;
}

}  // namespace

TEST(TriangleMeshTest, SharedEdgeBecomesNeighbor) {
  topmap::MeshBuildParams p;
  auto m = topmap::TriangleMesh::fromMeshData(TwoTriangles(), p);
  ASSERT_TRUE(m.ok()) << m.status().message;
  EXPECT_EQ(m.value().size(), 2u);
  EXPECT_EQ(m.value().neighbors(0), std::vector<topmap::TriangleId>{1});
  EXPECT_EQ(m.value().neighbors(1), std::vector<topmap::TriangleId>{0});
  ASSERT_NE(m.value().triangle(0).edges[0].neighbor, topmap::kInvalidId);
  EXPECT_NEAR(m.value().triangle(0).area, 0.5f, 1e-6f);
  EXPECT_GT(m.value().triangle(0).normal.z(), 0.99f);
  EXPECT_EQ(m.value().componentId(0), m.value().componentId(1));
}

TEST(TriangleMeshTest, OverlappingLayersNeverConnect) {
  topmap::MeshBuildParams p;
  auto m = topmap::TriangleMesh::fromMeshData(TwoOverlappingLayers(), p);
  ASSERT_TRUE(m.ok());
  EXPECT_TRUE(m.value().neighbors(0).empty());
  EXPECT_NE(m.value().componentId(0), m.value().componentId(1));
}

TEST(TriangleMeshTest, NonManifoldEdgeIsToleratedAndCounted) {
  // R9: three faces share one edge. The two smallest ids keep adjacency, the
  // third counts as discarded; 1/3 > max_discarded_face_ratio(0.05) -> failure.
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}};
  d.faces = {{0, 1, 2}, {0, 1, 3}, {0, 1, 4}};
  auto m = topmap::TriangleMesh::fromMeshData(d, topmap::MeshBuildParams{});
  ASSERT_FALSE(m.ok());
  EXPECT_EQ(m.status().code, topmap::ErrorCode::kMeshNotManifold);
}

TEST(TriangleMeshTest, FewNonManifoldEdgesAreTolerated) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}};
  d.faces = {{0, 1, 2}, {0, 1, 3}, {0, 1, 4}};
  topmap::MeshBuildParams p;
  p.max_discarded_face_ratio = 0.50f;
  auto m = topmap::TriangleMesh::fromMeshData(d, p);
  ASSERT_TRUE(m.ok()) << m.status().message;
  EXPECT_EQ(m.value().size(), 3u);
  EXPECT_GT(m.value().discardedFaces(), 0);
}

TEST(TriangleMeshTest, VertexFeaturesTransferToTriangles) {
  // R7: per-vertex roughness/curvature must reach the triangles.
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}, {0, 2, 3}};
  d.vertex_roughness = {0.01f, 0.02f, 0.03f, 0.04f};
  d.vertex_curvature = {0.00f, 0.01f, 0.02f, 0.03f};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(m.ok()) << m.status().message;
  EXPECT_NEAR(m.value().triangle(0).roughness, 0.02f, 1e-6f);
  EXPECT_NEAR(m.value().triangle(0).curvature, 0.01f, 1e-6f);
}

TEST(TriangleMeshTest, MismatchedFeatureLengthRejected) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  d.faces = {{0, 1, 2}};
  d.vertex_roughness = {0.01f, 0.02f};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_FALSE(m.ok());
  EXPECT_EQ(m.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TriangleMeshTest, MismatchedCurvatureLengthRejected) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  d.faces = {{0, 1, 2}};
  d.vertex_curvature = {0.01f};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_FALSE(m.ok());
  EXPECT_EQ(m.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TriangleMeshTest, EmptyFacesRejected) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_FALSE(m.ok());
  EXPECT_EQ(m.status().code, topmap::ErrorCode::kMeshEmpty);
}

TEST(TriangleMeshTest, InvalidVertexIndexRejected) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  d.faces = {{0, 1, 7}};
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_FALSE(m.ok());
  EXPECT_EQ(m.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TriangleMeshTest, DegenerateFacesAreDiscardedAndCounted) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}, {0, 0, 1}};
  topmap::MeshBuildParams p;
  p.max_discarded_face_ratio = 0.50f;
  auto m = topmap::TriangleMesh::fromMeshData(d, p);
  ASSERT_TRUE(m.ok()) << m.status().message;
  EXPECT_EQ(m.value().size(), 1u);
  EXPECT_EQ(m.value().discardedFaces(), 1);
}

TEST(TriangleMeshTest, CollinearFacesAreDiscarded) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}, {0, 1, 3}};
  topmap::MeshBuildParams p;
  p.max_discarded_face_ratio = 0.50f;
  auto m = topmap::TriangleMesh::fromMeshData(d, p);
  ASSERT_TRUE(m.ok()) << m.status().message;
  EXPECT_EQ(m.value().size(), 1u);
  EXPECT_EQ(m.value().discardedFaces(), 1);
}

TEST(TriangleMeshTest, TooManyDiscardedFacesFail) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}, {0, 0, 1}};
  auto m = topmap::TriangleMesh::fromMeshData(d, topmap::MeshBuildParams{});
  ASSERT_FALSE(m.ok());
  EXPECT_EQ(m.status().code, topmap::ErrorCode::kMeshNotManifold);
}

TEST(TriangleMeshTest, KRingExpandsByHops) {
  auto m = topmap::TriangleMesh::fromMeshData(TriangleStrip(), {});
  ASSERT_TRUE(m.ok()) << m.status().message;
  ASSERT_EQ(m.value().size(), 4u);
  EXPECT_EQ(m.value().neighbors(0), (std::vector<topmap::TriangleId>{1, 3}));

  auto ring1 = m.value().kRing(0, 1);
  ASSERT_TRUE(ring1.ok()) << ring1.status().message;
  std::vector<topmap::TriangleId> expected1{1, 3};
  auto got1 = ring1.value();
  std::sort(got1.begin(), got1.end());
  EXPECT_EQ(got1, expected1);

  auto ring2 = m.value().kRing(0, 2);
  ASSERT_TRUE(ring2.ok()) << ring2.status().message;
  auto got2 = ring2.value();
  std::sort(got2.begin(), got2.end());
  EXPECT_EQ(got2, (std::vector<topmap::TriangleId>{1, 2, 3}));

  auto bad = m.value().kRing(99, 1);
  EXPECT_FALSE(bad.ok());
}

TEST(TriangleMeshTest, ComponentIdsCoverWholeMesh) {
  auto m = topmap::TriangleMesh::fromMeshData(TriangleStrip(), {});
  ASSERT_TRUE(m.ok());
  EXPECT_EQ(m.value().componentIds().size(), 4u);
  for (topmap::TriangleId i = 0; i < 4; ++i) {
    EXPECT_GE(m.value().componentId(i), 0);
  }
}

TEST(TriangleMeshTest, FromTriangleNodesRebuildsAdjacencyAndComponents) {
  auto built = topmap::TriangleMesh::fromMeshData(TwoTriangles(), {});
  ASSERT_TRUE(built.ok());
  std::vector<Eigen::Vector3f> vertices = built.value().vertices();
  std::vector<topmap::TriangleNode> nodes = built.value().triangles();
  auto rebuilt = topmap::TriangleMesh::fromTriangleNodes(vertices, nodes);
  ASSERT_TRUE(rebuilt.ok()) << rebuilt.status().message;
  EXPECT_EQ(rebuilt.value().size(), 2u);
  EXPECT_EQ(rebuilt.value().neighbors(0), std::vector<topmap::TriangleId>{1});
  EXPECT_EQ(rebuilt.value().componentId(0), rebuilt.value().componentId(1));
}

TEST(TriangleMeshTest, FromTriangleNodesRejectsInvalidVertexId) {
  std::vector<Eigen::Vector3f> vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  topmap::TriangleNode node;
  node.id = 0;
  node.vertex_ids = {0, 1, 5};
  auto rebuilt = topmap::TriangleMesh::fromTriangleNodes(vertices, {node});
  ASSERT_FALSE(rebuilt.ok());
  EXPECT_EQ(rebuilt.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TriangleMeshTest, TriangleNeighborsMethodMatchesMesh) {
  auto m = topmap::TriangleMesh::fromMeshData(TriangleStrip(), {});
  ASSERT_TRUE(m.ok());
  EXPECT_EQ(m.value().triangle(0).neighbors(), (std::vector<topmap::TriangleId>{1, 3}));
}
