#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "topmap_core/binary_io.hpp"
#include "topmap_surface/mesh_serialization.hpp"
#include "topmap_surface/traversability.hpp"

namespace {

topmap::MeshData Quad() {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}, {0, 2, 3}};
  return d;
}

topmap::MeshData Tilted() {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0.2f}, {1, 1, 0.4f}, {0, 1, 0.2f}};
  d.faces = {{0, 1, 2}, {0, 2, 3}};
  return d;
}

topmap::TriangleMesh Build(const topmap::MeshData& d) {
  auto m = topmap::TriangleMesh::fromMeshData(d, {});
  EXPECT_TRUE(m.ok()) << m.status().message;
  return m.take();
}

// Serialize a structurally-valid header plus arbitrary triangle vertex ids.
std::vector<std::uint8_t> PayloadWithVertexIds(std::int32_t v0, std::int32_t v1, std::int32_t v2) {
  topmap::BinaryWriter w;
  w.writeU32(3);
  w.writeVector3f(Eigen::Vector3f(0, 0, 0));
  w.writeVector3f(Eigen::Vector3f(1, 0, 0));
  w.writeVector3f(Eigen::Vector3f(0, 1, 0));
  w.writeU32(1);
  w.writeI32(v0);
  w.writeI32(v1);
  w.writeI32(v2);
  w.writeVector3f(Eigen::Vector3f(0.3f, 0.3f, 0.0f));  // centroid
  w.writeVector3f(Eigen::Vector3f::UnitZ());           // normal
  w.writeF32(0.0f);                                    // slope
  w.writeF32(0.0f);                                    // roughness
  w.writeF32(0.0f);                                    // curvature
  w.writeF32(0.5f);                                    // area
  w.writeF32(0.0f);                                    // min_clearance
  w.writeBool(true);                                   // traversable
  return w.bytes();
}

}  // namespace

TEST(MeshSerializationTest, RoundTripPreservesGeometryAndAdjacency) {
  topmap::TriangleMesh m = Build(Quad());
  topmap::BinaryWriter w;
  topmap::Status s = topmap::writeTriangleMesh(w, m);
  ASSERT_TRUE(s.ok()) << s.message;
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  ASSERT_TRUE(back.ok()) << back.status().message;
  EXPECT_EQ(back.value().size(), 2u);
  EXPECT_EQ(back.value().neighbors(0), std::vector<topmap::TriangleId>{1});
  EXPECT_TRUE(back.value().vertices()[2].isApprox(Eigen::Vector3f(1, 1, 0)));
}

TEST(MeshSerializationTest, RoundTripPreservesEveryTriangleField) {
  topmap::TriangleMesh m = Build(Tilted());
  topmap::TraversabilityEvaluator::apply(m, topmap::TraversabilityParams{});
  topmap::BinaryWriter w;
  ASSERT_TRUE(topmap::writeTriangleMesh(w, m).ok());
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  ASSERT_TRUE(back.ok()) << back.status().message;

  ASSERT_EQ(back.value().vertices().size(), m.vertices().size());
  for (std::size_t i = 0; i < m.vertices().size(); ++i) {
    EXPECT_TRUE(back.value().vertices()[i].isApprox(m.vertices()[i])) << i;
  }
  ASSERT_EQ(back.value().size(), m.size());
  for (topmap::TriangleId i = 0; i < static_cast<topmap::TriangleId>(m.size()); ++i) {
    const topmap::TriangleNode& a = m.triangle(i);
    const topmap::TriangleNode& b = back.value().triangle(i);
    EXPECT_EQ(b.id, a.id);
    EXPECT_EQ(b.vertex_ids, a.vertex_ids);
    EXPECT_TRUE(b.centroid.isApprox(a.centroid)) << i;
    EXPECT_TRUE(b.normal.isApprox(a.normal)) << i;
    EXPECT_FLOAT_EQ(b.slope, a.slope) << i;
    EXPECT_FLOAT_EQ(b.roughness, a.roughness) << i;
    EXPECT_FLOAT_EQ(b.curvature, a.curvature) << i;
    EXPECT_FLOAT_EQ(b.area, a.area) << i;
    EXPECT_FLOAT_EQ(b.min_clearance, a.min_clearance) << i;
    EXPECT_EQ(b.traversable, a.traversable) << i;
    EXPECT_EQ(b.neighbors(), a.neighbors()) << i;
  }
  EXPECT_EQ(back.value().componentIds(), m.componentIds());
}

TEST(MeshSerializationTest, RoundTripRebuildsComponents) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 3}, {1, 0, 3}, {0, 1, 3}};
  d.faces = {{0, 1, 2}, {3, 4, 5}};  // two disconnected shells
  topmap::TriangleMesh m = Build(d);
  topmap::BinaryWriter w;
  ASSERT_TRUE(topmap::writeTriangleMesh(w, m).ok());
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  ASSERT_TRUE(back.ok());
  EXPECT_NE(back.value().componentId(0), back.value().componentId(1));
}

TEST(MeshSerializationTest, TruncatedPayloadFails) {
  std::vector<std::uint8_t> data{1, 2};
  topmap::BinaryReader r(data);
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MeshSerializationTest, NegativeVertexIndexFails) {
  std::vector<std::uint8_t> data = PayloadWithVertexIds(-1, 0, 1);
  topmap::BinaryReader r(data);
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MeshSerializationTest, OutOfRangeVertexIndexFails) {
  std::vector<std::uint8_t> data = PayloadWithVertexIds(0, 1, 3);
  topmap::BinaryReader r(data);
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MeshSerializationTest, TruncatedTriangleFails) {
  std::vector<std::uint8_t> data = PayloadWithVertexIds(0, 1, 2);
  data.resize(data.size() - 3);  // drop the tail of the single triangle
  topmap::BinaryReader r(data);
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MeshSerializationTest, AbsurdVertexCountFailsWithoutAllocating) {
  topmap::BinaryWriter w;
  w.writeU32(0xffffffffu);  // vertex count with no vertex payload
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MeshSerializationTest, AbsurdTriangleCountFailsWithoutAllocating) {
  topmap::BinaryWriter w;
  w.writeU32(3);
  w.writeVector3f(Eigen::Vector3f(0, 0, 0));
  w.writeVector3f(Eigen::Vector3f(1, 0, 0));
  w.writeVector3f(Eigen::Vector3f(0, 1, 0));
  w.writeU32(0xffffffffu);  // triangle count with no triangle payload
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}

TEST(MeshSerializationTest, EmptyTriangleListFails) {
  topmap::BinaryWriter w;
  w.writeU32(3);
  w.writeVector3f(Eigen::Vector3f(0, 0, 0));
  w.writeVector3f(Eigen::Vector3f(1, 0, 0));
  w.writeVector3f(Eigen::Vector3f(0, 1, 0));
  w.writeU32(0);
  topmap::BinaryReader r(w.bytes());
  auto back = topmap::readTriangleMesh(r);
  EXPECT_FALSE(back.ok());
  EXPECT_EQ(back.status().code, topmap::ErrorCode::kSerializationIoError);
}
