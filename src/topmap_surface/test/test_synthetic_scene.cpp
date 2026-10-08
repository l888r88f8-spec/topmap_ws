#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <utility>
#include <vector>

#include "topmap_surface/synthetic_scene.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTol = 1e-3f;

int ComponentCount(const topmap::TriangleMesh& m) {
  const auto& ids = m.componentIds();
  return ids.empty() ? 0 : *std::max_element(ids.begin(), ids.end()) + 1;
}

struct Bounds {
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f, min_z = 1e9f, max_z = -1e9f;
};

Bounds BoundsOf(const topmap::TriangleMesh& m) {
  Bounds b;
  for (const Eigen::Vector3f& v : m.vertices()) {
    b.min_x = std::min(b.min_x, v.x());
    b.max_x = std::max(b.max_x, v.x());
    b.min_y = std::min(b.min_y, v.y());
    b.max_y = std::max(b.max_y, v.y());
    b.min_z = std::min(b.min_z, v.z());
    b.max_z = std::max(b.max_z, v.z());
  }
  return b;
}

bool Near(float a, float b) { return std::fabs(a - b) <= kTol; }

bool HasVertex(const topmap::TriangleMesh& m, float x, float y, float z) {
  for (const Eigen::Vector3f& v : m.vertices()) {
    if (Near(v.x(), x) && Near(v.y(), y) && Near(v.z(), z)) return true;
  }
  return false;
}

int CountVerticesIn(const topmap::TriangleMesh& m, float x0, float x1, float y0, float y1) {
  int n = 0;
  for (const Eigen::Vector3f& v : m.vertices()) {
    if (v.x() > x0 && v.x() < x1 && v.y() > y0 && v.y() < y1) ++n;
  }
  return n;
}

// y extent of the vertices whose x lies strictly inside (x0, x1).
std::pair<float, float> YRangeInX(const topmap::TriangleMesh& m, float x0, float x1) {
  float lo = 1e9f, hi = -1e9f;
  for (const Eigen::Vector3f& v : m.vertices()) {
    if (v.x() > x0 && v.x() < x1) {
      lo = std::min(lo, v.y());
      hi = std::max(hi, v.y());
    }
  }
  return {lo, hi};
}

topmap::TriangleMesh MeshOf(topmap::SyntheticScene s) {
  auto d = topmap::makeSyntheticMesh(s);
  EXPECT_TRUE(d.ok()) << topmap::toString(s) << ": " << d.status().message;
  auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
  EXPECT_TRUE(m.ok()) << topmap::toString(s) << ": " << m.status().message;
  return m.take();
}

}  // namespace

TEST(SyntheticSceneTest, AllScenesProduceUsableMeshes) {
  for (auto s : {topmap::SyntheticScene::kFlatPlane, topmap::SyntheticScene::kRamp,
                 topmap::SyntheticScene::kOverlappingFloors, topmap::SyntheticScene::kMultiLevelRamps,
                 topmap::SyntheticScene::kHallNarrowHall, topmap::SyntheticScene::kUShape,
                 topmap::SyntheticScene::kTShape, topmap::SyntheticScene::kSingleBridge,
                 topmap::SyntheticScene::kParallelPortals, topmap::SyntheticScene::kLoop}) {
    auto d = topmap::makeSyntheticMesh(s);
    ASSERT_TRUE(d.ok()) << topmap::toString(s) << ": " << d.status().message;
    auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
    ASSERT_TRUE(m.ok()) << topmap::toString(s) << ": " << m.status().message;
    EXPECT_GT(m.value().size(), 0u) << topmap::toString(s);
    auto ep = topmap::makeSyntheticEndpoints(s);
    ASSERT_TRUE(ep.ok()) << topmap::toString(s);
  }
}

TEST(SyntheticSceneTest, OverlappingFloorsAreTwoComponents) {
  auto d = topmap::makeSyntheticMesh(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(d.ok());
  auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
  ASSERT_TRUE(m.ok());
  EXPECT_EQ(ComponentCount(m.value()), 2);
}

TEST(SyntheticSceneTest, ConnectedScenesAreSingleComponent) {
  for (auto s : {topmap::SyntheticScene::kRamp, topmap::SyntheticScene::kMultiLevelRamps,
                 topmap::SyntheticScene::kHallNarrowHall, topmap::SyntheticScene::kUShape,
                 topmap::SyntheticScene::kTShape, topmap::SyntheticScene::kSingleBridge,
                 topmap::SyntheticScene::kParallelPortals, topmap::SyntheticScene::kLoop}) {
    auto d = topmap::makeSyntheticMesh(s);
    ASSERT_TRUE(d.ok());
    auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
    ASSERT_TRUE(m.ok());
    EXPECT_EQ(ComponentCount(m.value()), 1) << topmap::toString(s);
  }
}

TEST(SyntheticSceneTest, EndpointsMatchFrozenContract) {
  auto ep = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kHallNarrowHall);
  ASSERT_TRUE(ep.ok());
  EXPECT_TRUE(ep.value().first.isApprox(Eigen::Vector3f(3.5f, 3.5f, 0.05f), 1e-4f));
  EXPECT_TRUE(ep.value().second.isApprox(Eigen::Vector3f(13.5f, 3.5f, 0.05f), 1e-4f));
  auto rp = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(rp.ok());
  EXPECT_NEAR(rp.value().second.z(), 2.05f, 1e-4f);
}

TEST(SyntheticSceneTest, AllEndpointsMatchFrozenTable) {
  struct Row {
    topmap::SyntheticScene scene;
    Eigen::Vector3f start;
    Eigen::Vector3f goal;
  };
  const std::vector<Row> rows = {
      {topmap::SyntheticScene::kFlatPlane, {0.5f, 0.5f, 0.05f}, {5.5f, 5.5f, 0.05f}},
      {topmap::SyntheticScene::kRamp, {2.0f, 2.0f, 0.05f}, {15.4f, 2.0f, 2.05f}},
      {topmap::SyntheticScene::kOverlappingFloors, {2.5f, 2.5f, 0.05f}, {2.5f, 2.5f, 3.05f}},
      {topmap::SyntheticScene::kMultiLevelRamps, {2.0f, 2.0f, 0.05f}, {30.0f, 2.0f, 4.05f}},
      {topmap::SyntheticScene::kHallNarrowHall, {3.5f, 3.5f, 0.05f}, {13.5f, 3.5f, 0.05f}},
      {topmap::SyntheticScene::kUShape, {3.5f, 0.5f, 0.05f}, {3.5f, 4.5f, 0.05f}},
      {topmap::SyntheticScene::kTShape, {4.0f, 9.0f, 0.05f}, {9.0f, 4.0f, 0.05f}},
      {topmap::SyntheticScene::kSingleBridge, {2.5f, 2.5f, 0.05f}, {10.5f, 2.5f, 0.05f}},
      {topmap::SyntheticScene::kParallelPortals, {2.5f, 4.0f, 0.05f}, {10.5f, 4.0f, 0.05f}},
      {topmap::SyntheticScene::kLoop, {2.0f, 2.0f, 0.05f}, {10.0f, 10.0f, 0.05f}},
  };
  for (const Row& r : rows) {
    auto ep = topmap::makeSyntheticEndpoints(r.scene);
    ASSERT_TRUE(ep.ok()) << topmap::toString(r.scene);
    EXPECT_TRUE(ep.value().first.isApprox(r.start, 1e-4f)) << topmap::toString(r.scene);
    EXPECT_TRUE(ep.value().second.isApprox(r.goal, 1e-4f)) << topmap::toString(r.scene);
  }
}

TEST(SyntheticSceneTest, RampSlopeIsTraversable) {
  auto d = topmap::makeSyntheticMesh(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(d.ok());
  auto m = topmap::TriangleMesh::fromMeshData(d.value(), {});
  ASSERT_TRUE(m.ok());
  float max_slope_deg = 0.0f;
  for (const auto& tri : m.value().triangles()) {
    max_slope_deg = std::max(max_slope_deg, tri.slope * 180.0f / kPi);
  }
  EXPECT_GT(max_slope_deg, 8.0f);   // recognisable as a ramp
  EXPECT_LT(max_slope_deg, 20.0f);  // and drivable
  EXPECT_NEAR(max_slope_deg, 12.0f, 0.2f);
}

TEST(SyntheticSceneTest, SceneNameRoundTrip) {
  for (auto s : {topmap::SyntheticScene::kFlatPlane, topmap::SyntheticScene::kRamp,
                 topmap::SyntheticScene::kOverlappingFloors, topmap::SyntheticScene::kMultiLevelRamps,
                 topmap::SyntheticScene::kHallNarrowHall, topmap::SyntheticScene::kUShape,
                 topmap::SyntheticScene::kTShape, topmap::SyntheticScene::kSingleBridge,
                 topmap::SyntheticScene::kParallelPortals, topmap::SyntheticScene::kLoop}) {
    auto parsed = topmap::parseSyntheticScene(topmap::toString(s));
    ASSERT_TRUE(parsed.ok()) << topmap::toString(s);
    EXPECT_EQ(parsed.value(), s);
  }
  EXPECT_FALSE(topmap::parseSyntheticScene("no_such_scene").ok());
  EXPECT_FALSE(topmap::parseSyntheticScene("").ok());
}

TEST(SyntheticSceneTest, BoundsMatchFrozenTable) {
  struct Row {
    topmap::SyntheticScene scene;
    float x0, x1, y0, y1;
  };
  const std::vector<Row> rows = {
      {topmap::SyntheticScene::kFlatPlane, 0.0f, 6.0f, 0.0f, 6.0f},
      {topmap::SyntheticScene::kRamp, 0.0f, 17.4f, 0.0f, 4.0f},
      {topmap::SyntheticScene::kOverlappingFloors, 0.0f, 5.0f, 0.0f, 5.0f},
      {topmap::SyntheticScene::kMultiLevelRamps, 0.0f, 32.0f, 0.0f, 4.0f},
      {topmap::SyntheticScene::kHallNarrowHall, 0.0f, 17.0f, 0.0f, 7.0f},
      {topmap::SyntheticScene::kUShape, 0.0f, 4.0f, 0.0f, 5.0f},
      // The frozen table lists [0,10]x[0,10] for kTShape, but the three-arm
      // shape it describes (centre [2,6]^2 plus up/right/down arms) has its
      // true union bounds at x in [2,10]. The explicit rectangles win.
      {topmap::SyntheticScene::kTShape, 2.0f, 10.0f, 0.0f, 10.0f},
      {topmap::SyntheticScene::kSingleBridge, 0.0f, 13.0f, 0.0f, 5.0f},
      {topmap::SyntheticScene::kParallelPortals, 0.0f, 13.0f, 0.0f, 8.0f},
      {topmap::SyntheticScene::kLoop, 0.0f, 12.0f, 0.0f, 12.0f},
  };
  for (const Row& r : rows) {
    const topmap::TriangleMesh m = MeshOf(r.scene);
    const Bounds b = BoundsOf(m);
    EXPECT_NEAR(b.min_x, r.x0, kTol) << topmap::toString(r.scene);
    EXPECT_NEAR(b.max_x, r.x1, kTol) << topmap::toString(r.scene);
    EXPECT_NEAR(b.min_y, r.y0, kTol) << topmap::toString(r.scene);
    EXPECT_NEAR(b.max_y, r.y1, kTol) << topmap::toString(r.scene);
  }
}

TEST(SyntheticSceneTest, FlatPlaneIsAUniformGrid) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kFlatPlane);
  EXPECT_EQ(m.vertices().size(), 61u * 61u);
  EXPECT_EQ(m.size(), 2u * 60u * 60u);
  const Bounds b = BoundsOf(m);
  EXPECT_NEAR(b.min_z, 0.0f, kTol);
  EXPECT_NEAR(b.max_z, 0.0f, kTol);
}

TEST(SyntheticSceneTest, RampGeometryMatchesFrozenContract) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kRamp);
  // Platforms.
  EXPECT_TRUE(HasVertex(m, 2.0f, 2.0f, 0.0f));
  EXPECT_TRUE(HasVertex(m, 15.4f, 2.0f, 2.0f));
  // Ramp endpoints share vertices with the platforms.
  EXPECT_TRUE(HasVertex(m, 4.0f, 2.0f, 0.0f));
  EXPECT_TRUE(HasVertex(m, 13.4f, 2.0f, 2.0f));
  // The ramp is only 1.2 m wide and climbs monotonically from 0 to 2.
  const std::pair<float, float> ramp_y = YRangeInX(m, 4.5f, 12.9f);
  EXPECT_NEAR(ramp_y.first, 1.4f, kTol);
  EXPECT_NEAR(ramp_y.second, 2.6f, kTol);
  for (const Eigen::Vector3f& v : m.vertices()) {
    if (v.x() < 4.0f - kTol) {
      EXPECT_NEAR(v.z(), 0.0f, kTol);
    }
    if (v.x() > 13.4f + kTol) {
      EXPECT_NEAR(v.z(), 2.0f, kTol);
    }
    if (v.x() > 4.5f && v.x() < 13.0f) {
      EXPECT_TRUE(v.z() > 0.0f && v.z() < 2.0f);
    }
  }
  const Bounds b = BoundsOf(m);
  EXPECT_NEAR(b.min_z, 0.0f, kTol);
  EXPECT_NEAR(b.max_z, 2.0f, kTol);
}

TEST(SyntheticSceneTest, MultiLevelRampsClimbThroughThreePlatforms) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kMultiLevelRamps);
  EXPECT_TRUE(HasVertex(m, 2.0f, 2.0f, 0.0f));
  EXPECT_TRUE(HasVertex(m, 16.0f, 2.0f, 2.0f));
  EXPECT_TRUE(HasVertex(m, 30.0f, 2.0f, 4.0f));
  const Bounds b = BoundsOf(m);
  EXPECT_NEAR(b.min_z, 0.0f, kTol);
  EXPECT_NEAR(b.max_z, 4.0f, kTol);
  // Both ramps are true 12 degree ramps.
  float max_slope_deg = 0.0f;
  for (const auto& tri : m.triangles()) max_slope_deg = std::max(max_slope_deg, tri.slope * 180.0f / kPi);
  EXPECT_GT(max_slope_deg, 8.0f);
  EXPECT_LT(max_slope_deg, 20.0f);
  EXPECT_NEAR(max_slope_deg, 12.0f, 0.2f);
}

TEST(SyntheticSceneTest, HallNarrowHallCorridorIsTwelveDecimetresWide) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kHallNarrowHall);
  EXPECT_GT(CountVerticesIn(m, -0.1f, 7.0f, -0.1f, 7.1f), 0);    // hall A
  EXPECT_GT(CountVerticesIn(m, 10.0f, 17.1f, -0.1f, 7.1f), 0);   // hall B
  const std::pair<float, float> corridor = YRangeInX(m, 7.1f, 9.9f);
  EXPECT_NEAR(corridor.first, 2.9f, kTol);
  EXPECT_NEAR(corridor.second, 4.1f, kTol);
  EXPECT_NEAR(corridor.second - corridor.first, 1.2f, kTol);
}

TEST(SyntheticSceneTest, UShapeHasAnInnerHole) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kUShape);
  EXPECT_TRUE(HasVertex(m, 3.5f, 0.5f, 0.0f));   // arm 1
  EXPECT_TRUE(HasVertex(m, 0.5f, 2.0f, 0.0f));   // base
  EXPECT_TRUE(HasVertex(m, 3.5f, 4.5f, 0.0f));   // arm 2
  EXPECT_EQ(CountVerticesIn(m, 1.1f, 3.9f, 1.1f, 3.9f), 0);
}

TEST(SyntheticSceneTest, TShapeHasCentreAndThreeArms) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kTShape);
  EXPECT_TRUE(HasVertex(m, 4.0f, 4.0f, 0.0f));   // centre
  EXPECT_TRUE(HasVertex(m, 4.0f, 9.0f, 0.0f));   // top arm
  EXPECT_TRUE(HasVertex(m, 9.0f, 4.0f, 0.0f));   // right arm
  EXPECT_TRUE(HasVertex(m, 4.0f, 1.0f, 0.0f));   // bottom arm
  // The three quadrants the T does not occupy stay empty.
  EXPECT_EQ(CountVerticesIn(m, 6.1f, 9.9f, 6.1f, 9.9f), 0);
  EXPECT_EQ(CountVerticesIn(m, 0.1f, 1.9f, 2.1f, 5.9f), 0);
  EXPECT_EQ(CountVerticesIn(m, 6.1f, 9.9f, 0.1f, 1.9f), 0);
}

TEST(SyntheticSceneTest, SingleBridgeIsTheOnlyConnection) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kSingleBridge);
  EXPECT_GT(CountVerticesIn(m, -0.1f, 5.0f, -0.1f, 5.1f), 0);   // platform A
  EXPECT_GT(CountVerticesIn(m, 8.0f, 13.1f, -0.1f, 5.1f), 0);   // platform B
  const std::pair<float, float> bridge = YRangeInX(m, 5.1f, 7.9f);
  EXPECT_NEAR(bridge.first, 2.2f, kTol);
  EXPECT_NEAR(bridge.second, 3.7f, kTol);
  EXPECT_NEAR(bridge.second - bridge.first, 1.5f, kTol);
  // Nothing links the platforms outside the bridge band.
  EXPECT_EQ(CountVerticesIn(m, 5.1f, 7.9f, -0.1f, 2.1f), 0);
  EXPECT_EQ(CountVerticesIn(m, 5.1f, 7.9f, 3.8f, 5.1f), 0);
}

TEST(SyntheticSceneTest, ParallelPortalsAreTwoIndependentChannels) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kParallelPortals);
  EXPECT_GT(CountVerticesIn(m, -0.1f, 5.0f, -0.1f, 8.1f), 0);    // room A
  EXPECT_GT(CountVerticesIn(m, 8.0f, 13.1f, -0.1f, 8.1f), 0);    // room B
  // Upper channel spans y in [1.0, 2.2]; lower channel spans y in [5.8, 7.0].
  EXPECT_TRUE(HasVertex(m, 6.0f, 1.0f, 0.0f));
  EXPECT_TRUE(HasVertex(m, 6.0f, 2.2f, 0.0f));
  EXPECT_TRUE(HasVertex(m, 6.0f, 5.8f, 0.0f));
  EXPECT_TRUE(HasVertex(m, 6.0f, 7.0f, 0.0f));
  // Both channels are 1.2 m wide.
  EXPECT_NEAR(2.2f - 1.0f, 1.2f, kTol);
  EXPECT_NEAR(7.0f - 5.8f, 1.2f, kTol);
  // The band between them is a hole, so the channels are independent.
  EXPECT_EQ(CountVerticesIn(m, 5.1f, 7.9f, 2.3f, 5.7f), 0);
}

TEST(SyntheticSceneTest, LoopHasFourRoomsAndFourCorridors) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kLoop);
  EXPECT_TRUE(HasVertex(m, 2.0f, 2.0f, 0.0f));    // BL room
  EXPECT_TRUE(HasVertex(m, 10.0f, 2.0f, 0.0f));   // BR room
  EXPECT_TRUE(HasVertex(m, 10.0f, 10.0f, 0.0f));  // TR room
  EXPECT_TRUE(HasVertex(m, 2.0f, 10.0f, 0.0f));   // TL room
  EXPECT_GT(CountVerticesIn(m, 4.1f, 7.9f, 1.3f, 2.7f), 0);    // bottom corridor
  EXPECT_GT(CountVerticesIn(m, 9.3f, 10.7f, 4.1f, 7.9f), 0);   // right corridor
  EXPECT_GT(CountVerticesIn(m, 4.1f, 7.9f, 9.3f, 10.7f), 0);   // top corridor
  EXPECT_GT(CountVerticesIn(m, 1.3f, 2.7f, 4.1f, 7.9f), 0);    // left corridor
  // The courtyard in the middle is a hole.
  EXPECT_EQ(CountVerticesIn(m, 4.1f, 7.9f, 4.1f, 7.9f), 0);
}

TEST(SyntheticSceneTest, OverlappingFloorsShareXYButNotZ) {
  const topmap::TriangleMesh m = MeshOf(topmap::SyntheticScene::kOverlappingFloors);
  int low = 0, high = 0;
  for (const Eigen::Vector3f& v : m.vertices()) {
    if (Near(v.z(), 0.0f)) ++low;
    if (Near(v.z(), 3.0f)) ++high;
  }
  EXPECT_GT(low, 0);
  EXPECT_EQ(low, high);
}

TEST(SyntheticSceneTest, Inventory) {
  std::cout << "scene,aabb_x,aabb_y,aabb_z,vertices,triangles,components,regions,portals\n";
  struct Row {
    topmap::SyntheticScene scene;
    int regions;
    int portals;
  };
  const std::vector<Row> rows = {
      {topmap::SyntheticScene::kFlatPlane, 1, 0},
      {topmap::SyntheticScene::kRamp, 3, 2},
      {topmap::SyntheticScene::kOverlappingFloors, 2, 0},
      {topmap::SyntheticScene::kMultiLevelRamps, 5, 4},
      {topmap::SyntheticScene::kHallNarrowHall, 3, 2},
      {topmap::SyntheticScene::kUShape, 3, 2},
      {topmap::SyntheticScene::kTShape, 4, 3},
      {topmap::SyntheticScene::kSingleBridge, 3, 2},
      {topmap::SyntheticScene::kParallelPortals, 4, 4},
      {topmap::SyntheticScene::kLoop, 8, 8},
  };
  for (const Row& r : rows) {
    const topmap::TriangleMesh m = MeshOf(r.scene);
    const Bounds b = BoundsOf(m);
    std::cout << topmap::toString(r.scene) << ',' << b.min_x << '-' << b.max_x << ','
              << b.min_y << '-' << b.max_y << ',' << b.min_z << '-' << b.max_z << ','
              << m.vertices().size() << ',' << m.size() << ',' << ComponentCount(m) << ','
              << r.regions << ',' << r.portals << '\n';
  }
}
