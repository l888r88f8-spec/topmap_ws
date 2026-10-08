#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/region_growing.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

topmap::TriangleMesh MeshOf(topmap::SyntheticScene s, topmap::TraversabilityParams* tp = nullptr) {
  topmap::TraversabilityParams local;
  auto d = topmap::makeSyntheticMesh(s);
  auto m = topmap::TriangleMesh::fromMeshData(d.take(), {});
  topmap::TraversabilityEvaluator::apply(m.value(), tp ? *tp : local);
  return m.take();
}

// A triangle with an up-facing normal; only the fields the score reads are set.
topmap::TriangleNode FlatTriangle(Eigen::Vector3f c) {
  topmap::TriangleNode t;
  t.centroid = c;
  t.normal = Eigen::Vector3f::UnitZ();
  t.slope = 0.0f;
  t.roughness = 0.0f;
  return t;
}

// Every region must be connected through shared edges (R20) and the regions
// must partition the triangle set without overlap.
void ExpectRegionsConnectedAndDisjoint(const topmap::TriangleMesh& mesh,
                                       const std::vector<topmap::SurfaceRegion>& regions) {
  std::vector<int> owner(mesh.size(), -1);
  for (const auto& region : regions) {
    ASSERT_FALSE(region.triangle_ids.empty()) << "region " << region.id << " is empty";
    std::set<topmap::TriangleId> members(region.triangle_ids.begin(), region.triangle_ids.end());
    for (topmap::TriangleId id : region.triangle_ids) {
      ASSERT_GE(id, 0);
      ASSERT_LT(id, static_cast<topmap::TriangleId>(mesh.size()));
      EXPECT_EQ(owner[id], -1) << "triangle " << id << " appears in two regions";
      owner[id] = region.id;
    }
    // BFS over shared edges restricted to this region.
    std::vector<topmap::TriangleId> stack{region.triangle_ids.front()};
    std::set<topmap::TriangleId> seen{region.triangle_ids.front()};
    while (!stack.empty()) {
      const topmap::TriangleId cur = stack.back();
      stack.pop_back();
      for (const topmap::TriangleEdge& e : mesh.triangle(cur).edges) {
        if (e.neighbor == topmap::kInvalidId) continue;
        if (members.count(e.neighbor) == 0) continue;
        if (seen.insert(e.neighbor).second) stack.push_back(e.neighbor);
      }
    }
    EXPECT_EQ(seen.size(), members.size()) << "region " << region.id << " is not edge-connected";
  }
}

}  // namespace

TEST(RegionGrowTest, FlatSceneIsSingleRegion) {
  auto m = MeshOf(topmap::SyntheticScene::kFlatPlane);
  topmap::RegionParams p;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), 1u);
  EXPECT_GT(r.value()[0].triangle_ids.size(), 10u);
}

TEST(RegionGrowTest, HallNarrowHallIsNotSplitByGrowing) {
  auto m = MeshOf(topmap::SyntheticScene::kHallNarrowHall);
  topmap::RegionParams p;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value().size(), 1u);   // the bottleneck cut belongs to T14
}

TEST(RegionGrowTest, TooFewTraversableTrianglesFails) {
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  d.faces = {{0, 1, 2}};
  auto mr = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(mr.ok());
  topmap::TriangleMesh m = mr.take();
  topmap::TraversabilityEvaluator::apply(m, topmap::TraversabilityParams{});
  topmap::RegionParams p;
  p.min_region_triangles = 5;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kRegionGrowingFailed);
}

TEST(RegionGrowTest, LongRampGrowsAsOneRegion) {
  // R2: a 12 degree / 2 m climb must not be fragmented by the height-span
  // constraint (an unconditional 0.30 m span would produce ~7 pieces).
  auto m = MeshOf(topmap::SyntheticScene::kRamp);
  topmap::RegionParams p;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_LE(r.value().size(), 3u) << "ramp was fragmented by height-span constraint";
  int ramp_triangles = 0;
  for (const auto& region : r.value()) {
    if (region.is_ramp) ramp_triangles += static_cast<int>(region.triangle_ids.size());
  }
  EXPECT_GT(ramp_triangles, 0);
}

TEST(RegionGrowTest, OverlappingFloorsGrowIntoTwoComponents) {
  auto m = MeshOf(topmap::SyntheticScene::kOverlappingFloors);
  topmap::RegionParams p;
  auto r = topmap::RegionGrower().grow(m, p);
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), 2u);
  std::set<float> heights;
  for (const auto& region : r.value()) heights.insert(std::round(region.centroid.z()));
  EXPECT_EQ(heights.size(), 2u);
}

TEST(RegionGrowTest, GrownRegionsAreEdgeConnected) {
  for (auto s : {topmap::SyntheticScene::kFlatPlane, topmap::SyntheticScene::kRamp,
                 topmap::SyntheticScene::kHallNarrowHall, topmap::SyntheticScene::kUShape,
                 topmap::SyntheticScene::kTShape, topmap::SyntheticScene::kSingleBridge,
                 topmap::SyntheticScene::kParallelPortals, topmap::SyntheticScene::kLoop}) {
    auto m = MeshOf(s);
    topmap::RegionParams p;
    auto r = topmap::RegionGrower().grow(m, p);
    ASSERT_TRUE(r.ok()) << topmap::toString(s) << ": " << r.status().message;
    ExpectRegionsConnectedAndDisjoint(m, r.value());
    // Every region triangle must be robot-placeable.
    for (const auto& region : r.value()) {
      for (topmap::TriangleId id : region.triangle_ids) {
        EXPECT_TRUE(topmap::TraversabilityEvaluator::isRobotPlaceable(
            m.triangle(id), topmap::TraversabilityParams{}));
      }
    }
  }
}

TEST(RegionGrowTest, JoinScoreUsesTraversabilityNormalisation) {
  topmap::RegionParams p;
  // Identical flat triangles -> zero score.
  EXPECT_NEAR(topmap::regionJoinScore(FlatTriangle(Eigen::Vector3f::Zero()),
                                      FlatTriangle(Eigen::Vector3f::Zero()), p),
              0.0f, 1e-6f);

  // A slope difference equal to max_slope_diff_deg (15 deg) normalises to 1.0,
  // so only the 0.5 slope weight remains. An un-normalised score would give
  // 0.5 * 15 deg in radians ~ 0.131.
  topmap::TriangleNode a = FlatTriangle(Eigen::Vector3f::Zero());
  topmap::TriangleNode b = FlatTriangle(Eigen::Vector3f::Zero());
  b.slope = 15.0f * kPi / 180.0f;
  EXPECT_NEAR(topmap::regionJoinScore(a, b, p), 0.5f, 1e-4f);

  // A roughness difference equal to max_roughness (0.03) normalises to 1.0, so
  // only the 0.5 roughness weight remains.
  topmap::TriangleNode c = FlatTriangle(Eigen::Vector3f::Zero());
  topmap::TriangleNode e = FlatTriangle(Eigen::Vector3f::Zero());
  e.roughness = 0.03f;
  EXPECT_NEAR(topmap::regionJoinScore(c, e, p), 0.5f, 1e-4f);

  // A real flat<->ramp pair must score above the threshold, which is what keeps
  // the ramp a separate region without any height-span fragmentation.
  topmap::TriangleNode flat;
  flat.normal = Eigen::Vector3f::UnitZ();
  flat.slope = 0.0f;
  flat.centroid = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  topmap::TriangleNode ramp;
  ramp.slope = 12.0f * kPi / 180.0f;
  ramp.normal = Eigen::Vector3f(0.0f, -std::sin(ramp.slope), std::cos(ramp.slope));
  ramp.centroid = Eigen::Vector3f(0.03f, 0.0f, 0.03f * std::tan(ramp.slope));
  EXPECT_GT(topmap::regionJoinScore(flat, ramp, p), p.region_score_threshold);
}
