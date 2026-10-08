#include <gtest/gtest.h>

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"

namespace {

struct Prepared {
  bool ok = false;
  std::string error;
  topmap::TriangleMesh mesh;
  std::vector<topmap::SurfaceRegion> regions;
};

Prepared GrowAndSplit(topmap::SyntheticScene s, const topmap::RegionParams& p) {
  Prepared out;
  auto d = topmap::makeSyntheticMesh(s);
  if (!d.ok()) {
    out.error = "makeSyntheticMesh: " + d.status().message;
    return out;
  }
  auto mr = topmap::TriangleMesh::fromMeshData(d.value(), {});
  if (!mr.ok()) {
    out.error = "fromMeshData: " + mr.status().message;
    return out;
  }
  out.mesh = mr.take();
  topmap::TraversabilityEvaluator::apply(out.mesh, topmap::TraversabilityParams{});
  auto g = topmap::RegionGrower().grow(out.mesh, p);
  if (!g.ok()) {
    out.error = "grow: " + g.status().message;
    return out;
  }
  auto sp = topmap::RegionSplitter().split(out.mesh, g.value(), p);
  if (!sp.ok()) {
    out.error = "split: " + sp.status().message;
    return out;
  }
  out.regions = sp.take();
  out.ok = true;
  return out;
}

void ExpectRegionsConnected(const Prepared& out) {
  std::vector<int> owner(out.mesh.size(), -1);
  for (const auto& region : out.regions) {
    ASSERT_FALSE(region.triangle_ids.empty());
    std::set<topmap::TriangleId> members(region.triangle_ids.begin(), region.triangle_ids.end());
    for (topmap::TriangleId id : region.triangle_ids) {
      EXPECT_EQ(owner[id], -1) << "triangle " << id << " is in two output regions";
      owner[id] = region.id;
    }
    std::vector<topmap::TriangleId> stack{region.triangle_ids.front()};
    std::set<topmap::TriangleId> seen{region.triangle_ids.front()};
    while (!stack.empty()) {
      const topmap::TriangleId cur = stack.back();
      stack.pop_back();
      for (const topmap::TriangleEdge& e : out.mesh.triangle(cur).edges) {
        if (e.neighbor == topmap::kInvalidId) continue;
        if (members.count(e.neighbor) == 0) continue;
        if (seen.insert(e.neighbor).second) stack.push_back(e.neighbor);
      }
    }
    EXPECT_EQ(seen.size(), members.size()) << "region " << region.id << " is not edge-connected";
  }
}

int RampRegions(const Prepared& out) {
  int n = 0;
  for (const auto& r : out.regions) {
    if (r.is_ramp) ++n;
  }
  return n;
}

}  // namespace

TEST(RegionSplitTest, HallNarrowHallSplitsIntoThree) {
  topmap::RegionParams p;
  auto out = GrowAndSplit(topmap::SyntheticScene::kHallNarrowHall, p);
  ASSERT_TRUE(out.ok) << out.error;
  ExpectRegionsConnected(out);
  EXPECT_EQ(out.regions.size(), 3u);
}

TEST(RegionSplitTest, BridgeAndParallelChannelsBecomeOwnRegions) {
  topmap::RegionParams p;
  auto bridge = GrowAndSplit(topmap::SyntheticScene::kSingleBridge, p);
  ASSERT_TRUE(bridge.ok) << bridge.error;
  ExpectRegionsConnected(bridge);
  EXPECT_EQ(bridge.regions.size(), 3u);

  auto parallel = GrowAndSplit(topmap::SyntheticScene::kParallelPortals, p);
  ASSERT_TRUE(parallel.ok) << parallel.error;
  ExpectRegionsConnected(parallel);
  EXPECT_EQ(parallel.regions.size(), 4u);
}

TEST(RegionSplitTest, UShapeAndTShapeSplitIntoLimbRegions) {
  topmap::RegionParams p;
  auto u = GrowAndSplit(topmap::SyntheticScene::kUShape, p);
  ASSERT_TRUE(u.ok) << u.error;
  ExpectRegionsConnected(u);
  EXPECT_EQ(u.regions.size(), 3u);

  auto t = GrowAndSplit(topmap::SyntheticScene::kTShape, p);
  ASSERT_TRUE(t.ok) << t.error;
  ExpectRegionsConnected(t);
  EXPECT_EQ(t.regions.size(), 4u);
}

TEST(RegionSplitTest, RampIsSeparatedFromFlatAndNotFragmented) {
  topmap::RegionParams p;
  auto out = GrowAndSplit(topmap::SyntheticScene::kRamp, p);
  ASSERT_TRUE(out.ok) << out.error;
  ExpectRegionsConnected(out);
  // R2: the 12 degree / 2 m climb must stay a single ramp region.
  EXPECT_EQ(out.regions.size(), 3u);
  EXPECT_EQ(RampRegions(out), 1);
}

TEST(RegionSplitTest, FlatPlaneIsNotSplit) {
  topmap::RegionParams p;
  auto out = GrowAndSplit(topmap::SyntheticScene::kFlatPlane, p);
  ASSERT_TRUE(out.ok) << out.error;
  ExpectRegionsConnected(out);
  EXPECT_EQ(out.regions.size(), 1u);
}

TEST(RegionSplitTest, LoopSplitsIntoEightRegions) {
  topmap::RegionParams p;
  auto out = GrowAndSplit(topmap::SyntheticScene::kLoop, p);
  ASSERT_TRUE(out.ok) << out.error;
  ExpectRegionsConnected(out);
  EXPECT_EQ(out.regions.size(), 8u);
}

TEST(RegionSplitTest, OverlappingFloorsAndMultiLevelRampsMatchFrozenTable) {
  topmap::RegionParams p;
  auto overlap = GrowAndSplit(topmap::SyntheticScene::kOverlappingFloors, p);
  ASSERT_TRUE(overlap.ok) << overlap.error;
  ExpectRegionsConnected(overlap);
  EXPECT_EQ(overlap.regions.size(), 2u);

  auto multi = GrowAndSplit(topmap::SyntheticScene::kMultiLevelRamps, p);
  ASSERT_TRUE(multi.ok) << multi.error;
  ExpectRegionsConnected(multi);
  EXPECT_EQ(multi.regions.size(), 5u);
  EXPECT_EQ(RampRegions(multi), 2);
}
