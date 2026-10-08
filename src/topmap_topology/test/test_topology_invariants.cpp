#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "topmap_topology/topology_invariants.hpp"

namespace {

using topmap::GraphSignature;
using topmap::Portal;
using topmap::PortalId;
using topmap::RegionId;
using topmap::SurfaceRegion;
using topmap::TopologyGraph;
using topmap::TopologyInvariants;

constexpr float kPi = 3.14159265358979323846f;

SurfaceRegion R(int id, float z = 0.0f) {
  SurfaceRegion r;
  r.id = id;
  r.area = 10.0f;
  r.min_z = z;
  r.max_z = z + 0.2f;
  r.centroid = Eigen::Vector3f(static_cast<float>(id), 0.0f, z);
  return r;
}

SurfaceRegion RTri(int id, std::vector<topmap::TriangleId> triangles) {
  SurfaceRegion r = R(id);
  r.triangle_ids = std::move(triangles);
  return r;
}

Portal P(int id, int a, int b) {
  Portal p;
  p.id = id;
  p.region_a = a;
  p.region_b = b;
  p.valid_width = 1.5f;
  return p;
}

}  // namespace

// ---------------------------------------------------------------------------
// Reference tests from the implementation plan (T18 step 1).
// ---------------------------------------------------------------------------

TEST(TopologyInvariantsTest, BridgePortalDetected) {
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)});
  ASSERT_TRUE(g.ok()) << g.status().message;
  EXPECT_EQ(TopologyInvariants::bridgePortals(g.value()).size(), 2u);
  auto sig = TopologyInvariants::capture(g.value());
  EXPECT_EQ(sig.connected_components, 1);
  EXPECT_EQ(sig.critical_origin_pairs.size(), 2u);
  EXPECT_EQ(sig.cycle_rank, 0);
  EXPECT_EQ(sig.origin_component_of.size(), 3u);
}

TEST(TopologyInvariantsTest, ParallelPortalPairNotCritical) {
  auto g = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)});
  ASSERT_TRUE(g.ok());
  auto sig = TopologyInvariants::capture(g.value());
  EXPECT_TRUE(sig.critical_origin_pairs.empty());
  EXPECT_EQ(sig.parallel_origin_pairs.size(), 1u);
  EXPECT_EQ(sig.cycle_rank, 1);  // 2 portals - 2 regions + 1 component
}

TEST(TopologyInvariantsTest, CycleRankCountsLoops) {
  auto g = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3), P(3, 3, 0)});
  ASSERT_TRUE(g.ok());
  auto sig = TopologyInvariants::capture(g.value());
  EXPECT_EQ(sig.cycle_rank, 1);
  EXPECT_TRUE(TopologyInvariants::bridgePortals(g.value()).empty());
}

TEST(TopologyInvariantsTest, ReachabilityLossIsDetected) {
  auto dense = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  auto broken = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1)}).take();
  std::string reason;
  EXPECT_FALSE(TopologyInvariants::sameConnectivity(TopologyInvariants::capture(dense),
                                                    TopologyInvariants::capture(broken), &reason));
  EXPECT_NE(reason.find("reachab"), std::string::npos) << reason;
}

TEST(TopologyInvariantsTest, CycleLossIsDetected) {
  auto looped = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)}).take();
  auto flat = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1)}).take();
  std::string reason;
  EXPECT_FALSE(TopologyInvariants::sameConnectivity(TopologyInvariants::capture(looped),
                                                    TopologyInvariants::capture(flat), &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(TopologyInvariantsTest, DegreeTwoRegionIsContractibleButArticulation) {
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  auto arts = TopologyInvariants::articulationRegions(g);
  EXPECT_NE(std::find(arts.begin(), arts.end(), 1), arts.end());
  auto blocked = TopologyInvariants::nonContractibleRegions(g);
  EXPECT_EQ(std::find(blocked.begin(), blocked.end(), 1), blocked.end());
  EXPECT_NE(std::find(blocked.begin(), blocked.end(), 0), blocked.end());
}

TEST(TopologyInvariantsTest, SlopeRegimeDistinguishesRampFromFlat) {
  const topmap::RegionParams rp;
  SurfaceRegion flat_a = R(0);
  SurfaceRegion flat_b = R(1);
  SurfaceRegion ramp = R(2);
  ramp.is_ramp = true;
  ramp.mean_slope = 12.0f * kPi / 180.0f;
  EXPECT_TRUE(topmap::sameSlopeRegime(flat_a, flat_b, rp));
  EXPECT_FALSE(topmap::sameSlopeRegime(flat_a, ramp, rp));
  EXPECT_TRUE(topmap::sameSlopeRegime(ramp, ramp, rp));
}

TEST(TopologyInvariantsTest, UniqueCrossLayerPortalIsProtected) {
  auto g = TopologyGraph::build({R(0, 0.0f), R(1, 3.0f)}, {P(0, 0, 1)}).take();
  auto cross = TopologyInvariants::uniqueCrossLayerPortals(g);
  EXPECT_EQ(cross.size(), 1u);
  EXPECT_EQ(cross[0], 0);
}

// ---------------------------------------------------------------------------
// Additional contract tests requested for T18.
// ---------------------------------------------------------------------------

TEST(TopologyInvariantsTest, SparseIdsAreCapturedCanonically) {
  auto g = TopologyGraph::build({R(10), R(20), R(42)},
                                {P(100, 10, 20), P(200, 20, 42)})
               .take();
  auto sig = TopologyInvariants::capture(g);
  EXPECT_EQ(sig.region_count, 3);
  EXPECT_EQ(sig.connected_components, 1);
  EXPECT_EQ(sig.cycle_rank, 0);
  EXPECT_EQ(TopologyInvariants::bridgePortals(g),
            (std::vector<PortalId>{100, 200}));
  EXPECT_EQ(sig.origin_component_of,
            (std::vector<std::pair<RegionId, RegionId>>{{10, 10}, {20, 10}, {42, 10}}));
  EXPECT_EQ(sig.critical_origin_pairs,
            (std::vector<std::pair<RegionId, RegionId>>{{10, 20}, {20, 42}}));
  // Same graph, shifted ids: the canonical origin pairs stay sorted.
  auto h = TopologyGraph::build({R(42), R(10), R(20)},
                                {P(200, 20, 42), P(100, 10, 20)})
               .take();
  EXPECT_EQ(TopologyInvariants::capture(h).critical_origin_pairs, sig.critical_origin_pairs);
}

TEST(TopologyInvariantsTest, TriangleOwnershipIsResolvable) {
  auto g = TopologyGraph::build({RTri(10, {0, 1}), RTri(20, {2})}, {}).take();
  EXPECT_EQ(g.regionOfTriangle(0).value(), 10);
  EXPECT_EQ(g.regionOfTriangle(2).value(), 20);
  EXPECT_FALSE(g.regionOfTriangle(7).ok());
}

TEST(TopologyInvariantsTest, InvalidBuildsAreRejected) {
  EXPECT_FALSE(TopologyGraph::build({R(0), R(0)}, {}).ok());          // duplicate region id
  EXPECT_FALSE(TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(0, 1, 0)}).ok());  // duplicate portal
  EXPECT_FALSE(TopologyGraph::build({R(0)}, {P(0, 0, 0)}).ok());     // self loop
  EXPECT_FALSE(TopologyGraph::build({R(0)}, {P(0, 0, 9)}).ok());     // missing endpoint
}

TEST(TopologyInvariantsTest, ParallelEdgesAreNotBridges) {
  // 0 == 1 (two portals), 1 -- 2 -- 3. Only the last two are bridges.
  auto g = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                {P(0, 0, 1), P(1, 0, 1), P(2, 1, 2), P(3, 2, 3)})
               .take();
  EXPECT_EQ(TopologyInvariants::bridgePortals(g), (std::vector<PortalId>{2, 3}));
  auto sig = TopologyInvariants::capture(g);
  EXPECT_EQ(sig.parallel_origin_pairs,
            (std::vector<std::pair<RegionId, RegionId>>{{0, 1}}));
  EXPECT_EQ(sig.critical_origin_pairs,
            (std::vector<std::pair<RegionId, RegionId>>{{1, 2}, {2, 3}}));
}

TEST(TopologyInvariantsTest, ParallelEdgesAreNotArticulation) {
  // Removing either endpoint of a double edge still leaves them connected.
  auto g = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)}).take();
  EXPECT_TRUE(TopologyInvariants::articulationRegions(g).empty());
}

TEST(TopologyInvariantsTest, ArticulationWithParallelEdgeToParent) {
  // 1 is a cut vertex even though it has two parallel portals to 0.
  auto g = TopologyGraph::build({R(0), R(1), R(2)},
                                {P(0, 0, 1), P(1, 0, 1), P(2, 1, 2)})
               .take();
  auto arts = TopologyInvariants::articulationRegions(g);
  EXPECT_NE(std::find(arts.begin(), arts.end(), 1), arts.end());
  EXPECT_EQ(std::find(arts.begin(), arts.end(), 0), arts.end());
  EXPECT_EQ(std::find(arts.begin(), arts.end(), 2), arts.end());
}

TEST(TopologyInvariantsTest, OriginMergePreservesSignature) {
  // Chain 0-1-2, then contract region 1 into region 0 exactly as T19 will.
  auto g = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3)})
               .take();
  const GraphSignature before = TopologyInvariants::capture(g);

  g.replaceRegionsPortals({R(0), R(2), R(3)}, {P(0, 0, 2), P(2, 2, 3)});
  g.setOriginRegions(0, {0, 1});
  ASSERT_EQ(g.currentRegionOfOrigin(1).value(), 0);
  const GraphSignature after = TopologyInvariants::capture(g);

  EXPECT_EQ(before.origin_component_of, after.origin_component_of);
  EXPECT_EQ(before.cycle_rank, after.cycle_rank);
  std::string reason;
  EXPECT_TRUE(TopologyInvariants::sameConnectivity(before, after, &reason)) << reason;
}

TEST(TopologyInvariantsTest, NewCriticalConnectionIsRejected) {
  // a has no bridge connecting 0 and 2 directly, so a b bridge 0-2 is new.
  GraphSignature a;
  a.connected_components = 1;
  a.origin_component_of = {{0, 0}, {1, 0}, {2, 0}};
  a.origin_region_of = {{0, 0}, {1, 1}, {2, 2}};
  a.critical_origin_pairs = {{0, 1}};
  a.cycle_rank = 0;
  GraphSignature b = a;
  b.critical_origin_pairs = {{0, 2}};
  std::string reason;
  EXPECT_FALSE(TopologyInvariants::sameConnectivity(a, b, &reason));
  EXPECT_NE(reason.find("critical"), std::string::npos) << reason;
}

TEST(TopologyInvariantsTest, ParallelPairDisappearanceRequiresMerge) {
  // a: two regions joined by two parallel portals (cycle rank 1).
  GraphSignature a;
  a.connected_components = 1;
  a.origin_component_of = {{0, 0}, {1, 0}};
  a.origin_region_of = {{0, 0}, {1, 1}};
  a.parallel_origin_pairs = {{0, 1}};
  a.cycle_rank = 1;

  // b: the parallel pair vanished because the two origins were merged into one
  // super Region. The rule is satisfied (other checks are held equal).
  GraphSignature merged = a;
  merged.parallel_origin_pairs.clear();
  merged.origin_region_of = {{0, 5}, {1, 5}};
  std::string reason;
  EXPECT_TRUE(TopologyInvariants::sameConnectivity(a, merged, &reason)) << reason;

  // c: the parallel pair vanished without a merge, so the connection was lost.
  GraphSignature unmerged = a;
  unmerged.parallel_origin_pairs.clear();
  unmerged.origin_region_of = {{0, 5}, {1, 6}};
  reason.clear();
  EXPECT_FALSE(TopologyInvariants::sameConnectivity(a, unmerged, &reason));
  EXPECT_NE(reason.find("parallel"), std::string::npos) << reason;
}

TEST(TopologyInvariantsTest, ParallelDisappearanceInRealGraph) {
  auto g = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)}).take();
  const GraphSignature before = TopologyInvariants::capture(g);
  ASSERT_EQ(before.parallel_origin_pairs.size(), 1u);

  // Contract both regions into one super Region; the parallel portals are gone
  // but the two origins are now the same current region.
  g.replaceRegionsPortals({R(7)}, {});
  g.setOriginRegions(7, {0, 1});
  const GraphSignature after = TopologyInvariants::capture(g);
  EXPECT_TRUE(after.parallel_origin_pairs.empty());
  EXPECT_EQ(after.origin_region_of,
            (std::vector<std::pair<RegionId, RegionId>>{{0, 7}, {1, 7}}));
  // cycle_rank necessarily drops when a 2-cycle is collapsed, so the full
  // signature check is expected to reject; this records that invariant.
  std::string reason;
  EXPECT_FALSE(TopologyInvariants::sameConnectivity(before, after, &reason));
  EXPECT_NE(reason.find("cycle"), std::string::npos) << reason;
}

TEST(TopologyInvariantsTest, CrossLayerAlternatePathIsNotUnique) {
  // 0 (z=0) -- 1 (z=3) is the direct cross-layer portal, but 0 can also reach 1
  // via 2 (z=0). Neither cross-layer portal is a bridge, so neither is unique.
  auto g = TopologyGraph::build({R(0, 0.0f), R(1, 3.0f), R(2, 0.0f)},
                                {P(0, 0, 1), P(1, 1, 2), P(2, 0, 2)})
               .take();
  EXPECT_TRUE(TopologyInvariants::uniqueCrossLayerPortals(g).empty());
}

TEST(TopologyInvariantsTest, ParallelCrossLayerPortalsAreNotUnique) {
  auto g = TopologyGraph::build({R(0, 0.0f), R(1, 3.0f)},
                                {P(0, 0, 1), P(1, 0, 1)})
               .take();
  EXPECT_TRUE(TopologyInvariants::uniqueCrossLayerPortals(g).empty());
}

TEST(TopologyInvariantsTest, SameLayerPortalIsNotCrossLayer) {
  auto g = TopologyGraph::build({R(0, 0.0f), R(1, 0.0f)}, {P(0, 0, 1)}).take();
  EXPECT_TRUE(TopologyInvariants::uniqueCrossLayerPortals(g).empty());
}

TEST(TopologyInvariantsTest, NonContractibleReasons) {
  // 0 -- 1 (two parallel portals): both degree 2 but collapsing would kill the
  // loop, so both are non-contractible.
  auto parallel = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)}).take();
  auto blocked = TopologyInvariants::nonContractibleRegions(parallel);
  EXPECT_EQ(blocked, (std::vector<RegionId>{0, 1}));

  // Unique cross-layer portal: region 1 has degree 2 with distinct neighbors,
  // yet it carries the only cross-height connection and must be protected.
  auto cross = TopologyGraph::build({R(0, 0.0f), R(1, 0.0f), R(2, 3.0f)},
                                    {P(0, 0, 1), P(1, 1, 2)})
                   .take();
  auto cross_blocked = TopologyInvariants::nonContractibleRegions(cross);
  EXPECT_NE(std::find(cross_blocked.begin(), cross_blocked.end(), 1), cross_blocked.end());
}

TEST(TopologyInvariantsTest, SetCriticalFlagsWritesBridgesBack) {
  auto chain = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  auto sig = TopologyInvariants::capture(chain);
  chain.setCriticalFlags(sig);
  for (const auto& p : chain.portals()) EXPECT_TRUE(p.is_critical);
  for (const auto& r : chain.regions()) EXPECT_TRUE(r.is_topology_critical);

  auto parallel = TopologyGraph::build({R(0), R(1)}, {P(0, 0, 1), P(1, 0, 1)}).take();
  parallel.setCriticalFlags(TopologyInvariants::capture(parallel));
  for (const auto& p : parallel.portals()) EXPECT_FALSE(p.is_critical);
  for (const auto& r : parallel.regions()) EXPECT_FALSE(r.is_topology_critical);

  auto loop = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                   {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3), P(3, 3, 0)})
                  .take();
  loop.setCriticalFlags(TopologyInvariants::capture(loop));
  for (const auto& p : loop.portals()) EXPECT_FALSE(p.is_critical);
  for (const auto& r : loop.regions()) EXPECT_FALSE(r.is_topology_critical);
}

TEST(TopologyInvariantsTest, SameSlopeRegimeFallsBackToMeanSlope) {
  const topmap::RegionParams rp;  // ramp_slope_deg = 8
  SurfaceRegion flat = R(0);
  SurfaceRegion steep = R(1);
  steep.mean_slope = 12.0f * kPi / 180.0f;
  SurfaceRegion gentle = R(2);
  gentle.mean_slope = 3.0f * kPi / 180.0f;
  SurfaceRegion flagged = R(3);
  flagged.is_ramp = true;  // flag wins even with a flat mean slope

  EXPECT_TRUE(topmap::sameSlopeRegime(flat, gentle, rp));
  EXPECT_FALSE(topmap::sameSlopeRegime(flat, steep, rp));
  EXPECT_TRUE(topmap::sameSlopeRegime(steep, steep, rp));
  EXPECT_FALSE(topmap::sameSlopeRegime(flat, flagged, rp));
}

TEST(TopologyInvariantsTest, DisconnectedComponentsAndEmptyGraph) {
  auto g = TopologyGraph::build({R(0), R(1), R(2)},
                                {P(0, 0, 1), P(1, 0, 1)})
               .take();
  auto sig = TopologyInvariants::capture(g);
  EXPECT_EQ(sig.connected_components, 2);
  EXPECT_EQ(sig.cycle_rank, 1);  // 2 - 3 + 2

  auto empty = TopologyGraph::build({}, {}).take();
  auto empty_sig = TopologyInvariants::capture(empty);
  EXPECT_EQ(empty_sig.region_count, 0);
  EXPECT_EQ(empty_sig.connected_components, 0);
  EXPECT_EQ(empty_sig.cycle_rank, 0);
  std::string reason;
  EXPECT_TRUE(TopologyInvariants::sameConnectivity(empty_sig, empty_sig, &reason));
}

TEST(TopologyInvariantsTest, CycleRankChangeIsReported) {
  auto looped = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                     {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3), P(3, 3, 0)})
                    .take();
  auto tree = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                   {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3)})
                  .take();
  std::string reason;
  EXPECT_FALSE(TopologyInvariants::sameConnectivity(TopologyInvariants::capture(looped),
                                                    TopologyInvariants::capture(tree), &reason));
  EXPECT_NE(reason.find("cycle"), std::string::npos) << reason;
}

TEST(TopologyInvariantsTest, SignaturesIgnoreRegionOrdering) {
  auto a = TopologyGraph::build({R(2), R(0), R(1)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  auto b = TopologyGraph::build({R(0), R(1), R(2)}, {P(1, 1, 2), P(0, 0, 1)}).take();
  auto sig_a = TopologyInvariants::capture(a);
  auto sig_b = TopologyInvariants::capture(b);
  EXPECT_EQ(sig_a.origin_component_of, sig_b.origin_component_of);
  EXPECT_EQ(sig_a.critical_origin_pairs, sig_b.critical_origin_pairs);
  std::string reason;
  EXPECT_TRUE(TopologyInvariants::sameConnectivity(sig_a, sig_b, &reason)) << reason;
}
