#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_topology/topology_compressor.hpp"
#include "topmap_topology/topology_invariants.hpp"

namespace {

using topmap::CompressionParams;
using topmap::CompressionStats;
using topmap::Portal;
using topmap::PortalId;
using topmap::RegionId;
using topmap::SurfaceRegion;
using topmap::TopologyCompressor;
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
  r.mean_slope = 0.0f;
  r.roughness = 0.0f;
  return r;
}

SurfaceRegion RAt(int id, const Eigen::Vector3f& centroid, float area = 10.0f) {
  SurfaceRegion r = R(id);
  r.centroid = centroid;
  r.area = area;
  return r;
}

// R12-b: ramp Region (is_ramp set, slope inside the ramp regime).
SurfaceRegion Ramp(int id, float z = 0.0f) {
  SurfaceRegion r = R(id, z);
  r.is_ramp = true;
  r.mean_slope = 12.0f * kPi / 180.0f;
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

const topmap::RegionParams kRegionParams;

std::vector<RegionId> RegionIds(const TopologyGraph& g) {
  std::vector<RegionId> ids;
  for (const SurfaceRegion& r : g.regions()) ids.push_back(r.id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::vector<std::pair<PortalId, std::pair<RegionId, RegionId>>> PortalEdges(const TopologyGraph& g) {
  std::vector<std::pair<PortalId, std::pair<RegionId, RegionId>>> edges;
  for (const Portal& p : g.portals()) {
    RegionId a = p.region_a, b = p.region_b;
    if (a > b) std::swap(a, b);
    edges.push_back({p.id, {a, b}});
  }
  std::sort(edges.begin(), edges.end());
  return edges;
}

}  // namespace

// ---------------------------------------------------------------------------
// Reference tests from the implementation plan (T19 step 1).
// ---------------------------------------------------------------------------

TEST(TopologyCompressorTest, DegreeTwoChainRegionMerges) {
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  CompressionParams c;
  c.max_merge_cost = 1.0f;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().second.regions_before, 3);
  EXPECT_EQ(out.value().second.regions_after, 2);
  EXPECT_EQ(out.value().second.merges_applied, 1);
  std::string reason;
  EXPECT_TRUE(TopologyInvariants::sameConnectivity(TopologyInvariants::capture(g),
                                                   TopologyInvariants::capture(out.value().first),
                                                   &reason))
      << reason;
}

TEST(TopologyCompressorTest, OriginReachabilityIsPreservedAfterMerge) {
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  CompressionParams c;
  c.max_merge_cost = 1.0f;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok());
  const auto& ng = out.value().first;
  for (RegionId origin : {0, 1, 2}) {
    auto cur = ng.currentRegionOfOrigin(origin);
    ASSERT_TRUE(cur.ok()) << "origin " << origin << " lost";
  }
  EXPECT_EQ(ng.connectedComponents(), 1);
}

TEST(TopologyCompressorTest, UniqueCrossLayerPortalSurvives) {
  // R22: the real protection scenario -- 0(z=0) - 1(z=0) - 2(z=3); Portal(1,2)
  // is the only cross-layer connection. Region 1 is degree 2 and cheap to merge,
  // but contracting it would swallow the cross-layer portal.
  auto g = TopologyGraph::build({R(0, 0.0f), R(1, 0.0f), R(2, 3.0f)},
                                {P(0, 0, 1), P(1, 1, 2)})
               .take();
  CompressionParams c;
  c.max_merge_cost = 100.0f;
  c.preserve_topology = true;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok());
  auto cross = TopologyInvariants::uniqueCrossLayerPortals(out.value().first);
  EXPECT_EQ(cross.size(), 1u) << "unique cross-layer connection was lost";
  EXPECT_GE(out.value().first.portals().size(), 1u);
}

TEST(TopologyCompressorTest, LoopIsNotCollapsedIntoTree) {
  // Four-region loop, all degree 2; compression may remove regions but the
  // cycle_rank must stay 1.
  auto g = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3), P(3, 3, 0)})
               .take();
  CompressionParams c;
  c.max_merge_cost = 1.0f;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok());
  auto before = TopologyInvariants::capture(g);
  auto after = TopologyInvariants::capture(out.value().first);
  EXPECT_EQ(after.cycle_rank, before.cycle_rank);
}

TEST(TopologyCompressorTest, RampIsNotMergedIntoFlatRegion) {
  // R12-b: flat(0)-ramp(1)-flat(2); the ramp is degree 2 and cheap, but merging
  // would erase the is_ramp semantics, so it is structurally forbidden.
  auto g = TopologyGraph::build({R(0), Ramp(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  CompressionParams c;
  c.max_merge_cost = 1000.0f;
  c.forbid_cross_regime_merge = true;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().second.merges_applied, 0);
  EXPECT_GE(out.value().second.rejected_by_regime, 1);
  EXPECT_EQ(out.value().first.regions().size(), 3u);
  int ramps = 0;
  for (const auto& r : out.value().first.regions())
    if (r.is_ramp) ++ramps;
  EXPECT_EQ(ramps, 1);
}

TEST(TopologyCompressorTest, CostGateAloneWouldNotProtectRamp) {
  // R12-b counter-example: with the hard prohibition off, the same graph merges
  // the ramp once the cost gate is opened -- proving that a cost penalty alone
  // is not protection.
  auto g = TopologyGraph::build({R(0), Ramp(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  CompressionParams c;
  c.max_merge_cost = 1000.0f;
  c.forbid_cross_regime_merge = false;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().second.merges_applied, 1);
  EXPECT_EQ(out.value().first.regions().size(), 2u);
}

TEST(TopologyCompressorTest, FlatChainStillMergesWhenProhibitionIsOn) {
  // R12-b must not harm same-regime merges: an all-flat degree-2 chain compresses.
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  CompressionParams c;
  c.max_merge_cost = 1.0f;
  c.forbid_cross_regime_merge = true;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().second.merges_applied, 1);
  EXPECT_EQ(out.value().second.rejected_by_regime, 0);
}

TEST(TopologyCompressorTest, PreserveTopologyDisabledAllowsMoreMerges) {
  auto g = TopologyGraph::build({R(0, 0.0f), R(1, 0.0f), R(2, 3.0f)},
                                {P(0, 0, 1), P(1, 1, 2)})
               .take();
  CompressionParams strict;
  strict.max_merge_cost = 100.0f;
  strict.preserve_topology = true;
  CompressionParams loose;
  loose.max_merge_cost = 100.0f;
  loose.preserve_topology = false;
  auto a = TopologyCompressor().compress(g, strict, kRegionParams);
  auto b = TopologyCompressor().compress(g, loose, kRegionParams);
  ASSERT_TRUE(a.ok());
  ASSERT_TRUE(b.ok());
  EXPECT_GE(a.value().second.merges_rejected, 1);
  EXPECT_LE(b.value().second.regions_after, a.value().second.regions_after);
}

// ---------------------------------------------------------------------------
// Additional contract tests requested for T19.
// ---------------------------------------------------------------------------

TEST(TopologyCompressorTest, SparseIdsAndOriginsPreserved) {
  auto g = TopologyGraph::build({R(10), R(20), R(42)}, {P(100, 10, 20), P(200, 20, 42)}).take();
  CompressionParams c;
  c.max_merge_cost = 100.0f;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  const auto& ng = out.value().first;

  // Every original id must still be resolvable and none may be lost.
  for (RegionId origin : {10, 20, 42}) {
    EXPECT_TRUE(ng.currentRegionOfOrigin(origin).ok()) << "origin " << origin << " lost";
  }
  // Sparse ids stay sparse: region 10 survives and absorbs 20; 42 stays untouched.
  EXPECT_TRUE(ng.hasRegion(10));
  EXPECT_TRUE(ng.hasRegion(42));
  EXPECT_EQ(ng.currentRegionOfOrigin(20).value(), 10);
  EXPECT_EQ(ng.originRegions(10), (std::vector<RegionId>{10, 20}));
}

TEST(TopologyCompressorTest, TriangleOwnershipSurvivesMerge) {
  SurfaceRegion a = R(0);
  a.triangle_ids = {0, 1};
  SurfaceRegion b = R(1);
  b.triangle_ids = {2};
  SurfaceRegion c = R(2);
  c.triangle_ids = {3};
  auto g = TopologyGraph::build({a, b, c}, {P(0, 0, 1), P(1, 1, 2)}).take();
  CompressionParams p;
  p.max_merge_cost = 1.0f;
  auto out = TopologyCompressor().compress(g, p, kRegionParams);
  ASSERT_TRUE(out.ok());
  const auto& ng = out.value().first;

  auto owner0 = ng.regionOfTriangle(0);
  auto owner2 = ng.regionOfTriangle(2);
  ASSERT_TRUE(owner0.ok());
  ASSERT_TRUE(owner2.ok());
  EXPECT_EQ(owner0.value(), owner2.value());
  const SurfaceRegion* super = ng.region(owner0.value());
  ASSERT_NE(super, nullptr);
  EXPECT_NE(std::find(super->triangle_ids.begin(), super->triangle_ids.end(), 0),
            super->triangle_ids.end());
  EXPECT_NE(std::find(super->triangle_ids.begin(), super->triangle_ids.end(), 2),
            super->triangle_ids.end());
  EXPECT_EQ(ng.regionOfTriangle(3).value(), 2);
}

TEST(TopologyCompressorTest, ParallelPortalsArePreserved) {
  // 0 == 2 already carries two parallel portals; region 1 is the only
  // contractible link. Contracting 1 into 0 must reattach (1,2) as (0,2) and
  // keep all three parallel portals instead of deduplicating them.
  auto g = TopologyGraph::build({R(0), R(1), R(2)},
                                {P(0, 0, 1), P(1, 1, 2), P(2, 0, 2), P(3, 0, 2)})
               .take();
  CompressionParams c;
  c.max_merge_cost = 1.0f;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  const auto& ng = out.value().first;
  EXPECT_EQ(ng.regions().size(), 2u);
  EXPECT_EQ(ng.portalCountBetween(0, 2), 3u);
  EXPECT_EQ(ng.portals().size(), 3u);
  EXPECT_EQ(TopologyInvariants::capture(ng).cycle_rank,
            TopologyInvariants::capture(g).cycle_rank);
}

TEST(TopologyCompressorTest, MergeDirectionChoosesCheaperTarget) {
  {
    auto g = TopologyGraph::build({RAt(0, {-0.1f, 0, 0}), RAt(1, {0, 0, 0}), RAt(2, {100, 0, 0})},
                                  {P(0, 0, 1), P(1, 1, 2)})
                 .take();
    CompressionParams c;
    c.max_merge_cost = 1000.0f;
    auto out = TopologyCompressor().compress(g, c, kRegionParams);
    ASSERT_TRUE(out.ok());
    // Cheaper neighbour is 0, so region 1 is absorbed there.
    EXPECT_EQ(out.value().first.currentRegionOfOrigin(1).value(), 0);
  }
  {
    auto g = TopologyGraph::build({RAt(0, {100, 0, 0}), RAt(1, {0, 0, 0}), RAt(2, {0.1f, 0, 0})},
                                  {P(0, 0, 1), P(1, 1, 2)})
                 .take();
    CompressionParams c;
    c.max_merge_cost = 1000.0f;
    auto out = TopologyCompressor().compress(g, c, kRegionParams);
    ASSERT_TRUE(out.ok());
    EXPECT_EQ(out.value().first.currentRegionOfOrigin(1).value(), 2);
  }
}

TEST(TopologyCompressorTest, CostGateRejectionIsCounted) {
  // Default max_merge_cost (0.25) is below the chain cost; nothing merges.
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  const CompressionParams c;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.value().second.merges_applied, 0);
  EXPECT_GE(out.value().second.merges_rejected, 1);
  EXPECT_EQ(out.value().second.rejected_by_invariant, 0);
  EXPECT_EQ(out.value().second.rejected_by_regime, 0);
  EXPECT_EQ(out.value().first.regions().size(), 3u);
}

TEST(TopologyCompressorTest, DeterministicAcrossRunsAndInputOrder) {
  auto g = TopologyGraph::build({R(0), R(1), R(2), R(3)},
                                {P(0, 0, 1), P(1, 1, 2), P(2, 2, 3), P(3, 3, 0)})
               .take();
  auto h = TopologyGraph::build({R(2), R(0), R(3), R(1)},
                                {P(2, 2, 3), P(0, 0, 1), P(3, 3, 0), P(1, 1, 2)})
               .take();
  CompressionParams c;
  c.max_merge_cost = 1.0f;

  auto a = TopologyCompressor().compress(g, c, kRegionParams).take();
  auto b = TopologyCompressor().compress(g, c, kRegionParams).take();
  auto d = TopologyCompressor().compress(h, c, kRegionParams).take();

  EXPECT_EQ(RegionIds(a.first), RegionIds(b.first));
  EXPECT_EQ(PortalEdges(a.first), PortalEdges(b.first));
  EXPECT_EQ(a.second.merges_applied, b.second.merges_applied);
  EXPECT_EQ(a.second.merges_rejected, b.second.merges_rejected);

  EXPECT_EQ(RegionIds(a.first), RegionIds(d.first));
  EXPECT_EQ(PortalEdges(a.first), PortalEdges(d.first));
  EXPECT_EQ(a.second.merges_applied, d.second.merges_applied);
  EXPECT_EQ(a.second.merges_rejected, d.second.merges_rejected);
}

TEST(TopologyCompressorTest, MultiRoundCompressionTerminates) {
  std::vector<SurfaceRegion> regions;
  std::vector<Portal> portals;
  for (int i = 0; i < 10; ++i) regions.push_back(R(i));
  for (int i = 0; i + 1 < 10; ++i) portals.push_back(P(i, i, i + 1));
  auto g = TopologyGraph::build(regions, portals).take();

  CompressionParams c;
  c.max_merge_cost = 1.0f;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok()) << out.status().message;
  EXPECT_EQ(out.value().second.regions_before, 10);
  EXPECT_LT(out.value().second.regions_after, out.value().second.regions_before);
  EXPECT_EQ(out.value().second.merges_applied,
            out.value().second.regions_before - out.value().second.regions_after);
  EXPECT_EQ(TopologyInvariants::capture(out.value().first).cycle_rank,
            TopologyInvariants::capture(g).cycle_rank);
}

TEST(TopologyCompressorTest, EmptyGraphProducesZeroStats) {
  auto g = TopologyGraph::build({}, {}).take();
  const CompressionParams c;
  auto out = TopologyCompressor().compress(g, c, kRegionParams);
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.value().second.regions_before, 0);
  EXPECT_EQ(out.value().second.regions_after, 0);
  EXPECT_EQ(out.value().second.merges_applied, 0);
  EXPECT_EQ(out.value().second.merges_rejected, 0);
}

TEST(TopologyCompressorTest, InvalidParamsAreRejected) {
  auto g = TopologyGraph::build({R(0), R(1), R(2)}, {P(0, 0, 1), P(1, 1, 2)}).take();
  const auto is_invalid = [&g](const CompressionParams& c, const topmap::RegionParams& rp) {
    auto out = TopologyCompressor().compress(g, c, rp);
    return !out.ok() && out.status().code == topmap::ErrorCode::kInvalidArgument;
  };

  CompressionParams negative;
  negative.max_merge_cost = -1.0f;
  EXPECT_TRUE(is_invalid(negative, kRegionParams));

  CompressionParams nan_cost;
  nan_cost.max_merge_cost = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(is_invalid(nan_cost, kRegionParams));

  CompressionParams inf_cost;
  inf_cost.max_merge_cost = std::numeric_limits<float>::infinity();
  EXPECT_TRUE(is_invalid(inf_cost, kRegionParams));

  topmap::RegionParams negative_weight;
  negative_weight.slope_weight = -0.5f;
  EXPECT_TRUE(is_invalid(CompressionParams{}, negative_weight));

  topmap::RegionParams nan_weight;
  nan_weight.normal_weight = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(is_invalid(CompressionParams{}, nan_weight));
}
