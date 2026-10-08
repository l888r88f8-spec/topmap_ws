#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include <Eigen/Core>

#include "topmap_topology/topology_graph.hpp"

namespace {

using topmap::ErrorCode;
using topmap::Portal;
using topmap::PortalId;
using topmap::RegionId;
using topmap::SurfaceRegion;
using topmap::TopologyGraph;
using topmap::TriangleId;

SurfaceRegion Region(int id) {
  SurfaceRegion r;
  r.id = id;
  r.centroid = Eigen::Vector3f(static_cast<float>(id), 0.0f, 0.0f);
  return r;
}

SurfaceRegion RegionWithTriangles(int id, std::vector<TriangleId> triangles) {
  SurfaceRegion r = Region(id);
  r.triangle_ids = std::move(triangles);
  return r;
}

Portal MakePortal(int id, int a, int b, float cost = 1.0f) {
  Portal p;
  p.id = id;
  p.region_a = a;
  p.region_b = b;
  p.traversal_cost = cost;
  return p;
}

}  // namespace

// The three reference tests from the implementation plan (T17 step 1).
TEST(TopologyGraphTest, BuildsBipartiteAdjacency) {
  auto g = TopologyGraph::build({Region(0), Region(1), Region(2)},
                                {MakePortal(0, 0, 1, 1.0f), MakePortal(1, 1, 2, 2.0f)});
  ASSERT_TRUE(g.ok()) << g.status().message;
  EXPECT_EQ(g.value().portalsOf(1), (std::vector<PortalId>{0, 1}));
  EXPECT_EQ(g.value().neighborRegions(1), (std::vector<RegionId>{0, 2}));
  EXPECT_EQ(g.value().otherRegion(1, 1).value(), 2);
  EXPECT_EQ(g.value().connectedComponents(), 1);
}

TEST(TopologyGraphTest, DeletedPortalLeavesTwoComponents) {
  auto g = TopologyGraph::build({Region(0), Region(1)}, {MakePortal(0, 0, 1, 1.0f)});
  ASSERT_TRUE(g.ok());
  auto g2 = TopologyGraph::build({Region(0), Region(1)}, {});
  ASSERT_TRUE(g2.ok());
  EXPECT_EQ(g.value().connectedComponents(), 1);
  EXPECT_EQ(g2.value().connectedComponents(), 2);
}

TEST(TopologyGraphTest, InvalidRegionReferenceFails) {
  auto g = TopologyGraph::build({Region(0)}, {MakePortal(0, 0, 9, 1.0f)});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

// RegionId and PortalId are sparse: they are not vector indices, so every
// accessor must resolve through the id, not through a positional guess.
TEST(TopologyGraphTest, SparseRegionAndPortalIdsAreSupported) {
  auto g = TopologyGraph::build({Region(10), Region(20), Region(42)},
                                {MakePortal(100, 10, 20), MakePortal(200, 20, 42)});
  ASSERT_TRUE(g.ok()) << g.status().message;
  ASSERT_TRUE(g.value().hasRegion(20));
  EXPECT_EQ(g.value().region(42)->id, 42);
  EXPECT_EQ(g.value().portal(200)->id, 200);
  EXPECT_EQ(g.value().portalsOf(20), (std::vector<PortalId>{100, 200}));
  EXPECT_EQ(g.value().neighborRegions(20), (std::vector<RegionId>{10, 42}));
  EXPECT_EQ(g.value().otherRegion(100, 10).value(), 20);
  EXPECT_EQ(g.value().portalCountBetween(10, 20), 1u);
  EXPECT_EQ(g.value().connectedComponents(), 1);
  EXPECT_EQ(g.value().allOriginIds(), (std::vector<RegionId>{10, 20, 42}));
}

TEST(TopologyGraphTest, DuplicateRegionIdIsRejected) {
  auto g = TopologyGraph::build({Region(0), Region(0)}, {});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, DuplicatePortalIdIsRejected) {
  auto g = TopologyGraph::build({Region(0), Region(1)},
                                {MakePortal(7, 0, 1), MakePortal(7, 1, 0)});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, InvalidRegionIdIsRejected) {
  auto g = TopologyGraph::build({Region(0), Region(-1)}, {});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, InvalidPortalIdIsRejected) {
  auto g = TopologyGraph::build({Region(0), Region(1)}, {MakePortal(-1, 0, 1)});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, SelfLoopIsRejected) {
  auto g = TopologyGraph::build({Region(0), Region(1)}, {MakePortal(0, 1, 1)});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, MissingPortalEndpointIsRejected) {
  auto g = TopologyGraph::build({Region(0)}, {MakePortal(0, 0, 9)});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, TriangleLookupResolvesOwnerRegion) {
  auto g = TopologyGraph::build(
      {RegionWithTriangles(0, {0, 1}), RegionWithTriangles(1, {2, 3})}, {});
  ASSERT_TRUE(g.ok()) << g.status().message;
  EXPECT_EQ(g.value().regionOfTriangle(0).value(), 0);
  EXPECT_EQ(g.value().regionOfTriangle(1).value(), 0);
  EXPECT_EQ(g.value().regionOfTriangle(2).value(), 1);
  EXPECT_EQ(g.value().regionOfTriangle(3).value(), 1);
  EXPECT_FALSE(g.value().regionOfTriangle(4).ok());
  EXPECT_FALSE(g.value().regionOfTriangle(-1).ok());
}

TEST(TopologyGraphTest, TriangleOwnedByTwoRegionsIsRejected) {
  auto g = TopologyGraph::build(
      {RegionWithTriangles(0, {0, 1}), RegionWithTriangles(1, {1, 2})}, {});
  EXPECT_FALSE(g.ok());
  EXPECT_EQ(g.status().code, ErrorCode::kInvalidArgument);
}

TEST(TopologyGraphTest, EmptyPortalsIsLegal) {
  auto g = TopologyGraph::build({Region(5), Region(9)}, {});
  ASSERT_TRUE(g.ok()) << g.status().message;
  EXPECT_TRUE(g.value().portals().empty());
  EXPECT_EQ(g.value().connectedComponents(), 2);
  EXPECT_TRUE(g.value().portalsOf(5).empty());
  EXPECT_TRUE(g.value().neighborRegions(5).empty());
}

TEST(TopologyGraphTest, ParallelPortalsAreAllKeptAndNeighborsDeduped) {
  auto g = TopologyGraph::build({Region(0), Region(1)},
                                {MakePortal(2, 0, 1), MakePortal(0, 0, 1), MakePortal(1, 1, 0)});
  ASSERT_TRUE(g.ok()) << g.status().message;
  // Returns are ascending by id, independent of insertion order.
  EXPECT_EQ(g.value().portalsOf(0), (std::vector<PortalId>{0, 1, 2}));
  EXPECT_EQ(g.value().neighborRegions(0), (std::vector<RegionId>{1}));
  EXPECT_EQ(g.value().portalCountBetween(0, 1), 3u);
  EXPECT_EQ(g.value().portalCountBetween(1, 0), 3u);
  EXPECT_EQ(g.value().portalCountBetween(0, 0), 0u);
}

TEST(TopologyGraphTest, UnknownIdsQuerySafely) {
  auto g = TopologyGraph::build({Region(0)}, {}).take();
  EXPECT_FALSE(g.hasRegion(99));
  EXPECT_EQ(g.region(99), nullptr);
  EXPECT_EQ(g.portal(99), nullptr);
  EXPECT_TRUE(g.portalsOf(99).empty());
  EXPECT_TRUE(g.neighborRegions(99).empty());
  EXPECT_FALSE(g.otherRegion(99, 0).ok());
  EXPECT_FALSE(g.regionOfTriangle(0).ok());
}

TEST(TopologyGraphTest, OtherRegionRejectsNonIncidentRegion) {
  auto g = TopologyGraph::build({Region(0), Region(1), Region(2)},
                                {MakePortal(0, 0, 1)})
               .take();
  EXPECT_FALSE(g.otherRegion(0, 2).ok());
  EXPECT_EQ(g.otherRegion(0, 1).value(), 0);
}

TEST(TopologyGraphTest, DefaultOriginIsSelfAndReverseLookupWorks) {
  auto g = TopologyGraph::build({Region(3), Region(8), Region(11)}, {}).take();
  for (RegionId id : {3, 8, 11}) {
    ASSERT_EQ(g.originRegions(id), (std::vector<RegionId>{id}));
    auto cur = g.currentRegionOfOrigin(id);
    ASSERT_TRUE(cur.ok()) << cur.status().message;
    EXPECT_EQ(cur.value(), id);
  }
  EXPECT_EQ(g.allOriginIds(), (std::vector<RegionId>{3, 8, 11}));
  EXPECT_FALSE(g.currentRegionOfOrigin(99).ok());
}

TEST(TopologyGraphTest, SetOriginRegionsMergesAndRejectsConflict) {
  auto g = TopologyGraph::build({Region(0), Region(1), Region(2)}, {}).take();
  // A compressor first removes the absorbed region, then folds its origin into
  // the surviving super Region (this is the only legal order: while region 1
  // still exists, origin 1 belongs to it).
  g.replaceRegionsPortals({Region(0), Region(2)}, {});
  g.setOriginRegions(0, {0, 1});
  EXPECT_EQ(g.originRegions(0), (std::vector<RegionId>{0, 1}));
  EXPECT_EQ(g.currentRegionOfOrigin(1).value(), 0);
  EXPECT_EQ(g.allOriginIds(), (std::vector<RegionId>{0, 1, 2}));

  // Origin 1 already belongs to region 0: assigning it to region 2 with a
  // conflicting origin must leave the whole mapping untouched.
  g.setOriginRegions(2, {1});
  EXPECT_EQ(g.originRegions(2), (std::vector<RegionId>{2}));
  EXPECT_EQ(g.currentRegionOfOrigin(1).value(), 0);
  EXPECT_EQ(g.currentRegionOfOrigin(2).value(), 2);

  // Assigning a region to a completely missing id is a no-op.
  g.setOriginRegions(77, {77});
  EXPECT_FALSE(g.currentRegionOfOrigin(77).ok());
}

TEST(TopologyGraphTest, ReplaceRegionsPortalsIsTransactional) {
  auto g = TopologyGraph::build({Region(0), Region(1), Region(2)},
                                {MakePortal(0, 0, 1), MakePortal(1, 1, 2)})
               .take();
  const std::size_t regions_before = g.regions().size();
  const std::size_t portals_before = g.portals().size();

  // Invalid replacement (missing endpoint) must not leave a half-updated graph.
  g.replaceRegionsPortals({Region(0), Region(1)}, {MakePortal(0, 0, 9)});
  EXPECT_EQ(g.regions().size(), regions_before);
  EXPECT_EQ(g.portals().size(), portals_before);
  EXPECT_EQ(g.connectedComponents(), 1);
  EXPECT_EQ(g.otherRegion(0, 0).value(), 1);
  EXPECT_EQ(g.otherRegion(1, 1).value(), 2);

  // Self-loop and duplicate ids are rejected by the same validation.
  g.replaceRegionsPortals({Region(0), Region(1)}, {MakePortal(0, 1, 1)});
  EXPECT_EQ(g.regions().size(), regions_before);

  // A legal replacement commits atomically.
  g.replaceRegionsPortals({Region(0), Region(1)}, {MakePortal(0, 0, 1)});
  EXPECT_EQ(g.regions().size(), 2u);
  EXPECT_EQ(g.portals().size(), 1u);
  EXPECT_EQ(g.connectedComponents(), 1);
}

TEST(TopologyGraphTest, ReplaceRegionsPortalsRefreshesLookups) {
  auto g = TopologyGraph::build({Region(0), Region(1)}, {MakePortal(0, 0, 1)}).take();
  g.replaceRegionsPortals({Region(0), Region(1), Region(2)},
                          {MakePortal(0, 0, 1), MakePortal(1, 1, 2)});
  EXPECT_EQ(g.regions().size(), 3u);
  EXPECT_EQ(g.portalCountBetween(1, 2), 1u);
  EXPECT_EQ(g.portal(1)->id, 1);
  EXPECT_TRUE(g.portalCountBetween(0, 2) == 0u);
}
