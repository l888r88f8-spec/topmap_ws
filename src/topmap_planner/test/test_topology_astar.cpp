#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "topmap_planner/topology_astar.hpp"

namespace {

using topmap::ErrorCode;
using topmap::Portal;
using topmap::PortalId;
using topmap::RegionId;
using topmap::SurfaceRegion;
using topmap::TopologyAStar;
using topmap::TopologyGraph;
using topmap::TopologyPath;

// Region at (x, 0, 0).
SurfaceRegion R(RegionId id, float x) {
  SurfaceRegion region;
  region.id = id;
  region.centroid = Eigen::Vector3f(x, 0.0f, 0.0f);
  return region;
}

// Portal between a and b; its center sits midway between the two region
// centroids on the x axis, so distances are easy to reason about.
Portal P(PortalId id, RegionId a, RegionId b, float width, float cost) {
  Portal portal;
  portal.id = id;
  portal.region_a = a;
  portal.region_b = b;
  portal.valid_width = width;
  portal.center = Eigen::Vector3f(0.5f * (static_cast<float>(a) + static_cast<float>(b)), 0.0f, 0.0f);
  portal.traversal_cost = cost;
  return portal;
}

TopologyGraph Graph(std::vector<SurfaceRegion> regions, std::vector<Portal> portals) {
  auto result = TopologyGraph::build(regions, portals);
  EXPECT_TRUE(result.ok()) << result.status().message;
  return result.take();
}

TEST(TopologyAStarTest, FindsChainAndPortalOrder) {
  TopologyGraph graph = Graph({R(0, 0.0f), R(1, 1.0f), R(2, 2.0f), R(3, 3.0f)},
                              {P(0, 0, 1, 1.5f, 1.0f), P(1, 1, 2, 1.5f, 1.0f), P(2, 2, 3, 1.5f, 1.0f)});
  int expanded = 0;
  auto result = TopologyAStar().search(graph, 0, 3, 1.0f, &expanded);

  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value().regions, (std::vector<RegionId>{0, 1, 2, 3}));
  EXPECT_EQ(result.value().portals, (std::vector<PortalId>{0, 1, 2}));
  // 0.5 + (1.0 + 0.5) + 0.5 + (1.0 + 0.5) + 0.5 + (1.0 + 0.5) = 6.0
  EXPECT_FLOAT_EQ(result.value().cost, 6.0f);
  EXPECT_GT(expanded, 0);
  // Four Regions and three Portals, all of them reached and expanded.
  EXPECT_EQ(expanded, 7);
}

TEST(TopologyAStarTest, PrefersCheaperParallelPortal) {
  TopologyGraph graph =
      Graph({R(0, 0.0f), R(1, 1.0f)}, {P(0, 0, 1, 1.2f, 5.0f), P(1, 0, 1, 1.2f, 1.0f)});
  auto result = TopologyAStar().search(graph, 0, 1);

  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value().regions, (std::vector<RegionId>{0, 1}));
  EXPECT_EQ(result.value().portals, (std::vector<PortalId>{1}));
  EXPECT_FLOAT_EQ(result.value().cost, 2.0f);  // 0.5 + (1.0 + 0.5)
}

TEST(TopologyAStarTest, ParallelEqualCostPrefersLowerPortalId) {
  // The tie-break tuple (f, g, type, id) must be strict and deterministic.
  TopologyGraph graph =
      Graph({R(0, 0.0f), R(1, 1.0f)}, {P(0, 0, 1, 1.2f, 1.0f), P(1, 0, 1, 1.2f, 1.0f)});
  auto first = TopologyAStar().search(graph, 0, 1);
  auto second = TopologyAStar().search(graph, 0, 1);

  ASSERT_TRUE(first.ok()) << first.status().message;
  ASSERT_TRUE(second.ok()) << second.status().message;
  EXPECT_EQ(first.value().portals, (std::vector<PortalId>{0}));
  EXPECT_EQ(second.value().portals, first.value().portals);
  EXPECT_EQ(second.value().regions, first.value().regions);
  EXPECT_FLOAT_EQ(second.value().cost, first.value().cost);
}

TEST(TopologyAStarTest, HandlesSparseRegionAndPortalIds) {
  // Sparse ids must never be used as vector positions. The portals are built by
  // hand because P() derives its center from the (dense) region ids.
  Portal first = P(5, 10, 20, 1.5f, 1.0f);
  first.center = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
  Portal second = P(7, 20, 30, 1.5f, 1.0f);
  second.center = Eigen::Vector3f(1.5f, 0.0f, 0.0f);

  TopologyGraph graph =
      Graph({R(10, 0.0f), R(20, 1.0f), R(30, 2.0f)}, {first, second});
  int expanded = 0;
  auto result = TopologyAStar().search(graph, 10, 30, 1.0f, &expanded);

  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value().regions, (std::vector<RegionId>{10, 20, 30}));
  EXPECT_EQ(result.value().portals, (std::vector<PortalId>{5, 7}));
  EXPECT_FLOAT_EQ(result.value().cost, 4.0f);
  EXPECT_EQ(expanded, 5);
}

TEST(TopologyAStarTest, StartEqualsGoalIsTrivial) {
  TopologyGraph graph = Graph({R(4, 0.0f), R(9, 1.0f)}, {P(0, 4, 9, 1.5f, 1.0f)});
  int expanded = -1;
  auto result = TopologyAStar().search(graph, 4, 4, 1.0f, &expanded);

  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value().regions, (std::vector<RegionId>{4}));
  EXPECT_TRUE(result.value().portals.empty());
  EXPECT_FLOAT_EQ(result.value().cost, 0.0f);
  EXPECT_EQ(expanded, 0);
}

TEST(TopologyAStarTest, UnreachableIsReported) {
  TopologyGraph graph = Graph({R(0, 0.0f), R(1, 1.0f)}, {});
  auto result = TopologyAStar().search(graph, 0, 1);

  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kTopologyUnreachable);
}

TEST(TopologyAStarTest, RejectsInvalidWeightAndInitializesExpandedNodes) {
  TopologyGraph graph =
      Graph({R(0, 0.0f), R(1, 1.0f)}, {P(0, 0, 1, 1.5f, 1.0f)});

  for (float weight : {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                       std::numeric_limits<float>::quiet_NaN()}) {
    int expanded = 7;
    auto result = TopologyAStar().search(graph, 0, 1, weight, &expanded);
    ASSERT_FALSE(result.ok()) << "weight " << weight << " should be rejected";
    EXPECT_EQ(result.status().code, ErrorCode::kInvalidArgument);
    EXPECT_EQ(expanded, 0);
  }
}

TEST(TopologyAStarTest, RejectsUnknownStartOrGoalRegion) {
  TopologyGraph graph =
      Graph({R(0, 0.0f), R(1, 1.0f)}, {P(0, 0, 1, 1.5f, 1.0f)});

  EXPECT_EQ(TopologyAStar().search(graph, 99, 1).status().code, ErrorCode::kInvalidArgument);
  EXPECT_EQ(TopologyAStar().search(graph, 0, 99).status().code, ErrorCode::kInvalidArgument);
  EXPECT_EQ(TopologyAStar().search(graph, 99, 99).status().code, ErrorCode::kInvalidArgument);
}

}  // namespace
