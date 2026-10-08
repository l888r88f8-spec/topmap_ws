#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "test_support_plan.hpp"
#include "topmap_planner/region_locator.hpp"
#include "topmap_surface/synthetic_scene.hpp"

namespace {

using topmap::ErrorCode;
using topmap::PlanParams;
using topmap::RegionId;
using topmap::RegionLocator;
using topmap::TopologyGraph;
using topmap::TriangleMesh;

TEST(RegionLocatorTest, ProjectsToNearestRegionOnRamp) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TriangleMesh& mesh = built.value().mesh;
  const TopologyGraph& graph = built.value().graph;
  auto endpoints = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok()) << endpoints.status().message;

  PlanParams params;
  auto start = RegionLocator().locate(mesh, graph, endpoints.value().first, params);
  ASSERT_TRUE(start.ok()) << start.status().message;
  EXPECT_NE(graph.region(start.value()), nullptr);

  auto goal = RegionLocator().locate(mesh, graph, endpoints.value().second, params);
  ASSERT_TRUE(goal.ok()) << goal.status().message;
  EXPECT_NE(graph.region(goal.value()), nullptr);
}

TEST(RegionLocatorTest, TooFarFromSurfaceFails) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TriangleMesh& mesh = built.value().mesh;
  const TopologyGraph& graph = built.value().graph;

  PlanParams params;
  auto result = RegionLocator().locate(mesh, graph, Eigen::Vector3f(0.0f, 0.0f, 100.0f), params);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kStartNotOnSurface);
}

TEST(RegionLocatorTest, DoesNotSnapThroughCeilingToLowerFloor) {
  // R6: the query point sits just below the upper floor (z=3) and is far from
  // the lower floor (z=0). Snapping must not pick the upper floor's back face.
  auto built = topmap::test::Build(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TriangleMesh& mesh = built.value().mesh;
  const TopologyGraph& graph = built.value().graph;

  PlanParams params;
  params.max_project_radius = 0.30f;
  auto result = RegionLocator().locate(mesh, graph, Eigen::Vector3f(2.5f, 2.5f, 2.90f), params);
  if (result.ok()) {
    const auto* region = graph.region(result.value());
    ASSERT_NE(region, nullptr);
    EXPECT_GT(region->min_z, 2.0f);
  } else {
    EXPECT_EQ(result.status().code, ErrorCode::kStartNotOnSurface);
  }
}

TEST(RegionLocatorTest, RejectsNonFinitePoint) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TriangleMesh& mesh = built.value().mesh;
  const TopologyGraph& graph = built.value().graph;

  PlanParams params;
  auto result =
      RegionLocator().locate(mesh, graph, Eigen::Vector3f(0.5f, 0.5f, std::nanf("")), params);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kInvalidArgument);
}

TEST(RegionLocatorTest, RejectsNegativeOrNonFiniteParams) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TriangleMesh& mesh = built.value().mesh;
  const TopologyGraph& graph = built.value().graph;
  const Eigen::Vector3f point(0.5f, 0.5f, 0.05f);

  PlanParams negative_radius;
  negative_radius.max_project_radius = -0.1f;
  EXPECT_EQ(RegionLocator().locate(mesh, graph, point, negative_radius).status().code,
            ErrorCode::kInvalidArgument);

  PlanParams infinite_radius;
  infinite_radius.max_project_radius = std::numeric_limits<float>::infinity();
  EXPECT_EQ(RegionLocator().locate(mesh, graph, point, infinite_radius).status().code,
            ErrorCode::kInvalidArgument);

  PlanParams negative_normal;
  negative_normal.max_project_normal_diff_deg = -1.0f;
  EXPECT_EQ(RegionLocator().locate(mesh, graph, point, negative_normal).status().code,
            ErrorCode::kInvalidArgument);

  PlanParams infinite_normal;
  infinite_normal.max_project_normal_diff_deg = std::numeric_limits<float>::infinity();
  EXPECT_EQ(RegionLocator().locate(mesh, graph, point, infinite_normal).status().code,
            ErrorCode::kInvalidArgument);
}

TEST(RegionLocatorTest, EmptyMeshAndEmptyGraphFailStructurally) {
  const TriangleMesh empty_mesh;
  auto empty_graph = TopologyGraph::build({}, {}).take();
  PlanParams params;

  auto on_empty_mesh =
      RegionLocator().locate(empty_mesh, empty_graph, Eigen::Vector3f::Zero(), params);
  EXPECT_EQ(on_empty_mesh.status().code, ErrorCode::kMeshEmpty);

  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  auto on_empty_graph = RegionLocator().locate(
      built.value().mesh, empty_graph, Eigen::Vector3f(0.5f, 0.5f, 0.05f), params);
  EXPECT_EQ(on_empty_graph.status().code, ErrorCode::kInvalidArgument);
}

TEST(RegionLocatorTest, BuildSupportHandlesScenesWithoutPortals) {
  // flat_plane and overlapping_floors legitimately have no cross-Region shared
  // edge, so PortalExtractor fails and the helper must fall back to an empty
  // Portal graph instead of propagating that error.
  auto flat = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(flat.ok()) << flat.status().message;
  EXPECT_FALSE(flat.value().graph.regions().empty());
  EXPECT_TRUE(flat.value().graph.portals().empty());

  auto floors = topmap::test::Build(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(floors.ok()) << floors.status().message;
  EXPECT_EQ(floors.value().graph.regions().size(), 2u);
  EXPECT_TRUE(floors.value().graph.portals().empty());
  EXPECT_EQ(floors.value().graph.connectedComponents(), 2);
}

}  // namespace
