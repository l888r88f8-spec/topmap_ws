#include <gtest/gtest.h>

#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "test_support_plan.hpp"
#include "topmap_planner/corridor_builder.hpp"
#include "topmap_planner/topology_astar.hpp"
#include "topmap_surface/synthetic_scene.hpp"

namespace {

using topmap::Corridor;
using topmap::CorridorBuilder;
using topmap::ErrorCode;
using topmap::MeshData;
using topmap::PointId;
using topmap::Portal;
using topmap::PortalId;
using topmap::RegionId;
using topmap::SurfaceRegion;
using topmap::TopologyAStar;
using topmap::TopologyGraph;
using topmap::TopologyPath;
using topmap::TriangleId;
using topmap::TriangleMesh;

template <typename Id>
bool Contains(const std::vector<Id>& values, Id value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

struct Fixture {
  TriangleMesh mesh;
  TopologyGraph graph;
};

// A straight strip of `region_ids.size()` grid cells (each split into two
// triangles) laid out along +X, plus optionally one isolated quad in a second
// connected component. Consecutive strip cells are joined by a Portal whose
// center lies on their shared edge.
Fixture MakeStrip(const std::vector<RegionId>& region_ids, PortalId first_portal_id,
                  bool isolated_component) {
  MeshData data;
  for (int i = 0; i <= static_cast<int>(region_ids.size()); ++i) {
    data.vertices.emplace_back(static_cast<float>(i), 0.0f, 0.0f);
    data.vertices.emplace_back(static_cast<float>(i), 1.0f, 0.0f);
  }
  for (int i = 0; i < static_cast<int>(region_ids.size()); ++i) {
    const PointId a = static_cast<PointId>(2 * i);
    const PointId b = static_cast<PointId>(2 * i + 2);
    const PointId c = static_cast<PointId>(2 * i + 3);
    const PointId d = static_cast<PointId>(2 * i + 1);
    data.faces.push_back({a, b, c});
    data.faces.push_back({a, c, d});
  }
  if (isolated_component) {
    const PointId base = static_cast<PointId>(data.vertices.size());
    data.vertices.emplace_back(10.0f, 0.0f, 0.0f);
    data.vertices.emplace_back(11.0f, 0.0f, 0.0f);
    data.vertices.emplace_back(11.0f, 1.0f, 0.0f);
    data.vertices.emplace_back(10.0f, 1.0f, 0.0f);
    data.faces.push_back({base, base + 1, base + 2});
    data.faces.push_back({base, base + 2, base + 3});
  }

  auto mesh_result = TriangleMesh::fromMeshData(data, {});
  EXPECT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  TriangleMesh mesh = mesh_result.take();

  std::vector<SurfaceRegion> regions;
  for (std::size_t i = 0; i < region_ids.size(); ++i) {
    SurfaceRegion region;
    region.id = region_ids[i];
    region.triangle_ids = {static_cast<TriangleId>(2 * i), static_cast<TriangleId>(2 * i + 1)};
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (TriangleId t : region.triangle_ids) centroid += mesh.triangle(t).centroid;
    region.centroid = centroid / static_cast<float>(region.triangle_ids.size());
    regions.push_back(region);
  }
  if (isolated_component) {
    SurfaceRegion island;
    island.id = region_ids.back() + 1;
    island.triangle_ids = {8, 9};
    island.centroid = Eigen::Vector3f(10.5f, 0.5f, 0.0f);
    regions.push_back(island);
  }

  std::vector<Portal> portals;
  for (std::size_t i = 0; i + 1 < region_ids.size(); ++i) {
    Portal portal;
    portal.id = first_portal_id + static_cast<PortalId>(i);
    portal.region_a = region_ids[i];
    portal.region_b = region_ids[i + 1];
    portal.center = Eigen::Vector3f(static_cast<float>(i) + 1.0f, 0.5f, 0.0f);
    portal.traversal_cost = 1.0f;
    portal.valid_width = 1.0f;
    portals.push_back(portal);
  }

  auto graph_result = TopologyGraph::build(regions, portals);
  EXPECT_TRUE(graph_result.ok()) << graph_result.status().message;
  Fixture fixture;
  fixture.mesh = std::move(mesh);
  fixture.graph = graph_result.take();
  return fixture;
}

TEST(CorridorBuilderTest, CorridorIsStrictSubsetOnMultiRegionScene) {
  // R28: kLoop has 8 Regions but the topology path covers only part of the
  // ring, so the corridor is a strict subset of the mesh.
  auto built = topmap::test::Build(topmap::SyntheticScene::kLoop);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TriangleMesh& mesh = built.value().mesh;
  const TopologyGraph& graph = built.value().graph;
  ASSERT_GE(graph.regions().size(), 4u);

  auto path =
      TopologyAStar().search(graph, graph.regions().front().id, graph.regions().back().id);
  ASSERT_TRUE(path.ok()) << path.status().message;

  auto corridor = CorridorBuilder().build(mesh, graph, path.value(), 1);
  ASSERT_TRUE(corridor.ok()) << corridor.status().message;
  EXPECT_LT(corridor.value().triangles.size(), mesh.size());
  EXPECT_FALSE(corridor.value().regions.empty());
  EXPECT_EQ(corridor.value().regions.size(), path.value().regions.size());
}

TEST(CorridorBuilderTest, EmptyTopologyPathFails) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  TopologyPath empty;
  auto corridor = CorridorBuilder().build(built.value().mesh, built.value().graph, empty, 1);
  ASSERT_FALSE(corridor.ok());
  EXPECT_EQ(corridor.status().code, ErrorCode::kCorridorBuildFailed);
}

TEST(CorridorBuilderTest, RejectsMismatchedRegionPortalCount) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, false);

  TopologyPath too_many_regions;
  too_many_regions.regions = {0, 1, 2};
  too_many_regions.portals = {0};
  EXPECT_EQ(CorridorBuilder().build(fixture.mesh, fixture.graph, too_many_regions, 1).status().code,
            ErrorCode::kCorridorBuildFailed);

  TopologyPath too_many_portals;
  too_many_portals.regions = {0};
  too_many_portals.portals = {0};
  EXPECT_EQ(CorridorBuilder().build(fixture.mesh, fixture.graph, too_many_portals, 1).status().code,
            ErrorCode::kCorridorBuildFailed);
}

TEST(CorridorBuilderTest, RejectsNegativeExpansionRings) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, false);
  TopologyPath path;
  path.regions = {0, 1};
  path.portals = {0};
  auto corridor = CorridorBuilder().build(fixture.mesh, fixture.graph, path, -1);
  ASSERT_FALSE(corridor.ok());
  EXPECT_EQ(corridor.status().code, ErrorCode::kCorridorBuildFailed);
}

TEST(CorridorBuilderTest, RejectsUnknownRegionAndPortal) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, false);

  TopologyPath unknown_region;
  unknown_region.regions = {0, 99};
  unknown_region.portals = {0};
  EXPECT_EQ(
      CorridorBuilder().build(fixture.mesh, fixture.graph, unknown_region, 1).status().code,
      ErrorCode::kCorridorBuildFailed);

  TopologyPath unknown_portal;
  unknown_portal.regions = {0, 1};
  unknown_portal.portals = {99};
  EXPECT_EQ(
      CorridorBuilder().build(fixture.mesh, fixture.graph, unknown_portal, 1).status().code,
      ErrorCode::kCorridorBuildFailed);
}

TEST(CorridorBuilderTest, RejectsPortalNotConnectingAdjacentRegions) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, false);
  TopologyPath path;
  path.regions = {0, 2};
  path.portals = {0};  // Portal 0 connects 0<->1, not 0<->2.
  auto corridor = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 1);
  ASSERT_FALSE(corridor.ok());
  EXPECT_EQ(corridor.status().code, ErrorCode::kCorridorBuildFailed);
}

TEST(CorridorBuilderTest, SparseIdsArePreserved) {
  Fixture fixture = MakeStrip({100, 200, 300, 400}, 77, false);
  TopologyPath path;
  path.regions = {100, 200};
  path.portals = {77};
  auto corridor = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 1);

  ASSERT_TRUE(corridor.ok()) << corridor.status().message;
  EXPECT_EQ(corridor.value().regions, (std::vector<RegionId>{100, 200}));
  EXPECT_EQ(corridor.value().portals, (std::vector<PortalId>{77}));
}

TEST(CorridorBuilderTest, SingleRegionPathUsesOnlyThatRegionsTriangles) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, false);
  TopologyPath path;
  path.regions = {2};
  auto corridor = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 3);

  ASSERT_TRUE(corridor.ok()) << corridor.status().message;
  EXPECT_EQ(corridor.value().regions, (std::vector<RegionId>{2}));
  EXPECT_TRUE(corridor.value().portals.empty());
  EXPECT_EQ(corridor.value().triangles, (std::vector<TriangleId>{4, 5}));
}

TEST(CorridorBuilderTest, ExpansionGrowsPortalNeighborhood) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, false);
  TopologyPath path;
  path.regions = {0, 1};
  path.portals = {0};

  auto rings_zero = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 0);
  auto rings_one = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 1);
  auto rings_three = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 3);
  ASSERT_TRUE(rings_zero.ok()) << rings_zero.status().message;
  ASSERT_TRUE(rings_one.ok()) << rings_one.status().message;
  ASSERT_TRUE(rings_three.ok()) << rings_three.status().message;

  // portal_rounds = max(1, expansion_rings), so rings 0 and 1 coincide and stay
  // inside the two path Regions.
  EXPECT_EQ(rings_zero.value().triangles, rings_one.value().triangles);
  EXPECT_EQ(rings_one.value().triangles, (std::vector<TriangleId>{0, 1, 2, 3}));

  // Three rings reach the next (non-path) Region's triangles.
  for (TriangleId t : rings_one.value().triangles) {
    EXPECT_TRUE(Contains(rings_three.value().triangles, t));
  }
  EXPECT_LT(rings_one.value().triangles.size(), rings_three.value().triangles.size());
  EXPECT_TRUE(Contains(rings_three.value().triangles, static_cast<TriangleId>(5)));
  // Expansion neighbours must never leak into the declared Region list.
  EXPECT_EQ(rings_three.value().regions, (std::vector<RegionId>{0, 1}));
}

TEST(CorridorBuilderTest, DoesNotExpandAcrossComponents) {
  Fixture fixture = MakeStrip({0, 1, 2, 3}, 0, true);
  ASSERT_EQ(fixture.mesh.componentIds().size(), 10u);
  EXPECT_EQ(fixture.mesh.componentId(0), 0);
  EXPECT_EQ(fixture.mesh.componentId(8), 1);

  TopologyPath path;
  path.regions = {0, 1};
  path.portals = {0};
  auto corridor = CorridorBuilder().build(fixture.mesh, fixture.graph, path, 8);

  ASSERT_TRUE(corridor.ok()) << corridor.status().message;
  for (TriangleId t : corridor.value().triangles) {
    EXPECT_EQ(fixture.mesh.componentId(t), 0) << "triangle " << t << " crossed components";
  }
  EXPECT_FALSE(Contains(corridor.value().triangles, static_cast<TriangleId>(8)));
  EXPECT_FALSE(Contains(corridor.value().triangles, static_cast<TriangleId>(9)));
  EXPECT_EQ(corridor.value().regions, (std::vector<RegionId>{0, 1}));
}

}  // namespace
