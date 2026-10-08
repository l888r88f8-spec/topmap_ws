#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "test_support_plan.hpp"
#include "topmap_planner/hierarchical_planner.hpp"
#include "topmap_planner/safety_validator.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/region_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace {

using topmap::ErrorCode;
using topmap::HierarchicalPlanner;
using topmap::PlanRequest;
using topmap::SafetyValidator;
using topmap::SyntheticScene;
using topmap::TopmapConfig;
using topmap::TopologyGraph;
using topmap::TraversabilityEvaluator;
using topmap::TraversabilityParams;

struct Scene {
  topmap::test::Prepared prepared;
  Eigen::Vector3f start = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal = Eigen::Vector3f::Zero();
};

// Runs the real T12-T17 pipeline and resolves the frozen endpoints once.
topmap::Result<Scene> BuildScene(SyntheticScene scene,
                                 const TopmapConfig& config = TopmapConfig{}) {
  auto built = topmap::test::Build(scene, config);
  if (!built.ok()) return built.status();
  auto endpoints = topmap::makeSyntheticEndpoints(scene);
  if (!endpoints.ok()) return endpoints.status();

  Scene result;
  result.prepared = built.take();
  result.start = endpoints.value().first;
  result.goal = endpoints.value().second;
  return result;
}

PlanRequest MakeRequest(const Scene& scene) {
  PlanRequest request;
  request.start = scene.start;
  request.goal = scene.goal;
  return request;
}

float PathLength(const std::vector<Eigen::Vector3f>& path) {
  float length = 0.0f;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

int PlaceableTriangleCount(const topmap::TriangleMesh& mesh, const TraversabilityParams& trav) {
  int count = 0;
  for (topmap::TriangleId id = 0; id < static_cast<topmap::TriangleId>(mesh.size()); ++id) {
    if (TraversabilityEvaluator::isRobotPlaceable(mesh.triangle(id), trav)) ++count;
  }
  return count;
}

// Final-plan example 1: the happy path on the ramp runs the whole hierarchy and
// must not report a fallback.
TEST(HierarchicalPlannerTest, FullPipelineOnRamp) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kRamp, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;

  const auto r = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, MakeRequest(scene.value()));
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_TRUE(r.value().status.ok());
  EXPECT_GE(r.value().path.size(), 2u);
  EXPECT_GT(r.value().stats.path_length, 0.0f);
  EXPECT_FALSE(r.value().topology.portals.empty());
  EXPECT_FALSE(r.value().stats.fallback_used);
  EXPECT_GE(r.value().stats.topology_expanded_nodes, 1);
  EXPECT_GE(r.value().stats.surface_expanded_nodes, 1);
  EXPECT_GE(r.value().stats.regions_in_corridor, 1);
  EXPECT_GE(r.value().stats.triangles_in_corridor, 1);
  EXPECT_GT(r.value().stats.min_clearance, 0.0f);
}

// Final-plan example 2: the two overlapping floors are disconnected in the
// topology graph, so the plan is rejected before any surface search.
TEST(HierarchicalPlannerTest, UnreachableGoalReturnsTopologyError) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kOverlappingFloors, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;

  const auto r = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, MakeRequest(scene.value()));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, ErrorCode::kTopologyUnreachable);
}

// Final-plan example 3 (R29): the deterministic corridor-failure injection makes
// both CorridorBuilder attempts fail, so the planner must fall back to the
// full-mesh corridor and still succeed.
TEST(HierarchicalPlannerTest, FallsBackToFullMeshWhenCorridorIsUnusable) {
  const TopmapConfig base;
  auto scene = BuildScene(SyntheticScene::kHallNarrowHall, base);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;

  TopmapConfig cfg = base;
  cfg.plan.force_corridor_failure = true;
  cfg.plan.fallback_full_mesh = true;

  const auto r = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, MakeRequest(scene.value()));
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_TRUE(r.value().stats.fallback_used);
  EXPECT_GE(r.value().path.size(), 2u);
  // The stats describe the corridor actually used: the full placeable mesh.
  EXPECT_EQ(r.value().stats.regions_in_corridor,
            static_cast<int>(prepared.graph.regions().size()));
  EXPECT_EQ(r.value().stats.triangles_in_corridor,
            PlaceableTriangleCount(prepared.mesh, cfg.traversability));
}

// Final-plan example 4: with the fallback disabled the injected corridor failure
// must propagate unchanged.
TEST(HierarchicalPlannerTest, FallbackDisabledPropagatesCorridorError) {
  const TopmapConfig base;
  auto scene = BuildScene(SyntheticScene::kHallNarrowHall, base);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;

  TopmapConfig cfg = base;
  cfg.plan.force_corridor_failure = true;
  cfg.plan.fallback_full_mesh = false;

  const auto r = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, MakeRequest(scene.value()));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, ErrorCode::kCorridorBuildFailed);
}

// Final-plan example 5: a start and a goal off the surface are reported with
// distinct error codes.
TEST(HierarchicalPlannerTest, StartOffSurfaceIsDistinguishedFromGoal) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kFlatPlane, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;
  const Scene& resolved = scene.value();

  PlanRequest bad_start;
  bad_start.start = Eigen::Vector3f(0.0f, 0.0f, 100.0f);
  bad_start.goal = resolved.goal;
  const auto a = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, bad_start);
  ASSERT_FALSE(a.ok());
  EXPECT_EQ(a.status().code, ErrorCode::kStartNotOnSurface);

  PlanRequest bad_goal;
  bad_goal.start = resolved.start;
  bad_goal.goal = Eigen::Vector3f(0.0f, 0.0f, 100.0f);
  const auto b = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, bad_goal);
  ASSERT_FALSE(b.ok());
  EXPECT_EQ(b.status().code, ErrorCode::kGoalNotOnSurface);
}

// A same-Region topology path is valid: one Region, no Portals, and the plan
// still produces a usable two-point-or-longer path.
TEST(HierarchicalPlannerTest, SameRegionTopologyPathSucceeds) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kFlatPlane, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;
  ASSERT_EQ(prepared.graph.regions().size(), 1u);

  const auto r = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, MakeRequest(scene.value()));
  ASSERT_TRUE(r.ok()) << r.status().message;
  ASSERT_EQ(r.value().topology.regions.size(), 1u);
  EXPECT_TRUE(r.value().topology.portals.empty());
  EXPECT_GE(r.value().path.size(), 2u);
  EXPECT_FALSE(r.value().stats.fallback_used);

  // Degenerate start == goal still resolves to the same single-Region path.
  PlanRequest same_point;
  same_point.start = scene.value().start;
  same_point.goal = scene.value().start;
  const auto degenerate =
      HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, same_point);
  ASSERT_TRUE(degenerate.ok()) << degenerate.status().message;
  ASSERT_EQ(degenerate.value().topology.regions.size(), 1u);
}

// Non-finite requests must fail as invalid arguments before any search runs.
TEST(HierarchicalPlannerTest, RejectsNonFiniteRequest) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kFlatPlane, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  PlanRequest bad_start = MakeRequest(scene.value());
  bad_start.start = Eigen::Vector3f(nan, 0.0f, 0.0f);
  const auto a = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, bad_start);
  ASSERT_FALSE(a.ok());
  EXPECT_EQ(a.status().code, ErrorCode::kInvalidArgument);

  PlanRequest bad_goal = MakeRequest(scene.value());
  bad_goal.goal = Eigen::Vector3f(0.0f, inf, 0.0f);
  const auto b = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, bad_goal);
  ASSERT_FALSE(b.ok());
  EXPECT_EQ(b.status().code, ErrorCode::kInvalidArgument);
}

// Invalid planning parameters are rejected as kInvalidArgument deterministically
// instead of being misreported as a start/goal localization failure.
TEST(HierarchicalPlannerTest, RejectsInvalidConfig) {
  const TopmapConfig good;
  auto scene = BuildScene(SyntheticScene::kFlatPlane, good);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;
  const PlanRequest request = MakeRequest(scene.value());

  struct Case {
    const char* name;
    TopmapConfig config;
  };
  std::vector<Case> cases;
  {
    TopmapConfig c = good;
    c.plan.max_project_radius = -1.0f;
    cases.push_back({"negative max_project_radius", c});
  }
  {
    TopmapConfig c = good;
    c.plan.max_project_radius = 0.0f;
    cases.push_back({"zero max_project_radius", c});
  }
  {
    TopmapConfig c = good;
    c.plan.check_step = 0.0f;
    cases.push_back({"zero check_step", c});
  }
  {
    TopmapConfig c = good;
    c.plan.topology_weight = 0.0f;
    cases.push_back({"zero topology_weight", c});
  }
  {
    TopmapConfig c = good;
    c.plan.smoothing_iterations = -1;
    cases.push_back({"negative smoothing_iterations", c});
  }
  {
    TopmapConfig c = good;
    c.robot.max_path_curvature = 0.0f;
    cases.push_back({"zero max_path_curvature", c});
  }
  {
    TopmapConfig c = good;
    c.traversability.max_slope_deg = std::numeric_limits<float>::quiet_NaN();
    cases.push_back({"nan max_slope_deg", c});
  }

  for (const Case& item : cases) {
    const auto r =
        HierarchicalPlanner(item.config).plan(prepared.mesh, prepared.graph, request);
    ASSERT_FALSE(r.ok()) << item.name;
    EXPECT_EQ(r.status().code, ErrorCode::kInvalidArgument) << item.name;
  }
}

// The reported statistics are recomputed independently from the returned path.
TEST(HierarchicalPlannerTest, StatsDescribeReturnedPath) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kRamp, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;

  const auto r = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, MakeRequest(scene.value()));
  ASSERT_TRUE(r.ok()) << r.status().message;

  const auto report = SafetyValidator().validate(r.value().path, prepared.mesh, cfg.traversability,
                                                 cfg.robot, cfg.plan.check_step);
  ASSERT_TRUE(report.valid) << "returned path should pass the safety validator";
  EXPECT_NEAR(r.value().stats.min_clearance, report.min_clearance, 1e-4f);
  EXPECT_NEAR(r.value().stats.max_slope_deg, report.max_slope_deg, 1e-3f);
  EXPECT_NEAR(r.value().stats.max_curvature, report.max_curvature, 1e-4f);
  EXPECT_NEAR(r.value().stats.path_length, PathLength(r.value().path), 1e-3f);
  EXPECT_EQ(r.value().stats.regions_in_corridor,
            static_cast<int>(r.value().topology.regions.size()));
}

// Planning the same request twice returns identical geometry and counters.
TEST(HierarchicalPlannerTest, PlanIsDeterministic) {
  const TopmapConfig cfg;
  auto scene = BuildScene(SyntheticScene::kRamp, cfg);
  ASSERT_TRUE(scene.ok()) << scene.status().message;
  const auto& prepared = scene.value().prepared;
  const PlanRequest request = MakeRequest(scene.value());

  const auto first = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, request);
  const auto second = HierarchicalPlanner(cfg).plan(prepared.mesh, prepared.graph, request);
  ASSERT_TRUE(first.ok()) << first.status().message;
  ASSERT_TRUE(second.ok()) << second.status().message;

  ASSERT_EQ(first.value().path.size(), second.value().path.size());
  for (std::size_t i = 0; i < first.value().path.size(); ++i) {
    EXPECT_TRUE(first.value().path[i].isApprox(second.value().path[i]))
        << "path point " << i << " differs";
  }
  EXPECT_EQ(first.value().topology.regions, second.value().topology.regions);
  EXPECT_EQ(first.value().topology.portals, second.value().topology.portals);
  EXPECT_EQ(first.value().stats.regions_in_corridor, second.value().stats.regions_in_corridor);
  EXPECT_EQ(first.value().stats.triangles_in_corridor, second.value().stats.triangles_in_corridor);
  EXPECT_EQ(first.value().stats.topology_expanded_nodes,
            second.value().stats.topology_expanded_nodes);
  EXPECT_EQ(first.value().stats.surface_expanded_nodes,
            second.value().stats.surface_expanded_nodes);
  EXPECT_FLOAT_EQ(first.value().stats.path_length, second.value().stats.path_length);
  EXPECT_FLOAT_EQ(first.value().stats.min_clearance, second.value().stats.min_clearance);
  EXPECT_FLOAT_EQ(first.value().stats.max_slope_deg, second.value().stats.max_slope_deg);
  EXPECT_FLOAT_EQ(first.value().stats.max_curvature, second.value().stats.max_curvature);
  EXPECT_EQ(first.value().stats.fallback_used, second.value().stats.fallback_used);
}

// A geometrically disconnected Portal (fabricated on top of the real
// overlapping-floors map) yields a topology path but no surface path; the
// surface error must propagate in both fallback modes.
TEST(HierarchicalPlannerTest, SurfaceFailurePropagatesOnDisconnectedPortal) {
  const TopmapConfig cfg;
  auto built = topmap::test::Build(SyntheticScene::kOverlappingFloors, cfg);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const auto& mesh = built.value().mesh;
  const auto& graph = built.value().graph;
  ASSERT_EQ(graph.regions().size(), 2u);

  std::vector<topmap::SurfaceRegion> regions(graph.regions().begin(), graph.regions().end());
  topmap::Portal portal;
  portal.id = 0;
  portal.region_a = regions[0].id;
  portal.region_b = regions[1].id;
  portal.center = 0.5f * (regions[0].centroid + regions[1].centroid);
  portal.left = portal.center;
  portal.right = portal.center;
  const std::vector<topmap::Portal> portals{portal};
  auto fabricated = TopologyGraph::build(regions, portals);
  ASSERT_TRUE(fabricated.ok()) << fabricated.status().message;

  auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(endpoints.ok());
  PlanRequest request;
  request.start = endpoints.value().first;
  request.goal = endpoints.value().second;

  for (const bool fallback : {false, true}) {
    TopmapConfig mode = cfg;
    mode.plan.fallback_full_mesh = fallback;
    const auto r = HierarchicalPlanner(mode).plan(mesh, fabricated.value(), request);
    ASSERT_FALSE(r.ok()) << "fallback=" << fallback;
    EXPECT_EQ(r.status().code, ErrorCode::kSurfaceSearchFailed) << "fallback=" << fallback;
  }
}

}  // namespace
