#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_planner/corridor_builder.hpp"
#include "topmap_planner/region_locator.hpp"
#include "topmap_planner/safety_validator.hpp"
#include "topmap_planner/surface_astar.hpp"
#include "topmap_planner/topology_astar.hpp"
#include "test_support_plan.hpp"

namespace {

using topmap::MeshData;
using topmap::PathValidationReport;
using topmap::Portal;
using topmap::PortalId;
using topmap::RobotParams;
using topmap::SafetyValidator;
using topmap::SurfaceRegion;
using topmap::TopologyGraph;
using topmap::TraversabilityParams;
using topmap::TriangleMesh;

constexpr float kPi = 3.14159265358979323846f;

// Shared scene fixture: runs the real T12-T17 pipeline once per call.
topmap::test::Prepared BuildScene(topmap::SyntheticScene scene) {
  auto built = topmap::test::Build(scene);
  EXPECT_TRUE(built.ok()) << built.status().message;
  return built.take();
}

TriangleMesh MakeMesh(const std::vector<Eigen::Vector3f>& vertices,
                      const std::vector<std::array<topmap::PointId, 3>>& faces) {
  MeshData data;
  data.vertices = vertices;
  data.faces = faces;
  auto result = TriangleMesh::fromMeshData(data, {});
  EXPECT_TRUE(result.ok()) << result.status().message;
  return result.take();
}

// Builds the hall-narrow-hall topology path, corridor and Surface A* path used
// by the Portal-order tests.
struct HallPlan {
  topmap::test::Prepared prepared;
  std::vector<PortalId> expected;
  std::vector<Eigen::Vector3f> path;
};

HallPlan MakeHallPlan() {
  HallPlan plan;
  plan.prepared = BuildScene(topmap::SyntheticScene::kHallNarrowHall);
  const TriangleMesh& mesh = plan.prepared.mesh;
  const TopologyGraph& graph = plan.prepared.graph;

  auto topology = topmap::TopologyAStar().search(graph, graph.regions().front().id,
                                                 graph.regions().back().id);
  EXPECT_TRUE(topology.ok()) << topology.status().message;
  plan.expected = topology.value().portals;

  auto corridor = topmap::CorridorBuilder().build(mesh, graph, topology.value(), 1);
  EXPECT_TRUE(corridor.ok()) << corridor.status().message;

  auto endpoints = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kHallNarrowHall);
  std::vector<Eigen::Vector3f> waypoints{endpoints.value().first};
  for (PortalId pid : plan.expected) waypoints.push_back(graph.portal(pid)->center);
  waypoints.push_back(endpoints.value().second);

  auto path =
      topmap::SurfaceAStar().search(mesh, corridor.value(), waypoints, TraversabilityParams{});
  EXPECT_TRUE(path.ok()) << path.status().message;
  plan.path = path.value();
  return plan;
}

// Same plan construction for the sloped ramp scene, whose flat<->ramp Portal
// boundary chains are horizontal while the planner's chords sit above them.
HallPlan MakeRampPlan() {
  HallPlan plan;
  plan.prepared = BuildScene(topmap::SyntheticScene::kRamp);
  const TriangleMesh& mesh = plan.prepared.mesh;
  const TopologyGraph& graph = plan.prepared.graph;

  auto endpoints = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kRamp);
  EXPECT_TRUE(endpoints.ok()) << endpoints.status().message;
  const topmap::RegionLocator locator;
  const topmap::PlanParams params;
  auto start_region = locator.locate(mesh, graph, endpoints.value().first, params);
  auto goal_region = locator.locate(mesh, graph, endpoints.value().second, params);
  EXPECT_TRUE(start_region.ok()) << start_region.status().message;
  EXPECT_TRUE(goal_region.ok()) << goal_region.status().message;

  auto topology = topmap::TopologyAStar().search(graph, start_region.value(), goal_region.value(),
                                                 1.0f, nullptr);
  EXPECT_TRUE(topology.ok()) << topology.status().message;
  plan.expected = topology.value().portals;

  auto corridor = topmap::CorridorBuilder().build(mesh, graph, topology.value(), 1);
  EXPECT_TRUE(corridor.ok()) << corridor.status().message;

  std::vector<Eigen::Vector3f> waypoints{endpoints.value().first};
  for (PortalId pid : plan.expected) waypoints.push_back(graph.portal(pid)->center);
  waypoints.push_back(endpoints.value().second);

  auto path =
      topmap::SurfaceAStar().search(mesh, corridor.value(), waypoints, TraversabilityParams{});
  EXPECT_TRUE(path.ok()) << path.status().message;
  plan.path = path.value();
  return plan;
}

// Plan example: a three-point path near the centre of the flat plane is safe.
TEST(SafetyValidatorTest, ValidPathPasses) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{3.0f, 3.0f, 0}, {3.5f, 3.0f, 0}, {4.0f, 3.0f, 0}};
  auto rep = SafetyValidator().validate(path, prepared.mesh, TraversabilityParams{},
                                        RobotParams{}, 0.05f);
  EXPECT_TRUE(rep.valid) << "violations=" << rep.violating_indices.size();
}

// Plan example: a tall off-surface detour must fail.
TEST(SafetyValidatorTest, PointOffSurfaceFails) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{3.0f, 3.0f, 0}, {3.0f, 3.0f, 10.0f}, {4.0f, 3.0f, 0}};
  auto rep = SafetyValidator().validate(path, prepared.mesh, TraversabilityParams{},
                                        RobotParams{}, 0.05f);
  EXPECT_FALSE(rep.valid);
  EXPECT_FALSE(rep.violating_indices.empty());
}

// Plan example: locatePoint reports the true distance.
TEST(SafetyValidatorTest, LocatePointReturnsDistance) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  float d = -1.0f;
  auto t = SafetyValidator().locatePoint(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, 1.0f), 2.0f,
                                         &d);
  ASSERT_TRUE(t.ok()) << t.status().message;
  EXPECT_NEAR(d, 1.0f, 1e-3f);
}

// Plan example (R13): the Surface A* path follows the topology Portal order.
TEST(SafetyValidatorTest, PortalOrderIsVerifiable) {
  const HallPlan plan = MakeHallPlan();
  ASSERT_EQ(plan.expected.size(), 2u);
  EXPECT_EQ(SafetyValidator::crossedPortals(plan.path, plan.prepared.mesh, plan.prepared.graph),
            plan.expected);
  EXPECT_TRUE(SafetyValidator::portalOrderMatches(plan.path, plan.prepared.mesh,
                                                  plan.prepared.graph, plan.expected));
}

// A-03: on the sloped ramp the flat<->ramp boundary chain is horizontal but the
// path chord can lie above it in z, so the crossing must be detected as an XY
// projected boundary-chain intersection (a vertical curtain) rather than by the
// tilted local surface plane.
TEST(SafetyValidatorTest, CrossedPortalsDetectsRampBoundaryCrossing) {
  const HallPlan plan = MakeRampPlan();
  ASSERT_EQ(plan.expected.size(), 2u);
  EXPECT_EQ(SafetyValidator::crossedPortals(plan.path, plan.prepared.mesh, plan.prepared.graph),
            plan.expected);
  EXPECT_TRUE(SafetyValidator::portalOrderMatches(plan.path, plan.prepared.mesh,
                                                  plan.prepared.graph, plan.expected));
}

// A-03: a chord that passes over a ramp boundary chain in the XY plane must be
// detected even when its z is far from the (horizontal) chain, because the
// crossing test is a vertical curtain, not the tilted local surface plane.
TEST(SafetyValidatorTest, CrossedPortalsDetectsChordAboveRampBoundary) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kRamp);
  const TriangleMesh& mesh = prepared.mesh;
  const TopologyGraph& graph = prepared.graph;
  ASSERT_FALSE(graph.portals().empty());
  const Portal& portal = graph.portals().front();
  ASSERT_GE(portal.boundary_chain.size(), 2u);

  const Eigen::Vector3f c = mesh.vertices()[portal.boundary_chain.front()];
  const Eigen::Vector3f d = mesh.vertices()[portal.boundary_chain.back()];
  const Eigen::Vector3f midpoint = 0.5f * (c + d);
  Eigen::Vector2f direction(d.x() - c.x(), d.y() - c.y());
  if (direction.norm() < 1e-6f) direction = Eigen::Vector2f(1.0f, 0.0f);
  direction.normalize();
  const Eigen::Vector2f perpendicular(-direction.y(), direction.x());
  // A horizontal chord 0.6 m above the chain, crossing it at its midpoint.
  const float z = midpoint.z() + 0.6f;
  const std::vector<Eigen::Vector3f> path = {
      Eigen::Vector3f(midpoint.x() + perpendicular.x() * 0.5f,
                      midpoint.y() + perpendicular.y() * 0.5f, z),
      Eigen::Vector3f(midpoint.x() - perpendicular.x() * 0.5f,
                      midpoint.y() - perpendicular.y() * 0.5f, z)};

  const std::vector<PortalId> crossed =
      SafetyValidator::crossedPortals(path, mesh, graph);
  ASSERT_EQ(crossed.size(), 1u);
  EXPECT_EQ(crossed[0], portal.id);
}

// locatePoint must reject non-finite queries, invalid radii and empty meshes.
TEST(SafetyValidatorTest, LocatePointRejectsInvalidArguments) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (float radius : {-1.0f, nan, inf}) {
    auto r = validator.locatePoint(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, 0.0f), radius);
    EXPECT_FALSE(r.ok()) << "radius " << radius;
    if (!r.ok()) {
      EXPECT_EQ(r.status().code, topmap::ErrorCode::kInvalidArgument);
    }
  }
  EXPECT_FALSE(
      validator.locatePoint(prepared.mesh, Eigen::Vector3f(nan, 3.0f, 0.0f), 1.0f).ok());
  TriangleMesh empty;
  auto r = validator.locatePoint(empty, Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0f);
  EXPECT_FALSE(r.ok());
  if (!r.ok()) {
    EXPECT_EQ(r.status().code, topmap::ErrorCode::kInvalidArgument);
  }
}

// max_distance is an inclusive boundary.
TEST(SafetyValidatorTest, LocatePointHonoursMaxDistanceBoundary) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;
  float d = -1.0f;
  auto at_limit =
      validator.locatePoint(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, 1.0f), 1.0f, &d);
  ASSERT_TRUE(at_limit.ok()) << at_limit.status().message;
  EXPECT_NEAR(d, 1.0f, 1e-4f);
  auto beyond = validator.locatePoint(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, 1.0f), 0.999f);
  EXPECT_FALSE(beyond.ok());
  if (!beyond.ok()) {
    EXPECT_EQ(beyond.status().code, topmap::ErrorCode::kInvalidArgument);
  }
}

// Equidistant traversable triangles resolve to the smallest id.
TEST(SafetyValidatorTest, LocatePointTieBreaksBySmallestId) {
  std::vector<Eigen::Vector3f> vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0},
                                           {2, 0, 0}, {3, 0, 0}, {3, 1, 0}};
  std::vector<std::array<topmap::PointId, 3>> faces = {{0, 1, 2}, {3, 4, 5}};
  TriangleMesh mesh = MakeMesh(vertices, faces);
  topmap::TraversabilityEvaluator::apply(mesh, TraversabilityParams{});

  float d = -1.0f;
  auto result =
      SafetyValidator().locatePoint(mesh, Eigen::Vector3f(1.5f, 0.5f, 0.0f), 5.0f, &d);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_EQ(result.value(), 0);
  EXPECT_NEAR(d, 0.5f, 1e-4f);
}

// R6: an oriented query must not snap to the underside of the upper floor.
TEST(SafetyValidatorTest, LocatePointOrientedRejectsBackface) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kOverlappingFloors);
  const Eigen::Vector3f below_upper(2.5f, 2.5f, 2.5f);

  float unoriented_d = -1.0f;
  auto unoriented = SafetyValidator().locatePoint(prepared.mesh, below_upper, 1.0f, &unoriented_d);
  ASSERT_TRUE(unoriented.ok()) << unoriented.status().message;
  EXPECT_NEAR(unoriented_d, 0.5f, 1e-3f);

  float oriented_d = -1.0f;
  auto oriented =
      SafetyValidator().locatePointOriented(prepared.mesh, below_upper, 1.0f, 1.0f, &oriented_d);
  EXPECT_FALSE(oriented.ok());
  if (!oriented.ok()) {
    EXPECT_EQ(oriented.status().code, topmap::ErrorCode::kInvalidArgument);
  }
}

// R6: the normal-angle gate is enforced at its boundary.
TEST(SafetyValidatorTest, LocatePointOrientedHonoursNormalLimit) {
  const float slope_rad = 30.0f * kPi / 180.0f;
  const float rise = std::tan(slope_rad);
  std::vector<Eigen::Vector3f> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, rise}};
  std::vector<std::array<topmap::PointId, 3>> faces = {{0, 1, 2}};
  TriangleMesh mesh = MakeMesh(vertices, faces);
  TraversabilityParams trav;
  trav.max_slope_deg = 35.0f;
  topmap::TraversabilityEvaluator::apply(mesh, trav);

  const auto& triangle = mesh.triangle(0);
  const Eigen::Vector3f query = triangle.centroid + triangle.normal * 0.1f;

  float strict_d = -1.0f;
  auto strict =
      SafetyValidator().locatePointOriented(mesh, query, 1.0f, 25.0f * kPi / 180.0f, &strict_d);
  EXPECT_FALSE(strict.ok());

  float loose_d = -1.0f;
  auto loose =
      SafetyValidator().locatePointOriented(mesh, query, 1.0f, 35.0f * kPi / 180.0f, &loose_d);
  ASSERT_TRUE(loose.ok()) << loose.status().message;
  EXPECT_NEAR(loose_d, 0.1f, 1e-3f);
}

TEST(SafetyValidatorTest, LocatePointOrientedRejectsInvalidArguments) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(validator
                   .locatePointOriented(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, 0.0f), 1.0f, -1.0f)
                   .ok());
  EXPECT_FALSE(validator
                   .locatePointOriented(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, 0.0f), 1.0f, nan)
                   .ok());
  EXPECT_FALSE(validator
                   .locatePointOriented(prepared.mesh, Eigen::Vector3f(3.0f, 3.0f, nan), 1.0f, 1.0f)
                   .ok());
}

// Every malformed input yields valid == false without throwing.
TEST(SafetyValidatorTest, ValidateRejectsInvalidInputs) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;
  const TraversabilityParams trav;
  const RobotParams robot;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  std::vector<Eigen::Vector3f> good = {{3.0f, 3.0f, 0}, {3.5f, 3.0f, 0}};
  EXPECT_FALSE(validator.validate(good, prepared.mesh, trav, robot, 0.0f).valid);
  EXPECT_FALSE(validator.validate(good, prepared.mesh, trav, robot, -1.0f).valid);
  EXPECT_FALSE(validator.validate(good, prepared.mesh, trav, robot, nan).valid);

  std::vector<Eigen::Vector3f> nonfinite = {{3.0f, 3.0f, 0}, {nan, 3.0f, 0}};
  const auto nonfinite_report = validator.validate(nonfinite, prepared.mesh, trav, robot, 0.05f);
  EXPECT_FALSE(nonfinite_report.valid);
  EXPECT_FALSE(nonfinite_report.violating_indices.empty());

  std::vector<Eigen::Vector3f> single = {{3.0f, 3.0f, 0}};
  EXPECT_FALSE(validator.validate(single, prepared.mesh, trav, robot, 0.05f).valid);
  EXPECT_FALSE(validator.validate({}, prepared.mesh, trav, robot, 0.05f).valid);

  TriangleMesh empty;
  EXPECT_FALSE(validator.validate(good, empty, trav, robot, 0.05f).valid);

  TraversabilityParams bad_trav;
  bad_trav.required_clearance = -1.0f;
  EXPECT_FALSE(validator.validate(good, prepared.mesh, bad_trav, robot, 0.05f).valid);
  RobotParams bad_robot;
  bad_robot.max_path_curvature = 0.0f;
  EXPECT_FALSE(validator.validate(good, prepared.mesh, trav, bad_robot, 0.05f).valid);
}

// A safe path fills every statistic and leaves no violation.
TEST(SafetyValidatorTest, ValidateFillsStatsForSafePath) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{3.0f, 3.0f, 0}, {3.5f, 3.0f, 0}, {4.0f, 3.0f, 0}};
  auto rep = SafetyValidator().validate(path, prepared.mesh, TraversabilityParams{}, RobotParams{},
                                        0.05f);
  ASSERT_TRUE(rep.valid);
  EXPECT_GT(rep.min_clearance, 0.45f);
  EXPECT_NEAR(rep.max_step, 0.0f, 1e-4f);
  EXPECT_NEAR(rep.max_slope_deg, 0.0f, 1e-3f);
  EXPECT_NEAR(rep.max_curvature, 0.0f, 1e-3f);
  EXPECT_TRUE(rep.violating_indices.empty());
}

// A path hugging the mesh boundary has insufficient clearance.
TEST(SafetyValidatorTest, ValidateFlagsInsufficientClearance) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{1.0f, 0.2f, 0}, {2.0f, 0.2f, 0}, {3.0f, 0.2f, 0}};
  auto rep = SafetyValidator().validate(path, prepared.mesh, TraversabilityParams{}, RobotParams{},
                                        0.05f);
  EXPECT_FALSE(rep.valid);
  EXPECT_FALSE(rep.violating_indices.empty());
  EXPECT_LT(rep.min_clearance, 0.45f);
}

// A sharp turn exceeds robot.max_path_curvature while a gentle bend does not.
TEST(SafetyValidatorTest, ValidateFlagsExcessiveCurvature) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;

  std::vector<Eigen::Vector3f> sharp = {{2.5f, 3.0f, 0}, {3.0f, 3.0f, 0}, {3.0f, 3.5f, 0}};
  auto sharp_report =
      validator.validate(sharp, prepared.mesh, TraversabilityParams{}, RobotParams{}, 0.05f);
  EXPECT_FALSE(sharp_report.valid);
  EXPECT_GT(sharp_report.max_curvature, RobotParams{}.max_path_curvature);

  std::vector<Eigen::Vector3f> gentle = {{2.0f, 3.0f, 0}, {3.0f, 3.0f, 0}, {4.0f, 3.002f, 0}};
  auto gentle_report =
      validator.validate(gentle, prepared.mesh, TraversabilityParams{}, RobotParams{}, 0.05f);
  EXPECT_TRUE(gentle_report.valid) << "violations=" << gentle_report.violating_indices.size();
  EXPECT_LT(gentle_report.max_curvature, RobotParams{}.max_path_curvature);
  EXPECT_GT(sharp_report.max_curvature, gentle_report.max_curvature);
}

// A path that leaves the surface is rejected.
TEST(SafetyValidatorTest, ValidateFlagsUntraversableStep) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  std::vector<Eigen::Vector3f> path = {{3.0f, 3.0f, 0}, {3.0f, 3.0f, 0.5f}, {3.2f, 3.0f, 0.5f}};
  auto rep = SafetyValidator().validate(path, prepared.mesh, TraversabilityParams{}, RobotParams{},
                                        0.05f);
  EXPECT_FALSE(rep.valid);
  EXPECT_FALSE(rep.violating_indices.empty());
}

// Crossed-Portal order reverses with the path and repeats are de-duplicated.
TEST(SafetyValidatorTest, CrossedPortalsReverseAndRepeat) {
  const HallPlan plan = MakeHallPlan();
  ASSERT_EQ(plan.expected.size(), 2u);
  EXPECT_EQ(SafetyValidator::crossedPortals(plan.path, plan.prepared.mesh, plan.prepared.graph),
            plan.expected);

  std::vector<Eigen::Vector3f> reversed(plan.path.rbegin(), plan.path.rend());
  std::vector<PortalId> expected_reversed(plan.expected.rbegin(), plan.expected.rend());
  EXPECT_EQ(SafetyValidator::crossedPortals(reversed, plan.prepared.mesh, plan.prepared.graph),
            expected_reversed);
  EXPECT_FALSE(SafetyValidator::portalOrderMatches(reversed, plan.prepared.mesh, plan.prepared.graph,
                                                   plan.expected));
  EXPECT_TRUE(SafetyValidator::portalOrderMatches(reversed, plan.prepared.mesh, plan.prepared.graph,
                                                  expected_reversed));

  // There and back again: each Portal is crossed twice but reported once.
  std::vector<Eigen::Vector3f> there_and_back = plan.path;
  there_and_back.insert(there_and_back.end(), reversed.begin(), reversed.end());
  EXPECT_EQ(
      SafetyValidator::crossedPortals(there_and_back, plan.prepared.mesh, plan.prepared.graph),
      plan.expected);
}

// Explicit sequence semantics: order and cardinality both matter.
TEST(SafetyValidatorTest, PortalOrderMatchesRequiresExactSequence) {
  const HallPlan plan = MakeHallPlan();
  std::vector<PortalId> swapped(plan.expected.rbegin(), plan.expected.rend());
  EXPECT_FALSE(SafetyValidator::portalOrderMatches(plan.path, plan.prepared.mesh,
                                                   plan.prepared.graph, swapped));
  std::vector<PortalId> truncated(plan.expected.begin(), plan.expected.end() - 1);
  EXPECT_FALSE(SafetyValidator::portalOrderMatches(plan.path, plan.prepared.mesh,
                                                   plan.prepared.graph, truncated));
  std::vector<PortalId> extended = plan.expected;
  extended.push_back(plan.expected.back());
  EXPECT_FALSE(SafetyValidator::portalOrderMatches(plan.path, plan.prepared.mesh,
                                                   plan.prepared.graph, extended));
}

// Empty / degenerate Portal queries are safe.
TEST(SafetyValidatorTest, CrossedPortalsHandlesEmptyInputs) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  EXPECT_TRUE(SafetyValidator::crossedPortals({}, prepared.mesh, prepared.graph).empty());
  EXPECT_TRUE(
      SafetyValidator::crossedPortals({{0, 0, 0}}, prepared.mesh, prepared.graph).empty());
  EXPECT_TRUE(SafetyValidator::portalOrderMatches({}, prepared.mesh, prepared.graph, {}));
}

// Geometry must separate two Portals that join the same Region pair.
TEST(SafetyValidatorTest, CrossedPortalsDistinguishesParallelPortals) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kParallelPortals);
  const TriangleMesh& mesh = prepared.mesh;
  const TopologyGraph& graph = prepared.graph;
  ASSERT_GE(graph.portals().size(), 2u);
  const Portal& first = graph.portals()[0];
  const Portal& second = graph.portals()[1];
  ASSERT_GT((first.center - second.center).norm(), 1.0f);
  ASSERT_GE(first.boundary_chain.size(), 2u);
  ASSERT_GE(second.boundary_chain.size(), 2u);

  // Both Portals connect the very same Region pair, so only the boundary-chain
  // geometry can tell them apart.
  std::vector<SurfaceRegion> regions(2);
  regions[0].id = 0;
  regions[1].id = 1;
  std::vector<Portal> portals(2);
  for (int i = 0; i < 2; ++i) {
    const Portal& source = graph.portals()[static_cast<std::size_t>(i)];
    portals[i].id = static_cast<PortalId>(i);
    portals[i].region_a = 0;
    portals[i].region_b = 1;
    portals[i].boundary_chain = source.boundary_chain;
    portals[i].left = source.left;
    portals[i].right = source.right;
    portals[i].center = source.center;
  }
  auto built = TopologyGraph::build(regions, portals);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TopologyGraph remapped = built.take();
  ASSERT_EQ(remapped.portalCountBetween(0, 1), 2u);

  const auto crossing_path = [&](const Portal& portal) {
    Eigen::Vector3f midpoint = Eigen::Vector3f::Zero();
    for (topmap::PointId v : portal.boundary_chain) midpoint += mesh.vertices()[v];
    midpoint /= static_cast<float>(portal.boundary_chain.size());
    const Eigen::Vector3f a = mesh.vertices()[portal.boundary_chain.front()];
    const Eigen::Vector3f b = mesh.vertices()[portal.boundary_chain.back()];
    Eigen::Vector2f direction(b.x() - a.x(), b.y() - a.y());
    if (direction.norm() < 1e-6f) direction = Eigen::Vector2f(1.0f, 0.0f);
    direction.normalize();
    const Eigen::Vector2f perpendicular(-direction.y(), direction.x());
    return std::vector<Eigen::Vector3f>{
        midpoint + Eigen::Vector3f(perpendicular.x(), perpendicular.y(), 0.0f) * 0.3f,
        midpoint - Eigen::Vector3f(perpendicular.x(), perpendicular.y(), 0.0f) * 0.3f};
  };

  const auto crossed_first = SafetyValidator::crossedPortals(crossing_path(first), mesh, remapped);
  ASSERT_EQ(crossed_first.size(), 1u);
  EXPECT_EQ(crossed_first[0], 0);

  const auto crossed_second =
      SafetyValidator::crossedPortals(crossing_path(second), mesh, remapped);
  ASSERT_EQ(crossed_second.size(), 1u);
  EXPECT_EQ(crossed_second[0], 1);
}

// A path may hit a Portal exactly on a boundary-chain vertex (e.g. the planner
// routes through the Portal centre). The hit must be reported exactly once even
// though the two adjacent path segments touch and then leave the vertex.
TEST(SafetyValidatorTest, CrossedPortalsDetectsExactBoundaryVertexTouch) {
  // Two 0.1 m wide columns of quads share the vertical line x = 0.1. The Portal
  // chain is [v1, v4, v7], so v4 = (0.1, 0.1) is an interior chain vertex.
  std::vector<Eigen::Vector3f> vertices = {
      {0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}, {0.2f, 0.0f, 0.0f},
      {0.0f, 0.1f, 0.0f}, {0.1f, 0.1f, 0.0f}, {0.2f, 0.1f, 0.0f},
      {0.0f, 0.2f, 0.0f}, {0.1f, 0.2f, 0.0f}, {0.2f, 0.2f, 0.0f}};
  std::vector<std::array<topmap::PointId, 3>> faces = {{0, 1, 4}, {0, 4, 3}, {3, 4, 7}, {3, 7, 6},
                                                       {1, 2, 5}, {1, 5, 4}, {4, 5, 8}, {4, 8, 7}};
  TriangleMesh mesh = MakeMesh(vertices, faces);

  std::vector<SurfaceRegion> regions(2);
  regions[0].id = 0;
  regions[0].triangle_ids = {0, 1, 2, 3};
  regions[1].id = 1;
  regions[1].triangle_ids = {4, 5, 6, 7};
  std::vector<Portal> portals(1);
  portals[0].id = 0;
  portals[0].region_a = 0;
  portals[0].region_b = 1;
  portals[0].boundary_chain = {1, 4, 7};
  portals[0].center = Eigen::Vector3f(0.1f, 0.1f, 0.0f);
  auto built = TopologyGraph::build(regions, portals);
  ASSERT_TRUE(built.ok()) << built.status().message;
  const TopologyGraph graph = built.take();

  const std::vector<PortalId> expected{0};
  // Touch the interior vertex and leave it on the far side.
  EXPECT_EQ(SafetyValidator::crossedPortals(
                {{0.05f, 0.1f, 0.0f}, {0.1f, 0.1f, 0.0f}, {0.15f, 0.1f, 0.0f}}, mesh, graph),
            expected);
  // Same geometry traversed in reverse.
  EXPECT_EQ(SafetyValidator::crossedPortals(
                {{0.15f, 0.1f, 0.0f}, {0.1f, 0.1f, 0.0f}, {0.05f, 0.1f, 0.0f}}, mesh, graph),
            expected);
  // Touch and return to the same side: a hit is a hit, reported once.
  EXPECT_EQ(SafetyValidator::crossedPortals(
                {{0.15f, 0.1f, 0.0f}, {0.1f, 0.1f, 0.0f}, {0.15f, 0.15f, 0.0f}}, mesh, graph),
            expected);
  // A path parallel to the chain never reports a crossing.
  EXPECT_TRUE(SafetyValidator::crossedPortals({{0.05f, 0.0f, 0.0f}, {0.05f, 0.2f, 0.0f}}, mesh,
                                              graph)
                  .empty());
}

// A path that never touches a boundary chain reports no crossing.
TEST(SafetyValidatorTest, CrossedPortalsDoesNotInventCrossings) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kHallNarrowHall);
  ASSERT_FALSE(prepared.graph.portals().empty());
  std::vector<Eigen::Vector3f> path = {{1.0f, 1.0f, 0}, {2.0f, 1.0f, 0}};
  EXPECT_TRUE(
      SafetyValidator::crossedPortals(path, prepared.mesh, prepared.graph).empty());
}

// T33 batch API contract: malformed arguments are reported as kInvalidArgument
// and never as a silently empty result. An empty batch on a valid mesh is a
// legitimate empty answer, not an error.
TEST(SafetyValidatorTest, ValidateSegmentsRejectsInvalidArguments) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;
  const TraversabilityParams trav;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> good = {
      {Eigen::Vector3f(3.0f, 3.0f, 0.0f), Eigen::Vector3f(3.5f, 3.0f, 0.0f)}};

  for (float step : {0.0f, -1.0f, nan, inf}) {
    auto r = validator.validateSegments(good, prepared.mesh, trav, step);
    EXPECT_FALSE(r.ok()) << "check_step=" << step;
    if (!r.ok()) {
      EXPECT_EQ(r.status().code, topmap::ErrorCode::kInvalidArgument);
    }
  }

  TriangleMesh empty;
  auto on_empty = validator.validateSegments(good, empty, trav, 0.05f);
  EXPECT_FALSE(on_empty.ok());
  if (!on_empty.ok()) {
    EXPECT_EQ(on_empty.status().code, topmap::ErrorCode::kInvalidArgument);
  }

  const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> nonfinite = {
      {Eigen::Vector3f(3.0f, 3.0f, 0.0f), Eigen::Vector3f(nan, 3.0f, 0.0f)}};
  auto bad_point = validator.validateSegments(nonfinite, prepared.mesh, trav, 0.05f);
  EXPECT_FALSE(bad_point.ok());
  if (!bad_point.ok()) {
    EXPECT_EQ(bad_point.status().code, topmap::ErrorCode::kInvalidArgument);
  }

  TraversabilityParams bad_trav;
  bad_trav.required_clearance = -1.0f;
  auto bad_params = validator.validateSegments(good, prepared.mesh, bad_trav, 0.05f);
  EXPECT_FALSE(bad_params.ok());
  if (!bad_params.ok()) {
    EXPECT_EQ(bad_params.status().code, topmap::ErrorCode::kInvalidArgument);
  }

  auto none = validator.validateSegments({}, prepared.mesh, trav, 0.05f);
  ASSERT_TRUE(none.ok()) << none.status().message;
  EXPECT_TRUE(none.value().empty());
}

// T33 batch API correctness: each returned bit must agree with validate() run on
// the same two-point path, which is exactly the guarantee BuildPointGraph relies
// on to drop chords that leave the placeable free space.
TEST(SafetyValidatorTest, ValidateSegmentsMatchesValidatePerSegment) {
  const topmap::test::Prepared prepared = BuildScene(topmap::SyntheticScene::kFlatPlane);
  const SafetyValidator validator;
  const TraversabilityParams trav;
  const RobotParams robot;
  const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> segments = {
      // Mid-floor chord: placeable and clear.
      {Eigen::Vector3f(3.0f, 3.0f, 0.0f), Eigen::Vector3f(3.5f, 3.0f, 0.0f)},
      // Chord hugging the mesh boundary: clear < required_clearance.
      {Eigen::Vector3f(1.0f, 0.2f, 0.0f), Eigen::Vector3f(2.0f, 0.2f, 0.0f)},
      // Chord that leaves the surface.
      {Eigen::Vector3f(3.0f, 3.0f, 0.0f), Eigen::Vector3f(3.0f, 3.0f, 1.0f)}};

  auto batch = validator.validateSegments(segments, prepared.mesh, trav, 0.05f);
  ASSERT_TRUE(batch.ok()) << batch.status().message;
  ASSERT_EQ(batch.value().size(), segments.size());
  EXPECT_EQ(batch.value()[0], 1);
  EXPECT_EQ(batch.value()[1], 0);
  EXPECT_EQ(batch.value()[2], 0);

  for (std::size_t i = 0; i < segments.size(); ++i) {
    const std::vector<Eigen::Vector3f> single = {segments[i].first, segments[i].second};
    const PathValidationReport report =
        validator.validate(single, prepared.mesh, trav, robot, 0.05f);
    EXPECT_EQ(static_cast<bool>(batch.value()[i]), report.valid)
        << "segment " << i << " batch=" << static_cast<int>(batch.value()[i])
        << " validate=" << report.valid;
  }
}

}  // namespace
