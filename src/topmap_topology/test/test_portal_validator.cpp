#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_surface/mesh_types.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/portal_extractor.hpp"
#include "topmap_topology/portal_validator.hpp"
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"

namespace {

using topmap::ErrorCode;
using topmap::Portal;
using topmap::PortalExtractor;
using topmap::PortalParams;
using topmap::PortalRejectionReason;
using topmap::PortalValidationReport;
using topmap::PortalValidator;
using topmap::RegionParams;
using topmap::RobotParams;
using topmap::SurfaceRegion;
using topmap::SyntheticScene;
using topmap::TriangleMesh;

constexpr float kPi = 3.14159265358979323846f;

float deg(float value) { return value * kPi / 180.0f; }

struct Fixture {
  TriangleMesh mesh;
  std::vector<SurfaceRegion> regions;
};

// Two right triangles sharing the edge (0, 2). The caller controls the R8
// clearance field, the slope of both faces and the height jump between the two
// centroids, so every validation rule has an exact, geometry-backed value.
Fixture MakeTwin(float clearance, float slope_deg, float z_gap) {
  std::vector<Eigen::Vector3f> vertices = {{0.0f, 0.0f, 0.0f},
                                           {1.0f, 0.0f, 0.0f},
                                           {1.0f, 1.0f, 0.0f},
                                           {0.0f, 1.0f, 0.0f}};
  topmap::TriangleNode t0;
  t0.vertex_ids = {0, 1, 2};
  t0.centroid = Eigen::Vector3f(2.0f / 3.0f, 1.0f / 3.0f, 0.0f);
  t0.normal = Eigen::Vector3f::UnitZ();
  t0.slope = deg(slope_deg);
  t0.area = 0.5f;
  t0.min_clearance = clearance;
  topmap::TriangleNode t1;
  t1.vertex_ids = {0, 2, 3};
  t1.centroid = Eigen::Vector3f(1.0f / 3.0f, 2.0f / 3.0f, z_gap);
  t1.normal = Eigen::Vector3f::UnitZ();
  t1.slope = deg(slope_deg);
  t1.area = 0.5f;
  t1.min_clearance = clearance;

  auto mr = TriangleMesh::fromTriangleNodes(vertices, {t0, t1});
  EXPECT_TRUE(mr.ok()) << mr.status().message;
  Fixture fixture;
  fixture.mesh = mr.take();
  SurfaceRegion a;
  a.id = 0;
  a.triangle_ids = {0};
  SurfaceRegion b;
  b.id = 1;
  b.triangle_ids = {1};
  fixture.regions = {a, b};
  return fixture;
}

Portal OnePortal(const Fixture& fixture, const PortalParams& params) {
  auto r = PortalExtractor().extract(fixture.mesh, fixture.regions, params);
  EXPECT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), 1u);
  return r.value().empty() ? Portal{} : r.value().front();
}

struct Prepared {
  bool ok = false;
  std::string error;
  TriangleMesh mesh;
  std::vector<SurfaceRegion> regions;
};

Prepared Split(SyntheticScene scene, const RegionParams& params) {
  Prepared out;
  auto d = topmap::makeSyntheticMesh(scene);
  if (!d.ok()) {
    out.error = "makeSyntheticMesh: " + d.status().message;
    return out;
  }
  auto mr = TriangleMesh::fromMeshData(d.value(), {});
  if (!mr.ok()) {
    out.error = "fromMeshData: " + mr.status().message;
    return out;
  }
  out.mesh = mr.take();
  topmap::TraversabilityEvaluator::apply(out.mesh, topmap::TraversabilityParams{});
  auto g = topmap::RegionGrower().grow(out.mesh, params);
  if (!g.ok()) {
    out.error = "grow: " + g.status().message;
    return out;
  }
  auto sp = topmap::RegionSplitter().split(out.mesh, g.value(), params);
  if (!sp.ok()) {
    out.error = "split: " + sp.status().message;
    return out;
  }
  out.regions = sp.take();
  out.ok = true;
  return out;
}

const PortalRejectionReason* FindReason(const PortalValidationReport& report,
                                        PortalRejectionReason reason) {
  for (const auto& rejection : report.rejections) {
    if (rejection.reason == reason) return &rejection.reason;
  }
  return nullptr;
}

}  // namespace

TEST(PortalValidatorTest, WidthStrictlyAboveRequiredWidth) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  EXPECT_NEAR(robot.required_width(), 0.90f, 1e-5f);

  // clearance 0.45 => valid_width 0.90, which must NOT satisfy "width >
  // required_width()" (the rule is strict).
  const Fixture at = MakeTwin(0.45f, 0.0f, 0.0f);
  const Portal p_at = OnePortal(at, params);
  auto rejected = PortalValidator().validate(at.mesh, at.regions, {p_at}, robot, params);
  EXPECT_FALSE(rejected.ok());
  EXPECT_EQ(rejected.status().code, ErrorCode::kPortalValidationFailed);
  auto detail = PortalValidator().validateDetailed(at.mesh, at.regions, {p_at}, robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  EXPECT_TRUE(detail.value().portals.empty());
  const PortalRejectionReason* narrow = FindReason(detail.value(), PortalRejectionReason::kTooNarrow);
  ASSERT_NE(narrow, nullptr);
  bool has_narrow_code = false;
  for (const auto& rejection : detail.value().rejections) {
    if (rejection.reason == PortalRejectionReason::kTooNarrow) {
      has_narrow_code = rejection.code == ErrorCode::kPortalTooNarrow;
      EXPECT_FALSE(rejection.detail.empty());
    }
  }
  EXPECT_TRUE(has_narrow_code);

  // clearance 0.46 => valid_width 0.92 > 0.90, accepted.
  const Fixture above = MakeTwin(0.46f, 0.0f, 0.0f);
  const Portal p_above = OnePortal(above, params);
  auto accepted = PortalValidator().validate(above.mesh, above.regions, {p_above}, robot, params);
  ASSERT_TRUE(accepted.ok()) << accepted.status().message;
  ASSERT_EQ(accepted.value().size(), 1u);
  EXPECT_NEAR(accepted.value()[0].valid_width, 0.92f, 1e-4f);
  EXPECT_GT(accepted.value()[0].valid_width, robot.required_width());
  EXPECT_GE(accepted.value()[0].min_clearance, robot.required_clearance());
  EXPECT_GE(accepted.value()[0].traversal_cost, 1.0f);
  EXPECT_FALSE(accepted.value()[0].is_critical);
}

TEST(PortalValidatorTest, WidthMarginRaisesThreshold) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.46f, 0.0f, 0.0f);  // valid_width 0.92
  const Portal portal = OnePortal(fixture, params);

  PortalParams margin = params;
  margin.width_margin = 0.03f;  // threshold becomes 0.93
  EXPECT_FALSE(
      PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, margin).ok());
  auto detail =
      PortalValidator().validateDetailed(fixture.mesh, fixture.regions, {portal}, robot, margin);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  EXPECT_NE(FindReason(detail.value(), PortalRejectionReason::kTooNarrow), nullptr);

  // With the default margin the same portal passes.
  auto accepted = PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, params);
  EXPECT_TRUE(accepted.ok()) << accepted.status().message;
}

TEST(PortalValidatorTest, StepRejects) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.10f);  // 0.10 > max_step_height 0.05
  const Portal portal = OnePortal(fixture, params);
  auto result = PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, params);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kPortalValidationFailed);

  auto detail =
      PortalValidator().validateDetailed(fixture.mesh, fixture.regions, {portal}, robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  ASSERT_EQ(detail.value().rejections.size(), 1u);
  EXPECT_EQ(detail.value().rejections[0].reason, PortalRejectionReason::kStep);
}

TEST(PortalValidatorTest, SlopeRejects) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.50f, 30.0f, 0.0f);  // 30 deg > max_slope 20 deg
  const Portal portal = OnePortal(fixture, params);
  auto result = PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, params);
  EXPECT_FALSE(result.ok());

  auto detail =
      PortalValidator().validateDetailed(fixture.mesh, fixture.regions, {portal}, robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  ASSERT_EQ(detail.value().rejections.size(), 1u);
  EXPECT_EQ(detail.value().rejections[0].reason, PortalRejectionReason::kSlope);
}

TEST(PortalValidatorTest, ClearanceUsesSingleRequiredClearance) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  EXPECT_FLOAT_EQ(robot.required_clearance(), 0.45f) << "float identity of the derived clearance";

  // valid_width 0.88 < 0.90 and min_clearance 0.44 < required_clearance 0.45.
  const Fixture fixture = MakeTwin(0.44f, 0.0f, 0.0f);
  const Portal portal = OnePortal(fixture, params);
  auto result = PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, params);
  EXPECT_FALSE(result.ok());

  auto detail =
      PortalValidator().validateDetailed(fixture.mesh, fixture.regions, {portal}, robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  EXPECT_NE(FindReason(detail.value(), PortalRejectionReason::kClearance), nullptr);
}

TEST(PortalValidatorTest, RejectsEmptyAndInvalidInputs) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.0f);
  const Portal portal = OnePortal(fixture, params);

  const TriangleMesh empty;
  auto no_mesh = PortalValidator().validate(empty, fixture.regions, {portal}, robot, params);
  EXPECT_FALSE(no_mesh.ok());
  EXPECT_EQ(no_mesh.status().code, ErrorCode::kMeshEmpty);

  auto no_regions = PortalValidator().validate(fixture.mesh, {}, {portal}, robot, params);
  EXPECT_FALSE(no_regions.ok());
  EXPECT_EQ(no_regions.status().code, ErrorCode::kRegionEmpty);

  auto no_candidates = PortalValidator().validate(fixture.mesh, fixture.regions, {}, robot, params);
  EXPECT_FALSE(no_candidates.ok());
  EXPECT_EQ(no_candidates.status().code, ErrorCode::kInvalidArgument);

  Portal bad = portal;
  bad.region_a = 99;
  auto bad_region = PortalValidator().validate(fixture.mesh, fixture.regions, {bad}, robot, params);
  EXPECT_FALSE(bad_region.ok());
  EXPECT_EQ(bad_region.status().code, ErrorCode::kInvalidArgument);
}

TEST(PortalValidatorTest, ChainThatDoesNotMatchTheInterfaceIsRejected) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.0f);
  Portal bogus = OnePortal(fixture, params);
  // The shared edge is (0, 2); (0, 1) is a boundary edge of triangle 0 only.
  bogus.boundary_chain = {0, 1};

  auto detail =
      PortalValidator().validateDetailed(fixture.mesh, fixture.regions, {bogus}, robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  EXPECT_TRUE(detail.value().portals.empty());
  ASSERT_EQ(detail.value().rejections.size(), 1u);
  EXPECT_EQ(detail.value().rejections[0].reason, PortalRejectionReason::kInvalidChain);
}

TEST(PortalValidatorTest, RealScenesPassUnderTheDefaultRobot) {
  RobotParams robot;
  PortalParams params;
  for (SyntheticScene scene :
       {SyntheticScene::kHallNarrowHall, SyntheticScene::kSingleBridge,
        SyntheticScene::kParallelPortals, SyntheticScene::kLoop}) {
    const Prepared prepared = Split(scene, RegionParams{});
    ASSERT_TRUE(prepared.ok) << topmap::toString(scene) << ": " << prepared.error;
    auto extracted = PortalExtractor().extract(prepared.mesh, prepared.regions, params);
    ASSERT_TRUE(extracted.ok()) << topmap::toString(scene) << ": " << extracted.status().message;
    ASSERT_GT(extracted.value().size(), 0u);

    auto validated = PortalValidator().validate(prepared.mesh, prepared.regions,
                                                extracted.value(), robot, params);
    ASSERT_TRUE(validated.ok()) << topmap::toString(scene) << ": " << validated.status().message;
    EXPECT_EQ(validated.value().size(), extracted.value().size())
        << topmap::toString(scene) << ": valid portals must not be dropped or merged";

    bool recomputed_from_mesh = false;
    for (const Portal& portal : validated.value()) {
      EXPECT_GT(portal.valid_width, robot.required_width());
      EXPECT_LE(portal.mean_slope, robot.max_slope_rad());
      EXPECT_LE(portal.max_step, robot.max_step_height);
      EXPECT_GE(portal.min_clearance, robot.required_clearance());
      EXPECT_GE(portal.traversal_cost, 1.0f);
      EXPECT_FALSE(portal.is_critical);
      if (std::fabs(portal.raw_width - portal.valid_width) > 1e-3f) recomputed_from_mesh = true;
    }
    // valid_width is a mesh-clearance quantity, not the raw boundary-chain arc
    // length (for kLoop the chain is only ~0.4 m while the passage is ~1.3 m).
    EXPECT_TRUE(recomputed_from_mesh)
        << topmap::toString(scene) << ": valid_width was copied from the chain length";
  }
}

TEST(PortalValidatorTest, ParallelPortalsArePreserved) {
  RobotParams robot;
  PortalParams params;
  const Prepared prepared = Split(SyntheticScene::kParallelPortals, RegionParams{});
  ASSERT_TRUE(prepared.ok) << prepared.error;
  auto extracted = PortalExtractor().extract(prepared.mesh, prepared.regions, params);
  ASSERT_TRUE(extracted.ok()) << extracted.status().message;
  ASSERT_EQ(extracted.value().size(), 4u);

  auto validated =
      PortalValidator().validate(prepared.mesh, prepared.regions, extracted.value(), robot, params);
  ASSERT_TRUE(validated.ok()) << validated.status().message;
  ASSERT_EQ(validated.value().size(), 4u);
  std::set<topmap::PortalId> ids;
  for (const Portal& portal : validated.value()) ids.insert(portal.id);
  EXPECT_EQ(ids.size(), 4u);
}

TEST(PortalValidatorTest, PortalLocalWidthMatchesRegionBoundaryDistance) {
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.0f);
  const std::vector<topmap::TriangleId> region_union = {0, 1};
  const float width = topmap::portalLocalWidth(fixture.mesh, region_union, 0);
  const std::vector<float> distance =
      topmap::regionBoundaryDistance(fixture.mesh, region_union);
  EXPECT_NEAR(width, 2.0f * distance[0], 1e-5f);
}
