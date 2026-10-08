#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
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
using topmap::PointId;
using topmap::Portal;
using topmap::PortalExtractor;
using topmap::PortalParams;
using topmap::PortalRejection;
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

// Two right triangles sharing the edge (0, 2) of a `side` x `side` square. The
// caller controls the R8 clearance field, the slope of both faces and the height
// jump between the two centroids, so every validation rule except width has an
// exact, geometry-backed value. Width is a pure region-union geometry quantity:
// the union is the whole square and the sampled free distance is side/3, so
// valid_width = 2 * side / 3 (independent of `clearance`).
Fixture MakeTwin(float clearance, float slope_deg, float z_gap, float side = 2.0f) {
  std::vector<Eigen::Vector3f> vertices = {{0.0f, 0.0f, 0.0f},
                                           {side, 0.0f, 0.0f},
                                           {side, side, 0.0f},
                                           {0.0f, side, 0.0f}};
  topmap::TriangleNode t0;
  t0.vertex_ids = {0, 1, 2};
  t0.centroid = Eigen::Vector3f(2.0f * side / 3.0f, 1.0f * side / 3.0f, 0.0f);
  t0.normal = Eigen::Vector3f::UnitZ();
  t0.slope = deg(slope_deg);
  t0.area = 0.5f * side * side;
  t0.min_clearance = clearance;
  topmap::TriangleNode t1;
  t1.vertex_ids = {0, 2, 3};
  t1.centroid = Eigen::Vector3f(1.0f * side / 3.0f, 2.0f * side / 3.0f, z_gap);
  t1.normal = Eigen::Vector3f::UnitZ();
  t1.slope = deg(slope_deg);
  t1.area = 0.5f * side * side;
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

struct RingFixture {
  TriangleMesh mesh;
  std::vector<SurfaceRegion> regions;
};

// An nx by ny grid of cells; Region A owns the interior block [ax0, ax1) x
// [ay0, ay1) and Region B owns every other cell. The A/B interface is a closed
// chain (no wall-touching endpoints), so the sampled region-union boundary
// distance is controlled purely by the placement of the block inside the outer
// grid. `clearance` seeds TriangleNode.min_clearance independently.
RingFixture MakeRing(int nx, int ny, float cell, int ax0, int ay0, int ax1, int ay1,
                     float clearance) {
  topmap::MeshData data;
  std::map<std::pair<int, int>, PointId> welded;
  const auto vertexAt = [&](int i, int j) -> PointId {
    const std::pair<int, int> key{i, j};
    const auto it = welded.find(key);
    if (it != welded.end()) return it->second;
    const PointId id = static_cast<PointId>(data.vertices.size());
    data.vertices.emplace_back(i * cell, j * cell, 0.0f);
    welded.emplace(key, id);
    return id;
  };
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const PointId a = vertexAt(i, j);
      const PointId b = vertexAt(i + 1, j);
      const PointId c = vertexAt(i + 1, j + 1);
      const PointId d = vertexAt(i, j + 1);
      data.faces.push_back({a, b, c});
      data.faces.push_back({a, c, d});
    }
  }
  auto mr = TriangleMesh::fromMeshData(data, {});
  EXPECT_TRUE(mr.ok()) << mr.status().message;
  RingFixture out;
  out.mesh = mr.take();
  for (topmap::TriangleNode& t : out.mesh.triangles()) t.min_clearance = clearance;

  SurfaceRegion a;
  a.id = 0;
  SurfaceRegion b;
  b.id = 1;
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const int t = 2 * (j * nx + i);
      const bool in_block = i >= ax0 && i < ax1 && j >= ay0 && j < ay1;
      SurfaceRegion& owner = in_block ? a : b;
      owner.triangle_ids.push_back(t);
      owner.triangle_ids.push_back(t + 1);
    }
  }
  out.regions = {a, b};
  return out;
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

  // Twin width is 2 * side / 3, so side 1.34 -> ~0.893 (< 0.90) and side 1.37
  // -> ~0.913 (> 0.90). The rule `valid_width > required_width()` is strict.
  const Fixture at = MakeTwin(0.50f, 0.0f, 0.0f, 1.34f);
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

  const Fixture above = MakeTwin(0.50f, 0.0f, 0.0f, 1.37f);
  const Portal p_above = OnePortal(above, params);
  auto accepted = PortalValidator().validate(above.mesh, above.regions, {p_above}, robot, params);
  ASSERT_TRUE(accepted.ok()) << accepted.status().message;
  ASSERT_EQ(accepted.value().size(), 1u);
  EXPECT_NEAR(accepted.value()[0].valid_width, 2.0f * 1.37f / 3.0f, 1e-3f);
  EXPECT_GT(accepted.value()[0].valid_width, robot.required_width());
  EXPECT_GE(accepted.value()[0].min_clearance, robot.required_clearance());
  EXPECT_GE(accepted.value()[0].traversal_cost, 1.0f);
  EXPECT_FALSE(accepted.value()[0].is_critical);
}

TEST(PortalValidatorTest, WidthRuleIsStrictAtTheSampledBoundary) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.0f, 2.0f);
  const Portal portal = OnePortal(fixture, params);
  auto base = PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, params);
  ASSERT_TRUE(base.ok()) << base.status().message;
  const float sampled = base.value()[0].valid_width;

  // Gate a hair above the sampled width: `valid_width > gate` is false.
  PortalParams at_gate = params;
  at_gate.width_margin = sampled - robot.required_width() + 1e-3f;
  EXPECT_FALSE(
      PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, at_gate).ok());

  // Gate a hair below: the same portal passes.
  PortalParams below_gate = params;
  below_gate.width_margin = sampled - robot.required_width() - 1e-2f;
  EXPECT_TRUE(
      PortalValidator().validate(fixture.mesh, fixture.regions, {portal}, robot, below_gate).ok());
}

TEST(PortalValidatorTest, WidthMarginRaisesThreshold) {
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.0f, 2.0f);  // valid_width 4/3
  const Portal portal = OnePortal(fixture, params);

  PortalParams margin = params;
  margin.width_margin = 0.50f;  // threshold becomes 1.40 > 4/3
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

TEST(PortalValidatorTest, ClearanceDoesNotFeedValidWidth) {
  // TriangleNode.min_clearance is the R8 footprint-erosion yardstick only; it is
  // deliberately independent from the region-union width sample.
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture high = MakeTwin(1.00f, 0.0f, 0.0f);
  const Fixture low = MakeTwin(0.30f, 0.0f, 0.0f);
  const Portal p_high = OnePortal(high, params);
  const Portal p_low = OnePortal(low, params);

  auto accepted = PortalValidator().validate(high.mesh, high.regions, {p_high}, robot, params);
  ASSERT_TRUE(accepted.ok()) << accepted.status().message;

  // The low-clearance mesh fails the default footprint gate...
  auto detail = PortalValidator().validateDetailed(low.mesh, low.regions, {p_low}, robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  EXPECT_NE(FindReason(detail.value(), PortalRejectionReason::kClearance), nullptr);

  // ...but its valid_width is identical once the clearance gate is relaxed.
  RobotParams relaxed = robot;
  relaxed.width = 0.10f;
  relaxed.safety_distance = 0.05f;  // required_clearance 0.10, required_width 0.20
  auto low_accepted =
      PortalValidator().validate(low.mesh, low.regions, {p_low}, relaxed, params);
  ASSERT_TRUE(low_accepted.ok()) << low_accepted.status().message;
  EXPECT_FLOAT_EQ(low_accepted.value()[0].valid_width, accepted.value()[0].valid_width);
}

TEST(PortalValidatorTest, ValidWidthFollowsUnionBoundaryGeometry) {
  // valid_width is a region-union boundary-distance quantity: growing the union
  // square grows the sampled free distance linearly.
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const Fixture small = MakeTwin(0.50f, 0.0f, 0.0f, 2.0f);
  const Fixture large = MakeTwin(0.50f, 0.0f, 0.0f, 3.0f);
  const Portal p_small = OnePortal(small, params);
  const Portal p_large = OnePortal(large, params);
  auto a = PortalValidator().validate(small.mesh, small.regions, {p_small}, robot, params);
  auto b = PortalValidator().validate(large.mesh, large.regions, {p_large}, robot, params);
  ASSERT_TRUE(a.ok()) << a.status().message;
  ASSERT_TRUE(b.ok()) << b.status().message;
  EXPECT_NEAR(a.value()[0].valid_width, 4.0f / 3.0f, 1e-3f);
  EXPECT_NEAR(b.value()[0].valid_width, 2.0f, 1e-3f);
  EXPECT_GT(b.value()[0].valid_width, a.value()[0].valid_width);
}

TEST(PortalValidatorTest, NarrowestChainSampleDeterminesValidWidth) {
  // The ring chain is a closed loop whose samples have different free widths.
  // valid_width must be the minimum over the chain, not the maximum.
  RobotParams robot;
  PortalParams params;
  params.boundary_min_edges = 1;
  const RingFixture ring = MakeRing(8, 6, 1.0f, 2, 1, 6, 5, 1.00f);
  auto extracted = PortalExtractor().extract(ring.mesh, ring.regions, params);
  ASSERT_TRUE(extracted.ok()) << extracted.status().message;
  ASSERT_EQ(extracted.value().size(), 1u);
  const Portal candidate = extracted.value().front();

  std::vector<topmap::TriangleId> uni = ring.regions[0].triangle_ids;
  uni.insert(uni.end(), ring.regions[1].triangle_ids.begin(), ring.regions[1].triangle_ids.end());
  const std::vector<float> distance = topmap::regionBoundaryDistance(ring.mesh, uni);
  std::set<std::pair<PointId, PointId>> chain_edges;
  for (std::size_t k = 1; k < candidate.boundary_chain.size(); ++k) {
    const PointId x = candidate.boundary_chain[k - 1];
    const PointId y = candidate.boundary_chain[k];
    chain_edges.insert({std::min(x, y), std::max(x, y)});
  }
  float sampled_min = std::numeric_limits<float>::infinity();
  float sampled_max = -1.0f;
  for (topmap::TriangleId t = 0; t < static_cast<topmap::TriangleId>(ring.mesh.size()); ++t) {
    const bool t_in_a =
        std::find(ring.regions[0].triangle_ids.begin(), ring.regions[0].triangle_ids.end(), t) !=
        ring.regions[0].triangle_ids.end();
    if (!t_in_a) continue;
    for (const topmap::TriangleEdge& e : ring.mesh.triangle(t).edges) {
      const topmap::TriangleId u = e.neighbor;
      if (u == topmap::kInvalidId) continue;
      const bool u_in_b =
          std::find(ring.regions[1].triangle_ids.begin(), ring.regions[1].triangle_ids.end(), u) !=
          ring.regions[1].triangle_ids.end();
      if (!u_in_b) continue;
      if (chain_edges.count({std::min(e.v0, e.v1), std::max(e.v0, e.v1)}) == 0) continue;
      const float local = std::min(2.0f * distance[t], 2.0f * distance[u]);
      sampled_min = std::min(sampled_min, local);
      sampled_max = std::max(sampled_max, local);
    }
  }
  ASSERT_LT(sampled_min, sampled_max) << "fixture must expose a narrow and a wide chain sample";

  auto validated =
      PortalValidator().validate(ring.mesh, ring.regions, extracted.value(), robot, params);
  ASSERT_TRUE(validated.ok()) << validated.status().message;
  ASSERT_EQ(validated.value().size(), 1u);
  EXPECT_NEAR(validated.value()[0].valid_width, sampled_min, 1e-4f);
  EXPECT_LT(validated.value()[0].valid_width, sampled_max);
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

  // valid_width 4/3 > 0.90 (width passes) but min_clearance 0.44 < 0.45.
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

TEST(PortalValidatorTest, FrozenScenesPinExtractionCountsAndUnionWidthOutcome) {
  // R15 freezes the extracted portal counts. Under the plan's region-union width
  // rule the sampled center-band width is below the 0.90 gate for these scenes:
  // T14 keeps only robot-placeable triangles in a Region, so the union boundary
  // sits on the required_clearance contour rather than on the mesh wall. This
  // test pins the extraction contract and the conservative validation outcome;
  // the implementation never special-cases a scene name or coordinate.
  const std::vector<std::pair<SyntheticScene, std::size_t>> frozen = {
      {SyntheticScene::kHallNarrowHall, 2u},
      {SyntheticScene::kSingleBridge, 2u},
      {SyntheticScene::kParallelPortals, 4u},
      {SyntheticScene::kLoop, 8u}};
  RobotParams robot;
  PortalParams params;
  for (const auto& entry : frozen) {
    const Prepared prepared = Split(entry.first, RegionParams{});
    ASSERT_TRUE(prepared.ok) << topmap::toString(entry.first) << ": " << prepared.error;
    auto extracted = PortalExtractor().extract(prepared.mesh, prepared.regions, params);
    ASSERT_TRUE(extracted.ok()) << topmap::toString(entry.first) << ": "
                                << extracted.status().message;
    EXPECT_EQ(extracted.value().size(), entry.second) << topmap::toString(entry.first);

    auto detail = PortalValidator().validateDetailed(prepared.mesh, prepared.regions,
                                                     extracted.value(), robot, params);
    ASSERT_TRUE(detail.ok()) << topmap::toString(entry.first) << ": " << detail.status().message;
    std::set<topmap::PortalId> seen;
    for (const Portal& portal : detail.value().portals) seen.insert(portal.id);
    for (const PortalRejection& rejection : detail.value().rejections) seen.insert(rejection.id);
    EXPECT_EQ(seen.size(), extracted.value().size())
        << topmap::toString(entry.first) << ": a candidate was silently dropped";
    EXPECT_TRUE(detail.value().portals.empty())
        << topmap::toString(entry.first) << ": union-width rule should reject these candidates";
  }
}

TEST(PortalValidatorTest, ParallelPortalsAreNotMergedByValidation) {
  RobotParams robot;
  PortalParams params;
  const Prepared prepared = Split(SyntheticScene::kParallelPortals, RegionParams{});
  ASSERT_TRUE(prepared.ok) << prepared.error;
  auto extracted = PortalExtractor().extract(prepared.mesh, prepared.regions, params);
  ASSERT_TRUE(extracted.ok()) << extracted.status().message;
  ASSERT_EQ(extracted.value().size(), 4u);

  auto detail = PortalValidator().validateDetailed(prepared.mesh, prepared.regions,
                                                   extracted.value(), robot, params);
  ASSERT_TRUE(detail.ok()) << detail.status().message;
  std::set<topmap::PortalId> ids;
  for (const Portal& portal : detail.value().portals) ids.insert(portal.id);
  for (const PortalRejection& rejection : detail.value().rejections) ids.insert(rejection.id);
  EXPECT_EQ(ids.size(), 4u);  // parallel portals stay distinct; merging is a T17 concern
}

TEST(PortalValidatorTest, PortalLocalWidthMatchesRegionBoundaryDistance) {
  const Fixture fixture = MakeTwin(0.50f, 0.0f, 0.0f);
  const std::vector<topmap::TriangleId> region_union = {0, 1};
  const float width = topmap::portalLocalWidth(fixture.mesh, region_union, 0);
  const std::vector<float> distance =
      topmap::regionBoundaryDistance(fixture.mesh, region_union);
  EXPECT_NEAR(width, 2.0f * distance[0], 1e-5f);
}
