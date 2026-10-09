// T25: tests for PathOptimizer (shortcut / funnel / curvature-limited smooth).
//
// All flat-plane probes use interior points with real clearance (never the
// x=0 / y=0 mesh boundary), so the tests exercise the R10 placeability gate
// instead of relaxing it. The overlapping-floors case proves that a shortcut
// can never jump between mesh components / layers.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "test_support_plan.hpp"
#include "topmap_core/geometry.hpp"
#include "topmap_planner/path_optimizer.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"

namespace {

using topmap::Corridor;
using topmap::ErrorCode;
using topmap::PathOptimizer;
using topmap::PlanParams;
using topmap::Portal;
using topmap::RobotParams;
using topmap::TraversabilityEvaluator;
using topmap::TraversabilityParams;
using topmap::TriangleId;
using topmap::TriangleMesh;

// Builds only the surface stage (mesh + traversability) because PathOptimizer
// does not consume the Region/Portal graph; this keeps the tests focused.
topmap::Result<TriangleMesh> BuildMesh(topmap::SyntheticScene scene) {
  auto data = topmap::makeSyntheticMesh(scene);
  if (!data.ok()) return data.status();
  auto mesh = TriangleMesh::fromMeshData(data.value(), {});
  if (!mesh.ok()) return mesh.status();
  TraversabilityEvaluator::apply(mesh.value(), TraversabilityParams{});
  return mesh.take();
}

Corridor MakeCorridor(const TriangleMesh& mesh) {
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(mesh);
  return corridor;
}

Portal MakePortal(const Eigen::Vector3f& left, const Eigen::Vector3f& right) {
  Portal portal;
  portal.left = left;
  portal.right = right;
  portal.center = 0.5f * (left + right);
  return portal;
}

float CurvatureAt(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f ab = a - b, cb = c - b;
  const float lab = ab.norm(), lcb = cb.norm(), lac = (c - a).norm();
  if (lab < 1e-6f || lcb < 1e-6f || lac < 1e-6f) return 0.0f;
  return 2.0f * ab.cross(cb).norm() / (lab * lcb * lac);
}

float MaxCurvature(const std::vector<Eigen::Vector3f>& pts) {
  float maximum = 0.0f;
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    maximum = std::max(maximum, CurvatureAt(pts[i - 1], pts[i], pts[i + 1]));
  }
  return maximum;
}

// Nearest mesh triangle (all components) to a point, for component proofs.
TriangleId NearestTriangle(const TriangleMesh& mesh, const Eigen::Vector3f& p) {
  TriangleId best = topmap::kInvalidId;
  float best_distance = std::numeric_limits<float>::infinity();
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const topmap::TriangleNode& t = mesh.triangle(id);
    const float distance =
        topmap::pointTriangleDistance(p, mesh.vertices()[t.vertex_ids[0]],
                                      mesh.vertices()[t.vertex_ids[1]],
                                      mesh.vertices()[t.vertex_ids[2]]);
    if (best == topmap::kInvalidId || distance < best_distance) {
      best = id;
      best_distance = distance;
    }
  }
  return best;
}

constexpr float kFlatZ = 0.0f;

}  // namespace

TEST(PathOptimizerTest, ShortcutRemovesCollinearPoints) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const PlanParams params;
  const TraversabilityParams trav;
  std::vector<Eigen::Vector3f> pts = {{2.0f, 2.0f, kFlatZ},
                                      {2.1f, 2.0f, kFlatZ},
                                      {2.2f, 2.0f, kFlatZ},
                                      {2.3f, 2.0f, kFlatZ}};
  std::size_t removed = 0;
  const auto out = PathOptimizer().shortcut(pts, mesh, corridor, params, trav, &removed);
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(removed, 2u);
  EXPECT_TRUE(out.front().isApprox(pts.front()));
  EXPECT_TRUE(out.back().isApprox(pts.back()));
}

// The straight p0->p3 chord cuts across the rectangular hole of the U scene
// (non-mesh) / its low-clearance boundary, so only the safe p0->p2 shortcut is
// allowed and exactly one interior point is removed.
TEST(PathOptimizerTest, ShortcutRejectsSegmentCrossingHoleAndLowClearance) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kUShape);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const PlanParams params;
  const TraversabilityParams trav;
  const std::vector<Eigen::Vector3f> pts = {{2.0f, 0.5f, kFlatZ},
                                            {1.25f, 0.5f, kFlatZ},
                                            {0.5f, 0.5f, kFlatZ},
                                            {0.5f, 2.0f, kFlatZ}};
  std::size_t removed = 0;
  const auto out = PathOptimizer().shortcut(pts, mesh, corridor, params, trav, &removed);
  EXPECT_EQ(removed, 1u);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_TRUE(out[0].isApprox(pts[0]));
  EXPECT_TRUE(out[1].isApprox(pts[2]));
  EXPECT_TRUE(out[2].isApprox(pts[3]));
}

// Overlapping floors: the direct chord p0(low)->p2(high) is metres away from any
// triangle and joins two different mesh components, so no shortcut may be taken.
TEST(PathOptimizerTest, ShortcutRejectsCrossComponentShortcut) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const PlanParams params;
  const TraversabilityParams trav;
  const std::vector<Eigen::Vector3f> pts = {{2.5f, 2.5f, 0.0f},
                                            {3.0f, 2.5f, 0.0f},
                                            {2.5f, 2.5f, 3.0f}};

  const TriangleId lower = NearestTriangle(mesh, pts.front());
  const TriangleId upper = NearestTriangle(mesh, pts.back());
  ASSERT_NE(lower, topmap::kInvalidId);
  ASSERT_NE(upper, topmap::kInvalidId);
  EXPECT_NE(mesh.componentId(lower), mesh.componentId(upper));

  std::size_t removed = 7;  // must be overwritten
  const auto out = PathOptimizer().shortcut(pts, mesh, corridor, params, trav, &removed);
  EXPECT_EQ(removed, 0u);
  ASSERT_EQ(out.size(), pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) EXPECT_TRUE(out[i].isApprox(pts[i]));
}

TEST(PathOptimizerTest, ShortcutInvalidInputReturnsOriginalWithZeroRemoved) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const TraversabilityParams trav;
  const std::vector<Eigen::Vector3f> pts = {{2.0f, 2.0f, kFlatZ},
                                            {2.1f, 2.0f, kFlatZ},
                                            {2.2f, 2.0f, kFlatZ}};

  PlanParams bad_step;
  bad_step.check_step = 0.0f;
  std::size_t removed = 3;
  auto zero_step = PathOptimizer().shortcut(pts, mesh, corridor, bad_step, trav, &removed);
  EXPECT_EQ(removed, 0u);
  ASSERT_EQ(zero_step.size(), pts.size());

  const std::vector<Eigen::Vector3f> non_finite = {pts[0], {2.1f, 2.0f, std::nanf("")},
                                                   pts[2]};
  removed = 9;
  const PlanParams params;
  auto nan = PathOptimizer().shortcut(non_finite, mesh, corridor, params, trav, &removed);
  EXPECT_EQ(removed, 0u);
  ASSERT_EQ(nan.size(), non_finite.size());
}

TEST(PathOptimizerTest, FunnelTurnsAtInnerCorner) {
  const std::vector<Eigen::Vector3f> pts = {{0.0f, 0.0f, 0.0f}, {2.0f, 3.0f, 0.0f}};
  const std::vector<Portal> portals = {MakePortal({1.0f, 1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}),
                                       MakePortal({1.0f, 1.0f, 0.0f}, {3.0f, 1.0f, 0.0f})};
  const auto out = PathOptimizer().funnel(pts, portals);
  ASSERT_GE(out.size(), 2u);
  EXPECT_EQ(out.size(), 3u);
  EXPECT_TRUE(out.front().isApprox(pts.front()));
  EXPECT_TRUE(out.back().isApprox(pts.back()));
  EXPECT_NEAR((out[1] - Eigen::Vector3f(1.0f, 1.0f, 0.0f)).norm(), 0.0f, 1e-5f);
}

// Same geometry, but every portal stores its endpoints reversed relative to the
// direction of travel. Funnel must derive the sides from the forward direction.
TEST(PathOptimizerTest, FunnelHandlesReversedLeftRight) {
  const std::vector<Eigen::Vector3f> pts = {{0.0f, 0.0f, 0.0f}, {2.0f, 3.0f, 0.0f}};
  const std::vector<Portal> portals = {MakePortal({1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}),
                                       MakePortal({3.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 0.0f})};
  const auto out = PathOptimizer().funnel(pts, portals);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_NEAR((out[1] - Eigen::Vector3f(1.0f, 1.0f, 0.0f)).norm(), 0.0f, 1e-5f);
}

TEST(PathOptimizerTest, FunnelEmptyPortalsIsDeterministicNoOp) {
  const std::vector<Eigen::Vector3f> pts = {{1.0f, 1.0f, 0.0f}, {2.0f, 2.0f, 0.0f},
                                            {3.0f, 3.0f, 0.0f}};
  const auto first = PathOptimizer().funnel(pts, {});
  const auto second = PathOptimizer().funnel(pts, {});
  ASSERT_EQ(first.size(), pts.size());
  ASSERT_EQ(second.size(), pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_TRUE(first[i].isApprox(pts[i]));
    EXPECT_TRUE(first[i].isApprox(second[i]));
  }
}

TEST(PathOptimizerTest, FunnelHandlesDegeneratePortalAndPreservesZ) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  (void)mesh_result;

  // A fully degenerate (zero-width) portal must not crash and must still yield
  // an ordered path of at least two points.
  const std::vector<Eigen::Vector3f> pts = {{1.5f, 1.5f, 0.0f}, {3.5f, 3.5f, 0.0f}};
  const std::vector<Portal> degenerate = {MakePortal({2.5f, 2.5f, 0.0f}, {2.5f, 2.5f, 0.0f})};
  const auto out = PathOptimizer().funnel(pts, degenerate);
  ASSERT_GE(out.size(), 2u);
  EXPECT_TRUE(out.front().isApprox(pts.front()));
  EXPECT_TRUE(out.back().isApprox(pts.back()));

  // 3D z must survive: the emitted corner carries the portal endpoint's z.
  const std::vector<Eigen::Vector3f> slanted = {{0.0f, 0.0f, 0.0f}, {2.0f, 3.0f, 0.7f}};
  const std::vector<Portal> portals = {MakePortal({1.0f, 1.0f, 0.7f}, {1.0f, -1.0f, 0.7f}),
                                       MakePortal({1.0f, 1.0f, 0.7f}, {3.0f, 1.0f, 0.7f})};
  const auto corner = PathOptimizer().funnel(slanted, portals);
  ASSERT_GE(corner.size(), 2u);
  bool has_z_corner = false;
  for (const Eigen::Vector3f& p : corner) {
    if (std::fabs(p.z() - 0.7f) < 1e-5f && std::fabs(p.x() - 1.0f) < 1e-5f &&
        std::fabs(p.y() - 1.0f) < 1e-5f) {
      has_z_corner = true;
    }
  }
  EXPECT_TRUE(has_z_corner);
}

TEST(PathOptimizerTest, SmoothKeepsEndpointsAndBoundsCurvature) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.smoothing_iterations = 10;
  const std::vector<Eigen::Vector3f> pts = {{2.0f, 2.0f, kFlatZ},
                                            {3.0f, 2.2f, kFlatZ},
                                            {4.0f, 2.0f, kFlatZ}};
  ASSERT_GT(MaxCurvature(pts), robot.max_path_curvature);

  const auto out = PathOptimizer().smooth(pts, mesh, params, robot, trav);
  ASSERT_EQ(out.size(), pts.size());
  EXPECT_TRUE(out.front().isApprox(pts.front()));
  EXPECT_TRUE(out.back().isApprox(pts.back()));
  EXPECT_LE(MaxCurvature(out), robot.max_path_curvature + 1e-5f);
  EXPECT_LT(out[1].y(), pts[1].y());  // the corner really moved inward
}

// A sharp right angle cannot be made feasible by moving the single interior
// point, so the candidate is reverted (not deleted) instead of hiding the
// violation by dropping points.
TEST(PathOptimizerTest, SmoothRevertsCandidateThatViolatesCurvature) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.smoothing_iterations = 10;
  const std::vector<Eigen::Vector3f> pts = {{2.0f, 2.0f, kFlatZ},
                                            {3.0f, 3.0f, kFlatZ},
                                            {4.0f, 2.0f, kFlatZ}};
  const auto out = PathOptimizer().smooth(pts, mesh, params, robot, trav);
  ASSERT_EQ(out.size(), pts.size());
  EXPECT_TRUE(out[1].isApprox(pts[1]));
}

TEST(PathOptimizerTest, SmoothNegativeIterationsReturnsInput) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.smoothing_iterations = -3;
  const std::vector<Eigen::Vector3f> pts = {{2.0f, 2.0f, kFlatZ},
                                            {3.0f, 2.2f, kFlatZ},
                                            {4.0f, 2.0f, kFlatZ}};
  const auto out = PathOptimizer().smooth(pts, mesh, params, robot, trav);
  ASSERT_EQ(out.size(), pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) EXPECT_TRUE(out[i].isApprox(pts[i]));
}

TEST(PathOptimizerTest, OptimizeAllSwitchesOffReturnsSafeInput) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.0f, kFlatZ},
                                            {2.5f, 2.0f, kFlatZ},
                                            {3.0f, 2.0f, kFlatZ}};
  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_EQ(result.value().size(), raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) EXPECT_TRUE(result.value()[i].isApprox(raw[i]));
}

TEST(PathOptimizerTest, OptimizeRejectsUnsafeCurvatureWhenSmoothingDisabled) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.0f, kFlatZ},
                                            {3.0f, 3.0f, kFlatZ},
                                            {4.0f, 2.0f, kFlatZ}};
  ASSERT_GT(MaxCurvature(raw), robot.max_path_curvature);

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kPathOptimizationFailed);
}

TEST(PathOptimizerTest, OptimizeAppliesSmoothingAndFinalValidation) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = true;
  params.smoothing_iterations = 10;
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.0f, kFlatZ},
                                            {3.0f, 2.2f, kFlatZ},
                                            {4.0f, 2.0f, kFlatZ}};
  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_EQ(result.value().size(), raw.size());
  EXPECT_TRUE(result.value().front().isApprox(raw.front()));
  EXPECT_TRUE(result.value().back().isApprox(raw.back()));
  EXPECT_LE(MaxCurvature(result.value()), robot.max_path_curvature + 1e-5f);
}

TEST(PathOptimizerTest, OptimizeRejectsInvalidParametersAndNonFinitePoints) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const PlanParams good_params;
  const RobotParams good_robot;
  const TraversabilityParams trav;
  const std::vector<Eigen::Vector3f> safe = {{2.0f, 2.0f, kFlatZ}, {2.5f, 2.0f, kFlatZ}};

  const auto code = [&](const PlanParams& params, const RobotParams& robot,
                        const std::vector<Eigen::Vector3f>& raw) {
    return PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav).status().code;
  };

  PlanParams zero_step = good_params;
  zero_step.check_step = 0.0f;
  EXPECT_EQ(code(zero_step, good_robot, safe), ErrorCode::kInvalidArgument);

  PlanParams nan_step = good_params;
  nan_step.check_step = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(code(nan_step, good_robot, safe), ErrorCode::kInvalidArgument);

  PlanParams negative_iterations = good_params;
  negative_iterations.smoothing_iterations = -1;
  EXPECT_EQ(code(negative_iterations, good_robot, safe), ErrorCode::kInvalidArgument);

  RobotParams zero_curvature = good_robot;
  zero_curvature.max_path_curvature = 0.0f;
  EXPECT_EQ(code(good_params, zero_curvature, safe), ErrorCode::kInvalidArgument);

  const std::vector<Eigen::Vector3f> non_finite = {safe[0],
                                                   {2.5f, 2.0f, std::nanf("")}};
  EXPECT_EQ(code(good_params, good_robot, non_finite), ErrorCode::kInvalidArgument);
}

// The frozen six-argument optimize cannot recover Portal geometry from
// Corridor.portal ids, so funnel is a deterministic no-op there; the compatible
// overload takes the ordered Portals and engages funnel.
TEST(PathOptimizerTest, OverloadUsesOrderedPortalsWhileFrozenSignatureSkipsFunnel) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  RobotParams robot;
  // This case isolates ordered-portal funneling. Curvature enforcement is
  // covered independently, so use a permissive limit for the deliberate bend.
  robot.max_path_curvature = 10.0f;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.smoothing_enabled = false;  // isolate funnel
  params.funnel_enabled = true;
  const std::vector<Eigen::Vector3f> raw = {{1.5f, 1.5f, kFlatZ},
                                            {2.5f, 1.5f, kFlatZ},
                                            {3.5f, 1.5f, kFlatZ},
                                            {4.5f, 1.5f, kFlatZ}};
  const std::vector<Portal> portals = {MakePortal({3.0f, 3.0f, kFlatZ}, {3.0f, 2.0f, kFlatZ})};

  const auto frozen = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(frozen.ok()) << frozen.status().message;
  EXPECT_EQ(frozen.value().size(), raw.size());

  const auto overload =
      PathOptimizer().optimize(raw, mesh, corridor, portals, params, robot, trav);
  ASSERT_TRUE(overload.ok()) << overload.status().message;
  EXPECT_LT(overload.value().size(), raw.size());
}

TEST(PathOptimizerTest, OverloadRunsShortcutFunnelSmoothInOrder) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.smoothing_iterations = 20;
  const std::vector<Eigen::Vector3f> raw = {{1.5f, 1.5f, kFlatZ},
                                            {2.5f, 1.5f, kFlatZ},
                                            {3.5f, 1.5f, kFlatZ},
                                            {4.5f, 1.5f, kFlatZ}};
  const std::vector<Portal> portals = {MakePortal({3.0f, 3.0f, kFlatZ}, {3.0f, 2.0f, kFlatZ})};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, portals, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_EQ(result.value().size(), 3u);
  EXPECT_TRUE(result.value().front().isApprox(raw.front()));
  EXPECT_TRUE(result.value().back().isApprox(raw.back()));
  // Funnel lifted the path to the gate; smooth kept a feasible corner.
  EXPECT_GT(result.value()[1].y(), 1.5f);
  EXPECT_LE(MaxCurvature(result.value()), robot.max_path_curvature + 1e-5f);

  const auto again = PathOptimizer().optimize(raw, mesh, corridor, portals, params, robot, trav);
  ASSERT_TRUE(again.ok()) << again.status().message;
  ASSERT_EQ(again.value().size(), result.value().size());
  for (std::size_t i = 0; i < result.value().size(); ++i) {
    EXPECT_TRUE(again.value()[i].isApprox(result.value()[i]));
  }
}

// T34: a 90-degree corner whose incident segments are long enough to hold the
// tangent distance R * tan(45) = 1 / max_path_curvature * 1.05 = 4.2 m is
// replaced by a safe circular arc. On this flat 6x6 scene the incoming segment
// leaves 0.6 m of straight, the outgoing segment is exactly the tangent length,
// and the arc (radius 4.2, clearance >= 0.6 m) stays well inside the surface.
TEST(PathOptimizerTest, FilletRoundsFeasibleNinetyDegreeTurn) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  const PlanParams params;
  const std::vector<Eigen::Vector3f> raw = {{0.6f, 0.6f, kFlatZ},
                                            {0.6f, 5.4f, kFlatZ},
                                            {4.8f, 5.4f, kFlatZ}};
  ASSERT_GT(MaxCurvature(raw), robot.max_path_curvature);

  const auto out = PathOptimizer().fillet(raw, mesh, corridor, params, robot, trav);
  EXPECT_GT(out.size(), raw.size());
  ASSERT_GE(out.size(), 2u);
  EXPECT_TRUE(out.front().isApprox(raw.front()));
  EXPECT_TRUE(out.back().isApprox(raw.back()));
  EXPECT_LE(MaxCurvature(out), robot.max_path_curvature);
  // check_step bounds the *arc sample* chord, not the straight portions of the
  // input path: 0.6 m of the incoming segment is legitimately left straight, so
  // only non-collinear (arc) segments are required to stay within check_step.
  for (std::size_t i = 0; i + 1 < out.size(); ++i) {
    const Eigen::Vector3f segment = out[i + 1] - out[i];
    EXPECT_GT(segment.norm(), 0.0f) << "duplicate point at " << i;
    bool collinear_with_input = false;
    for (std::size_t j = 0; j + 1 < raw.size(); ++j) {
      const Eigen::Vector3f input = raw[j + 1] - raw[j];
      if (segment.cross(input).norm() <= 1e-5f * segment.norm() * input.norm()) {
        collinear_with_input = true;
        break;
      }
    }
    if (!collinear_with_input) {
      EXPECT_LE(segment.norm(), params.check_step + 1e-4f)
          << "arc chord exceeds check_step at " << i;
    }
  }
  // The arc moves the path away from the sharp corner at (0.6, 5.4): no emitted
  // point may sit on the original corner.
  for (const Eigen::Vector3f& point : out) {
    EXPECT_GT((point - raw[1]).norm(), 1e-3f);
  }
  // The same result is deterministic and the authoritative optimizer accepts it
  // when the pre-fillet shortcut is disabled.
  const auto again = PathOptimizer().fillet(raw, mesh, corridor, params, robot, trav);
  ASSERT_EQ(again.size(), out.size());
  for (std::size_t i = 0; i < out.size(); ++i) EXPECT_TRUE(again[i].isApprox(out[i]));

  PlanParams isolated = params;
  isolated.shortcut_enabled = false;
  isolated.funnel_enabled = false;
  const auto optimized = PathOptimizer().optimize(raw, mesh, corridor, isolated, robot, trav);
  ASSERT_TRUE(optimized.ok()) << optimized.status().message;
  EXPECT_LE(MaxCurvature(optimized.value()), robot.max_path_curvature);
}

// T34: the same 90-degree turn on a much shorter pair of segments cannot hold
// the 4.2 m tangent distance. The radius is never shrunk below the curvature
// limit, so the corner is left unchanged and optimize() reports the failure
// instead of hiding it with degenerate points.
TEST(PathOptimizerTest, FilletLeavesTooShortTurnInfeasible) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  const PlanParams params;
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.0f, kFlatZ},
                                            {3.0f, 3.0f, kFlatZ},
                                            {2.0f, 4.0f, kFlatZ}};
  ASSERT_GT(MaxCurvature(raw), robot.max_path_curvature);

  const auto out = PathOptimizer().fillet(raw, mesh, corridor, params, robot, trav);
  ASSERT_EQ(out.size(), raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) EXPECT_TRUE(out[i].isApprox(raw[i]));
  EXPECT_GT(MaxCurvature(out), robot.max_path_curvature);

  PlanParams isolated = params;
  isolated.shortcut_enabled = false;
  isolated.funnel_enabled = false;
  isolated.smoothing_enabled = false;
  const auto optimized = PathOptimizer().optimize(raw, mesh, corridor, isolated, robot, trav);
  ASSERT_FALSE(optimized.ok());
  EXPECT_EQ(optimized.status().code, ErrorCode::kPathOptimizationFailed);
}

// T34: a U-turn needs an infinite tangent, so it can never be rounded with a
// finite-radius arc; the input is returned unchanged (endpoints included).
TEST(PathOptimizerTest, FilletRejectsUTurn) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  const PlanParams params;
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.0f, kFlatZ},
                                            {4.0f, 2.0f, kFlatZ},
                                            {2.0f, 2.2f, kFlatZ}};
  const auto out = PathOptimizer().fillet(raw, mesh, corridor, params, robot, trav);
  ASSERT_EQ(out.size(), raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) EXPECT_TRUE(out[i].isApprox(raw[i]));
}

// T34: a nearly collinear corner is not a violating corner and must not be
// rounded, so the point count and geometry are untouched.
TEST(PathOptimizerTest, FilletLeavesCollinearCornerUntouched) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  const PlanParams params;
  const std::vector<Eigen::Vector3f> raw = {{1.5f, 2.0f, kFlatZ},
                                            {2.5f, 2.0f, kFlatZ},
                                            {3.5f, 2.0f, kFlatZ}};
  const auto out = PathOptimizer().fillet(raw, mesh, corridor, params, robot, trav);
  ASSERT_EQ(out.size(), raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) EXPECT_TRUE(out[i].isApprox(raw[i]));
}
