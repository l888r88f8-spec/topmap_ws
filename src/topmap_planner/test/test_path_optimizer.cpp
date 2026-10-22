// T25: tests for PathOptimizer (shortcut / funnel / curvature-limited smooth).
//
// All flat-plane probes use interior points with real clearance (never the
// x=0 / y=0 mesh boundary), so the tests exercise the R10 placeability gate
// instead of relaxing it. The overlapping-floors case proves that a shortcut
// can never jump between mesh components / layers.

#include <gtest/gtest.h>

#include <algorithm>
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

struct CreaseCrossing {
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  float segment_parameter = 0.0f;
};

// Independent geometry query: scans actual shared mesh edges whose two adjacent
// face planes differ, then intersects those edge segments with the guide's XY
// segment. The point is interpolated from the real edge endpoints, so a test can
// demand the planner emit that exact edge point rather than a hard-coded x/z.
std::vector<CreaseCrossing> CreaseCrossingsAlong(const TriangleMesh& mesh,
                                                 const Eigen::Vector3f& from,
                                                 const Eigen::Vector3f& to) {
  std::vector<CreaseCrossing> crossings;
  const Eigen::Vector2f a = from.head<2>();
  const Eigen::Vector2f r = to.head<2>() - a;
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const topmap::TriangleNode& triangle = mesh.triangle(id);
    for (const topmap::TriangleEdge& edge : triangle.edges) {
      if (edge.neighbor == topmap::kInvalidId || edge.neighbor < id) continue;
      const topmap::TriangleNode& neighbour = mesh.triangle(edge.neighbor);
      // Only a real change of face plane is a terrain crease. Coplanar grid
      // triangles (same plane, shared diagonal) are intentionally ignored.
      if (triangle.normal.cross(neighbour.normal).norm() <= 1e-4f) continue;
      const Eigen::Vector3f& e0 = mesh.vertices()[edge.v0];
      const Eigen::Vector3f& e1 = mesh.vertices()[edge.v1];
      const Eigen::Vector2f e0_2d = e0.head<2>();
      const Eigen::Vector2f s = e1.head<2>() - e0_2d;
      const float denominator = r.x() * s.y() - r.y() * s.x();
      if (std::fabs(denominator) <= 1e-12f) continue;
      const Eigen::Vector2f offset = e0_2d - a;
      const float t = (offset.x() * s.y() - offset.y() * s.x()) / denominator;
      const float u = (offset.x() * r.y() - offset.y() * r.x()) / denominator;
      if (t < -1e-6f || t > 1.0f + 1e-6f || u < -1e-6f || u > 1.0f + 1e-6f) continue;
      CreaseCrossing crossing;
      crossing.segment_parameter = t;
      crossing.point = e0 + u * (e1 - e0);
      crossings.push_back(crossing);
    }
  }
  std::sort(crossings.begin(), crossings.end(),
            [](const CreaseCrossing& lhs, const CreaseCrossing& rhs) {
              return lhs.segment_parameter < rhs.segment_parameter;
            });
  std::vector<CreaseCrossing> unique;
  for (const CreaseCrossing& crossing : crossings) {
    if (unique.empty() || (crossing.point - unique.back().point).norm() > 1e-5f) {
      unique.push_back(crossing);
    }
  }
  return unique;
}

float DistanceToPath(const std::vector<Eigen::Vector3f>& path, const Eigen::Vector3f& point) {
  float best = std::numeric_limits<float>::infinity();
  for (const Eigen::Vector3f& candidate : path) {
    best = std::min(best, (candidate - point).norm());
  }
  return best;
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

// Independent point-to-surface measurement used by the adherence regressions.
// It scans every mesh triangle (no optimizer/validator grid or projection
// tolerance) so it cannot inherit the bug it is meant to catch.
float DistanceToMesh(const TriangleMesh& mesh, const Eigen::Vector3f& p) {
  float best = std::numeric_limits<float>::infinity();
  for (TriangleId id = 0; id < static_cast<TriangleId>(mesh.size()); ++id) {
    const topmap::TriangleNode& t = mesh.triangle(id);
    best = std::min(best, topmap::pointTriangleDistance(
                              p, mesh.vertices()[t.vertex_ids[0]],
                              mesh.vertices()[t.vertex_ids[1]],
                              mesh.vertices()[t.vertex_ids[2]]));
  }
  return best;
}

float MaxPathDistanceToMesh(const TriangleMesh& mesh, const std::vector<Eigen::Vector3f>& path,
                            float step) {
  float maximum = 0.0f;
  for (const Eigen::Vector3f& point : path) {
    maximum = std::max(maximum, DistanceToMesh(mesh, point));
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const float length = (path[i + 1] - path[i]).norm();
    int samples = static_cast<int>(std::ceil(length / step));
    if (samples < 1) samples = 1;
    for (int s = 1; s < samples; ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(samples);
      maximum = std::max(maximum, DistanceToMesh(mesh, path[i] + t * (path[i + 1] - path[i])));
    }
  }
  return maximum;
}

constexpr float kStrictSurfaceTolerance = 0.005f;

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
  // Even with every optional guide stage off, the final stage still lifts the
  // returned path onto the real mesh. Without shortcutting the lifted path is
  // dense rather than the three input vertices, but it must keep the exact
  // endpoints and stay surface-adherent.
  EXPECT_GE(result.value().size(), raw.size());
  EXPECT_TRUE(result.value().front().isApprox(raw.front()));
  EXPECT_TRUE(result.value().back().isApprox(raw.back()));
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
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
  EXPECT_GE(result.value().size(), raw.size());
  EXPECT_TRUE(result.value().front().isApprox(raw.front()));
  EXPECT_TRUE(result.value().back().isApprox(raw.back()));
  EXPECT_LE(MaxCurvature(result.value()), robot.max_path_curvature + 1e-5f);
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
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
  // Without Portal geometry the frozen signature cannot engage funnel; the
  // dense lifted straight guide stays on y = 1.5.
  float frozen_max_y = -std::numeric_limits<float>::infinity();
  for (const auto& point : frozen.value()) frozen_max_y = std::max(frozen_max_y, point.y());
  EXPECT_NEAR(frozen_max_y, 1.5f, 1e-3f);

  const auto overload =
      PathOptimizer().optimize(raw, mesh, corridor, portals, params, robot, trav);
  ASSERT_TRUE(overload.ok()) << overload.status().message;
  EXPECT_TRUE(overload.value().front().isApprox(raw.front()));
  EXPECT_TRUE(overload.value().back().isApprox(raw.back()));
  // Funnel must lift the guide through the Portal; the final surface stage may
  // keep it dense (shortcutting is disabled here) but must not erase the gate.
  float overload_max_y = -std::numeric_limits<float>::infinity();
  for (const auto& point : overload.value()) {
    overload_max_y = std::max(overload_max_y, point.y());
  }
  EXPECT_GT(overload_max_y, 1.9f);
  EXPECT_GE(overload.value().size(), 3u);
  EXPECT_LE(MaxPathDistanceToMesh(mesh, overload.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
}

// A forced Portal at x = 3 that is at least 0.5 m off a straight guide forces a
// detour which cannot cross the gate and stay under the default 0.25 1/m
// curvature limit within the available x span. The optimizer must reject this
// guide instead of returning a path that silently misses the Portal or exceeds
// the curvature contract. (The earlier positive version of this test only
// "succeeded" because smoothing dropped the Portal.)
TEST(PathOptimizerTest, OverloadRejectsInfeasiblePortalCurvature) {
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
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kPathOptimizationFailed);

  // Rejection is deterministic; a second run must not fall back to the raw
  // straight guide, which never crosses the forced Portal.
  const auto again = PathOptimizer().optimize(raw, mesh, corridor, portals, params, robot, trav);
  ASSERT_FALSE(again.ok());
  EXPECT_EQ(again.status().code, ErrorCode::kPathOptimizationFailed);
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

// A guide whose straight XY chord crosses the U-shaped scene's hole cannot be
// repaired by snapping to the nearest boundary: every sample must be contained
// by a traversable triangle at its own XY.
TEST(PathOptimizerTest, OptimizeRejectsGuideCrossingHole) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kUShape);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  // Both endpoints are on the U, but the straight chord crosses the open void.
  const std::vector<Eigen::Vector3f> raw = {{3.5f, 4.5f, kFlatZ}, {35.0f, 45.0f, kFlatZ}};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kPathOptimizationFailed);
}

// A vertical guide between overlapping floors must not hop components just
// because both floors contain the same XY.
TEST(PathOptimizerTest, OptimizeRejectsCrossComponentVerticalGuide) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const std::vector<Eigen::Vector3f> raw = {{2.5f, 2.5f, 0.0f}, {2.5f, 2.5f, 3.0f}};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, ErrorCode::kPathOptimizationFailed);
}

// Multilayer/index-boundary regression: both floors contain the same XY, and the
// guide starts exactly on a lattice line then crosses many index cells. Dense
// sampling must stay on the lower layer selected by the first sample instead of
// swapping to the upper floor at a cell boundary.
TEST(PathOptimizerTest, OptimizeKeepsLowerLayerAcrossGridCellBoundaries) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.5f, kFlatZ},
                                            {3.0f, 2.5f, kFlatZ},
                                            {4.0f, 2.5f, kFlatZ}};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_GE(result.value().size(), 2u);
  for (const Eigen::Vector3f& point : result.value()) {
    EXPECT_NEAR(point.z(), kFlatZ, kStrictSurfaceTolerance + 1e-4f);
  }
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
}

// Regression: a sparse shortcut across the flat -> ramp crease can stay within
// the 0.30 m endpoint-projection radius while floating almost 0.30 m above the
// real surface. The final optimizer output must follow the mesh instead.
TEST(PathOptimizerTest, OptimizeLiftsFloatingRampEntryChordToSurface) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const Eigen::Vector3f start(2.0f, 2.0f, 0.05f);
  const Eigen::Vector3f ramp(7.4f, 2.0f, 0.7234043f);
  const std::vector<Eigen::Vector3f> raw = {start, ramp};

  // The raw chord is accepted by the old 0.30 m surface search: it floats
  // ~0.299 m above the flat/ramp junction.
  ASSERT_GT(MaxPathDistanceToMesh(mesh, raw, params.check_step), 0.20f);

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_GE(result.value().size(), 2u);
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
  EXPECT_NEAR(result.value().front().x(), start.x(), 1e-4f);
  EXPECT_NEAR(result.value().front().y(), start.y(), 1e-4f);
  EXPECT_NEAR(result.value().back().x(), ramp.x(), 1e-4f);
  EXPECT_NEAR(result.value().back().y(), ramp.y(), 1e-4f);
}

// Reverse travel must get the same surface adherence, including the same crease.
TEST(PathOptimizerTest, OptimizeLiftsFloatingRampExitChordInReverse) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const Eigen::Vector3f ramp(7.4f, 2.0f, 0.7234043f);
  const Eigen::Vector3f start(2.0f, 2.0f, 0.05f);
  const std::vector<Eigen::Vector3f> raw = {ramp, start};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
  EXPECT_NEAR(result.value().front().x(), ramp.x(), 1e-4f);
  EXPECT_NEAR(result.value().back().x(), start.x(), 1e-4f);
}

// RED regression for the approved crease-preserving path: the guide crosses two
// real shared edges where the flat/ramp and ramp/platform planes change. With
// default optimizations the returned path must contain the exact mesh edge
// intersections (the foot and top of the ramp), not merely points sampled close
// to them. The expected points are computed from the mesh edges themselves.
TEST(PathOptimizerTest, OptimizePreservesExactRampFootAndTopCreases) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  const PlanParams params;  // default shortcut/funnel/smoothing stay enabled
  const std::vector<Eigen::Vector3f> raw = {{2.0f, 2.0f, 0.05f},
                                            {3.9f, 2.0f, 0.0f},
                                            {8.0f, 2.0f, 0.8510638f},
                                            {13.5f, 2.0f, 2.0f},
                                            {15.4f, 2.0f, 2.0f}};
  const auto crossings = CreaseCrossingsAlong(mesh, raw.front(), raw.back());
  ASSERT_GE(crossings.size(), 2u) << "synthetic ramp lost its detected creases";
  // The guide crosses the foot first and the top last.
  const std::vector<Eigen::Vector3f> expected = {crossings.front().point,
                                                 crossings.back().point};
  ASSERT_NEAR(expected.front().z(), 0.0f, 1e-5f);
  ASSERT_NEAR(expected.back().z(), 2.0f, 1e-5f);

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
  for (const Eigen::Vector3f& crease : expected) {
    EXPECT_LE(DistanceToPath(result.value(), crease), 1e-5f)
        << "missing exact crease point [" << crease.x() << ',' << crease.y() << ','
        << crease.z() << ']';
  }
}

// Reverse travel must keep the same exact crease points; the intersection order
// and the guide direction are reversed, not the geometry contract.
TEST(PathOptimizerTest, OptimizePreservesExactRampCreasesInReverse) {
  auto mesh_result = BuildMesh(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  const TriangleMesh& mesh = mesh_result.value();
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  const TraversabilityParams trav;
  const PlanParams params;
  const std::vector<Eigen::Vector3f> forward = {{2.0f, 2.0f, 0.05f},
                                                {3.9f, 2.0f, 0.0f},
                                                {8.0f, 2.0f, 0.8510638f},
                                                {13.5f, 2.0f, 2.0f},
                                                {15.4f, 2.0f, 2.0f}};
  std::vector<Eigen::Vector3f> raw(forward.rbegin(), forward.rend());
  const auto crossings = CreaseCrossingsAlong(mesh, raw.front(), raw.back());
  ASSERT_GE(crossings.size(), 2u);
  const std::vector<Eigen::Vector3f> expected = {crossings.front().point,
                                                 crossings.back().point};
  ASSERT_NEAR(expected.front().z(), 2.0f, 1e-5f);
  ASSERT_NEAR(expected.back().z(), 0.0f, 1e-5f);

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_LE(MaxPathDistanceToMesh(mesh, result.value(), params.check_step),
            kStrictSurfaceTolerance + 1e-4f);
  for (const Eigen::Vector3f& crease : expected) {
    EXPECT_LE(DistanceToPath(result.value(), crease), 1e-5f)
        << "missing exact reverse crease point [" << crease.x() << ',' << crease.y()
        << ',' << crease.z() << ']';
  }
}

// Regression for the indexed-containment review finding: a very acute triangle
// can contain (within ContainsXy's relative epsilon) a point that lies just
// outside its XY AABB. The previous epsilon/min-edge margin registered the
// triangle one cell late, so the surface lift missed it (origin=-1, cell=1).
// The point below is accepted by the same signed-edge test, and this guide's
// first sample is exactly that point.
TEST(PathOptimizerTest, OptimizeFindsAcuteTriangleAcceptedOutsideItsCellAabb) {
  topmap::MeshData data;
  data.vertices = {
      // A helper triangle fixes the index origin at x=-1 and keeps the median
      // edge length at 1.0 m without touching the acute triangle.
      {-1.0f, 0.0f, 0.0f},
      {-1.0f, 1.0f, 0.0f},
      {0.0f, 1.0f, 0.0f},
      // The review counterexample: a ~0.57 degree wedge.
      {0.00015f, 0.0f, 0.0f},
      {1.00015f, 0.0f, 0.0f},
      {1.00015f, 0.01f, 0.0f},
  };
  data.faces = {{0, 1, 2}, {3, 4, 5}};
  auto mesh_result = TriangleMesh::fromMeshData(data, {});
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  TriangleMesh& mesh = mesh_result.value();
  TraversabilityParams trav;
  trav.required_clearance = 0.0f;
  TraversabilityEvaluator::apply(mesh, trav);
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  const std::vector<Eigen::Vector3f> raw = {{-0.00001f, -0.0000009f, 0.0f},
                                            {0.0005f, 0.000001f, 0.0f}};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_TRUE(result.ok()) << result.status().message;
  ASSERT_GE(result.value().size(), 2u);
  EXPECT_NEAR(result.value().front().x(), raw.front().x(), 1e-6f);
  EXPECT_NEAR(result.value().front().y(), raw.front().y(), 1e-6f);
}

// RED regression for the fast crease candidate: a same-component 6 mm hole is
// small enough that a straight endpoint chord is within the 0.30 m projection
// radius, so the old endpoint-only segment check accepts it. The candidate must
// be lifted at the strict SurfaceLiftStep sampling interval first; the midpoint
// lies over the hole and has no containing triangle, so optimize must fail
// rather than return a path that bridges the void.
TEST(PathOptimizerTest, OptimizeRejectsGuideCrossingTinySixMillimetreHole) {
  topmap::MeshData data;
  data.vertices = {{-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f},
                   {1.0f, 1.0f, 0.0f},   {-1.0f, 1.0f, 0.0f},
                   {-0.003f, -0.003f, 0.0f}, {0.003f, -0.003f, 0.0f},
                   {0.003f, 0.003f, 0.0f},   {-0.003f, 0.003f, 0.0f}};
  data.faces = {{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  auto mesh_result = TriangleMesh::fromMeshData(data, {});
  ASSERT_TRUE(mesh_result.ok()) << mesh_result.status().message;
  TriangleMesh& mesh = mesh_result.value();
  TraversabilityParams trav;
  trav.required_clearance = 0.0f;
  TraversabilityEvaluator::apply(mesh, trav);
  const Corridor corridor = MakeCorridor(mesh);
  const RobotParams robot;
  PlanParams params;
  params.shortcut_enabled = false;
  params.funnel_enabled = false;
  params.smoothing_enabled = false;
  // Both endpoints are on the same mesh component; only the 6 mm interior hole
  // separates them. The chord midpoint is (0, 0) in the hole.
  const std::vector<Eigen::Vector3f> raw = {{-0.014f, 0.0f, 0.0f},
                                            {0.014f, 0.0f, 0.0f}};

  const auto result = PathOptimizer().optimize(raw, mesh, corridor, params, robot, trav);
  ASSERT_FALSE(result.ok())
      << "optimizer accepted a straight chord across the 6 mm hole";
  EXPECT_EQ(result.status().code, ErrorCode::kPathOptimizationFailed);
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
