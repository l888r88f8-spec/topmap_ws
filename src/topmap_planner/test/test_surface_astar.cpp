#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "test_support_plan.hpp"
#include "topmap_core/geometry.hpp"
#include "topmap_planner/corridor_builder.hpp"
#include "topmap_planner/region_locator.hpp"
#include "topmap_planner/surface_astar.hpp"
#include "topmap_planner/topology_astar.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"

namespace {

using topmap::Corridor;
using topmap::CorridorBuilder;
using topmap::ErrorCode;
using topmap::PlanParams;
using topmap::RegionLocator;
using topmap::SurfaceAStar;
using topmap::TopologyAStar;
using topmap::TraversabilityEvaluator;
using topmap::TraversabilityParams;
using topmap::TriangleId;
using topmap::TriangleMesh;

struct CorridorFixture {
  TriangleMesh mesh;
  topmap::TopologyGraph graph;
  Corridor corridor;
  std::vector<Eigen::Vector3f> waypoints;
};

// Builds the real T12-T17 pipeline and the start->goal topology corridor. Note:
// the actual Region id order on kRamp is [ramp, high, low], so the start/goal
// Regions are located with RegionLocator (the same helper T27 uses) rather than
// relying on regions().front()/back().
//
// Returns the failure Status instead of a half-built fixture: a test that kept
// going after EXPECT_TRUE(ok()) and then called Result::value() would read an
// unconstructed value (UB). Callers must ASSERT_TRUE before touching the value.
topmap::Result<CorridorFixture> MakeCorridorFixture(topmap::SyntheticScene scene) {
  auto built = topmap::test::Build(scene);
  if (!built.ok()) return built.status();
  CorridorFixture fixture;
  fixture.mesh = std::move(built.value().mesh);
  fixture.graph = std::move(built.value().graph);

  auto endpoints = topmap::makeSyntheticEndpoints(scene);
  if (!endpoints.ok()) return endpoints.status();
  const PlanParams params;
  auto start_region =
      RegionLocator().locate(fixture.mesh, fixture.graph, endpoints.value().first, params);
  if (!start_region.ok()) return start_region.status();
  auto goal_region =
      RegionLocator().locate(fixture.mesh, fixture.graph, endpoints.value().second, params);
  if (!goal_region.ok()) return goal_region.status();

  auto topology = TopologyAStar().search(fixture.graph, start_region.value(), goal_region.value());
  if (!topology.ok()) return topology.status();
  auto corridor = CorridorBuilder().build(fixture.mesh, fixture.graph, topology.value(), 1);
  if (!corridor.ok()) return corridor.status();
  fixture.corridor = corridor.take();

  fixture.waypoints.push_back(endpoints.value().first);
  for (const topmap::PortalId portal_id : topology.value().portals) {
    fixture.waypoints.push_back(fixture.graph.portal(portal_id)->center);
  }
  fixture.waypoints.push_back(endpoints.value().second);
  return fixture;
}

// A fine XY grid over the corridor centroids so the test can find the nearest
// corridor triangle to a sample in ~O(1) instead of scanning all ~24k triangles
// (the frozen verification build is Debug, where a full scan is prohibitively
// slow). Each cell holds the triangles whose centroid falls in it; a triangle's
// centroid is within ~0.1 m of any point on that 0.1 m-lattice triangle, so the
// local 3x3 neighbourhood of one 0.15 m cell always contains it. A full scan of
// the corridor is kept as a correctness fallback so the query can never report
// a false negative.
class CorridorGrid {
 public:
  CorridorGrid(const TriangleMesh& mesh, const Corridor& corridor) : mesh_(mesh) {
    for (const TriangleId id : corridor.triangles) {
      if (id < 0 || id >= static_cast<TriangleId>(mesh.size())) continue;
      corridor_triangles_.push_back(id);
      const Eigen::Vector3f& centroid = mesh.triangle(id).centroid;
      cells_[Key(CellOf(centroid.x()), CellOf(centroid.y()))].push_back(id);
    }
  }

  bool NearestIsRobotPlaceable(const Eigen::Vector3f& point,
                               const TraversabilityParams& trav) const {
    const long cx = CellOf(point.x());
    const long cy = CellOf(point.y());
    TriangleId nearest = topmap::kInvalidId;
    float nearest_distance = std::numeric_limits<float>::infinity();
    const auto consider = [&](TriangleId id) {
      const topmap::TriangleNode& triangle = mesh_.triangle(id);
      const float distance =
          topmap::pointTriangleDistance(point, mesh_.vertices()[triangle.vertex_ids[0]],
                                        mesh_.vertices()[triangle.vertex_ids[1]],
                                        mesh_.vertices()[triangle.vertex_ids[2]]);
      if (nearest == topmap::kInvalidId || distance < nearest_distance) {
        nearest = id;
        nearest_distance = distance;
      }
    };
    for (long dx = -1; dx <= 1; ++dx) {
      for (long dy = -1; dy <= 1; ++dy) {
        const auto it = cells_.find(Key(cx + dx, cy + dy));
        if (it == cells_.end()) continue;
        for (const TriangleId id : it->second) consider(id);
      }
    }
    if (nearest == topmap::kInvalidId) {
      for (const TriangleId id : corridor_triangles_) consider(id);
    }
    if (nearest == topmap::kInvalidId) return false;
    return TraversabilityEvaluator::isRobotPlaceable(mesh_.triangle(nearest), trav);
  }

 private:
  static constexpr float kCellSize = 0.15f;

  static long CellOf(float value) {
    return static_cast<long>(std::floor(value / kCellSize));
  }

  static long Key(long cx, long cy) { return (cx + 1000L) * 100000L + (cy + 1000L); }

  const TriangleMesh& mesh_;
  std::vector<TriangleId> corridor_triangles_;
  std::unordered_map<long, std::vector<TriangleId>> cells_;
};

TEST(SurfaceAStarTest, PathConnectsStartGoalThroughWaypoints) {
  auto fixture_result = MakeCorridorFixture(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(fixture_result.ok()) << fixture_result.status().message;
  CorridorFixture fixture = fixture_result.take();
  int expanded = 0;
  const TraversabilityParams trav;
  auto path = SurfaceAStar().search(fixture.mesh, fixture.corridor, fixture.waypoints, trav,
                                    &expanded);
  ASSERT_TRUE(path.ok()) << path.status().message;
  ASSERT_GE(path.value().size(), 2u);
  EXPECT_GT(expanded, 0);
  EXPECT_LT((path.value().front() - fixture.waypoints.front()).norm(), 0.5f);
  EXPECT_LT((path.value().back() - fixture.waypoints.back()).norm(), 0.5f);
}

TEST(SurfaceAStarTest, PreservesWaypointOrderAlongPolyline) {
  auto fixture_result = MakeCorridorFixture(topmap::SyntheticScene::kHallNarrowHall);
  ASSERT_TRUE(fixture_result.ok()) << fixture_result.status().message;
  CorridorFixture fixture = fixture_result.take();
  ASSERT_GE(fixture.waypoints.size(), 3u);
  const TraversabilityParams trav;
  auto path = SurfaceAStar().search(fixture.mesh, fixture.corridor, fixture.waypoints, trav);
  ASSERT_TRUE(path.ok()) << path.status().message;

  // Every input waypoint (or its exact projection) must appear in order.
  std::size_t cursor = 0;
  for (const Eigen::Vector3f& waypoint : fixture.waypoints) {
    bool found = false;
    for (std::size_t i = cursor; i < path.value().size(); ++i) {
      if ((path.value()[i] - waypoint).norm() < 0.5f) {
        cursor = i + 1;
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "waypoint not found in order";
  }
}

TEST(SurfaceAStarTest, PathStaysOnPlaceableCorridorTriangles) {
  auto fixture_result = MakeCorridorFixture(topmap::SyntheticScene::kHallNarrowHall);
  ASSERT_TRUE(fixture_result.ok()) << fixture_result.status().message;
  CorridorFixture fixture = fixture_result.take();
  const TraversabilityParams trav;
  auto path = SurfaceAStar().search(fixture.mesh, fixture.corridor, fixture.waypoints, trav);
  ASSERT_TRUE(path.ok()) << path.status().message;
  const CorridorGrid grid(fixture.mesh, fixture.corridor);

  // R10 regression: sample the path (vertices and a few interior points of every
  // edge) and require the nearest corridor triangle to be robot-placeable, so A*
  // can never take a clearance-violating shortcut.
  for (std::size_t i = 0; i + 1 < path.value().size(); ++i) {
    const Eigen::Vector3f& a = path.value()[i];
    const Eigen::Vector3f& b = path.value()[i + 1];
    for (int s = 0; s <= 3; ++s) {
      const float t = static_cast<float>(s) / 3.0f;
      const Eigen::Vector3f sample = a + (b - a) * t;
      EXPECT_TRUE(grid.NearestIsRobotPlaceable(sample, trav))
          << "path crosses a non-placeable triangle at sample " << s << " of edge " << i;
    }
  }
}

TEST(SurfaceAStarTest, MissingCorridorTriangleFailsAndInitializesExpanded) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor empty;
  const TraversabilityParams trav;
  int expanded = 12345;
  auto path = SurfaceAStar().search(built.value().mesh, empty,
                                    {Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 0, 1)}, trav,
                                    &expanded);
  ASSERT_FALSE(path.ok());
  EXPECT_EQ(path.status().code, ErrorCode::kSurfaceSearchFailed);
  EXPECT_EQ(expanded, 0);
}

TEST(SurfaceAStarTest, RejectsFewerThanTwoWaypoints) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  const TraversabilityParams trav;
  auto path = SurfaceAStar().search(built.value().mesh, corridor,
                                    {Eigen::Vector3f(3.0f, 3.0f, 0.0f)}, trav);
  ASSERT_FALSE(path.ok());
  EXPECT_EQ(path.status().code, ErrorCode::kInvalidArgument);
}

TEST(SurfaceAStarTest, RejectsNonFiniteWaypoint) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  const TraversabilityParams trav;
  std::vector<Eigen::Vector3f> waypoints = {Eigen::Vector3f(3.0f, 3.0f, 0.0f),
                                            Eigen::Vector3f(3.5f, 3.0f, std::nanf(""))};
  auto path = SurfaceAStar().search(built.value().mesh, corridor, waypoints, trav);
  ASSERT_FALSE(path.ok());
  EXPECT_EQ(path.status().code, ErrorCode::kInvalidArgument);
}

TEST(SurfaceAStarTest, WaypointOutsideCorridorFails) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  const TraversabilityParams trav;
  std::vector<Eigen::Vector3f> waypoints = {Eigen::Vector3f(3.0f, 3.0f, 0.0f),
                                            Eigen::Vector3f(3.0f, 3.0f, 100.0f)};
  auto path = SurfaceAStar().search(built.value().mesh, corridor, waypoints, trav);
  ASSERT_FALSE(path.ok());
  EXPECT_EQ(path.status().code, ErrorCode::kSurfaceSearchFailed);
}

TEST(SurfaceAStarTest, CrossComponentSearchFails) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  auto endpoints = topmap::makeSyntheticEndpoints(topmap::SyntheticScene::kOverlappingFloors);
  ASSERT_TRUE(endpoints.ok()) << endpoints.status().message;
  const TraversabilityParams trav;
  std::vector<Eigen::Vector3f> waypoints = {endpoints.value().first, endpoints.value().second};
  auto path = SurfaceAStar().search(built.value().mesh, corridor, waypoints, trav);
  ASSERT_FALSE(path.ok());
  EXPECT_EQ(path.status().code, ErrorCode::kSurfaceSearchFailed);
}

TEST(SurfaceAStarTest, SearchIsDeterministic) {
  auto fixture_result = MakeCorridorFixture(topmap::SyntheticScene::kRamp);
  ASSERT_TRUE(fixture_result.ok()) << fixture_result.status().message;
  CorridorFixture fixture = fixture_result.take();
  const TraversabilityParams trav;
  int first_expanded = 0;
  int second_expanded = 0;
  auto first =
      SurfaceAStar().search(fixture.mesh, fixture.corridor, fixture.waypoints, trav, &first_expanded);
  auto second = SurfaceAStar().search(fixture.mesh, fixture.corridor, fixture.waypoints, trav,
                                      &second_expanded);
  ASSERT_TRUE(first.ok()) << first.status().message;
  ASSERT_TRUE(second.ok()) << second.status().message;
  ASSERT_EQ(first.value().size(), second.value().size());
  EXPECT_EQ(first_expanded, second_expanded);
  for (std::size_t i = 0; i < first.value().size(); ++i) {
    EXPECT_TRUE(first.value()[i].isApprox(second.value()[i]));
  }
}

TEST(SurfaceAStarTest, RejectsInvalidCorridorTriangleId) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  const TraversabilityParams trav;
  const std::vector<Eigen::Vector3f> waypoints = {Eigen::Vector3f(1.0f, 1.0f, 0.0f),
                                                  Eigen::Vector3f(3.0f, 1.0f, 0.0f)};

  // Any id outside [0, mesh.size()) makes the corridor malformed. It must be a
  // structured failure, never a silent skip that lets the search succeed.
  Corridor too_high = corridor;
  too_high.triangles.push_back(static_cast<TriangleId>(built.value().mesh.size()));
  auto high = SurfaceAStar().search(built.value().mesh, too_high, waypoints, trav);
  ASSERT_FALSE(high.ok());
  EXPECT_EQ(high.status().code, ErrorCode::kInvalidArgument);

  Corridor negative = corridor;
  negative.triangles.push_back(static_cast<TriangleId>(-2));
  auto low = SurfaceAStar().search(built.value().mesh, negative, waypoints, trav);
  ASSERT_FALSE(low.ok());
  EXPECT_EQ(low.status().code, ErrorCode::kInvalidArgument);
}

TEST(SurfaceAStarTest, RejectsInvalidTraversabilityParams) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  const std::vector<Eigen::Vector3f> waypoints = {Eigen::Vector3f(1.0f, 1.0f, 0.0f),
                                                  Eigen::Vector3f(3.0f, 1.0f, 0.0f)};
  const TraversabilityParams valid;
  const auto code_of = [&](const TraversabilityParams& trav) {
    return SurfaceAStar().search(built.value().mesh, corridor, waypoints, trav).status().code;
  };

  TraversabilityParams negative_step = valid;
  negative_step.max_step_height = -1.0f;
  EXPECT_EQ(code_of(negative_step), ErrorCode::kInvalidArgument);

  TraversabilityParams nan_normal = valid;
  nan_normal.max_normal_diff_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(code_of(nan_normal), ErrorCode::kInvalidArgument);

  TraversabilityParams negative_clearance = valid;
  negative_clearance.required_clearance = -0.1f;
  EXPECT_EQ(code_of(negative_clearance), ErrorCode::kInvalidArgument);

  TraversabilityParams infinite_slope = valid;
  infinite_slope.max_slope_deg = std::numeric_limits<float>::infinity();
  EXPECT_EQ(code_of(infinite_slope), ErrorCode::kInvalidArgument);

  TraversabilityParams negative_area = valid;
  negative_area.min_triangle_area = -1.0f;
  EXPECT_EQ(code_of(negative_area), ErrorCode::kInvalidArgument);
}

TEST(SurfaceAStarTest, PreservesRepeatedWaypointAtSamePoint) {
  auto built = topmap::test::Build(topmap::SyntheticScene::kFlatPlane);
  ASSERT_TRUE(built.ok()) << built.status().message;
  Corridor corridor;
  corridor.triangles = topmap::test::AllTriangleIds(built.value().mesh);
  const TraversabilityParams trav;
  const Eigen::Vector3f repeated(3.0f, 1.0f, 0.0f);
  const std::vector<Eigen::Vector3f> waypoints = {Eigen::Vector3f(1.0f, 1.0f, 0.0f), repeated,
                                                  repeated};
  auto path = SurfaceAStar().search(built.value().mesh, corridor, waypoints, trav);
  ASSERT_TRUE(path.ok()) << path.status().message;

  // The two repeated waypoints are semantically distinct inputs even though
  // they project to the same point, so the junction dedup must not collapse
  // them: the projection must appear twice and each waypoint must be found in
  // order.
  std::size_t exact_occurrences = 0;
  for (const Eigen::Vector3f& point : path.value()) {
    if ((point - repeated).norm() <= 1e-6f) ++exact_occurrences;
  }
  EXPECT_GE(exact_occurrences, 2u);

  std::size_t cursor = 0;
  for (const Eigen::Vector3f& waypoint : waypoints) {
    bool found = false;
    for (std::size_t i = cursor; i < path.value().size(); ++i) {
      if ((path.value()[i] - waypoint).norm() < 0.5f) {
        cursor = i + 1;
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "repeated waypoint not found in order";
  }
}

}  // namespace
