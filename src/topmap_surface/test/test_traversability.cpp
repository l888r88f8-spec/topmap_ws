#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "topmap_surface/traversability.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;

topmap::MeshData Strip(float center_z, float slope_deg) {
  topmap::MeshData d;
  const float t = std::tan(slope_deg * kPi / 180.0f);
  d.vertices = {{0, 0, center_z - t}, {2, 0, center_z + t}, {1, 1, center_z}};
  d.faces = {{0, 1, 2}};
  return d;
}

// A flat (z = 0) axis-aligned grid of nx*ny cells with edge length `step`.
topmap::MeshData GridMesh(int nx, int ny, float step) {
  topmap::MeshData d;
  d.vertices.reserve(static_cast<std::size_t>(nx + 1) * (ny + 1));
  for (int j = 0; j <= ny; ++j) {
    for (int i = 0; i <= nx; ++i) {
      d.vertices.emplace_back(i * step, j * step, 0.0f);
    }
  }
  const auto vid = [nx](int i, int j) { return j * (nx + 1) + i; };
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      d.faces.push_back({vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)});
      d.faces.push_back({vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)});
    }
  }
  return d;
}

topmap::TriangleNode MakeTriangle(const Eigen::Vector3f& centroid, const Eigen::Vector3f& normal,
                                  float slope, float area) {
  topmap::TriangleNode t;
  t.centroid = centroid;
  t.normal = normal;
  t.slope = slope;
  t.area = area;
  return t;
}

}  // namespace

TEST(TraversabilityTest, SlopeBoundaryAt20Degrees) {
  topmap::TraversabilityParams p;
  auto flat = topmap::TriangleMesh::fromMeshData(Strip(0.0f, 0.0f), {});
  auto steep = topmap::TriangleMesh::fromMeshData(Strip(0.0f, 25.0f), {});
  ASSERT_TRUE(flat.ok());
  ASSERT_TRUE(steep.ok());
  EXPECT_TRUE(topmap::TraversabilityEvaluator::isTriangleTraversable(flat.value().triangle(0), p));
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isTriangleTraversable(steep.value().triangle(0), p));
}

TEST(TraversabilityTest, StepAndNormalDifferenceRejectEdges) {
  topmap::TraversabilityParams p;
  topmap::TriangleNode a, b;
  a.normal = Eigen::Vector3f::UnitZ();
  b.normal = Eigen::Vector3f::UnitZ();
  a.centroid = Eigen::Vector3f(0, 0, 0);
  b.centroid = Eigen::Vector3f(0.1f, 0, 0.10f);
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isEdgeTraversable(a, b, p));
  b.centroid.z() = 0.0f;
  b.normal = Eigen::Vector3f(0, std::sin(0.7f), std::cos(0.7f));
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isEdgeTraversable(a, b, p));
}

TEST(TraversabilityTest, HeightJumpTakesWorstPlaneDistance) {
  topmap::TriangleNode a = MakeTriangle(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f::UnitZ(), 0.0f, 0.01f);
  topmap::TriangleNode b = MakeTriangle(Eigen::Vector3f(0.1f, 0, 0.10f), Eigen::Vector3f::UnitZ(), 0.0f, 0.01f);
  EXPECT_NEAR(topmap::TraversabilityEvaluator::heightJump(a, b), 0.10f, 1e-6f);
  EXPECT_NEAR(topmap::TraversabilityEvaluator::heightJump(a, a), 0.0f, 1e-6f);
}

TEST(TraversabilityTest, ClearanceDecaysFromMeshBoundary) {
  // R8: on a fully traversable plane clearance must grow from the boundary
  // inwards and stay finite. If the boundary source were omitted the interior
  // clearance would be infinite.
  auto d = GridMesh(60, 60, 0.10f);
  auto mr = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(mr.ok()) << mr.status().message;
  topmap::TriangleMesh m = mr.take();
  topmap::TraversabilityEvaluator::apply(m, topmap::TraversabilityParams{});

  float max_c = 0.0f, min_c = 1e9f;
  for (const auto& tri : m.triangles()) {
    max_c = std::max(max_c, tri.min_clearance);
    min_c = std::min(min_c, tri.min_clearance);
  }
  EXPECT_LT(min_c, 0.2f);    // boundary adjacent triangles are close to the edge
  EXPECT_GT(max_c, 1.0f);    // the centre is far from the boundary
  EXPECT_LT(max_c, 100.0f);  // ... but strictly finite
}

TEST(TraversabilityTest, NonTraversableTrianglesActAsClearanceSources) {
  // R8: a non-traversable obstacle in the middle of a plane must pull clearance
  // down locally, i.e. it is a geodesic source just like the mesh boundary.
  auto d = GridMesh(60, 60, 0.10f);
  d.vertices[static_cast<std::size_t>(30 * 61 + 30)].z() = 1.0f;  // sharp spike
  auto mr = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(mr.ok()) << mr.status().message;
  topmap::TriangleMesh m = mr.take();
  topmap::TraversabilityEvaluator::apply(m, topmap::TraversabilityParams{});

  int obstacle_count = 0;
  float near_obstacle = 1e9f;   // on a traversable triangle adjacent to the spike
  float far_from_all = 0.0f;    // a traversable triangle far from spike and boundary
  for (const auto& tri : m.triangles()) {
    if (!tri.traversable) {
      ++obstacle_count;
      EXPECT_FLOAT_EQ(tri.min_clearance, 0.0f);
    }
  }
  EXPECT_GT(obstacle_count, 0);
  // The spike sits at (3,3); the mesh boundary is ~3 m away, so any triangle
  // whose clearance is small can only be explained by the obstacle source.
  for (const auto& tri : m.triangles()) {
    if (!tri.traversable) continue;
    const float dc = (tri.centroid - Eigen::Vector3f(3.0f, 3.0f, 0.0f)).norm();
    if (dc < 0.4f) near_obstacle = std::min(near_obstacle, tri.min_clearance);
    if (dc > 1.5f) far_from_all = std::max(far_from_all, tri.min_clearance);
  }
  EXPECT_LT(near_obstacle, 0.5f);
  EXPECT_GT(far_from_all, 0.5f);
}

TEST(TraversabilityTest, FlatToRampTransitionStaysTraversable) {
  // R1-b: a 12 degree ramp meets a flat platform -> traversable.
  topmap::TraversabilityParams p;
  topmap::TriangleNode flat = MakeTriangle(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f::UnitZ(), 0.0f, 0.01f);
  const float a = 12.0f * kPi / 180.0f;
  topmap::TriangleNode ramp =
      MakeTriangle(Eigen::Vector3f(0.1f, 0, 0.0f), Eigen::Vector3f(std::sin(a), 0, std::cos(a)), a, 0.01f);
  EXPECT_TRUE(topmap::TraversabilityEvaluator::isEdgeTraversable(flat, ramp, p));
}

TEST(TraversabilityTest, SlopeTransitionExemptionOnlyWhenEnabled) {
  // R1-b: slope difference above max_slope_diff_deg(15) is still accepted when
  // the normal difference and step are legal, but only if allow_slope_transition.
  const float a = 17.0f * kPi / 180.0f;
  topmap::TriangleNode flat = MakeTriangle(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f::UnitZ(), 0.0f, 0.01f);
  topmap::TriangleNode ramp =
      MakeTriangle(Eigen::Vector3f(0.1f, 0, 0.0f), Eigen::Vector3f(std::sin(a), 0, std::cos(a)), a, 0.01f);

  topmap::TraversabilityParams on;
  on.allow_slope_transition = true;
  EXPECT_TRUE(topmap::TraversabilityEvaluator::isEdgeTraversable(flat, ramp, on));

  topmap::TraversabilityParams off;
  off.allow_slope_transition = false;
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isEdgeTraversable(flat, ramp, off));
}

TEST(TraversabilityTest, ClearanceIsASeparateGateFromSlope) {
  // R10: geometrically standable but the footprint does not fit.
  topmap::TraversabilityParams p;
  topmap::TriangleNode t = MakeTriangle(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f::UnitZ(), 0.0f, 0.01f);
  t.roughness = 0.0f;
  t.curvature = 0.0f;
  t.min_clearance = 0.20f;  // < required_clearance 0.45
  EXPECT_TRUE(topmap::TraversabilityEvaluator::isTriangleTraversable(t, p));
  EXPECT_FALSE(topmap::TraversabilityEvaluator::hasRequiredClearance(t, p));
  EXPECT_FALSE(topmap::TraversabilityEvaluator::isRobotPlaceable(t, p));
  t.min_clearance = 0.60f;
  EXPECT_TRUE(topmap::TraversabilityEvaluator::isRobotPlaceable(t, p));
}

TEST(TraversabilityTest, ApplyFillsTraversableAndEdgeFlags) {
  // A plane where the right half is a steep ramp: geometry is standable only on
  // the flat part, and the crossing edge must be flagged non-traversable.
  topmap::MeshData d;
  d.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {2, 0, 1.0f}, {2, 1, 1.0f}};
  d.faces = {{0, 1, 2}, {1, 3, 4}, {1, 4, 2}};  // last two are very steep
  auto mr = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(mr.ok()) << mr.status().message;
  topmap::TriangleMesh m = mr.take();
  topmap::TraversabilityEvaluator::apply(m, topmap::TraversabilityParams{});

  EXPECT_TRUE(m.triangle(0).traversable);
  EXPECT_FALSE(m.triangle(1).traversable);
  for (const auto& t : m.triangles()) {
    for (const auto& e : t.edges) {
      if (e.neighbor == topmap::kInvalidId) continue;
      EXPECT_EQ(e.traversable, t.traversable && m.triangle(e.neighbor).traversable);
    }
  }
}

TEST(TraversabilityTest, RegionBoundaryDistanceIsRelativeToTheRegion) {
  auto d = GridMesh(40, 40, 0.10f);  // 4 x 4 m plane
  auto mr = topmap::TriangleMesh::fromMeshData(d, {});
  ASSERT_TRUE(mr.ok()) << mr.status().message;
  topmap::TriangleMesh m = mr.take();
  topmap::TraversabilityEvaluator::apply(m, topmap::TraversabilityParams{});

  // Region = a 1 x 1 m block in the middle, far from the mesh boundary.
  std::vector<topmap::TriangleId> region;
  for (const auto& t : m.triangles()) {
    if (t.centroid.x() > 1.5f && t.centroid.x() < 2.5f && t.centroid.y() > 1.5f &&
        t.centroid.y() < 2.5f) {
      region.push_back(t.id);
    }
  }
  ASSERT_FALSE(region.empty());
  std::vector<float> rbd = topmap::regionBoundaryDistance(m, region);
  ASSERT_EQ(rbd.size(), m.size());

  float max_rbd = 0.0f, max_clearance = 0.0f;
  int border = 0;
  for (topmap::TriangleId id : region) {
    EXPECT_LE(rbd[id], m.triangle(id).min_clearance + 1e-4f);
    max_rbd = std::max(max_rbd, rbd[id]);
    max_clearance = std::max(max_clearance, m.triangle(id).min_clearance);
    if (rbd[id] < 0.2f) ++border;
  }
  EXPECT_GT(border, 0);                 // the region boundary is a real source
  EXPECT_GT(max_rbd, 0.4f);             // the region centre is far from that boundary
  EXPECT_LT(max_rbd, max_clearance);    // and closer than the mesh boundary
  for (topmap::TriangleId id = 0; id < static_cast<topmap::TriangleId>(m.size()); ++id) {
    if (std::find(region.begin(), region.end(), id) == region.end()) {
      EXPECT_FLOAT_EQ(rbd[id], 0.0f);
    }
  }
}
