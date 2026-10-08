#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
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
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"

namespace {

using topmap::MeshData;
using topmap::PointId;
using topmap::Portal;
using topmap::PortalExtractor;
using topmap::PortalParams;
using topmap::RegionParams;
using topmap::SurfaceRegion;
using topmap::SyntheticScene;
using topmap::TriangleMesh;

// R19: grow/split take a const TriangleMesh&, so the Result must be taken out
// into an entity before it is passed on.
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

// A welded regular lattice of right triangles. Cell (i, j) owns the two
// triangles CellTri0(i, j) and CellTri0(i, j) + 1, matching the synthetic scene
// triangulation so shared cell walls become shared mesh edges.
struct Grid {
  TriangleMesh mesh;
  int nx = 0;
  int ny = 0;
  float step = 0.1f;
};

Grid MakeGrid(int nx, int ny, float step) {
  MeshData data;
  std::map<std::pair<int, int>, PointId> welded;
  const auto V = [&](int i, int j) -> PointId {
    const std::pair<int, int> key{i, j};
    const auto it = welded.find(key);
    if (it != welded.end()) return it->second;
    const PointId id = static_cast<PointId>(data.vertices.size());
    data.vertices.emplace_back(i * step, j * step, 0.0f);
    welded.emplace(key, id);
    return id;
  };
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const PointId a = V(i, j);
      const PointId b = V(i + 1, j);
      const PointId c = V(i + 1, j + 1);
      const PointId d = V(i, j + 1);
      data.faces.push_back({a, b, c});
      data.faces.push_back({a, c, d});
    }
  }
  auto mr = TriangleMesh::fromMeshData(data, {});
  EXPECT_TRUE(mr.ok()) << mr.status().message;
  Grid g;
  g.mesh = mr.take();
  g.nx = nx;
  g.ny = ny;
  g.step = step;
  return g;
}

int CellTri0(const Grid& g, int i, int j) { return 2 * (j * g.nx + i); }

// The R15 frozen Portal counts.
constexpr std::size_t kHallPortals = 2;
constexpr std::size_t kBridgePortals = 2;
constexpr std::size_t kParallelPortals = 4;
constexpr std::size_t kLoopPortals = 8;

}  // namespace

TEST(PortalExtractorTest, HallNarrowHallHasTwoPortals) {
  const Prepared p = Split(SyntheticScene::kHallNarrowHall, RegionParams{});
  ASSERT_TRUE(p.ok) << p.error;
  auto r = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), kHallPortals);
  for (const Portal& portal : r.value()) {
    EXPECT_GE(portal.boundary_chain.size(), 2u);
    EXPECT_NE(portal.region_a, portal.region_b);
    EXPECT_GE(portal.region_a, 0);
    EXPECT_GE(portal.region_b, 0);

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    float chain_length = 0.0f;
    for (std::size_t k = 0; k < portal.boundary_chain.size(); ++k) {
      const PointId v = portal.boundary_chain[k];
      ASSERT_GE(v, 0);
      ASSERT_LT(v, static_cast<PointId>(p.mesh.vertices().size()));
      mean += p.mesh.vertices()[v];
      if (k > 0) {
        chain_length += (p.mesh.vertices()[portal.boundary_chain[k]] -
                         p.mesh.vertices()[portal.boundary_chain[k - 1]])
                            .norm();
      }
    }
    mean /= static_cast<float>(portal.boundary_chain.size());
    EXPECT_TRUE(portal.center.isApprox(mean, 1e-4f));
    // left/right are the polygon endpoints of the chain.
    EXPECT_TRUE(portal.left.isApprox(p.mesh.vertices()[portal.boundary_chain.front()], 1e-4f));
    EXPECT_TRUE(portal.right.isApprox(p.mesh.vertices()[portal.boundary_chain.back()], 1e-4f));
    // raw_width is the chain arc length (basic geometry). It is explicitly NOT
    // claimed to be the traversal width - T16 recomputes that from the mesh.
    EXPECT_NEAR(portal.raw_width, chain_length, 1e-3f);
    EXPECT_GT(portal.raw_width, 0.0f);
  }
}

TEST(PortalExtractorTest, BridgeHasTwoPortals) {
  const Prepared p = Split(SyntheticScene::kSingleBridge, RegionParams{});
  ASSERT_TRUE(p.ok) << p.error;
  auto r = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), kBridgePortals);
}

TEST(PortalExtractorTest, ParallelPortalsAreNotMerged) {
  const Prepared p = Split(SyntheticScene::kParallelPortals, RegionParams{});
  ASSERT_TRUE(p.ok) << p.error;
  auto r = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  ASSERT_TRUE(r.ok()) << r.status().message;
  // R15: two rooms + two corridors = four regions, each corridor has two ends.
  EXPECT_EQ(r.value().size(), kParallelPortals);

  std::set<std::pair<int, int>> pairs;
  for (const Portal& portal : r.value()) {
    const int a = std::min(portal.region_a, portal.region_b);
    const int b = std::max(portal.region_a, portal.region_b);
    pairs.insert({a, b});
    EXPECT_GE(portal.boundary_chain.size(), 2u);
  }
  // The two parallel channels are four distinct region pairs, so nothing may be
  // collapsed into a single connection.
  EXPECT_EQ(pairs.size(), kParallelPortals);
}

TEST(PortalExtractorTest, LoopHasEightPortals) {
  const Prepared p = Split(SyntheticScene::kLoop, RegionParams{});
  ASSERT_TRUE(p.ok) << p.error;
  auto r = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  ASSERT_TRUE(r.ok()) << r.status().message;
  EXPECT_EQ(r.value().size(), kLoopPortals);
}

TEST(PortalExtractorTest, BrokenChainKeepsIndependentPortalsPerRegionPair) {
  // Two rows (j = 4, 5) are deliberately left out of both regions, so the
  // shared-edge set between region 0 and region 1 breaks into two separate
  // chains along x = 0.5. They must stay two portals, not be merged.
  Grid g = MakeGrid(10, 10, 0.1f);
  SurfaceRegion a;
  a.id = 0;
  SurfaceRegion b;
  b.id = 1;
  for (int j = 0; j < 10; ++j) {
    if (j == 4 || j == 5) continue;
    for (int i = 0; i < 10; ++i) {
      const int t = CellTri0(g, i, j);
      if (i < 5) {
        a.triangle_ids.push_back(t);
        a.triangle_ids.push_back(t + 1);
      } else {
        b.triangle_ids.push_back(t);
        b.triangle_ids.push_back(t + 1);
      }
    }
  }
  auto r = PortalExtractor().extract(g.mesh, {a, b}, PortalParams{});
  ASSERT_TRUE(r.ok()) << r.status().message;
  ASSERT_EQ(r.value().size(), 2u);
  EXPECT_EQ(r.value()[0].region_a, r.value()[1].region_a);
  EXPECT_EQ(r.value()[0].region_b, r.value()[1].region_b);
  EXPECT_NE(r.value()[0].boundary_chain.front(), r.value()[1].boundary_chain.front());
  EXPECT_NEAR(r.value()[0].raw_width, 0.4f, 1e-3f);
  EXPECT_NEAR(r.value()[1].raw_width, 0.4f, 1e-3f);
}

TEST(PortalExtractorTest, ShortChainsAreFilteredByBoundaryMinEdges) {
  Grid g = MakeGrid(2, 1, 0.1f);
  SurfaceRegion a;
  a.id = 0;
  SurfaceRegion b;
  b.id = 1;
  const int t0 = CellTri0(g, 0, 0);
  const int t1 = CellTri0(g, 1, 0);
  a.triangle_ids = {t0, t0 + 1};
  b.triangle_ids = {t1, t1 + 1};

  PortalParams strict;
  strict.boundary_min_edges = 2;  // one shared edge -> discarded
  auto filtered = PortalExtractor().extract(g.mesh, {a, b}, strict);
  EXPECT_FALSE(filtered.ok());
  EXPECT_EQ(filtered.status().code, topmap::ErrorCode::kPortalExtractionFailed);

  PortalParams relaxed;
  relaxed.boundary_min_edges = 1;
  auto kept = PortalExtractor().extract(g.mesh, {a, b}, relaxed);
  ASSERT_TRUE(kept.ok()) << kept.status().message;
  ASSERT_EQ(kept.value().size(), 1u);
  EXPECT_EQ(kept.value()[0].boundary_chain.size(), 2u);
  EXPECT_NEAR(kept.value()[0].raw_width, 0.1f, 1e-3f);
}

TEST(PortalExtractorTest, RejectsEmptyAndInvalidInputs) {
  const TriangleMesh empty;
  auto no_mesh = PortalExtractor().extract(empty, {}, PortalParams{});
  EXPECT_FALSE(no_mesh.ok());
  EXPECT_EQ(no_mesh.status().code, topmap::ErrorCode::kMeshEmpty);

  Grid g = MakeGrid(2, 1, 0.1f);
  auto no_regions = PortalExtractor().extract(g.mesh, {}, PortalParams{});
  EXPECT_FALSE(no_regions.ok());
  EXPECT_EQ(no_regions.status().code, topmap::ErrorCode::kRegionEmpty);

  SurfaceRegion out_of_range;
  out_of_range.id = 0;
  out_of_range.triangle_ids = {99999};
  auto bad_triangle = PortalExtractor().extract(g.mesh, {out_of_range}, PortalParams{});
  EXPECT_FALSE(bad_triangle.ok());
  EXPECT_EQ(bad_triangle.status().code, topmap::ErrorCode::kInvalidArgument);

  SurfaceRegion share_a;
  share_a.id = 0;
  share_a.triangle_ids = {0};
  SurfaceRegion share_b;
  share_b.id = 1;
  share_b.triangle_ids = {0};  // triangle owned twice
  auto overlapping = PortalExtractor().extract(g.mesh, {share_a, share_b}, PortalParams{});
  EXPECT_FALSE(overlapping.ok());
  EXPECT_EQ(overlapping.status().code, topmap::ErrorCode::kInvalidArgument);

  SurfaceRegion dup_a;
  dup_a.id = 0;
  dup_a.triangle_ids = {0};
  SurfaceRegion dup_b;
  dup_b.id = 0;  // duplicate RegionId
  dup_b.triangle_ids = {1};
  auto duplicate_ids = PortalExtractor().extract(g.mesh, {dup_a, dup_b}, PortalParams{});
  EXPECT_FALSE(duplicate_ids.ok());
  EXPECT_EQ(duplicate_ids.status().code, topmap::ErrorCode::kInvalidArgument);

  PortalParams bad_params;
  bad_params.boundary_min_edges = 0;
  auto bad_min_edges = PortalExtractor().extract(g.mesh, {share_a, share_b}, bad_params);
  EXPECT_FALSE(bad_min_edges.ok());
  EXPECT_EQ(bad_min_edges.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(PortalExtractorTest, NoSharedEdgesFails) {
  const Prepared p = Split(SyntheticScene::kFlatPlane, RegionParams{});
  ASSERT_TRUE(p.ok) << p.error;
  auto r = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status().code, topmap::ErrorCode::kPortalExtractionFailed);
}

TEST(PortalExtractorTest, ExtractionIsDeterministic) {
  const Prepared p = Split(SyntheticScene::kParallelPortals, RegionParams{});
  ASSERT_TRUE(p.ok) << p.error;
  auto first = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  auto second = PortalExtractor().extract(p.mesh, p.regions, PortalParams{});
  ASSERT_TRUE(first.ok()) << first.status().message;
  ASSERT_TRUE(second.ok()) << second.status().message;
  ASSERT_EQ(first.value().size(), second.value().size());
  for (std::size_t i = 0; i < first.value().size(); ++i) {
    EXPECT_EQ(first.value()[i].id, second.value()[i].id);
    EXPECT_EQ(first.value()[i].region_a, second.value()[i].region_a);
    EXPECT_EQ(first.value()[i].region_b, second.value()[i].region_b);
    EXPECT_EQ(first.value()[i].boundary_chain, second.value()[i].boundary_chain);
    EXPECT_FLOAT_EQ(first.value()[i].raw_width, second.value()[i].raw_width);
  }
}
