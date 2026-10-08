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
using topmap::TriangleId;
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

// The shared-edge graph of a Region pair is not always a simple path: a
// checkerboard assignment of two Regions makes every interior interface vertex
// degree 4. The extractor must decompose such a graph into simple chains whose
// consecutive vertices are real shared edges of that pair and whose per-chain
// vertex degree never exceeds 2 - a backtracking walk that dumps every branch
// tip into one vertex list would place non-adjacent tips next to each other.
TEST(PortalExtractorTest, BranchingInterfaceIsSplitIntoValidPaths) {
  Grid g = MakeGrid(6, 6, 0.1f);
  SurfaceRegion a;
  a.id = 0;
  SurfaceRegion b;
  b.id = 1;
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < 6; ++i) {
      const int t = CellTri0(g, i, j);
      SurfaceRegion& owner = ((i + j) % 2 == 0) ? a : b;
      owner.triangle_ids.push_back(t);
      owner.triangle_ids.push_back(t + 1);
    }
  }

  // Shared edges of the (0, 1) interface, collected independently.
  std::set<std::pair<PointId, PointId>> shared;
  for (TriangleId t = 0; t < static_cast<TriangleId>(g.mesh.size()); ++t) {
    for (const topmap::TriangleEdge& e : g.mesh.triangle(t).edges) {
      const TriangleId u = e.neighbor;
      if (u == topmap::kInvalidId || u < t) continue;
      const bool t_in_a = std::find(a.triangle_ids.begin(), a.triangle_ids.end(), t) != a.triangle_ids.end();
      const bool t_in_b = std::find(b.triangle_ids.begin(), b.triangle_ids.end(), t) != b.triangle_ids.end();
      const bool u_in_a = std::find(a.triangle_ids.begin(), a.triangle_ids.end(), u) != a.triangle_ids.end();
      const bool u_in_b = std::find(b.triangle_ids.begin(), b.triangle_ids.end(), u) != b.triangle_ids.end();
      if ((t_in_a && u_in_b) || (t_in_b && u_in_a)) {
        shared.insert({std::min(e.v0, e.v1), std::max(e.v0, e.v1)});
      }
    }
  }
  ASSERT_FALSE(shared.empty());

  auto r = PortalExtractor().extract(g.mesh, {a, b}, PortalParams{});
  ASSERT_TRUE(r.ok()) << r.status().message;
  ASSERT_GT(r.value().size(), 0u);

  for (const Portal& portal : r.value()) {
    ASSERT_GE(portal.boundary_chain.size(), 2u);
    std::map<PointId, int> degree;
    float arc_length = 0.0f;
    for (std::size_t k = 1; k < portal.boundary_chain.size(); ++k) {
      const PointId from = portal.boundary_chain[k - 1];
      const PointId to = portal.boundary_chain[k];
      // Adjacent chain vertices must be connected by an actual shared mesh edge.
      EXPECT_EQ(shared.count({std::min(from, to), std::max(from, to)}), 1u)
          << "consecutive chain vertices " << from << " and " << to << " are not a shared edge";
      ++degree[from];
      ++degree[to];
      arc_length += (g.mesh.vertices()[to] - g.mesh.vertices()[from]).norm();
    }
    for (const auto& entry : degree) {
      EXPECT_LE(entry.second, 2) << "chain vertex " << entry.first << " has degree > 2";
    }
    EXPECT_NEAR(portal.raw_width, arc_length, 1e-3f);
  }

  // A second extraction over the same graph must be byte-for-byte reproducible.
  auto again = PortalExtractor().extract(g.mesh, {a, b}, PortalParams{});
  ASSERT_TRUE(again.ok()) << again.status().message;
  ASSERT_EQ(again.value().size(), r.value().size());
  for (std::size_t i = 0; i < r.value().size(); ++i) {
    EXPECT_EQ(r.value()[i].boundary_chain, again.value()[i].boundary_chain);
    EXPECT_FLOAT_EQ(r.value()[i].raw_width, again.value()[i].raw_width);
  }
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
