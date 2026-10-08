#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"
#include "topmap_core/params.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_topology/portal_extractor.hpp"
#include "topmap_topology/portal_validator.hpp"
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"
#include "topmap_topology/region_types.hpp"
#include "topmap_topology/topology_graph.hpp"

// R27: shared planner test fixture. It runs the real T12-T17 pipeline so the
// planner tests exercise the same surface and topology the production map
// archive holds. It lives in test/ and is never installed.
namespace topmap {
namespace test {

struct Prepared {
  TriangleMesh mesh;
  TopologyGraph graph;
};

// True when at least one mesh edge joins two different Regions. A scene with a
// single Region per component (flat_plane, overlapping_floors) legitimately has
// no such edge and therefore no Portal.
inline bool HasCrossRegionSharedEdge(const TriangleMesh& mesh,
                                     const std::vector<SurfaceRegion>& regions) {
  std::vector<RegionId> owner(static_cast<std::size_t>(mesh.size()), kInvalidId);
  for (const SurfaceRegion& region : regions) {
    for (TriangleId t : region.triangle_ids) {
      if (t >= 0 && t < static_cast<TriangleId>(mesh.size())) owner[static_cast<std::size_t>(t)] = region.id;
    }
  }
  for (TriangleId t = 0; t < static_cast<TriangleId>(mesh.size()); ++t) {
    if (owner[static_cast<std::size_t>(t)] == kInvalidId) continue;
    for (const TriangleEdge& edge : mesh.triangle(t).edges) {
      if (edge.neighbor == kInvalidId || edge.neighbor < t) continue;
      const RegionId other = owner[static_cast<std::size_t>(edge.neighbor)];
      if (other != kInvalidId && other != owner[static_cast<std::size_t>(t)]) return true;
    }
  }
  return false;
}

// Runs makeSyntheticMesh -> TriangleMesh -> traversability -> grow -> split ->
// extract -> validate -> TopologyGraph. Every Stage result is checked: the only
// failure tolerated is PortalExtractor's kPortalExtractionFailed when the
// partition genuinely has no cross-Region shared edge, in which case the graph
// is built with an empty Portal set. Any other error (including a
// kPortalExtractionFailed that does hide shared edges) is propagated.
inline Result<Prepared> Build(SyntheticScene scene, const TopmapConfig& config = TopmapConfig{}) {
  auto data = makeSyntheticMesh(scene);
  if (!data.ok()) return data.status();

  auto mesh_result = TriangleMesh::fromMeshData(data.value(), {});
  if (!mesh_result.ok()) return mesh_result.status();
  TriangleMesh mesh = mesh_result.take();

  TraversabilityEvaluator::apply(mesh, config.traversability);

  auto grown = RegionGrower().grow(mesh, config.region);
  if (!grown.ok()) return grown.status();

  auto split = RegionSplitter().split(mesh, grown.value(), config.region);
  if (!split.ok()) return split.status();
  std::vector<SurfaceRegion> regions = split.take();

  std::vector<Portal> portals;
  auto extracted = PortalExtractor().extract(mesh, regions, config.portal);
  if (extracted.ok()) {
    portals = extracted.take();
  } else if (extracted.status().code == ErrorCode::kPortalExtractionFailed &&
             !HasCrossRegionSharedEdge(mesh, regions)) {
    portals.clear();  // Legitimate: regions exist but share no inter-Region edge.
  } else {
    return extracted.status();
  }

  if (!portals.empty()) {
    auto validated = PortalValidator().validate(mesh, regions, portals, config.robot, config.portal);
    if (!validated.ok()) return validated.status();
    portals = validated.take();
  }

  auto graph = TopologyGraph::build(regions, portals);
  if (!graph.ok()) return graph.status();

  Prepared prepared;
  prepared.mesh = std::move(mesh);
  prepared.graph = graph.take();
  return prepared;
}

inline std::vector<TriangleId> AllTriangleIds(const TriangleMesh& mesh) {
  std::vector<TriangleId> ids(mesh.size());
  for (std::size_t i = 0; i < mesh.size(); ++i) ids[i] = static_cast<TriangleId>(i);
  return ids;
}

inline std::vector<TriangleId> ComponentTriangleIds(const TriangleMesh& mesh, int component) {
  std::vector<TriangleId> ids;
  for (std::size_t i = 0; i < mesh.size(); ++i) {
    if (mesh.componentId(static_cast<TriangleId>(i)) == component) {
      ids.push_back(static_cast<TriangleId>(i));
    }
  }
  return ids;
}

}  // namespace test
}  // namespace topmap
