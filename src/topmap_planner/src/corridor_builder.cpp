#include "topmap_planner/corridor_builder.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/geometry.hpp"

namespace topmap {
namespace {

template <typename Id>
void SortUnique(std::vector<Id>& values) {
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
}

Status CorridorFailure(std::string message) {
  return Status::Error(ErrorCode::kCorridorBuildFailed, std::move(message));
}

}  // namespace

Result<Corridor> CorridorBuilder::build(const TriangleMesh& mesh, const TopologyGraph& graph,
                                        const TopologyPath& path, int expansion_rings) const {
  if (expansion_rings < 0) return CorridorFailure("expansion_rings must be non-negative");
  if (path.regions.empty()) return CorridorFailure("topology path has no regions");
  if (path.regions.size() != path.portals.size() + 1u) {
    return CorridorFailure("topology path regions and portals do not alternate");
  }
  for (const RegionId region_id : path.regions) {
    if (!graph.hasRegion(region_id)) {
      return CorridorFailure("topology path references a region that is not in the graph");
    }
  }
  for (std::size_t i = 0; i < path.portals.size(); ++i) {
    const Portal* portal = graph.portal(path.portals[i]);
    if (portal == nullptr) {
      return CorridorFailure("topology path references a portal that is not in the graph");
    }
    const RegionId a = path.regions[i];
    const RegionId b = path.regions[i + 1];
    const bool connects = (portal->region_a == a && portal->region_b == b) ||
                          (portal->region_a == b && portal->region_b == a);
    if (!connects) {
      return CorridorFailure("a portal does not connect its adjacent path regions");
    }
  }
  if (mesh.size() == 0) return CorridorFailure("cannot build a corridor on an empty mesh");
  if (graph.regions().empty()) {
    return CorridorFailure("cannot build a corridor on an empty topology graph");
  }

  // Start with every triangle of every Region on the path.
  std::vector<TriangleId> triangles;
  for (const RegionId region_id : path.regions) {
    const SurfaceRegion* region = graph.region(region_id);
    triangles.insert(triangles.end(), region->triangle_ids.begin(), region->triangle_ids.end());
  }

  // Then widen around each Portal: map its center to the nearest triangle that
  // belongs to one of the two endpoint Regions, and add that seed plus its
  // max(1, expansion_rings) ring neighbourhood.
  const int portal_rounds = std::max(1, expansion_rings);
  for (const PortalId portal_id : path.portals) {
    const Portal* portal = graph.portal(portal_id);
    const SurfaceRegion* region_a = graph.region(portal->region_a);
    const SurfaceRegion* region_b = graph.region(portal->region_b);

    TriangleId seed = kInvalidId;
    float seed_distance = 0.0f;
    const auto consider = [&](const SurfaceRegion* region) {
      if (region == nullptr) return;
      for (const TriangleId triangle_id : region->triangle_ids) {
        if (triangle_id < 0 || triangle_id >= static_cast<TriangleId>(mesh.size())) continue;
        if (!graph.regionOfTriangle(triangle_id).ok()) continue;
        const TriangleNode& triangle = mesh.triangle(triangle_id);
        const float distance =
            pointTriangleDistance(portal->center, mesh.vertices()[triangle.vertex_ids[0]],
                                  mesh.vertices()[triangle.vertex_ids[1]],
                                  mesh.vertices()[triangle.vertex_ids[2]]);
        // Deterministic: ties fall to the smallest TriangleId.
        if (seed == kInvalidId || distance < seed_distance ||
            (distance == seed_distance && triangle_id < seed)) {
          seed = triangle_id;
          seed_distance = distance;
        }
      }
    };
    consider(region_a);
    consider(region_b);
    if (seed == kInvalidId) {
      return CorridorFailure("portal center cannot be mapped to a mesh triangle");
    }

    const int seed_component = mesh.componentId(seed);
    triangles.push_back(seed);
    auto ring = mesh.kRing(seed, portal_rounds);
    if (!ring.ok()) return CorridorFailure("portal neighborhood expansion failed");
    for (const TriangleId triangle_id : ring.value()) {
      // kRing already stays inside one connected component; keep the guard
      // explicit so the corridor can never jump to an unrelated component.
      if (mesh.componentId(triangle_id) == seed_component) triangles.push_back(triangle_id);
    }
  }

  SortUnique(triangles);
  if (triangles.empty()) return CorridorFailure("corridor triangle union is empty");

  Corridor corridor;
  corridor.regions = path.regions;
  SortUnique(corridor.regions);
  corridor.portals = path.portals;
  SortUnique(corridor.portals);
  corridor.triangles = std::move(triangles);
  return corridor;
}

}  // namespace topmap
