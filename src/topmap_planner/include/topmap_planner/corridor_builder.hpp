#pragma once

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap {

// T23: turn a topology path into the surface corridor that Surface A* may
// search. `regions` is exactly the path's Regions (sorted and deduped) - never
// the expansion neighbours; `portals` is the path's Portal set; `triangles` is
// the union of the path Regions' triangles plus a kRing neighbourhood grown
// from every Portal center. Expansion never crosses mesh components.
class CorridorBuilder {
 public:
  Result<Corridor> build(const TriangleMesh& mesh, const TopologyGraph& graph,
                         const TopologyPath& path, int expansion_rings) const;
};

}  // namespace topmap
