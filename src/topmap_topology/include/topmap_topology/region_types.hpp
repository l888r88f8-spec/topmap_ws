#pragma once

#include <vector>

#include <Eigen/Core>

#include "topmap_core/ids.hpp"

namespace topmap {

// A4.7 SurfaceRegion: a connected, traversability-homogeneous patch of the
// triangle mesh. Portal fields are populated by T15; they are declared here to
// match the frozen interface but stay empty until then.
struct SurfaceRegion {
  RegionId id = kInvalidId;
  std::vector<TriangleId> triangle_ids;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  float area = 0.0f, min_z = 0.0f, max_z = 0.0f, mean_slope = 0.0f, max_slope = 0.0f,
        slope_variance = 0.0f, roughness = 0.0f;
  bool is_ramp = false, is_topology_critical = false;
  std::vector<PortalId> portal_ids;
};

}  // namespace topmap
