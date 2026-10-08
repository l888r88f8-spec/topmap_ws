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

// A4.7 Portal: the candidate connection between two adjacent Regions. T15 fills
// the boundary chain and its raw geometry; T16 fills the robot-aware fields
// (valid_width, mean_slope, max_step, min_clearance, traversal_cost).
struct Portal {
  PortalId id = kInvalidId;
  RegionId region_a = kInvalidId, region_b = kInvalidId;
  std::vector<PointId> boundary_chain;
  Eigen::Vector3f left = Eigen::Vector3f::Zero(), right = Eigen::Vector3f::Zero(),
                  center = Eigen::Vector3f::Zero();
  float raw_width = 0.0f, valid_width = 0.0f, mean_slope = 0.0f, max_step = 0.0f,
        min_clearance = 0.0f, traversal_cost = 0.0f;
  bool is_critical = false;
};

}  // namespace topmap
