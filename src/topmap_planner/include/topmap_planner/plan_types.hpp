#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"

namespace topmap {

// A4.8: the frozen planner-facing types. Every planner stage shares these plain
// aggregates so the hierarchical planner (T27) can fill them field by field and
// the ROS layer (T29) can convert them without touching the algorithms.
struct PlanRequest {
  Eigen::Vector3f start = Eigen::Vector3f::Zero(), goal = Eigen::Vector3f::Zero();
};

// Strictly alternating Region sequence and Portal sequence:
// regions.size() == portals.size() + 1. `cost` is the accumulated topology cost
// reported by TopologyAStar.
struct TopologyPath {
  std::vector<RegionId> regions;
  std::vector<PortalId> portals;
  float cost = 0.0f;
};

struct PlanStats {
  int regions_in_corridor = 0, triangles_in_corridor = 0,
      topology_expanded_nodes = 0, surface_expanded_nodes = 0;
  double topology_time_sec = 0.0, surface_time_sec = 0.0, optimization_time_sec = 0.0;
  float path_length = 0.0f, min_clearance = 0.0f, max_slope_deg = 0.0f, max_curvature = 0.0f;
  bool fallback_used = false;
};

struct PlanResult {
  Status status;
  std::vector<Eigen::Vector3f> path;
  TopologyPath topology;
  PlanStats stats;
};

struct PathValidationReport {
  bool valid = false;
  float min_clearance = 0.0f, max_step = 0.0f, max_slope_deg = 0.0f, max_curvature = 0.0f;
  std::vector<std::size_t> violating_indices;
};

struct Corridor {
  std::vector<RegionId> regions;
  std::vector<TriangleId> triangles;
  std::vector<PortalId> portals;
};

}  // namespace topmap
