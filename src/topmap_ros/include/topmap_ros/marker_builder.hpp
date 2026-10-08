#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include "topmap_planner/plan_types.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_topology/region_types.hpp"
#include "topmap_topology/topology_graph.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

// T29: RViz visualization for the A4.6-A4.8 structures. Every builder is pure
// (no ROS graph, no node handle), so it is directly unit-testable. All produced
// markers share the same conventions: header.frame_id/stamp are taken from the
// arguments, ns starts with "topmap/", action is ADD, frame_locked is false,
// lifetime is {0, 0}, pose.orientation.w is 1, and colours/scales/alpha are set.
// Invalid or out-of-range ids are skipped rather than dereferenced, so a
// malformed graph can never crash the visualizer.
namespace topmap_ros {

// One TRIANGLE_LIST marker per valid region, built from the region's actual mesh
// triangles (three points per triangle). The colour is a deterministic HSV value
// derived from the region id. Regions whose id is negative, that own no valid
// triangle, or whose triangles reference missing vertices produce no marker.
visualization_msgs::msg::MarkerArray buildRegionMarkers(
    const topmap::TriangleMesh& mesh,
    const std::vector<topmap::SurfaceRegion>& regions,
    const std::string& frame_id,
    const rclcpp::Time& stamp);
visualization_msgs::msg::MarkerArray buildRegionMarkers(
    const topmap::TriangleMesh& mesh,
    const topmap::TopologyGraph& graph,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

// One red LINE_LIST marker per valid portal, spanning its left and right edge
// points. Portals with a negative id produce no marker.
visualization_msgs::msg::MarkerArray buildPortalMarkers(
    const std::vector<topmap::Portal>& portals,
    const std::string& frame_id,
    const rclcpp::Time& stamp);
visualization_msgs::msg::MarkerArray buildPortalMarkers(
    const topmap::TopologyGraph& graph,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

// One TRIANGLE_LIST marker covering the corridor's mesh triangles.
visualization_msgs::msg::MarkerArray buildCorridorMarkers(
    const topmap::TriangleMesh& mesh,
    const topmap::Corridor& corridor,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

// The ordered topology path: a LINE_STRIP through the region centroids and a
// SPHERE_LIST at the portal centers. Unknown ids are skipped, so a path that
// references a region/portal absent from the graph still renders the known
// prefix safely.
visualization_msgs::msg::MarkerArray buildTopologyPathMarkers(
    const topmap::TopologyGraph& graph,
    const topmap::TopologyPath& path,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

// Two SPHERE markers: the green start and the red goal. ids 0 and 1 are stable.
visualization_msgs::msg::MarkerArray buildStartGoalMarkers(
    const Eigen::Vector3f& start,
    const Eigen::Vector3f& goal,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

}  // namespace topmap_ros
