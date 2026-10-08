#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include "topmap_core/error.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_topology/map_archive.hpp"
#include "topmap_topology/region_types.hpp"
#include "topmap_topology/topology_graph.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "topmap_msgs/msg/map_stats.hpp"
#include "topmap_msgs/msg/plan_stats.hpp"
#include "topmap_msgs/msg/portal.hpp"
#include "topmap_msgs/msg/region.hpp"
#include "topmap_msgs/msg/topology_graph.hpp"
#include "topmap_msgs/msg/topology_path.hpp"

// T29: the single translation layer between the ROS-free topmap algorithm types
// (A4.6-A4.8) and the frozen A4.9 message contracts. Nothing here reads ROS
// parameters or touches the graph, so every function is unit-testable without
// rclcpp::init().
namespace topmap_ros {

// ---------------------------------------------------------------------------
// geometry_msgs::Point
// ---------------------------------------------------------------------------
geometry_msgs::msg::Point toMsg(const Eigen::Vector3f& point);
Eigen::Vector3f fromMsg(const geometry_msgs::msg::Point& point);

// ---------------------------------------------------------------------------
// SurfaceRegion <-> topmap_msgs::msg::Region
// ---------------------------------------------------------------------------
// The message has no triangle_ids field, so a conversion round trip cannot
// retain SurfaceRegion::triangle_ids.
topmap_msgs::msg::Region toMsg(const topmap::SurfaceRegion& region);
topmap::SurfaceRegion fromMsg(const topmap_msgs::msg::Region& msg);

// ---------------------------------------------------------------------------
// Portal <-> topmap_msgs::msg::Portal
// ---------------------------------------------------------------------------
// R14/A4.9: the message has no boundary_chain field, so a wire round trip cannot
// retain Portal::boundary_chain. Every other field maps one to one.
topmap_msgs::msg::Portal toMsg(const topmap::Portal& portal);
topmap::Portal fromMsg(const topmap_msgs::msg::Portal& msg);

// ---------------------------------------------------------------------------
// TopologyGraph <-> topmap_msgs::msg::TopologyGraph
// ---------------------------------------------------------------------------
// toMsg emits regions and portals sorted ascending by id (deterministic,
// independent of the graph's internal storage order) and a flat incidence edge
// list: for every portal one (region_a, portal_id) and one (region_b, portal_id)
// pair, sorted by (region_id, portal_id).
//
// fromMsg rebuilds the graph through TopologyGraph::build (so duplicate ids,
// missing regions, self loops and invalid triangle ownership are rejected) and
// then checks that edge_region_ids/edge_portal_ids has equal length and encodes
// exactly the incidence list implied by the converted portals. Any mismatch is
// a kInvalidArgument Result.
topmap_msgs::msg::TopologyGraph toMsg(const topmap::TopologyGraph& graph,
                                      const std::string& frame_id = "map",
                                      const rclcpp::Time& stamp = rclcpp::Time(0));
topmap::Result<topmap::TopologyGraph> fromMsg(const topmap_msgs::msg::TopologyGraph& msg);

// ---------------------------------------------------------------------------
// TopologyPath <-> topmap_msgs::msg::TopologyPath
// ---------------------------------------------------------------------------
topmap_msgs::msg::TopologyPath toMsg(const topmap::TopologyPath& path);
topmap::TopologyPath fromMsg(const topmap_msgs::msg::TopologyPath& msg);

// ---------------------------------------------------------------------------
// PlanStats <-> topmap_msgs::msg::PlanStats
// ---------------------------------------------------------------------------
// The C++ counters are signed while the message counters are uint32. toMsg maps
// negative counters to 0; fromMsg saturates values above INT32_MAX to INT32_MAX
// so no out-of-range narrowing is ever performed.
topmap_msgs::msg::PlanStats toMsg(const topmap::PlanStats& stats);
topmap::PlanStats fromMsg(const topmap_msgs::msg::PlanStats& msg);

// ---------------------------------------------------------------------------
// MapMetadata -> topmap_msgs::msg::MapStats
// ---------------------------------------------------------------------------
// Maps every field that exists in the current MapStats.msg. format_version comes
// from kTopmapFormatVersion because MapMetadata does not carry it;
// MapMetadata::regions_before_compression has no message counterpart and is not
// serialized here.
topmap_msgs::msg::MapStats toMsg(const topmap::MapMetadata& metadata);

// ---------------------------------------------------------------------------
// std::vector<Eigen::Vector3f> -> nav_msgs::msg::Path
// ---------------------------------------------------------------------------
// Every pose gets the same frame id and stamp plus an identity orientation
// (w = 1) so RViz renders the path without quaternion warnings.
nav_msgs::msg::Path toMsg(const std::vector<Eigen::Vector3f>& points,
                          const std::string& frame_id,
                          const rclcpp::Time& stamp);

}  // namespace topmap_ros
