#include "topmap_ros/type_conversions.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>

#include "topmap_core/binary_io.hpp"

namespace topmap_ros {
namespace {

// PlanStats counters are signed in C++ and uint32 on the wire. Clamp in both
// directions so a bad value can never turn into an out-of-range narrowing.
std::uint32_t checkedCounterToUint32(int value) {
  return value < 0 ? 0u : static_cast<std::uint32_t>(value);
}

int checkedCounterToInt(std::uint32_t value) {
  constexpr std::uint32_t kIntMax = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  return value > kIntMax ? std::numeric_limits<int>::max() : static_cast<int>(value);
}

}  // namespace

geometry_msgs::msg::Point toMsg(const Eigen::Vector3f& point) {
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

Eigen::Vector3f fromMsg(const geometry_msgs::msg::Point& point) {
  return Eigen::Vector3f(static_cast<float>(point.x), static_cast<float>(point.y),
                         static_cast<float>(point.z));
}

topmap_msgs::msg::Region toMsg(const topmap::SurfaceRegion& region) {
  topmap_msgs::msg::Region msg;
  msg.id = region.id;
  msg.centroid = toMsg(region.centroid);
  msg.area = region.area;
  msg.min_z = region.min_z;
  msg.max_z = region.max_z;
  msg.mean_slope = region.mean_slope;
  msg.max_slope = region.max_slope;
  msg.roughness = region.roughness;
  msg.is_ramp = region.is_ramp;
  msg.is_topology_critical = region.is_topology_critical;
  msg.portal_ids.assign(region.portal_ids.begin(), region.portal_ids.end());
  return msg;
}

topmap::SurfaceRegion fromMsg(const topmap_msgs::msg::Region& msg) {
  topmap::SurfaceRegion region;
  region.id = static_cast<topmap::RegionId>(msg.id);
  region.centroid = fromMsg(msg.centroid);
  region.area = msg.area;
  region.min_z = msg.min_z;
  region.max_z = msg.max_z;
  region.mean_slope = msg.mean_slope;
  region.max_slope = msg.max_slope;
  region.roughness = msg.roughness;
  region.is_ramp = msg.is_ramp;
  region.is_topology_critical = msg.is_topology_critical;
  region.portal_ids.assign(msg.portal_ids.begin(), msg.portal_ids.end());
  // Region.msg has no triangle_ids field; it is intentionally left empty.
  return region;
}

topmap_msgs::msg::Portal toMsg(const topmap::Portal& portal) {
  topmap_msgs::msg::Portal msg;
  msg.id = portal.id;
  msg.region_a = portal.region_a;
  msg.region_b = portal.region_b;
  msg.left = toMsg(portal.left);
  msg.right = toMsg(portal.right);
  msg.center = toMsg(portal.center);
  msg.raw_width = portal.raw_width;
  msg.valid_width = portal.valid_width;
  msg.mean_slope = portal.mean_slope;
  msg.max_step = portal.max_step;
  msg.min_clearance = portal.min_clearance;
  msg.traversal_cost = portal.traversal_cost;
  msg.is_critical = portal.is_critical;
  // Portal.msg has no boundary_chain field; the wire round trip cannot retain it.
  return msg;
}

topmap::Portal fromMsg(const topmap_msgs::msg::Portal& msg) {
  topmap::Portal portal;
  portal.id = static_cast<topmap::PortalId>(msg.id);
  portal.region_a = static_cast<topmap::RegionId>(msg.region_a);
  portal.region_b = static_cast<topmap::RegionId>(msg.region_b);
  portal.left = fromMsg(msg.left);
  portal.right = fromMsg(msg.right);
  portal.center = fromMsg(msg.center);
  portal.raw_width = msg.raw_width;
  portal.valid_width = msg.valid_width;
  portal.mean_slope = msg.mean_slope;
  portal.max_step = msg.max_step;
  portal.min_clearance = msg.min_clearance;
  portal.traversal_cost = msg.traversal_cost;
  portal.is_critical = msg.is_critical;
  return portal;
}

topmap_msgs::msg::TopologyGraph toMsg(const topmap::TopologyGraph& graph,
                                      const std::string& frame_id,
                                      const rclcpp::Time& stamp) {
  topmap_msgs::msg::TopologyGraph msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;

  // Deterministic arrays: sort by id instead of relying on storage order.
  std::vector<topmap::SurfaceRegion> regions = graph.regions();
  std::sort(regions.begin(), regions.end(),
            [](const topmap::SurfaceRegion& a, const topmap::SurfaceRegion& b) { return a.id < b.id; });
  msg.regions.reserve(regions.size());
  for (const topmap::SurfaceRegion& region : regions) msg.regions.push_back(toMsg(region));

  std::vector<topmap::Portal> portals = graph.portals();
  std::sort(portals.begin(), portals.end(),
            [](const topmap::Portal& a, const topmap::Portal& b) { return a.id < b.id; });
  msg.portals.reserve(portals.size());
  for (const topmap::Portal& portal : portals) msg.portals.push_back(toMsg(portal));

  // Flat region-portal incidence list: two edges per portal, sorted by
  // (region_id, portal_id).
  std::vector<std::pair<topmap::RegionId, topmap::PortalId>> edges;
  edges.reserve(portals.size() * 2u);
  for (const topmap::Portal& portal : portals) {
    edges.emplace_back(portal.region_a, portal.id);
    edges.emplace_back(portal.region_b, portal.id);
  }
  std::sort(edges.begin(), edges.end());
  msg.edge_region_ids.reserve(edges.size());
  msg.edge_portal_ids.reserve(edges.size());
  for (const auto& edge : edges) {
    msg.edge_region_ids.push_back(edge.first);
    msg.edge_portal_ids.push_back(edge.second);
  }
  return msg;
}

topmap::Result<topmap::TopologyGraph> fromMsg(const topmap_msgs::msg::TopologyGraph& msg) {
  if (msg.edge_region_ids.size() != msg.edge_portal_ids.size()) {
    return topmap::Status::Error(topmap::ErrorCode::kInvalidArgument,
                                 "TopologyGraph edge arrays have different lengths");
  }

  std::vector<topmap::SurfaceRegion> regions;
  regions.reserve(msg.regions.size());
  for (const topmap_msgs::msg::Region& region : msg.regions) regions.push_back(fromMsg(region));

  std::vector<topmap::Portal> portals;
  portals.reserve(msg.portals.size());
  for (const topmap_msgs::msg::Portal& portal : msg.portals) portals.push_back(fromMsg(portal));

  topmap::Result<topmap::TopologyGraph> graph =
      topmap::TopologyGraph::build(regions, portals);
  if (!graph.ok()) return graph.status();

  // The edges are redundant with Portal.region_a/region_b, so require them to
  // encode exactly the incidence list implied by the converted portals.
  std::vector<std::pair<topmap::RegionId, topmap::PortalId>> expected;
  expected.reserve(graph.value().portals().size() * 2u);
  for (const topmap::Portal& portal : graph.value().portals()) {
    expected.emplace_back(portal.region_a, portal.id);
    expected.emplace_back(portal.region_b, portal.id);
  }
  std::sort(expected.begin(), expected.end());

  std::vector<std::pair<topmap::RegionId, topmap::PortalId>> actual;
  actual.reserve(msg.edge_region_ids.size());
  for (std::size_t i = 0; i < msg.edge_region_ids.size(); ++i) {
    actual.emplace_back(msg.edge_region_ids[i], msg.edge_portal_ids[i]);
  }
  std::sort(actual.begin(), actual.end());

  if (actual != expected) {
    return topmap::Status::Error(
        topmap::ErrorCode::kInvalidArgument,
        "TopologyGraph edge arrays do not match the portal incidence list");
  }
  return graph;
}

topmap_msgs::msg::TopologyPath toMsg(const topmap::TopologyPath& path) {
  topmap_msgs::msg::TopologyPath msg;
  msg.region_ids.assign(path.regions.begin(), path.regions.end());
  msg.portal_ids.assign(path.portals.begin(), path.portals.end());
  msg.cost = path.cost;
  return msg;
}

topmap::TopologyPath fromMsg(const topmap_msgs::msg::TopologyPath& msg) {
  topmap::TopologyPath path;
  path.regions.assign(msg.region_ids.begin(), msg.region_ids.end());
  path.portals.assign(msg.portal_ids.begin(), msg.portal_ids.end());
  path.cost = msg.cost;
  return path;
}

topmap_msgs::msg::PlanStats toMsg(const topmap::PlanStats& stats) {
  topmap_msgs::msg::PlanStats msg;
  msg.fallback_used = stats.fallback_used;
  msg.regions_in_corridor = checkedCounterToUint32(stats.regions_in_corridor);
  msg.triangles_in_corridor = checkedCounterToUint32(stats.triangles_in_corridor);
  msg.topology_expanded_nodes = checkedCounterToUint32(stats.topology_expanded_nodes);
  msg.surface_expanded_nodes = checkedCounterToUint32(stats.surface_expanded_nodes);
  msg.topology_time_sec = stats.topology_time_sec;
  msg.surface_time_sec = stats.surface_time_sec;
  msg.optimization_time_sec = stats.optimization_time_sec;
  msg.path_length = stats.path_length;
  msg.min_clearance = stats.min_clearance;
  msg.max_slope_deg = stats.max_slope_deg;
  msg.max_curvature = stats.max_curvature;
  return msg;
}

topmap::PlanStats fromMsg(const topmap_msgs::msg::PlanStats& msg) {
  topmap::PlanStats stats;
  stats.fallback_used = msg.fallback_used;
  stats.regions_in_corridor = checkedCounterToInt(msg.regions_in_corridor);
  stats.triangles_in_corridor = checkedCounterToInt(msg.triangles_in_corridor);
  stats.topology_expanded_nodes = checkedCounterToInt(msg.topology_expanded_nodes);
  stats.surface_expanded_nodes = checkedCounterToInt(msg.surface_expanded_nodes);
  stats.topology_time_sec = msg.topology_time_sec;
  stats.surface_time_sec = msg.surface_time_sec;
  stats.optimization_time_sec = msg.optimization_time_sec;
  stats.path_length = msg.path_length;
  stats.min_clearance = msg.min_clearance;
  stats.max_slope_deg = msg.max_slope_deg;
  stats.max_curvature = msg.max_curvature;
  return stats;
}

topmap_msgs::msg::MapStats toMsg(const topmap::MapMetadata& metadata) {
  topmap_msgs::msg::MapStats msg;
  msg.format_version = topmap::kTopmapFormatVersion;
  msg.frame_id = metadata.frame_id;
  msg.point_count = metadata.point_count;
  msg.triangle_count = metadata.triangle_count;
  msg.region_count = metadata.region_count;
  msg.portal_count = metadata.portal_count;
  msg.connected_components = metadata.connected_components;
  msg.build_time_sec = metadata.build_time_sec;
  msg.memory_mb = metadata.memory_mb;
  // MapMetadata::regions_before_compression has no MapStats field (T29 must not
  // change the message), so it is deliberately not represented on the wire.
  return msg;
}

nav_msgs::msg::Path toMsg(const std::vector<Eigen::Vector3f>& points,
                          const std::string& frame_id,
                          const rclcpp::Time& stamp) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(points.size());
  for (const Eigen::Vector3f& point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = toMsg(point);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace topmap_ros
