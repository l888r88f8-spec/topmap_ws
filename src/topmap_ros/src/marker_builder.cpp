#include "topmap_ros/marker_builder.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "topmap_ros/type_conversions.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace topmap_ros {
namespace {

void InitMarker(visualization_msgs::msg::Marker& marker, const std::string& ns, int id, int type,
                const std::string& frame_id, const rclcpp::Time& stamp) {
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.frame_locked = false;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
  marker.pose.orientation.w = 1.0;
}

// Standard HSV -> RGB with h wrapped into [0, 1). s/v in [0, 1].
void HsvToRgb(float h, float s, float v, std_msgs::msg::ColorRGBA& color) {
  h -= std::floor(h);
  const float sector = h * 6.0f;
  const int index = static_cast<int>(sector) % 6;
  const float f = sector - std::floor(sector);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - f * s);
  const float t = v * (1.0f - (1.0f - f) * s);
  switch (index) {
    case 0: color.r = v; color.g = t; color.b = p; break;
    case 1: color.r = q; color.g = v; color.b = p; break;
    case 2: color.r = p; color.g = v; color.b = t; break;
    case 3: color.r = p; color.g = q; color.b = v; break;
    case 4: color.r = t; color.g = p; color.b = v; break;
    default: color.r = v; color.g = p; color.b = q; break;
  }
  color.a = 1.0f;
}

// Deterministic region colour: the golden-ratio conjugate spreads consecutive
// ids over the hue circle while remaining a pure function of the id.
std_msgs::msg::ColorRGBA RegionColor(topmap::RegionId id) {
  std_msgs::msg::ColorRGBA color;
  const float hue = static_cast<float>(id) * 0.61803398875f;
  HsvToRgb(hue, 0.65f, 0.90f, color);
  return color;
}

// Appends a full triangle (three points) or nothing at all. A triangle whose id
// is out of range or that references a missing vertex is skipped as a unit so
// the TRIANGLE_LIST stays aligned.
void AppendTrianglePoints(const topmap::TriangleMesh& mesh, topmap::TriangleId triangle_id,
                          std::vector<geometry_msgs::msg::Point>& points) {
  if (triangle_id < 0 || static_cast<std::size_t>(triangle_id) >= mesh.size()) return;
  const topmap::TriangleNode& triangle = mesh.triangle(triangle_id);
  const std::vector<Eigen::Vector3f>& vertices = mesh.vertices();
  for (const topmap::PointId vertex_id : triangle.vertex_ids) {
    if (vertex_id < 0 || static_cast<std::size_t>(vertex_id) >= vertices.size()) return;
  }
  for (const topmap::PointId vertex_id : triangle.vertex_ids) {
    points.push_back(toMsg(vertices[vertex_id]));
  }
}

}  // namespace

visualization_msgs::msg::MarkerArray buildRegionMarkers(
    const topmap::TriangleMesh& mesh, const std::vector<topmap::SurfaceRegion>& regions,
    const std::string& frame_id, const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray array;
  for (const topmap::SurfaceRegion& region : regions) {
    if (region.id < 0) continue;  // kInvalidId carries no stable marker id
    visualization_msgs::msg::Marker marker;
    InitMarker(marker, "topmap/regions", region.id, visualization_msgs::msg::Marker::TRIANGLE_LIST,
               frame_id, stamp);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = RegionColor(region.id);
    for (const topmap::TriangleId triangle_id : region.triangle_ids) {
      AppendTrianglePoints(mesh, triangle_id, marker.points);
    }
    if (marker.points.empty()) continue;  // region owns no renderable triangle
    array.markers.push_back(std::move(marker));
  }
  return array;
}

visualization_msgs::msg::MarkerArray buildRegionMarkers(
    const topmap::TriangleMesh& mesh, const topmap::TopologyGraph& graph,
    const std::string& frame_id, const rclcpp::Time& stamp) {
  return buildRegionMarkers(mesh, graph.regions(), frame_id, stamp);
}

visualization_msgs::msg::MarkerArray buildPortalMarkers(
    const std::vector<topmap::Portal>& portals, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray array;
  for (const topmap::Portal& portal : portals) {
    if (portal.id < 0) continue;
    visualization_msgs::msg::Marker marker;
    InitMarker(marker, "topmap/portals", portal.id, visualization_msgs::msg::Marker::LINE_LIST,
               frame_id, stamp);
    marker.scale.x = 0.05;  // line width
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.points.push_back(toMsg(portal.left));
    marker.points.push_back(toMsg(portal.right));
    array.markers.push_back(std::move(marker));
  }
  return array;
}

visualization_msgs::msg::MarkerArray buildPortalMarkers(const topmap::TopologyGraph& graph,
                                                        const std::string& frame_id,
                                                        const rclcpp::Time& stamp) {
  return buildPortalMarkers(graph.portals(), frame_id, stamp);
}

visualization_msgs::msg::MarkerArray buildCorridorMarkers(
    const topmap::TriangleMesh& mesh, const topmap::Corridor& corridor, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  InitMarker(marker, "topmap/corridor", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST, frame_id,
             stamp);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 0.8f;
  marker.color.b = 1.0f;
  marker.color.a = 0.5f;
  for (const topmap::TriangleId triangle_id : corridor.triangles) {
    AppendTrianglePoints(mesh, triangle_id, marker.points);
  }
  array.markers.push_back(std::move(marker));
  return array;
}

visualization_msgs::msg::MarkerArray buildTopologyPathMarkers(
    const topmap::TopologyGraph& graph, const topmap::TopologyPath& path,
    const std::string& frame_id, const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker regions_marker;
  InitMarker(regions_marker, "topmap/topology_path", 0,
             visualization_msgs::msg::Marker::LINE_STRIP, frame_id, stamp);
  regions_marker.scale.x = 0.05;  // line width
  regions_marker.color.r = 0.1f;
  regions_marker.color.g = 1.0f;
  regions_marker.color.b = 0.2f;
  regions_marker.color.a = 1.0f;
  for (const topmap::RegionId region_id : path.regions) {
    const topmap::SurfaceRegion* region = graph.region(region_id);
    if (region == nullptr) continue;  // unknown id: skip safely
    regions_marker.points.push_back(toMsg(region->centroid));
  }
  array.markers.push_back(std::move(regions_marker));

  visualization_msgs::msg::Marker portals_marker;
  InitMarker(portals_marker, "topmap/topology_path_portals", 1,
             visualization_msgs::msg::Marker::SPHERE_LIST, frame_id, stamp);
  portals_marker.scale.x = 0.15;
  portals_marker.scale.y = 0.15;
  portals_marker.scale.z = 0.15;
  portals_marker.color.r = 1.0f;
  portals_marker.color.g = 0.85f;
  portals_marker.color.b = 0.1f;
  portals_marker.color.a = 1.0f;
  for (const topmap::PortalId portal_id : path.portals) {
    const topmap::Portal* portal = graph.portal(portal_id);
    if (portal == nullptr) continue;  // unknown id: skip safely
    portals_marker.points.push_back(toMsg(portal->center));
  }
  array.markers.push_back(std::move(portals_marker));

  return array;
}

visualization_msgs::msg::MarkerArray buildStartGoalMarkers(const Eigen::Vector3f& start,
                                                           const Eigen::Vector3f& goal,
                                                           const std::string& frame_id,
                                                           const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker start_marker;
  InitMarker(start_marker, "topmap/start", 0, visualization_msgs::msg::Marker::SPHERE, frame_id,
             stamp);
  start_marker.pose.position = toMsg(start);
  start_marker.scale.x = 0.3;
  start_marker.scale.y = 0.3;
  start_marker.scale.z = 0.3;
  start_marker.color.r = 0.1f;
  start_marker.color.g = 1.0f;
  start_marker.color.b = 0.1f;
  start_marker.color.a = 1.0f;
  array.markers.push_back(std::move(start_marker));

  visualization_msgs::msg::Marker goal_marker;
  InitMarker(goal_marker, "topmap/goal", 1, visualization_msgs::msg::Marker::SPHERE, frame_id,
             stamp);
  goal_marker.pose.position = toMsg(goal);
  goal_marker.scale.x = 0.3;
  goal_marker.scale.y = 0.3;
  goal_marker.scale.z = 0.3;
  goal_marker.color.r = 1.0f;
  goal_marker.color.g = 0.1f;
  goal_marker.color.b = 0.1f;
  goal_marker.color.a = 1.0f;
  array.markers.push_back(std::move(goal_marker));

  return array;
}

}  // namespace topmap_ros
