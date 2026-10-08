// T29: type conversion contract tests. No ROS graph is created and no executor
// is spun; rclcpp::Time is usable without rclcpp::init().
#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include "topmap_ros/type_conversions.hpp"

namespace {

const rclcpp::Time kStamp(12, 345000000);

topmap::SurfaceRegion MakeRegion(topmap::RegionId id, float x) {
  topmap::SurfaceRegion region;
  region.id = id;
  region.centroid = Eigen::Vector3f(x, 0.5f, 0.25f);
  region.area = 1.5f;
  region.min_z = -0.1f;
  region.max_z = 0.2f;
  region.mean_slope = 3.0f;
  region.max_slope = 7.0f;
  region.roughness = 0.02f;
  region.is_ramp = true;
  region.is_topology_critical = true;
  region.portal_ids = {9, 4};
  return region;
}

topmap::Portal MakePortal(topmap::PortalId id, topmap::RegionId a, topmap::RegionId b) {
  topmap::Portal portal;
  portal.id = id;
  portal.region_a = a;
  portal.region_b = b;
  portal.boundary_chain = {11, 12, 13};
  portal.left = Eigen::Vector3f(0.0f, -0.6f, 0.0f);
  portal.right = Eigen::Vector3f(0.0f, 0.6f, 0.0f);
  portal.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  portal.raw_width = 1.2f;
  portal.valid_width = 1.0f;
  portal.mean_slope = 2.0f;
  portal.max_step = 0.03f;
  portal.min_clearance = 0.5f;
  portal.traversal_cost = 1.7f;
  portal.is_critical = true;
  return portal;
}

topmap::TopologyGraph MakeGraph() {
  const std::vector<topmap::SurfaceRegion> regions = {MakeRegion(3, 0.0f), MakeRegion(7, 2.0f)};
  const std::vector<topmap::Portal> portals = {MakePortal(9, 3, 7), MakePortal(4, 7, 3)};
  auto graph = topmap::TopologyGraph::build(regions, portals);
  return graph.take();
}

}  // namespace

TEST(TypeConversionsTest, PointRoundTrip) {
  const Eigen::Vector3f point(1.5f, -2.25f, 3.75f);
  const geometry_msgs::msg::Point msg = topmap_ros::toMsg(point);
  EXPECT_FLOAT_EQ(msg.x, 1.5f);
  EXPECT_FLOAT_EQ(msg.y, -2.25f);
  EXPECT_FLOAT_EQ(msg.z, 3.75f);
  const Eigen::Vector3f back = topmap_ros::fromMsg(msg);
  EXPECT_FLOAT_EQ(back.x(), point.x());
  EXPECT_FLOAT_EQ(back.y(), point.y());
  EXPECT_FLOAT_EQ(back.z(), point.z());
}

TEST(TypeConversionsTest, RegionRoundTrip) {
  topmap::SurfaceRegion region = MakeRegion(4, 1.0f);
  region.portal_ids = {7, 8};

  const topmap_msgs::msg::Region msg = topmap_ros::toMsg(region);
  EXPECT_EQ(msg.id, 4);
  EXPECT_FLOAT_EQ(msg.centroid.x, 1.0f);
  EXPECT_FLOAT_EQ(msg.centroid.y, 0.5f);
  EXPECT_FLOAT_EQ(msg.centroid.z, 0.25f);
  EXPECT_FLOAT_EQ(msg.area, 1.5f);
  EXPECT_FLOAT_EQ(msg.min_z, -0.1f);
  EXPECT_FLOAT_EQ(msg.max_z, 0.2f);
  EXPECT_FLOAT_EQ(msg.mean_slope, 3.0f);
  EXPECT_FLOAT_EQ(msg.max_slope, 7.0f);
  EXPECT_FLOAT_EQ(msg.roughness, 0.02f);
  EXPECT_TRUE(msg.is_ramp);
  EXPECT_TRUE(msg.is_topology_critical);
  ASSERT_EQ(msg.portal_ids.size(), 2u);
  EXPECT_EQ(msg.portal_ids[0], 7);
  EXPECT_EQ(msg.portal_ids[1], 8);

  const topmap::SurfaceRegion back = topmap_ros::fromMsg(msg);
  EXPECT_EQ(back.id, 4);
  EXPECT_EQ(back.portal_ids, (std::vector<topmap::PortalId>{7, 8}));
  EXPECT_FLOAT_EQ(back.centroid.x(), 1.0f);
  EXPECT_TRUE(back.is_ramp);
  EXPECT_TRUE(back.is_topology_critical);
}

TEST(TypeConversionsTest, PortalRoundTripDropsBoundaryChain) {
  const topmap::Portal portal = MakePortal(5, 1, 2);

  const topmap_msgs::msg::Portal msg = topmap_ros::toMsg(portal);
  EXPECT_EQ(msg.id, 5);
  EXPECT_EQ(msg.region_a, 1);
  EXPECT_EQ(msg.region_b, 2);
  EXPECT_FLOAT_EQ(msg.left.y, -0.6f);
  EXPECT_FLOAT_EQ(msg.right.y, 0.6f);
  EXPECT_FLOAT_EQ(msg.raw_width, 1.2f);
  EXPECT_FLOAT_EQ(msg.valid_width, 1.0f);
  EXPECT_FLOAT_EQ(msg.mean_slope, 2.0f);
  EXPECT_FLOAT_EQ(msg.max_step, 0.03f);
  EXPECT_FLOAT_EQ(msg.min_clearance, 0.5f);
  EXPECT_FLOAT_EQ(msg.traversal_cost, 1.7f);
  EXPECT_TRUE(msg.is_critical);

  const topmap::Portal back = topmap_ros::fromMsg(msg);
  EXPECT_EQ(back.id, 5);
  EXPECT_EQ(back.region_a, 1);
  EXPECT_EQ(back.region_b, 2);
  EXPECT_FLOAT_EQ(back.center.z(), 0.0f);
  EXPECT_TRUE(back.is_critical);
  // A4.9 Portal.msg has no boundary_chain field: the wire round trip cannot
  // retain it, which is the documented contract.
  EXPECT_TRUE(back.boundary_chain.empty());
}

TEST(TypeConversionsTest, TopologyGraphToMsgIsDeterministic) {
  const topmap::TopologyGraph graph = MakeGraph();

  const topmap_msgs::msg::TopologyGraph msg = topmap_ros::toMsg(graph, "odom", kStamp);
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 345000000u);

  ASSERT_EQ(msg.regions.size(), 2u);
  EXPECT_EQ(msg.regions[0].id, 3);
  EXPECT_EQ(msg.regions[1].id, 7);
  ASSERT_EQ(msg.portals.size(), 2u);
  EXPECT_EQ(msg.portals[0].id, 4);
  EXPECT_EQ(msg.portals[1].id, 9);

  ASSERT_EQ(msg.edge_region_ids.size(), 4u);
  ASSERT_EQ(msg.edge_portal_ids.size(), 4u);
  EXPECT_EQ(msg.edge_region_ids, (std::vector<std::int32_t>{3, 3, 7, 7}));
  EXPECT_EQ(msg.edge_portal_ids, (std::vector<std::int32_t>{4, 9, 4, 9}));

  // toMsg must not depend on the graph's internal storage order.
  const std::vector<topmap::SurfaceRegion> reversed_regions = {MakeRegion(7, 2.0f),
                                                               MakeRegion(3, 0.0f)};
  const std::vector<topmap::Portal> reversed_portals = {MakePortal(4, 7, 3), MakePortal(9, 3, 7)};
  auto reversed = topmap::TopologyGraph::build(reversed_regions, reversed_portals);
  ASSERT_TRUE(reversed.ok());
  const topmap_msgs::msg::TopologyGraph other = topmap_ros::toMsg(reversed.value(), "odom", kStamp);
  EXPECT_EQ(other.regions[0].id, msg.regions[0].id);
  EXPECT_EQ(other.regions[1].id, msg.regions[1].id);
  EXPECT_EQ(other.portals[0].id, msg.portals[0].id);
  EXPECT_EQ(other.portals[1].id, msg.portals[1].id);
  EXPECT_EQ(other.edge_region_ids, msg.edge_region_ids);
  EXPECT_EQ(other.edge_portal_ids, msg.edge_portal_ids);
}

TEST(TypeConversionsTest, TopologyGraphRoundTrip) {
  const topmap::TopologyGraph graph = MakeGraph();
  auto restored = topmap_ros::fromMsg(topmap_ros::toMsg(graph, "map", kStamp));
  ASSERT_TRUE(restored.ok()) << restored.status().message;

  EXPECT_TRUE(restored.value().hasRegion(3));
  EXPECT_TRUE(restored.value().hasRegion(7));
  EXPECT_FALSE(restored.value().hasRegion(4));
  EXPECT_TRUE(restored.value().portal(4) != nullptr);
  EXPECT_TRUE(restored.value().portal(9) != nullptr);
  EXPECT_EQ(restored.value().portalsOf(3), (std::vector<topmap::PortalId>{4, 9}));
  EXPECT_EQ(restored.value().neighborRegions(3), (std::vector<topmap::RegionId>{7}));
  ASSERT_NE(restored.value().region(3), nullptr);
  EXPECT_FLOAT_EQ(restored.value().region(3)->centroid.x(), 0.0f);
  EXPECT_EQ(restored.value().connectedComponents(), 1);
}

TEST(TypeConversionsTest, TopologyGraphFromMsgRejectsMismatchedEdgeArrays) {
  const topmap_msgs::msg::TopologyGraph msg = topmap_ros::toMsg(MakeGraph(), "map", kStamp);
  topmap_msgs::msg::TopologyGraph broken = msg;
  broken.edge_portal_ids.pop_back();
  auto result = topmap_ros::fromMsg(broken);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code, topmap::ErrorCode::kInvalidArgument);
}

TEST(TypeConversionsTest, TopologyGraphFromMsgRejectsUnknownEdgeEndpoint) {
  const topmap_msgs::msg::TopologyGraph msg = topmap_ros::toMsg(MakeGraph(), "map", kStamp);
  {
    topmap_msgs::msg::TopologyGraph broken = msg;
    broken.edge_region_ids[0] = 42;  // no such region
    EXPECT_FALSE(topmap_ros::fromMsg(broken).ok());
  }
  {
    topmap_msgs::msg::TopologyGraph broken = msg;
    broken.edge_portal_ids[0] = 99;  // no such portal
    EXPECT_FALSE(topmap_ros::fromMsg(broken).ok());
  }
}

TEST(TypeConversionsTest, TopologyGraphFromMsgRejectsMissingEdge) {
  const topmap_msgs::msg::TopologyGraph msg = topmap_ros::toMsg(MakeGraph(), "map", kStamp);
  topmap_msgs::msg::TopologyGraph broken = msg;
  broken.edge_region_ids.pop_back();
  broken.edge_portal_ids.pop_back();
  EXPECT_FALSE(topmap_ros::fromMsg(broken).ok());
}

TEST(TypeConversionsTest, TopologyGraphFromMsgRejectsMalformedGraph) {
  // A duplicated region id is rejected by TopologyGraph::build.
  topmap_msgs::msg::TopologyGraph duplicate = topmap_ros::toMsg(MakeGraph(), "map", kStamp);
  topmap_msgs::msg::Region copy = duplicate.regions[0];
  duplicate.regions.push_back(copy);
  EXPECT_FALSE(topmap_ros::fromMsg(duplicate).ok());

  // A portal referencing a missing region is rejected by TopologyGraph::build.
  topmap_msgs::msg::TopologyGraph missing = topmap_ros::toMsg(MakeGraph(), "map", kStamp);
  missing.portals[0].region_a = 123;
  EXPECT_FALSE(topmap_ros::fromMsg(missing).ok());
}

TEST(TypeConversionsTest, TopologyGraphFromMsgAcceptsEmptyGraph) {
  topmap_msgs::msg::TopologyGraph empty;
  auto result = topmap_ros::fromMsg(empty);
  ASSERT_TRUE(result.ok()) << result.status().message;
  EXPECT_TRUE(result.value().regions().empty());
  EXPECT_TRUE(result.value().portals().empty());
}

TEST(TypeConversionsTest, TopologyPathRoundTrip) {
  topmap::TopologyPath path;
  path.regions = {3, 7, 11};
  path.portals = {4, 9};
  path.cost = 6.5f;

  const topmap_msgs::msg::TopologyPath msg = topmap_ros::toMsg(path);
  EXPECT_EQ(msg.region_ids, (std::vector<std::int32_t>{3, 7, 11}));
  EXPECT_EQ(msg.portal_ids, (std::vector<std::int32_t>{4, 9}));
  EXPECT_FLOAT_EQ(msg.cost, 6.5f);

  const topmap::TopologyPath back = topmap_ros::fromMsg(msg);
  EXPECT_EQ(back.regions, path.regions);
  EXPECT_EQ(back.portals, path.portals);
  EXPECT_FLOAT_EQ(back.cost, 6.5f);
}

TEST(TypeConversionsTest, PlanStatsRoundTrip) {
  topmap::PlanStats stats;
  stats.regions_in_corridor = 5;
  stats.triangles_in_corridor = 41;
  stats.topology_expanded_nodes = 17;
  stats.surface_expanded_nodes = 203;
  stats.topology_time_sec = 0.25;
  stats.surface_time_sec = 1.5;
  stats.optimization_time_sec = 0.75;
  stats.path_length = 12.25f;
  stats.min_clearance = 0.5f;
  stats.max_slope_deg = 8.0f;
  stats.max_curvature = 0.1f;
  stats.fallback_used = true;

  const topmap_msgs::msg::PlanStats msg = topmap_ros::toMsg(stats);
  EXPECT_EQ(msg.regions_in_corridor, 5u);
  EXPECT_EQ(msg.triangles_in_corridor, 41u);
  EXPECT_EQ(msg.topology_expanded_nodes, 17u);
  EXPECT_EQ(msg.surface_expanded_nodes, 203u);
  EXPECT_DOUBLE_EQ(msg.topology_time_sec, 0.25);
  EXPECT_DOUBLE_EQ(msg.surface_time_sec, 1.5);
  EXPECT_DOUBLE_EQ(msg.optimization_time_sec, 0.75);
  EXPECT_FLOAT_EQ(msg.path_length, 12.25f);
  EXPECT_FLOAT_EQ(msg.min_clearance, 0.5f);
  EXPECT_FLOAT_EQ(msg.max_slope_deg, 8.0f);
  EXPECT_FLOAT_EQ(msg.max_curvature, 0.1f);
  EXPECT_TRUE(msg.fallback_used);

  const topmap::PlanStats back = topmap_ros::fromMsg(msg);
  EXPECT_EQ(back.regions_in_corridor, 5);
  EXPECT_EQ(back.surface_expanded_nodes, 203);
  EXPECT_DOUBLE_EQ(back.optimization_time_sec, 0.75);
  EXPECT_FLOAT_EQ(back.max_curvature, 0.1f);
  EXPECT_TRUE(back.fallback_used);
}

TEST(TypeConversionsTest, PlanStatsNegativeCountersClampToZero) {
  topmap::PlanStats stats;
  stats.regions_in_corridor = -3;
  stats.triangles_in_corridor = -1;
  const topmap_msgs::msg::PlanStats msg = topmap_ros::toMsg(stats);
  EXPECT_EQ(msg.regions_in_corridor, 0u);
  EXPECT_EQ(msg.triangles_in_corridor, 0u);
}

TEST(TypeConversionsTest, PlanStatsHugeUnsignedCountersSaturate) {
  topmap_msgs::msg::PlanStats msg;
  msg.topology_expanded_nodes = std::numeric_limits<std::uint32_t>::max();
  msg.surface_expanded_nodes = static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) + 1u;
  const topmap::PlanStats stats = topmap_ros::fromMsg(msg);
  EXPECT_EQ(stats.topology_expanded_nodes, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(stats.surface_expanded_nodes, std::numeric_limits<std::int32_t>::max());
}

TEST(TypeConversionsTest, MapMetadataMapsEveryExistingMapStatsField) {
  topmap::MapMetadata metadata;
  metadata.frame_id = "lidar";
  metadata.point_count = 1000u;
  metadata.triangle_count = 2000u;
  metadata.region_count = 12u;
  metadata.portal_count = 15u;
  metadata.connected_components = 3u;
  metadata.regions_before_compression = 20u;  // no MapStats counterpart
  metadata.build_time_sec = 2.5;
  metadata.memory_mb = 128.0;

  const topmap_msgs::msg::MapStats msg = topmap_ros::toMsg(metadata);
  EXPECT_EQ(msg.format_version, topmap::kTopmapFormatVersion);
  EXPECT_EQ(msg.frame_id, "lidar");
  EXPECT_EQ(msg.point_count, 1000u);
  EXPECT_EQ(msg.triangle_count, 2000u);
  EXPECT_EQ(msg.region_count, 12u);
  EXPECT_EQ(msg.portal_count, 15u);
  EXPECT_EQ(msg.connected_components, 3u);
  EXPECT_DOUBLE_EQ(msg.build_time_sec, 2.5);
  EXPECT_DOUBLE_EQ(msg.memory_mb, 128.0);
}

TEST(TypeConversionsTest, NavPathCarriesFrameStampAndIdentityOrientations) {
  const std::vector<Eigen::Vector3f> points = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.5f}, {2.0f, 1.0f, 0.5f}};
  const nav_msgs::msg::Path path = topmap_ros::toMsg(points, "map", kStamp);

  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.header.stamp.sec, 12);
  EXPECT_EQ(path.header.stamp.nanosec, 345000000u);
  ASSERT_EQ(path.poses.size(), 3u);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(path.poses[i].header.frame_id, "map");
    EXPECT_EQ(path.poses[i].header.stamp.sec, 12);
    EXPECT_FLOAT_EQ(path.poses[i].pose.position.x, points[i].x());
    EXPECT_FLOAT_EQ(path.poses[i].pose.position.y, points[i].y());
    EXPECT_FLOAT_EQ(path.poses[i].pose.position.z, points[i].z());
    EXPECT_FLOAT_EQ(path.poses[i].pose.orientation.w, 1.0f);
    EXPECT_FLOAT_EQ(path.poses[i].pose.orientation.x, 0.0f);
    EXPECT_FLOAT_EQ(path.poses[i].pose.orientation.y, 0.0f);
    EXPECT_FLOAT_EQ(path.poses[i].pose.orientation.z, 0.0f);
  }

  const nav_msgs::msg::Path empty = topmap_ros::toMsg(std::vector<Eigen::Vector3f>{}, "map", kStamp);
  EXPECT_TRUE(empty.poses.empty());
  EXPECT_EQ(empty.header.frame_id, "map");
}
