// T29: marker builder contract tests. The builders never touch a Node or an
// executor, so the tests run without rclcpp::init() and without spinning.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include "topmap_ros/marker_builder.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace {

const rclcpp::Time kStamp(12, 345000000);

topmap::TriangleMesh TwoTriangleMesh() {
  topmap::MeshData data;
  data.vertices = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
  data.faces = {{0, 1, 2}, {0, 2, 3}};
  auto mesh = topmap::TriangleMesh::fromMeshData(data, topmap::MeshBuildParams{});
  return mesh.take();
}

topmap::SurfaceRegion MakeRegion(topmap::RegionId id, float x,
                                 std::vector<topmap::TriangleId> triangles = {}) {
  topmap::SurfaceRegion region;
  region.id = id;
  region.triangle_ids = std::move(triangles);
  region.centroid = Eigen::Vector3f(x, 0.5f, 0.25f);
  return region;
}

topmap::Portal MakePortal(topmap::PortalId id, topmap::RegionId a, topmap::RegionId b) {
  topmap::Portal portal;
  portal.id = id;
  portal.region_a = a;
  portal.region_b = b;
  portal.left = Eigen::Vector3f(0.0f, -0.6f, 0.0f);
  portal.right = Eigen::Vector3f(0.0f, 0.6f, 0.0f);
  portal.center = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  return portal;
}

topmap::TopologyGraph TwoRegionGraph() {
  // The two regions must not claim the same triangle ids: TopologyGraph::build
  // rejects duplicate ownership, so give each region one distinct triangle.
  auto graph = topmap::TopologyGraph::build({MakeRegion(0, 0.0f, {0}), MakeRegion(1, 2.0f, {1})},
                                            {MakePortal(5, 0, 1)});
  return graph.take();
}

void ExpectCommonMarkerProperties(const visualization_msgs::msg::Marker& marker) {
  EXPECT_EQ(marker.header.frame_id, "map") << marker.ns;
  EXPECT_EQ(marker.header.stamp.sec, 12) << marker.ns;
  EXPECT_EQ(marker.header.stamp.nanosec, 345000000u) << marker.ns;
  EXPECT_EQ(marker.ns.rfind("topmap/", 0), 0u) << marker.ns;
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD) << marker.ns;
  EXPECT_FALSE(marker.frame_locked) << marker.ns;
  EXPECT_EQ(marker.lifetime.sec, 0) << marker.ns;
  EXPECT_EQ(marker.lifetime.nanosec, 0u) << marker.ns;
  EXPECT_FLOAT_EQ(marker.pose.orientation.w, 1.0f) << marker.ns;
  EXPECT_FLOAT_EQ(marker.pose.orientation.x, 0.0f) << marker.ns;
  EXPECT_FLOAT_EQ(marker.pose.orientation.y, 0.0f) << marker.ns;
  EXPECT_FLOAT_EQ(marker.pose.orientation.z, 0.0f) << marker.ns;
  EXPECT_GT(marker.color.a, 0.0f) << marker.ns;
  EXPECT_GT(marker.scale.x, 0.0f) << marker.ns;
}

}  // namespace

TEST(MarkerBuilderTest, RegionMarkersUseActualMeshTriangles) {
  const topmap::TriangleMesh mesh = TwoTriangleMesh();
  const std::vector<topmap::SurfaceRegion> regions = {MakeRegion(4, 0.5f, {0, 1})};

  const auto markers = topmap_ros::buildRegionMarkers(mesh, regions, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 1u);
  const visualization_msgs::msg::Marker& marker = markers.markers[0];
  ExpectCommonMarkerProperties(marker);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TRIANGLE_LIST);
  EXPECT_EQ(marker.id, 4);

  // Every region triangle contributes its three mesh vertices, in the mesh's
  // own triangle vertex order (which fromMeshData may rotate).
  std::vector<Eigen::Vector3f> expected;
  for (const topmap::TriangleId triangle_id : {0, 1}) {
    for (const topmap::PointId vertex_id : mesh.triangle(triangle_id).vertex_ids) {
      expected.push_back(mesh.vertices()[vertex_id]);
    }
  }
  ASSERT_EQ(marker.points.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_FLOAT_EQ(marker.points[i].x, expected[i].x());
    EXPECT_FLOAT_EQ(marker.points[i].y, expected[i].y());
    EXPECT_FLOAT_EQ(marker.points[i].z, expected[i].z());
  }
}

TEST(MarkerBuilderTest, RegionColorsAreDeterministicByRegionId) {
  const topmap::TriangleMesh mesh = TwoTriangleMesh();
  const std::vector<topmap::SurfaceRegion> regions = {MakeRegion(0, 0.0f, {0}),
                                                     MakeRegion(1, 2.0f, {1})};

  const auto first = topmap_ros::buildRegionMarkers(mesh, regions, "map", kStamp);
  const auto second = topmap_ros::buildRegionMarkers(mesh, regions, "map", kStamp);
  ASSERT_EQ(first.markers.size(), 2u);
  ASSERT_EQ(second.markers.size(), 2u);
  for (std::size_t i = 0; i < first.markers.size(); ++i) {
    EXPECT_FLOAT_EQ(first.markers[i].color.r, second.markers[i].color.r);
    EXPECT_FLOAT_EQ(first.markers[i].color.g, second.markers[i].color.g);
    EXPECT_FLOAT_EQ(first.markers[i].color.b, second.markers[i].color.b);
  }
  // Different ids must not collapse to the same colour.
  const bool same = first.markers[0].color.r == first.markers[1].color.r &&
                    first.markers[0].color.g == first.markers[1].color.g &&
                    first.markers[0].color.b == first.markers[1].color.b;
  EXPECT_FALSE(same);
}

TEST(MarkerBuilderTest, RegionMarkersSkipInvalidIdsAndTriangles) {
  const topmap::TriangleMesh mesh = TwoTriangleMesh();
  const std::vector<topmap::SurfaceRegion> regions = {
      MakeRegion(-1, 0.0f),                 // invalid region id
      MakeRegion(4, 0.5f, {0, 99, -3}),     // only triangle 0 is valid
      MakeRegion(6, 1.0f, {99, -3}),        // no valid triangle
      MakeRegion(8, 2.0f, {}),              // no triangles at all
  };

  const auto markers = topmap_ros::buildRegionMarkers(mesh, regions, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].id, 4);
  EXPECT_EQ(markers.markers[0].points.size(), 3u);
}

TEST(MarkerBuilderTest, RegionMarkersGraphOverloadMatchesVectorOverload) {
  const topmap::TriangleMesh mesh = TwoTriangleMesh();
  const topmap::TopologyGraph graph = TwoRegionGraph();

  const auto via_vector = topmap_ros::buildRegionMarkers(mesh, graph.regions(), "map", kStamp);
  const auto via_graph = topmap_ros::buildRegionMarkers(mesh, graph, "map", kStamp);
  ASSERT_EQ(via_vector.markers.size(), via_graph.markers.size());
  for (std::size_t i = 0; i < via_graph.markers.size(); ++i) {
    EXPECT_EQ(via_graph.markers[i].id, via_vector.markers[i].id);
    EXPECT_EQ(via_graph.markers[i].points.size(), via_vector.markers[i].points.size());
  }
}

TEST(MarkerBuilderTest, PortalMarkersAreRedLineLists) {
  topmap::Portal portal;
  portal.id = 0;
  portal.region_a = 0;
  portal.region_b = 1;
  portal.left = Eigen::Vector3f(0.0f, -0.6f, 0.0f);
  portal.right = Eigen::Vector3f(0.0f, 0.6f, 0.0f);

  const auto markers = topmap_ros::buildPortalMarkers({portal}, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 1u);
  const visualization_msgs::msg::Marker& marker = markers.markers[0];
  ExpectCommonMarkerProperties(marker);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(marker.id, 0);
  ASSERT_EQ(marker.points.size(), 2u);
  EXPECT_FLOAT_EQ(marker.points[0].y, -0.6f);
  EXPECT_FLOAT_EQ(marker.points[1].y, 0.6f);
  EXPECT_FLOAT_EQ(marker.color.r, 1.0f);
  EXPECT_FLOAT_EQ(marker.color.g, 0.0f);
  EXPECT_FLOAT_EQ(marker.color.b, 0.0f);
  EXPECT_FLOAT_EQ(marker.color.a, 1.0f);
}

TEST(MarkerBuilderTest, PortalMarkersSkipInvalidIds) {
  topmap::Portal invalid = MakePortal(-1, 0, 1);
  topmap::Portal valid = MakePortal(7, 0, 1);

  const auto markers = topmap_ros::buildPortalMarkers({invalid, valid}, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].id, 7);
}

TEST(MarkerBuilderTest, CorridorMarkersUseCorridorTriangles) {
  const topmap::TriangleMesh mesh = TwoTriangleMesh();
  topmap::Corridor corridor;
  corridor.regions = {0};
  corridor.triangles = {0, 1};

  const auto markers = topmap_ros::buildCorridorMarkers(mesh, corridor, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 1u);
  ExpectCommonMarkerProperties(markers.markers[0]);
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::TRIANGLE_LIST);
  EXPECT_EQ(markers.markers[0].points.size(), 6u);

  topmap::Corridor partial;
  partial.triangles = {0, 42, -1};
  const auto partial_markers = topmap_ros::buildCorridorMarkers(mesh, partial, "map", kStamp);
  ASSERT_EQ(partial_markers.markers.size(), 1u);
  EXPECT_EQ(partial_markers.markers[0].points.size(), 3u);
}

TEST(MarkerBuilderTest, TopologyPathMarkersUseOrderedGeometry) {
  const topmap::TopologyGraph graph = TwoRegionGraph();
  topmap::TopologyPath path;
  path.regions = {0, 1};
  path.portals = {5};

  const auto markers = topmap_ros::buildTopologyPathMarkers(graph, path, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 2u);

  const visualization_msgs::msg::Marker& regions = markers.markers[0];
  ExpectCommonMarkerProperties(regions);
  EXPECT_EQ(regions.type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(regions.ns, "topmap/topology_path");
  ASSERT_EQ(regions.points.size(), 2u);
  EXPECT_FLOAT_EQ(regions.points[0].x, 0.0f);
  EXPECT_FLOAT_EQ(regions.points[1].x, 2.0f);

  const visualization_msgs::msg::Marker& portals = markers.markers[1];
  ExpectCommonMarkerProperties(portals);
  EXPECT_EQ(portals.type, visualization_msgs::msg::Marker::SPHERE_LIST);
  EXPECT_EQ(portals.ns, "topmap/topology_path_portals");
  ASSERT_EQ(portals.points.size(), 1u);
  EXPECT_FLOAT_EQ(portals.points[0].x, 1.0f);
}

TEST(MarkerBuilderTest, TopologyPathMarkersSkipUnknownIds) {
  const topmap::TopologyGraph graph = TwoRegionGraph();
  topmap::TopologyPath path;
  path.regions = {0, 99};
  path.portals = {5, 77};

  const auto markers = topmap_ros::buildTopologyPathMarkers(graph, path, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 2u);
  EXPECT_EQ(markers.markers[0].points.size(), 1u);
  EXPECT_EQ(markers.markers[1].points.size(), 1u);

  topmap::TopologyPath unknown;
  unknown.regions = {123};
  unknown.portals = {456};
  const auto empty = topmap_ros::buildTopologyPathMarkers(graph, unknown, "map", kStamp);
  ASSERT_EQ(empty.markers.size(), 2u);
  EXPECT_TRUE(empty.markers[0].points.empty());
  EXPECT_TRUE(empty.markers[1].points.empty());
}

TEST(MarkerBuilderTest, StartGoalMarkersHaveStableIdsAndPositions) {
  const Eigen::Vector3f start(-1.0f, 2.0f, 0.0f);
  const Eigen::Vector3f goal(5.0f, -3.0f, 1.5f);

  const auto markers = topmap_ros::buildStartGoalMarkers(start, goal, "map", kStamp);
  ASSERT_EQ(markers.markers.size(), 2u);
  const visualization_msgs::msg::Marker& start_marker = markers.markers[0];
  const visualization_msgs::msg::Marker& goal_marker = markers.markers[1];
  ExpectCommonMarkerProperties(start_marker);
  ExpectCommonMarkerProperties(goal_marker);

  EXPECT_EQ(start_marker.ns, "topmap/start");
  EXPECT_EQ(start_marker.id, 0);
  EXPECT_EQ(start_marker.type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_FLOAT_EQ(start_marker.pose.position.x, -1.0f);
  EXPECT_FLOAT_EQ(start_marker.pose.position.y, 2.0f);
  EXPECT_FLOAT_EQ(start_marker.pose.position.z, 0.0f);

  EXPECT_EQ(goal_marker.ns, "topmap/goal");
  EXPECT_EQ(goal_marker.id, 1);
  EXPECT_EQ(goal_marker.type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_FLOAT_EQ(goal_marker.pose.position.x, 5.0f);
  EXPECT_FLOAT_EQ(goal_marker.pose.position.y, -3.0f);
  EXPECT_FLOAT_EQ(goal_marker.pose.position.z, 1.5f);
}
