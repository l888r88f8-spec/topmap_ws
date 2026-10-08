// T30: offline MapBuilderNode / /topmap/build_map service tests.
//
// Follows the R35 executor pattern through test_support_ros.hpp::callService:
// the node under test and a separate client node are spun by a
// MultiThreadedExecutor on a background thread, so the service callback and the
// future wait are never on the same thread.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_ros/map_builder_node.hpp"
#include "topmap_surface/synthetic_scene.hpp"
#include "topmap_topology/topology_graph.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "topmap_msgs/msg/map_stats.hpp"
#include "topmap_msgs/srv/build_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "test_support_ros.hpp"

namespace {

namespace fs = std::filesystem;

using topmap::ErrorCode;
using topmap::SyntheticScene;
using topmap_ros::MapBuilderNode;
using topmap_ros::test::TempFiles;
using topmap_msgs::srv::BuildMap;

rclcpp::NodeOptions Options(const std::string& pcd, const std::string& output,
                            const std::string& config = topmap_ros::test::validConfigPath(),
                            const std::string& frame_id = "map") {
  return rclcpp::NodeOptions()
      .append_parameter_override("pcd_path", pcd)
      .append_parameter_override("output_map_path", output)
      .append_parameter_override("config_file", config)
      .append_parameter_override("frame_id", frame_id);
}

std::shared_ptr<BuildMap::Request> Request(const std::string& pcd, const std::string& output) {
  auto request = std::make_shared<BuildMap::Request>();
  request->pcd_path = pcd;
  request->output_map_path = output;
  return request;
}

// A single latched subscriber node that captures the four map publishers.
struct CapturedMaps {
  bool got_cloud = false, got_regions = false, got_portals = false, got_stats = false;
  sensor_msgs::msg::PointCloud2 cloud;
  visualization_msgs::msg::MarkerArray regions, portals;
  topmap_msgs::msg::MapStats stats;
};

CapturedMaps CaptureLatchedMaps(const rclcpp::Node::SharedPtr& server_node,
                                std::chrono::milliseconds timeout = std::chrono::seconds(15)) {
  CapturedMaps captured;
  auto sub_node = rclcpp::Node::make_shared(
      "topmap_build_map_sub_" + std::to_string(static_cast<long>(::getpid())));
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  auto sub_cloud = sub_node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/topmap/original_cloud", qos,
      [&captured](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        captured.cloud = *msg;
        captured.got_cloud = true;
      });
  auto sub_regions = sub_node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/topmap/regions", qos,
      [&captured](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        captured.regions = *msg;
        captured.got_regions = true;
      });
  auto sub_portals = sub_node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/topmap/portals", qos,
      [&captured](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        captured.portals = *msg;
        captured.got_portals = true;
      });
  auto sub_stats = sub_node->create_subscription<topmap_msgs::msg::MapStats>(
      "/topmap/map_stats", qos, [&captured](topmap_msgs::msg::MapStats::ConstSharedPtr msg) {
        captured.stats = *msg;
        captured.got_stats = true;
      });

  // The server node does not need to spin: transient-local durability is a
  // middleware responsibility and a late-joining subscriber receives the sample
  // once discovery completes.
  (void)server_node;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline &&
         !(captured.got_cloud && captured.got_regions && captured.got_portals &&
           captured.got_stats)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  executor.remove_node(sub_node);
  return captured;
}

}  // namespace

// Final-plan Hall build example: the service builds the map, the archive it
// writes is self-consistent, the compressed graph is still a valid topology and
// the four map topics are published with the configured frame id.
TEST(MapBuilderNodeTest, BuildsHallMapFromPcdAndPublishesLatchedMaps) {
  topmap_ros::test::RclcppGuard ros;
  TempFiles temp;
  const std::string pcd = temp.Add("t30_hall.pcd");
  const std::string output = temp.Add("t30_hall.bin");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kHallNarrowHall, pcd).empty());

  auto node = std::make_shared<MapBuilderNode>(Options(pcd, output, topmap_ros::test::validConfigPath(),
                                                       "hall_map"));
  ASSERT_TRUE(node->configOk()) << node->configStatus().message;

  auto call = topmap_ros::test::callService<BuildMap>(node, "/topmap/build_map",
                                                      Request(pcd, output));
  ASSERT_TRUE(call.service_available) << "/topmap/build_map is not available";
  ASSERT_TRUE(call.ready) << "build_map call timed out";
  const BuildMap::Response& response = *call.response;
  ASSERT_TRUE(response.success) << response.error_message;
  EXPECT_EQ(response.error_code, 0);
  EXPECT_TRUE(response.error_message.empty());
  EXPECT_EQ(response.stats.format_version, topmap::kTopmapFormatVersion);
  EXPECT_EQ(response.stats.frame_id, "hall_map");
  EXPECT_GT(response.stats.point_count, 0u);
  EXPECT_GT(response.stats.triangle_count, 0u);
  EXPECT_GT(response.stats.region_count, 0u);
  EXPECT_GT(response.stats.portal_count, 0u);
  EXPECT_GE(response.stats.connected_components, 1u);
  EXPECT_GT(response.stats.build_time_sec, 0.0);
  EXPECT_GT(response.stats.memory_mb, 0.0);

  auto loaded = topmap::loadMapArchive(output);
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;
  const topmap::MapArchive& archive = loaded.value();
  EXPECT_EQ(archive.format_version, topmap::kTopmapFormatVersion);
  EXPECT_EQ(archive.metadata.frame_id, "hall_map");
  EXPECT_EQ(archive.metadata.point_count, response.stats.point_count);
  EXPECT_EQ(archive.metadata.triangle_count, static_cast<std::uint32_t>(archive.mesh.size()));
  EXPECT_EQ(archive.metadata.triangle_count, response.stats.triangle_count);
  EXPECT_EQ(archive.metadata.region_count, static_cast<std::uint32_t>(archive.graph.regions().size()));
  EXPECT_EQ(archive.metadata.region_count, response.stats.region_count);
  EXPECT_EQ(archive.metadata.portal_count, static_cast<std::uint32_t>(archive.graph.portals().size()));
  EXPECT_EQ(archive.metadata.portal_count, response.stats.portal_count);
  EXPECT_EQ(archive.metadata.connected_components, response.stats.connected_components);
  EXPECT_EQ(static_cast<int>(archive.metadata.connected_components),
            archive.graph.connectedComponents());
  // Compression may only shrink the region set (R12-b keeps the ramp semantics).
  EXPECT_GE(archive.metadata.regions_before_compression, archive.metadata.region_count);
  // The processed mesh cannot reference more vertices than the loaded cloud.
  EXPECT_LE(archive.mesh.vertices().size(), archive.metadata.point_count);

  // The compressed graph must still satisfy TopologyGraph::build's full
  // validation (ids, ownership, edge incidence).
  auto rebuilt = topmap::TopologyGraph::build(archive.graph.regions(), archive.graph.portals());
  ASSERT_TRUE(rebuilt.ok()) << rebuilt.status().message;
  EXPECT_EQ(rebuilt.value().connectedComponents(), archive.graph.connectedComponents());
  for (const topmap::SurfaceRegion& region : archive.graph.regions()) {
    EXPECT_FALSE(archive.graph.originRegions(region.id).empty());
  }

  // R7: the per-triangle feature channels are finite and non-negative (the flat
  // Hall scene legitimately has ~zero roughness; the non-zero propagation case
  // is covered by PropagatesRoughnessAndCurvatureToTriangles).
  for (const topmap::TriangleNode& triangle : archive.mesh.triangles()) {
    EXPECT_TRUE(std::isfinite(triangle.roughness));
    EXPECT_TRUE(std::isfinite(triangle.curvature));
    EXPECT_GE(triangle.roughness, 0.0f);
    EXPECT_GE(triangle.curvature, 0.0f);
  }

  const CapturedMaps captured = CaptureLatchedMaps(node);
  ASSERT_TRUE(captured.got_cloud);
  ASSERT_TRUE(captured.got_stats);
  ASSERT_TRUE(captured.got_regions);
  ASSERT_TRUE(captured.got_portals);
  EXPECT_EQ(captured.cloud.header.frame_id, "hall_map");
  EXPECT_EQ(captured.stats.frame_id, "hall_map");
  // R4: keep_raw_cloud=true in the test config, so the published cloud is the
  // pre-voxel loaded cloud and its width equals the loaded point count.
  EXPECT_EQ(captured.cloud.width, response.stats.point_count);
  EXPECT_FALSE(captured.regions.markers.empty());
  EXPECT_FALSE(captured.portals.markers.empty());
  for (const auto& marker : captured.regions.markers) {
    EXPECT_EQ(marker.header.frame_id, "hall_map");
  }
  for (const auto& marker : captured.portals.markers) {
    EXPECT_EQ(marker.header.frame_id, "hall_map");
  }
}

// An empty request must fall back to the node parameters.
TEST(MapBuilderNodeTest, EmptyRequestUsesParameters) {
  topmap_ros::test::RclcppGuard ros;
  TempFiles temp;
  const std::string pcd = temp.Add("t30_params.pcd");
  const std::string output = temp.Add("t30_params.bin");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kUShape, pcd).empty());

  auto node = std::make_shared<MapBuilderNode>(Options(pcd, output));
  ASSERT_TRUE(node->configOk()) << node->configStatus().message;

  auto request = std::make_shared<BuildMap::Request>();  // both fields empty
  auto call = topmap_ros::test::callService<BuildMap>(node, "/topmap/build_map", request);
  ASSERT_TRUE(call.ready) << "build_map call timed out";
  EXPECT_TRUE(call.response->success) << call.response->error_message;
  EXPECT_TRUE(fs::exists(output));
  EXPECT_GT(fs::file_size(output), 0u);
}

// A non-empty request path overrides the parameter path (even when the parameter
// path is unusable).
TEST(MapBuilderNodeTest, RequestPathsOverrideParameters) {
  topmap_ros::test::RclcppGuard ros;
  TempFiles temp;
  const std::string pcd = temp.Add("t30_override.pcd");
  const std::string output = temp.Add("t30_override.bin");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kSingleBridge, pcd).empty());

  auto node = std::make_shared<MapBuilderNode>(
      Options("this_parameter_pcd_does_not_exist.pcd", "no_such_parameter_dir/out.bin"));
  ASSERT_TRUE(node->configOk()) << node->configStatus().message;

  auto call = topmap_ros::test::callService<BuildMap>(node, "/topmap/build_map",
                                                      Request(pcd, output));
  ASSERT_TRUE(call.ready) << "build_map call timed out";
  ASSERT_TRUE(call.response->success) << call.response->error_message;
  EXPECT_TRUE(fs::exists(output));
  EXPECT_FALSE(fs::exists("no_such_parameter_dir"));
}

// Every rejected input returns the exact structured error code and never leaves
// a partial archive behind.
TEST(MapBuilderNodeTest, StructuredErrorsForBadInputs) {
  topmap_ros::test::RclcppGuard ros;
  TempFiles temp;
  const std::string pcd = temp.Add("t30_ok.pcd");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kUShape, pcd).empty());

  const std::string empty_pcd = temp.Add("t30_empty.pcd");
  ASSERT_TRUE(topmap_ros::test::WriteNanPcd(empty_pcd));

  const auto expect_failure = [](const std::shared_ptr<MapBuilderNode>& node,
                                 const std::shared_ptr<BuildMap::Request>& request,
                                 ErrorCode code, const std::string& message_fragment) {
    auto call = topmap_ros::test::callService<BuildMap>(node, "/topmap/build_map", request);
    EXPECT_TRUE(call.ready) << "build_map call timed out";
    if (!call.response) return;
    EXPECT_FALSE(call.response->success);
    EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(code));
    const std::string code_text = topmap::toString(code);
    EXPECT_NE(call.response->error_message.find(code_text), std::string::npos)
        << call.response->error_message;
    EXPECT_NE(call.response->error_message.find(message_fragment), std::string::npos)
        << call.response->error_message;
  };

  // Missing PCD file.
  {
    auto node = std::make_shared<MapBuilderNode>(
        Options(pcd, temp.Add("t30_missing.bin")));
    expect_failure(node, Request(temp.Add("t30_does_not_exist.pcd"), temp.Add("t30_missing.bin")),
                   ErrorCode::kInvalidArgument, "PCD");
  }

  // Empty PCD file (load succeeds, preprocessing rejects it).
  {
    auto node = std::make_shared<MapBuilderNode>(
        Options(pcd, temp.Add("t30_empty.bin")));
    expect_failure(node, Request(empty_pcd, temp.Add("t30_empty.bin")),
                   ErrorCode::kPointCloudEmpty, "empty");
  }

  // Output path inside a directory that does not exist: the pipeline succeeds,
  // the atomic archive write fails, and no partial file or temp file survives.
  {
    const std::string missing_dir = topmap_ros::test::UniqueName("t30_missing_dir");
    const std::string bad_output = missing_dir + "/t30_bad.bin";
    auto node = std::make_shared<MapBuilderNode>(Options(pcd, temp.Add("t30_bad.bin")));
    expect_failure(node, Request(pcd, bad_output), ErrorCode::kSerializationIoError, "cannot open");
    EXPECT_FALSE(fs::exists(missing_dir));
    EXPECT_FALSE(fs::exists(bad_output));
  }

  // Both request paths empty and both parameters empty.
  {
    auto node = std::make_shared<MapBuilderNode>(
        rclcpp::NodeOptions().append_parameter_override("config_file",
                                                        topmap_ros::test::validConfigPath()));
    expect_failure(node, std::make_shared<BuildMap::Request>(), ErrorCode::kInvalidArgument,
                   "pcd_path");
  }
}

TEST(MapBuilderNodeTest, DirectBuildRejectsEmptyRequiredStrings) {
  TempFiles temp;
  const std::string pcd = temp.Add("t30_required.pcd");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kUShape, pcd).empty());

  auto loaded = topmap::loadTopmapConfig(topmap_ros::test::validConfigPath());
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;

  const auto empty_pcd = topmap_ros::buildMap("", temp.Add("t30_required.bin"),
                                              loaded.value(), "map");
  ASSERT_FALSE(empty_pcd.ok());
  EXPECT_EQ(empty_pcd.status().code, ErrorCode::kInvalidArgument);
  EXPECT_NE(empty_pcd.status().message.find("pcd_path"), std::string::npos);

  const auto empty_output = topmap_ros::buildMap(pcd, "", loaded.value(), "map");
  ASSERT_FALSE(empty_output.ok());
  EXPECT_EQ(empty_output.status().code, ErrorCode::kInvalidArgument);
  EXPECT_NE(empty_output.status().message.find("output_map_path"), std::string::npos);

  const auto empty_frame =
      topmap_ros::buildMap(pcd, temp.Add("t30_required_frame.bin"), loaded.value(), "");
  ASSERT_FALSE(empty_frame.ok());
  EXPECT_EQ(empty_frame.status().code, ErrorCode::kInvalidArgument);
  EXPECT_NE(empty_frame.status().message.find("frame_id"), std::string::npos);
}

// An unreadable/invalid config must not escape the constructor; the node stays
// callable and reports the preserved configuration error.
TEST(MapBuilderNodeTest, InvalidConfigDoesNotThrowAndReportsFailure) {
  topmap_ros::test::RclcppGuard ros;
  TempFiles temp;
  const std::string pcd = temp.Add("t30_badcfg.pcd");
  const std::string output = temp.Add("t30_badcfg.bin");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kUShape, pcd).empty());

  std::shared_ptr<MapBuilderNode> node;
  ASSERT_NO_THROW(node = std::make_shared<MapBuilderNode>(
                      Options(pcd, output, topmap_ros::test::SourcePath("test/testdata/does_not_exist.yaml"))));
  ASSERT_TRUE(node != nullptr);
  EXPECT_FALSE(node->configOk());
  EXPECT_EQ(node->configStatus().code, ErrorCode::kInvalidArgument);

  auto call = topmap_ros::test::callService<BuildMap>(node, "/topmap/build_map",
                                                      Request(pcd, output));
  ASSERT_TRUE(call.ready) << "build_map call timed out";
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kInvalidArgument));
  EXPECT_NE(call.response->error_message.find(topmap::toString(ErrorCode::kInvalidArgument)),
            std::string::npos);
  EXPECT_FALSE(fs::exists(output));
}

// R4: keep_raw_cloud selects the pre-voxel cloud; disabling it publishes the
// processed (downsampled) cloud.
TEST(MapBuilderNodeTest, KeepRawCloudControlsPublishedCloud) {
  TempFiles temp;
  const std::string pcd = temp.Add("t30_raw.pcd");
  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kUShape, pcd).empty());

  auto loaded = topmap::loadTopmapConfig(topmap_ros::test::validConfigPath());
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;

  topmap::TopmapConfig raw_config = loaded.value();
  raw_config.map.keep_raw_cloud = true;
  auto raw = topmap_ros::buildMap(pcd, temp.Add("t30_raw.bin"), raw_config, "map");
  ASSERT_TRUE(raw.ok()) << raw.status().message;
  EXPECT_TRUE(raw.value().published_cloud_is_raw);
  EXPECT_EQ(raw.value().published_cloud.size(),
            static_cast<std::size_t>(raw.value().archive.metadata.point_count));

  topmap::TopmapConfig processed_config = loaded.value();
  processed_config.map.keep_raw_cloud = false;
  // Force real downsampling while keeping the edge gate wide enough for the
  // coarser lattice to triangulate.
  processed_config.map.voxel_size = 0.20f;
  processed_config.map.max_edge_length = 0.50f;
  auto processed = topmap_ros::buildMap(pcd, temp.Add("t30_processed.bin"), processed_config, "map");
  ASSERT_TRUE(processed.ok()) << processed.status().message;
  EXPECT_FALSE(processed.value().published_cloud_is_raw);
  EXPECT_LT(processed.value().published_cloud.size(),
            static_cast<std::size_t>(processed.value().archive.metadata.point_count));
}

// R7: roughness/curvature computed on a curved cloud must reach the stored
// TriangleNodes, otherwise the traversability criteria would be permanently
// true on those channels.
TEST(MapBuilderNodeTest, PropagatesRoughnessAndCurvatureToTriangles) {
  TempFiles temp;
  const std::string pcd = temp.Add("t30_bumpy.pcd");
  ASSERT_TRUE(
      topmap_ros::test::WritePcd(pcd, topmap_ros::test::MakeBumpyPlaneCloud()));

  auto loaded = topmap::loadTopmapConfig(topmap_ros::test::validConfigPath());
  ASSERT_TRUE(loaded.ok()) << loaded.status().message;
  auto outcome = topmap_ros::buildMap(pcd, temp.Add("t30_bumpy.bin"), loaded.value(), "map");
  ASSERT_TRUE(outcome.ok()) << outcome.status().message;

  bool nonzero_roughness = false;
  bool nonzero_curvature = false;
  for (const topmap::TriangleNode& triangle : outcome.value().archive.mesh.triangles()) {
    if (triangle.roughness > 0.0f) nonzero_roughness = true;
    if (triangle.curvature > 0.0f) nonzero_curvature = true;
  }
  EXPECT_TRUE(nonzero_roughness);
  EXPECT_TRUE(nonzero_curvature);
}
