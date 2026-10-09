// T31: online PlannerNode / /topmap/plan_path service tests.
//
// Every service call follows the R35 executor pattern implemented by
// test_support_ros.hpp::callService: the node under test and a separate client
// node are spun by a MultiThreadedExecutor on a background thread, so the
// service callback and the future wait are never on the same thread.
//
// The ramp archive is produced in-process by BuildRampArchive (the real
// buildMap pipeline), never by shelling out.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include "topmap_core/error.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_ros/planner_node.hpp"
#include "topmap_ros/type_conversions.hpp"
#include "topmap_surface/synthetic_scene.hpp"

#include "nav_msgs/msg/path.hpp"
#include "topmap_msgs/msg/plan_stats.hpp"
#include "topmap_msgs/msg/topology_path.hpp"
#include "topmap_msgs/srv/plan_path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "test_support_ros.hpp"

namespace {

using topmap::ErrorCode;
using topmap::SyntheticScene;
using topmap_msgs::srv::PlanPath;
using topmap_ros::PlannerNode;

rclcpp::NodeOptions Options(const std::string& map,
                            const std::string& config = topmap_ros::test::validConfigPath(),
                            const std::string& frame_id = "map") {
  return rclcpp::NodeOptions()
      .append_parameter_override("map_path", map)
      .append_parameter_override("config_file", config)
      .append_parameter_override("frame_id", frame_id);
}

void SetPose(geometry_msgs::msg::PoseStamped& pose, const Eigen::Vector3f& point,
             const std::string& frame_id) {
  pose.header.frame_id = frame_id;
  pose.pose.position = topmap_ros::toMsg(point);
}

std::shared_ptr<PlanPath::Request> Request(const Eigen::Vector3f& start, const Eigen::Vector3f& goal,
                                           const std::string& start_frame = "",
                                           const std::string& goal_frame = "") {
  auto request = std::make_shared<PlanPath::Request>();
  SetPose(request->start, start, start_frame);
  SetPose(request->goal, goal, goal_frame);
  return request;
}

// A single latched subscriber node that captures the four plan publishers. The
// publishers are transient-local, so a subscriber created after the service call
// still receives the last sample without spinning the server node.
struct CapturedPlan {
  bool got_path = false, got_topology = false, got_corridor = false, got_start_goal = false;
  nav_msgs::msg::Path path;
  visualization_msgs::msg::MarkerArray topology, corridor, start_goal;
};

CapturedPlan CaptureLatchedPlan(std::chrono::milliseconds timeout = std::chrono::seconds(15)) {
  CapturedPlan captured;
  auto sub_node = rclcpp::Node::make_shared(
      "topmap_plan_sub_" + std::to_string(static_cast<long>(::getpid())));
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  auto sub_path = sub_node->create_subscription<nav_msgs::msg::Path>(
      "/topmap/final_path", qos, [&captured](nav_msgs::msg::Path::ConstSharedPtr msg) {
        captured.path = *msg;
        captured.got_path = true;
      });
  auto sub_topology = sub_node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/topmap/topology_path", qos, [&captured](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        captured.topology = *msg;
        captured.got_topology = true;
      });
  auto sub_corridor = sub_node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/topmap/corridor", qos, [&captured](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        captured.corridor = *msg;
        captured.got_corridor = true;
      });
  auto sub_start_goal = sub_node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/topmap/start_goal", qos, [&captured](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        captured.start_goal = *msg;
        captured.got_start_goal = true;
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline &&
         !(captured.got_path && captured.got_topology && captured.got_corridor &&
           captured.got_start_goal)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  executor.remove_node(sub_node);
  return captured;
}

// Runs one service call and asserts the R35 plumbing succeeded. Every test uses
// this so a timeout reports the request that hung rather than an opaque failure.
topmap_ros::test::ServiceCall<PlanPath> Call(const std::shared_ptr<PlannerNode>& node,
                                             const std::shared_ptr<PlanPath::Request>& request,
                                             std::chrono::nanoseconds timeout =
                                                 std::chrono::seconds(60)) {
  auto call = topmap_ros::test::callService<PlanPath>(node, "/topmap/plan_path", request, timeout);
  EXPECT_TRUE(call.service_available) << "/topmap/plan_path is not available";
  EXPECT_TRUE(call.ready) << "plan_path call timed out";
  return call;
}

}  // namespace

// The final-plan ramp example: the node loads the archive, plans start -> goal
// through the hierarchical planner, fills path/topology/stats and publishes all
// four latched visualization topics in the resolved frame.
TEST(PlannerNodeTest, PlansRampArchiveAndPublishesLatchedVisualization) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;
  const std::string map = topmap_ros::test::BuildRampArchive(temp.Add("t31_ramp.bin"));
  ASSERT_FALSE(map.empty()) << "failed to build the ramp test archive";

  auto node = std::make_shared<PlannerNode>(Options(map, topmap_ros::test::validConfigPath(), "ramp_map"));
  ASSERT_TRUE(node->mapOk()) << node->mapStatus().message;
  ASSERT_TRUE(node->configOk()) << node->configStatus().message;

  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok()) << endpoints.status().message;
  auto call = Call(node, Request(endpoints.value().first, endpoints.value().second));
  ASSERT_TRUE(call.response != nullptr);
  const PlanPath::Response& response = *call.response;
  ASSERT_TRUE(response.success) << response.error_message;
  EXPECT_EQ(response.error_code, 0);
  EXPECT_TRUE(response.error_message.empty());

  // Path is populated with at least the two endpoints, in the parameter frame.
  ASSERT_GT(response.path.poses.size(), 1u);
  EXPECT_EQ(response.path.header.frame_id, "ramp_map");
  for (const geometry_msgs::msg::PoseStamped& pose : response.path.poses) {
    EXPECT_EQ(pose.header.frame_id, "ramp_map");
  }

  // Topology path is a strictly alternating Region/Portal sequence.
  ASSERT_FALSE(response.topology_path.region_ids.empty());
  EXPECT_EQ(response.topology_path.region_ids.size(), response.topology_path.portal_ids.size() + 1u);

  // Stats describe the returned path.
  EXPECT_GT(response.stats.path_length, 0.0f);
  EXPECT_GE(response.stats.regions_in_corridor, 1u);
  EXPECT_GE(response.stats.triangles_in_corridor, 1u);

  const CapturedPlan captured = CaptureLatchedPlan();
  ASSERT_TRUE(captured.got_path);
  ASSERT_TRUE(captured.got_topology);
  ASSERT_TRUE(captured.got_corridor);
  ASSERT_TRUE(captured.got_start_goal);
  EXPECT_EQ(captured.path.header.frame_id, "ramp_map");
  EXPECT_EQ(captured.path.poses.size(), response.path.poses.size());
  EXPECT_FALSE(captured.topology.markers.empty());
  EXPECT_FALSE(captured.corridor.markers.empty());
  ASSERT_EQ(captured.start_goal.markers.size(), 2u);
  for (const visualization_msgs::msg::Marker& marker : captured.topology.markers) {
    EXPECT_EQ(marker.header.frame_id, "ramp_map");
  }
  for (const visualization_msgs::msg::Marker& marker : captured.corridor.markers) {
    EXPECT_EQ(marker.header.frame_id, "ramp_map");
  }
  for (const visualization_msgs::msg::Marker& marker : captured.start_goal.markers) {
    EXPECT_EQ(marker.header.frame_id, "ramp_map");
  }
}

// The request frame wins over the parameter frame, on both start and goal.
TEST(PlannerNodeTest, RequestHeaderFrameOverridesParameter) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;
  const std::string map = topmap_ros::test::BuildRampArchive(temp.Add("t31_frame.bin"));
  ASSERT_FALSE(map.empty());

  auto node = std::make_shared<PlannerNode>(Options(map, topmap_ros::test::validConfigPath(), "map"));
  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok());
  auto call = Call(node, Request(endpoints.value().first, endpoints.value().second, "sensor", "sensor"));
  ASSERT_TRUE(call.response != nullptr);
  ASSERT_TRUE(call.response->success) << call.response->error_message;
  EXPECT_EQ(call.response->path.header.frame_id, "sensor");
  for (const geometry_msgs::msg::PoseStamped& pose : call.response->path.poses) {
    EXPECT_EQ(pose.header.frame_id, "sensor");
  }
  // A non-empty request frame also decides the visualization frame.
  const CapturedPlan captured = CaptureLatchedPlan();
  ASSERT_TRUE(captured.got_path);
  EXPECT_EQ(captured.path.header.frame_id, "sensor");
  ASSERT_TRUE(captured.got_start_goal);
  ASSERT_EQ(captured.start_goal.markers.size(), 2u);
  EXPECT_EQ(captured.start_goal.markers.front().header.frame_id, "sensor");
}

// start and goal may carry at most one distinct frame; a contradiction is
// rejected before the planner runs.
TEST(PlannerNodeTest, IncompatibleStartGoalFramesRejected) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;
  const std::string map = topmap_ros::test::BuildRampArchive(temp.Add("t31_mismatch.bin"));
  ASSERT_FALSE(map.empty());

  auto node = std::make_shared<PlannerNode>(Options(map));
  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok());
  auto call = Call(node, Request(endpoints.value().first, endpoints.value().second, "start_frame", "goal_frame"));
  ASSERT_TRUE(call.response != nullptr);
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kInvalidArgument));
  EXPECT_NE(call.response->error_message.find(topmap::toString(ErrorCode::kInvalidArgument)),
            std::string::npos);
  EXPECT_NE(call.response->error_message.find("frame"), std::string::npos);
}

// No request frame and an empty parameter frame is a required-value error.
TEST(PlannerNodeTest, EmptyFrameEverywhereRejected) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;
  const std::string map = topmap_ros::test::BuildRampArchive(temp.Add("t31_emptyframe.bin"));
  ASSERT_FALSE(map.empty());

  auto node = std::make_shared<PlannerNode>(Options(map, topmap_ros::test::validConfigPath(), ""));
  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok());
  auto call = Call(node, Request(endpoints.value().first, endpoints.value().second));
  ASSERT_TRUE(call.response != nullptr);
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kInvalidArgument));
  EXPECT_NE(call.response->error_message.find("frame_id"), std::string::npos);
}

// A missing/invalid archive must not escape the constructor: the node stays
// callable and reports the preserved structured load error.
TEST(PlannerNodeTest, InvalidMapArchiveDoesNotThrowAndReportsFailure) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;

  std::shared_ptr<PlannerNode> node;
  const std::string missing = temp.Add("t31_does_not_exist.bin");
  ASSERT_NO_THROW(node = std::make_shared<PlannerNode>(Options(missing)));
  ASSERT_TRUE(node != nullptr);
  EXPECT_FALSE(node->mapOk());
  EXPECT_EQ(node->mapStatus().code, ErrorCode::kSerializationIoError);

  auto call = Call(node, Request(Eigen::Vector3f::Zero(), Eigen::Vector3f(1.0f, 0.0f, 0.0f)));
  ASSERT_TRUE(call.response != nullptr);
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kSerializationIoError));
  EXPECT_NE(call.response->error_message.find(topmap::toString(ErrorCode::kSerializationIoError)),
            std::string::npos);
}

// An empty map_path is rejected as a required value before touching the file
// system, and the node remains callable.
TEST(PlannerNodeTest, EmptyMapPathRejected) {
  topmap_ros::test::RclcppGuard ros;
  auto node = std::make_shared<PlannerNode>(Options(""));
  EXPECT_FALSE(node->mapOk());
  EXPECT_EQ(node->mapStatus().code, ErrorCode::kInvalidArgument);

  auto call = Call(node, Request(Eigen::Vector3f::Zero(), Eigen::Vector3f(1.0f, 0.0f, 0.0f)));
  ASSERT_TRUE(call.response != nullptr);
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kInvalidArgument));
  EXPECT_NE(call.response->error_message.find("map_path"), std::string::npos);
}

// An unreadable config must not escape the constructor; the preserved config
// error is reported instead of the (valid) archive being planned.
TEST(PlannerNodeTest, InvalidConfigDoesNotThrowAndReportsFailure) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;
  const std::string map = topmap_ros::test::BuildRampArchive(temp.Add("t31_badcfg.bin"));
  ASSERT_FALSE(map.empty());

  std::shared_ptr<PlannerNode> node;
  ASSERT_NO_THROW(node = std::make_shared<PlannerNode>(
                      Options(map, topmap_ros::test::SourcePath("test/testdata/does_not_exist.yaml"))));
  ASSERT_TRUE(node != nullptr);
  EXPECT_TRUE(node->mapOk());
  EXPECT_FALSE(node->configOk());
  EXPECT_EQ(node->configStatus().code, ErrorCode::kInvalidArgument);

  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok());
  auto call = Call(node, Request(endpoints.value().first, endpoints.value().second));
  ASSERT_TRUE(call.response != nullptr);
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kInvalidArgument));
  EXPECT_NE(call.response->error_message.find(topmap::toString(ErrorCode::kInvalidArgument)),
            std::string::npos);
}

// A start point far from the surface is a structured planner failure, not a
// crash or an opaque error.
TEST(PlannerNodeTest, StartOffSurfaceReportsStructuredError) {
  topmap_ros::test::RclcppGuard ros;
  topmap_ros::test::TempFiles temp;
  const std::string map = topmap_ros::test::BuildRampArchive(temp.Add("t31_offsurface.bin"));
  ASSERT_FALSE(map.empty());

  auto node = std::make_shared<PlannerNode>(Options(map));
  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kRamp);
  ASSERT_TRUE(endpoints.ok());
  auto call = Call(node, Request(Eigen::Vector3f(100.0f, 100.0f, 100.0f), endpoints.value().second));
  ASSERT_TRUE(call.response != nullptr);
  EXPECT_FALSE(call.response->success);
  EXPECT_EQ(call.response->error_code, static_cast<std::int32_t>(ErrorCode::kStartNotOnSurface));
  EXPECT_NE(call.response->error_message.find(topmap::toString(ErrorCode::kStartNotOnSurface)),
            std::string::npos);
}

// The ROS-free frame resolver keeps the request/parameter precedence and the
// mismatch/empty rules directly testable.
TEST(PlannerNodeTest, ResolvePlanFramePrefersRequestThenParameter) {
  const auto from_start = topmap_ros::resolvePlanFrame("sensor", "", "map");
  ASSERT_TRUE(from_start.ok());
  EXPECT_EQ(from_start.value(), "sensor");

  const auto from_goal = topmap_ros::resolvePlanFrame("", "goal_frame", "map");
  ASSERT_TRUE(from_goal.ok());
  EXPECT_EQ(from_goal.value(), "goal_frame");

  const auto equal = topmap_ros::resolvePlanFrame("odom", "odom", "map");
  ASSERT_TRUE(equal.ok());
  EXPECT_EQ(equal.value(), "odom");

  const auto from_parameter = topmap_ros::resolvePlanFrame("", "", "map");
  ASSERT_TRUE(from_parameter.ok());
  EXPECT_EQ(from_parameter.value(), "map");

  const auto mismatch = topmap_ros::resolvePlanFrame("a", "b", "map");
  ASSERT_FALSE(mismatch.ok());
  EXPECT_EQ(mismatch.status().code, ErrorCode::kInvalidArgument);

  const auto empty = topmap_ros::resolvePlanFrame("", "", "");
  ASSERT_FALSE(empty.ok());
  EXPECT_EQ(empty.status().code, ErrorCode::kInvalidArgument);
  EXPECT_NE(empty.status().message.find("frame_id"), std::string::npos);
}
