// T32: ROS build-map -> plan-path end-to-end closed loop on the synthetic Hall.
//
// This is the T32 acceptance closure: the offline MapBuilderNode turns a Hall
// PCD into a versioned archive through /topmap/build_map, and the online
// PlannerNode loads exactly that archive and serves /topmap/plan_path. Both
// service calls use the R35 helper in test_support_ros.hpp (separate client node
// on a MultiThreadedExecutor with a background spin thread), so a same-thread
// future wait can never deadlock. Scratch files are owned by TempFiles and
// removed even when an assertion fails; no stage shells out.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include "topmap_core/error.hpp"
#include "topmap_ros/map_builder_node.hpp"
#include "topmap_ros/planner_node.hpp"
#include "topmap_ros/type_conversions.hpp"
#include "topmap_surface/synthetic_scene.hpp"

#include "topmap_msgs/srv/build_map.hpp"
#include "topmap_msgs/srv/plan_path.hpp"

#include "test_support_ros.hpp"

namespace {

using topmap::SyntheticScene;
using topmap_ros::MapBuilderNode;
using topmap_ros::PlannerNode;
using topmap_ros::test::callService;
using topmap_ros::test::RclcppGuard;
using topmap_ros::test::TempFiles;
using topmap_msgs::srv::BuildMap;
using topmap_msgs::srv::PlanPath;

// Structured diagnostics: the ROS response carries both the numeric ErrorCode
// and the human-readable "toString(code): message" text. A bare EXPECT would
// otherwise hide which stage failed.
std::string DescribeFailure(std::int32_t code, const std::string& message) {
  return "error_code=" + std::to_string(code) + " error_message=" + message;
}

}  // namespace

TEST(EndToEndTest, BuildThenPlanClosedLoopOnHall) {
  RclcppGuard ros;
  TempFiles temp;
  const std::string pcd = temp.Add("t32_e2e_hall.pcd");
  const std::string map = temp.Add("t32_e2e_hall.bin");

  ASSERT_FALSE(topmap_ros::test::WriteScenePcd(SyntheticScene::kHallNarrowHall, pcd).empty())
      << "failed to write the synthetic Hall PCD";

  // Step 1: offline build through the real /topmap/build_map service.
  auto builder = std::make_shared<MapBuilderNode>(
      rclcpp::NodeOptions()
          .append_parameter_override("pcd_path", pcd)
          .append_parameter_override("output_map_path", map)
          .append_parameter_override("config_file", topmap_ros::test::validConfigPath())
          .append_parameter_override("frame_id", "map"));
  ASSERT_TRUE(builder->configOk()) << builder->configStatus().message;

  auto build_request = std::make_shared<BuildMap::Request>();
  build_request->pcd_path = pcd;
  build_request->output_map_path = map;
  auto build = callService<BuildMap>(builder, "/topmap/build_map", build_request,
                                     std::chrono::seconds(60));
  ASSERT_TRUE(build.service_available) << "/topmap/build_map is not available";
  ASSERT_TRUE(build.ready) << "build_map call timed out";
  ASSERT_TRUE(build.response != nullptr);
  ASSERT_TRUE(build.response->success)
      << "build_map failed: " << DescribeFailure(build.response->error_code,
                                                 build.response->error_message);
  EXPECT_GT(build.response->stats.region_count, 0u);
  EXPECT_GT(build.response->stats.portal_count, 0u);

  // Step 2: online planning from exactly the archive the builder wrote.
  auto planner = std::make_shared<PlannerNode>(
      rclcpp::NodeOptions()
          .append_parameter_override("map_path", map)
          .append_parameter_override("config_file", topmap_ros::test::validConfigPath())
          .append_parameter_override("frame_id", "map"));
  ASSERT_TRUE(planner->configOk()) << planner->configStatus().message;
  ASSERT_TRUE(planner->mapOk()) << planner->mapStatus().message;

  const auto endpoints = topmap::makeSyntheticEndpoints(SyntheticScene::kHallNarrowHall);
  ASSERT_TRUE(endpoints.ok()) << endpoints.status().message;

  auto plan_request = std::make_shared<PlanPath::Request>();
  plan_request->start.header.frame_id = "map";
  plan_request->start.pose.position = topmap_ros::toMsg(endpoints.value().first);
  plan_request->goal.header.frame_id = "map";
  plan_request->goal.pose.position = topmap_ros::toMsg(endpoints.value().second);
  auto plan = callService<PlanPath>(planner, "/topmap/plan_path", plan_request,
                                    std::chrono::seconds(60));
  ASSERT_TRUE(plan.service_available) << "/topmap/plan_path is not available";
  ASSERT_TRUE(plan.ready) << "plan_path call timed out";
  ASSERT_TRUE(plan.response != nullptr);
  ASSERT_TRUE(plan.response->success)
      << "plan_path failed: " << DescribeFailure(plan.response->error_code,
                                                 plan.response->error_message);

  EXPECT_GT(plan.response->path.poses.size(), 1u);
  EXPECT_GT(plan.response->stats.path_length, 0.0f);
  // R36: the acceptance yardstick is the robot-derived required_clearance
  // (0.5*width + safety_distance = 0.45), not the trivially-true ">= 0".
  EXPECT_GE(plan.response->stats.min_clearance, 0.45f)
      << "min_clearance=" << plan.response->stats.min_clearance
      << " is below robot.required_clearance()";
}
