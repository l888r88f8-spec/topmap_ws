#include "topmap_ros/planner_node.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

#include "topmap_planner/corridor_builder.hpp"
#include "topmap_planner/hierarchical_planner.hpp"
#include "topmap_ros/marker_builder.hpp"
#include "topmap_ros/type_conversions.hpp"
#include "topmap_surface/traversability.hpp"

namespace topmap_ros {
namespace {

using topmap::ErrorCode;
using topmap::PlanResult;
using topmap::Status;

std::string StatusText(const Status& status) {
  std::string text(toString(status.code));
  if (!status.message.empty()) {
    text += ": " + status.message;
  }
  return text;
}

void FillError(const Status& status, topmap_msgs::srv::PlanPath::Response& response) {
  response.success = false;
  response.error_code = static_cast<std::int32_t>(status.code);
  response.error_message = StatusText(status);
}

}  // namespace

topmap::Result<std::string> resolvePlanFrame(const std::string& start_frame,
                                             const std::string& goal_frame,
                                             const std::string& parameter_frame) {
  if (!start_frame.empty() && !goal_frame.empty() && start_frame != goal_frame) {
    return Status::Error(ErrorCode::kInvalidArgument, "start frame '" + start_frame +
                                                          "' and goal frame '" + goal_frame +
                                                          "' are incompatible");
  }
  const std::string request_frame = !start_frame.empty() ? start_frame : goal_frame;
  const std::string resolved = !request_frame.empty() ? request_frame : parameter_frame;
  if (resolved.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "frame_id must not be empty");
  }
  return resolved;
}

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("topmap_planner", options) {
  map_path_ = declare_parameter<std::string>("map_path", "");
  config_file_ = declare_parameter<std::string>("config_file", "");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  auto config = topmap::loadTopmapConfig(config_file_);
  if (config.ok()) {
    config_ = config.take();
    config_ok_ = true;
  } else {
    config_status_ = config.status();
    config_ok_ = false;
  }

  if (map_path_.empty()) {
    map_status_ = Status::Error(ErrorCode::kInvalidArgument, "map_path must not be empty");
    map_ok_ = false;
  } else {
    auto loaded = topmap::loadMapArchive(map_path_);
    if (loaded.ok()) {
      archive_ = loaded.take();
      map_ok_ = true;
      // loadMapArchive restores the per-triangle attributes, but edge
      // traversability is derived state that buildAdjacency leaves false. The
      // planner's surface search needs it, so re-derive it from the archive's
      // stored geometry/features and this node's traversability config.
      if (config_ok_) {
        topmap::TraversabilityEvaluator::apply(archive_.mesh, config_.traversability);
      }
    } else {
      map_status_ = loaded.status();
      map_ok_ = false;
    }
  }

  const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  final_path_pub_ = create_publisher<nav_msgs::msg::Path>("/topmap/final_path", latched);
  // The RViz contract (T32) renders /topmap/topology_path as a MarkerArray; the
  // structured topmap_msgs/TopologyPath travels in the service response and is
  // converted back into this MarkerArray here.
  topology_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/topmap/topology_path", latched);
  corridor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/topmap/corridor", latched);
  start_goal_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/topmap/start_goal", latched);

  service_ = create_service<topmap_msgs::srv::PlanPath>(
      "/topmap/plan_path",
      std::bind(&PlannerNode::handlePlanPath, this, std::placeholders::_1,
                std::placeholders::_2));
}

void PlannerNode::handlePlanPath(
    const std::shared_ptr<topmap_msgs::srv::PlanPath::Request> request,
    std::shared_ptr<topmap_msgs::srv::PlanPath::Response> response) {
  // Preserved construction errors take precedence over the request so the
  // failure mode is identical for every call on a misconfigured node.
  if (!config_ok_) {
    FillError(config_status_, *response);
    return;
  }
  if (!map_ok_) {
    FillError(map_status_, *response);
    return;
  }

  auto frame = resolvePlanFrame(request->start.header.frame_id, request->goal.header.frame_id,
                                frame_id_);
  if (!frame.ok()) {
    FillError(frame.status(), *response);
    return;
  }

  topmap::PlanRequest plan_request;
  plan_request.start = fromMsg(request->start.pose.position);
  plan_request.goal = fromMsg(request->goal.pose.position);

  const topmap::HierarchicalPlanner planner(config_);
  auto planned = planner.plan(archive_.mesh, archive_.graph, plan_request);
  if (!planned.ok()) {
    FillError(planned.status(), *response);
    return;
  }

  const PlanResult& result = planned.value();
  const std::string frame_id = frame.value();
  const rclcpp::Time stamp = now();

  response->success = true;
  response->error_code = 0;
  response->error_message.clear();
  response->path = toMsg(result.path, frame_id, stamp);
  response->topology_path = toMsg(result.topology);
  response->stats = toMsg(result.stats);

  publishPlan(result, plan_request, frame_id, stamp);
}

void PlannerNode::publishPlan(const PlanResult& result, const topmap::PlanRequest& request,
                              const std::string& frame_id, const rclcpp::Time& stamp) {
  final_path_pub_->publish(toMsg(result.path, frame_id, stamp));
  topology_pub_->publish(buildTopologyPathMarkers(archive_.graph, result.topology, frame_id, stamp));
  corridor_pub_->publish(
      buildCorridorMarkers(archive_.mesh, visualizationCorridor(result.topology), frame_id, stamp));
  start_goal_pub_->publish(buildStartGoalMarkers(request.start, request.goal, frame_id, stamp));
}

topmap::Corridor PlannerNode::visualizationCorridor(const topmap::TopologyPath& path) const {
  auto built = topmap::CorridorBuilder().build(archive_.mesh, archive_.graph, path,
                                               config_.plan.corridor_expansion_rings);
  if (built.ok()) return built.take();

  topmap::Corridor corridor;
  corridor.regions = path.regions;
  corridor.portals = path.portals;
  return corridor;
}

}  // namespace topmap_ros
