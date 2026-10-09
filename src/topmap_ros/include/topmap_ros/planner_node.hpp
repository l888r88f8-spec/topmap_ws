#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_planner/plan_types.hpp"
#include "topmap_topology/map_archive.hpp"

#include "nav_msgs/msg/path.hpp"
#include "topmap_msgs/srv/plan_path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// T31: online planner. `PlannerNode` loads a map archive and a TopmapConfig once
// at construction and serves /topmap/plan_path by delegating to the frozen
// HierarchicalPlanner. A failed archive or config load is preserved as a
// structured Status, so the node stays alive and callable and every request
// returns that error instead of the process aborting.
namespace topmap_ros {

// Resolves the single frame used for planning and visualization:
//   * start and goal may carry at most one distinct non-empty frame - a
//     contradiction is kInvalidArgument (there is no TF lookup in this node);
//   * a non-empty request frame wins over the `frame_id` parameter;
//   * an empty result is kInvalidArgument, because every published header and
//     the returned path must name a frame.
// ROS-free so it is directly unit-testable without rclcpp::init().
topmap::Result<std::string> resolvePlanFrame(const std::string& start_frame,
                                             const std::string& goal_frame,
                                             const std::string& parameter_frame);

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // False when the configured archive could not be loaded at construction; the
  // preserved error is then reported by every plan request.
  bool mapOk() const { return map_ok_; }
  const topmap::Status& mapStatus() const { return map_status_; }

  // False when the configured YAML could not be loaded at construction; the
  // preserved error is then reported by every plan request.
  bool configOk() const { return config_ok_; }
  const topmap::Status& configStatus() const { return config_status_; }

 private:
  void handlePlanPath(const std::shared_ptr<topmap_msgs::srv::PlanPath::Request> request,
                      std::shared_ptr<topmap_msgs::srv::PlanPath::Response> response);
  void publishPlan(const topmap::PlanResult& result, const topmap::PlanRequest& request,
                   const std::string& frame_id, const rclcpp::Time& stamp);
  // The corridor the /topmap/corridor markers show. It is rebuilt from the
  // frozen CorridorBuilder because PlanResult does not carry it; if that build
  // fails (the planner may have used a degraded corridor) the topology path's
  // Regions/Portals are published so RViz still shows the returned plan.
  topmap::Corridor visualizationCorridor(const topmap::TopologyPath& path) const;

  std::string map_path_;
  std::string config_file_;
  std::string frame_id_;

  bool map_ok_ = false;
  topmap::Status map_status_;
  bool config_ok_ = false;
  topmap::Status config_status_;

  topmap::TopmapConfig config_;
  topmap::MapArchive archive_;

  rclcpp::Service<topmap_msgs::srv::PlanPath>::SharedPtr service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr final_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topology_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr start_goal_pub_;
};

}  // namespace topmap_ros
