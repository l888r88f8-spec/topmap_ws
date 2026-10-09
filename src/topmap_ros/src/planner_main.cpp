// T31: online planner executable.
//
// `ros2 run topmap_ros planner_node` starts the /topmap/plan_path service.
// Parameters (map_path, config_file, frame_id) and the request semantics are
// documented by PlannerNode.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "topmap_ros/planner_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<topmap_ros::PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
