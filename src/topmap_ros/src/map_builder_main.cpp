// T30: offline map builder executable.
//
// `ros2 run topmap_ros map_builder_node` starts the /topmap/build_map service.
// Parameters (pcd_path, output_map_path, config_file, frame_id) and per-request
// overrides are documented by MapBuilderNode.
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "topmap_ros/map_builder_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<topmap_ros::MapBuilderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
