#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"
#include "topmap_topology/map_archive.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "topmap_msgs/msg/map_stats.hpp"
#include "topmap_msgs/srv/build_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// T30: offline map builder. `buildMap` is the ROS-free pipeline (A4.6-A4.8 plus
// T20 serialization) and is directly unit-testable; `MapBuilderNode` wraps it in
// the /topmap/build_map service and the latched map publishers.
namespace topmap_ros {

// Result of a successful offline build. `published_cloud` is the raw pre-voxel
// cloud when config.map.keep_raw_cloud is set (R4) and the processed cloud
// otherwise; `published_cloud_is_raw` states which one it is.
struct MapBuildOutcome {
  topmap::MapArchive archive;
  pcl::PointCloud<pcl::PointXYZ> published_cloud;
  bool published_cloud_is_raw = false;
};

// Runs loadPCD -> process -> estimate -> triangulate (R7 feature pointers) ->
// fromMeshData -> traversability -> grow -> split -> extract -> validate ->
// TopologyGraph::build -> compress (R12-b) -> saveMapArchive.
//
// The archive is written exactly once, after the whole pipeline succeeds, so a
// failure never leaves a partial output file. Every stage failure is returned as
// the stage's structured Status.
topmap::Result<MapBuildOutcome> buildMap(const std::string& pcd_path,
                                         const std::string& output_map_path,
                                         const topmap::TopmapConfig& config,
                                         const std::string& frame_id);

class MapBuilderNode : public rclcpp::Node {
 public:
  explicit MapBuilderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // False when the configured YAML could not be loaded at construction. The
  // node stays alive and callable; every build then reports configStatus().
  bool configOk() const { return config_ok_; }
  const topmap::Status& configStatus() const { return config_status_; }

 private:
  void handleBuildMap(const std::shared_ptr<topmap_msgs::srv::BuildMap::Request> request,
                      std::shared_ptr<topmap_msgs::srv::BuildMap::Response> response);
  void publishOutcome(const MapBuildOutcome& outcome);

  std::string pcd_path_;
  std::string output_map_path_;
  std::string config_file_;
  std::string frame_id_;

  bool config_ok_ = false;
  topmap::Status config_status_;
  topmap::TopmapConfig config_;

  rclcpp::Service<topmap_msgs::srv::BuildMap>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr portal_pub_;
  rclcpp::Publisher<topmap_msgs::msg::MapStats>::SharedPtr stats_pub_;
};

}  // namespace topmap_ros
