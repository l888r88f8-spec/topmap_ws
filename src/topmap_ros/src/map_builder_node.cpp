#include "topmap_ros/map_builder_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>

#include "topmap_ros/marker_builder.hpp"
#include "topmap_ros/type_conversions.hpp"
#include "topmap_surface/point_cloud_processor.hpp"
#include "topmap_surface/surface_features.hpp"
#include "topmap_surface/traversability.hpp"
#include "topmap_surface/triangulator.hpp"
#include "topmap_topology/portal_extractor.hpp"
#include "topmap_topology/portal_validator.hpp"
#include "topmap_topology/region_growing.hpp"
#include "topmap_topology/region_split.hpp"
#include "topmap_topology/topology_compressor.hpp"
#include "topmap_topology/topology_graph.hpp"

namespace topmap_ros {
namespace {

using topmap::CompressionStats;
using topmap::ErrorCode;
using topmap::MapArchive;
using topmap::MapBuildParams;
using topmap::MeshBuildParams;
using topmap::MeshData;
using topmap::Portal;
using topmap::RegionId;
using topmap::Result;
using topmap::Status;
using topmap::SurfaceRegion;
using topmap::TopologyGraph;
using topmap::TriangleMesh;
using topmap::TriangleNode;

// The planner fixture's legitimate-empty-portals rule: PortalExtractor reports
// kPortalExtractionFailed when the whole partition has no cross-region shared
// edge (single-region-per-component scenes); that is not an error, so the graph
// is built with an empty portal set. Any extractor failure that does hide shared
// edges is propagated.
bool HasCrossRegionSharedEdge(const TriangleMesh& mesh,
                              const std::vector<SurfaceRegion>& regions) {
  std::vector<RegionId> owner(static_cast<std::size_t>(mesh.size()), topmap::kInvalidId);
  for (const SurfaceRegion& region : regions) {
    for (topmap::TriangleId triangle : region.triangle_ids) {
      if (triangle >= 0 && triangle < static_cast<topmap::TriangleId>(mesh.size())) {
        owner[static_cast<std::size_t>(triangle)] = region.id;
      }
    }
  }
  for (topmap::TriangleId triangle = 0;
       triangle < static_cast<topmap::TriangleId>(mesh.size()); ++triangle) {
    if (owner[static_cast<std::size_t>(triangle)] == topmap::kInvalidId) continue;
    for (const topmap::TriangleEdge& edge : mesh.triangle(triangle).edges) {
      if (edge.neighbor == topmap::kInvalidId || edge.neighbor < triangle) continue;
      const RegionId other = owner[static_cast<std::size_t>(edge.neighbor)];
      if (other != topmap::kInvalidId &&
          other != owner[static_cast<std::size_t>(triangle)]) {
        return true;
      }
    }
  }
  return false;
}

// R14: deterministic approximate archive footprint in MiB, using the same
// fixed-record widths as the T20 serializer. It is a pure function of the final
// archive contents, so tests and users can reason about it without measuring the
// process.
double ApproximateMemoryMb(const MapArchive& archive) {
  std::uint64_t bytes = 0;
  bytes += archive.mesh.vertices().size() * (3u * 4u);
  bytes += static_cast<std::uint64_t>(archive.mesh.size()) * (3u * 4u + 6u * 4u + 5u * 4u + 1u);
  for (const SurfaceRegion& region : archive.graph.regions()) {
    bytes += 4u + 4u + region.triangle_ids.size() * 4u + 12u + 7u * 4u + 1u + 1u + 4u +
             region.portal_ids.size() * 4u;
  }
  for (const Portal& portal : archive.graph.portals()) {
    bytes += 3u * 4u + 4u + portal.boundary_chain.size() * 4u + 3u * 12u + 6u * 4u + 1u;
  }
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

std::string StatusText(const Status& status) {
  std::string text(toString(status.code));
  if (!status.message.empty()) {
    text += ": " + status.message;
  }
  return text;
}

void FillError(const Status& status, topmap_msgs::srv::BuildMap::Response& response) {
  response.success = false;
  response.error_code = static_cast<std::int32_t>(status.code);
  response.error_message = StatusText(status);
}

}  // namespace

Result<MapBuildOutcome> buildMap(const std::string& pcd_path,
                                 const std::string& output_map_path,
                                 const topmap::TopmapConfig& config,
                                 const std::string& frame_id) {
  if (pcd_path.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "pcd_path must not be empty");
  }
  if (output_map_path.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "output_map_path must not be empty");
  }
  if (frame_id.empty()) {
    return Status::Error(ErrorCode::kInvalidArgument, "frame_id must not be empty");
  }

  const auto started = std::chrono::steady_clock::now();

  auto loaded = topmap::PointCloudProcessor::loadPCD(pcd_path);
  if (!loaded.ok()) return loaded.status();
  const pcl::PointCloud<pcl::PointXYZ> original = loaded.take();

  auto processed = topmap::PointCloudProcessor::process(original, config.map);
  if (!processed.ok()) return processed.status();

  auto features = topmap::SurfaceFeatureEstimator::estimate(processed.value(), config.map);
  if (!features.ok()) return features.status();

  // R7: carry roughness/curvature through the triangulator into MeshData and
  // then into TriangleNode, otherwise those traversability criteria are always
  // true and the region join roughness term is always zero.
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> normals;
  std::vector<float> roughness;
  std::vector<float> curvature;
  points.reserve(features.value().size());
  normals.reserve(features.value().size());
  roughness.reserve(features.value().size());
  curvature.reserve(features.value().size());
  for (const topmap::SurfaceFeature& feature : features.value()) {
    points.push_back(feature.position);
    normals.push_back(feature.normal);
    roughness.push_back(feature.roughness);
    curvature.push_back(feature.curvature);
  }

  MeshBuildParams mesh_params;
  mesh_params.max_edge_length = config.map.max_edge_length;
  mesh_params.max_normal_deviation_deg = config.map.max_normal_deviation_deg;
  mesh_params.prefer_layered = config.map.prefer_layered_triangulation;

  auto triangulated = topmap::SurfaceTriangulator::triangulate(points, normals, mesh_params,
                                                               &roughness, &curvature);
  if (!triangulated.ok()) return triangulated.status();
  const MeshData& mesh_data = triangulated.value().mesh;
  if (mesh_data.faces.empty()) {
    return Status::Error(ErrorCode::kMeshEmpty, "triangulation produced no faces");
  }

  auto mesh_result = TriangleMesh::fromMeshData(mesh_data, mesh_params);
  if (!mesh_result.ok()) return mesh_result.status();
  TriangleMesh mesh = mesh_result.take();

  topmap::TraversabilityEvaluator::apply(mesh, config.traversability);

  auto grown = topmap::RegionGrower().grow(mesh, config.region);
  if (!grown.ok()) return grown.status();

  auto split = topmap::RegionSplitter().split(mesh, grown.value(), config.region);
  if (!split.ok()) return split.status();
  std::vector<SurfaceRegion> regions = split.take();

  std::vector<Portal> portals;
  auto extracted = topmap::PortalExtractor().extract(mesh, regions, config.portal);
  if (extracted.ok()) {
    portals = extracted.take();
  } else if (extracted.status().code == ErrorCode::kPortalExtractionFailed &&
             !HasCrossRegionSharedEdge(mesh, regions)) {
    portals.clear();  // Legitimate: no inter-region shared edge.
  } else {
    return extracted.status();
  }

  if (!portals.empty()) {
    auto validated =
        topmap::PortalValidator().validate(mesh, regions, portals, config.robot, config.portal);
    if (!validated.ok()) return validated.status();
    portals = validated.take();
  }

  auto graph_result = TopologyGraph::build(regions, portals);
  if (!graph_result.ok()) return graph_result.status();
  TopologyGraph graph = graph_result.take();

  // R12-b: compression takes both the compression and the region parameters, so
  // cross-slope-regime merges can be forbidden structurally.
  auto compressed = topmap::TopologyCompressor().compress(graph, config.compression, config.region);
  if (!compressed.ok()) return compressed.status();
  auto compressed_pair = compressed.take();
  TopologyGraph final_graph = std::move(compressed_pair.first);
  const CompressionStats compression_stats = compressed_pair.second;
  (void)compression_stats;

  MapArchive archive;
  archive.format_version = topmap::kTopmapFormatVersion;
  archive.metadata.frame_id = frame_id;
  archive.metadata.robot = config.robot;
  archive.metadata.map = config.map;
  archive.metadata.point_count = static_cast<std::uint32_t>(original.size());
  archive.metadata.triangle_count = static_cast<std::uint32_t>(mesh.size());
  archive.metadata.region_count = static_cast<std::uint32_t>(final_graph.regions().size());
  archive.metadata.portal_count = static_cast<std::uint32_t>(final_graph.portals().size());
  archive.metadata.connected_components =
      static_cast<std::uint32_t>(final_graph.connectedComponents());
  archive.metadata.regions_before_compression = static_cast<std::uint32_t>(graph.regions().size());
  const auto finished = std::chrono::steady_clock::now();
  archive.metadata.build_time_sec =
      std::chrono::duration<double>(finished - started).count();
  archive.mesh = std::move(mesh);
  archive.graph = std::move(final_graph);
  archive.metadata.memory_mb = ApproximateMemoryMb(archive);

  // The archive is written exactly once, after the whole pipeline succeeded, so
  // a failure never leaves a partial output file (T20 also writes atomically).
  const Status saved = topmap::saveMapArchive(output_map_path, archive);
  if (!saved.ok()) return saved;

  MapBuildOutcome outcome;
  outcome.archive = std::move(archive);
  if (config.map.keep_raw_cloud) {
    // R4: retain and publish the pre-voxel-loaded cloud. The frozen
    // PortalValidator API cannot consume it, so it is only exposed for the
    // raw-resolution rechecks; it is not silently claimed as validator input.
    outcome.published_cloud = original;
    outcome.published_cloud_is_raw = true;
  } else {
    outcome.published_cloud = processed.value();
    outcome.published_cloud_is_raw = false;
  }
  return outcome;
}

MapBuilderNode::MapBuilderNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("topmap_map_builder", options) {
  pcd_path_ = declare_parameter<std::string>("pcd_path", "");
  output_map_path_ = declare_parameter<std::string>("output_map_path", "");
  config_file_ = declare_parameter<std::string>("config_file", "");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  auto config = topmap::loadTopmapConfig(config_file_);
  if (config.ok()) {
    config_ = config.take();
    config_ok_ = true;
  } else {
    // Preserve the structured error so the service stays callable and every
    // build reports it instead of the process aborting at construction.
    config_status_ = config.status();
    config_ok_ = false;
  }

  const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/topmap/original_cloud", latched);
  region_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/topmap/regions", latched);
  portal_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/topmap/portals", latched);
  stats_pub_ = create_publisher<topmap_msgs::msg::MapStats>("/topmap/map_stats", latched);

  service_ = create_service<topmap_msgs::srv::BuildMap>(
      "/topmap/build_map",
      std::bind(&MapBuilderNode::handleBuildMap, this, std::placeholders::_1,
                std::placeholders::_2));
}

void MapBuilderNode::handleBuildMap(
    const std::shared_ptr<topmap_msgs::srv::BuildMap::Request> request,
    std::shared_ptr<topmap_msgs::srv::BuildMap::Response> response) {
  const std::string pcd_path = request->pcd_path.empty() ? pcd_path_ : request->pcd_path;
  const std::string output_map_path =
      request->output_map_path.empty() ? output_map_path_ : request->output_map_path;

  if (!config_ok_) {
    FillError(config_status_, *response);
    return;
  }
  if (pcd_path.empty()) {
    FillError(Status::Error(ErrorCode::kInvalidArgument, "pcd_path must not be empty"), *response);
    return;
  }
  if (output_map_path.empty()) {
    FillError(Status::Error(ErrorCode::kInvalidArgument, "output_map_path must not be empty"),
              *response);
    return;
  }

  auto built = buildMap(pcd_path, output_map_path, config_, frame_id_);
  if (!built.ok()) {
    FillError(built.status(), *response);
    return;
  }

  const MapBuildOutcome& outcome = built.value();
  publishOutcome(outcome);
  response->success = true;
  response->stats = toMsg(outcome.archive.metadata);
  response->error_code = 0;
  response->error_message.clear();
}

void MapBuilderNode::publishOutcome(const MapBuildOutcome& outcome) {
  const rclcpp::Time stamp = now();

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(outcome.published_cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.header.stamp = stamp;
  cloud_pub_->publish(cloud_msg);

  region_pub_->publish(
      buildRegionMarkers(outcome.archive.mesh, outcome.archive.graph, frame_id_, stamp));
  portal_pub_->publish(buildPortalMarkers(outcome.archive.graph, frame_id_, stamp));
  stats_pub_->publish(toMsg(outcome.archive.metadata));
}

}  // namespace topmap_ros
