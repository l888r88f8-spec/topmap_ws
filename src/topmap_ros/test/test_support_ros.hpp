// T30/R35: shared test support for the topmap_ros service nodes.
//
// Everything here is test-only and lives under test/, so it is never installed
// nor linked into a production target. It provides the three helpers the T30/T31
// plan calls out: a safe rclcpp init/shutdown RAII guard, absolute test-data
// paths and a binary PCD writer, and the R35 service-call helper that runs a
// separate client node on a MultiThreadedExecutor with a background spin thread.
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include "topmap_core/error.hpp"
#include "topmap_surface/mesh_types.hpp"
#include "topmap_surface/synthetic_scene.hpp"

#ifndef TOPMAP_ROS_TEST_SOURCE_DIR
#error "TOPMAP_ROS_TEST_SOURCE_DIR must be defined by CMake for the topmap_ros tests"
#endif

namespace topmap_ros {
namespace test {

// R35: rclcpp::init on construction, rclcpp::shutdown on destruction. The guard
// is idempotent when a test binary already holds an initialised context.
class RclcppGuard {
 public:
  RclcppGuard() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  ~RclcppGuard() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
  RclcppGuard(const RclcppGuard&) = delete;
  RclcppGuard& operator=(const RclcppGuard&) = delete;
};

// Absolute path of any file under the topmap_ros source test directory. Building
// it from the CMake-provided compile definition keeps tests independent of the
// process working directory.
inline std::string SourcePath(const std::string& relative) {
  return std::string(TOPMAP_ROS_TEST_SOURCE_DIR) + "/" + relative;
}

inline std::string validConfigPath() { return SourcePath("test/testdata/topmap_valid.yaml"); }

// A process-unique stem so parallel/repeated tests never collide and no test
// depends on the working directory being clean.
inline std::string UniqueName(const std::string& stem) {
  static std::atomic<unsigned long> counter{0};
  return stem + "." + std::to_string(static_cast<long>(::getpid())) + "." +
         std::to_string(counter.fetch_add(1));
}

// RAII scratch files: every Add()-ed path is removed on destruction (best
// effort), which keeps the build/test cwd free of residue.
class TempFiles {
 public:
  TempFiles() = default;
  ~TempFiles() { Clear(); }
  TempFiles(const TempFiles&) = delete;
  TempFiles& operator=(const TempFiles&) = delete;

  std::string Add(const std::string& stem) {
    std::string path = UniqueName(stem);
    paths_.push_back(path);
    return path;
  }

  void Clear() {
    for (const std::string& path : paths_) std::remove(path.c_str());
    paths_.clear();
  }

 private:
  std::vector<std::string> paths_;
};

// Binary PCD writer. Returns false on any PCL I/O failure.
inline bool WritePcd(const std::string& path, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  return pcl::io::savePCDFileBinary(path, cloud) == 0;
}

// Converts makeSyntheticMesh vertices to a point cloud. The meshes are welded on
// the shared lattice, so the cloud is exactly the frozen scene vertex set.
inline pcl::PointCloud<pcl::PointXYZ> CloudFromMesh(const topmap::MeshData& mesh) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(mesh.vertices.size());
  for (const Eigen::Vector3f& vertex : mesh.vertices) {
    cloud.push_back(pcl::PointXYZ(vertex.x(), vertex.y(), vertex.z()));
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

// Writes a synthetic scene's vertices as a binary PCD. Returns the path on
// success and an empty string on failure.
inline std::string WriteScenePcd(topmap::SyntheticScene scene, const std::string& path) {
  auto mesh = topmap::makeSyntheticMesh(scene);
  if (!mesh.ok()) return {};
  return WritePcd(path, CloudFromMesh(mesh.value())) ? path : std::string{};
}

// A syntactically valid PCD holding a single NaN point. PCL's writer refuses a
// zero-point cloud, but PointCloudProcessor removes NaNs, so loading this file
// exercises the "empty cloud after NaN removal" path (kPointCloudEmpty).
inline bool WriteNanPcd(const std::string& path) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) return false;
  out << "# .PCD v0.7 - Point Cloud Data file format\n"
         "VERSION 0.7\n"
         "FIELDS x y z\n"
         "SIZE 4 4 4\n"
         "TYPE F F F\n"
         "COUNT 1 1 1\n"
         "WIDTH 1\n"
         "HEIGHT 1\n"
         "VIEWPOINT 0 0 0 1 0 0 0\n"
         "POINTS 1\n"
         "DATA ascii\n"
         "nan nan nan\n";
  return static_cast<bool>(out);
}

// A gently curved plane used to exercise the R7 roughness/curvature wiring: a
// flat synthetic scene has zero roughness, so only a curved point cloud makes
// the feature channels observable in the archive.
inline pcl::PointCloud<pcl::PointXYZ> MakeBumpyPlaneCloud() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  constexpr float kHalf = 1.0f;
  constexpr float kStep = 0.10f;
  for (float x = -kHalf; x <= kHalf + 1e-4f; x += kStep) {
    for (float y = -kHalf; y <= kHalf + 1e-4f; y += kStep) {
      const float z = 0.05f * std::sin(3.0f * x) * std::cos(3.0f * y);
      cloud.push_back(pcl::PointXYZ(x, y, z));
    }
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

// R35: outcome of one service call. The helper never throws/asserts internally
// on a timeout, so the caller decides how to report it.
template <typename SrvT>
struct ServiceCall {
  bool service_available = false;
  bool ready = false;
  typename SrvT::Response::SharedPtr response;
};

// R35: calls `service_name` on `server_node` through a *separate* client node.
// Both nodes run on a MultiThreadedExecutor with a background spin thread, so a
// same-thread client waiting on its own service can never deadlock. The
// executor is always cancelled and joined, including on an exception.
template <typename SrvT>
ServiceCall<SrvT> callService(
    const rclcpp::Node::SharedPtr& server_node, const std::string& service_name,
    const typename SrvT::Request::SharedPtr& request,
    std::chrono::nanoseconds response_timeout = std::chrono::seconds(120),
    std::chrono::nanoseconds service_timeout = std::chrono::seconds(10)) {
  static std::atomic<unsigned long> counter{0};
  ServiceCall<SrvT> result;

  auto client_node = rclcpp::Node::make_shared(
      "topmap_test_client_" + std::to_string(counter.fetch_add(1)));
  auto client = client_node->create_client<SrvT>(service_name);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spinner([&executor]() { executor.spin(); });
  const auto stop = [&executor, &spinner]() {
    executor.cancel();
    if (spinner.joinable()) spinner.join();
  };

  try {
    if (!client->wait_for_service(service_timeout)) {
      stop();
      return result;
    }
    result.service_available = true;
    auto future = client->async_send_request(request);
    if (future.wait_for(response_timeout) != std::future_status::ready) {
      stop();
      return result;
    }
    result.response = future.get();
    result.ready = result.response != nullptr;
  } catch (...) {
    stop();
    throw;
  }
  stop();
  return result;
}

}  // namespace test
}  // namespace topmap_ros
