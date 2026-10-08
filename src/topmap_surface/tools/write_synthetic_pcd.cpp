// R18: CLI used by run_acceptance.sh as
//   ros2 run topmap_surface write_synthetic_pcd --scene flat_plane --output /tmp/x.pcd
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "topmap_surface/synthetic_scene.hpp"

namespace {

void PrintUsage() {
  std::cerr << "usage: write_synthetic_pcd --scene <name> --output <path>\n"
            << "  names: flat_plane, ramp, overlapping_floors, multi_level_ramps,\n"
            << "         hall_narrow_hall, u_shape, t_shape, single_bridge,\n"
            << "         parallel_portals, loop\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string scene_name;
  std::string output_path;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scene" && i + 1 < argc) {
      scene_name = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      PrintUsage();
      return 2;
    }
  }
  if (scene_name.empty() || output_path.empty()) {
    PrintUsage();
    return 2;
  }

  auto scene = topmap::parseSyntheticScene(scene_name);
  if (!scene.ok()) {
    std::cerr << scene.status().message << "\n";
    return 1;
  }
  auto mesh = topmap::makeSyntheticMesh(scene.value());
  if (!mesh.ok()) {
    std::cerr << mesh.status().message << "\n";
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(mesh.value().vertices.size());
  for (const Eigen::Vector3f& v : mesh.value().vertices) {
    cloud.push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
  }
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;

  if (pcl::io::savePCDFileASCII(output_path, cloud) != 0) {
    std::cerr << "failed to write PCD file: " << output_path << "\n";
    return 1;
  }
  std::cout << "wrote " << cloud.size() << " points (" << scene_name << ") to " << output_path
            << "\n";
  return 0;
}
