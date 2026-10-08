#include "topmap_surface/point_cloud_processor.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace topmap {
namespace {

bool hasInfiniteCoordinate(const pcl::PointXYZ& p) {
  return std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z);
}

}  // namespace

Result<pcl::PointCloud<pcl::PointXYZ>> PointCloudProcessor::loadPCD(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) != 0) {
    return Status::Error(ErrorCode::kInvalidArgument, "failed to load PCD file: " + path);
  }
  return cloud;
}

Result<pcl::PointCloud<pcl::PointXYZ>> PointCloudProcessor::process(
    const pcl::PointCloud<pcl::PointXYZ>& input, const MapBuildParams& params) {
  if (input.empty()) {
    return Status::Error(ErrorCode::kPointCloudEmpty, "input point cloud is empty");
  }

  // NaN is a normal sensor artifact and is removed below; +/-inf is not a
  // coordinate at all and must be reported (spec section 8).
  for (const auto& p : input.points) {
    if (hasInfiniteCoordinate(p)) {
      return Status::Error(ErrorCode::kPointCloudInvalidCoordinates,
                           "input point cloud contains infinite coordinates");
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<int> index;
  // PCL's removeNaNFromPointCloud short-circuits when is_dense is set (the
  // PointCloud default), silently keeping NaN. Force the non-dense branch so
  // sensor NaNs are always removed.
  pcl::PointCloud<pcl::PointXYZ> non_dense = input;
  non_dense.is_dense = false;
  pcl::removeNaNFromPointCloud(non_dense, cloud, index);
  if (cloud.empty()) {
    return Status::Error(ErrorCode::kPointCloudEmpty,
                         "point cloud is empty after removing NaN points");
  }
  // Fewer than three usable input points cannot support normal estimation or
  // triangulation downstream (spec section 8).
  if (cloud.size() < 3) {
    return Status::Error(ErrorCode::kPointCloudTooSmall,
                         "point cloud has fewer than 3 usable points");
  }

  if (params.outlier_k > 0 && cloud.size() > static_cast<std::size_t>(params.outlier_k)) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setMeanK(params.outlier_k);
    sor.setStddevMulThresh(params.outlier_std_ratio);
    pcl::PointCloud<pcl::PointXYZ> filtered;
    sor.filter(filtered);
    cloud = filtered;
  }

  if (params.voxel_size > 0.0f) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud.makeShared());
    voxel.setLeafSize(params.voxel_size, params.voxel_size, params.voxel_size);
    pcl::PointCloud<pcl::PointXYZ> downsampled;
    voxel.filter(downsampled);
    cloud = downsampled;
  }

  return cloud;
}

}  // namespace topmap
