#pragma once
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"

namespace topmap {

// T06 point-cloud preprocessing (A4.6).
//
// `process` is the frozen A4.6 entry point: NaN removal, optional statistical
// outlier removal, then voxel-grid downsampling. `loadPCD` is the PCD ingress
// explicitly required by the T06 scope; the A4.6 snapshot did not name an
// ingress helper, so it is added as an additive (non-breaking) static method.
class PointCloudProcessor {
 public:
  static Result<pcl::PointCloud<pcl::PointXYZ>> loadPCD(const std::string& path);
  static Result<pcl::PointCloud<pcl::PointXYZ>> process(const pcl::PointCloud<pcl::PointXYZ>& input,
                                                        const MapBuildParams& params);
};

}  // namespace topmap
