#pragma once
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "topmap_core/error.hpp"
#include "topmap_core/params.hpp"

namespace topmap {

// T07 per-point surface features (A4.6).
struct SurfaceFeature {
  Eigen::Vector3f position, normal;
  float curvature = 0.0f, roughness = 0.0f, slope = 0.0f;
};

class SurfaceFeatureEstimator {
 public:
  static Result<std::vector<SurfaceFeature>> estimate(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                                      const MapBuildParams& params);
};

}  // namespace topmap
