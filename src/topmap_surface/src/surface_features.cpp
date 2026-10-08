#include "topmap_surface/surface_features.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include "topmap_core/geometry.hpp"

namespace topmap {
namespace {

constexpr float kEpsilon = 1e-12f;

bool hasNonFiniteCoordinate(const pcl::PointXYZ& p) {
  return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
}

}  // namespace

Result<std::vector<SurfaceFeature>> SurfaceFeatureEstimator::estimate(
    const pcl::PointCloud<pcl::PointXYZ>& cloud, const MapBuildParams& params) {
  if (cloud.size() < 3) {
    return Status::Error(ErrorCode::kPointCloudTooSmall,
                         "point cloud has fewer than 3 points");
  }
  if (!(params.normal_search_radius > 0.0f) || !std::isfinite(params.normal_search_radius)) {
    return Status::Error(ErrorCode::kInvalidArgument,
                         "normal_search_radius must be a positive finite value");
  }
  for (const auto& p : cloud.points) {
    if (hasNonFiniteCoordinate(p)) {
      return Status::Error(ErrorCode::kPointCloudInvalidCoordinates,
                           "point cloud contains non-finite coordinates");
    }
  }

  // NormalEstimation uses a KdTree radius search and PCA per point. Points with
  // fewer than three neighbours get a degenerate normal, which is replaced by
  // +Z below.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(cloud.makeShared());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>());
  normal_estimator.setSearchMethod(search);
  normal_estimator.setRadiusSearch(params.normal_search_radius);
  pcl::PointCloud<pcl::Normal> normals;
  normal_estimator.compute(normals);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud.makeShared());

  std::vector<SurfaceFeature> features(cloud.size());
  std::vector<int> neighbors;
  std::vector<float> squared_distances;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    features[i].position = cloud[i].getVector3fMap();

    Eigen::Vector3f normal(normals[i].normal_x, normals[i].normal_y, normals[i].normal_z);
    if (!normal.allFinite() || normal.norm() <= kEpsilon) {
      normal = Eigen::Vector3f::UnitZ();
      features[i].curvature = 0.0f;
    } else {
      normal.normalize();
      normal = orientNormalUp(normal, Eigen::Vector3f::UnitZ());
      const float curvature = normals[i].curvature;
      features[i].curvature = (std::isfinite(curvature) && curvature >= 0.0f) ? curvature : 0.0f;
    }
    features[i].normal = normal;
    features[i].slope = slopeFromNormal(normal);

    // Roughness: RMS distance of neighbourhood points to their PCA plane, i.e.
    // the square root of the smallest covariance eigenvalue.
    const int found = kdtree.radiusSearch(cloud[i], params.normal_search_radius, neighbors,
                                          squared_distances);
    if (found < 3) {
      features[i].roughness = 0.0f;
      continue;
    }
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int k = 0; k < found; ++k) {
      mean += cloud[neighbors[k]].getVector3fMap();
    }
    mean /= static_cast<float>(found);

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int k = 0; k < found; ++k) {
      const Eigen::Vector3f d = cloud[neighbors[k]].getVector3fMap() - mean;
      covariance.noalias() += d * d.transpose();
    }
    covariance /= static_cast<float>(found);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    const float smallest = solver.eigenvalues()(0);
    features[i].roughness = smallest > 0.0f ? std::sqrt(smallest) : 0.0f;
  }
  return features;
}

}  // namespace topmap
