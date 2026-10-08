#pragma once
#include <vector>

#include <Eigen/Core>

namespace topmap {

float clampf(float v, float lo, float hi);
float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b);           // [0, pi]
float slopeFromNormal(const Eigen::Vector3f& n);                                  // angle to +Z
Eigen::Vector3f orientNormalUp(const Eigen::Vector3f& n, const Eigen::Vector3f& view_dir);
float triangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
Eigen::Vector3f triangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
float pointTriangleDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                            const Eigen::Vector3f& b, const Eigen::Vector3f& c);
float pointPlaneDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& n, const Eigen::Vector3f& q);
float segmentDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b);
float polylineLength(const std::vector<Eigen::Vector3f>& pts);

}  // namespace topmap
