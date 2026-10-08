#include "topmap_core/geometry.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace topmap {

float clampf(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }

float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  return std::acos(clampf(a.normalized().dot(b.normalized()), -1.0f, 1.0f));
}

float slopeFromNormal(const Eigen::Vector3f& n) {
  return std::acos(clampf(n.normalized().dot(Eigen::Vector3f::UnitZ()), -1.0f, 1.0f));
}

Eigen::Vector3f orientNormalUp(const Eigen::Vector3f& n, const Eigen::Vector3f& view_dir) {
  return n.dot(view_dir) >= 0.0f ? n : Eigen::Vector3f(-n);
}

float triangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  return 0.5f * (b - a).cross(c - a).norm();
}

Eigen::Vector3f triangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f n = (b - a).cross(c - a);
  const float norm = n.norm();
  if (norm <= 1e-12f) return Eigen::Vector3f::UnitZ();
  return n / norm;
}

float segmentDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  const Eigen::Vector3f ab = b - a;
  const float len2 = ab.squaredNorm();
  if (len2 <= 1e-12f) return (p - a).norm();
  const float t = clampf((p - a).dot(ab) / len2, 0.0f, 1.0f);
  return (p - (a + t * ab)).norm();
}

float pointPlaneDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& n, const Eigen::Vector3f& q) {
  const float nn = n.norm();
  if (nn <= 1e-12f) return (p - q).norm();
  return std::fabs(n.dot(p - q) / nn);
}

float pointTriangleDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& a,
                            const Eigen::Vector3f& b, const Eigen::Vector3f& c) {
  const Eigen::Vector3f ab = b - a, ac = c - a, n = ab.cross(ac);
  const float area2 = n.squaredNorm();
  if (area2 <= 1e-12f) return std::min({(p - a).norm(), (p - b).norm(), (p - c).norm()});
  const Eigen::Vector3f nn = n / std::sqrt(area2);
  const float d = nn.dot(p - a);
  const Eigen::Vector3f proj = p - d * nn;
  // Barycentric in-triangle test; outside the triangle fall back to the closest
  // of the three edge distances.
  const Eigen::Vector3f v0 = b - a, v1 = c - a, v2 = proj - a;
  const float d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1), d20 = v2.dot(v0), d21 = v2.dot(v1);
  const float denom = d00 * d11 - d01 * d01;
  const float u = (d11 * d20 - d01 * d21) / denom;
  const float v = (d00 * d21 - d01 * d20) / denom;
  if (u >= 0.0f && v >= 0.0f && u + v <= 1.0f) return std::fabs(d);
  return std::min({segmentDistance(p, a, b), segmentDistance(p, b, c), segmentDistance(p, c, a)});
}

float polylineLength(const std::vector<Eigen::Vector3f>& pts) {
  float total = 0.0f;
  for (std::size_t i = 1; i < pts.size(); ++i) total += (pts[i] - pts[i - 1]).norm();
  return total;
}

}  // namespace topmap
