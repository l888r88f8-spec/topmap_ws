#include <gtest/gtest.h>

#include <cmath>

#include "topmap_core/geometry.hpp"

TEST(GeometryTest, SlopeAndOrientation) {
  Eigen::Vector3f n = Eigen::Vector3f(0.0f, std::sin(20.0f * M_PI / 180.0f),
                                      std::cos(20.0f * M_PI / 180.0f));
  EXPECT_NEAR(topmap::slopeFromNormal(n) * 180.0f / M_PI, 20.0f, 1e-3f);
  EXPECT_GT(topmap::orientNormalUp(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f::UnitZ()).z(), 0.0f);
}

TEST(GeometryTest, AreaAndDistance) {
  Eigen::Vector3f a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
  EXPECT_NEAR(topmap::triangleArea(a, b, c), 0.5f, 1e-6f);
  EXPECT_NEAR(topmap::pointTriangleDistance(Eigen::Vector3f(0.25f, 0.25f, 2.0f), a, b, c), 2.0f, 1e-4f);
  // Plan correction (minimal deviation, reported): for a=(0,0,0), b=(1,0,0),
  // c=(0,1,0) a polyline is |ab|+|bc| = 1 + sqrt(2) ~= 2.41421, so the plan's
  // literal 2.0f contradicts the frozen polylineLength semantics. Use the exact
  // value here (the plan's other T05 constants are all correct).
  EXPECT_NEAR(topmap::polylineLength({a, b, c}), 1.0f + std::sqrt(2.0f), 1e-6f);
  // R26: the initial draft referenced a nonexistent heightJumpPlaneTest_helper
  // and asked to "change the test in step 3", violating TDD discipline. The final
  // form is written directly here.
  EXPECT_NEAR(topmap::pointPlaneDistance(Eigen::Vector3f(0, 0, 1), Eigen::Vector3f::UnitZ(), a),
              1.0f, 1e-6f);
  EXPECT_NEAR(topmap::segmentDistance(Eigen::Vector3f(0.5f, 1.0f, 0.0f), a, b), 1.0f, 1e-6f);
}
