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

TEST(GeometryTest, PointTriangleDistanceHandlesDegenerateCollinear) {
  // A degenerate (collinear) triangle must fall back to the closest point on
  // its three segments, not merely the closest vertex.
  Eigen::Vector3f a(0, 0, 0), b(2, 0, 0), c(4, 0, 0);
  Eigen::Vector3f p(1, 1, 0);  // closest point on ab is (1,0,0) => distance 1
  EXPECT_NEAR(topmap::pointTriangleDistance(p, a, b, c), 1.0f, 1e-5f);

  // Coincident vertices and a zero-length triangle must remain finite too.
  EXPECT_NEAR(topmap::pointTriangleDistance(p, a, a, a), std::sqrt(2.0f), 1e-5f);
  EXPECT_TRUE(std::isfinite(topmap::pointTriangleDistance(p, a, b, a)));
}

TEST(GeometryTest, AngleHelpersAreFiniteForZeroVectors) {
  // The frozen plan does not define a direction for a zero vector. This test
  // only pins down the safety property that the helpers never return NaN/Inf;
  // it deliberately does not invent a specific angle.
  EXPECT_TRUE(std::isfinite(topmap::angleBetween(Eigen::Vector3f::Zero(), Eigen::Vector3f::UnitZ())));
  EXPECT_TRUE(std::isfinite(topmap::angleBetween(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero())));
  EXPECT_TRUE(std::isfinite(topmap::slopeFromNormal(Eigen::Vector3f::Zero())));
}
