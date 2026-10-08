#include <gtest/gtest.h>

#include "topmap_core/binary_io.hpp"

TEST(BinaryIoTest, RoundTripsEverything) {
  topmap::BinaryWriter w;
  w.writeU32(topmap::kTopmapMagic);
  w.writeU32(topmap::kTopmapFormatVersion);
  w.writeF32(0.6f);
  w.writeString("map");
  w.writeVector3f(Eigen::Vector3f(1, 2, 3));
  w.writeVecI32({4, 5, 6});
  topmap::BinaryReader r(w.bytes());
  std::uint32_t magic = 0, ver = 0;
  ASSERT_TRUE(r.readU32(magic));
  ASSERT_TRUE(r.readU32(ver));
  EXPECT_EQ(magic, topmap::kTopmapMagic);
  EXPECT_EQ(ver, 1u);
  float f = 0;
  ASSERT_TRUE(r.readF32(f));
  EXPECT_FLOAT_EQ(f, 0.6f);
  std::string s;
  ASSERT_TRUE(r.readString(s));
  EXPECT_EQ(s, "map");
  Eigen::Vector3f v;
  ASSERT_TRUE(r.readVector3f(v));
  EXPECT_TRUE(v.isApprox(Eigen::Vector3f(1, 2, 3)));
  std::vector<std::int32_t> xs;
  ASSERT_TRUE(r.readVecI32(xs));
  EXPECT_EQ(xs, (std::vector<std::int32_t>{4, 5, 6}));
  EXPECT_TRUE(r.eof());
}

TEST(BinaryIoTest, ShortReadFailsWithoutAdvancing) {
  std::vector<std::uint8_t> data{1, 2, 3};
  topmap::BinaryReader r(data);
  std::uint32_t x = 0;
  EXPECT_FALSE(r.readU32(x));
  EXPECT_EQ(r.remaining(), 3u);
}
