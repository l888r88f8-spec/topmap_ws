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

TEST(BinaryIoTest, WritesFixedLittleEndianBytes) {
  topmap::BinaryWriter u32;
  u32.writeU32(0x01020304u);
  EXPECT_EQ(u32.bytes(), (std::vector<std::uint8_t>{0x04, 0x03, 0x02, 0x01}));

  topmap::BinaryWriter i32;
  i32.writeI32(-2);  // 0xFFFFFFFE
  EXPECT_EQ(i32.bytes(), (std::vector<std::uint8_t>{0xFE, 0xFF, 0xFF, 0xFF}));

  topmap::BinaryWriter u64;
  u64.writeU64(0x0102030405060708ull);
  EXPECT_EQ(u64.bytes(),
            (std::vector<std::uint8_t>{0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01}));

  topmap::BinaryWriter f32;
  f32.writeF32(1.0f);  // IEEE-754 0x3F800000
  EXPECT_EQ(f32.bytes(), (std::vector<std::uint8_t>{0x00, 0x00, 0x80, 0x3F}));

  topmap::BinaryWriter f64;
  f64.writeF64(1.0);  // IEEE-754 0x3FF0000000000000
  EXPECT_EQ(f64.bytes(),
            (std::vector<std::uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F}));
}

TEST(BinaryIoTest, CompositeReadFailurePreservesOffsetAndOutput) {
  // Only 8 of the 12 Vector3f bytes are present.
  std::vector<std::uint8_t> data(8, 0xAB);
  topmap::BinaryReader r(data);
  const Eigen::Vector3f sentinel(9.0f, 8.0f, 7.0f);
  Eigen::Vector3f v = sentinel;
  EXPECT_FALSE(r.readVector3f(v));
  EXPECT_EQ(r.remaining(), data.size());   // offset untouched
  EXPECT_TRUE(v.isApprox(sentinel));       // output untouched
}

TEST(BinaryIoTest, VectorReadFailurePreservesOffsetAndOutput) {
  // Declares three int32 elements but carries only one.
  std::vector<std::uint8_t> data{0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  topmap::BinaryReader r(data);
  std::vector<std::int32_t> out{42, 43};
  EXPECT_FALSE(r.readVecI32(out));
  EXPECT_EQ(r.remaining(), data.size());
  EXPECT_EQ(out, (std::vector<std::int32_t>{42, 43}));

  // Declares two Vector3f elements but carries none.
  std::vector<std::uint8_t> data2{0x02, 0x00, 0x00, 0x00};
  topmap::BinaryReader r2(data2);
  std::vector<Eigen::Vector3f> pts{Eigen::Vector3f(1.0f, 1.0f, 1.0f)};
  EXPECT_FALSE(r2.readVecVector3f(pts));
  EXPECT_EQ(r2.remaining(), data2.size());
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_TRUE(pts[0].isApprox(Eigen::Vector3f(1.0f, 1.0f, 1.0f)));
}

TEST(BinaryIoTest, HugeDeclaredLengthIsRejectedWithoutAllocating) {
  std::vector<std::uint8_t> data{0xFF, 0xFF, 0xFF, 0xFF};  // 4294967295 elements
  topmap::BinaryReader r(data);
  std::vector<float> out{1.0f};
  EXPECT_FALSE(r.readVecF32(out));
  EXPECT_EQ(r.remaining(), data.size());
  EXPECT_EQ(out, (std::vector<float>{1.0f}));

  std::vector<std::uint8_t> data2{0xFF, 0xFF, 0xFF, 0xFF};
  topmap::BinaryReader r2(data2);
  std::string s = "keep";
  EXPECT_FALSE(r2.readString(s));
  EXPECT_EQ(r2.remaining(), data2.size());
  EXPECT_EQ(s, "keep");
}
