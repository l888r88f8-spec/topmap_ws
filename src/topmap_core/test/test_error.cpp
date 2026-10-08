#include <gtest/gtest.h>

#include "topmap_core/error.hpp"
#include "topmap_core/ids.hpp"

TEST(ErrorTest, ToStringStable) {
  EXPECT_STREQ(topmap::toString(topmap::ErrorCode::kOk), "kOk");
  EXPECT_STREQ(topmap::toString(topmap::ErrorCode::kTopologyUnreachable), "kTopologyUnreachable");
  EXPECT_STREQ(topmap::toString(topmap::ErrorCode::kSerializationVersionUnsupported),
               "kSerializationVersionUnsupported");
}

TEST(ResultTest, CarriesValueAndStatus) {
  topmap::Result<int> ok(7);
  EXPECT_TRUE(ok.ok());
  EXPECT_EQ(ok.value(), 7);
  topmap::Result<int> bad(topmap::Status::Error(topmap::ErrorCode::kMeshEmpty, "empty"));
  EXPECT_FALSE(bad.ok());
  EXPECT_EQ(bad.status().code, topmap::ErrorCode::kMeshEmpty);
  EXPECT_EQ(topmap::kInvalidId, -1);
}
