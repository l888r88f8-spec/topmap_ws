#include <gtest/gtest.h>

#include "topmap_msgs/msg/region.hpp"
#include "topmap_msgs/msg/portal.hpp"
#include "topmap_msgs/srv/plan_path.hpp"

TEST(MsgsRoundtrip, FieldsExist) {
  topmap_msgs::msg::Region r;
  r.id = 3;
  topmap_msgs::msg::Portal p;
  p.valid_width = 0.9f;
  topmap_msgs::srv::PlanPath::Request req;
  EXPECT_EQ(r.id, 3);  // R25: int32 field compared against a signed constant
  EXPECT_FLOAT_EQ(p.valid_width, 0.9f);
  (void)req;
}
