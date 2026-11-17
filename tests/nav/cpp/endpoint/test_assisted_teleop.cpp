#include <gtest/gtest.h>

#include "control/authority.hpp"

TEST(AssistedTeleop, TakeoverAndResumeKeepsOperatorAuthority) {
  lingtu::nav::endpoint::ControlAuthority authority;
  ASSERT_TRUE(authority.activatePath());

  nav_kernel::Twist intent;
  intent.vx = 0.3;
  ASSERT_TRUE(authority.takeover(intent));
  EXPECT_TRUE(authority.operatorHeld());
  EXPECT_FALSE(authority.pathActive());
  EXPECT_FALSE(authority.activatePath());

  authority.holdOperator();
  EXPECT_TRUE(authority.operatorHeld());
  ASSERT_TRUE(authority.resume());
  EXPECT_FALSE(authority.pathActive());
  EXPECT_FALSE(authority.teleopRequest().has_value());
  EXPECT_TRUE(authority.activatePath());
}
