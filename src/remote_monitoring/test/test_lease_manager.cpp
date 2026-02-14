#include <gtest/gtest.h>
#include "remote_monitoring/core/lease_manager.hpp"

using remote_monitoring::core::LeaseManager;

class LeaseManagerTest : public ::testing::Test {
protected:
  LeaseManager mgr_;
};

TEST_F(LeaseManagerTest, NoLeaseInitially) {
  EXPECT_FALSE(mgr_.HasActiveLease());
}

TEST_F(LeaseManagerTest, AcquireAndRelease) {
  robot::v1::OperatorLease lease;
  bool ok = mgr_.AcquireLease("client-1", &lease);
  EXPECT_TRUE(ok);
  EXPECT_FALSE(lease.token().empty());
  EXPECT_TRUE(mgr_.HasActiveLease());

  bool released = mgr_.ReleaseLease(lease.token());
  EXPECT_TRUE(released);
  EXPECT_FALSE(mgr_.HasActiveLease());
}

TEST_F(LeaseManagerTest, ConflictOnDoubleAcquire) {
  robot::v1::OperatorLease lease1, lease2;
  EXPECT_TRUE(mgr_.AcquireLease("client-1", &lease1));
  EXPECT_FALSE(mgr_.AcquireLease("client-2", &lease2));
}

TEST_F(LeaseManagerTest, SameClientCanReacquire) {
  robot::v1::OperatorLease lease1, lease2;
  EXPECT_TRUE(mgr_.AcquireLease("client-1", &lease1));
  // Same holder re-acquiring should succeed (idempotent)
  EXPECT_TRUE(mgr_.AcquireLease("client-1", &lease2));
}

TEST_F(LeaseManagerTest, ValidateHoldsToken) {
  robot::v1::OperatorLease lease;
  mgr_.AcquireLease("client-1", &lease);
  EXPECT_TRUE(mgr_.ValidateLease(lease.token()));
  EXPECT_FALSE(mgr_.ValidateLease("wrong-token"));
}

TEST_F(LeaseManagerTest, ReleaseWithWrongToken) {
  robot::v1::OperatorLease lease;
  mgr_.AcquireLease("client-1", &lease);

  EXPECT_FALSE(mgr_.ReleaseLease("wrong-token"));
  EXPECT_TRUE(mgr_.HasActiveLease());
}

TEST_F(LeaseManagerTest, RenewExtendsTTL) {
  robot::v1::OperatorLease lease;
  mgr_.AcquireLease("client-1", &lease);

  robot::v1::OperatorLease renewed;
  EXPECT_TRUE(mgr_.RenewLease(lease.token(), &renewed));
  EXPECT_TRUE(mgr_.HasActiveLease());
}

TEST_F(LeaseManagerTest, RenewWithWrongTokenFails) {
  robot::v1::OperatorLease lease;
  mgr_.AcquireLease("client-1", &lease);

  robot::v1::OperatorLease renewed;
  EXPECT_FALSE(mgr_.RenewLease("bogus", &renewed));
}

TEST_F(LeaseManagerTest, ForceRelease) {
  robot::v1::OperatorLease lease;
  mgr_.AcquireLease("client-1", &lease);

  mgr_.ForceRelease();
  EXPECT_FALSE(mgr_.HasActiveLease());

  // New client can now acquire
  robot::v1::OperatorLease lease2;
  EXPECT_TRUE(mgr_.AcquireLease("client-2", &lease2));
}

TEST_F(LeaseManagerTest, ReleaseWhenNoLeaseFails) {
  EXPECT_FALSE(mgr_.ReleaseLease("any-token"));
}

TEST_F(LeaseManagerTest, ForceReleaseWhenNoLeaseIsNoop) {
  mgr_.ForceRelease();
  EXPECT_FALSE(mgr_.HasActiveLease());
}

TEST_F(LeaseManagerTest, CheckTimeoutDoesNotCrash) {
  // Should not crash even without active lease
  mgr_.CheckTimeout();

  robot::v1::OperatorLease lease;
  mgr_.AcquireLease("client-1", &lease);
  // Should not crash with active lease
  mgr_.CheckTimeout();
}
