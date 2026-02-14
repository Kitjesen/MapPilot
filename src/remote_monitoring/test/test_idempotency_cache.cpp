#include <gtest/gtest.h>
#include "remote_monitoring/core/idempotency_cache.hpp"

using remote_monitoring::core::IdempotencyCache;

class IdempotencyCacheTest : public ::testing::Test {
protected:
  // TTL = 3600s (long enough for tests), max = 100 entries
  IdempotencyCache cache_{3600, 100};
};

// ── Basic CRUD ──

TEST_F(IdempotencyCacheTest, MissOnEmpty) {
  std::string out;
  EXPECT_FALSE(cache_.TryGet("req-1", &out));
}

TEST_F(IdempotencyCacheTest, HitAfterSet) {
  cache_.Set("req-1", "response-data");
  std::string out;
  EXPECT_TRUE(cache_.TryGet("req-1", &out));
  EXPECT_EQ(out, "response-data");
}

TEST_F(IdempotencyCacheTest, OverwriteExisting) {
  cache_.Set("req-1", "v1");
  cache_.Set("req-1", "v2");
  std::string out;
  EXPECT_TRUE(cache_.TryGet("req-1", &out));
  EXPECT_EQ(out, "v2");
}

TEST_F(IdempotencyCacheTest, EmptyRequestIdIgnored) {
  // Set with empty key should be ignored
  cache_.Set("", "value");
  std::string out;
  EXPECT_FALSE(cache_.TryGet("", &out));
}

TEST_F(IdempotencyCacheTest, NullOutResponseIsOk) {
  cache_.Set("req-1", "value");
  // TryGet with nullptr should still return true (just no output)
  EXPECT_TRUE(cache_.TryGet("req-1", nullptr));
}

// ── Capacity eviction ──

TEST_F(IdempotencyCacheTest, EvictsOldestWhenFull) {
  // Fill to capacity: req-0 through req-99
  for (int i = 0; i < 100; ++i) {
    cache_.Set("req-" + std::to_string(i), "val-" + std::to_string(i));
  }

  // All 100 should be present
  std::string out;
  EXPECT_TRUE(cache_.TryGet("req-0", &out));
  EXPECT_TRUE(cache_.TryGet("req-99", &out));

  // Adding req-100 should evict the oldest (req-0)
  cache_.Set("req-100", "val-100");
  EXPECT_FALSE(cache_.TryGet("req-0", &out));
  EXPECT_TRUE(cache_.TryGet("req-100", &out));
}

// ── TTL expiration ──

TEST_F(IdempotencyCacheTest, ExpiredEntryReturnsMiss) {
  // Create cache with 0-second TTL (instant expiry)
  IdempotencyCache ephemeral(0, 100);
  ephemeral.Set("req-1", "value");

  // Entry should already be expired
  std::string out;
  EXPECT_FALSE(ephemeral.TryGet("req-1", &out));
}

TEST_F(IdempotencyCacheTest, CleanupRemovesExpired) {
  IdempotencyCache ephemeral(0, 100);
  ephemeral.Set("req-1", "v1");
  ephemeral.Set("req-2", "v2");

  // Cleanup should purge all (0s TTL = everything expired)
  ephemeral.Cleanup();

  std::string out;
  EXPECT_FALSE(ephemeral.TryGet("req-1", &out));
  EXPECT_FALSE(ephemeral.TryGet("req-2", &out));
}

// ── Concurrency smoke test ──

TEST_F(IdempotencyCacheTest, ConcurrentSetAndGet) {
  const int kIterations = 1000;
  std::vector<std::thread> threads;

  // Writer threads
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([this, t, kIterations]() {
      for (int i = 0; i < kIterations; ++i) {
        cache_.Set("thread-" + std::to_string(t) + "-" + std::to_string(i),
                   "value");
      }
    });
  }

  // Reader threads
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([this, t, kIterations]() {
      std::string out;
      for (int i = 0; i < kIterations; ++i) {
        cache_.TryGet("thread-" + std::to_string(t) + "-" + std::to_string(i),
                      &out);
      }
    });
  }

  for (auto &th : threads) th.join();
  // No crash = pass
}
