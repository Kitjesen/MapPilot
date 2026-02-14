#include <gtest/gtest.h>
#include <thread>
#include "remote_monitoring/core/event_buffer.hpp"

using remote_monitoring::core::EventBuffer;

class EventBufferTest : public ::testing::Test {
protected:
  EventBuffer buffer_{100};  // max 100 events
};

TEST_F(EventBufferTest, StartsEmpty) {
  auto events = buffer_.GetLatestEvents(10);
  EXPECT_TRUE(events.empty());
}

TEST_F(EventBufferTest, AddAndRetrieve) {
  buffer_.AddEvent(
      robot::v1::EVENT_TYPE_SYSTEM_BOOT,
      robot::v1::EVENT_SEVERITY_INFO,
      "System started",
      "Boot sequence complete");

  auto events = buffer_.GetLatestEvents(10);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type(), robot::v1::EVENT_TYPE_SYSTEM_BOOT);
  EXPECT_EQ(events[0].title(), "System started");
  EXPECT_FALSE(events[0].event_id().empty());
}

TEST_F(EventBufferTest, EventIdsAreUnique) {
  buffer_.AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT,
                   robot::v1::EVENT_SEVERITY_INFO, "A", "");
  buffer_.AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT,
                   robot::v1::EVENT_SEVERITY_INFO, "B", "");

  auto events = buffer_.GetLatestEvents(10);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_NE(events[0].event_id(), events[1].event_id());
}

TEST_F(EventBufferTest, EventIdsAreMonotonicallyIncreasing) {
  for (int i = 0; i < 5; ++i) {
    buffer_.AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT,
                     robot::v1::EVENT_SEVERITY_INFO,
                     "event-" + std::to_string(i), "");
  }

  auto events = buffer_.GetLatestEvents(10);
  for (size_t i = 1; i < events.size(); ++i) {
    EXPECT_GT(events[i].event_id(), events[i - 1].event_id());
  }
}

TEST_F(EventBufferTest, GetEventsSinceFilters) {
  buffer_.AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT,
                   robot::v1::EVENT_SEVERITY_INFO, "first", "");
  auto events = buffer_.GetLatestEvents(1);
  std::string first_id = events[0].event_id();

  buffer_.AddEvent(robot::v1::EVENT_TYPE_MODE_CHANGE,
                   robot::v1::EVENT_SEVERITY_INFO, "second", "");

  auto since = buffer_.GetEventsSince(first_id);
  ASSERT_EQ(since.size(), 1u);
  EXPECT_EQ(since[0].title(), "second");
}

TEST_F(EventBufferTest, BoundsAtMaxSize) {
  for (int i = 0; i < 150; ++i) {
    buffer_.AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT,
                     robot::v1::EVENT_SEVERITY_INFO,
                     "event-" + std::to_string(i), "");
  }
  // Buffer is bounded at 100
  auto events = buffer_.GetLatestEvents(200);
  EXPECT_LE(events.size(), 100u);
}

TEST_F(EventBufferTest, AckEventDoesNotCrash) {
  buffer_.AddEvent(robot::v1::EVENT_TYPE_SYSTEM_BOOT,
                   robot::v1::EVENT_SEVERITY_INFO, "ack-test", "");
  auto events = buffer_.GetLatestEvents(1);
  // Should not throw
  buffer_.AckEvent(events[0].event_id());
  buffer_.AckEvent("nonexistent-id");
}

TEST_F(EventBufferTest, WaitForEventAfterTimeout) {
  robot::v1::Event event;
  bool got = buffer_.WaitForEventAfter(
      "", &event, std::chrono::milliseconds(50));
  EXPECT_FALSE(got);
}

TEST_F(EventBufferTest, WaitForEventAfterWakes) {
  // Add event in background thread after short delay
  std::thread producer([this]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    buffer_.AddEvent(robot::v1::EVENT_TYPE_SAFETY_ESTOP,
                     robot::v1::EVENT_SEVERITY_CRITICAL,
                     "estop", "test");
  });

  robot::v1::Event event;
  bool got = buffer_.WaitForEventAfter(
      "", &event, std::chrono::milliseconds(500));
  EXPECT_TRUE(got);
  EXPECT_EQ(event.title(), "estop");
  producer.join();
}
