#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "message/cpp/navigation_command.hpp"
#include "motion/command_ack_journal.hpp"

namespace {

using lingtu::message::NavigationCommandKind;
using lingtu::nav::endpoint::CommandAckJournal;
using lingtu::nav::endpoint::CommandIdentity;
using lingtu::nav::endpoint::CommandPayload;
using lingtu::nav::endpoint::JournalDisposition;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

CommandIdentity goalIdentity(const std::string &client_id, const std::string &request_id, double x,
                             double y) {
  CommandPayload payload;
  payload.frame_id = "map";
  payload.goal[0] = x;
  payload.goal[1] = y;
  payload.goal[6] = 1.0;
  return {
      client_id,
      request_id,
      NavigationCommandKind::Goal,
      payload.canonical(),
  };
}

void testPayloadConflictIsRejected() {
  using namespace std::chrono_literals;
  CommandAckJournal journal(16, 10min);
  const auto now = CommandAckJournal::Clock::time_point{10s};
  const auto first = goalIdentity("gateway", "goal-1", 1.0, 2.0);
  journal.remember(first, {true, "accepted"}, now);

  const auto replay = journal.lookup(first, now + 1s);
  require(replay.disposition == JournalDisposition::Replay, "identical payload must replay");
  require(replay.ack.has_value() && replay.ack->accepted, "replayed ACK must be preserved");

  const auto conflict = journal.lookup(goalIdentity("gateway", "goal-1", 20.0, 30.0), now + 2s);
  require(conflict.disposition == JournalDisposition::Conflict,
          "same request id with a different payload must conflict");
}

void testClientIdentityNamespacesRequestIds() {
  using namespace std::chrono_literals;
  CommandAckJournal journal(16, 10min);
  const auto now = CommandAckJournal::Clock::time_point{20s};
  journal.remember(goalIdentity("gateway-a", "goal-1", 1.0, 2.0), {true, "accepted"}, now);

  const auto other_client = journal.lookup(goalIdentity("gateway-b", "goal-1", 3.0, 4.0), now + 1s);
  require(other_client.disposition == JournalDisposition::Miss,
          "different clients must not share request-id state");
}

void testCommandKindNamespacesRequestIds() {
  using namespace std::chrono_literals;
  CommandAckJournal journal(16, 10min);
  const auto now = CommandAckJournal::Clock::time_point{25s};
  const auto goal = goalIdentity("gateway", "request-1", 1.0, 2.0);
  journal.remember(goal, {true, "accepted"}, now);

  CommandPayload payload;
  payload.frame_id = "body";
  payload.reason = "operator_stop";
  const CommandIdentity stop{
      "gateway",
      "request-1",
      NavigationCommandKind::Stop,
      payload.canonical(),
  };
  require(journal.lookup(stop, now + 1s).disposition == JournalDisposition::Miss,
          "command kind must be part of the idempotency key");
}

void testExpiredEntriesDoNotReplay() {
  using namespace std::chrono_literals;
  CommandAckJournal journal(16, 5s);
  const auto now = CommandAckJournal::Clock::time_point{30s};
  const auto identity = goalIdentity("gateway", "goal-expired", 1.0, 2.0);
  journal.remember(identity, {true, "accepted"}, now);

  const auto expired = journal.lookup(identity, now + 6s);
  require(expired.disposition == JournalDisposition::Miss,
          "expired command entries must not replay forever");
}

void testTeleopJournalCannotEvictBusinessCommands() {
  using namespace std::chrono_literals;
  CommandAckJournal business(8, 10min);
  CommandAckJournal teleop(2, 5s);
  const auto now = CommandAckJournal::Clock::time_point{40s};
  const auto goal = goalIdentity("gateway", "goal-stable", 1.0, 2.0);
  business.remember(goal, {true, "accepted"}, now);

  for (int index = 0; index < 20; ++index) {
    CommandPayload payload;
    payload.frame_id = "body";
    payload.velocity[0] = 0.01 * static_cast<double>(index);
    teleop.remember(
        {
            "gateway",
            "teleop-" + std::to_string(index),
            NavigationCommandKind::Teleop,
            payload.canonical(),
        },
        {true, "accepted"}, now + std::chrono::milliseconds(index));
  }

  require(business.lookup(goal, now + 1s).disposition == JournalDisposition::Replay,
          "teleop churn must not evict business command ACKs");
}

void testCanonicalPayloadNormalizesSignedZero() {
  CommandPayload positive;
  positive.goal[0] = 0.0;
  CommandPayload negative;
  negative.goal[0] = -0.0;
  require(positive.canonical() == negative.canonical(),
          "signed zero must have one canonical payload representation");
}

}  // namespace

int main() {
  try {
    testPayloadConflictIsRejected();
    testClientIdentityNamespacesRequestIds();
    testCommandKindNamespacesRequestIds();
    testExpiredEntriesDoNotReplay();
    testTeleopJournalCannotEvictBusinessCommands();
    testCanonicalPayloadNormalizesSignedZero();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_command_ack_journal: FAIL: %s\n", exc.what());
    return 1;
  }
}
