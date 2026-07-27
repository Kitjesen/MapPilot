#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "explore/directed_exploration_intent.hpp"

namespace {

using lingtu::explore::ExploreMapIdentity;
using lingtu::nav::endpoint::DirectedExplorationIntentStore;

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ExploreMapIdentity map(std::uint64_t generation, std::uint64_t reset_epoch = 1U,
                       std::string session_id = "rolling-session") {
  ExploreMapIdentity value;
  value.frame_id = "map";
  value.session_id = std::move(session_id);
  value.reset_epoch = reset_epoch;
  value.generation = generation;
  value.live = true;
  return value;
}

void testSetBindsSessionAndMapEpoch() {
  DirectedExplorationIntentStore store;
  const auto accepted = store.Set("explore-session", map(1U), 12.0, -4.0, 30.0, 100.0);
  require(accepted.accepted && accepted.changed, "set must be accepted");
  require(accepted.revision == 1U, "first set revision must be one");

  const auto same_epoch_next_generation = store.current("explore-session", map(2U), 101.0);
  require(same_epoch_next_generation.has_value(),
          "generation update must not invalidate a rolling intent");
  require(same_epoch_next_generation->target.x == 12.0 &&
              same_epoch_next_generation->target.y == -4.0,
          "target must be retained");

  require(!store.current("other-session", map(2U), 101.0).has_value(),
          "intent must not cross exploration sessions");
  require(!store.current("explore-session", map(2U, 2U), 101.0).has_value(),
          "reset epoch change must invalidate intent visibility");
  require(
      !store.current("explore-session", map(2U, 1U, "other-rolling-session"), 101.0).has_value(),
      "map source change must invalidate intent visibility");
}

void testClearAndExpiryAdvanceRevision() {
  DirectedExplorationIntentStore store;
  require(store.Set("explore-session", map(1U), 1.0, 2.0, 5.0, 100.0).accepted, "set setup");
  require(!store.Expire(104.9), "intent must remain until TTL expires");
  require(store.Expire(105.0), "TTL expiry must clear the intent");
  require(store.revision() == 2U, "expiry must advance intent revision");
  require(!store.current("explore-session", map(2U), 105.0).has_value(),
          "expired intent must not be returned");

  require(store.Set("explore-session", map(3U), 3.0, 4.0, 5.0, 106.0).accepted, "second set");
  const auto cleared = store.Clear();
  require(cleared.accepted && cleared.changed, "clear must remove an intent");
  require(cleared.revision == 4U, "clear must advance revision");
  const auto repeated = store.Clear();
  require(repeated.accepted && !repeated.changed && repeated.revision == 4U,
          "clear must be idempotent");
}

void testRejectsUnsafeInputs() {
  DirectedExplorationIntentStore store;
  require(!store.Set("", map(1U), 1.0, 2.0, 5.0, 100.0).accepted, "empty session must be rejected");
  require(!store
               .Set("session", map(1U), DirectedExplorationIntentStore::kMaxCoordinateMeters + 1.0,
                    2.0, 5.0, 100.0)
               .accepted,
          "out-of-range coordinate must be rejected");
  require(!store
               .Set("session", map(1U), 1.0, 2.0,
                    DirectedExplorationIntentStore::kMaxTtlSeconds + 1.0, 100.0)
               .accepted,
          "excessive ttl must be rejected");

  auto invalid_map = map(1U);
  invalid_map.generation = 0U;
  require(!store.Set("session", invalid_map, 1.0, 2.0, 5.0, 100.0).accepted,
          "invalid map identity must be rejected");
}

}  // namespace

int main() {
  testSetBindsSessionAndMapEpoch();
  testClearAndExpiryAdvanceRevision();
  testRejectsUnsafeInputs();
  std::cout << "test_directed_exploration_intent passed\n";
  return 0;
}
