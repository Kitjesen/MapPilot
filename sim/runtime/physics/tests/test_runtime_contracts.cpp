#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "lingtu/sim/command_inbox.hpp"
#include "lingtu/sim/runtime_contracts.hpp"
#include "lingtu/sim/snapshot_bus.hpp"

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

constexpr std::string_view kSessionId = "session-a";

lingtu::sim::EntityState entity(const char *instance_id,
                                      const char *frame_id,
                                      const char *stable_id,
                                      double x) {
  lingtu::sim::EntityState state;
  require(state.instance_id.assign(instance_id), "instance ID must fit the contract");
  require(state.frame_id.assign(frame_id), "frame ID must fit the contract");
  require(state.stable_id.assign(stable_id), "stable ID must fit the contract");
  state.position_m[0] = x;
  state.linear_velocity_mps[0] = 0.5;
  state.angular_velocity_rps[2] = 0.25;
  require(state.joint_position_rad.push_back(x), "joint position must fit");
  require(state.joint_velocity_rps.push_back(x + 1.0), "joint velocity must fit");
  return state;
}

void populate_snapshot(lingtu::sim::TruthSnapshotEnvelope &value,
                       std::uint64_t sequence) {
  require(value.session_id.assign(kSessionId), "session id must fit the contract");
  value.generation = {4, 2};
  value.sequence = sequence;
  value.sim_time_ns = sequence * 2'000'000;
  value.entities.clear();
  require(value.entities.push_back(entity("alpha", "base_link", "alpha/base_link", static_cast<double>(sequence))), "first entity must fit");
  require(value.entities.push_back(entity("beta", "base_link", "beta/base_link", static_cast<double>(sequence + 1))), "second entity must fit");
}

lingtu::sim::TruthSnapshotEnvelope snapshot(std::uint64_t sequence) {
  lingtu::sim::TruthSnapshotEnvelope value;
  populate_snapshot(value, sequence);
  return value;
}

lingtu::sim::CommandEnvelope command(std::uint64_t sequence,
                                      lingtu::sim::GenerationStamp generation,
                                      const char *instance_id) {
  lingtu::sim::CommandEnvelope value;
  require(value.source_id.assign("test-source"), "source ID must fit");
  require(value.instance_id.assign(instance_id), "instance ID must fit");
  value.generation = generation;
  value.sequence = sequence;
  value.apply_time_ns = sequence * 1'000'000;
  require(value.type.assign("joint-position"), "command type must fit");
  const std::array<double, 2> payload{static_cast<double>(sequence), 0.25};
  require(value.set_payload(payload), "command payload must fit");
  return value;
}

void test_contract_shapes_and_fixed_capacity() {
  const auto value = snapshot(7);
  require(value.generation == lingtu::sim::GenerationStamp{4, 2}, "generation must round-trip");
  require(value.session_id.view() == kSessionId &&
              value.session_id.size() == kSessionId.size(),
          "session id must fit the bounded field");
  require(value.sim_time_ns == 14'000'000, "simulation time must use integer nanoseconds");
  require(value.entities.size() == 2, "snapshot must contain both entities");
  require(value.entities[0].joint_position_rad.size() == 1, "joint arrays must be bounded");
  require(value.entities[0].stable_id.view() == "alpha/base_link", "stable ID must be preserved");
  require(value.entities[0].instance_id.view() == "alpha" &&
              value.entities[0].frame_id.view() == "base_link",
          "instance and frame IDs must be explicit");

  lingtu::sim::CommandEnvelope envelope = command(3, {4, 2}, "alpha");
  require(envelope.payload_size == sizeof(std::array<double, 2>), "payload size must be explicit");
  require(envelope.type.view() == "joint-position", "command type must be generic");
}

void test_snapshot_bus_publishes_only_complete_latest_value() {
  auto bus = std::make_unique<lingtu::sim::SnapshotBus<3>>();
  auto input = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();
  auto out = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();
  require(bus->capacity() == 3, "snapshot bus capacity must be explicit");
  require(!bus->latest(*out), "empty bus must have no latest snapshot");
  populate_snapshot(*input, 1);
  require(bus->push(*input) == lingtu::sim::SnapshotPushResult::published,
          "first snapshot must publish");
  populate_snapshot(*input, 2);
  require(bus->push(*input) == lingtu::sim::SnapshotPushResult::published_replacing_latest,
          "second snapshot must replace the latest value");
  populate_snapshot(*input, 3);
  require(bus->push(*input) == lingtu::sim::SnapshotPushResult::published_replacing_latest,
          "third snapshot must replace the latest value");
  require(bus->size() == 1, "latest-value bus must expose one logical value");
  require(bus->latest(*out), "latest must be readable");
  require(out->sequence == 3 && out->entities[0].position_m[0] == 3.0,
          "latest must win over stale snapshots");
  require(out->entities.size() == 2 && out->entities[1].stable_id.view() == "beta/base_link",
          "latest snapshot must be complete");
  require(bus->pop_latest(*out), "latest snapshot must be consumable");
  require(out->sequence == 3 && bus->size() == 0, "pop_latest must consume the current value");
  require(!bus->pop_latest(*out), "empty bus must not pop");
}

void test_snapshot_bus_capacity_one_replaces_latest_value() {
  auto bus = std::make_unique<lingtu::sim::SnapshotBus<1>>();
  auto input = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();
  auto out = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();

  populate_snapshot(*input, 1);
  require(bus->push(*input) == lingtu::sim::SnapshotPushResult::published,
          "capacity-one bus must publish its first value");
  populate_snapshot(*input, 2);
  require(bus->push(*input) == lingtu::sim::SnapshotPushResult::published_replacing_latest,
          "capacity-one bus must replace its published value");
  require(bus->latest(*out) && out->sequence == 2 &&
              out->generation == lingtu::sim::GenerationStamp{4, 2} &&
              out->entities.size() == 2 &&
              out->entities[0].position_m[0] == 2.0,
          "capacity-one bus must expose the complete replacement");
}

bool is_complete_snapshot(const lingtu::sim::TruthSnapshotEnvelope &value) {
  return value.session_id.view() == kSessionId &&
         value.generation == lingtu::sim::GenerationStamp{4, 2} &&
         value.entities.size() == 2 &&
         value.entities[0].instance_id.view() == "alpha" &&
         value.entities[0].frame_id.view() == "base_link" &&
         value.entities[0].stable_id.view() == "alpha/base_link" &&
         value.entities[1].instance_id.view() == "beta" &&
         value.entities[1].frame_id.view() == "base_link" &&
         value.entities[1].stable_id.view() == "beta/base_link" &&
         value.entities[0].position_m[0] == static_cast<double>(value.sequence) &&
         value.entities[1].position_m[0] == static_cast<double>(value.sequence + 1);
}

void test_snapshot_bus_concurrent_producer_consumers() {
  auto bus = std::make_unique<lingtu::sim::SnapshotBus<3>>();
  auto input = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();
  constexpr std::uint64_t total = 5'000;
  std::atomic<bool> producer_done{false};
  std::atomic<bool> invalid_snapshot{false};
  std::atomic<std::uint64_t> observed_snapshots{0};
  std::atomic<std::uint64_t> dropped_snapshots{0};

  auto consume = [&]() {
    auto out = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();
    while (!producer_done.load(std::memory_order_acquire)) {
      if (bus->latest(*out)) {
        if (!is_complete_snapshot(*out)) {
          invalid_snapshot.store(true, std::memory_order_release);
        }
        observed_snapshots.fetch_add(1, std::memory_order_relaxed);
      } else {
        std::this_thread::yield();
      }
    }
  };

  std::thread consumer_a(consume);
  std::thread consumer_b(consume);
  std::thread producer([&]() {
    for (std::uint64_t sequence = 1; sequence <= total; ++sequence) {
      populate_snapshot(*input, sequence);
      if (bus->push(*input) ==
          lingtu::sim::SnapshotPushResult::dropped_no_free_slot) {
        dropped_snapshots.fetch_add(1, std::memory_order_relaxed);
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  producer.join();
  consumer_a.join();
  consumer_b.join();

  require(!invalid_snapshot.load(std::memory_order_acquire),
          "concurrent readers must observe complete snapshots only");
  require(observed_snapshots.load(std::memory_order_relaxed) > 0,
          "concurrent readers must observe published snapshots");
  require(dropped_snapshots.load(std::memory_order_relaxed) == 0,
          "synchronized latest-value publication must not drop snapshots");

  populate_snapshot(*input, total);
  require(bus->push(*input) != lingtu::sim::SnapshotPushResult::dropped_no_free_slot,
          "a quiescent producer must have a free slot");
  auto out = std::make_unique<lingtu::sim::TruthSnapshotEnvelope>();
  require(bus->latest(*out) && is_complete_snapshot(*out) && out->sequence == total,
          "final snapshot must remain complete after pressure");
}

void test_command_inbox_generation_and_fifo() {
  const lingtu::sim::GenerationStamp active{4, 2};
  lingtu::sim::CommandInbox<2> inbox(active);
  require(inbox.push(command(1, active, "alpha")) == lingtu::sim::CommandPushResult::accepted,
          "current generation command must be accepted");
  require(inbox.push(command(2, active, "beta")) == lingtu::sim::CommandPushResult::accepted,
          "second command must be accepted");
  require(inbox.push(command(3, active, "gamma")) == lingtu::sim::CommandPushResult::rejected_queue_full,
          "full queue must return an explicit result");
  require(inbox.push(command(4, {3, 2}, "alpha")) ==
              lingtu::sim::CommandPushResult::rejected_stale_model_generation,
          "old model generation must be rejected");
  require(inbox.push(command(5, {4, 1}, "alpha")) ==
              lingtu::sim::CommandPushResult::rejected_stale_reset_generation,
          "old reset generation must be rejected");
  require(inbox.push(command(6, {5, 2}, "alpha")) ==
              lingtu::sim::CommandPushResult::rejected_future_model_generation,
          "future model generation must not enter the active queue");

  lingtu::sim::CommandEnvelope out;
  require(inbox.pop(out) && out.sequence == 1, "commands must pop FIFO");
  require(inbox.pop(out) && out.sequence == 2, "commands must preserve FIFO order");
  require(!inbox.pop(out), "empty command inbox must not pop");

  require(inbox.push(command(7, active, "alpha")) == lingtu::sim::CommandPushResult::accepted,
          "queue must accept after pop");
  inbox.set_generation({4, 3});
  require(inbox.size() == 0, "generation change must clear old queued commands");
  require(inbox.push(command(8, active, "alpha")) ==
              lingtu::sim::CommandPushResult::rejected_stale_reset_generation,
          "commands from a prior reset must remain rejected");
}

void test_command_inbox_concurrent_generation_switch() {
  const lingtu::sim::GenerationStamp initial{9, 0};
  lingtu::sim::CommandInbox<128> inbox(initial);
  constexpr std::uint64_t total = 5'000;
  std::atomic<bool> producer_done{false};
  std::atomic<bool> invalid_command{false};

  std::thread consumer([&]() {
    lingtu::sim::CommandEnvelope out;
    while (!producer_done.load(std::memory_order_acquire) || inbox.size() != 0) {
      if (inbox.pop(out)) {
        if (out.payload_size == 0 || out.type.empty()) {
          invalid_command.store(true, std::memory_order_release);
        }
      } else {
        std::this_thread::yield();
      }
    }
  });

  std::thread generation_switcher([&]() {
    for (std::uint64_t reset = 1; reset <= 64; ++reset) {
      inbox.set_generation({9, reset});
    }
  });

  std::thread producer([&]() {
    for (std::uint64_t sequence = 1; sequence <= total; ++sequence) {
      const auto result = inbox.push(command(sequence, {9, sequence % 65}, "alpha"));
      (void)result;
    }
    producer_done.store(true, std::memory_order_release);
  });

  producer.join();
  generation_switcher.join();
  consumer.join();

  require(!invalid_command.load(std::memory_order_acquire),
          "concurrent command push/pop/generation changes must remain coherent");
  inbox.set_generation({9, 100});
  require(inbox.generation() == lingtu::sim::GenerationStamp{9, 100},
          "generation switch must be visible after concurrent activity");
  require(inbox.size() == 0, "generation switch must clear queued commands");
}
void test_multiple_robot_ids_are_not_runtime_special_cases() {
  lingtu::sim::TruthSnapshotEnvelope value;
  require(value.entities.push_back(entity("thunder_01", "base_link", "thunder_01/base_link", 0.0)), "robot one must fit");
  require(value.entities.push_back(entity("omnicart_01", "base_link", "omnicart_01/base_link", 3.0)), "robot two must fit");
  require(value.entities[0].stable_id != value.entities[1].stable_id,
          "stable IDs must distinguish multiple robot instances");
  require(value.entities[0].stable_id.view() == "thunder_01/base_link", "first ID must be stable");
  require(value.entities[1].stable_id.view() == "omnicart_01/base_link", "second ID must be stable");
}

}  // namespace

int main() {
  try {
    test_contract_shapes_and_fixed_capacity();
    test_snapshot_bus_publishes_only_complete_latest_value();
    test_snapshot_bus_capacity_one_replaces_latest_value();
    test_snapshot_bus_concurrent_producer_consumers();
    test_command_inbox_generation_and_fifo();
    test_command_inbox_concurrent_generation_switch();
    test_multiple_robot_ids_are_not_runtime_special_cases();
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "PASS: runtime contracts, bounded snapshot bus, and command inbox\n";
  return EXIT_SUCCESS;
}
