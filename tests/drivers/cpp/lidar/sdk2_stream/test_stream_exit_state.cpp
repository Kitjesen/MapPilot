#include "stream_exit_state.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

namespace {

using lingtu::drivers::lidar::StreamExitState;

void expect(bool condition) {
  if (!condition) {
    std::abort();
  }
}

}  // namespace

int main() {
  using namespace std::chrono;

  // A fatal timestamp request publishes the fault before quit. Observing quit
  // must therefore make the fault and the non-flushing exit decision visible.
  for (int iteration = 0; iteration < 256; ++iteration) {
    StreamExitState state;
    std::thread producer([&state] {
      expect(state.request_timestamp_fault());
    });
    while (!state.quit_requested()) {
      std::this_thread::yield();
    }
    const auto decision = state.snapshot_exit_decision();
    expect(decision.timestamp_fault);
    expect(!decision.flush_final_batch);
    expect(decision.return_code == 1);
    producer.join();
    expect(!state.request_timestamp_fault());
  }

  // A fatal request that wins before the waiter starts must not be lost.
  StreamExitState already_stopped;
  expect(already_stopped.request_timestamp_fault());
  const auto before_wait = steady_clock::now();
  already_stopped.wait_for_stop();
  expect(steady_clock::now() - before_wait < milliseconds(50));

  // Signal handlers only perform the lock-free quit store. The timed predicate
  // wait must still observe it without relying on a condition-variable notify.
  StreamExitState signal_stopped;
  std::thread signaler([&signal_stopped] {
    std::this_thread::sleep_for(milliseconds(10));
    signal_stopped.request_signal_stop();
  });
  const auto signal_wait_start = steady_clock::now();
  signal_stopped.wait_for_stop();
  const auto signal_wait_elapsed = steady_clock::now() - signal_wait_start;
  signaler.join();
  expect(signal_wait_elapsed < milliseconds(250));
  const auto clean_decision = signal_stopped.snapshot_exit_decision();
  expect(!clean_decision.timestamp_fault);
  expect(clean_decision.flush_final_batch);
  expect(clean_decision.return_code == 0);

  return 0;
}
