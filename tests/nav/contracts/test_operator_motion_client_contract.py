from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


def test_cpp_operator_motion_client_uses_declared_frames_and_clock_sync() -> None:
    source = Path("src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")
    control = source[
        source.index("OperatorMotionCommandReceipt writeOperatorMotionControl") : source.index(
            "void writeOperatorMotionSample"
        )
    ]
    sample = source[source.index("void writeOperatorMotionSample") : source.index("static bool requiresEndpointClock")]

    assert 'fillHeader(message.header, nowSeconds(), "");' in control
    assert 'fillHeader(message.header, send_source_stamp_s, "body");' in sample
    assert "synchronizeEndpointClock(timeout_ms);" in sample
    assert "message.source_stamp_ns = sourceStampNs(send_source_stamp_s);" in sample


def test_cpp_operator_motion_client_keeps_clock_sync_as_stop_handshake() -> None:
    source = Path("src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")
    sync = source[source.index("void synchronizeEndpointClock") : source.index("std::string writeCommand")]

    assert "CommandKind::Stop" in sync
    assert 'sync.reason = const_cast<char*>("client_clock_sync");' in sync


@pytest.fixture(scope="module")
def operator_sample_probe(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Compile the production method with deterministic waits and a recording DDS sink."""
    compiler = next((path for name in ("c++", "g++", "clang++") if (path := shutil.which(name))), None)
    if compiler is None:
        pytest.skip("a C++ compiler is required for the operator sample behavior probe")
    source = Path("src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")
    sample = source[source.index("void writeOperatorMotionSample") : source.index("static bool requiresEndpointClock")]
    probe = r'''
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

struct SteadyClock {
  using time_point = std::chrono::steady_clock::time_point;
  static inline std::atomic<long long> elapsed_ms{0};
  static time_point now() {
    return time_point(std::chrono::milliseconds(elapsed_ms.load()));
  }
};
struct Vector { double x{}, y{}, z{}; };
struct Velocity { Vector linear{}, angular{}; };
struct Header {};
struct lingtu_dds_OperatorMotionSample {
  Header header{};
  char *source_id{}, *request_id{};
  std::uint64_t source_epoch{}, source_sequence{}, source_stamp_ns{};
  bool deadman{}, manual_mode{};
  Velocity velocity{};
  std::uint32_t freshness_budget_ms{};
};
std::optional<lingtu_dds_OperatorMotionSample> published;
int dds_write(int, const void *sample) {
  published = *static_cast<const lingtu_dds_OperatorMotionSample *>(sample);
  return 0;
}
void checked(int, const char *) {}
void fillHeader(Header &, double, const char *) {}
std::uint64_t sourceStampNs(double seconds) {
  return static_cast<std::uint64_t>(seconds * 1e9);
}
void requireFinite(double value, const char *) {
  if (!std::isfinite(value)) throw std::invalid_argument("nonfinite velocity");
}
std::string makeOperatorMotionSampleRequestId() { return "sample"; }

struct ClientProbe {
  mutable std::mutex clock_command_mutex;
  mutable std::atomic<bool> entered{false};
  mutable std::atomic<double> endpoint_clock_offset_s{0.0};
  int operator_motion_sample_writer{1};
  int reader_delay_ms{}, sync_delay_ms{};
  void requireOperatorSource(const std::string &, std::uint64_t) const {
    entered.store(true);
  }
  void waitForReader(int, const char *, int) const {
    SteadyClock::elapsed_ms.fetch_add(reader_delay_ms);
  }
  void synchronizeEndpointClock(int) const {
    SteadyClock::elapsed_ms.fetch_add(sync_delay_ms);
    endpoint_clock_offset_s.store(0.0);
  }
  double sourceNowSeconds() const {
    return 1000.0 + static_cast<double>(SteadyClock::elapsed_ms.load()) / 1000.0;
  }
''' + sample + r'''
};

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string scenario = argv[1];
  ClientProbe client;
  int expected_budget = 350;
  if (scenario == "reader_partial") {
    client.reader_delay_ms = 100;
    expected_budget = 250;
  } else if (scenario == "initial_sync_partial") {
    client.reader_delay_ms = 100;
    client.sync_delay_ms = 150;
    client.endpoint_clock_offset_s.store(std::numeric_limits<double>::quiet_NaN());
    expected_budget = 100;
  } else if (scenario == "reader_expired") {
    client.reader_delay_ms = 400;
    expected_budget = 0;
  } else if (scenario == "initial_sync_expired") {
    client.sync_delay_ms = 400;
    client.endpoint_clock_offset_s.store(std::numeric_limits<double>::quiet_NaN());
    expected_budget = 0;
  } else if (scenario == "mutex_expired") {
    expected_budget = 0;
  } else if (scenario != "ready") {
    return 2;
  }
  std::string error;
  const auto send = [&] {
    try {
      client.writeOperatorMotionSample("operator", 1, 2, 0.2, 0.0, 0.1,
                                       true, 350, 1000, "sample", true);
    } catch (const std::runtime_error &exc) {
      error = exc.what();
    }
  };
  if (scenario == "mutex_expired") {
    std::unique_lock<std::mutex> lock(client.clock_command_mutex);
    std::thread sender(send);
    while (!client.entered.load()) std::this_thread::yield();
    SteadyClock::elapsed_ms.store(400);
    lock.unlock();
    sender.join();
  } else {
    send();
  }
  std::cout << scenario << ": published=" << published.has_value()
            << " budget=" << (published ? published->freshness_budget_ms : 0)
            << " elapsed_ms=" << SteadyClock::elapsed_ms.load()
            << " error=" << error << '\n';
  if (expected_budget == 0) return !published && !error.empty() ? 0 : 1;
  if (!published || !error.empty()) return 1;
  return published->freshness_budget_ms == static_cast<std::uint32_t>(expected_budget)
      && published->velocity.linear.x == 0.2 && published->manual_mode
      && published->source_stamp_ns == sourceStampNs(client.sourceNowSeconds()) ? 0 : 1;
}
'''
    directory = tmp_path_factory.mktemp("operator_sample")
    source_path = directory / "probe.cpp"
    executable = directory / "probe.exe"
    source_path.write_text(probe, encoding="utf-8")
    result = subprocess.run(
        [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread", str(source_path), "-o", str(executable)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return executable


@pytest.mark.parametrize(
    "scenario",
    ["ready", "reader_partial", "initial_sync_partial", "reader_expired", "initial_sync_expired", "mutex_expired"],
)
def test_cpp_operator_motion_sample_waits_consume_freshness(operator_sample_probe: Path, scenario: str) -> None:
    result = subprocess.run([str(operator_sample_probe), scenario], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
