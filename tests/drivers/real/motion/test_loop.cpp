#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "body.hpp"
#include "config.hpp"
#include "dds.hpp"

int runDriver(int argc, char **argv);

namespace {
using namespace lingtu::driver;
using namespace std::chrono_literals;
const std::string host = "test-host";
std::string scenario;
std::optional<TwistSample> pending;
std::vector<Velocity> sent;
std::vector<bool> zero_outputs;
unsigned stop_calls = 0;
unsigned reads = 0;

TwistSample command(double vx) {
  TwistSample sample;
  sample.host_boot_id = host;
  sample.producer_boot_id = "nav";
  sample.received_at = Clock::now();
  sample.source_boottime_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(sample.received_at.time_since_epoch()).count());
  if (scenario == "clock") sample.source_boottime_ns += 9000000000ULL;
  sample.receive_boottime_ns = sample.source_boottime_ns;
  sample.vx = vx;
  return sample;
}

class TestBody final : public Body {
 public:
  Result refresh() override {
    // Newer commands arrive while the synchronous vendor call is in flight.
    if (scenario == "latest") {
      pending = command(0.2);
      pending = command(0.0);
      std::this_thread::sleep_for(30ms);
    } else if (scenario == "replace" || scenario == "stale") {
      pending = command(0.2);
      if (scenario == "stale") pending->source_boottime_ns -= 1000000000ULL;
      std::this_thread::sleep_for(30ms);
    } else if (scenario != "rejection" && scenario != "unready" &&
               scenario != "recovery" && scenario != "writer_missing") {
      std::this_thread::sleep_for(220ms);
    }
    Result result;
    result.ok = result.accepted = result.transport_ok = true;
    result.state.ready = result.state.connected = result.state.control_assured = true;
    result.state.motors_enabled = true;
    if (scenario == "unready") result.ok = result.state.ready = false;
    return result;
  }
  Result move(const Velocity &velocity) override {
    sent.push_back(velocity);
    if (scenario == "rejection") std::this_thread::sleep_for(100ms);
    if (sent.size() == (scenario == "recovery" ? 3U : 1U)) std::raise(SIGTERM);
    Result result;
    result.transport_ok = result.state.connected = true;
    result.ok = result.accepted = result.state.ready = result.state.control_assured = scenario != "rejection";
    if (scenario == "rejection") result.state.reason = "vendor_rejected";
    if (scenario == "recovery" && sent.size() == 1) {
      result.ok = result.accepted = false;
      result.state.reason = "vendor_rejected";
    }
    if (scenario == "recovery" && sent.size() == 2) pending = command(0.2);
    result.state.motors_enabled = true;
    return result;
  }
  Result stop() noexcept override {
    ++stop_calls;
    Result result;
    result.ok = result.accepted = result.transport_ok = result.stop_confirmed = true;
    return result;
  }
  Result act(BodyAction) override { return {}; }
  Capabilities capabilities() const noexcept override { return {}; }
  BodyState state() const override { return {}; }
  HealthState health() const override { return {}; }
  AdapterDiagnostics diagnostics() const override { return {}; }
};
}  // namespace

namespace lingtu::driver {
std::unique_ptr<Body> makeBody(const Config &) { return std::make_unique<TestBody>(); }
struct DdsReader::Impl {};
DdsReader::DdsReader(int) {
  pending = command(scenario == "replace" || scenario == "stale" || scenario == "rejection" ? 0.0 : 0.1);
}
DdsReader::~DdsReader() = default;
const std::string &DdsReader::hostBootId() const noexcept { return host; }
std::uint32_t DdsReader::matchedCommandWriters() const { return scenario == "writer_missing" ? 0 : 1; }
ReadResult DdsReader::takeLatest() {
  if (++reads > 3 && sent.empty()) std::raise(SIGTERM);
  ReadResult result;
  result.latest = std::exchange(pending, std::nullopt);
  result.valid_samples = result.latest ? 1 : 0;
  return result;
}
std::optional<ProfileRequest> DdsReader::takeProfileRequest(double) { return {}; }
std::optional<message::PostureRequest> DdsReader::takePostureRequest() { return {}; }
DdsReader::ExternalRead DdsReader::takeExternal() { return {}; }
bool DdsReader::writePostureAck(const message::PostureAck &, double) { return true; }
bool DdsReader::writeExternalState(const message::ExternalState &, double) { return true; }
bool DdsReader::writeProfileState(const ProfileStatus &, double) { return true; }
bool DdsReader::writeJointState(const JointState &) { return true; }
bool DdsReader::writeControlState(const ControlState &, bool zero_output, double) {
  zero_outputs.push_back(zero_output);
  return true;
}
}  // namespace lingtu::driver

int main(int argc, char **argv) {
  scenario = argc == 2 ? argv[1] : "latest";
  const auto file = std::filesystem::temp_directory_path() /
      ("lingtu-driver-loop-" + std::to_string(Clock::now().time_since_epoch().count()) + ".json");
  std::vector<std::string> arguments{
      "driver", "--robot", "go2", "--network-interface", "lo", "--poll-hz", "50",
      "--cmd-timeout-ms", "200", "--rpc-timeout-ms", "200", "--status-file", file.string()};
  std::vector<char *> pointers;
  for (auto &argument : arguments) pointers.push_back(argument.data());
  const int result = runDriver(static_cast<int>(pointers.size()), pointers.data());
  std::filesystem::remove(file);
  const double expected = scenario == "replace" ? 0.2 :
      (scenario == "unready" || scenario == "writer_missing" || scenario == "recovery") ? 0.1 : 0.0;
  const std::size_t expected_sends = scenario == "recovery" ? 3 : scenario == "rejection" ? 2 : 1;
  if (result != 0 || sent.size() != expected_sends || sent.front().vx_mps != expected || stop_calls != 1) {
    std::fputs("driver did not preserve the latest-command contract\n", stderr);
    return 1;
  }
  if (scenario == "recovery" && (sent[1].vx_mps != 0.0 || sent[2].vx_mps != 0.2)) {
    std::fputs("vendor failure must stop once and allow the next command without release\n", stderr);
    return 1;
  }
  if (zero_outputs.empty() || !zero_outputs.back()) {
    std::fputs("successful shutdown must report zero output\n", stderr);
    return 1;
  }
  if (scenario == "rejection" && zero_outputs.front()) {
    std::fputs("failed stop must not report zero output\n", stderr);
    return 1;
  }
  std::printf("driver scenario %s: latest command and truthful zero output\n", scenario.c_str());
  return 0;
}
