#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "dds_domain.hpp"
#include "local_endpoint_server.hpp"
#include "mujoco_driver_bridge_core.hpp"
#include "run_plan_process_environment.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <process.h>
#include <windows.h>
#else
#include <cerrno>
#include <poll.h>
#include <time.h>
#include <unistd.h>
#endif

#include <dds/dds.h>

#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"

namespace {

using lingtu::sim::driver_bridge::ActivateMessage;
using lingtu::sim::driver_bridge::AppliedMessage;
using lingtu::sim::driver_bridge::BridgeCommand;
using lingtu::sim::driver_bridge::BridgeCommandKind;
using lingtu::sim::driver_bridge::BridgeConfig;
using lingtu::sim::driver_bridge::BridgeFaultCode;
using lingtu::sim::driver_bridge::BridgeLifecycle;
using lingtu::sim::driver_bridge::BridgeStatus;
using lingtu::sim::driver_bridge::BridgeStopCause;
using lingtu::sim::driver_bridge::Clock;
using lingtu::sim::driver_bridge::ControllerMessage;
using lingtu::sim::driver_bridge::DeactivateMessage;
using lingtu::sim::driver_bridge::FaultMessage;
using lingtu::sim::driver_bridge::HeartbeatMessage;
using lingtu::sim::driver_bridge::HelloMessage;
using lingtu::sim::driver_bridge::MujocoDriverBridgeCore;
using lingtu::sim::driver_bridge::NavCommand;
using lingtu::sim::driver_bridge::ProtocolError;
using lingtu::sim::driver_bridge::ReadyMessage;
using lingtu::sim::driver_bridge::StopMessage;
using lingtu::sim::driver_bridge::TimePoint;

namespace local_endpoint = lingtu::sim::local_endpoint;
namespace run_plan_process = lingtu::sim::run_plan_process;

constexpr auto kControlStatePeriod = std::chrono::milliseconds(50);
constexpr auto kEndpointAcceptPoll = std::chrono::milliseconds(250);
constexpr auto kEndpointIoTimeout = std::chrono::seconds(2);
constexpr auto kEndpointPollTimeout = std::chrono::milliseconds(1);
constexpr std::string_view kEndpointRole = "driver_bridge";
constexpr std::string_view kEndpointProtocol = "driver-v2";

volatile std::sig_atomic_t g_stop_requested = 0;

void stopSignal(int) {
  g_stop_requested = 1;
}

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

long processId() noexcept {
#ifdef _WIN32
  return static_cast<long>(_getpid());
#else
  return static_cast<long>(getpid());
#endif
}

std::uint64_t bootTimeNanoseconds() {
#ifdef _WIN32
  LARGE_INTEGER counter{};
  LARGE_INTEGER frequency{};
  if (QueryPerformanceCounter(&counter) == 0 || QueryPerformanceFrequency(&frequency) == 0 ||
      counter.QuadPart <= 0 || frequency.QuadPart <= 0) {
    throw std::runtime_error("QueryPerformanceCounter failed");
  }
  const auto ticks = static_cast<std::uint64_t>(counter.QuadPart);
  const auto ticks_per_second = static_cast<std::uint64_t>(frequency.QuadPart);
  const auto seconds = ticks / ticks_per_second;
  const auto remainder = ticks % ticks_per_second;
  if (seconds > std::numeric_limits<std::uint64_t>::max() / 1'000'000'000ULL) {
    throw std::runtime_error("host boot clock overflow");
  }
  return seconds * 1'000'000'000ULL + remainder * 1'000'000'000ULL / ticks_per_second;
#elif defined(__linux__)
  timespec value{};
  if (clock_gettime(CLOCK_BOOTTIME, &value) != 0 || value.tv_sec < 0 || value.tv_nsec < 0) {
    throw std::runtime_error("clock_gettime(CLOCK_BOOTTIME) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
#else
  const auto count =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  if (count <= 0) {
    throw std::runtime_error("steady clock is not positive");
  }
  return static_cast<std::uint64_t>(count);
#endif
}

TimePoint bridgeNow() {
  const auto value = bootTimeNanoseconds();
  if (value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::runtime_error("host boot clock exceeds signed nanoseconds");
  }
  return TimePoint{std::chrono::duration_cast<Clock::duration>(
      std::chrono::nanoseconds(static_cast<std::int64_t>(value)))};
}

struct Config {
  int domain_id{0};
  std::filesystem::path ready_file;
  std::string bridge_boot_id;
  std::string expected_product_session_id;
  lingtu::driver::Limits limits;
  std::chrono::milliseconds heartbeat_timeout{500};
  std::chrono::milliseconds apply_timeout{500};
  std::chrono::milliseconds poll_period{5};
  bool local_endpoint{false};
  std::optional<run_plan_process::RunPlanProcessEnvironment> endpoint_environment;
  bool help{false};
};

std::string_view usage() noexcept {
  return "usage: lingtu_mujoco_driver_bridge --domain-id N --ready-file PATH "
         "--bridge-boot-id 32_HEX --expected-host-boot-id TOKEN "
         "or: lingtu_mujoco_driver_bridge --domain-id N --local-endpoint "
         "[--max-linear-mps N] [--max-angular-rps N] "
         "[--command-timeout-ms N] [--heartbeat-timeout-ms N] "
         "[--apply-timeout-ms N] [--poll-ms N]\n";
}

std::string randomBootId() {
  std::random_device random;
  constexpr char kHex[] = "0123456789abcdef";
  std::string result(32, '0');
  for (std::size_t index = 0; index < result.size(); index += 2) {
    const auto value = static_cast<unsigned int>(random()) & 0xffU;
    result[index] = kHex[value >> 4U];
    result[index + 1] = kHex[value & 0x0fU];
  }
  return result;
}

std::uint64_t parsePositiveInteger(std::string_view value, std::string_view option,
                                   std::uint64_t maximum) {
  std::uint64_t parsed = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed, 10);
  if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
      parsed == 0 || parsed > maximum) {
    throw std::runtime_error(std::string(option) + " must be a positive bounded integer");
  }
  return parsed;
}

double parsePositiveDouble(std::string_view value, std::string_view option) {
  double parsed = 0.0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed,
                                      std::chars_format::general);
  if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
      !std::isfinite(parsed) || parsed <= 0.0) {
    throw std::runtime_error(std::string(option) + " must be finite and positive");
  }
  return parsed;
}

Config parseArgs(int argc, char **argv) {
  Config config;
  bool ready_file_set = false;
  bool bridge_boot_id_set = false;
  bool expected_product_session_id_set = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto next = [&]() -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error("missing value for " + argument);
      }
      return argv[++index];
    };
    if (argument == "--domain-id") {
      config.domain_id = lingtu::sim::dds_adapter::parse_supported_dds_domain_id(next());
    } else if (argument == "--ready-file") {
      config.ready_file = next();
      ready_file_set = true;
    } else if (argument == "--bridge-boot-id") {
      config.bridge_boot_id = next();
      bridge_boot_id_set = true;
    } else if (argument == "--expected-host-boot-id") {
      config.expected_product_session_id = next();
      expected_product_session_id_set = true;
    } else if (argument == "--local-endpoint") {
      config.local_endpoint = true;
    } else if (argument == "--max-linear-mps") {
      config.limits.max_linear_mps = parsePositiveDouble(next(), argument);
    } else if (argument == "--max-angular-rps") {
      config.limits.max_angular_rps = parsePositiveDouble(next(), argument);
    } else if (argument == "--command-timeout-ms") {
      config.limits.command_timeout =
          std::chrono::milliseconds(parsePositiveInteger(next(), argument, 60'000));
    } else if (argument == "--heartbeat-timeout-ms") {
      config.heartbeat_timeout =
          std::chrono::milliseconds(parsePositiveInteger(next(), argument, 60'000));
    } else if (argument == "--apply-timeout-ms") {
      config.apply_timeout =
          std::chrono::milliseconds(parsePositiveInteger(next(), argument, 60'000));
    } else if (argument == "--poll-ms") {
      config.poll_period = std::chrono::milliseconds(parsePositiveInteger(next(), argument, 1'000));
    } else if (argument == "--help" || argument == "-h") {
      config.help = true;
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (config.help) {
    return config;
  }
  if (config.local_endpoint) {
    if (ready_file_set || bridge_boot_id_set || expected_product_session_id_set) {
      throw std::runtime_error("--local-endpoint rejects legacy ready-file and boot-id arguments");
    }
    config.endpoint_environment = run_plan_process::loadRunPlanProcessEnvironment(
        run_plan_process::EndpointFiles{"driver.ready.json", "driver.auth"});
    config.bridge_boot_id = randomBootId();
    config.expected_product_session_id = config.endpoint_environment->product_session_id;
    return config;
  }
  if (config.ready_file.empty()) {
    throw std::runtime_error("--ready-file is required");
  }
  if (!lingtu::sim::driver_bridge::validBootId(config.bridge_boot_id)) {
    throw std::runtime_error("--bridge-boot-id must be lowercase 32 hex");
  }
  if (!lingtu::sim::driver_bridge::validProducerToken(config.expected_product_session_id)) {
    throw std::runtime_error("--expected-host-boot-id must be a safe non-empty token");
  }
  return config;
}

class ReadyFile final {
 public:
  explicit ReadyFile(std::filesystem::path path) : path_(std::move(path)) {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
    std::filesystem::remove(temporaryPath(), ignored);
  }

  ReadyFile(const ReadyFile &) = delete;
  ReadyFile &operator=(const ReadyFile &) = delete;

  ~ReadyFile() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
    std::filesystem::remove(temporaryPath(), ignored);
  }

  void publish() {
    if (published_) {
      return;
    }
    if (path_.has_parent_path()) {
      std::filesystem::create_directories(path_.parent_path());
    }
    const auto temporary = temporaryPath();
    {
      std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
      if (!output) {
        throw std::runtime_error("failed to open bridge ready marker");
      }
      output << "ready\n";
      if (!output) {
        throw std::runtime_error("failed to write bridge ready marker");
      }
    }
    std::error_code rename_error;
    std::filesystem::rename(temporary, path_, rename_error);
    if (rename_error) {
      throw std::runtime_error("failed to publish bridge ready marker: " + rename_error.message());
    }
    published_ = true;
  }

 private:
  std::filesystem::path temporaryPath() const {
    auto path = path_;
    path += ".tmp";
    return path;
  }

  std::filesystem::path path_;
  bool published_{false};
};

struct InputBatch {
  std::vector<std::string> lines;
  bool eof{false};
};

void appendProtocolBytes(std::string &pending, const std::uint8_t *bytes, std::size_t size,
                         std::vector<std::string> &lines) {
  for (std::size_t index = 0; index < size; ++index) {
    const std::uint8_t value = bytes[index];
    if (value == static_cast<std::uint8_t>('\n')) {
      if (!pending.empty() && pending.back() == '\r') {
        pending.pop_back();
      }
      lines.push_back(std::move(pending));
      pending.clear();
      continue;
    }
    if ((value < 0x20U && value != static_cast<std::uint8_t>('\t') &&
         value != static_cast<std::uint8_t>('\r')) ||
        value > 0x7eU) {
      throw ProtocolError("controller protocol must be printable ASCII");
    }
    pending.push_back(static_cast<char>(value));
    if (pending.size() > lingtu::sim::driver_bridge::kMaxProtocolLineBytes) {
      throw ProtocolError("controller protocol line exceeds fixed byte limit");
    }
  }
}

void validateProtocolEof(const std::string &pending) {
  if (!pending.empty()) {
    throw ProtocolError("controller EOF split a protocol line");
  }
}

class StdinLines final {
 public:
  StdinLines() {
#ifdef _WIN32
    handle_ = GetStdHandle(STD_INPUT_HANDLE);
    if (handle_ == nullptr || handle_ == INVALID_HANDLE_VALUE ||
        GetFileType(handle_) != FILE_TYPE_PIPE) {
      throw std::runtime_error("controller stdin must be an inherited pipe");
    }
#endif
  }

  InputBatch readAvailable() {
    InputBatch batch;
    char bytes[1024];
    while (true) {
#ifdef _WIN32
      DWORD available = 0;
      if (!PeekNamedPipe(handle_, nullptr, 0, nullptr, &available, nullptr)) {
        const DWORD error = GetLastError();
        if (error == ERROR_BROKEN_PIPE) {
          batch.eof = true;
          break;
        }
        throw std::runtime_error("PeekNamedPipe(controller stdin) failed");
      }
      if (available == 0) {
        break;
      }
      const DWORD requested = std::min<DWORD>(available, static_cast<DWORD>(sizeof(bytes)));
      DWORD read_count = 0;
      if (!ReadFile(handle_, bytes, requested, &read_count, nullptr)) {
        const DWORD error = GetLastError();
        if (error == ERROR_BROKEN_PIPE) {
          batch.eof = true;
          break;
        }
        throw std::runtime_error("ReadFile(controller stdin) failed");
      }
      if (read_count == 0) {
        batch.eof = true;
        break;
      }
      appendProtocolBytes(pending_, reinterpret_cast<const std::uint8_t *>(bytes),
                          static_cast<std::size_t>(read_count), batch.lines);
#else
      pollfd input{};
      input.fd = STDIN_FILENO;
      input.events = POLLIN | POLLHUP;
      const int ready = poll(&input, 1, 0);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("poll(controller stdin) failed");
      }
      if (ready == 0) {
        break;
      }
      if ((input.revents & (POLLERR | POLLNVAL)) != 0) {
        throw std::runtime_error("controller stdin pipe failed");
      }
      const ssize_t read_count = read(STDIN_FILENO, bytes, sizeof(bytes));
      if (read_count < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("read(controller stdin) failed");
      }
      if (read_count == 0) {
        batch.eof = true;
        break;
      }
      appendProtocolBytes(pending_, reinterpret_cast<const std::uint8_t *>(bytes),
                          static_cast<std::size_t>(read_count), batch.lines);
#endif
    }
    if (batch.eof) {
      validateProtocolEof(pending_);
    }
    return batch;
  }

 private:
  std::string pending_;
#ifdef _WIN32
  HANDLE handle_{nullptr};
#endif
};

class EndpointLines final {
 public:
  explicit EndpointLines(local_endpoint::ClientStream stream) : stream_(std::move(stream)) {}

  InputBatch readAvailable() {
    InputBatch batch;
    std::optional<std::vector<std::uint8_t>> chunk;
    try {
      chunk = stream_.readSome(1024, kEndpointPollTimeout);
    } catch (const local_endpoint::EndpointTimeout &) {
      return batch;
    }
    if (!chunk.has_value()) {
      batch.eof = true;
      validateProtocolEof(pending_);
      return batch;
    }
    appendProtocolBytes(pending_, chunk->data(), chunk->size(), batch.lines);
    return batch;
  }

  void writeLine(const std::string &line) {
    if (line.size() > lingtu::sim::driver_bridge::kMaxProtocolLineBytes) {
      throw ProtocolError("controller protocol line exceeds fixed byte limit");
    }
    std::vector<std::uint8_t> payload(line.begin(), line.end());
    payload.push_back(static_cast<std::uint8_t>('\n'));
    stream_.writeAll(payload, kEndpointIoTimeout);
  }

 private:
  local_endpoint::ClientStream stream_;
  std::string pending_;
};

class ControllerTransport final {
 public:
  explicit ControllerTransport(const Config &config) : local_(config.local_endpoint) {
    if (local_) {
      if (!config.endpoint_environment.has_value()) {
        throw std::runtime_error("local endpoint environment is missing");
      }
      const auto &identity = *config.endpoint_environment;
      endpoint_server_ =
          std::make_unique<local_endpoint::LocalEndpointServer>(local_endpoint::ServerConfig{
              std::string(kEndpointRole),
              std::string(kEndpointProtocol),
              identity.product_session_id,
              identity.readiness_path,
              identity.auth_file_name,
          });
      return;
    }
    stdin_lines_ = std::make_unique<StdinLines>();
    ready_file_ = std::make_unique<ReadyFile>(config.ready_file);
  }

  void startTransport() {
    if (!local_ || endpoint_started_) {
      return;
    }
    endpoint_server_->start();
    endpoint_started_ = true;
  }

  bool prepare(const std::string &bridge_boot_id) {
    if (prepared_) {
      return true;
    }
    if (!local_) {
      ready_file_->publish();
      prepared_ = true;
      return true;
    }
    startTransport();
    try {
      endpoint_lines_ = std::make_unique<EndpointLines>(
          endpoint_server_->acceptAuthenticated(kEndpointAcceptPoll));
    } catch (const local_endpoint::EndpointTimeout &) {
      return false;
    }
    prepared_ = true;
    writeLine(lingtu::sim::driver_bridge::serializeHello(HelloMessage{bridge_boot_id}));
    return true;
  }

  InputBatch readAvailable() {
    if (!prepared_) {
      return {};
    }
    return local_ ? endpoint_lines_->readAvailable() : stdin_lines_->readAvailable();
  }

  void writeLine(const std::string &line) {
    if (local_) {
      if (!prepared_ || endpoint_lines_ == nullptr) {
        throw local_endpoint::EndpointError("driver endpoint has no authenticated controller");
      }
      endpoint_lines_->writeLine(line);
      return;
    }
    if (line.size() > lingtu::sim::driver_bridge::kMaxProtocolLineBytes ||
        std::fwrite(line.data(), 1, line.size(), stdout) != line.size() ||
        std::fputc('\n', stdout) == EOF || std::fflush(stdout) != 0) {
      throw std::runtime_error("failed to write controller protocol stdout");
    }
  }

  [[nodiscard]] bool local() const noexcept { return local_; }

 private:
  bool local_{false};
  bool prepared_{false};
  bool endpoint_started_{false};
  std::unique_ptr<StdinLines> stdin_lines_;
  std::unique_ptr<ReadyFile> ready_file_;
  std::unique_ptr<local_endpoint::LocalEndpointServer> endpoint_server_;
  std::unique_ptr<EndpointLines> endpoint_lines_;
};

const char *lifecycleName(BridgeLifecycle lifecycle) noexcept {
  switch (lifecycle) {
    case BridgeLifecycle::AwaitWriter:
      return "await_writer";
    case BridgeLifecycle::AwaitController:
      return "await_controller";
    case BridgeLifecycle::ActivatingZero:
      return "activating_zero";
    case BridgeLifecycle::Ready:
      return "ready";
    case BridgeLifecycle::DeactivatingZero:
      return "deactivating_zero";
    case BridgeLifecycle::Stopped:
      return "stopped";
    case BridgeLifecycle::FaultClosed:
      return "fault_closed";
  }
  return "unknown";
}

class DdsEndpoint final {
 public:
  explicit DdsEndpoint(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant");
    try {
      const dds_entity_t subscriber =
          checked(dds_create_subscriber(participant_, nullptr, nullptr), "dds_create_subscriber");
      const dds_entity_t publisher =
          checked(dds_create_publisher(participant_, nullptr, nullptr), "dds_create_publisher");

      const auto &command_contract = lingtu::message::kNavCmdVel;
      const dds_entity_t command_topic =
          checked(dds_create_topic(participant_, &lingtu_dds_FinalVelocityCommand_desc,
                                   command_contract.dds_topic.data(), nullptr, nullptr),
                  "dds_create_topic(cmd_vel)");
      auto command_qos =
          lingtu::dds::make_qos(lingtu::dds::qos_for_topic(command_contract.dds_topic));
      command_reader_ =
          checked(dds_create_reader(subscriber, command_topic, command_qos.get(), nullptr),
                  "dds_create_reader(cmd_vel)");

      const auto &state_contract = lingtu::message::kDriverControlState;
      const dds_entity_t state_topic =
          checked(dds_create_topic(participant_, &lingtu_dds_DriverControlState_desc,
                                   state_contract.dds_topic.data(), nullptr, nullptr),
                  "dds_create_topic(driver_control_state)");
      auto state_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(state_contract.dds_topic));
      state_writer_ = checked(dds_create_writer(publisher, state_topic, state_qos.get(), nullptr),
                              "dds_create_writer(driver_control_state)");
    } catch (...) {
      dds_delete(participant_);
      participant_ = 0;
      throw;
    }
  }

  DdsEndpoint(const DdsEndpoint &) = delete;
  DdsEndpoint &operator=(const DdsEndpoint &) = delete;

  ~DdsEndpoint() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  std::uint32_t matchedCommandWriters() const {
    const dds_return_t count = dds_get_matched_publications(command_reader_, nullptr, 0);
    if (count < 0) {
      throw std::runtime_error(std::string("dds_get_matched_publications(cmd_vel): ") +
                               dds_strretcode(-count));
    }
    return static_cast<std::uint32_t>(count);
  }

  std::optional<NavCommand> takeLatest() {
    void *samples[32]{};
    dds_sample_info_t infos[32]{};
    const dds_return_t count = dds_take(command_reader_, samples, infos, 32, 32);
    if (count < 0) {
      throw std::runtime_error(std::string("dds_take(cmd_vel): ") + dds_strretcode(-count));
    }
    const auto arrival_time = bridgeNow();
    std::optional<NavCommand> latest;
    for (dds_return_t index = 0; index < count; ++index) {
      if (!infos[index].valid_data || samples[index] == nullptr) {
        continue;
      }
      const auto &message = *static_cast<const lingtu_dds_FinalVelocityCommand *>(samples[index]);
      TimePoint source_time{};
      if (message.source_boottime_ns <=
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
        source_time = TimePoint{std::chrono::duration_cast<Clock::duration>(
            std::chrono::nanoseconds(static_cast<std::int64_t>(message.source_boottime_ns)))};
      }
      latest = NavCommand{
          message.host_boot_id == nullptr ? std::string{} : std::string(message.host_boot_id),
          message.producer_boot_id == nullptr ? std::string{}
                                              : std::string(message.producer_boot_id),
          message.output_seq,
          source_time,
          arrival_time,
          "body",
          message.twist.linear.x,
          message.twist.linear.y,
          message.twist.angular.z,
      };
    }
    if (count > 0) {
      checked(dds_return_loan(command_reader_, samples, count), "dds_return_loan(cmd_vel)");
    }
    return latest;
  }

  void publishControlState(const BridgeStatus &status) {
    lingtu_dds_DriverControlState message{};
    const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
    if (wall_ns <= 0) {
      throw std::runtime_error("system wall clock is not positive");
    }
    constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
    const auto seconds = wall_ns / kNanosecondsPerSecond;
    if (seconds > std::numeric_limits<std::int32_t>::max()) {
      throw std::runtime_error("DDS Header seconds exceed int32 range");
    }
    message.header.stamp.sec = static_cast<std::int32_t>(seconds);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(wall_ns % kNanosecondsPerSecond);
    message.header.frame_id = const_cast<char *>("base_link");

    const bool terminal = status.lifecycle == BridgeLifecycle::Stopped ||
                          status.lifecycle == BridgeLifecycle::FaultClosed;
    const bool connected = !status.controller_boot_id.empty() && !terminal;
    const auto output_ack = status.output_ack;
    const bool walking = status.applied_walk.x != 0.0 || status.applied_walk.y != 0.0 ||
                         status.applied_walk.z != 0.0;
    const std::string fsm =
        status.ready ? (walking ? "walking" : "standing") : lifecycleName(status.lifecycle);
    const std::string owner = status.lease_valid ? "grpc" : "none";
    const std::string owner_id = status.lease_valid ? "lingtu-driver@robot" : "";
    const std::string reason =
        status.fault != BridgeFaultCode::None
            ? lingtu::sim::driver_bridge::faultCodeName(status.fault)
            : (status.stop_cause != BridgeStopCause::None
                   ? lingtu::sim::driver_bridge::bridgeStopCauseName(status.stop_cause)
                   : (status.ready ? "" : lifecycleName(status.lifecycle)));

    message.connected = connected;
    message.ready = status.ready;
    message.motors_enabled = status.ready;
    message.critical_fault = status.fault != BridgeFaultCode::None;
    message.control_assured = status.lease_valid;
    message.lease_valid = status.lease_valid;
    message.lease_remaining_ms = status.lease_remaining_ms;
    message.accepted_sequence = status.accepted_sequence;
    message.accepted_producer_boot_id = const_cast<char *>(output_ack.producerBootId().c_str());
    message.accepted_output_sequence = output_ack.outputSequence();
    message.last_command_accepted = output_ack.accepted();
    message.fsm = const_cast<char *>(fsm.c_str());
    message.owner = const_cast<char *>(owner.c_str());
    message.owner_id = const_cast<char *>(owner_id.c_str());
    message.reason = const_cast<char *>(reason.c_str());
    checked(dds_write(state_writer_, &message), "dds_write(driver_control_state)");
  }

 private:
  dds_entity_t participant_{0};
  dds_entity_t command_reader_{0};
  dds_entity_t state_writer_{0};
};

void emitCommand(ControllerTransport &transport, const std::optional<BridgeCommand> &command) {
  if (command.has_value()) {
    transport.writeLine(lingtu::sim::driver_bridge::serializeCommand(*command));
  }
}

std::optional<BridgeCommand> dispatch(MujocoDriverBridgeCore &core,
                                      const ControllerMessage &message, TimePoint now) {
  return std::visit(
      [&](const auto &typed) -> std::optional<BridgeCommand> {
        using Message = std::decay_t<decltype(typed)>;
        if constexpr (std::is_same_v<Message, ActivateMessage>) {
          return core.onActivate(typed, now);
        } else if constexpr (std::is_same_v<Message, AppliedMessage>) {
          return core.onApplied(typed, now);
        } else if constexpr (std::is_same_v<Message, HeartbeatMessage>) {
          return core.onHeartbeat(typed, now);
        } else if constexpr (std::is_same_v<Message, DeactivateMessage>) {
          return core.onDeactivate(typed, now);
        }
      },
      message);
}

int run(const Config &config) {
  DdsEndpoint dds(config.domain_id);
  ControllerTransport controller(config);
  controller.startTransport();
  MujocoDriverBridgeCore core(BridgeConfig{
      config.bridge_boot_id,
      config.expected_product_session_id,
      config.limits,
      config.heartbeat_timeout,
      config.apply_timeout,
  });

  if (!controller.local()) {
    std::printf("LT_PID_V1\t%ld\n", processId());
    std::fflush(stdout);
  }
  std::fprintf(stderr,
               "lingtu_mujoco_driver_bridge: domain=%d cmd_topic=%s state_topic=%s "
               "command_timeout=%lldms heartbeat_timeout=%lldms apply_timeout=%lldms\n",
               config.domain_id, lingtu::message::kNavCmdVel.dds_topic.data(),
               lingtu::message::kDriverControlState.dds_topic.data(),
               static_cast<long long>(config.limits.command_timeout.count()),
               static_cast<long long>(config.heartbeat_timeout.count()),
               static_cast<long long>(config.apply_timeout.count()));

  auto next_control_state = bridgeNow();
  std::uint64_t last_ready_sequence = 0;
  bool fault_emitted = false;
  bool signal_stop_started = false;
  std::uint64_t signal_zero_sequence = 0;
  const auto emit_runtime_command = [&](const std::optional<BridgeCommand> &command) {
    if (signal_stop_started && command.has_value() &&
        command->kind == BridgeCommandKind::SafetyZero) {
      signal_zero_sequence = command->bridge_command_seq;
    }
    emitCommand(controller, command);
  };

  while (true) {
    const auto now = bridgeNow();
    bool publish_now = false;

    const auto writer_action = core.onWriterCount(dds.matchedCommandWriters(), now);
    emit_runtime_command(writer_action);
    publish_now = publish_now || writer_action.has_value();

    const auto before_input = core.status(now);
    if (before_input.lifecycle == BridgeLifecycle::AwaitController) {
      (void)controller.prepare(config.bridge_boot_id);
    }

    try {
      const InputBatch input = controller.readAvailable();
      for (const auto &line : input.lines) {
        const auto action =
            dispatch(core, lingtu::sim::driver_bridge::parseControllerLine(line), bridgeNow());
        emit_runtime_command(action);
        publish_now = true;
      }
      if (input.eof) {
        core.controllerEof();
        publish_now = true;
      }
    } catch (const ProtocolError &) {
      core.protocolFault();
      publish_now = true;
    } catch (const local_endpoint::EndpointError &) {
      core.controllerEof();
      publish_now = true;
    }

    if (const auto nav = dds.takeLatest(); nav.has_value()) {
      const auto action = core.submitNav(*nav);
      emit_runtime_command(action);
      publish_now = true;
    }

    const auto poll_action = core.poll(bridgeNow());
    emit_runtime_command(poll_action);
    publish_now = publish_now || poll_action.has_value();

    if (g_stop_requested != 0 && !signal_stop_started) {
      signal_stop_started = true;
      const auto stop_action = core.requestSafetyStop(bridgeNow());
      emit_runtime_command(stop_action);
      publish_now = true;
    }

    const auto status_now = bridgeNow();
    const auto status = core.status(status_now);
    if (status.ready && status.accepted_sequence != last_ready_sequence) {
      controller.writeLine(lingtu::sim::driver_bridge::serializeReady(ReadyMessage{
          config.bridge_boot_id,
          status.controller_boot_id,
          status.accepted_sequence,
          status.output_ack.producerBootId(),
          status.output_ack.outputSequence(),
      }));
      last_ready_sequence = status.accepted_sequence;
      publish_now = true;
    }

    if (publish_now || status_now >= next_control_state) {
      dds.publishControlState(status);
      next_control_state = status_now + kControlStatePeriod;
    }

    if (status.lifecycle == BridgeLifecycle::FaultClosed) {
      if (!fault_emitted) {
        controller.writeLine(lingtu::sim::driver_bridge::serializeFault(FaultMessage{
            config.bridge_boot_id,
            status.controller_boot_id,
            status.fault,
        }));
        fault_emitted = true;
      }
      return 1;
    }
    if (status.lifecycle == BridgeLifecycle::Stopped) {
      const auto &stopped = core.stoppedEvidence();
      if (!stopped.has_value()) {
        throw std::runtime_error("Stopped lifecycle is missing physical deactivate evidence");
      }
      controller.writeLine(lingtu::sim::driver_bridge::serializeStopped(*stopped));
      return 0;
    }
    if (signal_stop_started &&
        (status.controller_boot_id.empty() ||
         (signal_zero_sequence != 0 && status.accepted_sequence == signal_zero_sequence))) {
      return 0;
    }

    std::this_thread::sleep_for(config.poll_period);
  }
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Config config = parseArgs(argc, argv);
    if (config.help) {
      std::fputs(usage().data(), stdout);
      return 0;
    }
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
#ifdef _WIN32
    std::signal(SIGBREAK, stopSignal);
#endif
    return run(config);
  } catch (const std::exception &exception) {
    std::fprintf(stderr, "lingtu_mujoco_driver_bridge: fatal: %s\n", exception.what());
    return 1;
  }
}
