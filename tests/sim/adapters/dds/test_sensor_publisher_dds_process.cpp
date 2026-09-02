#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "drivers/real/camera/native/camera_record.hpp"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "native/module.hpp"
#include "test_dds_domain.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

namespace lidar = lingtu::drivers::lidar;
namespace camera_record = lingtu::drivers::camera::record;

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

int test_domain_id() {
  return lingtu::sim::dds_adapter::test::domain_id_from_environment();
}

bool close_enough(double left, double right) {
  return std::abs(left - right) < 1e-6;
}

void append_u32(std::string &out, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<char>((value >> (8U * i)) & 0xFFU));
  }
}

void append_u64(std::string &out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<char>((value >> (8U * i)) & 0xFFU));
  }
}

template <typename T>
void append_pod(std::string &out, const T &value) {
  const auto *bytes = reinterpret_cast<const char *>(&value);
  out.append(bytes, bytes + sizeof(T));
}

void append_record(std::string &out, std::uint8_t type, std::uint64_t timestamp_ns,
                   std::uint32_t sequence, std::uint32_t count, const std::string &payload) {
  out.append("LTU1", 4);
  out.push_back(static_cast<char>(type));
  out.append(3, '\0');
  append_u64(out, timestamp_ns);
  append_u32(out, sequence);
  append_u32(out, count);
  append_u32(out, static_cast<std::uint32_t>(payload.size()));
  out += payload;
}

std::string make_sensor_fixture() {
  std::string records;
  constexpr std::uint64_t kBaseNs = 2000000000ULL;
  auto intrinsics = camera_record::makeRecordHeader(camera_record::kKindIntrinsics);
  intrinsics.width = 2;
  intrinsics.height = 2;
  intrinsics.timestamp_s = 2.0;
  intrinsics.fx = 100.0;
  intrinsics.fy = 101.0;
  intrinsics.cx = 1.0;
  intrinsics.cy = 1.0;
  std::string intrinsics_payload;
  append_pod(intrinsics_payload, intrinsics);
  append_record(records, 5, kBaseNs, 1000, 1, intrinsics_payload);

  auto color = camera_record::makeRecordHeader(camera_record::kKindColor);
  color.width = 2;
  color.height = 2;
  color.channels = 3;
  color.format = camera_record::kFormatRgb8;
  color.timestamp_s = 2.1;
  color.payload_size = 12;
  std::string color_payload;
  append_pod(color_payload, color);
  color_payload.append("\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c", 12);
  append_record(records, 5, kBaseNs + 100000000ULL, 1001, 1, color_payload);

  auto depth = camera_record::makeRecordHeader(camera_record::kKindDepth);
  depth.width = 2;
  depth.height = 2;
  depth.channels = 1;
  depth.format = camera_record::kFormatDepthU16;
  depth.timestamp_s = 2.2;
  depth.payload_size = 8;
  std::string depth_payload;
  append_pod(depth_payload, depth);
  const std::array<std::uint16_t, 4> depth_values{{100, 200, 300, 400}};
  depth_payload.append(reinterpret_cast<const char *>(depth_values.data()),
                       reinterpret_cast<const char *>(depth_values.data() + depth_values.size()));
  append_record(records, 5, kBaseNs + 200000000ULL, 1002, 1, depth_payload);

  for (std::uint32_t i = 0; i < 30; ++i) {
    lidar::ImuSample imu{};
    imu.gyro_x = 0.25F;
    imu.gyro_y = -0.5F;
    imu.gyro_z = 0.75F;
    imu.acc_x = 0.15F;
    imu.acc_y = 0.25F;
    imu.acc_z = 1.0F;
    std::string imu_payload;
    append_pod(imu_payload, imu);
    append_record(records, 2, kBaseNs + i * 50000000ULL, i * 2U, 1, imu_payload);

    lidar::Point point{};
    point.x = 1.0F + static_cast<float>(i);
    point.y = 2.0F;
    point.z = 3.0F;
    point.intensity = 42.0F;
    point.offset_time_ns = 1000;
    point.tag = 7;
    point.line = 8;
    std::string point_payload;
    append_pod(point_payload, point);
    append_record(records, 1, kBaseNs + i * 50000000ULL + 1000000ULL, i * 2U + 1U, 1,
                  point_payload);
  }
  return records;
}

std::string make_color_without_intrinsics_fixture(std::uint32_t count = 1) {
  auto color = camera_record::makeRecordHeader(camera_record::kKindColor);
  color.width = 1;
  color.height = 1;
  color.channels = 3;
  color.format = camera_record::kFormatRgb8;
  color.timestamp_s = 2.0;
  color.payload_size = 3;
  std::string payload;
  append_pod(payload, color);
  payload.append("\x01\x02\x03", 3);
  std::string record;
  append_record(record, 5, 2000000000ULL, 1, count, payload);
  return record;
}

bool take_matching_camera_image(dds_entity_t reader, const char *encoding, std::uint32_t step,
                                std::uint32_t expected_nanosec,
                                const std::vector<std::uint8_t> &expected) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take(camera_image): ") + dds_strretcode(-count));
  }
  if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
    const auto *msg = static_cast<const lingtu_dds_Image *>(samples[0]);
    const bool match =
        msg->header.frame_id != nullptr && std::string(msg->header.frame_id) == "camera_test" &&
        msg->header.stamp.sec == 2 && msg->header.stamp.nanosec == expected_nanosec &&
        msg->height == 2 && msg->width == 2 && msg->encoding != nullptr &&
        std::string(msg->encoding) == encoding && !msg->is_bigendian && msg->step == step &&
        msg->data._length == expected.size() && msg->data._buffer != nullptr &&
        std::equal(expected.begin(), expected.end(), msg->data._buffer);
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(camera_image)");
    return match;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(camera_image)");
  }
  return false;
}

bool take_matching_camera_info(dds_entity_t reader) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take(camera_info): ") + dds_strretcode(-count));
  }
  if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
    const auto *msg = static_cast<const lingtu_dds_CameraInfo *>(samples[0]);
    const bool match =
        msg->header.frame_id != nullptr && std::string(msg->header.frame_id) == "camera_test" &&
        msg->height == 2 && msg->width == 2 && msg->header.stamp.sec == 2 &&
        msg->header.stamp.nanosec == 0 && close_enough(msg->depth_scale, 0.001) &&
        msg->distortion_model != nullptr && std::string(msg->distortion_model) == "plumb_bob" &&
        msg->d._length == 5 && msg->d._buffer != nullptr && close_enough(msg->d._buffer[0], 0.0) &&
        close_enough(msg->k[0], 100.0) && close_enough(msg->k[4], 101.0) &&
        close_enough(msg->k[2], 1.0) && close_enough(msg->k[5], 1.0) &&
        close_enough(msg->r[0], 1.0) && close_enough(msg->r[4], 1.0) &&
        close_enough(msg->r[8], 1.0) && close_enough(msg->p[0], 100.0) &&
        close_enough(msg->p[5], 101.0) && close_enough(msg->p[10], 1.0);
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(camera_info)");
    return match;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(camera_info)");
  }
  return false;
}

class PublisherProcess {
 public:
  PublisherProcess(const std::filesystem::path &publisher, const std::filesystem::path &ready,
                   int domain_id,
                   std::optional<std::pair<std::string, std::string>> extra_option = std::nullopt) {
    start(publisher, ready, domain_id, std::move(extra_option));
  }

  PublisherProcess(const PublisherProcess &) = delete;
  PublisherProcess &operator=(const PublisherProcess &) = delete;

  ~PublisherProcess() {
    if (!wait(std::chrono::milliseconds(1))) {
      terminate();
    }
    close_stdin();
  }

  bool wait(std::chrono::milliseconds timeout) {
#ifdef _WIN32
    if (process_ == nullptr) {
      return true;
    }
    const DWORD rc = WaitForSingleObject(process_, static_cast<DWORD>(timeout.count()));
    if (rc == WAIT_OBJECT_0) {
      DWORD code = 0;
      if (!GetExitCodeProcess(process_, &code)) {
        throw std::runtime_error("failed to read publisher process exit code");
      }
      exit_code_ = static_cast<int>(code);
      CloseHandle(process_);
      process_ = nullptr;
      if (thread_ != nullptr) {
        CloseHandle(thread_);
        thread_ = nullptr;
      }
      return true;
    }
    return false;
#else
    if (pid_ <= 0) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      int status = 0;
      const pid_t rc = waitpid(pid_, &status, WNOHANG);
      if (rc == pid_) {
        exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status)
                                       : 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
        pid_ = -1;
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
#endif
  }

  std::optional<int> exit_code() const noexcept { return exit_code_; }

  void write_records(const std::string &records) {
#ifdef _WIN32
    if (stdin_write_ == nullptr) {
      throw std::runtime_error("publisher stdin is closed");
    }
    std::size_t offset = 0;
    while (offset < records.size()) {
      const DWORD chunk =
          static_cast<DWORD>(std::min<std::size_t>(records.size() - offset, 1024U * 1024U));
      DWORD written = 0;
      if (!WriteFile(stdin_write_, records.data() + offset, chunk, &written, nullptr)) {
        throw std::runtime_error("failed to write publisher stdin");
      }
      offset += written;
    }
#else
    if (stdin_write_ < 0) {
      throw std::runtime_error("publisher stdin is closed");
    }
    std::size_t offset = 0;
    while (offset < records.size()) {
      const ssize_t written = write(stdin_write_, records.data() + offset, records.size() - offset);
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written <= 0) {
        throw std::runtime_error("failed to write publisher stdin");
      }
      offset += static_cast<std::size_t>(written);
    }
#endif
    close_stdin();
  }

  void terminate() {
    close_stdin();
#ifdef _WIN32
    if (process_ != nullptr) {
      TerminateProcess(process_, 1);
      WaitForSingleObject(process_, 2000);
      CloseHandle(process_);
      process_ = nullptr;
    }
    if (thread_ != nullptr) {
      CloseHandle(thread_);
      thread_ = nullptr;
    }
#else
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      if (!wait(std::chrono::milliseconds(500))) {
        kill(pid_, SIGKILL);
        wait(std::chrono::milliseconds(2000));
      }
    }
#endif
  }

 private:
  void close_stdin() noexcept {
#ifdef _WIN32
    if (stdin_write_ != nullptr) {
      CloseHandle(stdin_write_);
      stdin_write_ = nullptr;
    }
#else
    if (stdin_write_ >= 0) {
      close(stdin_write_);
      stdin_write_ = -1;
    }
#endif
  }

#ifdef _WIN32
  static std::wstring quote(const std::filesystem::path &value) {
    return L"\"" + value.wstring() + L"\"";
  }

  static std::wstring quote_arg(const std::string &value) {
    return L"\"" + std::wstring(value.begin(), value.end()) + L"\"";
  }
#endif

  void start(const std::filesystem::path &publisher, const std::filesystem::path &ready,
             int domain_id, std::optional<std::pair<std::string, std::string>> extra_option) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES security{};
    security.nLength = sizeof(security);
    security.bInheritHandle = TRUE;
    HANDLE stdin_read = nullptr;
    if (!CreatePipe(&stdin_read, &stdin_write_, &security, 0)) {
      throw std::runtime_error("failed to create publisher stdin pipe");
    }
    if (!SetHandleInformation(stdin_write_, HANDLE_FLAG_INHERIT, 0)) {
      CloseHandle(stdin_read);
      close_stdin();
      throw std::runtime_error("failed to configure publisher stdin pipe");
    }

    std::wstring command = quote(publisher) + L" --stdin-records --dds --domain-id " +
                           std::to_wstring(domain_id) + L" --lidar-frame " +
                           quote_arg("lidar_test") + L" --imu-frame " + quote_arg("imu_test") +
                           L" --camera-frame " + quote_arg("camera_test") +
                           L" --scan-window 0 --ready-file " + quote(ready);
    if (extra_option.has_value()) {
      command += L" " + quote_arg(extra_option->first) + L" " + quote_arg(extra_option->second);
    }
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = stdin_read;
    si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    PROCESS_INFORMATION pi{};
    std::vector<wchar_t> mutable_command(command.begin(), command.end());
    mutable_command.push_back(L'\0');
    const BOOL ok = CreateProcessW(nullptr, mutable_command.data(), nullptr, nullptr, TRUE, 0,
                                   nullptr, nullptr, &si, &pi);
    CloseHandle(stdin_read);
    if (!ok) {
      close_stdin();
      throw std::runtime_error("failed to launch publisher process");
    }
    process_ = pi.hProcess;
    thread_ = pi.hThread;
#else
    int stdin_pipe[2]{};
    if (pipe(stdin_pipe) != 0) {
      throw std::runtime_error("failed to create publisher stdin pipe");
    }
    pid_ = fork();
    if (pid_ < 0) {
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
      throw std::runtime_error("failed to fork publisher process");
    }
    if (pid_ == 0) {
      close(stdin_pipe[1]);
      dup2(stdin_pipe[0], STDIN_FILENO);
      close(stdin_pipe[0]);
      const std::string domain = std::to_string(domain_id);
      if (extra_option.has_value()) {
        execl(publisher.c_str(), publisher.c_str(), "--stdin-records", "--dds", "--domain-id",
              domain.c_str(), "--lidar-frame", "lidar_test", "--imu-frame", "imu_test",
              "--camera-frame", "camera_test", "--scan-window", "0", "--ready-file", ready.c_str(),
              extra_option->first.c_str(), extra_option->second.c_str(),
              static_cast<char *>(nullptr));
      } else {
        execl(publisher.c_str(), publisher.c_str(), "--stdin-records", "--dds", "--domain-id",
              domain.c_str(), "--lidar-frame", "lidar_test", "--imu-frame", "imu_test",
              "--camera-frame", "camera_test", "--scan-window", "0", "--ready-file", ready.c_str(),
              static_cast<char *>(nullptr));
      }
      _exit(127);
    }
    close(stdin_pipe[0]);
    stdin_write_ = stdin_pipe[1];
#endif
  }

#ifdef _WIN32
  HANDLE process_{nullptr};
  HANDLE thread_{nullptr};
  HANDLE stdin_write_{nullptr};
#else
  pid_t pid_{-1};
  int stdin_write_{-1};
#endif
  std::optional<int> exit_code_;
};

void require_cli_rejected(const std::filesystem::path &publisher, int domain_id,
                          const std::string &option, const std::string &value) {
  const auto ready =
      std::filesystem::temp_directory_path() /
      ("lingtu_sensor_publisher_invalid_ready_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  PublisherProcess process(publisher, ready, domain_id, std::pair{option, value});
  const bool exited = process.wait(std::chrono::seconds(4));
  std::filesystem::remove(ready);
  if (!exited) {
    throw std::runtime_error(option + " " + value + " did not exit");
  }
  if (process.exit_code() != std::optional<int>{2}) {
    throw std::runtime_error(option + " " + value + " was not rejected");
  }
}

void wait_for_ready_marker(PublisherProcess &process, const std::filesystem::path &ready) {
  constexpr const char *kReadySchema = "lingtu.mujoco_sensor_publisher.ready.v1";
  const std::regex marker_pattern(
      R"json(^\s*\{\s*"ready"\s*:\s*(true|false)\s*,\s*"schema"\s*:\s*"([^"]+)"\s*\}\s*$)json");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::filesystem::exists(ready)) {
      if (std::filesystem::file_size(ready) > 4096U) {
        throw std::runtime_error("publisher readiness JSON is too large");
      }
      std::ifstream input(ready, std::ios::binary);
      const std::string payload{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
      std::smatch fields;
      if (!input.is_open() || !std::regex_match(payload, fields, marker_pattern)) {
        throw std::runtime_error("publisher readiness marker is not valid JSON");
      }
      if (fields[1].str() != "true") {
        throw std::runtime_error("publisher readiness JSON did not assert ready=true");
      }
      if (fields[2].str() != kReadySchema) {
        throw std::runtime_error("publisher readiness JSON schema mismatch");
      }
      return;
    }
    if (process.wait(std::chrono::milliseconds(1))) {
      throw std::runtime_error("publisher exited before writing readiness JSON");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("publisher readiness JSON timed out");
}

void require_records_rejected(const std::filesystem::path &publisher, int domain_id,
                              const std::string &records) {
  const auto ready =
      std::filesystem::temp_directory_path() /
      ("lingtu_sensor_publisher_reject_ready_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  std::filesystem::remove(ready);
  PublisherProcess process(publisher, ready, domain_id);
  wait_for_ready_marker(process, ready);
  process.write_records(records);
  const bool exited = process.wait(std::chrono::seconds(4));
  std::filesystem::remove(ready);
  if (!exited || process.exit_code() != std::optional<int>{2}) {
    throw std::runtime_error("malformed camera records were not rejected");
  }
}

bool reader_has_match(dds_entity_t reader, const char *label) {
  const dds_return_t count = dds_get_matched_publications(reader, nullptr, 0);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_get_matched_publications(") + label +
                             "): " + dds_strretcode(-count));
  }
  return count > 0;
}

int reader_match_count(dds_entity_t reader, const char *label) {
  const dds_return_t count = dds_get_matched_publications(reader, nullptr, 0);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_get_matched_publications(") + label +
                             "): " + dds_strretcode(-count));
  }
  return static_cast<int>(count);
}

void wait_for_camera_reader_matches(PublisherProcess &process, dds_entity_t color_reader,
                                    dds_entity_t depth_reader, dds_entity_t info_reader) {
  bool color_matched = false;
  bool depth_matched = false;
  bool info_matched = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    color_matched = color_matched || reader_has_match(color_reader, "camera_color");
    depth_matched = depth_matched || reader_has_match(depth_reader, "camera_depth");
    info_matched = info_matched || reader_has_match(info_reader, "camera_info");
    if (color_matched && depth_matched && info_matched) {
      return;
    }
    if (process.wait(std::chrono::milliseconds(1))) {
      throw std::runtime_error("publisher exited before camera DDS discovery completed");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("publisher camera DDS discovery timed out");
}

void require_no_camera_reader_matches(dds_entity_t color_reader, dds_entity_t depth_reader,
                                      dds_entity_t info_reader) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (std::chrono::steady_clock::now() < deadline) {
    if (reader_has_match(color_reader, "camera_color_before_record") ||
        reader_has_match(depth_reader, "camera_depth_before_record") ||
        reader_has_match(info_reader, "camera_info_before_record")) {
      throw std::runtime_error("camera DDS writer exists before the first camera record");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void wait_for_reader_matches(PublisherProcess &process, dds_entity_t imu_reader,
                             dds_entity_t lidar_packet_reader, dds_entity_t lidar_frame_reader) {
  bool imu_matched = false;
  bool lidar_packet_matched = false;
  bool lidar_frame_matched = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    imu_matched = imu_matched || reader_has_match(imu_reader, "imu");
    lidar_packet_matched =
        lidar_packet_matched || reader_has_match(lidar_packet_reader, "lidar_packet");
    lidar_frame_matched =
        lidar_frame_matched || reader_has_match(lidar_frame_reader, "lidar_frame");
    if (imu_matched && lidar_packet_matched && lidar_frame_matched) {
      return;
    }
    if (process.wait(std::chrono::milliseconds(1))) {
      throw std::runtime_error("publisher exited before DDS discovery completed");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error(std::string("publisher DDS discovery timed out: imu=") +
                           (imu_matched ? "yes" : "no") +
                           " lidar_packet=" + (lidar_packet_matched ? "yes" : "no") +
                           " lidar_frame=" + (lidar_frame_matched ? "yes" : "no"));
}

void wait_for_endpoint_reader_matches(dds_entity_t lidar_packet_reader,
                                       dds_entity_t lidar_frame_reader) {
  bool lidar_frame_matched = false;
  std::optional<std::chrono::steady_clock::time_point> stable_since;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    const int lidar_packet_matches =
        reader_match_count(lidar_packet_reader, "endpoint_lidar_packet");
    lidar_frame_matched =
        lidar_frame_matched || reader_has_match(lidar_frame_reader, "endpoint_lidar_frame");
    if (lidar_packet_matches != 0) {
      throw std::runtime_error(
          "formal lidar endpoint created a diagnostic raw_packet writer");
    }
    if (lidar_frame_matched) {
      if (!stable_since.has_value()) {
        stable_since = std::chrono::steady_clock::now();
      } else if (std::chrono::steady_clock::now() - *stable_since >=
                 std::chrono::seconds(1)) {
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error(std::string("endpoint DDS discovery timed out: lidar_packet=") +
                           std::to_string(reader_match_count(
                               lidar_packet_reader, "endpoint_lidar_packet")) +
                           " lidar_frame=" + (lidar_frame_matched ? "yes" : "no"));
}

bool take_matching_imu(dds_entity_t reader) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take(imu): ") + dds_strretcode(-count));
  }
  if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
    const auto *msg = static_cast<const lingtu_dds_Imu *>(samples[0]);
    const bool match = msg->header.frame_id != nullptr &&
                       std::string(msg->header.frame_id) == "imu_test" &&
                       close_enough(msg->angular_velocity.x, 0.25) &&
                       close_enough(msg->angular_velocity.y, -0.5) &&
                       close_enough(msg->angular_velocity.z, 0.75) &&
                       close_enough(msg->linear_acceleration.x, 0.15) &&
                       close_enough(msg->linear_acceleration.y, 0.25) &&
                       close_enough(msg->linear_acceleration.z, 1.0);
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(imu)");
    return match;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(imu)");
  }
  return false;
}

bool take_matching_lidar(dds_entity_t reader) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take(lidar): ") + dds_strretcode(-count));
  }
  if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
    const auto *msg = static_cast<const lingtu_dds_LivoxFrame *>(samples[0]);
    const bool match =
        msg->header.frame_id != nullptr && std::string(msg->header.frame_id) == "lidar_test" &&
        msg->point_num == 1 && msg->points._length == 1 && msg->points._buffer != nullptr &&
        close_enough(msg->points._buffer[0].y, 2.0) &&
        close_enough(msg->points._buffer[0].z, 3.0) && msg->points._buffer[0].reflectivity == 42 &&
        msg->points._buffer[0].tag == 7 && msg->points._buffer[0].line == 8;
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(lidar)");
    return match;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(lidar)");
  }
  return false;
}

bool take_matching_map_observation(dds_entity_t reader) {
  void *samples[1]{nullptr};
  dds_sample_info_t infos[1]{};
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  if (count < 0) {
    throw std::runtime_error(
        std::string("dds_take(map_observation): ") + dds_strretcode(-count));
  }
  if (count == 1 && infos[0].valid_data && samples[0] != nullptr) {
    const auto *msg = static_cast<const lingtu_dds_MapObservation *>(samples[0]);
    const bool match = msg->header.frame_id != nullptr &&
                       std::string(msg->header.frame_id) == "map" &&
                       msg->sensor_frame != nullptr &&
                       std::string(msg->sensor_frame) == "body" &&
                       msg->observation_sequence > 0 && msg->reset_epoch > 0 &&
                       close_enough(msg->map_sensor.translation.z, 0.4) &&
                       close_enough(msg->map_sensor.rotation.w, 1.0) &&
                       close_enough(msg->sensor_origin.z, 0.4) &&
                       close_enough(msg->pose_confidence, 1.0) &&
                       close_enough(msg->localization_quality, 1.0) &&
                       msg->pose_state != nullptr &&
                       std::string(msg->pose_state) == "TRACKING" &&
                       msg->pose_reason != nullptr &&
                       std::string(msg->pose_reason) == "mujoco_navigation_fixture" &&
                       msg->scan.header.frame_id != nullptr &&
                       std::string(msg->scan.header.frame_id) == "body" &&
                       msg->scan.width == 1 && msg->scan.data._length == 16;
    checked(dds_return_loan(reader, samples, count),
            "dds_return_loan(map_observation)");
    return match;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count),
            "dds_return_loan(map_observation)");
  }
  return false;
}

dds_entity_t create_reader(dds_entity_t participant, const lingtu::message::TopicContract &contract,
                           const dds_topic_descriptor_t *descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(reader)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(dds_create_reader(participant, topic, qos.get(), nullptr), "dds_create_reader");
}

void observe_endpoint_samples(const int domain_id, const std::filesystem::path &ready_path) {
  dds_entity_t participant = 0;
  try {
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(endpoint_reader)");
    const dds_entity_t lidar_packet_reader =
        create_reader(participant, lingtu::message::kLidarRawPacket, &lingtu_dds_LivoxFrame_desc);
    const dds_entity_t lidar_frame_reader =
        create_reader(participant, lingtu::message::kLidarRawFrame, &lingtu_dds_LivoxFrame_desc);
    const dds_entity_t map_observation_reader = create_reader(
        participant,
        lingtu::message::kSlamMapObservation,
        &lingtu_dds_MapObservation_desc);
    wait_for_endpoint_reader_matches(lidar_packet_reader, lidar_frame_reader);
    {
      std::ofstream ready(ready_path, std::ios::binary | std::ios::trunc);
      ready << "READY\n";
      if (!ready) {
        throw std::runtime_error("failed to publish endpoint observer readiness");
      }
    }

    bool saw_lidar_frame = false;
    bool saw_map_observation = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline &&
           (!saw_lidar_frame || !saw_map_observation)) {
      saw_lidar_frame = saw_lidar_frame || take_matching_lidar(lidar_frame_reader);
      saw_map_observation =
          saw_map_observation || take_matching_map_observation(map_observation_reader);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!saw_lidar_frame || !saw_map_observation) {
      throw std::runtime_error(std::string("endpoint DDS samples missing: lidar_frame=") +
                               (saw_lidar_frame ? "yes" : "no") +
                               " map_observation=" +
                               (saw_map_observation ? "yes" : "no"));
    }
    dds_delete(participant);
  } catch (...) {
    if (participant > 0) {
      dds_delete(participant);
    }
    throw;
  }
}

void observe_stream_writers(const int domain_id) {
  dds_entity_t participant = 0;
  try {
    participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(stream_writer_observer)");
    const std::array<dds_entity_t, 6> readers{{
        create_reader(participant, lingtu::message::kLidarRawPacket,
                      &lingtu_dds_LivoxFrame_desc),
        create_reader(participant, lingtu::message::kLidarRawFrame,
                      &lingtu_dds_LivoxFrame_desc),
        create_reader(participant, lingtu::message::kImuRaw, &lingtu_dds_Imu_desc),
        create_reader(participant, lingtu::message::kCameraColor, &lingtu_dds_Image_desc),
        create_reader(participant, lingtu::message::kCameraDepth, &lingtu_dds_Image_desc),
        create_reader(participant, lingtu::message::kCameraInfo, &lingtu_dds_CameraInfo_desc),
    }};
    const std::array<const char *, 6> labels{{"lidar_packet", "lidar_frame", "imu",
                                              "camera_color", "camera_depth", "camera_info"}};
    constexpr std::array<int, 6> expected{{0, 1, 1, 1, 1, 1}};
    std::array<int, 6> counts{};
    std::optional<std::chrono::steady_clock::time_point> stable_since;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline) {
      bool exact = true;
      for (std::size_t i = 0; i < readers.size(); ++i) {
        counts[i] = reader_match_count(readers[i], labels[i]);
        exact = exact && counts[i] == expected[i];
      }
      if (exact) {
        if (!stable_since.has_value()) {
          stable_since = std::chrono::steady_clock::now();
        } else if (std::chrono::steady_clock::now() - *stable_since >=
                   std::chrono::seconds(1)) {
          dds_delete(participant);
          return;
        }
      } else {
        stable_since.reset();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::string detail;
    for (std::size_t i = 0; i < counts.size(); ++i) {
      detail += std::string(i == 0 ? "" : " ") + labels[i] + "=" +
                std::to_string(counts[i]);
    }
    throw std::runtime_error("split sensor DDS writer counts are not exact: " + detail);
  } catch (...) {
    if (participant > 0) {
      dds_delete(participant);
    }
    throw;
  }
}

}  // namespace

int main(int argc, const char *const argv[]) {
  if (argc == 3 && std::string(argv[1]) == "--observe-stream-writers") {
    try {
      observe_stream_writers(std::stoi(argv[2]));
      std::printf("split sensor DDS writer observer passed\n");
      return 0;
    } catch (const std::exception &exc) {
      std::fprintf(stderr, "split sensor DDS writer observer failed: %s\n", exc.what());
      return 1;
    }
  }
  if (argc == 4 && std::string(argv[1]) == "--observe-endpoint") {
    try {
      observe_endpoint_samples(std::stoi(argv[2]), std::filesystem::path(argv[3]));
      std::printf("sensor publisher endpoint DDS observer passed\n");
      return 0;
    } catch (const std::exception &exc) {
      std::fprintf(stderr, "sensor publisher endpoint DDS observer failed: %s\n", exc.what());
      return 1;
    }
  }
  if (argc != 2) {
    std::fprintf(stderr, "usage: test_sensor_publisher_dds_process PUBLISHER_EXE\n");
    return 2;
  }

  dds_entity_t participant = 0;
  try {
    const int domain_id = test_domain_id();
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(reader)");
    const dds_entity_t imu_reader =
        create_reader(participant, lingtu::message::kImuRaw, &lingtu_dds_Imu_desc);
    const dds_entity_t lidar_packet_reader =
        create_reader(participant, lingtu::message::kLidarRawPacket, &lingtu_dds_LivoxFrame_desc);
    const dds_entity_t lidar_frame_reader =
        create_reader(participant, lingtu::message::kLidarRawFrame, &lingtu_dds_LivoxFrame_desc);
    const dds_entity_t color_reader =
        create_reader(participant, lingtu::message::kCameraColor, &lingtu_dds_Image_desc);
    const dds_entity_t depth_reader =
        create_reader(participant, lingtu::message::kCameraDepth, &lingtu_dds_Image_desc);
    const dds_entity_t info_reader =
        create_reader(participant, lingtu::message::kCameraInfo, &lingtu_dds_CameraInfo_desc);

    const std::string records = make_sensor_fixture();
    for (const char *value : {"-0.1", "nan", "inf", "-inf"}) {
      require_cli_rejected(argv[1], domain_id, "--scan-window", value);
      require_cli_rejected(argv[1], domain_id, "--imu-publish-freq", value);
    }
    for (const char *value : {"inf", "nan", "-0.1", "0"}) {
      require_cli_rejected(argv[1], domain_id, "--publish-freq", value);
    }
    require_cli_rejected(argv[1], domain_id, "--local-endpoint-ready-file", "legacy.json");
    require_cli_rejected(argv[1], domain_id, "--local-endpoint-fingerprint", std::string(64, 'a'));
    require_records_rejected(argv[1], domain_id, make_color_without_intrinsics_fixture());
    require_records_rejected(argv[1], domain_id, make_color_without_intrinsics_fixture(2));
    const auto ready =
        std::filesystem::temp_directory_path() /
        ("lingtu_sensor_publisher_ready_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
    std::filesystem::remove(ready);
    PublisherProcess publisher(argv[1], ready, domain_id);
    wait_for_ready_marker(publisher, ready);
    wait_for_reader_matches(publisher, imu_reader, lidar_packet_reader, lidar_frame_reader);
    require_no_camera_reader_matches(color_reader, depth_reader, info_reader);
    publisher.write_records(records);
    wait_for_camera_reader_matches(publisher, color_reader, depth_reader, info_reader);

    bool saw_imu = false;
    bool saw_lidar_packet = false;
    bool saw_lidar_frame = false;
    bool saw_color = false;
    bool saw_depth = false;
    bool saw_info = false;
    const std::vector<std::uint8_t> expected_color{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    const std::vector<std::uint8_t> expected_depth{100, 0, 200, 0, 44, 1, 144, 1};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline &&
           (!saw_imu || !saw_lidar_packet || !saw_lidar_frame || !saw_color || !saw_depth ||
            !saw_info)) {
      saw_imu = saw_imu || take_matching_imu(imu_reader);
      saw_lidar_packet = saw_lidar_packet || take_matching_lidar(lidar_packet_reader);
      saw_lidar_frame = saw_lidar_frame || take_matching_lidar(lidar_frame_reader);
      saw_color = saw_color ||
                  take_matching_camera_image(color_reader, "rgb8", 6, 100000000U, expected_color);
      saw_depth = saw_depth ||
                  take_matching_camera_image(depth_reader, "16UC1", 4, 200000000U, expected_depth);
      saw_info = saw_info || take_matching_camera_info(info_reader);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const bool exited = publisher.wait(std::chrono::seconds(4));
    std::filesystem::remove(ready);
    if (!exited) {
      throw std::runtime_error("publisher did not exit after fixture EOF");
    }
    if (publisher.exit_code() != std::optional<int>{0}) {
      throw std::runtime_error("publisher exited with code " +
                               (publisher.exit_code().has_value()
                                    ? std::to_string(*publisher.exit_code())
                                    : std::string("unknown")));
    }
    if (!saw_imu || !saw_lidar_packet || !saw_lidar_frame || !saw_color || !saw_depth ||
        !saw_info) {
      throw std::runtime_error(std::string("publisher DDS samples missing: imu=") +
                               (saw_imu ? "yes" : "no") +
                               " lidar_packet=" + (saw_lidar_packet ? "yes" : "no") +
                               " lidar_frame=" + (saw_lidar_frame ? "yes" : "no") +
                               " camera_color=" + (saw_color ? "yes" : "no") +
                               " camera_depth=" + (saw_depth ? "yes" : "no") +
                               " camera_info=" + (saw_info ? "yes" : "no"));
    }
    dds_delete(participant);
    std::printf(
        "sensor publisher DDS process passed: domain=%d imu=%s "
        "lidar_packet=%s lidar_frame=%s camera_color=%s camera_depth=%s camera_info=%s\n",
        domain_id, lingtu::message::kImuRaw.dds_topic.data(),
        lingtu::message::kLidarRawPacket.dds_topic.data(),
        lingtu::message::kLidarRawFrame.dds_topic.data(),
        lingtu::message::kCameraColor.dds_topic.data(),
        lingtu::message::kCameraDepth.dds_topic.data(),
        lingtu::message::kCameraInfo.dds_topic.data());
    return 0;
  } catch (const std::exception &exc) {
    if (participant > 0) {
      dds_delete(participant);
    }
    std::fprintf(stderr, "test_sensor_publisher_dds_process failed: %s\n", exc.what());
    return 1;
  }
}
