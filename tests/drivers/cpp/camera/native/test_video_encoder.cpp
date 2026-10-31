#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "shm_reader.hpp"
#include "video/encoder.hpp"

namespace {

namespace camera_shm = lingtu::drivers::camera::shm;
namespace camera_video = lingtu::drivers::camera::video;

int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (condition) {
    return;
  }
  std::cerr << "video encoder test failed at line " << line << ": " << expression << '\n';
  ++failures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

template <typename Function>
void expect_failure(Function&& function, const char* text) {
  try {
    function();
    CHECK(false);
  } catch (const std::exception& error) {
    CHECK(std::string(error.what()).find(text) != std::string::npos);
  }
}

class TemporaryFile {
 public:
  TemporaryFile() {
    std::string pattern = "/tmp/lingtu-video-encoder-XXXXXX";
    std::vector<char> mutable_pattern(pattern.begin(), pattern.end());
    mutable_pattern.push_back('\0');
    fd_ = ::mkstemp(mutable_pattern.data());
    if (fd_ < 0) {
      throw std::runtime_error("mkstemp failed");
    }
    path_ = mutable_pattern.data();
  }

  ~TemporaryFile() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    ::unlink(path_.c_str());
  }

  TemporaryFile(const TemporaryFile&) = delete;
  TemporaryFile& operator=(const TemporaryFile&) = delete;

  int fd() const { return fd_; }
  const std::string& path() const { return path_; }

  std::vector<std::uint8_t> bytes() const {
    if (::lseek(fd_, 0, SEEK_SET) < 0) {
      throw std::runtime_error("lseek failed");
    }
    std::vector<std::uint8_t> output;
    std::uint8_t buffer[4096];
    for (;;) {
      const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
      if (count == 0) {
        return output;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count < 0) {
        throw std::runtime_error("read failed");
      }
      output.insert(output.end(), buffer, buffer + count);
    }
  }

 private:
  int fd_{-1};
  std::string path_;
};

std::string self_executable() {
  std::vector<char> path(4096);
  const ssize_t size = ::readlink("/proc/self/exe", path.data(), path.size() - 1);
  if (size <= 0) {
    throw std::runtime_error("readlink /proc/self/exe failed");
  }
  path[static_cast<std::size_t>(size)] = '\0';
  return path.data();
}

camera_shm::ColorFrame make_frame(std::uint64_t sequence = 1, std::uint32_t width = 4,
                                  std::uint32_t height = 2, std::uint32_t stride = 16,
                                  std::string encoding = "rgb8") {
  camera_shm::ColorFrame frame;
  frame.source_generation = 17;
  frame.sequence = sequence;
  frame.timestamp_ns = 1'000'000'000 + sequence;
  frame.width = width;
  frame.height = height;
  frame.stride = stride;
  frame.encoding = std::move(encoding);
  frame.frame_id = "camera_color";
  frame.payload.resize(static_cast<std::size_t>(stride) * height);
  for (std::size_t index = 0; index < frame.payload.size(); ++index) {
    frame.payload[index] = static_cast<std::uint8_t>(index);
  }
  return frame;
}

camera_video::EncoderOptions fake_options(const std::string& mode) {
  camera_video::EncoderOptions options;
  options.executable = self_executable();
  options.codec = mode;
  options.fps = 20;
  options.bitrate_kbps = 900;
  options.gop = 10;
  options.write_timeout = std::chrono::milliseconds(500);
  return options;
}

std::string option_value(int argc, char** argv, const std::string& option) {
  for (int index = 1; index + 1 < argc; ++index) {
    if (argv[index] == option) {
      return argv[index + 1];
    }
  }
  return {};
}

int run_fake_encoder(int argc, char** argv) {
  const std::string mode = option_value(argc, argv, "-c:v");
  if (const char* argument_path = std::getenv("LINGTU_VIDEO_TEST_ARGUMENTS")) {
    std::ofstream output(argument_path);
    for (int index = 1; index < argc; ++index) {
      output << argv[index] << '\n';
    }
  }
  if (mode == "fake-exit") {
    return 9;
  }
  if (mode == "fake-hang") {
    for (;;) {
      ::pause();
    }
  }
  if (mode != "fake-copy") {
    return 10;
  }

  std::uint8_t buffer[4096];
  for (;;) {
    ssize_t count;
    do {
      count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    } while (count < 0 && errno == EINTR);
    if (count == 0) {
      return 0;
    }
    if (count < 0) {
      return 11;
    }
    std::size_t offset = 0;
    while (offset < static_cast<std::size_t>(count)) {
      ssize_t written;
      do {
        written = ::write(STDOUT_FILENO, buffer + offset,
                          static_cast<std::size_t>(count) - offset);
      } while (written < 0 && errno == EINTR);
      if (written <= 0) {
        return 12;
      }
      offset += static_cast<std::size_t>(written);
    }
  }
}

void test_padding_is_removed_and_arguments_are_explicit() {
  TemporaryFile output;
  TemporaryFile arguments;
  CHECK(::setenv("LINGTU_VIDEO_TEST_ARGUMENTS", arguments.path().c_str(), 1) == 0);
  const auto frame = make_frame();
  camera_video::Encoder encoder(fake_options("fake-copy"));
  encoder.begin(frame, output.fd());
  encoder.write(frame);
  encoder.finish();
  CHECK(::unsetenv("LINGTU_VIDEO_TEST_ARGUMENTS") == 0);

  const auto encoded = output.bytes();
  const std::vector<std::uint8_t> expected{
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
      16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
  CHECK(encoded == expected);

  std::ifstream argument_input(arguments.path());
  const std::string argument_text((std::istreambuf_iterator<char>(argument_input)),
                                  std::istreambuf_iterator<char>());
  CHECK(argument_text.find("rawvideo\n") != std::string::npos);
  CHECK(argument_text.find("rgb24\n") != std::string::npos);
  CHECK(argument_text.find("4x2\n") != std::string::npos);
  CHECK(argument_text.find("20\n") != std::string::npos);
  CHECK(argument_text.find("900k\n") != std::string::npos);
  CHECK(argument_text.find("h264\n") != std::string::npos);

  const std::uint8_t marker = 0x5a;
  CHECK(::write(output.fd(), &marker, 1) == 1);
}

void test_generation_or_geometry_change_requires_a_new_stream() {
  TemporaryFile output;
  auto frame = make_frame();
  camera_video::Encoder encoder(fake_options("fake-copy"));
  encoder.begin(frame, output.fd());
  encoder.write(frame);
  auto changed = make_frame(2);
  changed.source_generation = 18;
  expect_failure([&] { encoder.write(changed); }, "does not match");
}

void test_missing_executable_is_reported() {
  TemporaryFile output;
  auto options = fake_options("fake-copy");
  options.executable = "/definitely/missing/lingtu-ffmpeg";
  camera_video::Encoder encoder(options);
  expect_failure([&] { encoder.begin(make_frame(), output.fd()); }, "executing FFmpeg");
}

void test_nonreading_encoder_hits_one_frame_deadline() {
  TemporaryFile output;
  auto options = fake_options("fake-hang");
  options.write_timeout = std::chrono::milliseconds(500);
  camera_video::Encoder encoder(options);
  const auto frame = make_frame(1, 640, 480, 640 * 3);
  encoder.begin(frame, output.fd());
  const auto started = std::chrono::steady_clock::now();
  expect_failure([&] { encoder.write(frame); }, "timed out");
  const auto elapsed = std::chrono::steady_clock::now() - started;
  CHECK(elapsed >= std::chrono::milliseconds(350));
  CHECK(elapsed < std::chrono::milliseconds(1000));
}

void test_cancellation_aborts_the_stream() {
  TemporaryFile output;
  std::atomic_bool stop{false};
  auto options = fake_options("fake-copy");
  options.stop_requested = [&] { return stop.load(); };
  camera_video::Encoder encoder(options);
  const auto frame = make_frame();
  encoder.begin(frame, output.fd());
  stop.store(true);
  expect_failure([&] { encoder.write(frame); }, "cancelled");
  expect_failure([&] { encoder.finish(); }, "not running");
}

void test_early_child_exit_is_reported() {
  TemporaryFile output;
  camera_video::Encoder encoder(fake_options("fake-exit"));
  const auto frame = make_frame(1, 640, 480, 640 * 3);
  encoder.begin(frame, output.fd());
  expect_failure([&] { encoder.write(frame); }, "FFmpeg video encoder");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && option_value(argc, argv, "-c:v").find("fake-") == 0) {
    return run_fake_encoder(argc, argv);
  }
  test_padding_is_removed_and_arguments_are_explicit();
  test_generation_or_geometry_change_requires_a_new_stream();
  test_missing_executable_is_reported();
  test_nonreading_encoder_hits_one_frame_deadline();
  test_cancellation_aborts_the_stream();
  test_early_child_exit_is_reported();
  return failures == 0 ? 0 : 1;
}
