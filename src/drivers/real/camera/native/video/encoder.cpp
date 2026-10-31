#include "encoder.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "../shm_reader.hpp"

namespace lingtu::drivers::camera::video {
namespace {

[[noreturn]] void throw_system_error(const std::string& operation,
                                     int error_number = errno) {
  throw std::runtime_error(operation + " failed: " + std::strerror(error_number));
}

void close_fd(int& fd) noexcept {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

std::string child_status(int status) {
  if (WIFEXITED(status)) {
    return "exit code " + std::to_string(WEXITSTATUS(status));
  }
  if (WIFSIGNALED(status)) {
    return "signal " + std::to_string(WTERMSIG(status));
  }
  return "unknown status";
}

void validate_frame(const shm::ColorFrame& frame) {
  if (frame.source_generation == 0 || frame.sequence == 0 || frame.timestamp_ns == 0 ||
      frame.width == 0 || frame.height == 0 ||
      frame.width > std::numeric_limits<std::uint32_t>::max() / 3u ||
      frame.stride < frame.width * 3u ||
      frame.height > std::numeric_limits<std::size_t>::max() / frame.stride ||
      frame.payload.size() != static_cast<std::size_t>(frame.stride) * frame.height ||
      (frame.encoding != "rgb8" && frame.encoding != "bgr8")) {
    throw std::invalid_argument("video encoder received an invalid color frame");
  }
  if ((frame.width & 1u) != 0 || (frame.height & 1u) != 0) {
    throw std::invalid_argument("video encoder yuv420p output requires even frame dimensions");
  }
}

class SigpipeBlock {
 public:
  SigpipeBlock() {
    sigemptyset(&blocked_);
    sigaddset(&blocked_, SIGPIPE);
    const int error = ::pthread_sigmask(SIG_BLOCK, &blocked_, &previous_);
    if (error != 0) {
      throw_system_error("blocking SIGPIPE for video encoder", error);
    }
    sigset_t pending;
    was_pending_ = ::sigpending(&pending) == 0 && sigismember(&pending, SIGPIPE) == 1;
  }

  ~SigpipeBlock() {
    if (!was_pending_) {
      timespec no_wait{};
      while (::sigtimedwait(&blocked_, nullptr, &no_wait) < 0 && errno == EINTR) {
      }
    }
    ::pthread_sigmask(SIG_SETMASK, &previous_, nullptr);
  }

  SigpipeBlock(const SigpipeBlock&) = delete;
  SigpipeBlock& operator=(const SigpipeBlock&) = delete;

 private:
  sigset_t blocked_{};
  sigset_t previous_{};
  bool was_pending_{false};
};

int remaining_milliseconds(std::chrono::steady_clock::time_point deadline) {
  const auto remaining = deadline - std::chrono::steady_clock::now();
  if (remaining <= std::chrono::steady_clock::duration::zero()) {
    return 0;
  }
  const auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
  if (milliseconds >= std::chrono::milliseconds(std::numeric_limits<int>::max())) {
    return std::numeric_limits<int>::max();
  }
  return static_cast<int>(milliseconds.count()) + 1;
}

}  // namespace

struct Encoder::Impl {
  explicit Impl(EncoderOptions input_options) : options(std::move(input_options)) {
    if (options.executable.empty() || options.codec.empty() ||
        options.executable.find('\0') != std::string::npos ||
        options.codec.find('\0') != std::string::npos || options.fps == 0 ||
        options.bitrate_kbps == 0 || options.gop == 0 || options.write_timeout.count() <= 0) {
      throw std::invalid_argument("video encoder options are invalid");
    }
  }

  ~Impl() { abort(); }

  bool stopped() const { return options.stop_requested && options.stop_requested(); }

  void check_running() {
    if (child_pid <= 0 || input_fd < 0) {
      throw std::logic_error("video encoder is not running");
    }
    int status = 0;
    const pid_t result = ::waitpid(child_pid, &status, WNOHANG);
    if (result == 0) {
      return;
    }
    if (result < 0) {
      throw_system_error("checking FFmpeg video encoder");
    }
    child_pid = -1;
    close_fd(input_fd);
    throw std::runtime_error("FFmpeg video encoder exited early with " + child_status(status));
  }

  void begin(const shm::ColorFrame& frame, int output_fd) {
    if (child_pid > 0 || input_fd >= 0) {
      throw std::logic_error("video encoder is already running");
    }
    validate_frame(frame);
    if (output_fd < 0 || ::fcntl(output_fd, F_GETFD) < 0) {
      throw std::invalid_argument("video encoder output file descriptor is invalid");
    }
    if (stopped()) {
      throw Cancelled("video encoder start was cancelled");
    }

    int input_pipe[2]{-1, -1};
    int exec_pipe[2]{-1, -1};
    if (::pipe2(input_pipe, O_CLOEXEC) != 0 || ::pipe2(exec_pipe, O_CLOEXEC) != 0) {
      const int saved_errno = errno;
      close_fd(input_pipe[0]);
      close_fd(input_pipe[1]);
      close_fd(exec_pipe[0]);
      close_fd(exec_pipe[1]);
      throw_system_error("creating FFmpeg video encoder pipes", saved_errno);
    }

    const std::string pixel_format = frame.encoding == "rgb8" ? "rgb24" : "bgr24";
    const std::string video_size =
        std::to_string(frame.width) + "x" + std::to_string(frame.height);
    const std::string fps = std::to_string(options.fps);
    const std::string bitrate = std::to_string(options.bitrate_kbps) + "k";
    const std::string gop = std::to_string(options.gop);
    std::vector<std::string> arguments{
        options.executable, "-hide_banner", "-loglevel", "error", "-nostdin",
        "-f", "rawvideo", "-pixel_format", pixel_format, "-video_size", video_size,
        "-framerate", fps, "-i", "pipe:0", "-an", "-c:v", options.codec};
    if (options.codec == "libx264") {
      arguments.insert(arguments.end(),
                       {"-preset", "ultrafast", "-tune", "zerolatency", "-profile:v",
                        "baseline", "-x264-params",
                        "repeat-headers=1:aud=1:scenecut=0"});
    }
    arguments.insert(arguments.end(),
                     {"-b:v", bitrate, "-maxrate", bitrate, "-bufsize", bitrate, "-pix_fmt",
                      "yuv420p", "-bf", "0", "-g", gop, "-keyint_min", gop,
                      "-flush_packets", "1", "-f", "h264", "pipe:1"});

    std::vector<char*> argv;
    argv.reserve(arguments.size() + 1);
    for (auto& argument : arguments) {
      argv.push_back(argument.data());
    }
    argv.push_back(nullptr);

    const pid_t pid = ::fork();
    if (pid < 0) {
      const int saved_errno = errno;
      close_fd(input_pipe[0]);
      close_fd(input_pipe[1]);
      close_fd(exec_pipe[0]);
      close_fd(exec_pipe[1]);
      throw_system_error("forking FFmpeg video encoder", saved_errno);
    }
    if (pid == 0) {
      close_fd(input_pipe[1]);
      close_fd(exec_pipe[0]);
      if (input_pipe[0] != STDIN_FILENO && ::dup2(input_pipe[0], STDIN_FILENO) < 0) {
        const int saved_errno = errno;
        [[maybe_unused]] const auto reported = ::write(exec_pipe[1], &saved_errno, sizeof(saved_errno));
        _exit(127);
      }
      if (output_fd != STDOUT_FILENO && ::dup2(output_fd, STDOUT_FILENO) < 0) {
        const int saved_errno = errno;
        [[maybe_unused]] const auto reported = ::write(exec_pipe[1], &saved_errno, sizeof(saved_errno));
        _exit(127);
      }
      if (input_pipe[0] != STDIN_FILENO) {
        close_fd(input_pipe[0]);
      }
      if (output_fd != STDOUT_FILENO) {
        ::close(output_fd);
      }
      ::execvp(argv[0], argv.data());
      const int saved_errno = errno;
      [[maybe_unused]] const auto reported = ::write(exec_pipe[1], &saved_errno, sizeof(saved_errno));
      _exit(127);
    }

    close_fd(input_pipe[0]);
    close_fd(exec_pipe[1]);
    const int input_flags = ::fcntl(input_pipe[1], F_GETFL, 0);
    if (input_flags < 0 || ::fcntl(input_pipe[1], F_SETFL, input_flags | O_NONBLOCK) < 0) {
      const int saved_errno = errno;
      close_fd(input_pipe[1]);
      ::kill(pid, SIGKILL);
      int status = 0;
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      close_fd(exec_pipe[0]);
      throw_system_error("configuring FFmpeg video encoder input", saved_errno);
    }

    const auto startup_deadline = std::chrono::steady_clock::now() + options.write_timeout;
    int exec_ready = 0;
    bool cancelled = false;
    while (exec_ready == 0 && remaining_milliseconds(startup_deadline) > 0) {
      if (stopped()) {
        cancelled = true;
        break;
      }
      pollfd exec_status{};
      exec_status.fd = exec_pipe[0];
      exec_status.events = POLLIN;
      const int slice = std::min(remaining_milliseconds(startup_deadline), 20);
      exec_ready = ::poll(&exec_status, 1, slice);
      if (exec_ready < 0 && errno == EINTR) {
        exec_ready = 0;
      }
    }
    if (exec_ready <= 0 || cancelled) {
      const int saved_errno = exec_ready < 0 ? errno : ETIMEDOUT;
      close_fd(input_pipe[1]);
      close_fd(exec_pipe[0]);
      ::kill(pid, SIGKILL);
      int status = 0;
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      if (cancelled) {
        throw Cancelled("video encoder start was cancelled");
      }
      if (exec_ready == 0) {
        throw std::runtime_error("FFmpeg video encoder start timed out");
      }
      throw_system_error("waiting for FFmpeg video encoder startup", saved_errno);
    }
    int exec_error = 0;
    ssize_t exec_bytes;
    do {
      exec_bytes = ::read(exec_pipe[0], &exec_error, sizeof(exec_error));
    } while (exec_bytes < 0 && errno == EINTR);
    const int read_error = errno;
    close_fd(exec_pipe[0]);
    if (exec_bytes != 0) {
      close_fd(input_pipe[1]);
      int status = 0;
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      if (exec_bytes == static_cast<ssize_t>(sizeof(exec_error))) {
        throw_system_error("executing FFmpeg video encoder", exec_error);
      }
      throw_system_error("reading FFmpeg exec status", exec_bytes < 0 ? read_error : EIO);
    }

    input_fd = input_pipe[1];
    child_pid = pid;
    source_generation = frame.source_generation;
    width = frame.width;
    height = frame.height;
    stride = frame.stride;
    encoding = frame.encoding;
    last_sequence = 0;
  }

  void write_bytes(const std::uint8_t* data, std::size_t size,
                   std::chrono::steady_clock::time_point deadline) {
    SigpipeBlock sigpipe;
    while (size > 0) {
      if (stopped()) {
        throw Cancelled("video encoder frame write was cancelled");
      }
      const int timeout = std::min(remaining_milliseconds(deadline), 20);
      if (timeout == 0) {
        throw std::runtime_error("video encoder frame write timed out");
      }
      pollfd descriptor{};
      descriptor.fd = input_fd;
      descriptor.events = POLLOUT;
      const int ready = ::poll(&descriptor, 1, timeout);
      if (ready == 0) {
        continue;
      }
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready < 0) {
        throw_system_error("polling FFmpeg video encoder input");
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        check_running();
        throw std::runtime_error("FFmpeg video encoder closed its input");
      }
      ssize_t count;
      do {
        count = ::write(input_fd, data, size);
      } while (count < 0 && errno == EINTR);
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;
      }
      if (count < 0) {
        throw_system_error("writing raw frame to FFmpeg video encoder");
      }
      if (count == 0) {
        throw std::runtime_error("writing raw frame to FFmpeg video encoder made no progress");
      }
      data += count;
      size -= static_cast<std::size_t>(count);
    }
  }

  void write(const shm::ColorFrame& frame) {
    check_running();
    validate_frame(frame);
    if (frame.source_generation != source_generation || frame.width != width ||
        frame.height != height || frame.stride != stride || frame.encoding != encoding ||
        (last_sequence != 0 && frame.sequence <= last_sequence)) {
      throw std::runtime_error("color frame does not match the active video stream");
    }

    const auto deadline = std::chrono::steady_clock::now() + options.write_timeout;
    const std::size_t row_size = static_cast<std::size_t>(frame.width) * 3u;
    try {
      if (frame.stride == row_size) {
        write_bytes(frame.payload.data(), frame.payload.size(), deadline);
      } else {
        for (std::uint32_t row = 0; row < frame.height; ++row) {
          write_bytes(frame.payload.data() + static_cast<std::size_t>(row) * frame.stride,
                      row_size, deadline);
        }
      }
      last_sequence = frame.sequence;
    } catch (...) {
      abort();
      throw;
    }
  }

  bool wait_until(std::chrono::steady_clock::time_point deadline, int& status) noexcept {
    while (std::chrono::steady_clock::now() < deadline) {
      const pid_t result = ::waitpid(child_pid, &status, WNOHANG);
      if (result == child_pid || (result < 0 && errno == ECHILD)) {
        child_pid = -1;
        return true;
      }
      if (result < 0 && errno != EINTR) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  void clear_stream() noexcept {
    source_generation = 0;
    width = 0;
    height = 0;
    stride = 0;
    encoding.clear();
    last_sequence = 0;
  }

  void finish() {
    if (child_pid <= 0 || input_fd < 0) {
      throw std::logic_error("video encoder is not running");
    }
    close_fd(input_fd);
    int status = 0;
    if (!wait_until(std::chrono::steady_clock::now() + options.write_timeout, status)) {
      abort();
      throw std::runtime_error("FFmpeg video encoder did not finish before the deadline");
    }
    clear_stream();
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      throw std::runtime_error("FFmpeg video encoder failed with " + child_status(status));
    }
  }

  void abort() noexcept {
    close_fd(input_fd);
    if (child_pid <= 0) {
      clear_stream();
      return;
    }
    ::kill(child_pid, SIGTERM);
    int status = 0;
    if (!wait_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(100), status)) {
      ::kill(child_pid, SIGKILL);
      pid_t result;
      do {
        result = ::waitpid(child_pid, &status, 0);
      } while (result < 0 && errno == EINTR);
    }
    child_pid = -1;
    clear_stream();
  }

  EncoderOptions options;
  int input_fd{-1};
  pid_t child_pid{-1};
  std::uint64_t source_generation{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t stride{0};
  std::string encoding;
  std::uint64_t last_sequence{0};
};

Encoder::Encoder(EncoderOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}

Encoder::~Encoder() = default;

void Encoder::begin(const shm::ColorFrame& frame, int output_fd) {
  impl_->begin(frame, output_fd);
}

void Encoder::write(const shm::ColorFrame& frame) { impl_->write(frame); }

void Encoder::finish() { impl_->finish(); }

void Encoder::abort() noexcept { impl_->abort(); }

}  // namespace lingtu::drivers::camera::video
