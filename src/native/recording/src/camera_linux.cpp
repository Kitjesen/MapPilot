#include "lingtu/recording/camera_linux.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <locale>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "drivers/real/camera/native/shm_frame_ring.hpp"

namespace lingtu::recording {
namespace {

namespace camera_shm = lingtu::drivers::camera::shm;

[[noreturn]] void throw_system_error(const std::string &operation, int error_number = errno) {
  throw std::runtime_error(operation + " failed: " + std::strerror(error_number));
}

template <typename T>
T atomic_load(const T *address) {
  return __atomic_load_n(address, __ATOMIC_ACQUIRE);
}

void validate_shm_name(const std::string &name) {
  if (name.size() < 2 || name.front() != '/' || name.find('/', 1) != std::string::npos ||
      name.find('\0') != std::string::npos) {
    throw std::invalid_argument("camera POSIX SHM name must have exactly one leading slash");
  }
}

std::size_t mapping_size(const camera_shm::SharedHeader &header) {
  if (std::memcmp(header.magic, camera_shm::kMagic, sizeof(camera_shm::kMagic)) != 0) {
    throw std::runtime_error("camera SHM magic is not LTCSHM01");
  }
  if (header.schema_version != camera_shm::kSchemaVersion ||
      header.header_size != sizeof(camera_shm::SharedHeader) ||
      header.slot_header_size != sizeof(camera_shm::SlotHeader)) {
    throw std::runtime_error("camera SHM schema or layout is incompatible");
  }
  if (header.slot_count < 2 || header.slot_capacity == 0) {
    throw std::runtime_error("camera SHM slot layout is invalid");
  }
  const auto slot_size =
      sizeof(camera_shm::SlotHeader) + static_cast<std::size_t>(header.slot_capacity);
  if (header.slot_count >
      (std::numeric_limits<std::size_t>::max() - sizeof(camera_shm::SharedHeader)) / slot_size) {
    throw std::runtime_error("camera SHM mapping size overflows size_t");
  }
  return sizeof(camera_shm::SharedHeader) + static_cast<std::size_t>(header.slot_count) * slot_size;
}

std::string fixed_string(const char *data, std::size_t size, const char *field) {
  const void *terminator = std::memchr(data, '\0', size);
  if (terminator == nullptr) {
    throw std::runtime_error(std::string("camera SHM ") + field + " is not terminated");
  }
  return std::string(data, static_cast<const char *>(terminator));
}

std::string ffmpeg_number(double value) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return stream.str();
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

void close_fd(int &fd) noexcept {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

ssize_t write_blocking_sigpipe(int fd, const void *data, std::size_t size) {
  sigset_t blocked;
  sigset_t previous;
  sigemptyset(&blocked);
  sigaddset(&blocked, SIGPIPE);
  const int mask_error = ::pthread_sigmask(SIG_BLOCK, &blocked, &previous);
  if (mask_error != 0) {
    errno = mask_error;
    return -1;
  }

  sigset_t pending;
  bool pipe_was_pending = false;
  if (::sigpending(&pending) == 0) {
    pipe_was_pending = sigismember(&pending, SIGPIPE) == 1;
  }

  ssize_t result;
  do {
    result = ::write(fd, data, size);
  } while (result < 0 && errno == EINTR);
  const int write_error = errno;

  if (result < 0 && write_error == EPIPE && !pipe_was_pending) {
    timespec no_wait{};
    while (::sigtimedwait(&blocked, nullptr, &no_wait) < 0 && errno == EINTR) {}
  }
  const int restore_error = ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
  if (result >= 0 && restore_error != 0) {
    errno = restore_error;
    return -1;
  }
  errno = write_error;
  return result;
}

void write_all(int fd, const std::byte *data, std::size_t size) {
  while (size > 0) {
    const auto count = write_blocking_sigpipe(fd, data, size);
    if (count < 0) {
      throw_system_error("writing raw camera frame to FFmpeg");
    }
    if (count == 0) {
      throw std::runtime_error("writing raw camera frame to FFmpeg made no progress");
    }
    data += count;
    size -= static_cast<std::size_t>(count);
  }
}

}  // namespace

struct PosixShmCameraFrameSource::Impl {
  explicit Impl(PosixShmCameraSourceOptions input_options) : options(std::move(input_options)) {
    validate_shm_name(options.name);
    if (options.poll_interval.count() < 0) {
      throw std::invalid_argument("camera SHM poll interval must not be negative");
    }

    fd = ::shm_open(options.name.c_str(), O_RDONLY | O_CLOEXEC, 0);
    if (fd < 0) {
      throw_system_error("opening camera SHM " + options.name);
    }
    camera_shm::SharedHeader disk_header{};
    const auto bytes = ::pread(fd, &disk_header, sizeof(disk_header), 0);
    if (bytes != static_cast<ssize_t>(sizeof(disk_header))) {
      const int saved_errno = bytes < 0 ? errno : EIO;
      close_fd(fd);
      throw_system_error("reading camera SHM header", saved_errno);
    }
    expected_size = mapping_size(disk_header);
    struct stat info{};
    if (::fstat(fd, &info) != 0) {
      const int saved_errno = errno;
      close_fd(fd);
      throw_system_error("stat camera SHM", saved_errno);
    }
    if (info.st_size < 0 || static_cast<std::uintmax_t>(info.st_size) != expected_size) {
      close_fd(fd);
      throw std::runtime_error("camera SHM file size does not match its declared layout");
    }

    mapping = ::mmap(nullptr, expected_size, PROT_READ, MAP_SHARED, fd, 0);
    if (mapping == MAP_FAILED) {
      mapping = nullptr;
      const int saved_errno = errno;
      close_fd(fd);
      throw_system_error("mapping camera SHM", saved_errno);
    }
    header = static_cast<const camera_shm::SharedHeader *>(mapping);
    const auto mapped_size = mapping_size(*header);
    if (mapped_size != expected_size || header->slot_count != disk_header.slot_count ||
        header->slot_capacity != disk_header.slot_capacity) {
      close();
      throw std::runtime_error("camera SHM layout changed while it was opened");
    }
    slot_count = header->slot_count;
    slot_capacity = header->slot_capacity;
  }

  ~Impl() { close(); }

  void close() noexcept {
    if (mapping != nullptr) {
      ::munmap(mapping, expected_size);
      mapping = nullptr;
      header = nullptr;
      expected_size = 0;
    }
    close_fd(fd);
  }

  bool stopped() const {
    return (options.max_frames != 0 && delivered_frames >= options.max_frames) ||
           (options.stop_requested && options.stop_requested());
  }

  const camera_shm::SlotHeader *slot_header(std::uint32_t index) const {
    const auto *base = static_cast<const std::byte *>(mapping) + sizeof(camera_shm::SharedHeader);
    return reinterpret_cast<const camera_shm::SlotHeader *>(
        base + static_cast<std::size_t>(index) * (sizeof(camera_shm::SlotHeader) + slot_capacity));
  }

  const std::byte *slot_payload(const camera_shm::SlotHeader *slot) const {
    return reinterpret_cast<const std::byte *>(slot) + sizeof(camera_shm::SlotHeader);
  }

  bool try_read(CameraFrame &output) {
    if (std::memcmp(header->magic, camera_shm::kMagic, sizeof(camera_shm::kMagic)) != 0 ||
        header->schema_version != camera_shm::kSchemaVersion ||
        header->header_size != sizeof(camera_shm::SharedHeader) ||
        header->slot_header_size != sizeof(camera_shm::SlotHeader) ||
        header->slot_count != slot_count || header->slot_capacity != slot_capacity) {
      throw std::runtime_error("camera SHM layout changed while recording");
    }

    const auto published = atomic_load(&header->published_sequence);
    if (published == 0 || published == last_sequence) {
      return false;
    }
    if (published < last_sequence) {
      throw std::runtime_error("camera SHM published sequence regressed");
    }
    const auto active_slot = atomic_load(&header->active_slot);
    if (active_slot >= slot_count) {
      throw std::runtime_error("camera SHM active slot is out of range");
    }
    const auto oldest_available =
        published > slot_count ? published - static_cast<std::uint64_t>(slot_count) + 1u : 1u;
    auto target_sequence = last_sequence == 0 ? published : last_sequence + 1u;
    if (target_sequence < oldest_available) {
      target_sequence = oldest_available;
    }
    const auto index = static_cast<std::uint32_t>((target_sequence - 1u) % slot_count);
    const auto *slot = slot_header(index);
    const auto first_guard = atomic_load(&slot->guard_begin);
    if (first_guard == 0 || (first_guard & 1u) != 0) {
      return false;
    }

    camera_shm::SlotHeader snapshot{};
    std::memcpy(&snapshot, slot, sizeof(snapshot));
    if (snapshot.payload_capacity != slot_capacity || snapshot.payload_size > slot_capacity) {
      throw std::runtime_error("camera SHM slot payload layout is invalid");
    }
    std::vector<std::byte> payload(snapshot.payload_size);
    if (!payload.empty()) {
      std::memcpy(payload.data(), slot_payload(slot), payload.size());
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    const auto end_guard = atomic_load(&slot->guard_end);
    const auto final_guard = atomic_load(&slot->guard_begin);
    if (first_guard != end_guard || end_guard != final_guard ||
        snapshot.guard_begin != first_guard || snapshot.guard_end != first_guard) {
      return false;
    }
    if (snapshot.sequence != target_sequence || snapshot.sequence == 0 ||
        snapshot.sequence > std::numeric_limits<std::uint64_t>::max() / 2u ||
        first_guard != snapshot.sequence * 2u) {
      return false;
    }
    if (snapshot.schema_version != camera_shm::kSchemaVersion ||
        snapshot.header_size != sizeof(camera_shm::SlotHeader) ||
        snapshot.stream_kind != static_cast<std::uint16_t>(camera_shm::StreamKind::kColor)) {
      throw std::runtime_error("camera SHM slot schema or stream kind is invalid");
    }
    if (snapshot.encoding_size == 0 || snapshot.encoding_size >= sizeof(snapshot.encoding) ||
        snapshot.encoding[snapshot.encoding_size] != '\0') {
      throw std::runtime_error("camera SHM encoding field is invalid");
    }
    const std::string encoding(snapshot.encoding, snapshot.encoding_size);
    if (encoding != "rgb8" && encoding != "bgr8") {
      throw std::runtime_error("camera recorder accepts only rgb8 or bgr8 SHM frames");
    }
    const auto frame_id = fixed_string(snapshot.frame_id, sizeof(snapshot.frame_id), "frame_id");
    if (snapshot.timestamp_ns == 0 || snapshot.width == 0 || snapshot.height == 0 ||
        snapshot.width > std::numeric_limits<std::uint32_t>::max() / 3u ||
        snapshot.stride < snapshot.width * 3u ||
        snapshot.height > std::numeric_limits<std::size_t>::max() / snapshot.stride ||
        snapshot.payload_size != static_cast<std::size_t>(snapshot.stride) * snapshot.height) {
      throw std::runtime_error("camera SHM color geometry is invalid");
    }
    if (camera_shm::crc32(payload.data(), payload.size()) != snapshot.payload_crc32) {
      throw std::runtime_error("camera SHM payload CRC32 mismatch");
    }

    output.sequence = snapshot.sequence;
    output.timestamp_ns = snapshot.timestamp_ns;
    output.width = snapshot.width;
    output.height = snapshot.height;
    output.stride = snapshot.stride;
    output.encoding = encoding;
    output.frame_id = frame_id;
    output.fx = snapshot.fx;
    output.fy = snapshot.fy;
    output.cx = snapshot.cx;
    output.cy = snapshot.cy;
    output.depth_scale = snapshot.depth_scale;
    output.dist_k1 = snapshot.dist_k1;
    output.dist_k2 = snapshot.dist_k2;
    output.dist_p1 = snapshot.dist_p1;
    output.dist_p2 = snapshot.dist_p2;
    output.dist_k3 = snapshot.dist_k3;
    output.payload = std::move(payload);
    last_sequence = snapshot.sequence;
    ++delivered_frames;
    return true;
  }

  bool next(CameraFrame &frame) {
    while (!stopped()) {
      if (try_read(frame)) {
        return true;
      }
      std::this_thread::sleep_for(options.poll_interval);
    }
    return false;
  }

  PosixShmCameraSourceOptions options;
  int fd{-1};
  void *mapping{nullptr};
  std::size_t expected_size{0};
  const camera_shm::SharedHeader *header{nullptr};
  std::uint16_t slot_count{0};
  std::uint32_t slot_capacity{0};
  std::uint64_t last_sequence{0};
  std::uint64_t delivered_frames{0};
};

PosixShmCameraFrameSource::PosixShmCameraFrameSource(PosixShmCameraSourceOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

PosixShmCameraFrameSource::~PosixShmCameraFrameSource() = default;

bool PosixShmCameraFrameSource::next(CameraFrame &frame) {
  return impl_->next(frame);
}

struct FfmpegSegmentEncoder::Impl {
  explicit Impl(FfmpegSegmentEncoderOptions input_options) : options(std::move(input_options)) {
    if (options.executable.empty() || options.codec.empty() ||
        options.executable.find('\0') != std::string::npos ||
        options.codec.find('\0') != std::string::npos) {
      throw std::invalid_argument("FFmpeg executable and codec must not be empty");
    }
  }

  ~Impl() { abort(); }

  void check_running() {
    if (child_pid <= 0) {
      throw std::logic_error("FFmpeg camera segment is not open");
    }
    int status = 0;
    const auto result = ::waitpid(child_pid, &status, WNOHANG);
    if (result == 0) {
      return;
    }
    if (result < 0) {
      throw_system_error("checking FFmpeg camera encoder");
    }
    child_pid = -1;
    close_fd(input_fd);
    throw std::runtime_error("FFmpeg camera encoder exited early with " + child_status(status));
  }

  void begin(const std::filesystem::path &path, const CameraStreamDescription &input_stream) {
    if (child_pid > 0 || input_fd >= 0) {
      throw std::logic_error("FFmpeg camera segment is already open");
    }
    if (path.empty() || input_stream.width == 0 || input_stream.height == 0 ||
        input_stream.width > std::numeric_limits<std::uint32_t>::max() / 3u ||
        input_stream.stride < input_stream.width * 3u ||
        (input_stream.encoding != "rgb8" && input_stream.encoding != "bgr8") ||
        !std::isfinite(input_stream.nominal_fps) || input_stream.nominal_fps <= 0.0) {
      throw std::invalid_argument("FFmpeg camera stream description is invalid");
    }
    if (input_stream.width % 2u != 0 || input_stream.height % 2u != 0) {
      throw std::invalid_argument(
          "FFmpeg camera encoder uses yuv420p and requires even frame dimensions");
    }
    if (std::filesystem::exists(path)) {
      throw std::runtime_error("FFmpeg camera output already exists: " + path.string());
    }

    int input_pipe[2]{-1, -1};
    int exec_pipe[2]{-1, -1};
    if (::pipe2(input_pipe, O_CLOEXEC) != 0 || ::pipe2(exec_pipe, O_CLOEXEC) != 0) {
      const int saved_errno = errno;
      close_fd(input_pipe[0]);
      close_fd(input_pipe[1]);
      close_fd(exec_pipe[0]);
      close_fd(exec_pipe[1]);
      throw_system_error("creating FFmpeg pipes", saved_errno);
    }

    const std::string pixel_format = input_stream.encoding == "rgb8" ? "rgb24" : "bgr24";
    const std::string video_size =
        std::to_string(input_stream.width) + "x" + std::to_string(input_stream.height);
    const std::string fps = ffmpeg_number(input_stream.nominal_fps);
    const std::string output_path = path.string();
    std::vector<std::string> arguments{options.executable,
                                       "-hide_banner",
                                       "-loglevel",
                                       "error",
                                       "-nostdin",
                                       "-n",
                                       "-f",
                                       "rawvideo",
                                       "-pixel_format",
                                       pixel_format,
                                       "-video_size",
                                       video_size,
                                       "-framerate",
                                       fps,
                                       "-i",
                                       "pipe:0",
                                       "-an",
                                       "-c:v",
                                       options.codec};
    if (options.codec == "libx264") {
      arguments.push_back("-preset");
      arguments.push_back("ultrafast");
      arguments.push_back("-tune");
      arguments.push_back("zerolatency");
    }
    arguments.insert(arguments.end(), {"-pix_fmt", "yuv420p", "-f", "matroska", output_path});
    std::vector<char *> argv;
    argv.reserve(arguments.size() + 1);
    for (auto &argument : arguments) {
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
      throw_system_error("forking FFmpeg camera encoder", saved_errno);
    }
    if (pid == 0) {
      close_fd(input_pipe[1]);
      close_fd(exec_pipe[0]);
      if (::setpgid(0, 0) != 0) {
        const int saved_errno = errno;
        [[maybe_unused]] const auto reported =
            ::write(exec_pipe[1], &saved_errno, sizeof(saved_errno));
        _exit(127);
      }
      if (input_pipe[0] != STDIN_FILENO) {
        if (::dup2(input_pipe[0], STDIN_FILENO) < 0) {
          const int saved_errno = errno;
          [[maybe_unused]] const auto reported =
              ::write(exec_pipe[1], &saved_errno, sizeof(saved_errno));
          _exit(127);
        }
        close_fd(input_pipe[0]);
      }
      ::execvp(argv[0], argv.data());
      const int saved_errno = errno;
      [[maybe_unused]] const auto reported =
          ::write(exec_pipe[1], &saved_errno, sizeof(saved_errno));
      _exit(127);
    }

    close_fd(input_pipe[0]);
    close_fd(exec_pipe[1]);
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
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
      if (exec_bytes == static_cast<ssize_t>(sizeof(exec_error))) {
        throw_system_error("executing FFmpeg camera encoder", exec_error);
      }
      throw_system_error("reading FFmpeg exec status", exec_bytes < 0 ? read_error : EIO);
    }

    input_fd = input_pipe[1];
    child_pid = pid;
    stream = input_stream;
  }

  void verify() {
    if (child_pid > 0 || input_fd >= 0) {
      throw std::logic_error("cannot verify an open FFmpeg camera encoder");
    }

    const auto temporary_parent = std::filesystem::temp_directory_path();
    std::string directory_template = (temporary_parent / "lingtu-camera-encoder-XXXXXX").string();
    std::vector<char> mutable_template(directory_template.begin(), directory_template.end());
    mutable_template.push_back('\0');
    char *directory = ::mkdtemp(mutable_template.data());
    if (directory == nullptr) {
      throw_system_error("creating FFmpeg camera encoder preflight directory");
    }

    const std::filesystem::path probe_directory(directory);
    const auto output = probe_directory / "probe.mkv.tmp";
    std::error_code cleanup_error;
    try {
      CameraStreamDescription probe_stream;
      probe_stream.width = 16;
      probe_stream.height = 16;
      probe_stream.stride = 16 * 3;
      probe_stream.encoding = "bgr8";
      probe_stream.nominal_fps = 30.0;

      CameraFrame probe_frame;
      probe_frame.sequence = 1;
      probe_frame.timestamp_ns = 1;
      probe_frame.width = probe_stream.width;
      probe_frame.height = probe_stream.height;
      probe_frame.stride = probe_stream.stride;
      probe_frame.encoding = probe_stream.encoding;
      probe_frame.payload.assign(static_cast<std::size_t>(probe_frame.stride) * probe_frame.height,
                                 std::byte{0});

      begin(output, probe_stream);
      write(probe_frame);
      finish();
      if (!std::filesystem::is_regular_file(output) || std::filesystem::file_size(output) == 0) {
        throw std::runtime_error("FFmpeg camera encoder preflight produced no MKV output");
      }
    } catch (const std::exception &error) {
      abort();
      std::filesystem::remove_all(probe_directory, cleanup_error);
      throw std::runtime_error("FFmpeg camera encoder preflight failed for executable '" +
                               options.executable + "' and codec '" + options.codec +
                               "': " + error.what());
    }
    std::filesystem::remove_all(probe_directory, cleanup_error);
  }

  void write(const CameraFrame &frame) {
    check_running();
    if (frame.width != stream.width || frame.height != stream.height ||
        frame.stride != stream.stride || frame.encoding != stream.encoding ||
        frame.payload.size() != static_cast<std::size_t>(frame.stride) * frame.height) {
      throw std::runtime_error("camera frame does not match the open FFmpeg stream");
    }
    const auto visible_row_size = static_cast<std::size_t>(frame.width) * 3u;
    try {
      if (frame.stride == visible_row_size) {
        write_all(input_fd, frame.payload.data(), frame.payload.size());
      } else {
        for (std::uint32_t row = 0; row < frame.height; ++row) {
          write_all(input_fd, frame.payload.data() + static_cast<std::size_t>(row) * frame.stride,
                    visible_row_size);
        }
      }
    } catch (...) {
      abort();
      throw;
    }
  }

  void finish() {
    if (child_pid <= 0 || input_fd < 0) {
      throw std::logic_error("FFmpeg camera segment is not open");
    }
    close_fd(input_fd);
    int status = 0;
    pid_t result;
    do {
      result = ::waitpid(child_pid, &status, 0);
    } while (result < 0 && errno == EINTR);
    child_pid = -1;
    stream = {};
    if (result < 0) {
      throw_system_error("waiting for FFmpeg camera encoder");
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      throw std::runtime_error("FFmpeg camera encoder failed with " + child_status(status));
    }
  }

  void abort() noexcept {
    close_fd(input_fd);
    if (child_pid <= 0) {
      stream = {};
      return;
    }
    ::kill(child_pid, SIGTERM);
    int status = 0;
    for (int attempt = 0; attempt < 100; ++attempt) {
      const auto result = ::waitpid(child_pid, &status, WNOHANG);
      if (result == child_pid || (result < 0 && errno == ECHILD)) {
        child_pid = -1;
        stream = {};
        return;
      }
      if (result < 0 && errno != EINTR) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ::kill(child_pid, SIGKILL);
    while (::waitpid(child_pid, &status, 0) < 0 && errno == EINTR) {}
    child_pid = -1;
    stream = {};
  }

  FfmpegSegmentEncoderOptions options;
  int input_fd{-1};
  pid_t child_pid{-1};
  CameraStreamDescription stream;
};

FfmpegSegmentEncoder::FfmpegSegmentEncoder(FfmpegSegmentEncoderOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

FfmpegSegmentEncoder::~FfmpegSegmentEncoder() = default;

std::string FfmpegSegmentEncoder::codec() const {
  return impl_->options.codec;
}

std::string FfmpegSegmentEncoder::container_extension() const {
  return ".mkv";
}

void FfmpegSegmentEncoder::verify() {
  impl_->verify();
}

void FfmpegSegmentEncoder::begin(const std::filesystem::path &temporary_path,
                                 const CameraStreamDescription &stream) {
  impl_->begin(temporary_path, stream);
}

void FfmpegSegmentEncoder::write(const CameraFrame &frame) {
  impl_->write(frame);
}

void FfmpegSegmentEncoder::finish() {
  impl_->finish();
}

void FfmpegSegmentEncoder::abort() noexcept {
  impl_->abort();
}

}  // namespace lingtu::recording
