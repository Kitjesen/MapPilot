#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include "lingtu/recording/camera_linux.hpp"

namespace {

namespace recording = lingtu::recording;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int run_ffmpeg_decode(const char *ffmpeg, const std::filesystem::path &input) {
  std::vector<std::string> arguments{
      ffmpeg,         "-hide_banner", "-loglevel", "error", "-nostdin", "-i",
      input.string(), "-frames:v",    "1",         "-f",    "null",     "-",
  };
  std::vector<char *> argv;
  argv.reserve(arguments.size() + 1);
  for (auto &argument : arguments) {
    argv.push_back(argument.data());
  }
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) {
    throw std::runtime_error("failed to fork FFmpeg decoder");
  }
  if (pid == 0) {
    ::execv(argv.front(), argv.data());
    _exit(127);
  }

  int status = 0;
  pid_t result;
  do {
    result = ::waitpid(pid, &status, 0);
  } while (result < 0 && errno == EINTR);
  if (result < 0) {
    throw std::runtime_error("failed to wait for FFmpeg decoder");
  }
  if (!WIFEXITED(status)) {
    throw std::runtime_error("FFmpeg decoder did not exit normally");
  }
  return WEXITSTATUS(status);
}

recording::CameraStreamDescription test_stream() {
  recording::CameraStreamDescription stream;
  stream.width = 16;
  stream.height = 16;
  stream.stride = 16 * 3;
  stream.encoding = "bgr8";
  stream.nominal_fps = 30.0;
  return stream;
}

recording::CameraFrame test_frame(std::uint64_t sequence, std::uint64_t timestamp_ns) {
  const auto stream = test_stream();
  recording::CameraFrame frame;
  frame.sequence = sequence;
  frame.timestamp_ns = timestamp_ns;
  frame.width = stream.width;
  frame.height = stream.height;
  frame.stride = stream.stride;
  frame.encoding = stream.encoding;
  frame.payload.resize(static_cast<std::size_t>(frame.stride) * frame.height);
  for (std::size_t index = 0; index < frame.payload.size(); ++index) {
    frame.payload[index] = std::byte{static_cast<unsigned char>(index % 251u)};
  }
  return frame;
}

void test_software_ffmpeg_writes_a_decodable_mkv(const char *ffmpeg) {
  const auto directory = std::filesystem::temp_directory_path() /
                         ("lingtu_camera_ffmpeg_software_" + std::to_string(::getpid()));
  const auto output = directory / "software.mkv.tmp";
  std::error_code cleanup_error;
  std::filesystem::remove_all(directory, cleanup_error);
  std::filesystem::create_directories(directory);

  try {
    recording::FfmpegSegmentEncoder encoder({ffmpeg, "libx264"});
    encoder.begin(output, test_stream());
    encoder.write(test_frame(1, 1'000'000'000));
    encoder.write(test_frame(2, 1'033'333'333));
    encoder.finish();

    require(std::filesystem::is_regular_file(output), "software FFmpeg did not write an MKV");
    require(std::filesystem::file_size(output) > 0, "software FFmpeg wrote an empty MKV");
    require(run_ffmpeg_decode(ffmpeg, output) == 0, "software FFmpeg could not decode its MKV");
  } catch (...) {
    std::filesystem::remove_all(directory, cleanup_error);
    throw;
  }
  std::filesystem::remove_all(directory, cleanup_error);
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    return 2;
  }
  test_software_ffmpeg_writes_a_decodable_mkv(argv[1]);
  return 0;
}
