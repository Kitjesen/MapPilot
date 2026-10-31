#include "shm_frame_ring.hpp"

#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace shm = lingtu::drivers::camera::shm;

namespace {

std::uint64_t positive(const std::string& text, const char* field) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument(std::string(field) + " must be an unsigned integer");
  }
  const auto value = std::stoull(text);
  if (value == 0) {
    throw std::invalid_argument(std::string(field) + " must be positive");
  }
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: video_fixture /posix_shm_name\n";
    return 2;
  }
  const std::string name = argv[1];
  ::shm_unlink(name.c_str());
  std::unique_ptr<shm::FrameWriter> writer;
  std::uint64_t generation = 0;
  std::cout << "ready " << name << '\n' << std::flush;

  std::string line;
  try {
    while (std::getline(std::cin, line)) {
      std::istringstream command(line);
      std::string action;
      command >> action;
      if (action == "quit") {
        std::cout << "bye\n" << std::flush;
        writer.reset();
        ::shm_unlink(name.c_str());
        return 0;
      }
      if (action != "publish") {
        throw std::invalid_argument("expected: publish GENERATION COUNT AGE_MS rgb8|bgr8");
      }
      std::string generation_text;
      std::string count_text;
      std::string age_text;
      std::string encoding;
      std::string extra;
      if (!(command >> generation_text >> count_text >> age_text >> encoding) ||
          (command >> extra) || (encoding != "rgb8" && encoding != "bgr8")) {
        throw std::invalid_argument("expected: publish GENERATION COUNT AGE_MS rgb8|bgr8");
      }
      const auto requested_generation = positive(generation_text, "generation");
      const auto count = positive(count_text, "count");
      const auto age_ms = std::stoull(age_text);
      if (age_text.empty() || age_text.find_first_not_of("0123456789") != std::string::npos ||
          count > 10000 || age_ms > 60000) {
        throw std::invalid_argument("fixture count or age is out of range");
      }
      if (!writer || generation != requested_generation) {
        writer.reset();
        writer = std::make_unique<shm::FrameWriter>(
            shm::WriterConfig{name, 2, 2048, 0600, true, requested_generation});
        generation = requested_generation;
      }

      constexpr std::uint32_t width = 16;
      constexpr std::uint32_t height = 16;
      constexpr std::uint32_t stride = 52;
      std::vector<std::uint8_t> pixels(static_cast<std::size_t>(stride) * height, 0xa5);
      for (std::uint32_t row = 0; row < height; ++row) {
        for (std::uint32_t column = 0; column < width; ++column) {
          auto* pixel = pixels.data() + static_cast<std::size_t>(row) * stride + column * 3u;
          if (encoding == "rgb8") {
            pixel[0] = 240;
            pixel[1] = 16;
            pixel[2] = 16;
          } else {
            pixel[0] = 16;
            pixel[1] = 16;
            pixel[2] = 240;
          }
        }
      }
      shm::FrameMetadata metadata;
      metadata.stream_kind = shm::StreamKind::kColor;
      metadata.width = width;
      metadata.height = height;
      metadata.stride = stride;
      metadata.encoding = encoding;
      metadata.frame_id = "video_fixture";
      const std::uint64_t age_ns = age_ms * 1000000ULL;
      std::uint64_t first = 0;
      std::uint64_t last = 0;
      for (std::uint64_t index = 0; index < count; ++index) {
        const auto now = shm::unixTimeNs();
        metadata.timestamp_ns = now > age_ns ? now - age_ns : 1;
        last = writer->publish(metadata, pixels.data(), pixels.size());
        if (first == 0) {
          first = last;
        }
      }
      std::cout << "published " << generation << ' ' << first << ' ' << last << ' '
                << count << '\n' << std::flush;
    }
  } catch (const std::exception& error) {
    std::cerr << "video fixture failed: " << error.what() << '\n';
    writer.reset();
    ::shm_unlink(name.c_str());
    return 1;
  }

  writer.reset();
  ::shm_unlink(name.c_str());
  return 0;
}
