#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "camera.hpp"

namespace {

using lingtu::drivers::orbbec::Camera;
using lingtu::drivers::orbbec::CameraConfig;
using lingtu::drivers::orbbec::Frame;
using lingtu::drivers::orbbec::Intrinsics;
using lingtu::drivers::orbbec::PixelFormat;

constexpr uint16_t kVersion = 1;
constexpr uint16_t kKindIntrinsics = 1;
constexpr uint16_t kKindColor = 2;
constexpr uint16_t kKindDepth = 3;

#pragma pack(push, 1)
struct RecordHeader {
  char magic[4];
  uint16_t version;
  uint16_t kind;
  uint32_t width;
  uint32_t height;
  uint32_t channels;
  uint32_t format;
  double timestamp_s;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scale_m;
  uint32_t payload_size;
};
#pragma pack(pop)

static_assert(sizeof(RecordHeader) == 76, "unexpected Orbbec stream header size");

struct Options {
  std::string sdk_config_path;
  std::string serial_number;
  std::string uid;
  int product_id = 0;
  uint32_t device_index = 0;
  bool use_device_index = false;
  uint32_t connect_timeout_ms = 10000;
  uint32_t color_width = 640;
  uint32_t color_height = 480;
  uint32_t color_fps = 30;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;
  uint32_t timeout_ms = 1000;
  uint64_t max_frames = 0;
  bool enable_frame_sync = false;
  bool self_test = false;
};

double now_s() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

uint32_t parse_u32(const char *value, uint32_t fallback) {
  try {
    return static_cast<uint32_t>(std::stoul(value, nullptr, 0));
  } catch (...) {
    return fallback;
  }
}

uint64_t parse_u64(const char *value, uint64_t fallback) {
  try {
    return static_cast<uint64_t>(std::stoull(value));
  } catch (...) {
    return fallback;
  }
}

Options parse_args(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto next = [&]() -> const char * {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--width") {
      const uint32_t width = parse_u32(next(), options.color_width);
      options.color_width = width;
      options.depth_width = width;
    } else if (arg == "--height") {
      const uint32_t height = parse_u32(next(), options.color_height);
      options.color_height = height;
      options.depth_height = height;
    } else if (arg == "--fps") {
      const uint32_t fps = parse_u32(next(), options.color_fps);
      options.color_fps = fps;
      options.depth_fps = fps;
    } else if (arg == "--color-width") {
      options.color_width = parse_u32(next(), options.color_width);
    } else if (arg == "--color-height") {
      options.color_height = parse_u32(next(), options.color_height);
    } else if (arg == "--color-fps") {
      options.color_fps = parse_u32(next(), options.color_fps);
    } else if (arg == "--depth-width") {
      options.depth_width = parse_u32(next(), options.depth_width);
    } else if (arg == "--depth-height") {
      options.depth_height = parse_u32(next(), options.depth_height);
    } else if (arg == "--depth-fps") {
      options.depth_fps = parse_u32(next(), options.depth_fps);
    } else if (arg == "--timeout-ms") {
      options.timeout_ms = parse_u32(next(), options.timeout_ms);
    } else if (arg == "--connect-timeout-ms") {
      options.connect_timeout_ms = parse_u32(next(), options.connect_timeout_ms);
    } else if (arg == "--max-frames") {
      options.max_frames = parse_u64(next(), options.max_frames);
    } else if (arg == "--serial-number") {
      options.serial_number = next();
    } else if (arg == "--uid" || arg == "--usb-port") {
      options.uid = next();
    } else if (arg == "--product-id") {
      options.product_id = static_cast<int>(parse_u32(next(), 0));
    } else if (arg == "--device-index") {
      options.device_index = parse_u32(next(), options.device_index);
      options.use_device_index = true;
    } else if (arg == "--sdk-config") {
      options.sdk_config_path = next();
    } else if (arg == "--enable-frame-sync") {
      options.enable_frame_sync = true;
    } else if (arg == "--self-test") {
      options.self_test = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cerr
          << "usage: orbbec_capture [--width N] [--height N] [--fps N]\n"
          << "                            [--color-width N] [--color-height N] [--color-fps N]\n"
          << "                            [--depth-width N] [--depth-height N] [--depth-fps N]\n"
          << "                            [--serial-number SN] [--uid UID|--usb-port UID]\n"
          << "                            [--product-id PID] [--device-index N]\n"
          << "                            [--connect-timeout-ms N] [--timeout-ms N]\n"
          << "                            [--max-frames N] [--enable-frame-sync]\n";
      std::exit(0);
    }
  }
  return options;
}

CameraConfig camera_config(const Options &options) {
  CameraConfig config;
  config.sdk_config_path = options.sdk_config_path;
  config.serial_number = options.serial_number;
  config.uid = options.uid;
  config.product_id = options.product_id;
  config.device_index = options.device_index;
  config.use_device_index = options.use_device_index;
  config.connect_timeout_ms = options.connect_timeout_ms;
  config.color.width = options.color_width;
  config.color.height = options.color_height;
  config.color.fps = options.color_fps;
  config.depth.width = options.depth_width;
  config.depth.height = options.depth_height;
  config.depth.fps = options.depth_fps;
  config.enable_frame_sync = options.enable_frame_sync;
  return config;
}

void write_record(
    uint16_t kind,
    uint32_t width,
    uint32_t height,
    uint32_t channels,
    PixelFormat format,
    const void *payload,
    uint32_t payload_size,
    double fx = 0.0,
    double fy = 0.0,
    double cx = 0.0,
    double cy = 0.0,
    double depth_scale_m = 0.001) {
  RecordHeader header{};
  std::memcpy(header.magic, "LTOB", 4);
  header.version = kVersion;
  header.kind = kind;
  header.width = width;
  header.height = height;
  header.channels = channels;
  header.format = static_cast<uint32_t>(format);
  header.timestamp_s = now_s();
  header.fx = fx;
  header.fy = fy;
  header.cx = cx;
  header.cy = cy;
  header.depth_scale_m = depth_scale_m;
  header.payload_size = payload_size;

  std::cout.write(reinterpret_cast<const char *>(&header), sizeof(header));
  if (payload_size > 0 && payload != nullptr) {
    std::cout.write(reinterpret_cast<const char *>(payload), payload_size);
  }
  std::cout.flush();
}

void emit_intrinsics_record(const Intrinsics &intrinsics) {
  write_record(
      kKindIntrinsics,
      intrinsics.width,
      intrinsics.height,
      0,
      PixelFormat::kUnknown,
      nullptr,
      0,
      intrinsics.fx,
      intrinsics.fy,
      intrinsics.cx,
      intrinsics.cy,
      intrinsics.depth_scale_m);
}

void emit_color_record(const Frame &color) {
  if (!color.valid()) {
    return;
  }
  write_record(
      kKindColor,
      color.width,
      color.height,
      color.channels,
      color.format,
      color.data,
      color.data_size);
}

void emit_depth_record(const Frame &depth, double depth_scale_m) {
  if (!depth.valid()) {
    return;
  }
  write_record(
      kKindDepth,
      depth.width,
      depth.height,
      depth.channels,
      depth.format,
      depth.data,
      depth.data_size,
      0.0,
      0.0,
      0.0,
      0.0,
      depth_scale_m);
}

void emit_self_test() {
  const uint8_t color[] = {
      255, 0, 0, 0, 255, 0,
      0, 0, 255, 255, 255, 255,
  };
  const uint16_t depth[] = {1000, 1100, 1200, 1300};
  write_record(
      kKindIntrinsics,
      2,
      2,
      0,
      PixelFormat::kUnknown,
      nullptr,
      0,
      500.0,
      500.0,
      1.0,
      1.0,
      0.001);
  write_record(kKindColor, 2, 2, 3, PixelFormat::kRgb8, color, sizeof(color));
  write_record(kKindDepth, 2, 2, 1, PixelFormat::kDepthU16, depth, sizeof(depth));
}

}  // namespace

int main(int argc, char **argv) {
#ifdef _WIN32
  _setmode(_fileno(stdout), _O_BINARY);
#endif

  try {
    const Options options = parse_args(argc, argv);
    if (options.self_test) {
      emit_self_test();
      return 0;
    }

    Camera camera;
    camera.connect(camera_config(options));

    double depth_scale_m = 0.001;
    bool emitted_intrinsics = false;
    uint64_t frame_count = 0;

    while (options.max_frames == 0 || frame_count < options.max_frames) {
      auto frames = camera.read(options.timeout_ms);
      if (!frames.valid()) {
        continue;
      }

      if (frames.depth.valid()) {
        depth_scale_m = frames.depth_scale_m;
      }

      if (!emitted_intrinsics) {
        emit_intrinsics_record(camera.intrinsics(depth_scale_m));
        emitted_intrinsics = true;
      }

      emit_color_record(frames.color);
      emit_depth_record(frames.depth, depth_scale_m);

      ++frame_count;
    }

    camera.disconnect();
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "orbbec_capture: " << exc.what() << std::endl;
    return 1;
  }
}
