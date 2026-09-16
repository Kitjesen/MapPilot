#include <librealsense2/rs.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "../../native/camera_record.hpp"
#include "intrinsics.hpp"

namespace record = lingtu::drivers::camera::record;

namespace {
struct Options {
  int color_width{640}, color_height{480}, color_fps{30};
  int depth_width{640}, depth_height{480}, depth_fps{30};
  int timeout_ms{1000}, startup_timeout_ms{10000};
  int max_frames{0};
  std::string serial;
  bool list{false};
};

Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--list-devices") { o.list = true; continue; }
    if (arg == "--help") {
      std::cerr << "realsense_capture [--serial-number SN] [--list-devices]\n"
                   "  [--color-width N --color-height N --color-fps N]\n"
                   "  [--depth-width N --depth-height N --depth-fps N]\n"
                   "  [--timeout-ms N --startup-frame-timeout-ms N --max-frames N]\n";
      std::exit(0);
    }
    if (++i >= argc) throw std::runtime_error("missing value for " + arg);
    const std::string value = argv[i];
    if (arg == "--serial-number") { o.serial = value; continue; }
    std::size_t used = 0;
    const int n = std::stoi(value, &used);
    if (used != value.size() || n < 0 || (n == 0 && arg != "--max-frames"))
      throw std::runtime_error("invalid value for " + arg);
    if (arg == "--color-width") o.color_width = n;
    else if (arg == "--color-height") o.color_height = n;
    else if (arg == "--color-fps") o.color_fps = n;
    else if (arg == "--depth-width") o.depth_width = n;
    else if (arg == "--depth-height") o.depth_height = n;
    else if (arg == "--depth-fps") o.depth_fps = n;
    else if (arg == "--timeout-ms") o.timeout_ms = n;
    else if (arg == "--startup-frame-timeout-ms") o.startup_timeout_ms = n;
    else if (arg == "--max-frames") o.max_frames = n;
    else throw std::runtime_error("unknown argument: " + arg);
  }
  if (!record::isValidRecordTimeoutMs(o.timeout_ms) ||
      !record::isValidRecordTimeoutMs(o.startup_timeout_ms))
    throw std::runtime_error("timeout must be between 1 and 60000 ms");
  return o;
}

void emit(const record::RecordHeader& h, const void* data = nullptr) {
  const auto valid = record::validateRecordHeader(h);
  if (valid != record::RecordValidation::kValid)
    throw std::runtime_error(record::recordValidationReason(valid));
  const auto payload = record::validateRecordPayloadPointer(h, data);
  if (payload != record::RecordValidation::kValid)
    throw std::runtime_error(record::recordValidationReason(payload));
  std::cout.write(reinterpret_cast<const char*>(&h), sizeof(h));
  if (h.payload_size) std::cout.write(static_cast<const char*>(data), h.payload_size);
  if (!std::cout) throw std::runtime_error("camera_record_output_failed");
}
}

int main(int argc, char** argv) try {
  const auto o = parse(argc, argv);
  rs2::context context;
  const auto devices = context.query_devices();
  if (o.list) {
    for (const auto& device : devices)
      std::cout << device.get_info(RS2_CAMERA_INFO_NAME) << '\t'
                << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << '\n';
    return 0;
  }
  std::string serial = o.serial;
  int matches = 0;
  for (const auto& device : devices) {
    const std::string sn = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    if (!o.serial.empty() && sn != o.serial) continue;
    const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
    if (name.find("D435I") == std::string::npos && name.find("D435i") == std::string::npos)
      continue;
    serial = sn;
    ++matches;
  }
  if (matches != 1)
    throw std::runtime_error(matches == 0 ? "D435i not connected" : "multiple D435i devices: select --serial-number");
  rs2::config config;
  config.enable_device(serial);
  config.enable_stream(RS2_STREAM_COLOR, o.color_width, o.color_height, RS2_FORMAT_RGB8, o.color_fps);
  config.enable_stream(RS2_STREAM_DEPTH, o.depth_width, o.depth_height, RS2_FORMAT_Z16, o.depth_fps);
  rs2::pipeline pipeline(context);
  pipeline.start(config);
  rs2::align align(RS2_STREAM_COLOR);
  std::vector<std::uint8_t> rgb;
  std::vector<std::uint16_t> mm;
  for (int count = 0; o.max_frames == 0 || count < o.max_frames; ++count) {
    auto frames = pipeline.wait_for_frames(count == 0 ? o.startup_timeout_ms : o.timeout_ms);
    // Host reception time in Unix seconds; never relabel the device uptime as UTC.
    const double stamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frames = align.process(frames);
    const auto color = frames.get_color_frame();
    const auto depth = frames.get_depth_frame();
    if (!color || !depth) throw std::runtime_error("incomplete RGB-D frameset");
    const auto intr = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    if (!lingtu::drivers::realsense::recordCompatibleIntrinsics(intr)) {
      std::cerr << "color distortion model=" << rs2_distortion_to_string(intr.model)
                << " coefficients=" << intr.coeffs[0] << ',' << intr.coeffs[1]
                << ',' << intr.coeffs[2] << ',' << intr.coeffs[3] << ',' << intr.coeffs[4] << '\n';
      throw std::runtime_error("color distortion model not supported by camera record");
    }
    auto info = record::makeRecordHeader(record::kKindIntrinsics);
    info.width = intr.width; info.height = intr.height; info.timestamp_s = stamp;
    info.fx = intr.fx; info.fy = intr.fy; info.cx = intr.ppx; info.cy = intr.ppy;
    info.dist_k1 = intr.coeffs[0]; info.dist_k2 = intr.coeffs[1];
    info.dist_p1 = intr.coeffs[2]; info.dist_p2 = intr.coeffs[3]; info.dist_k3 = intr.coeffs[4];
    if (depth.get_width() != intr.width || depth.get_height() != intr.height)
      throw std::runtime_error("aligned depth dimensions do not match color");
    const auto pixels = static_cast<std::size_t>(intr.width) * intr.height;
    rgb.resize(pixels * 3); mm.resize(pixels);
    const double scale = depth.get_units();
    if (!record::isValidSourceDepthScale(scale)) throw std::runtime_error("invalid depth units");
    for (int y = 0; y < intr.height; ++y) {
      std::memcpy(rgb.data() + y * intr.width * 3,
          static_cast<const std::uint8_t*>(color.get_data()) + y * color.get_stride_in_bytes(), intr.width * 3);
      const auto* row = static_cast<const std::uint8_t*>(depth.get_data()) + y * depth.get_stride_in_bytes();
      for (int x = 0; x < intr.width; ++x)
        mm[y * intr.width + x] = record::normalizeDepthSampleMillimeters(row + x * 2, scale);
    }
    emit(info);
    auto image = record::makeRecordHeader(record::kKindColor);
    image.width = intr.width; image.height = intr.height; image.timestamp_s = stamp;
    image.channels = 3; image.format = record::kFormatRgb8; image.payload_size = rgb.size();
    emit(image, rgb.data());
    image = record::makeRecordHeader(record::kKindDepth);
    image.width = intr.width; image.height = intr.height; image.timestamp_s = stamp;
    image.channels = 1; image.format = record::kFormatDepthU16; image.payload_size = mm.size() * 2;
    emit(image, mm.data());
    std::cout.flush();
    if (!std::cout) throw std::runtime_error("camera_record_output_failed");
  }
  pipeline.stop();
  return 0;
} catch (const std::exception& error) {
  std::cerr << "realsense_capture: " << error.what() << '\n';
  return 1;
}
