#pragma once

#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace rerun {

struct Status {
  void exit_on_failure() const {}
};

struct Position3D {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct Vec3D {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct Color {
  std::uint8_t r{0};
  std::uint8_t g{0};
  std::uint8_t b{0};
};

struct Text {
  explicit Text(std::string value) : value(std::move(value)) {}
  std::string value;
};

struct Quaternion {
  static Quaternion from_xyzw(float x, float y, float z, float w) { return Quaternion{x, y, z, w}; }

  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float w{1.0F};
};

struct WidthHeight {
  WidthHeight(std::uint32_t width, std::uint32_t height) : width(width), height(height) {}

  std::uint32_t width{0};
  std::uint32_t height{0};
};

class Points3D {
 public:
  explicit Points3D(std::vector<Position3D> positions) : positions_(std::move(positions)) {}

  Points3D with_colors(std::vector<Color> colors) const {
    Points3D out(*this);
    out.colors_ = std::move(colors);
    return out;
  }

  Points3D with_colors(std::initializer_list<Color> colors) const {
    return with_colors(std::vector<Color>(colors));
  }

  Points3D with_labels(std::vector<Text> labels) const {
    Points3D out(*this);
    out.labels_ = std::move(labels);
    return out;
  }

 private:
  std::vector<Position3D> positions_;
  std::vector<Color> colors_;
  std::vector<Text> labels_;
};

class LineStrips3D {
 public:
  explicit LineStrips3D(std::vector<std::vector<Position3D>> strips) : strips_(std::move(strips)) {}

 private:
  std::vector<std::vector<Position3D>> strips_;
};

class Transform3D {
 public:
  Transform3D(Vec3D translation, Quaternion rotation)
      : translation_(translation), rotation_(rotation) {}

 private:
  Vec3D translation_;
  Quaternion rotation_;
};

class Image {
 public:
  static Image from_greyscale8(std::vector<std::uint8_t> pixels, WidthHeight size) {
    return Image(std::move(pixels), size);
  }

 private:
  Image(std::vector<std::uint8_t> pixels, WidthHeight size)
      : pixels_(std::move(pixels)), size_(size) {}

  std::vector<std::uint8_t> pixels_;
  WidthHeight size_;
};

class RecordingStream {
 public:
  explicit RecordingStream(std::string app_id) : app_id_(std::move(app_id)) {}

  Status save(const std::string &) { return {}; }
  Status spawn() { return {}; }
  Status connect_grpc() { return {}; }
  void flush_blocking() {}
  void set_time_sequence(std::string_view, std::int64_t) {}

  template <typename T>
  void log(const std::string &, const T &) {}

 private:
  std::string app_id_;
};

}  // namespace rerun
