#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace lingtu::drivers::camera::shm {
struct ColorFrame;
}

namespace lingtu::drivers::camera::video {

class Cancelled final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct EncoderOptions {
  std::string executable{"ffmpeg"};
  std::string codec{"libx264"};
  unsigned fps{15};
  unsigned bitrate_kbps{1500};
  unsigned gop{15};
  std::chrono::milliseconds write_timeout{500};
  std::function<bool()> stop_requested;
};

class Encoder {
 public:
  explicit Encoder(EncoderOptions options = {});
  ~Encoder();

  Encoder(const Encoder&) = delete;
  Encoder& operator=(const Encoder&) = delete;
  Encoder(Encoder&&) = delete;
  Encoder& operator=(Encoder&&) = delete;

  void begin(const shm::ColorFrame& frame, int output_fd);
  void write(const shm::ColorFrame& frame);
  void finish();
  void abort() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::drivers::camera::video
