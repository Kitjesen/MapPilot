#pragma once

#include <lingtu/settings.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace lingtu::drivers::camera::shm {
struct ColorFrame;
}

namespace lingtu::drivers::camera::video {

enum class GstOutput { Whep, H264Fd };

struct GstOptions {
  GstOutput output{GstOutput::Whep};
  int output_fd{-1};
  std::string listen_uri{lingtu::settings::camera::camera::video::listen};
  std::string stun_server{lingtu::settings::camera::camera::video::stun_server};
  std::string encoder_factory;
  unsigned fps{lingtu::settings::camera::camera::video::fps};
  unsigned min_bitrate{lingtu::settings::camera::camera::video::min_bitrate};
  unsigned start_bitrate{lingtu::settings::camera::camera::video::start_bitrate};
  unsigned max_bitrate{lingtu::settings::camera::camera::video::max_bitrate};
};

struct GstMetrics {
  std::uint64_t accepted_frames{0};
  std::uint64_t rejected_timestamps{0};
  std::uint64_t last_pts_ns{0};
  std::uint64_t last_running_time_ns{0};
  unsigned active_consumers{0};
  unsigned queued_raw_frames{0};
  std::string selected_encoder;
  std::string rtc_stats;
  std::string transport;
};

// One raw-color stream served through WHEP or written as fixed-rate Annex-B
// H.264. The camera remains owned by the native capture service; this class
// only accepts already-copied SHM frames.
class GstSession {
 public:
  GstSession(GstOptions options, const shm::ColorFrame &first_frame);
  ~GstSession();

  GstSession(const GstSession &) = delete;
  GstSession &operator=(const GstSession &) = delete;
  GstSession(GstSession &&) = delete;
  GstSession &operator=(GstSession &&) = delete;

  // Returns false for duplicate or regressing timestamps in this stream.
  // Queue pressure is handled by appsrc's one-frame downstream-leaky queue.
  bool push(const shm::ColorFrame &frame);
  void poll();
  GstMetrics metrics() const;
  void stop() noexcept;

  static std::string endpoint(const GstOptions &options);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::drivers::camera::video
