#pragma once

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
  std::string listen_uri{"http://127.0.0.1:9090"};
  std::string stun_server;
  std::string encoder_factory{"nvv4l2h264enc"};
  unsigned fps{20};
  unsigned min_bitrate{200000};
  unsigned start_bitrate{1500000};
  unsigned max_bitrate{4000000};
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
