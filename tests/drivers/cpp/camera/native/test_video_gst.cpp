#include <chrono>
#include <cstdint>
#include <gst/gst.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include "../../../../../src/drivers/real/camera/native/shm_reader.hpp"
#include "../../../../../src/drivers/real/camera/native/video/gst.hpp"

namespace camera_shm = lingtu::drivers::camera::shm;
namespace camera_video = lingtu::drivers::camera::video;

namespace {
camera_shm::ColorFrame frame(std::uint64_t timestamp, std::uint64_t sequence = 1) {
  camera_shm::ColorFrame result;
  result.source_generation = 7;
  result.sequence = sequence;
  result.timestamp_ns = timestamp;
  result.width = 18;
  result.height = 16;
  result.stride = 18 * 3 + 8;
  result.encoding = "rgb8";
  result.frame_id = "camera";
  result.payload.assign(static_cast<std::size_t>(result.stride) * result.height, 127);
  return result;
}

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
}  // namespace

int main() {
  gst_init(nullptr, nullptr);
  GstElementFactory *x264 = gst_element_factory_find("x264enc");
  GstElementFactory *parse = gst_element_factory_find("h264parse");
  const bool available = x264 && parse;
  if (x264)
    gst_object_unref(x264);
  if (parse)
    gst_object_unref(parse);
  if (!available) {
    std::cout << "SKIP: requires x264enc and h264parse\n";
    return 77;
  }

  char output_path[] = "/tmp/lingtu-video-gst-XXXXXX";
  const int output_fd = ::mkstemp(output_path);
  if (output_fd < 0)
    return 1;
  ::unlink(output_path);

  try {
    camera_video::GstOptions invalid_whep;
    invalid_whep.listen_uri = "http://127.0.0.1:9090/not-a-listen-address";
    bool rejected_path = false;
    try {
      camera_video::GstSession session(invalid_whep, frame(1000000000));
    } catch (const std::invalid_argument &) {
      rejected_path = true;
    }
    require(rejected_path, "WHEP listen URI path must be rejected");

    invalid_whep.listen_uri = "https://127.0.0.1:9090";
    bool rejected_tls = false;
    try {
      camera_video::GstSession session(invalid_whep, frame(1000000000));
    } catch (const std::invalid_argument &) {
      rejected_tls = true;
    }
    require(rejected_tls, "WHEP listener must not advertise unsupported TLS");

    invalid_whep.listen_uri = "http://127.0.0.1:9090";
    for (const std::string uri : {"https://stun.example:3478", "stun://stun.example",
                                  "stun://stun.example:3478/path"}) {
      invalid_whep.stun_server = uri;
      bool rejected_stun = false;
      try {
        camera_video::GstSession session(invalid_whep, frame(1000000000));
      } catch (const std::invalid_argument &) {
        rejected_stun = true;
      }
      require(rejected_stun, "unsupported STUN URI must be rejected: " + uri);
    }

    camera_video::GstOptions bad;
    bad.output = camera_video::GstOutput::H264Fd;
    bad.output_fd = output_fd;
    bad.encoder_factory = "lingtu_missing_encoder";
    bool rejected_missing = false;
    try {
      camera_video::GstSession session(bad, frame(1000000000));
    } catch (const std::exception &) {
      rejected_missing = true;
    }
    require(rejected_missing, "missing exact encoder must not fall back");

    camera_video::GstOptions options;
    options.output = camera_video::GstOutput::H264Fd;
    options.output_fd = output_fd;
    options.encoder_factory = "x264enc";
    options.fps = 15;
    options.min_bitrate = 100000;
    options.start_bitrate = 300000;
    options.max_bitrate = 800000;
    camera_video::GstSession session(options, frame(1000000000));

    require(session.push(frame(1000000000, 1)), "first frame should be accepted");
    require(!session.push(frame(1000000000, 2)), "duplicate timestamp should be rejected");
    require(session.push(frame(1066666666, 2)), "newer frame should be accepted");
    require(session.push(frame(1133333333, 3)), "third frame should be accepted");
    require(!session.push(frame(999999999, 4)), "older timestamp should be rejected");
    auto metrics = session.metrics();
    // appsrc updates current-level-buffers from its streaming thread. During
    // a concurrent dequeue the observable counter can briefly include the
    // buffer being handed downstream; require it to settle at the configured
    // one-buffer bound rather than treating that accounting instant as queued
    // memory growth.
    for (unsigned attempt = 0; metrics.queued_raw_frames > 1 && attempt < 100; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      metrics = session.metrics();
    }
    require(metrics.accepted_frames == 3, "accepted frame count is wrong");
    require(metrics.rejected_timestamps == 2, "rejected timestamp count is wrong");
    require(metrics.last_pts_ns == 133333333, "H.264 file timestamp origin must stay unchanged");
    require(metrics.queued_raw_frames <= 1,
            "raw queue must stay bounded to one frame, observed " +
                std::to_string(metrics.queued_raw_frames));
    require(metrics.selected_encoder == "x264enc", "exact x264 encoder was not selected");
    require(metrics.transport == "h264-fd", "transport identity is wrong");
    require(camera_video::GstSession::endpoint(options).empty(),
            "fixed H.264 output must not report a WHEP endpoint");
    session.stop();
    struct stat info{};
    require(::fstat(output_fd, &info) == 0 && info.st_size > 0, "H.264 encoder produced no bytes");

    GstElementFactory *whep = gst_element_factory_find("whepserversink");
    if (whep) {
      gst_object_unref(whep);
      camera_video::GstOptions live_options;
      live_options.encoder_factory = "x264enc";
      live_options.listen_uri = "http://127.0.0.1:0";
      const auto before_start = frame(camera_shm::unixTimeNs() - 5 * GST_SECOND);
      camera_video::GstSession live_session(live_options, before_start);
      require(live_session.push(frame(camera_shm::unixTimeNs(), 2)),
              "fresh WHEP frame should be accepted after delayed initialization");
      const auto live_metrics = live_session.metrics();
      require(live_metrics.last_pts_ns <= live_metrics.last_running_time_ns,
              "WHEP capture timestamps must not be ahead of the running pipeline clock");
      live_session.stop();
    } else {
      std::cout << "SKIP: WHEP clock alignment requires whepserversink\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    ::close(output_fd);
    return 1;
  }
  ::close(output_fd);
  return 0;
}
