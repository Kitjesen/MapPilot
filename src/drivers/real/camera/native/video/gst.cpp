#include "gst.hpp"

#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "../shm_reader.hpp"

namespace lingtu::drivers::camera::video {
namespace {

class Object {
 public:
  Object() = default;
  explicit Object(GstObject *value) : value_(value) {}
  ~Object() { reset(); }
  Object(const Object &) = delete;
  Object &operator=(const Object &) = delete;
  Object(Object &&other) noexcept : value_(std::exchange(other.value_, nullptr)) {}
  Object &operator=(Object &&other) noexcept {
    if (this != &other) {
      reset();
      value_ = std::exchange(other.value_, nullptr);
    }
    return *this;
  }
  void reset(GstObject *value = nullptr) {
    if (value_)
      gst_object_unref(value_);
    value_ = value;
  }
  GstObject *get() const { return value_; }
  template <typename T>
  T *as() const {
    return reinterpret_cast<T *>(value_);
  }

 private:
  GstObject *value_{nullptr};
};

class GObjectRef {
 public:
  explicit GObjectRef(GObject *value) : value_(value) {}
  ~GObjectRef() {
    if (value_)
      g_object_unref(value_);
  }
  GObjectRef(const GObjectRef &) = delete;
  GObjectRef &operator=(const GObjectRef &) = delete;
  GObject *get() const { return value_; }

 private:
  GObject *value_;
};

std::string gst_error(const std::string &context, GError *error, gchar *debug) {
  std::ostringstream result;
  result << context;
  if (error && error->message)
    result << ": " << error->message;
  if (debug && *debug)
    result << " [" << debug << ']';
  if (error)
    g_error_free(error);
  g_free(debug);
  return result.str();
}

void validate(const GstOptions &options, const shm::ColorFrame &frame) {
  if (options.fps == 0 || options.fps > 120) {
    throw std::invalid_argument("GStreamer video fps must be in [1, 120]");
  }
  if (options.min_bitrate == 0 || options.min_bitrate > options.start_bitrate ||
      options.start_bitrate > options.max_bitrate) {
    throw std::invalid_argument("GStreamer video bitrate must satisfy 0 < min <= start <= max");
  }
  if (options.output == GstOutput::Whep && options.listen_uri.rfind("http://", 0) != 0)
    throw std::invalid_argument("GStreamer WHEP listener supports plain HTTP only");
  if (options.output == GstOutput::Whep) {
    GstUri *uri = gst_uri_from_string(options.listen_uri.c_str());
    gchar *path = uri ? gst_uri_get_path(uri) : nullptr;
    gchar *query = uri ? gst_uri_get_query_string(uri) : nullptr;
    const bool valid =
        uri && gst_uri_get_host(uri) && (!path || *path == '\0' || std::strcmp(path, "/") == 0) &&
        !query && !gst_uri_get_fragment(uri) && !gst_uri_get_userinfo(uri);
    g_free(path);
    g_free(query);
    if (uri)
      gst_uri_unref(uri);
    if (!valid)
      throw std::invalid_argument("GStreamer WHEP listen URI must contain only host and port");
    if (!options.stun_server.empty()) {
      GstUri *stun = gst_uri_from_string(options.stun_server.c_str());
      const char *scheme = stun ? gst_uri_get_scheme(stun) : nullptr;
      const char *host = stun ? gst_uri_get_host(stun) : nullptr;
      gchar *stun_path = stun ? gst_uri_get_path(stun) : nullptr;
      gchar *stun_query = stun ? gst_uri_get_query_string(stun) : nullptr;
      const bool valid_stun = stun && scheme && std::strcmp(scheme, "stun") == 0 && host &&
                              *host != '\0' && gst_uri_get_port(stun) > 0 &&
                              (!stun_path || *stun_path == '\0') && !stun_query &&
                              !gst_uri_get_fragment(stun) && !gst_uri_get_userinfo(stun);
      g_free(stun_path);
      g_free(stun_query);
      if (stun)
        gst_uri_unref(stun);
      if (!valid_stun)
        throw std::invalid_argument("GStreamer STUN server must have the form stun://host:port");
    }
  }
  if (options.output == GstOutput::H264Fd && options.output_fd < 0)
    throw std::invalid_argument("GStreamer H.264 output requires a valid file descriptor");
  if (frame.source_generation == 0 || frame.timestamp_ns == 0 || frame.width == 0 ||
      frame.height == 0 || (frame.encoding != "rgb8" && frame.encoding != "bgr8") ||
      frame.stride < frame.width * 3u ||
      frame.payload.size() != static_cast<std::size_t>(frame.stride) * frame.height) {
    throw std::invalid_argument("invalid color frame for GStreamer video");
  }
}

std::string factory_name(GstElement *element) {
  GstElementFactory *factory = gst_element_get_factory(element);
  return factory ? GST_OBJECT_NAME(factory) : std::string{};
}

}  // namespace

struct GstSession::Impl {
  struct CallbackState {
    explicit CallbackState(std::string required) : required_encoder(std::move(required)) {}
    std::mutex mutex;
    std::string required_encoder;
    unsigned active_consumers{0};
    std::string selected_encoder;
    std::string error;
  };

  explicit Impl(GstOptions input, const shm::ColorFrame &first)
      : options(std::move(input)),
        generation(first.source_generation),
        width(first.width),
        height(first.height),
        stride(first.stride),
        encoding(first.encoding),
        first_timestamp_ns(first.timestamp_ns),
        callback_state(std::make_shared<CallbackState>(options.encoder_factory)) {
    GError *init_error = nullptr;
    if (!gst_init_check(nullptr, nullptr, &init_error)) {
      throw std::runtime_error(gst_error("GStreamer initialization failed", init_error, nullptr));
    }
    validate(options, first);
    try {
      if (options.output == GstOutput::Whep)
        selectExactEncoder();
      buildSource(first);
      if (options.output == GstOutput::Whep)
        buildWhep();
      else
        buildH264();
      start();
    } catch (...) {
      stop();
      restoreRanks();
      throw;
    }
  }

  ~Impl() {
    stop();
    restoreRanks();
  }

  void selectExactEncoder() {
    GstElementFactory *selected = gst_element_factory_find(options.encoder_factory.c_str());
    if (!selected) {
      throw std::runtime_error("required GStreamer encoder is unavailable: " +
                               options.encoder_factory);
    }
    gst_object_unref(selected);

    GstCaps *h264 = gst_caps_from_string("video/x-h264");
    GList *encoders = gst_element_factory_list_get_elements(GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
                                                            GST_RANK_NONE);
    GList *h264_encoders = gst_element_factory_list_filter(encoders, h264, GST_PAD_SRC, FALSE);
    gst_caps_unref(h264);
    gst_plugin_feature_list_free(encoders);

    bool found = false;
    for (GList *item = h264_encoders; item; item = item->next) {
      auto *feature = GST_PLUGIN_FEATURE(item->data);
      const std::string name = GST_OBJECT_NAME(feature);
      ranks.emplace_back(name, gst_plugin_feature_get_rank(feature));
      if (name == options.encoder_factory) {
        found = true;
        gst_plugin_feature_set_rank(feature, GST_RANK_PRIMARY + 100);
      } else {
        gst_plugin_feature_set_rank(feature, GST_RANK_NONE);
      }
    }
    gst_plugin_feature_list_free(h264_encoders);
    if (!found) {
      restoreRanks();
      throw std::runtime_error("required encoder does not advertise H.264 output: " +
                               options.encoder_factory);
    }
  }

  void restoreRanks() noexcept {
    for (const auto &[name, rank] : ranks) {
      if (GstElementFactory *factory = gst_element_factory_find(name.c_str())) {
        gst_plugin_feature_set_rank(GST_PLUGIN_FEATURE(factory), rank);
        gst_object_unref(factory);
      }
    }
    ranks.clear();
  }

  void buildSource(const shm::ColorFrame &first) {
    pipeline.reset(GST_OBJECT(gst_pipeline_new("lingtu-camera-webrtc")));
    appsrc.reset(GST_OBJECT(gst_element_factory_make("appsrc", "camera")));
    queue.reset(GST_OBJECT(gst_element_factory_make("queue", "latest-raw-frame")));
    if (!pipeline.get() || !appsrc.get() || !queue.get())
      throw std::runtime_error("GStreamer video requires appsrc and queue");

    const char *format = first.encoding == "rgb8" ? "RGB" : "BGR";
    GstCaps *raw_caps = gst_caps_new_simple(
        "video/x-raw", "format", G_TYPE_STRING, format, "width", G_TYPE_INT,
        static_cast<int>(first.width), "height", G_TYPE_INT, static_cast<int>(first.height),
        "framerate", GST_TYPE_FRACTION, static_cast<int>(options.fps), 1, nullptr);
    g_object_set(appsrc.get(), "caps", raw_caps, "is-live", TRUE, "format", GST_FORMAT_TIME,
                 "do-timestamp", FALSE, "block", FALSE, "max-buffers", guint64{1}, nullptr);
    gst_util_set_object_arg(G_OBJECT(appsrc.get()), "leaky-type", "downstream");
    gst_caps_unref(raw_caps);

    g_object_set(queue.get(), "max-size-buffers", guint{1}, "max-size-bytes", guint{0},
                 "max-size-time", guint64{0}, "leaky", gint{2}, nullptr);
  }

  void addToPipeline(std::initializer_list<GstElement *> elements) {
    for (GstElement *element : elements) {
      if (!gst_bin_add(GST_BIN(pipeline.get()), element))
        throw std::runtime_error("failed to add element to GStreamer video pipeline");
      gst_object_ref(element);
    }
  }

  void buildWhep() {
    sink.reset(GST_OBJECT(gst_element_factory_make("whepserversink", "webrtc")));
    if (!sink.get())
      throw std::runtime_error("WHEP output requires gst-plugin-webrtc 0.15+ whepserversink");
    if (g_signal_lookup("encoder-setup", G_OBJECT_TYPE(sink.get())) == 0 ||
        !g_object_class_find_property(G_OBJECT_GET_CLASS(sink.get()), "stats")) {
      throw std::runtime_error("whepserversink is missing the required encoder and statistics API");
    }

    GstCaps *video_caps = gst_caps_from_string("video/x-h264");
    const gchar *stun_server = options.stun_server.empty() ? nullptr : options.stun_server.c_str();
    g_object_set(sink.get(), "video-caps", video_caps, "min-bitrate", options.min_bitrate,
                 "start-bitrate", options.start_bitrate, "max-bitrate", options.max_bitrate,
                 "stun-server", stun_server, nullptr);
    gst_caps_unref(video_caps);

    GObjectRef signaller(
        gst_child_proxy_get_child_by_name(GST_CHILD_PROXY(sink.get()), "signaller"));
    if (!signaller.get() ||
        !g_object_class_find_property(G_OBJECT_GET_CLASS(signaller.get()), "host-addr")) {
      throw std::runtime_error("whepserversink signaller does not expose host-addr");
    }
    // Both upstream objects have public STUN defaults; null disables them for LAN sessions.
    g_object_set(signaller.get(), "host-addr", options.listen_uri.c_str(), "stun-server",
                 stun_server, nullptr);

    connectSignal("encoder-setup", G_CALLBACK(onEncoderSetup));
    connectSignal("consumer-added", G_CALLBACK(onConsumerAdded));
    connectSignal("consumer-removed", G_CALLBACK(onConsumerRemoved));

    addToPipeline({GST_ELEMENT(appsrc.get()), GST_ELEMENT(queue.get()), GST_ELEMENT(sink.get())});
    if (!gst_element_link_many(GST_ELEMENT(appsrc.get()), GST_ELEMENT(queue.get()),
                               GST_ELEMENT(sink.get()), nullptr)) {
      throw std::runtime_error("failed to link appsrc to whepserversink");
    }
  }

  void buildH264() {
    const int descriptor_flags = ::fcntl(options.output_fd, F_GETFL);
    if (descriptor_flags < 0 ||
        ::fcntl(options.output_fd, F_SETFL, descriptor_flags | O_APPEND) < 0) {
      throw std::runtime_error("failed to configure append-only H.264 output descriptor");
    }
    const bool nv = options.encoder_factory == "nvv4l2h264enc";
    converter.reset(GST_OBJECT(
        gst_element_factory_make(nv ? "nvvideoconvert" : "videoconvert", "color-convert")));
    capsfilter.reset(GST_OBJECT(gst_element_factory_make("capsfilter", "encoder-input")));
    encoder.reset(
        GST_OBJECT(gst_element_factory_make(options.encoder_factory.c_str(), "h264-encoder")));
    parser.reset(GST_OBJECT(gst_element_factory_make("h264parse", "h264-parser")));
    stream_capsfilter.reset(GST_OBJECT(gst_element_factory_make("capsfilter", "annexb-output")));
    sink.reset(GST_OBJECT(gst_element_factory_make("fdsink", "h264-output")));
    if (!converter.get() || !capsfilter.get() || !encoder.get() || !parser.get() ||
        !stream_capsfilter.get() || !sink.get())
      throw std::runtime_error(
          "H.264 output is missing the exact encoder or a required pipeline element");

    GstCaps *encoder_caps = nullptr;
    if (nv) {
      encoder_caps = gst_caps_from_string("video/x-raw(memory:NVMM),format=NV12");
      g_object_set(encoder.get(), "bitrate", options.start_bitrate, "iframeinterval", options.fps,
                   "idrinterval", options.fps, "insert-sps-pps", TRUE, "insert-vui", TRUE,
                   "insert-aud", TRUE, nullptr);
      if (g_object_class_find_property(G_OBJECT_GET_CLASS(encoder.get()), "maxperf-enable"))
        g_object_set(encoder.get(), "maxperf-enable", TRUE, nullptr);
      gst_util_set_object_arg(G_OBJECT(encoder.get()), "control-rate", "constant_bitrate");
    } else if (options.encoder_factory == "x264enc") {
      encoder_caps = gst_caps_from_string("video/x-raw,format=I420");
      g_object_set(encoder.get(), "bitrate", options.start_bitrate / 1000u, "key-int-max",
                   options.fps, "bframes", guint{0}, "byte-stream", TRUE, nullptr);
      gst_util_set_object_arg(G_OBJECT(encoder.get()), "tune", "zerolatency");
      gst_util_set_object_arg(G_OBJECT(encoder.get()), "speed-preset", "ultrafast");
    } else {
      throw std::invalid_argument("H.264 output supports exact x264enc or nvv4l2h264enc only");
    }
    g_object_set(capsfilter.get(), "caps", encoder_caps, nullptr);
    gst_caps_unref(encoder_caps);
    g_object_set(parser.get(), "config-interval", -1, nullptr);
    GstCaps *stream_caps =
        gst_caps_from_string("video/x-h264,stream-format=byte-stream,alignment=au");
    g_object_set(stream_capsfilter.get(), "caps", stream_caps, nullptr);
    gst_caps_unref(stream_caps);
    g_object_set(sink.get(), "fd", options.output_fd, "sync", FALSE, "async", FALSE, nullptr);
    {
      std::lock_guard<std::mutex> lock(callback_state->mutex);
      callback_state->selected_encoder = factory_name(GST_ELEMENT(encoder.get()));
      if (callback_state->selected_encoder != options.encoder_factory)
        throw std::runtime_error("failed to construct the exact requested H.264 encoder");
    }

    addToPipeline({GST_ELEMENT(appsrc.get()), GST_ELEMENT(queue.get()),
                   GST_ELEMENT(converter.get()), GST_ELEMENT(capsfilter.get()),
                   GST_ELEMENT(encoder.get()), GST_ELEMENT(parser.get()),
                   GST_ELEMENT(stream_capsfilter.get()), GST_ELEMENT(sink.get())});
    if (!gst_element_link_many(
            GST_ELEMENT(appsrc.get()), GST_ELEMENT(queue.get()), GST_ELEMENT(converter.get()),
            GST_ELEMENT(capsfilter.get()), GST_ELEMENT(encoder.get()), GST_ELEMENT(parser.get()),
            GST_ELEMENT(stream_capsfilter.get()), GST_ELEMENT(sink.get()), nullptr)) {
      throw std::runtime_error("failed to link exact H.264 output pipeline");
    }
  }

  void start() {
    bus.reset(GST_OBJECT(gst_element_get_bus(GST_ELEMENT(pipeline.get()))));
    const auto state = gst_element_set_state(GST_ELEMENT(pipeline.get()), GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      poll();
      throw std::runtime_error("failed to start GStreamer video pipeline");
    }
  }

  void connectSignal(const char *name, GCallback callback) {
    auto *holder = new std::shared_ptr<CallbackState>(callback_state);
    const gulong id = g_signal_connect_data(
        sink.get(), name, callback, holder,
        [](gpointer data, GClosure *) {
          delete static_cast<std::shared_ptr<CallbackState> *>(data);
        },
        static_cast<GConnectFlags>(0));
    if (id == 0) {
      delete holder;
      throw std::runtime_error(std::string("failed to connect GStreamer signal: ") + name);
    }
    signal_handlers.push_back(id);
  }

  static gboolean onEncoderSetup(GstElement *, const gchar *, const gchar *, GstElement *encoder,
                                 gpointer user_data) {
    const auto state = *static_cast<std::shared_ptr<CallbackState> *>(user_data);
    const auto actual = factory_name(encoder);
    std::lock_guard<std::mutex> lock(state->mutex);
    state->selected_encoder = actual;
    if (actual != state->required_encoder) {
      state->error =
          "whepserversink selected " + actual + " instead of required " + state->required_encoder;
    }
    // Allow webrtcsink's default handler to apply its low-latency and dynamic
    // bitrate settings for this known encoder.
    return FALSE;
  }

  static void onConsumerAdded(GstElement *, const gchar *, GstElement *, gpointer user_data) {
    const auto state = *static_cast<std::shared_ptr<CallbackState> *>(user_data);
    std::lock_guard<std::mutex> lock(state->mutex);
    ++state->active_consumers;
  }

  static void onConsumerRemoved(GstElement *, const gchar *, GstElement *, gpointer user_data) {
    const auto state = *static_cast<std::shared_ptr<CallbackState> *>(user_data);
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->active_consumers > 0)
      --state->active_consumers;
  }

  bool push(const shm::ColorFrame &frame) {
    poll();
    validate(options, frame);
    if (frame.source_generation != generation || frame.width != width || frame.height != height ||
        frame.stride != stride || frame.encoding != encoding) {
      throw std::invalid_argument("color stream identity changed; create a new GstSession");
    }
    if (frame.timestamp_ns < first_timestamp_ns ||
        (last_timestamp_ns != 0 && frame.timestamp_ns <= last_timestamp_ns)) {
      ++rejected_timestamps;
      return false;
    }

    GstVideoInfo info;
    gst_video_info_init(&info);
    const GstVideoFormat format =
        frame.encoding == "rgb8" ? GST_VIDEO_FORMAT_RGB : GST_VIDEO_FORMAT_BGR;
    if (!gst_video_info_set_format(&info, format, frame.width, frame.height) ||
        info.stride[0] <= 0) {
      throw std::runtime_error("failed to describe GStreamer color frame layout");
    }
    const std::size_t row_bytes = static_cast<std::size_t>(frame.width) * 3u;
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, info.size, nullptr);
    if (!buffer)
      throw std::runtime_error("failed to allocate GStreamer color buffer");

    GstMapInfo map{};
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      gst_buffer_unref(buffer);
      throw std::runtime_error("failed to map GStreamer color buffer");
    }
    std::memset(map.data, 0, map.size);
    for (std::uint32_t row = 0; row < frame.height; ++row) {
      std::memcpy(map.data + static_cast<std::size_t>(row) * info.stride[0],
                  frame.payload.data() + static_cast<std::size_t>(row) * frame.stride, row_bytes);
    }
    gst_buffer_unmap(buffer, &map);
    GstClockTime pts = frame.timestamp_ns - first_timestamp_ns;
    GstClockTime running_time = 0;
    if (options.output == GstOutput::Whep) {
      const std::uint64_t now_ns = shm::unixTimeNs();
      running_time = gst_element_get_current_running_time(GST_ELEMENT(pipeline.get()));
      if (!GST_CLOCK_TIME_IS_VALID(running_time)) {
        gst_buffer_unref(buffer);
        throw std::runtime_error("WHEP pipeline has no running clock");
      }
      // Capture time precedes pipeline initialization; preserve frame age in
      // the pipeline clock rather than placing fresh frames in its future.
      const std::uint64_t age_ns = now_ns > frame.timestamp_ns ? now_ns - frame.timestamp_ns : 0;
      pts = running_time > age_ns ? running_time - age_ns : 0;
    }
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, options.fps);
    GST_BUFFER_OFFSET(buffer) = frame.sequence;

    const GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc.get()), buffer);
    if (flow != GST_FLOW_OK) {
      throw std::runtime_error("GStreamer appsrc rejected color frame: " + std::to_string(flow));
    }
    ++accepted_frames;
    last_timestamp_ns = frame.timestamp_ns;
    last_pts_ns = pts;
    last_running_time_ns = running_time;
    return true;
  }

  void poll() {
    if (stopped || !bus.get())
      return;
    while (GstMessage *message = gst_bus_pop_filtered(
               bus.as<GstBus>(), static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS |
                                                             GST_MESSAGE_LATENCY))) {
      switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
          GError *error = nullptr;
          gchar *debug = nullptr;
          gst_message_parse_error(message, &error, &debug);
          const auto text = gst_error("GStreamer video pipeline failed", error, debug);
          gst_message_unref(message);
          throw std::runtime_error(text);
        }
        case GST_MESSAGE_EOS:
          gst_message_unref(message);
          throw std::runtime_error("GStreamer video pipeline ended unexpectedly");
        case GST_MESSAGE_LATENCY:
          gst_bin_recalculate_latency(GST_BIN(pipeline.get()));
          break;
        default:
          break;
      }
      gst_message_unref(message);
    }
    std::lock_guard<std::mutex> lock(callback_state->mutex);
    if (!callback_state->error.empty())
      throw std::runtime_error(callback_state->error);
  }

  GstMetrics metrics() const {
    GstMetrics result;
    {
      std::lock_guard<std::mutex> lock(callback_state->mutex);
      result.accepted_frames = accepted_frames;
      result.rejected_timestamps = rejected_timestamps;
      result.last_pts_ns = last_pts_ns;
      result.last_running_time_ns = last_running_time_ns;
      result.active_consumers = callback_state->active_consumers;
      result.selected_encoder = callback_state->selected_encoder;
    }
    if (appsrc.get()) {
      guint64 level = 0;
      g_object_get(appsrc.get(), "current-level-buffers", &level, nullptr);
      result.queued_raw_frames =
          static_cast<unsigned>(std::min<guint64>(level, std::numeric_limits<unsigned>::max()));
    }
    if (sink.get() && options.output == GstOutput::Whep) {
      GstStructure *stats = nullptr;
      g_object_get(sink.get(), "stats", &stats, nullptr);
      if (stats) {
        gchar *text = gst_structure_to_string(stats);
        if (text)
          result.rtc_stats = text;
        g_free(text);
        gst_structure_free(stats);
      }
    }
    result.transport = options.output == GstOutput::Whep ? "whep" : "h264-fd";
    return result;
  }

  void stop() noexcept {
    if (stopped)
      return;
    stopped = true;
    if (sink.get()) {
      for (const gulong id : signal_handlers)
        g_signal_handler_disconnect(sink.get(), id);
      signal_handlers.clear();
    }
    if (appsrc.get())
      gst_app_src_end_of_stream(GST_APP_SRC(appsrc.get()));
    if (options.output == GstOutput::H264Fd && bus.get()) {
      if (GstMessage *message = gst_bus_timed_pop_filtered(
              bus.as<GstBus>(), 2 * GST_SECOND,
              static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR))) {
        gst_message_unref(message);
      }
    }
    if (pipeline.get())
      gst_element_set_state(GST_ELEMENT(pipeline.get()), GST_STATE_NULL);
  }

  GstOptions options;
  std::uint64_t generation;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t stride;
  std::string encoding;
  std::uint64_t first_timestamp_ns;
  std::uint64_t last_timestamp_ns{0};
  Object pipeline;
  Object appsrc;
  Object queue;
  Object converter;
  Object capsfilter;
  Object encoder;
  Object parser;
  Object stream_capsfilter;
  Object sink;
  Object bus;
  std::vector<std::pair<std::string, guint>> ranks;
  std::shared_ptr<CallbackState> callback_state;
  std::vector<gulong> signal_handlers;
  std::uint64_t accepted_frames{0};
  std::uint64_t rejected_timestamps{0};
  std::uint64_t last_pts_ns{0};
  std::uint64_t last_running_time_ns{0};
  bool stopped{false};
};

GstSession::GstSession(GstOptions options, const shm::ColorFrame &first_frame)
    : impl_(std::make_unique<Impl>(std::move(options), first_frame)) {}

GstSession::~GstSession() = default;

bool GstSession::push(const shm::ColorFrame &frame) {
  return impl_->push(frame);
}

void GstSession::poll() {
  impl_->poll();
}

GstMetrics GstSession::metrics() const {
  return impl_->metrics();
}

void GstSession::stop() noexcept {
  impl_->stop();
}

std::string GstSession::endpoint(const GstOptions &options) {
  if (options.output != GstOutput::Whep)
    return {};
  std::string base = options.listen_uri;
  while (!base.empty() && base.back() == '/')
    base.pop_back();
  return base + "/whep/endpoint";
}

}  // namespace lingtu::drivers::camera::video
