#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "rapidjson/document.h"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <csignal>

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

double yawFromQuaternion(const lingtu_dds_Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

std::string jsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += ch;
        break;
    }
  }
  return out;
}

struct PathPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double ts = 0.0;
  std::string frame_id = "map";
};

struct PathPayload {
  std::string frame_id = "map";
  std::vector<PathPoint> points;
};

const rapidjson::Value* member(const rapidjson::Value& value, const char* key) {
  if (!value.IsObject()) {
    return nullptr;
  }
  const auto it = value.FindMember(key);
  if (it == value.MemberEnd()) {
    return nullptr;
  }
  return &it->value;
}

std::string stringMember(
    const rapidjson::Value& value,
    const char* key,
    const std::string& fallback) {
  const rapidjson::Value* field = member(value, key);
  if (field == nullptr || !field->IsString()) {
    return fallback;
  }
  return field->GetString();
}

std::optional<double> numberMember(const rapidjson::Value& value, const char* key) {
  const rapidjson::Value* field = member(value, key);
  if (field == nullptr || !field->IsNumber()) {
    return std::nullopt;
  }
  const double parsed = field->GetDouble();
  return std::isfinite(parsed) ? std::optional<double>(parsed) : std::nullopt;
}

PathPayload parsePathPayload(const rapidjson::Document& doc, const char* key) {
  PathPayload payload;
  const rapidjson::Value* object = member(doc, key);
  if (object == nullptr || !object->IsObject()) {
    return payload;
  }
  payload.frame_id = stringMember(*object, "frame_id", "map");
  const rapidjson::Value* path = member(*object, "path");
  if (path == nullptr || !path->IsArray()) {
    return payload;
  }
  for (const auto& item : path->GetArray()) {
    if (!item.IsObject()) {
      continue;
    }
    const auto x = numberMember(item, "x");
    const auto y = numberMember(item, "y");
    if (!x || !y) {
      continue;
    }
    PathPoint point;
    point.x = *x;
    point.y = *y;
    point.z = numberMember(item, "z").value_or(0.0);
    point.yaw = numberMember(item, "yaw").value_or(0.0);
    point.ts = numberMember(item, "ts").value_or(0.0);
    point.frame_id = stringMember(item, "frame_id", payload.frame_id);
    payload.points.push_back(std::move(point));
  }
  return payload;
}

struct TwistPayload {
  std::string frame_id = "base_link";
  double ts = 0.0;
  double lx = 0.0;
  double ly = 0.0;
  double lz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
};

std::optional<TwistPayload> parseTwistPayload(const rapidjson::Document& doc) {
  const rapidjson::Value* object = member(doc, "cmd_vel");
  if (object == nullptr || !object->IsObject()) {
    return std::nullopt;
  }
  TwistPayload payload;
  payload.frame_id = stringMember(*object, "frame_id", "base_link");
  payload.ts = numberMember(*object, "ts").value_or(0.0);
  const rapidjson::Value* linear = member(*object, "linear");
  const rapidjson::Value* angular = member(*object, "angular");
  if (linear != nullptr && linear->IsObject()) {
    payload.lx = numberMember(*linear, "x").value_or(0.0);
    payload.ly = numberMember(*linear, "y").value_or(0.0);
    payload.lz = numberMember(*linear, "z").value_or(0.0);
  }
  if (angular != nullptr && angular->IsObject()) {
    payload.ax = numberMember(*angular, "x").value_or(0.0);
    payload.ay = numberMember(*angular, "y").value_or(0.0);
    payload.az = numberMember(*angular, "z").value_or(0.0);
  }
  return payload;
}

struct HttpResponse {
  int status = 0;
  std::string body;
};

class HttpClient {
 public:
  HttpClient(std::string host, int port, double timeout_s = 1.0)
      : host_(std::move(host)), port_(port), timeout_s_(timeout_s) {}

  HttpResponse get(const std::string& path) const {
    return request("GET", path, "");
  }

  HttpResponse postJson(const std::string& path, const std::string& body) const {
    return request("POST", path, body);
  }

 private:
  HttpResponse request(const std::string& method, const std::string& path, const std::string& body) const {
    addrinfo hints{};
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_UNSPEC;
    addrinfo* result = nullptr;
    const std::string port_text = std::to_string(port_);
    const int gai = getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &result);
    if (gai != 0) {
      throw std::runtime_error(std::string("getaddrinfo: ") + gai_strerror(gai));
    }

    int fd = -1;
    for (addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
      fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (fd < 0) {
        continue;
      }
      timeval timeout{};
      timeout.tv_sec = static_cast<int>(timeout_s_);
      timeout.tv_usec = static_cast<int>((timeout_s_ - timeout.tv_sec) * 1000000.0);
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
      setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
        break;
      }
      close(fd);
      fd = -1;
    }
    freeaddrinfo(result);
    if (fd < 0) {
      throw std::runtime_error("connect gateway failed");
    }

    std::ostringstream req;
    req << method << " " << path << " HTTP/1.1\r\n"
        << "Host: " << host_ << ":" << port_ << "\r\n"
        << "Connection: close\r\n";
    if (method == "POST") {
      req << "Content-Type: application/json\r\n"
          << "Content-Length: " << body.size() << "\r\n";
    }
    req << "\r\n";
    if (method == "POST") {
      req << body;
    }
    const std::string wire = req.str();
    std::size_t sent = 0;
    while (sent < wire.size()) {
      const ssize_t n = send(fd, wire.data() + sent, wire.size() - sent, 0);
      if (n <= 0) {
        close(fd);
        throw std::runtime_error("send gateway request failed");
      }
      sent += static_cast<std::size_t>(n);
    }

    std::string response;
    char buf[4096];
    for (;;) {
      const ssize_t n = recv(fd, buf, sizeof(buf), 0);
      if (n < 0) {
        close(fd);
        throw std::runtime_error("recv gateway response failed");
      }
      if (n == 0) {
        break;
      }
      response.append(buf, static_cast<std::size_t>(n));
    }
    close(fd);

    HttpResponse parsed;
    const std::size_t status_start = response.find(' ');
    if (status_start != std::string::npos && status_start + 4 <= response.size()) {
      parsed.status = std::atoi(response.c_str() + status_start + 1);
    }
    const std::size_t body_start = response.find("\r\n\r\n");
    parsed.body = body_start == std::string::npos ? response : response.substr(body_start + 4);
    return parsed;
  }

  std::string host_;
  int port_ = 5050;
  double timeout_s_ = 1.0;
};

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void logDdsError(dds_return_t value, const char* what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

template <typename T, typename Handler>
void drainReader(
    dds_entity_t reader,
    const dds_topic_descriptor_t& descriptor,
    Handler&& handler) {
  constexpr std::size_t kMaxSamples = 16;
  void* samples[kMaxSamples];
  dds_sample_info_t infos[kMaxSamples];
  for (auto& sample : samples) {
    sample = dds_alloc(sizeof(T));
    std::memset(sample, 0, sizeof(T));
  }
  const dds_return_t count = dds_take(reader, samples, infos, kMaxSamples, kMaxSamples);
  if (count >= 0) {
    for (dds_return_t i = 0; i < count; ++i) {
      if (infos[i].valid_data) {
        handler(*static_cast<T*>(samples[i]));
      }
    }
  } else {
    logDdsError(count, "dds_take");
  }
  for (auto& sample : samples) {
    dds_sample_free(sample, &descriptor, DDS_FREE_ALL);
  }
}

class DdsNavEndpoint {
 public:
  explicit DdsNavEndpoint(int domain_id) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber_ = checked(dds_create_subscriber(participant_, nullptr, nullptr),
                          "dds_create_subscriber");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher");

    goal_reader_ = reader(
        lingtu::message::kNavGoalPose.dds_topic.data(),
        &lingtu_dds_PoseStamped_desc,
        "goal_pose");
    cancel_reader_ = reader(
        lingtu::message::kNavCancel.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "cancel");
    instruction_reader_ = reader(
        lingtu::message::kNavSemanticInstruction.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "instruction");
    global_path_writer_ = writer(
        lingtu::message::kNavGlobalPath.dds_topic.data(),
        &lingtu_dds_Path_desc,
        "global_path");
    local_path_writer_ = writer(
        lingtu::message::kNavLocalPath.dds_topic.data(),
        &lingtu_dds_Path_desc,
        "local_path");
    cmd_vel_writer_ = writer(
        lingtu::message::kNavCmdVel.dds_topic.data(),
        &lingtu_dds_TwistStamped_desc,
        "cmd_vel");
  }

  ~DdsNavEndpoint() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  DdsNavEndpoint(const DdsNavEndpoint&) = delete;
  DdsNavEndpoint& operator=(const DdsNavEndpoint&) = delete;

  template <typename Handler>
  void drainGoals(Handler&& handler) {
    drainReader<lingtu_dds_PoseStamped>(
        goal_reader_, lingtu_dds_PoseStamped_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCancels(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        cancel_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainInstructions(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        instruction_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  void writePath(const PathPayload& payload, bool local_path) {
    lingtu_dds_Path msg{};
    const double stamp_s = nowSeconds();
    fillHeader(msg.header, stamp_s, payload.frame_id.c_str());
    std::vector<lingtu_dds_PoseStamped> poses(payload.points.size());
    for (std::size_t i = 0; i < payload.points.size(); ++i) {
      const auto& point = payload.points[i];
      auto& pose = poses[i];
      fillHeader(
          pose.header,
          point.ts > 0.0 ? point.ts : stamp_s,
          point.frame_id.empty() ? payload.frame_id.c_str() : point.frame_id.c_str());
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation = quaternionFromYaw(point.yaw);
    }
    msg.poses._maximum = static_cast<std::uint32_t>(poses.size());
    msg.poses._length = static_cast<std::uint32_t>(poses.size());
    msg.poses._buffer = poses.data();
    msg.poses._release = false;
    logDdsError(
        dds_write(local_path ? local_path_writer_ : global_path_writer_, &msg),
        local_path ? "dds_write(local_path)" : "dds_write(global_path)");
  }

  void writeCmdVel(const TwistPayload& payload) {
    lingtu_dds_TwistStamped msg{};
    fillHeader(msg.header, payload.ts > 0.0 ? payload.ts : nowSeconds(), payload.frame_id.c_str());
    msg.twist.linear.x = payload.lx;
    msg.twist.linear.y = payload.ly;
    msg.twist.linear.z = payload.lz;
    msg.twist.angular.x = payload.ax;
    msg.twist.angular.y = payload.ay;
    msg.twist.angular.z = payload.az;
    logDdsError(dds_write(cmd_vel_writer_, &msg), "dds_write(cmd_vel)");
  }

 private:
  dds_entity_t reader(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    return checked(
        dds_create_reader(subscriber_, topic, nullptr, nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
  }

  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    return checked(
        dds_create_writer(publisher_, topic, nullptr, nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_ = 0;
  dds_entity_t subscriber_ = 0;
  dds_entity_t publisher_ = 0;
  dds_entity_t goal_reader_ = 0;
  dds_entity_t cancel_reader_ = 0;
  dds_entity_t instruction_reader_ = 0;
  dds_entity_t global_path_writer_ = 0;
  dds_entity_t local_path_writer_ = 0;
  dds_entity_t cmd_vel_writer_ = 0;
};

struct CliConfig {
  std::string gateway_host = "127.0.0.1";
  int gateway_port = 5050;
  int domain_id = 0;
  double tick_hz = 20.0;
  double publish_hz = 5.0;
  double log_status_s = 5.0;
};

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--gateway-host") {
      cfg.gateway_host = next();
    } else if (arg == "--gateway-port") {
      cfg.gateway_port = std::stoi(next());
    } else if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--publish-hz") {
      cfg.publish_hz = std::stod(next());
    } else if (arg == "--log-status-s") {
      cfg.log_status_s = std::stod(next());
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_nav_cyclone_endpoint [--domain-id N] "
          "[--gateway-host HOST] [--gateway-port PORT] [--tick-hz HZ] "
          "[--publish-hz HZ] [--log-status-s SECONDS]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.tick_hz = std::max(1.0, cfg.tick_hz);
  cfg.publish_hz = std::max(0.2, cfg.publish_hz);
  return cfg;
}

std::string requestId(const char* prefix, std::uint64_t seq) {
  return std::string(prefix) + "-" + std::to_string(static_cast<std::uint64_t>(nowSeconds() * 1000.0)) +
      "-" + std::to_string(seq);
}

bool httpOk(const HttpResponse& response) {
  return response.status >= 200 && response.status < 300;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    std::signal(SIGPIPE, SIG_IGN);

    const CliConfig cfg = parseArgs(argc, argv);
    DdsNavEndpoint dds(cfg.domain_id);
    HttpClient gateway(cfg.gateway_host, cfg.gateway_port);

    std::uint64_t goal_count = 0;
    std::uint64_t cancel_count = 0;
    std::uint64_t instruction_count = 0;
    std::uint64_t output_count = 0;
    std::uint64_t http_errors = 0;
    std::uint64_t seq = 0;
    double next_publish = 0.0;
    double next_log = nowSeconds() + cfg.log_status_s;

    std::fprintf(
        stderr,
        "lingtu_nav_cyclone_endpoint: domain=%d gateway=%s:%d publish_hz=%.2f\n",
        cfg.domain_id,
        cfg.gateway_host.c_str(),
        cfg.gateway_port,
        cfg.publish_hz);

    while (g_running) {
      dds.drainGoals([&](const lingtu_dds_PoseStamped& msg) {
        ++seq;
        ++goal_count;
        const double yaw = yawFromQuaternion(msg.pose.orientation);
        const char* frame = msg.header.frame_id && msg.header.frame_id[0] ? msg.header.frame_id : "map";
        const std::string body =
            "{\"x\":" + std::to_string(msg.pose.position.x) +
            ",\"y\":" + std::to_string(msg.pose.position.y) +
            ",\"z\":" + std::to_string(msg.pose.position.z) +
            ",\"yaw\":" + std::to_string(yaw) +
            ",\"frame_id\":\"map\"" +
            ",\"source\":\"api\"" +
            ",\"target_type\":\"coordinate\"" +
            ",\"client_id\":\"dds_nav_endpoint\"" +
            ",\"request_id\":\"" + requestId("dds-goal", seq) + "\"" +
            ",\"metadata\":{\"transport\":\"dds\",\"source_frame\":\"" + jsonEscape(frame) + "\"}}";
        try {
          const HttpResponse response = gateway.postJson("/api/v1/goal", body);
          if (!httpOk(response)) {
            ++http_errors;
            std::fprintf(stderr, "gateway goal rejected: http=%d body=%s\n", response.status, response.body.c_str());
          }
        } catch (const std::exception& exc) {
          ++http_errors;
          std::fprintf(stderr, "gateway goal post failed: %s\n", exc.what());
        }
      });

      dds.drainCancels([&](const lingtu_dds_Text& msg) {
        ++seq;
        ++cancel_count;
        const std::string reason = msg.data && msg.data[0] ? msg.data : "dds_cancel";
        const std::string body =
            "{\"reason\":\"" + jsonEscape(reason) +
            "\",\"client_id\":\"dds_nav_endpoint\"" +
            ",\"request_id\":\"" + requestId("dds-cancel", seq) + "\"}";
        try {
          const HttpResponse response = gateway.postJson("/api/v1/navigation/cancel", body);
          if (!httpOk(response)) {
            ++http_errors;
            std::fprintf(stderr, "gateway cancel rejected: http=%d body=%s\n", response.status, response.body.c_str());
          }
        } catch (const std::exception& exc) {
          ++http_errors;
          std::fprintf(stderr, "gateway cancel post failed: %s\n", exc.what());
        }
      });

      dds.drainInstructions([&](const lingtu_dds_Text& msg) {
        ++seq;
        ++instruction_count;
        const std::string text = msg.data && msg.data[0] ? msg.data : "";
        if (text.empty()) {
          return;
        }
        const std::string body =
            "{\"text\":\"" + jsonEscape(text) +
            "\",\"client_id\":\"dds_nav_endpoint\"" +
            ",\"request_id\":\"" + requestId("dds-instruction", seq) + "\"}";
        try {
          const HttpResponse response = gateway.postJson("/api/v1/instruction", body);
          if (!httpOk(response)) {
            ++http_errors;
            std::fprintf(stderr, "gateway instruction rejected: http=%d body=%s\n", response.status, response.body.c_str());
          }
        } catch (const std::exception& exc) {
          ++http_errors;
          std::fprintf(stderr, "gateway instruction post failed: %s\n", exc.what());
        }
      });

      const double now = nowSeconds();
      if (now >= next_publish) {
        next_publish = now + 1.0 / cfg.publish_hz;
        try {
          const HttpResponse response = gateway.get("/api/v1/navigation/dds_snapshot");
          if (!httpOk(response)) {
            ++http_errors;
          } else {
            rapidjson::Document doc;
            doc.Parse(response.body.c_str());
            if (doc.HasParseError() || !doc.IsObject()) {
              ++http_errors;
            } else {
              const PathPayload global_path = parsePathPayload(doc, "global_path");
              const PathPayload local_path = parsePathPayload(doc, "local_path");
              dds.writePath(global_path, false);
              dds.writePath(local_path, true);
              if (const auto cmd_vel = parseTwistPayload(doc)) {
                dds.writeCmdVel(*cmd_vel);
              }
              ++output_count;
            }
          }
        } catch (const std::exception& exc) {
          ++http_errors;
          std::fprintf(stderr, "gateway snapshot poll failed: %s\n", exc.what());
        }
      }

      if (cfg.log_status_s > 0.0 && now >= next_log) {
        next_log = now + cfg.log_status_s;
        std::fprintf(
            stderr,
            "nav_endpoint: goals=%llu cancels=%llu instructions=%llu outputs=%llu http_errors=%llu\n",
            static_cast<unsigned long long>(goal_count),
            static_cast<unsigned long long>(cancel_count),
            static_cast<unsigned long long>(instruction_count),
            static_cast<unsigned long long>(output_count),
            static_cast<unsigned long long>(http_errors));
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(1000.0 / cfg.tick_hz)));
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_nav_cyclone_endpoint failed: %s\n", exc.what());
    return 1;
  }
}
