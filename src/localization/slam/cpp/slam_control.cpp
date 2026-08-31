#include "message/cpp/topics.hpp"
#include "message/cpp/qos.hpp"

#include "dds/dds.h"
#include "messages.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using lingtu::dds::QosProfile;
using lingtu::dds::make_qos;

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
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

dds_entity_t checked(dds_entity_t entity, const char* what) {
  if (entity < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-entity));
  }
  return entity;
}

struct CliConfig {
  std::string command;
  std::string path;
  std::optional<double> x;
  std::optional<double> y;
  std::optional<double> z;
  std::optional<double> yaw;
  int domain_id = 0;
  double timeout_s = 30.0;
};

std::string normalizedCommand(std::string command) {
  if (command == "load" || command == "load-map") {
    return "load_map";
  }
  if (command == "relocalize-saved-map" || command == "relocalize_saved_map") {
    return "relocalize";
  }
  if (command == "global-relocalize" || command == "global_relocalize") {
    return "global_relocalize";
  }
  if (command == "status" || command == "query-status" || command == "query_status") {
    return "query_status";
  }
  if (command == "track-against-map" || command == "track_against_map") {
    return "track_against_map";
  }
  return command;
}

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    throw std::runtime_error(
        "usage: slamctl load-map PATH [--domain-id N] [--timeout-s SECONDS] (standalone only)\n"
        "       slamctl relocalize --x X --y Y [--z Z] [--yaw YAW] "
        "[--domain-id N] [--timeout-s SECONDS]\n"
        "       slamctl global-relocalize [--domain-id N] "
        "[--timeout-s SECONDS]\n"
        "       slamctl status [--domain-id N] [--timeout-s SECONDS]\n"
        "       slamctl track-against-map [--domain-id N] "
        "[--timeout-s SECONDS]");
  }
  cfg.command = normalizedCommand(argv[1]);
  if (cfg.command != "load_map" && cfg.command != "relocalize" &&
      cfg.command != "global_relocalize" &&
      cfg.command != "query_status" && cfg.command != "track_against_map") {
    throw std::runtime_error("unsupported command: " + cfg.command);
  }
  int i = 2;
  if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
    if (cfg.command != "load_map") {
      throw std::runtime_error(
          "map paths are selected by ProductControl; relocalization uses the map already loaded by slamd");
    }
    cfg.path = argv[i++];
  }
  if (cfg.command == "load_map" && cfg.path.empty()) {
    throw std::runtime_error("missing map path");
  }
  for (; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--timeout-s") {
      cfg.timeout_s = std::stod(next());
    } else if (arg == "--x") {
      cfg.x = std::stod(next());
    } else if (arg == "--y") {
      cfg.y = std::stod(next());
    } else if (arg == "--z") {
      cfg.z = std::stod(next());
    } else if (arg == "--yaw") {
      cfg.yaw = std::stod(next());
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return cfg;
}

std::string typedAction(const CliConfig& cfg) {
  if (cfg.command == "relocalize") {
    return "seeded_relocalize";
  }
  return cfg.command;
}

void fillInitialPose(lingtu_dds_Pose& pose, const CliConfig& cfg) {
  pose.position.x = cfg.x.value_or(0.0);
  pose.position.y = cfg.y.value_or(0.0);
  pose.position.z = cfg.z.value_or(0.0);
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  if (cfg.yaw.has_value()) {
    const double half_yaw = *cfg.yaw * 0.5;
    pose.orientation.z = std::sin(half_yaw);
    pose.orientation.w = std::cos(half_yaw);
  }
}

bool hasInitialPose(const CliConfig& cfg) {
  return cfg.x.has_value() || cfg.y.has_value() || cfg.z.has_value() || cfg.yaw.has_value();
}

std::string ddsPoseJson(const lingtu_dds_Pose& pose) {
  return std::string("{\"x\":") + std::to_string(pose.position.x) +
      ",\"y\":" + std::to_string(pose.position.y) +
      ",\"z\":" + std::to_string(pose.position.z) +
      ",\"qx\":" + std::to_string(pose.orientation.x) +
      ",\"qy\":" + std::to_string(pose.orientation.y) +
      ",\"qz\":" + std::to_string(pose.orientation.z) +
      ",\"qw\":" + std::to_string(pose.orientation.w) + "}";
}

std::string relocalizationResponseJson(
    const lingtu_dds_RelocalizationResponse& response,
    const CliConfig& cfg) {
  const std::string message = response.message ? response.message : "";
  const std::string action = response.action ? response.action : "";
  const std::string engine = response.engine ? response.engine : "";
  const std::string state = response.state ? response.state : "";
  const std::string refine_backend = response.refine_backend ? response.refine_backend : "";
  std::string json =
      "{\"schema_version\":\"lingtu.slam.relocalization_response.v1\","
      "\"request_id\":\"" + jsonEscape(response.request_id ? response.request_id : "") + "\"," +
      "\"action\":\"" + jsonEscape(action) + "\"," +
      "\"engine\":\"" + jsonEscape(engine) + "\"," +
      "\"success\":" + (response.success ? "true" : "false") + "," +
      "\"path\":\"" + jsonEscape(cfg.path) + "\"," +
      "\"message\":\"" + jsonEscape(message) + "\"," +
      "\"last_relocalization_message\":\"" + jsonEscape(message) + "\"," +
      "\"quality\":" + std::to_string(response.quality) + "," +
      "\"relocalization_quality\":" + std::to_string(response.quality) + "," +
      "\"map_loaded\":" + (response.map_loaded ? "true" : "false") + "," +
      "\"relocalization_state\":\"" + jsonEscape(state) + "\"," +
      "\"relocalization_refine_backend\":\"" + jsonEscape(refine_backend) + "\"," +
      "\"relocalization_refine_iterations\":" + std::to_string(response.refine_iterations) + "," +
      "\"relocalization_refine_inliers\":" + std::to_string(response.refine_inliers) + "," +
      "\"relocalization_refine_input_points\":" +
      std::to_string(response.refine_input_points) + "," +
      "\"relocalization_refine_evaluated_points\":" +
      std::to_string(response.refine_evaluated_points) + "," +
      "\"relocalization_refine_support_ratio\":" +
      std::to_string(response.refine_support_ratio) + "," +
      "\"relocalization_refine_overlap_inlier_ratio\":" +
      std::to_string(response.refine_overlap_inlier_ratio) + "," +
      "\"relocalization_refine_converged\":" +
      (response.refine_converged ? "true" : "false") + "," +
      "\"relocalization_refine_pos_cov_trace\":" +
      std::to_string(response.refine_pos_cov_trace) + "," +
      "\"track_against_map_supported\":" +
      (response.track_against_map_supported ? "true" : "false") + "," +
      "\"track_against_map_enabled\":" +
      (response.track_against_map_enabled ? "true" : "false") + "," +
      "\"track_against_map_failures\":" +
      std::to_string(response.track_against_map_failures);
  json += ",\"relocalization_map_body\":";
  json += response.has_map_body ? ddsPoseJson(response.map_body) : "null";
  json += ",\"map_odom_tf\":";
  json += response.has_map_odom
      ? std::string("{\"valid\":true,\"frame_id\":\"map\",\"child_frame_id\":\"odom\",") +
          "\"pose\":" + ddsPoseJson(response.map_odom) + "}"
      : "null";
  json += ",\"source\":\"cpp_typed_dds\"}";
  return json;
}

int runTypedRelocalizationService(const CliConfig& cfg, const std::string& request_id) {
  const dds_entity_t participant = checked(
      dds_create_participant(static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
      "dds_create_participant");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber");

  const dds_entity_t request_topic = checked(
      dds_create_topic(
          participant,
          &lingtu_dds_RelocalizationRequest_desc,
          lingtu::message::kSlamRelocalizationRequest.dds_topic.data(),
          nullptr,
          nullptr),
      "dds_create_topic(relocalization_request)");
  const dds_entity_t response_topic = checked(
      dds_create_topic(
          participant,
          &lingtu_dds_RelocalizationResponse_desc,
          lingtu::message::kSlamRelocalizationResponse.dds_topic.data(),
          nullptr,
          nullptr),
      "dds_create_topic(relocalization_response)");
  auto request_qos = make_qos(QosProfile::CommandRequest);
  auto response_qos = make_qos(QosProfile::CommandAck);
  const dds_entity_t writer = checked(
      dds_create_writer(publisher, request_topic, request_qos.get(), nullptr),
      "dds_create_writer(relocalization_request)");
  const dds_entity_t reader = checked(
      dds_create_reader(subscriber, response_topic, response_qos.get(), nullptr),
      "dds_create_reader(relocalization_response)");

  const std::string action = typedAction(cfg);
  const std::string engine = action == "global_relocalize"
      ? "bbs3d_gicp"
      : action == "seeded_relocalize" ? "seeded_gicp" : "auto";

  lingtu_dds_RelocalizationRequest request{};
  request.request_id = const_cast<char*>(request_id.c_str());
  request.action = const_cast<char*>(action.c_str());
  request.engine = const_cast<char*>(engine.c_str());
  request.map_path = const_cast<char*>(cfg.path.c_str());
  request.has_initial_pose = hasInitialPose(cfg);
  fillInitialPose(request.initial_pose, cfg);
  request.timeout_s = cfg.timeout_s;

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const dds_return_t write_ret = dds_write(writer, &request);
  if (write_ret < 0) {
    dds_delete(participant);
    throw std::runtime_error(
        std::string("dds_write(relocalization_request): ") + dds_strretcode(-write_ret));
  }

  const double deadline = nowSeconds() + cfg.timeout_s;
  while (nowSeconds() < deadline) {
    void* samples[8];
    dds_sample_info_t infos[8];
    for (auto& sample : samples) {
      sample = dds_alloc(sizeof(lingtu_dds_RelocalizationResponse));
      std::memset(sample, 0, sizeof(lingtu_dds_RelocalizationResponse));
    }
    const dds_return_t count = dds_take(reader, samples, infos, 8, 8);
    if (count >= 0) {
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto* response = static_cast<lingtu_dds_RelocalizationResponse*>(samples[i]);
        const std::string response_request_id =
            response->request_id ? response->request_id : "";
        if (response_request_id != request_id) {
          continue;
        }
        std::printf("%s\n", relocalizationResponseJson(*response, cfg).c_str());
        const bool ok = response->success;
        for (auto& sample : samples) {
          dds_sample_free(sample, &lingtu_dds_RelocalizationResponse_desc, DDS_FREE_ALL);
        }
        dds_delete(participant);
        return ok ? 0 : 3;
      }
    } else {
      std::fprintf(
          stderr,
          "dds_take(relocalization_response): %s\n",
          dds_strretcode(-count));
    }
    for (auto& sample : samples) {
      dds_sample_free(sample, &lingtu_dds_RelocalizationResponse_desc, DDS_FREE_ALL);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  dds_delete(participant);
  std::fprintf(stderr, "timeout waiting for SLAM relocalization response\n");
  return 4;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    const std::string request_id =
        "slamctl-" + std::to_string(static_cast<std::uint64_t>(nowSeconds() * 1000000.0));
    return runTypedRelocalizationService(cfg, request_id);
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }
}
