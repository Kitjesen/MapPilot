#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

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

std::optional<std::string> jsonStringValue(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = json.find(needle);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  pos = json.find('"', pos + 1);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  std::string value;
  bool escaped = false;
  for (std::size_t i = pos + 1; i < json.size(); ++i) {
    const char ch = json[i];
    if (escaped) {
      value += ch;
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return value;
    }
    value += ch;
  }
  return std::nullopt;
}

bool jsonBoolValue(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = json.find(needle);
  if (pos == std::string::npos) {
    return false;
  }
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) {
    return false;
  }
  while (pos + 1 < json.size() && std::isspace(static_cast<unsigned char>(json[pos + 1]))) {
    ++pos;
  }
  return json.compare(pos + 1, 4, "true") == 0;
}

dds_entity_t checked(dds_entity_t entity, const char* what) {
  if (entity < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-entity));
  }
  return entity;
}

struct CliConfig {
  std::string command = "save-map";
  std::string path;
  std::optional<double> x;
  std::optional<double> y;
  std::optional<double> z;
  std::optional<double> yaw;
  int domain_id = 0;
  double timeout_s = 30.0;
};

std::string normalizedCommand(std::string command) {
  if (command == "save" || command == "save-map") {
    return "save_map";
  }
  if (command == "load" || command == "load-map") {
    return "load_map";
  }
  if (command == "relocalize-saved-map" || command == "relocalize_saved_map") {
    return "relocalize";
  }
  if (command == "global-relocalize" || command == "global_relocalize") {
    return "global_relocalize";
  }
  return command;
}

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    throw std::runtime_error(
        "usage: lingtu_slam_control save-map PATH [--domain-id N] [--timeout-s SECONDS]\n"
        "       lingtu_slam_control load-map PATH [--domain-id N] [--timeout-s SECONDS]\n"
        "       lingtu_slam_control relocalize [PATH] --x X --y Y [--z Z] [--yaw YAW] "
        "[--domain-id N] [--timeout-s SECONDS]\n"
        "       lingtu_slam_control global-relocalize [PATH] [--domain-id N] "
        "[--timeout-s SECONDS]");
  }
  cfg.command = normalizedCommand(argv[1]);
  if (cfg.command != "save_map" && cfg.command != "load_map" &&
      cfg.command != "relocalize" && cfg.command != "global_relocalize") {
    throw std::runtime_error("unsupported command: " + cfg.command);
  }
  int i = 2;
  if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
    cfg.path = argv[i++];
  }
  if ((cfg.command == "save_map" || cfg.command == "load_map") && cfg.path.empty()) {
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

}  // namespace

int main(int argc, char** argv) {
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    const std::string request_id =
        "slamctl-" + std::to_string(static_cast<std::uint64_t>(nowSeconds() * 1000000.0));
    std::string payload =
        "{\"schema_version\":\"lingtu.slam.map_command.v1\","
        "\"request_id\":\"" + jsonEscape(request_id) + "\","
        "\"action\":\"" + jsonEscape(cfg.command) + "\"";
    if (!cfg.path.empty()) {
      payload += ",\"path\":\"" + jsonEscape(cfg.path) + "\"";
    }
    if (cfg.x.has_value()) {
      payload += ",\"x\":" + std::to_string(*cfg.x);
    }
    if (cfg.y.has_value()) {
      payload += ",\"y\":" + std::to_string(*cfg.y);
    }
    if (cfg.z.has_value()) {
      payload += ",\"z\":" + std::to_string(*cfg.z);
    }
    if (cfg.yaw.has_value()) {
      payload += ",\"yaw\":" + std::to_string(*cfg.yaw);
    }
    payload += "}";

    const dds_entity_t participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
        "dds_create_participant");
    const dds_entity_t publisher = checked(
        dds_create_publisher(participant, nullptr, nullptr),
        "dds_create_publisher");
    const dds_entity_t subscriber = checked(
        dds_create_subscriber(participant, nullptr, nullptr),
        "dds_create_subscriber");

    const dds_entity_t command_topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_Text_desc,
            lingtu::message::kSlamMapCommand.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(map_command)");
    const dds_entity_t event_topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_Text_desc,
            lingtu::message::kSlamMapEvent.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(map_event)");
    const dds_entity_t writer = checked(
        dds_create_writer(publisher, command_topic, nullptr, nullptr),
        "dds_create_writer(map_command)");
    const dds_entity_t reader = checked(
        dds_create_reader(subscriber, event_topic, nullptr, nullptr),
        "dds_create_reader(map_event)");

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    lingtu_dds_Text command_msg{};
    command_msg.data = const_cast<char*>(payload.c_str());
    const dds_return_t write_ret = dds_write(writer, &command_msg);
    if (write_ret < 0) {
      throw std::runtime_error(std::string("dds_write(map_command): ") + dds_strretcode(-write_ret));
    }

    const double deadline = nowSeconds() + cfg.timeout_s;
    while (nowSeconds() < deadline) {
      void* samples[8];
      dds_sample_info_t infos[8];
      for (auto& sample : samples) {
        sample = dds_alloc(sizeof(lingtu_dds_Text));
        std::memset(sample, 0, sizeof(lingtu_dds_Text));
      }
      const dds_return_t count = dds_take(reader, samples, infos, 8, 8);
      if (count >= 0) {
        for (dds_return_t i = 0; i < count; ++i) {
          if (!infos[i].valid_data) {
            continue;
          }
          const auto* event = static_cast<lingtu_dds_Text*>(samples[i]);
          const std::string text = event->data ? event->data : "";
          if (jsonStringValue(text, "request_id").value_or("") != request_id) {
            continue;
          }
          std::printf("%s\n", text.c_str());
          const bool ok = jsonBoolValue(text, "success");
          for (auto& sample : samples) {
            dds_sample_free(sample, &lingtu_dds_Text_desc, DDS_FREE_ALL);
          }
          dds_delete(participant);
          return ok ? 0 : 3;
        }
      } else {
        std::fprintf(stderr, "dds_take(map_event): %s\n", dds_strretcode(-count));
      }
      for (auto& sample : samples) {
        dds_sample_free(sample, &lingtu_dds_Text_desc, DDS_FREE_ALL);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    dds_delete(participant);
    std::fprintf(stderr, "timeout waiting for SLAM map save response\n");
    return 4;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }
}
