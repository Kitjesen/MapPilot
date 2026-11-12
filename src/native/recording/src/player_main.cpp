#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "lingtu/recording/cyclone_cdr.hpp"
#include "lingtu/recording/history_series.hpp"
#include "lingtu/recording/recorded_scene.hpp"
#include "lingtu/recording/message_preview.hpp"
#include "lingtu/recording/inspection_timeline_cdr.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "lingtu/recording/recording_core.hpp"
#include "lingtu/recording/topic_catalog.hpp"
#include "mcap/reader.hpp"

namespace {

struct Options {
  std::filesystem::path input;
  std::filesystem::path idl_path;
  int domain_id{84};
  double rate{1.0};
  bool allow_live_domain{false};
  bool dry_run{false};
  bool list_topics{false};
  bool info{false};
  bool series{false};
  bool scene{false};
  lingtu::recording::SceneQuery scene_query;
  lingtu::recording::SeriesQuery series_query;
  bool json{false};
  bool prefer_raw_inputs{false};
  std::string inspection_task_id;
  std::vector<std::string> topics;
};

void print_help() {
  std::cout << "usage: ltplay SESSION_DIR_OR_MCAP [options]\n"
            << "       lingtu_dds_player FILE [options]\n"
            << "       lingtu_dds_player --list-topics\n"
            << "       lingtu_dds_player --info FILE\n\n"
            << "  --scene-info FILE    List recorded cloud/pose/path streams offline\n"
            << "  --scene FILE         Read a scene window up to 30 seconds offline\n"
            << "  --max-frames N       Retain at most N cloud frames (1..32)\n"
            << "  --cloud-topic TOPIC  Select a recorded point cloud stream\n"
            << "  --pose-topic TOPIC   Select a recorded odometry stream\n"
            << "  --path-topic TOPIC   Select a recorded plan stream; repeat up to twice\n"
            << "  --max-poses N        Retain at most N trajectory poses (2..4000)\n"
            << "  --series FILE        Read an offline numeric field curve (never publishes)\n"
            << "  --field PATH         Numeric preview path, e.g. position_rad[0]\n"
            << "  --start S --end S    Window in seconds from recording start\n"
            << "  --max-points N       Curve samples (16..10000) or scene cloud points/frame (1..4096)\n"
            << "  --domain N           Isolated replay domain (default: 84)\n"
            << "  --rate N             Playback rate (default: 1.0)\n"
            << "  --topic TOPIC        Replay only this sensor topic; repeatable\n"
            << "  --dry-run            Check container/schema/CDR envelope without publishing\n"
            << "  --list-topics        List supported DDS topics without opening a file\n"
            << "  --info FILE          Show an offline MCAP topic summary\n"
            << "  --json               Machine-readable --info or --dry-run result\n"
            << "  --verify-inspection-task ID\n"
            << "                       Verify one task timeline; requires --dry-run\n"
            << "  --allow-live-domain  Explicitly permit domain 0\n"
            << "  --idl FILE           Expected LingTu OMG IDL schema\n";
}

Options parse_options(int argc, char **argv) {
  Options options;
  options.prefer_raw_inputs = std::filesystem::path(argv[0]).stem() == "ltplay";
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&]() -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument("missing value for " + argument);
      }
      return argv[index];
    };
    if (argument == "--domain") {
      options.domain_id = std::stoi(value());
    } else if (argument == "--rate" || argument == "-r") {
      options.rate = std::stod(value());
    } else if (argument == "--topic") {
      options.topics.push_back(value());
    } else if (argument == "--idl") {
      options.idl_path = value();
    } else if (argument == "--allow-live-domain") {
      options.allow_live_domain = true;
    } else if (argument == "--dry-run") {
      options.dry_run = true;
    } else if (argument == "--list-topics") {
      options.list_topics = true;
    } else if (argument == "--json") {
      options.json = true;
    } else if (argument == "--field") {
      options.series_query.field = value();
    } else if (argument == "--start") {
      options.series_query.start_s = std::stod(value());
      options.scene_query.start_s = options.series_query.start_s;
    } else if (argument == "--end") {
      options.series_query.end_s = std::stod(value());
      options.scene_query.end_s = *options.series_query.end_s;
    } else if (argument == "--max-points") {
      options.series_query.max_points = std::stoull(value());
      options.scene_query.max_points = options.series_query.max_points;
    } else if (argument == "--max-frames") {
      options.scene_query.max_frames = std::stoull(value());
    } else if (argument == "--max-poses") {
      options.scene_query.max_poses = std::stoull(value());
    } else if (argument == "--cloud-topic") {
      options.scene_query.cloud_topic = value();
    } else if (argument == "--pose-topic") {
      options.scene_query.pose_topic = value();
    } else if (argument == "--path-topic") {
      options.scene_query.path_topics.push_back(value());
    } else if (argument == "--scene" || argument == "--scene-info") {
      if (!options.input.empty()) throw std::invalid_argument("scene mode requires one input");
      options.input = value(); options.scene = true;
      options.scene_query.info = argument == "--scene-info";
    } else if (argument == "--series") {
      if (!options.input.empty()) throw std::invalid_argument("--series requires one input");
      options.input = value();
      options.series = true;
    } else if (argument == "--info") {
      if (!options.input.empty()) {
        throw std::invalid_argument("--info cannot be combined with a positional FILE");
      }
      options.input = value();
      options.info = true;
    } else if (argument == "--verify-inspection-task") {
      if (!options.inspection_task_id.empty()) {
        throw std::invalid_argument("--verify-inspection-task may be specified only once");
      }
      options.inspection_task_id = value();
      if (options.inspection_task_id.empty()) {
        throw std::invalid_argument("--verify-inspection-task requires a non-empty task_id");
      }
    } else if (argument == "-h" || argument == "--help") {
      print_help();
      std::exit(0);
    } else if (!argument.empty() && argument.front() == '-') {
      throw std::invalid_argument("unknown option: " + argument);
    } else if (options.input.empty()) {
      options.input = argument;
    } else {
      throw std::invalid_argument("unexpected positional argument: " + argument);
    }
  }
  if (options.input.empty() && !options.list_topics) {
    throw std::invalid_argument("MCAP input file is required");
  }
  if (options.prefer_raw_inputs && !options.input.empty() && !options.input.has_parent_path() &&
      !std::filesystem::exists(options.input)) {
    std::filesystem::path root;
    if (const auto *configured = std::getenv("LINGTU_RECORDING_ROOT"); configured && *configured) {
      root = configured;
    } else if (const auto *home = std::getenv("HOME"); home && *home) {
      root = std::filesystem::path(home) / "data/lingtu/recordings";
    } else if (const auto *home = std::getenv("USERPROFILE"); home && *home) {
      root = std::filesystem::path(home) / "data/lingtu/recordings";
    }
    const auto root_text = root.generic_string();
    if (root_text == "~" || root_text.rfind("~/", 0) == 0) {
      const char *home = std::getenv("HOME");
      if (!home || !*home) home = std::getenv("USERPROFILE");
      if (home && *home) root = std::filesystem::path(home) / root_text.substr(root_text == "~" ? 1 : 2);
    }
    if (!root.empty() && std::filesystem::is_directory(root / options.input)) {
      options.input = root / options.input;
    }
  }
  if (!options.input.empty() && std::filesystem::is_directory(options.input)) {
    options.input /= "dds/sensors.mcap";
  }
  if (options.list_topics && options.info) {
    throw std::invalid_argument("--list-topics and --info are mutually exclusive");
  }
  if (options.scene && (options.series || options.info || options.list_topics || options.dry_run))
    throw std::invalid_argument("select one read mode");
  if (options.series) {
    if (options.list_topics || options.info || options.dry_run || options.topics.size() != 1)
      throw std::invalid_argument("--series requires exactly one --topic and no other read mode");
    options.series_query.topic = options.topics.front();
  }
  if (!options.list_topics && !options.info && !options.series && !options.scene) {
    const std::string domain_error =
        lingtu::recording::validate_replay_domain(options.domain_id, options.allow_live_domain);
    if (!domain_error.empty()) {
      throw std::invalid_argument(domain_error);
    }
    lingtu::recording::replay_offset_ns(0, 1, options.rate);
    for (const auto &topic : options.topics) {
      if (!lingtu::recording::is_sensor_replay_topic(topic)) {
        throw std::invalid_argument("topic is not in the sensor replay allowlist: " + topic);
      }
    }
  }
  if (options.json && !options.info && !options.dry_run && !options.series && !options.scene) {
    throw std::invalid_argument("--json requires --info or --dry-run");
  }
  if (!options.inspection_task_id.empty() && !options.dry_run) {
    throw std::invalid_argument("--verify-inspection-task requires --dry-run");
  }
  if (options.inspection_task_id.find('\0') != std::string::npos ||
      options.inspection_task_id.size() > 256) {
    throw std::invalid_argument("--verify-inspection-task task_id must be at most 256 bytes");
  }
  return options;
}

std::string normalize_idl(std::string text) {
  std::string normalized;
  normalized.reserve(text.size());
  for (std::size_t index = 0; index < text.size(); ++index) {
    if (text[index] == '\r' && index + 1 < text.size() && text[index + 1] == '\n') {
      continue;
    }
    normalized.push_back(text[index]);
  }
  return normalized;
}

std::string schema_text(const mcap::Schema &schema) {
  return std::string(reinterpret_cast<const char *>(schema.data.data()), schema.data.size());
}

bool requested_topic(const Options &options, const lingtu::recording::TopicBinding &binding) {
  if (options.topics.empty()) {
    return true;
  }
  for (const auto &requested : options.topics) {
    const auto *selected = lingtu::recording::find_sensor_topic(requested);
    if (selected != nullptr && selected->contract == binding.contract) {
      return true;
    }
  }
  return false;
}

void print_topics() {
  for (const auto &binding : lingtu::recording::recording_topic_catalog()) {
    std::cout << binding.contract->topic << '\t' << binding.contract->dds_topic << '\t'
              << binding.contract->idl_type << '\t'
              << lingtu::recording::replay_policy_name(binding.replay_policy) << '\n';
  }
}
void print_info(const std::filesystem::path &input, bool json) {
  mcap::McapReader reader;
  const auto open_status = reader.open(input.string());
  if (!open_status.ok()) {
    throw std::runtime_error("failed to open MCAP: " + open_status.message);
  }
  if (!reader.header() || reader.header()->profile != lingtu::recording::kMcapProfile) {
    throw std::runtime_error("MCAP profile is not lingtu.dds.v1");
  }
  std::string parse_error;
  const auto summary_status =
      reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan,
                         [&](const mcap::Status &status) { parse_error = status.message; });
  if (!summary_status.ok()) {
    throw std::runtime_error("failed to read MCAP summary: " + summary_status.message);
  }
  if (!parse_error.empty()) {
    throw std::runtime_error("MCAP parse error: " + parse_error);
  }
  const auto &statistics = reader.statistics();
  if (!statistics) {
    throw std::runtime_error("MCAP summary is missing statistics");
  }
  const std::uint64_t duration_ns =
      statistics->messageCount == 0 || statistics->messageEndTime < statistics->messageStartTime
          ? 0
          : statistics->messageEndTime - statistics->messageStartTime;
  if (json) {
    std::cout << "{\"version\":1,\"ok\":true,\"profile\":" << std::quoted(reader.header()->profile)
              << ",\"message_count\":" << statistics->messageCount
              << ",\"start_time_ns\":" << statistics->messageStartTime
              << ",\"end_time_ns\":" << statistics->messageEndTime
              << ",\"duration_ns\":" << duration_ns << ",\"topics\":[";
  } else {
    std::cout << "profile=" << reader.header()->profile << " messages=" << statistics->messageCount
              << " duration_ns=" << duration_ns << '\n';
  }

  std::vector<mcap::ChannelPtr> channels;
  for (const auto &[channel_id, channel] : reader.channels()) {
    static_cast<void>(channel_id);
    channels.push_back(channel);
  }
  std::sort(channels.begin(), channels.end(), [](const auto &left, const auto &right) {
    return left->topic < right->topic;
  });
  bool first_channel = true;
  for (const auto &channel : channels) {
    const auto *binding = lingtu::recording::find_recording_topic(channel->topic);
    if (binding == nullptr) {
      throw std::runtime_error("MCAP contains a topic outside the native recording catalog: " +
                               channel->topic);
    }
    const auto count_it = statistics->channelMessageCounts.find(channel->id);
    const std::uint64_t count =
        count_it == statistics->channelMessageCounts.end() ? 0 : count_it->second;
    if (json) {
      if (!first_channel) std::cout << ',';
      first_channel = false;
      std::cout << "{\"topic\":" << std::quoted(std::string(binding->contract->topic))
                << ",\"dds_topic\":" << std::quoted(channel->topic)
                << ",\"type_name\":" << std::quoted(std::string(binding->contract->idl_type))
                << ",\"message_count\":" << count << ",\"replayable\":"
                << (binding->replay_policy == lingtu::recording::ReplayPolicy::Replayable ? "true" : "false") << '}';
    } else {
      std::cout << "topic=" << binding->contract->topic << " dds_topic=" << channel->topic
                << " count=" << count << " idl_type=" << binding->contract->idl_type
                << " policy=" << lingtu::recording::replay_policy_name(binding->replay_policy) << '\n';
    }
  }
  if (json) std::cout << "]}\n";
  reader.close();
}


dds_entity_t checked(dds_return_t result, const char *operation) {
  if (result < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-result));
  }
  return static_cast<dds_entity_t>(result);
}

struct Publisher {
  const lingtu::recording::TopicBinding *binding{nullptr};
  dds_entity_t topic{DDS_RETCODE_ERROR};
  dds_entity_t writer{DDS_RETCODE_ERROR};
};

}  // namespace

int main(int argc, char **argv) {
  dds_entity_t participant = DDS_RETCODE_ERROR;
  try {
    Options options = parse_options(argc, argv);
    if (options.list_topics) {
      print_topics();
      return 0;
    }
    if (options.scene) {
      std::cout << lingtu::recording::read_recorded_scene(options.input, options.scene_query).json() << "\n";
      return 0;
    }
    if (options.series) {
      std::cout << lingtu::recording::read_history_series(options.input, options.series_query).json() << "\n";
      return 0;
    }
    if (options.info) {
      print_info(options.input, options.json);
      return 0;
    }
    const auto idl_path = options.idl_path.empty()
                              ? lingtu::recording::resolve_recording_idl(
                                    lingtu::recording::recording_executable_path(argv[0]),
                                    LINGTU_RECORDING_DEFAULT_IDL)
                              : options.idl_path;
    const std::string expected_idl =
        normalize_idl(lingtu::recording::read_text_file(idl_path));

    mcap::McapReader reader;
    const auto open_status = reader.open(options.input.string());
    if (!open_status.ok()) {
      throw std::runtime_error("failed to open MCAP: " + open_status.message);
    }
    if (!reader.header() || reader.header()->profile != lingtu::recording::kMcapProfile) {
      throw std::runtime_error("MCAP profile is not lingtu.dds.v1");
    }
    std::string parse_error;
    const auto summary_status =
        reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan,
                           [&](const mcap::Status &status) { parse_error = status.message; });
    if (!summary_status.ok()) {
      throw std::runtime_error("failed to read MCAP summary: " + summary_status.message);
    }

    // The short operator entry defaults to raw estimator inputs when present.
    // Explicit topic choices still win; the established player entry retains
    // its existing all-sensor behavior for callers that rely on it.
    if (options.prefer_raw_inputs && options.topics.empty() && !options.dry_run) {
      for (const auto *name : {"/imu/raw", "/lidar/raw_frame"}) {
        const auto *binding = lingtu::recording::find_sensor_topic(name);
        for (const auto &entry : reader.channels()) {
          if (entry.second->topic == binding->contract->dds_topic) {
            const auto &statistics = reader.statistics();
            if (!statistics || (statistics->channelMessageCounts.count(entry.first) &&
                statistics->channelMessageCounts.at(entry.first) > 0)) {
              options.topics.emplace_back(name);
            }
          }
        }
      }
    }

    if (!options.dry_run) {
      participant = checked(
          dds_create_participant(static_cast<dds_domainid_t>(options.domain_id), nullptr, nullptr),
          "dds_create_participant");
    }

    std::unordered_map<std::string, Publisher> publishers;
    std::unordered_set<mcap::ChannelId> validated_channels;
    mcap::ReadMessageOptions read_options;
    read_options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
    std::uint64_t base_log_time = 0;
    std::chrono::steady_clock::time_point replay_start;
    std::uint64_t replayed = 0;
    std::uint64_t skipped_record_only = 0;
    lingtu::recording::InspectionTimelineCapture timeline_capture;

    for (const auto &view : reader.readMessages(
             [&](const mcap::Status &status) { parse_error = status.message; }, read_options)) {
      if (view.channel == nullptr || view.schema == nullptr) {
        throw std::runtime_error("MCAP message is missing channel or schema");
      }
      const auto *recording_binding = lingtu::recording::find_recording_topic(view.channel->topic);
      if (recording_binding == nullptr) {
        throw std::runtime_error("MCAP contains a topic outside the native recording catalog: " +
                                 view.channel->topic);
      }
      const auto *sensor_binding = lingtu::recording::find_sensor_topic(view.channel->topic);
      if (sensor_binding != nullptr && !requested_topic(options, *sensor_binding)) {
        continue;
      }
      if (sensor_binding == nullptr && !options.topics.empty() &&
          options.inspection_task_id.empty()) {
        continue;
      }
      if (validated_channels.insert(view.channel->id).second) {
        if (view.channel->messageEncoding != "cdr" || view.schema->encoding != "omgidl") {
          throw std::runtime_error("MCAP channel is not native DDS cdr/omgidl");
        }
        const auto type_metadata = view.channel->metadata.find("lingtu.idl_type");
        if (type_metadata == view.channel->metadata.end() ||
            type_metadata->second != recording_binding->contract->idl_type ||
            view.schema->name != recording_binding->contract->idl_type) {
          throw std::runtime_error("MCAP DDS type does not match the LingTu topic catalog");
        }
        if (normalize_idl(schema_text(*view.schema)) != expected_idl) {
          throw std::runtime_error("MCAP IDL schema differs from the local LingTu IDL");
        }
      }
      if (!options.inspection_task_id.empty()) {
        lingtu::recording::RecordedMessage recorded;
        recorded.wire_topic = view.channel->topic;
        recorded.log_time_ns = view.message.logTime;
        recorded.publish_time_ns = view.message.publishTime;
        recorded.sequence = view.message.sequence;
        if (view.message.data == nullptr || view.message.dataSize == 0) {
          throw std::runtime_error("MCAP message payload is empty: " + view.channel->topic);
        }
        recorded.payload.assign(view.message.data, view.message.data + view.message.dataSize);
        timeline_capture.observe(*recording_binding, recorded);
      }
      if (sensor_binding == nullptr) {
        const std::string validation_error = lingtu::recording::validate_cdr_payload(
            *recording_binding, view.message.data, static_cast<std::size_t>(view.message.dataSize));
        if (!validation_error.empty()) {
          throw std::runtime_error(validation_error + ": " + view.channel->topic);
        }
        ++skipped_record_only;
        if (options.dry_run) {
          ++replayed;
        }
        continue;
      }
      const std::string validation_error = lingtu::recording::validate_cdr_payload(
          *sensor_binding, view.message.data, static_cast<std::size_t>(view.message.dataSize));
      if (!validation_error.empty()) {
        throw std::runtime_error(validation_error + ": " + view.channel->topic);
      }

      if (!options.dry_run) {
        auto publisher_it = publishers.find(view.channel->topic);
        if (publisher_it == publishers.end()) {
          Publisher publisher;
          publisher.binding = sensor_binding;
          publisher.topic = checked(dds_create_topic(participant, sensor_binding->descriptor,
                                                     view.channel->topic.c_str(), nullptr, nullptr),
                                    "dds_create_topic");
          auto qos = lingtu::dds::make_qos(sensor_binding->qos_profile);
          dds_qset_lifespan(qos.get(), DDS_INFINITY);
          publisher.writer =
              checked(dds_create_writer(participant, publisher.topic, qos.get(), nullptr),
                      "dds_create_writer");
          publisher_it = publishers.emplace(view.channel->topic, publisher).first;
        }
        if (base_log_time == 0) {
          base_log_time = view.message.logTime;
          replay_start = std::chrono::steady_clock::now();
        }
        const auto offset = std::chrono::nanoseconds(
            lingtu::recording::replay_offset_ns(base_log_time, view.message.logTime, options.rate));
        std::this_thread::sleep_until(replay_start + offset);
        if (view.message.publishTime >
            static_cast<std::uint64_t>(std::numeric_limits<dds_time_t>::max())) {
          throw std::runtime_error("MCAP publish timestamp exceeds DDS time range");
        }
        const std::string forward_error = lingtu::recording::forward_cdr_message(
            publisher_it->second.writer, *sensor_binding, view.message.data,
            static_cast<std::size_t>(view.message.dataSize),
            static_cast<dds_time_t>(view.message.publishTime));
        if (!forward_error.empty()) {
          throw std::runtime_error(forward_error + ": " + view.channel->topic);
        }
      }
      ++replayed;
    }

    if (!parse_error.empty()) {
      throw std::runtime_error("MCAP parse error: " + parse_error);
    }
    reader.close();
    if (participant > 0) {
      dds_delete(participant);
      participant = DDS_RETCODE_ERROR;
    }
    if (replayed == 0) {
      throw std::runtime_error("no selected messages were found in the MCAP");
    }
    lingtu::recording::InspectionTimelineReport inspection_report;
    if (!options.inspection_task_id.empty()) {
      inspection_report = timeline_capture.verify(options.inspection_task_id);
      if (!inspection_report.ok) {
        throw std::runtime_error("inspection task timeline verification failed: " +
                                 inspection_report.summary());
      }
    }
    if (options.json) {
      std::cout << "{\"version\":1,\"ok\":true,\"mode\":\"validate\",\"validated_messages\":"
                << replayed << ",\"skipped_record_only\":" << skipped_record_only
                << ",\"domain\":" << options.domain_id << ",\"rate\":" << options.rate;
      if (!options.inspection_task_id.empty()) {
        std::cout << ",\"inspection_task_id\":" << std::quoted(options.inspection_task_id)
                  << ",\"terminal_state\":" << inspection_report.terminal_state
                  << ",\"final_output_sequence\":" << inspection_report.final_output_sequence;
      }
      std::cout << "}\n";
      return 0;
    }
    std::cout << (options.dry_run ? "validated=" : "replayed=") << replayed
              << " domain=" << options.domain_id << " rate=" << options.rate
              << " skipped_record_only=" << skipped_record_only;
    if (!options.inspection_task_id.empty()) {
      std::cout << " inspection_task_id=" << options.inspection_task_id
                << " terminal_state=" << inspection_report.terminal_state
                << " final_output_sequence=" << inspection_report.final_output_sequence;
    }
    std::cout << "\n";
    return 0;
  } catch (const std::exception &error) {
    if (participant > 0) {
      dds_delete(participant);
    }
    bool series_json = false, series_arg = false, scene_arg = false, json_arg = false;
    for (int index = 1; index < argc; ++index) {
      series_arg |= std::string(argv[index]) == "--series";
      scene_arg |= std::string(argv[index]) == "--scene" || std::string(argv[index]) == "--scene-info";
      json_arg |= std::string(argv[index]) == "--json";
    }
    series_json = (series_arg || scene_arg) && json_arg;
    if (series_json) {
      const auto *typed = dynamic_cast<const lingtu::recording::SeriesError *>(&error);
      std::cout << "{\"version\":1,\"ok\":false,\"error\":{\"code\":"
                << lingtu::recording::json_quote(typed ? typed->code : (scene_arg ? "recording_scene_invalid_query" : "recording_series_invalid_query"))
                << ",\"message\":" << lingtu::recording::json_quote(typed ? typed->what() : (scene_arg ? "Invalid recorded scene query." : "Invalid numeric series query.")) << "}}\n";
    } else std::cerr << "lingtu_dds_player: " << error.what() << "\n";
    return 2;
  }
}
