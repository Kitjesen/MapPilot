#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "lingtu/recording/cyclone_cdr.hpp"
#include "lingtu/recording/inspection_timeline_cdr.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "lingtu/recording/recording_core.hpp"
#include "lingtu/recording/topic_catalog.hpp"
#include "message/cpp/topics.hpp"

namespace {

std::atomic<bool> stop_requested{false};

void request_stop(int) {
  stop_requested.store(true);
}

struct Options {
  std::filesystem::path output;
  std::filesystem::path idl_path;
  int domain_id{0};
  double seconds{0.0};
  std::size_t queue_bytes{256U * 1024U * 1024U};
  std::uint64_t chunk_bytes{4U * 1024U * 1024U};
  std::vector<std::string> topics;
  std::vector<std::string> required_topics;
  std::string inspection_task_id;
};

void print_help() {
  std::cout << "usage: lingtu_dds_recorder --output FILE [options] [TOPIC ...]\n"
            << "\n"
            << "Native typed CycloneDDS recorder. Default topics are /lidar/raw_frame, /imu/raw,\n"
            << "/slam/odometry, and /slam/registered_cloud.\n\n"
            << "  --domain N       DDS domain to observe (default: 0)\n"
            << "  --seconds N      Stop after N seconds; 0 waits for SIGINT/SIGTERM\n"
            << "  --queue-mib N    Bounded CDR queue size (default: 256)\n"
            << "  --chunk-mib N    Uncompressed MCAP chunk target (default: 4)\n"
            << "  --require-topic TOPIC\n"
            << "                    Fail unless TOPIC contributes at least one sample\n"
            << "  --inspection-task-id ID\n"
            << "                    Require one complete task timeline and stop evidence\n"
            << "  --idl FILE       Self-contained LingTu OMG IDL schema\n";
}

std::size_t mib(const std::string &value, const char *option) {
  const auto count = std::stoull(value);
  if (count == 0 || count > 16384) {
    throw std::invalid_argument(std::string(option) + " must be between 1 and 16384");
  }
  return static_cast<std::size_t>(count * 1024ULL * 1024ULL);
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&]() -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument("missing value for " + argument);
      }
      return argv[index];
    };
    if (argument == "--output") {
      options.output = value();
    } else if (argument == "--domain") {
      options.domain_id = std::stoi(value());
    } else if (argument == "--seconds") {
      options.seconds = std::stod(value());
    } else if (argument == "--queue-mib") {
      options.queue_bytes = mib(value(), "--queue-mib");
    } else if (argument == "--chunk-mib") {
      options.chunk_bytes = mib(value(), "--chunk-mib");
    } else if (argument == "--idl") {
      options.idl_path = value();
    } else if (argument == "--require-topic") {
      options.required_topics.push_back(value());
    } else if (argument == "--inspection-task-id") {
      options.inspection_task_id = value();
    } else if (argument == "-h" || argument == "--help") {
      print_help();
      std::exit(0);
    } else if (!argument.empty() && argument.front() == '-') {
      throw std::invalid_argument("unknown option: " + argument);
    } else {
      options.topics.push_back(argument);
    }
  }
  if (options.output.empty()) {
    throw std::invalid_argument("--output is required");
  }
  if (options.seconds < 0.0) {
    throw std::invalid_argument("--seconds must not be negative");
  }
  if (options.domain_id < 0 || options.domain_id > 232) {
    throw std::invalid_argument("DDS domain must be between 0 and 232");
  }
  if (options.inspection_task_id.find('\0') != std::string::npos ||
      options.inspection_task_id.size() > 256) {
    throw std::invalid_argument("--inspection-task-id must be at most 256 bytes");
  }
  return options;
}

dds_entity_t checked(dds_return_t result, const char *operation) {
  if (result < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-result));
  }
  return static_cast<dds_entity_t>(result);
}

struct ReaderState {
  const lingtu::recording::TopicBinding *binding{nullptr};
  dds_entity_t topic{DDS_RETCODE_ERROR};
  dds_entity_t reader{DDS_RETCODE_ERROR};
  std::uint32_t sequence{0};
  std::uint64_t captured{0};
};

}  // namespace

int main(int argc, char **argv) {
  dds_entity_t participant = DDS_RETCODE_ERROR;
  try {
    const Options options = parse_options(argc, argv);
    const auto idl_path =
        options.idl_path.empty()
            ? lingtu::recording::resolve_recording_idl(
                  lingtu::recording::recording_executable_path(argv[0]), LINGTU_RECORDING_DEFAULT_IDL)
            : options.idl_path;
    std::vector<const lingtu::recording::TopicBinding *> bindings;
    if (options.topics.empty()) {
      bindings = lingtu::recording::default_recording_topics();
    } else {
      for (const auto &topic : options.topics) {
        const auto *binding = lingtu::recording::find_recording_topic(topic);
        if (binding == nullptr) {
          throw std::invalid_argument("topic is not in the native recording allowlist: " + topic);
        }
        bindings.push_back(binding);
      }
    }
    auto required_topics = options.required_topics;
    if (!options.inspection_task_id.empty()) {
      for (const auto *topic :
           {"/nav/inspection/task/event", "/nav/cmd_vel", "/driver/control_state"}) {
        if (std::find(required_topics.begin(), required_topics.end(), topic) ==
            required_topics.end()) {
          required_topics.emplace_back(topic);
        }
      }
    }
    for (const auto &required_topic : required_topics) {
      const auto *required_binding = lingtu::recording::find_recording_topic(required_topic);
      if (required_binding == nullptr) {
        throw std::invalid_argument("required topic is not in the native recording allowlist: " +
                                    required_topic);
      }
      const auto selected =
          std::any_of(bindings.begin(), bindings.end(), [required_binding](const auto *binding) {
            return binding != nullptr && binding->contract == required_binding->contract;
          });
      if (!selected) {
        throw std::invalid_argument("required topic was not selected for recording: " +
                                    required_topic);
      }
    }

    std::vector<lingtu::recording::ChannelDefinition> channels;
    channels.reserve(bindings.size());
    for (const auto *binding : bindings) {
      if (binding == nullptr || binding->contract == nullptr || binding->descriptor == nullptr) {
        throw std::logic_error("recording topic catalog contains an invalid binding");
      }
      channels.push_back(lingtu::recording::channel_definition(*binding));
    }

    lingtu::recording::McapSessionWriter storage(
        options.output, lingtu::recording::read_text_file(idl_path), std::move(channels),
        options.chunk_bytes);
    lingtu::recording::BoundedMessageQueue queue(options.queue_bytes);
    lingtu::recording::InspectionTimelineCapture timeline_capture;

    std::exception_ptr writer_error;
    std::thread writer_thread([&] {
      try {
        lingtu::recording::RecordedMessage message;
        while (queue.pop(message)) {
          storage.write(message);
        }
      } catch (...) {
        writer_error = std::current_exception();
        stop_requested.store(true);
      }
    });

    std::exception_ptr capture_error;
    std::uint64_t captured_total = 0;
    try {
      participant = checked(
          dds_create_participant(static_cast<dds_domainid_t>(options.domain_id), nullptr, nullptr),
          "dds_create_participant");
      std::vector<ReaderState> readers;
      readers.reserve(bindings.size());
      for (const auto *binding : bindings) {
        ReaderState state;
        state.binding = binding;
        const std::string wire_topic(binding->contract->dds_topic);
        state.topic = checked(dds_create_topic(participant, binding->descriptor, wire_topic.c_str(),
                                               nullptr, nullptr),
                              "dds_create_topic");
        auto observer_qos = lingtu::dds::make_qos(binding->qos_profile);
        state.reader =
            checked(dds_create_reader(participant, state.topic, observer_qos.get(), nullptr),
                    "dds_create_reader");
        readers.push_back(state);
      }

      std::signal(SIGINT, request_stop);
      std::signal(SIGTERM, request_stop);
      const auto started = std::chrono::steady_clock::now();
      const auto deadline = options.seconds > 0.0
                                ? started + std::chrono::duration<double>(options.seconds)
                                : std::chrono::steady_clock::time_point::max();

      while (!stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
        bool received = false;
        for (auto &reader : readers) {
          auto messages = lingtu::recording::take_cdr_messages(
              reader.reader, std::string(reader.binding->contract->dds_topic), reader.sequence);
          received = received || !messages.empty();
          reader.captured += messages.size();
          for (auto &message : messages) {
            if (!options.inspection_task_id.empty()) {
              timeline_capture.observe(*reader.binding, message);
            }
            queue.try_push(std::move(message));
          }
        }
        if (!received) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }

      if (stop_requested.load() && !options.inspection_task_id.empty() &&
          !timeline_capture.verify(options.inspection_task_id).ok) {
        std::vector<lingtu::recording::CdrDrainReader> drain_readers;
        std::vector<std::size_t> drain_reader_indexes;
        drain_readers.reserve(readers.size());
        drain_reader_indexes.reserve(readers.size());
        for (std::size_t index = 0; index < readers.size(); ++index) {
          auto &reader = readers[index];
          const auto *contract = reader.binding->contract;
          if (contract != &lingtu::message::kNavInspectionTaskEvent &&
              contract != &lingtu::message::kNavCmdVel &&
              contract != &lingtu::message::kDriverControlState) {
            continue;
          }
          drain_readers.push_back({reader.reader,
                                   std::string(reader.binding->contract->dds_topic),
                                   &reader.sequence});
          drain_reader_indexes.push_back(index);
        }
        static_cast<void>(lingtu::recording::bounded_drain_cdr_messages(
            drain_readers, std::chrono::milliseconds(500),
            [&] { return timeline_capture.verify(options.inspection_task_id).ok; },
            [&](std::size_t index, lingtu::recording::RecordedMessage &&message) {
              auto &reader = readers.at(drain_reader_indexes.at(index));
              ++reader.captured;
              timeline_capture.observe(*reader.binding, message);
              queue.try_push(std::move(message));
            }));
      }

      for (const auto &reader : readers) {
        captured_total += reader.captured;
        std::cout << reader.binding->contract->topic << " samples=" << reader.captured << "\n";
      }
      for (const auto &required_topic : required_topics) {
        const auto *required_binding = lingtu::recording::find_recording_topic(required_topic);
        const auto reader = std::find_if(
            readers.begin(), readers.end(), [required_binding](const ReaderState &state) {
              return state.binding != nullptr &&
                     state.binding->contract == required_binding->contract;
            });
        if (reader == readers.end() || reader->captured == 0) {
          throw std::runtime_error("required recording topic captured no samples: " +
                                   required_topic);
        }
      }
    } catch (...) {
      capture_error = std::current_exception();
    }

    if (participant > 0) {
      dds_delete(participant);
      participant = DDS_RETCODE_ERROR;
    }
    queue.close();
    writer_thread.join();
    if (capture_error) {
      std::rethrow_exception(capture_error);
    }
    if (writer_error) {
      std::rethrow_exception(writer_error);
    }
    if (captured_total == 0) {
      throw std::runtime_error("no sensor samples were captured");
    }
    if (queue.dropped_messages() != 0) {
      std::cerr << "lingtu_dds_recorder: bounded queue dropped " << queue.dropped_messages()
                << " messages; final MCAP was not published\n";
      return 4;
    }
    if (!options.inspection_task_id.empty()) {
      const auto report = timeline_capture.verify(options.inspection_task_id);
      if (!report.ok) {
        throw std::runtime_error("inspection task timeline verification failed: " +
                                 report.summary());
      }
      std::cout << "inspection_task_id=" << options.inspection_task_id
                << " terminal_state=" << report.terminal_state
                << " confirmed_output_sequence=" << report.confirmed_output_sequence << "\n";
    }

    storage.commit();
    std::cout << "output=" << storage.final_path() << " dropped=" << queue.dropped_messages()
              << " queue_high_watermark_bytes=" << queue.high_watermark_bytes() << "\n";
    return 0;
  } catch (const std::exception &error) {
    if (participant > 0) {
      dds_delete(participant);
    }
    std::cerr << "lingtu_dds_recorder: " << error.what() << "\n";
    return 2;
  }
}
