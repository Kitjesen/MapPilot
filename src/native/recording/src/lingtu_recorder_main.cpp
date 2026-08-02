#include <algorithm>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "lingtu/recording/recording_catalog.hpp"
#include "lingtu/recording/recording_core.hpp"
#include "lingtu/recording/recording_linux.hpp"

#ifndef LINGTU_RECORDING_HAS_DDS
#define LINGTU_RECORDING_HAS_DDS 0
#endif

namespace {

namespace recording = lingtu::recording;
volatile sig_atomic_t stop_signal_count = 0;

constexpr std::uint64_t kBytesPerGib = std::uint64_t{1} << 30U;

extern "C" void request_stop(int) {
  if (stop_signal_count < 2) {
    ++stop_signal_count;
  }
}

enum class Selection { kAuto, kOn, kOff };

struct Options {
  std::filesystem::path output_directory;
  std::string session_id;
  std::string product;
  std::string run_plan_fingerprint;
  std::string robot_id;
  std::string task_id;
  double seconds{0.0};
  std::uint64_t stop_grace_ms{5000};
  std::uint64_t minimum_free_bytes{5 * kBytesPerGib};
  Selection dds{Selection::kAuto};
  bool camera{true};
  int dds_domain{0};
  std::uint64_t dds_queue_mib{256};
  std::uint64_t dds_chunk_mib{4};
  std::filesystem::path dds_idl;
  std::string dds_preset{"generic-sensors-v1"};
  std::vector<std::string> dds_topics;
  std::string color_shm{"/lingtu_camera_color"};
  double camera_segment_seconds{5.0};
  double camera_fps{30.0};
  std::string camera_ffmpeg{"ffmpeg"};
  std::string camera_codec{"libx264"};
  std::filesystem::path dds_recorder;
  std::filesystem::path camera_recorder;
  bool dry_run{false};
};

struct StopOptions {
  std::filesystem::path session_directory;
  std::uint64_t timeout_ms{15000};
};

struct RootOptions {
  std::filesystem::path root;
  std::uint64_t timeout_ms{15000};
};

struct StartOptions {
  std::filesystem::path root;
  std::string prefix{"recording"};
  std::uint64_t startup_timeout_ms{5000};
  std::vector<std::string> record_arguments;
};

void print_help(std::ostream &output, const char *program) {
  output << "Usage:\n"
         << "  " << program << " record --output-dir DIR [options]\n"
         << "  " << program << " start --root RECORDING_ROOT [--prefix NAME] [options]\n"
         << "  " << program << " status SESSION_DIR\n"
         << "  " << program << " status --root RECORDING_ROOT\n"
         << "  " << program << " stop SESSION_DIR [--timeout-ms MS]\n"
         << "  " << program << " stop --root RECORDING_ROOT [--timeout-ms MS]\n\n"
         << "Native LingTu recording supervisor. It coordinates the existing DDS and\n"
         << "camera recorders as one durable session; it never invokes ROS or a shell.\n\n"
         << "Session options:\n"
         << "  --output-dir DIR             New session directory (required)\n"
         << "  --session-id ID              Default: output directory name\n"
         << "  --product NAME               Default: LINGTU_PRODUCT or manual\n"
         << "  --run-plan-fingerprint VALUE Default: LINGTU_RUN_PLAN_FINGERPRINT\n"
         << "  --robot-id ID                Default: LINGTU_ROBOT_ID or hostname\n"
         << "  --inspection-task-id ID     Verify one complete native inspection task\n"
         << "  --seconds SEC                0 waits for SIGINT/SIGTERM (default: 0)\n"
         << "  --stop-grace-ms MS           Whole-session stop deadline (default: 5000)\n"
         << "  --min-free-gib GIB           Startup storage gate (default: 5; 0 disables)\n"
         << "  --dry-run                    Print the child process plan without writing\n\n"
         << "Recorder selection:\n"
         << "  --dds auto|on|off            Default: auto (on in DDS-enabled builds)\n"
         << "  --camera on|off              Default: on\n\n"
         << "DDS options:\n"
         << "  --dds-domain N               Live observation domain (default: 0)\n"
         << "  --dds-preset NAME            generic-sensors-v1 or inspection-evidence-v1\n"
         << "  --dds-topic TOPIC            Repeatable explicit topic; requested topics must be "
            "captured\n"
         << "  --dds-queue-mib N            Default: 256\n"
         << "  --dds-chunk-mib N            Default: 4\n"
         << "  --dds-idl FILE               LingTu self-contained OMG IDL\n\n"
         << "Camera options:\n"
         << "  --camera-color-shm NAME      Default: /lingtu_camera_color\n"
         << "  --camera-segment-seconds N   Default: 5\n"
         << "  --camera-fps N               Default: 30\n"
         << "  --camera-ffmpeg PATH         Default: ffmpeg\n"
         << "  --camera-codec NAME          Default: libx264 (generic software)\n\n"
         << "Stop options:\n"
         << "  --timeout-ms MS              Wait for final manifest (default: 15000)\n\n"
         << "Exit status: 0 completed/live, 2 usage, 3 inspection/start error,\n"
         << "             4 failed/stale session, 5 stop timeout.\n";
}

std::string environment_or(const char *name, std::string fallback = {}) {
  const char *value = std::getenv(name);
  return value == nullptr ? std::move(fallback) : std::string(value);
}

std::string hostname() {
  std::vector<char> buffer(256, '\0');
  if (::gethostname(buffer.data(), buffer.size()) != 0) {
    return "unknown";
  }
  buffer.back() = '\0';
  return std::string(buffer.data());
}

std::string require_value(int &index, int argc, char **argv, std::string_view option) {
  if (++index >= argc) {
    throw std::invalid_argument(std::string(option) + " requires a value");
  }
  return argv[index];
}

std::uint64_t parse_uint64(const std::string &input, std::string_view option,
                           std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
  std::uint64_t value = 0;
  const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
  if (result.ec != std::errc{} || result.ptr != input.data() + input.size() || value > maximum) {
    throw std::invalid_argument(std::string(option) + " requires an unsigned integer in range");
  }
  return value;
}

double parse_nonnegative_double(const std::string &input, std::string_view option,
                                bool allow_zero) {
  char *end = nullptr;
  errno = 0;
  const double value = std::strtod(input.c_str(), &end);
  if (errno != 0 || end != input.c_str() + input.size() || !std::isfinite(value) || value < 0.0 ||
      (!allow_zero && value == 0.0)) {
    throw std::invalid_argument(std::string(option) + " requires a " +
                                (allow_zero ? "nonnegative" : "positive") + " number");
  }
  return value;
}

std::uint64_t parse_gibibytes(const std::string &input, std::string_view option) {
  const double gib = parse_nonnegative_double(input, option, true);
  const long double bytes = static_cast<long double>(gib) * kBytesPerGib;
  const long double byte_limit = std::ldexp(1.0L, std::numeric_limits<std::uint64_t>::digits);
  const long double rounded_bytes = std::ceil(bytes);
  if (rounded_bytes >= byte_limit) {
    throw std::invalid_argument(std::string(option) + " is too large");
  }
  return static_cast<std::uint64_t>(rounded_bytes);
}

Selection parse_selection(const std::string &value, std::string_view option, bool allow_auto) {
  if (allow_auto && value == "auto") {
    return Selection::kAuto;
  }
  if (value == "on") {
    return Selection::kOn;
  }
  if (value == "off") {
    return Selection::kOff;
  }
  throw std::invalid_argument(std::string(option) + " requires " +
                              (allow_auto ? "auto, on, or off" : "on or off"));
}

Options parse_record_options(int argc, char **argv) {
  Options options;
  options.product = environment_or("LINGTU_PRODUCT", "manual");
  options.run_plan_fingerprint = environment_or("LINGTU_RUN_PLAN_FINGERPRINT");
  options.robot_id = environment_or("LINGTU_ROBOT_ID", hostname());
  for (int index = 2; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "-h" || argument == "--help") {
      print_help(std::cout, argv[0]);
      std::exit(0);
    }
    if (argument == "--output-dir") {
      options.output_directory = require_value(index, argc, argv, argument);
    } else if (argument == "--session-id") {
      options.session_id = require_value(index, argc, argv, argument);
    } else if (argument == "--product") {
      options.product = require_value(index, argc, argv, argument);
    } else if (argument == "--run-plan-fingerprint") {
      options.run_plan_fingerprint = require_value(index, argc, argv, argument);
    } else if (argument == "--robot-id") {
      options.robot_id = require_value(index, argc, argv, argument);
    } else if (argument == "--inspection-task-id") {
      options.task_id = require_value(index, argc, argv, argument);
    } else if (argument == "--seconds") {
      options.seconds =
          parse_nonnegative_double(require_value(index, argc, argv, argument), argument, true);
    } else if (argument == "--stop-grace-ms") {
      options.stop_grace_ms =
          parse_uint64(require_value(index, argc, argv, argument), argument, 300'000);
    } else if (argument == "--min-free-gib") {
      options.minimum_free_bytes =
          parse_gibibytes(require_value(index, argc, argv, argument), argument);
    } else if (argument == "--dds") {
      options.dds = parse_selection(require_value(index, argc, argv, argument), argument, true);
    } else if (argument == "--camera") {
      options.camera = parse_selection(require_value(index, argc, argv, argument), argument,
                                       false) == Selection::kOn;
    } else if (argument == "--dds-domain") {
      options.dds_domain =
          static_cast<int>(parse_uint64(require_value(index, argc, argv, argument), argument, 232));
    } else if (argument == "--dds-topic") {
      options.dds_topics.push_back(require_value(index, argc, argv, argument));
    } else if (argument == "--dds-preset") {
      options.dds_preset = require_value(index, argc, argv, argument);
    } else if (argument == "--dds-queue-mib") {
      options.dds_queue_mib =
          parse_uint64(require_value(index, argc, argv, argument), argument, 16'384);
      if (options.dds_queue_mib == 0) {
        throw std::invalid_argument("--dds-queue-mib must be positive");
      }
    } else if (argument == "--dds-chunk-mib") {
      options.dds_chunk_mib =
          parse_uint64(require_value(index, argc, argv, argument), argument, 16'384);
      if (options.dds_chunk_mib == 0) {
        throw std::invalid_argument("--dds-chunk-mib must be positive");
      }
    } else if (argument == "--dds-idl") {
      options.dds_idl = require_value(index, argc, argv, argument);
    } else if (argument == "--camera-color-shm") {
      options.color_shm = require_value(index, argc, argv, argument);
    } else if (argument == "--camera-segment-seconds") {
      options.camera_segment_seconds =
          parse_nonnegative_double(require_value(index, argc, argv, argument), argument, false);
    } else if (argument == "--camera-fps") {
      options.camera_fps =
          parse_nonnegative_double(require_value(index, argc, argv, argument), argument, false);
    } else if (argument == "--camera-ffmpeg") {
      options.camera_ffmpeg = require_value(index, argc, argv, argument);
    } else if (argument == "--camera-codec") {
      options.camera_codec = require_value(index, argc, argv, argument);
    } else if (argument == "--dds-recorder") {
      options.dds_recorder = require_value(index, argc, argv, argument);
    } else if (argument == "--camera-recorder") {
      options.camera_recorder = require_value(index, argc, argv, argument);
    } else if (argument == "--dry-run") {
      options.dry_run = true;
    } else {
      throw std::invalid_argument("unknown record option: " + std::string(argument));
    }
  }
  if (options.output_directory.empty()) {
    throw std::invalid_argument("--output-dir is required");
  }
  options.output_directory = std::filesystem::absolute(options.output_directory).lexically_normal();
  if (options.session_id.empty()) {
    options.session_id = options.output_directory.filename().string();
  }
  if (options.session_id.empty()) {
    throw std::invalid_argument("--session-id is required when output has no directory name");
  }
  if (!options.task_id.empty() && options.dds_preset != "inspection-evidence-v1") {
    throw std::invalid_argument(
        "--inspection-task-id requires --dds-preset inspection-evidence-v1");
  }
  if (!options.task_id.empty() && options.product != "inspection") {
    throw std::invalid_argument("--inspection-task-id requires --product inspection");
  }
  if (!options.task_id.empty() && options.run_plan_fingerprint.empty()) {
    throw std::invalid_argument("--inspection-task-id requires --run-plan-fingerprint");
  }
  return options;
}

StopOptions parse_stop_options(int argc, char **argv) {
  if (argc < 3) {
    throw std::invalid_argument("stop requires one session directory");
  }
  StopOptions options;
  options.session_directory = argv[2];
  for (int index = 3; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--timeout-ms") {
      options.timeout_ms =
          parse_uint64(require_value(index, argc, argv, argument), argument, 300000);
    } else {
      throw std::invalid_argument("unknown stop option: " + std::string(argument));
    }
  }
  return options;
}

RootOptions parse_root_options(int argc, char **argv, std::string_view command,
                               bool allow_timeout) {
  RootOptions options;
  for (int index = 2; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--root") {
      if (!options.root.empty()) {
        throw std::invalid_argument(std::string(command) + " accepts --root only once");
      }
      options.root = require_value(index, argc, argv, argument);
    } else if (allow_timeout && argument == "--timeout-ms") {
      options.timeout_ms =
          parse_uint64(require_value(index, argc, argv, argument), argument, 300000);
    } else {
      throw std::invalid_argument("unknown " + std::string(command) +
                                  " option: " + std::string(argument));
    }
  }
  if (options.root.empty()) {
    throw std::invalid_argument(std::string(command) + " requires --root RECORDING_ROOT");
  }
  options.root = std::filesystem::absolute(options.root).lexically_normal();
  return options;
}

bool safe_session_prefix(std::string_view value) {
  if (value.empty() || value.size() > 40) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) != 0 || character == '-' || character == '_';
  });
}

StartOptions parse_start_options(int argc, char **argv) {
  StartOptions options;
  for (int index = 2; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--root") {
      if (!options.root.empty()) {
        throw std::invalid_argument("start accepts --root only once");
      }
      options.root = require_value(index, argc, argv, argument);
    } else if (argument == "--prefix") {
      options.prefix = require_value(index, argc, argv, argument);
    } else if (argument == "--startup-timeout-ms") {
      options.startup_timeout_ms =
          parse_uint64(require_value(index, argc, argv, argument), argument, 60000);
      if (options.startup_timeout_ms == 0) {
        throw std::invalid_argument("--startup-timeout-ms must be positive");
      }
    } else {
      if (argument == "--output-dir" || argument == "--session-id" || argument == "--dry-run") {
        throw std::invalid_argument("start owns " + std::string(argument));
      }
      options.record_arguments.emplace_back(argument);
    }
  }
  if (options.root.empty()) {
    throw std::invalid_argument("start requires --root RECORDING_ROOT");
  }
  if (!safe_session_prefix(options.prefix)) {
    throw std::invalid_argument("--prefix must use 1-40 ASCII letters, digits, '-' or '_'");
  }
  options.root = std::filesystem::absolute(options.root).lexically_normal();
  return options;
}

std::filesystem::path sibling_executable(const char *self, const char *name) {
  return recording::recording_executable_path(self).parent_path() / name;
}

std::string number(double value) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return output.str();
}

recording::RecordingSpec build_spec(const Options &options, const char *self) {
  const bool dds_enabled = options.dds == Selection::kOn ||
                           (options.dds == Selection::kAuto && LINGTU_RECORDING_HAS_DDS != 0);
  if (options.dds == Selection::kOn && LINGTU_RECORDING_HAS_DDS == 0) {
    throw std::invalid_argument("--dds on requires a DDS-enabled native recording build");
  }
  if (!dds_enabled && !options.camera) {
    throw std::invalid_argument("at least one of DDS or camera recording must be enabled");
  }
  if (!options.task_id.empty() && !dds_enabled) {
    throw std::invalid_argument("--inspection-task-id requires DDS recording");
  }

  recording::RecordingSpec spec;
  spec.session_directory = options.output_directory;
  spec.session_id = options.session_id;
  spec.context.product = options.product;
  spec.context.run_plan_fingerprint = options.run_plan_fingerprint;
  spec.context.robot_id = options.robot_id;
  spec.context.task_id = options.task_id;
  spec.minimum_free_bytes = options.minimum_free_bytes;

  if (dds_enabled) {
    const auto topic_plan = recording::dds_recording_plan(options.dds_preset, options.dds_topics);
    const auto &selected_topics = topic_plan.selected_topics;
    auto required_topics = topic_plan.required_topics;
    if (!options.task_id.empty()) {
      for (const auto *topic :
           {"/nav/inspection/task/event", "/nav/cmd_vel", "/driver/control_state"}) {
        if (std::find(selected_topics.begin(), selected_topics.end(), topic) ==
            selected_topics.end()) {
          throw std::invalid_argument("task-bound inspection recording requires selected topic " +
                                      std::string(topic));
        }
        if (std::find(required_topics.begin(), required_topics.end(), topic) ==
            required_topics.end()) {
          required_topics.emplace_back(topic);
        }
      }
    }
    const auto executable = options.dds_recorder.empty()
                                ? sibling_executable(self, "lingtu_dds_recorder")
                                : std::filesystem::absolute(options.dds_recorder);
    const auto idl_path =
        options.dds_idl.empty()
            ? recording::resolve_recording_idl(recording::recording_executable_path(self),
                                               LINGTU_RECORDING_DEFAULT_IDL)
            : options.dds_idl;
    std::vector<std::string> argv{
        executable.string(),
        "--output",
        "dds/sensors.mcap",
        "--domain",
        std::to_string(options.dds_domain),
        "--queue-mib",
        std::to_string(options.dds_queue_mib),
        "--chunk-mib",
        std::to_string(options.dds_chunk_mib),
        "--idl",
        std::filesystem::absolute(idl_path).string(),
    };
    for (const auto &required_topic : required_topics) {
      argv.push_back("--require-topic");
      argv.push_back(required_topic);
    }
    if (!options.task_id.empty()) {
      argv.push_back("--inspection-task-id");
      argv.push_back(options.task_id);
    }
    argv.insert(argv.end(), selected_topics.begin(), selected_topics.end());
    recording::RecordingChildSpec dds_child{"dds", std::move(argv), true, {"dds/sensors.mcap"}};
    dds_child.selected_topics = selected_topics;
    dds_child.required_topics = required_topics;
    spec.children.push_back(std::move(dds_child));
  }

  if (options.camera) {
    const auto executable = options.camera_recorder.empty()
                                ? sibling_executable(self, "lingtu_camera_recorder")
                                : std::filesystem::absolute(options.camera_recorder);
    spec.children.push_back(
        {"camera_color",
         {executable.string(), "--output-dir", ".", "--color-shm", options.color_shm,
          "--segment-seconds", number(options.camera_segment_seconds), "--fps",
          number(options.camera_fps), "--ffmpeg", options.camera_ffmpeg, "--codec",
          options.camera_codec},
         true,
         {"camera_color.mcap"}});
  }
  return spec;
}

void print_plan(const recording::RecordingSpec &spec) {
  std::cout << "session_id=" << spec.session_id << "\noutput=" << spec.session_directory.string()
            << "\nproduct=" << spec.context.product
            << "\nminimum_free_bytes=" << spec.minimum_free_bytes << "\n";
  for (const auto &child : spec.children) {
    std::cout << "child=" << child.name;
    for (const auto &argument : child.argv) {
      std::cout << ' ' << std::quoted(argument);
    }
    std::cout << '\n';
  }
}

std::string json_string(std::string_view value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

using ManifestSnapshot = recording::RecordingManifestSnapshot;

ManifestSnapshot read_manifest(const std::filesystem::path &session_directory) {
  return recording::read_recording_manifest(session_directory);
}

bool active_state(std::string_view state) {
  return recording::recording_state_is_active(state);
}

bool process_alive(pid_t process_id) {
  if (::kill(process_id, 0) == 0) {
    return true;
  }
  if (errno == ESRCH) {
    return false;
  }
  if (errno == EPERM) {
    return true;
  }
  throw std::runtime_error("failed to inspect recording manager process: " +
                           std::string(std::strerror(errno)));
}

std::vector<std::string> process_arguments(pid_t process_id) {
  const auto path = std::filesystem::path("/proc") / std::to_string(process_id) / "cmdline";
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to read recording manager command line");
  }
  constexpr std::size_t kMaximumCommandLineBytes = 1024U * 1024U;
  std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  if (bytes.empty() || bytes.size() > kMaximumCommandLineBytes) {
    throw std::runtime_error("recording manager command line size is invalid");
  }
  std::vector<std::string> arguments;
  std::size_t begin = 0;
  while (begin < bytes.size()) {
    const auto end = bytes.find('\0', begin);
    if (end == std::string::npos) {
      throw std::runtime_error("recording manager command line is malformed");
    }
    arguments.push_back(bytes.substr(begin, end - begin));
    begin = end + 1;
  }
  return arguments;
}

std::optional<std::string> manager_identity_error(pid_t process_id,
                                                  const std::filesystem::path &session_directory) {
  try {
    if (!process_alive(process_id)) {
      return "recording manager process is no longer running";
    }
    std::error_code error;
    const auto own_executable = std::filesystem::canonical("/proc/self/exe", error);
    if (error) {
      return "cannot resolve the current lingtu_recorder executable";
    }
    const auto target_executable = std::filesystem::canonical(
        std::filesystem::path("/proc") / std::to_string(process_id) / "exe", error);
    if (error || target_executable != own_executable) {
      return "manifest PID does not identify this lingtu_recorder executable";
    }

    const auto arguments = process_arguments(process_id);
    if (arguments.size() < 2 || arguments[1] != "record") {
      return "manifest PID is not running the record command";
    }
    std::optional<std::filesystem::path> requested_output;
    for (std::size_t index = 2; index + 1 < arguments.size(); ++index) {
      if (arguments[index] == "--output-dir") {
        requested_output = arguments[index + 1];
        break;
      }
    }
    if (!requested_output) {
      return "recording manager command has no output directory";
    }
    if (requested_output->is_relative()) {
      const auto target_cwd = std::filesystem::read_symlink(
          std::filesystem::path("/proc") / std::to_string(process_id) / "cwd", error);
      if (error) {
        return "cannot resolve the recording manager working directory";
      }
      *requested_output = target_cwd / *requested_output;
    }
    const auto actual_output = std::filesystem::weakly_canonical(*requested_output, error);
    if (error) {
      return "cannot resolve the recording manager output directory";
    }
    const auto expected_output = std::filesystem::weakly_canonical(session_directory, error);
    if (error || actual_output != expected_output) {
      return "manifest PID belongs to a different recording session";
    }
  } catch (const std::exception &error) {
    return error.what();
  }
  return std::nullopt;
}

int print_catalog_status(const recording::RecordingCatalogSnapshot &catalog) {
  bool healthy = true;
  std::string error;
  std::string_view state = "idle";
  if (catalog.selected) {
    state = catalog.selected->state;
    if (state == "failed") {
      healthy = false;
      error = "recording session failed";
    } else if (active_state(state)) {
      const auto identity_error =
          manager_identity_error(static_cast<pid_t>(catalog.selected->manager_process_id),
                                 catalog.selected->session_directory);
      if (identity_error) {
        healthy = false;
        error = *identity_error;
      }
    }
  }

  std::cout << "{\"control_version\":1,\"ok\":true,\"healthy\":" << (healthy ? "true" : "false")
            << ",\"state\":" << json_string(state)
            << ",\"root\":" << json_string(catalog.root.generic_string()) << ",\"session\":";
  if (catalog.selected) {
    std::cout << catalog.selected->manifest_json;
  } else {
    std::cout << "null";
  }
  std::cout << ",\"size_bytes\":" << catalog.selected_size_bytes
            << ",\"size_truncated\":" << (catalog.selected_size_truncated ? "true" : "false")
            << ",\"disk_free\":" << catalog.disk_free_bytes
            << ",\"disk_total\":" << catalog.disk_total_bytes << ",\"error\":";
  if (error.empty()) {
    std::cout << "null";
  } else {
    std::cout << json_string(error);
  }
  std::cout << "}\n";
  return healthy ? 0 : 4;
}

int print_root_status(const std::filesystem::path &root) {
  return print_catalog_status(recording::inspect_recording_catalog(root));
}

void print_control_error(const recording::RecordingCatalogError &error) {
  std::cout << "{\"control_version\":1,\"ok\":false,\"error\":{\"code\":"
            << json_string(error.code()) << ",\"message\":" << json_string(error.what()) << "}}\n";
}

class RootControlLock {
 public:
  explicit RootControlLock(const std::filesystem::path &root) {
    std::error_code error;
    std::filesystem::create_directories(root, error);
    if (error) {
      throw recording::RecordingCatalogError(
          "recording_catalog_unreadable", "recording root cannot be created: " + error.message());
    }
    const auto status = std::filesystem::symlink_status(root, error);
    if (error || std::filesystem::is_symlink(status) || !std::filesystem::is_directory(status)) {
      throw recording::RecordingCatalogError("recording_catalog_unsafe",
                                             "recording root is not a safe directory");
    }
    const int root_fd = ::open(root.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (root_fd < 0) {
      throw recording::RecordingCatalogError("recording_catalog_unreadable",
                                             "recording root cannot be opened: " +
                                                 std::string(std::strerror(errno)));
    }
    descriptor_ =
        ::openat(root_fd, ".control.lock", O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    const int open_error = errno;
    ::close(root_fd);
    if (descriptor_ < 0) {
      throw recording::RecordingCatalogError("recording_catalog_unreadable",
                                             "recording control lock cannot be opened: " +
                                                 std::string(std::strerror(open_error)));
    }
    if (::flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
      const int lock_error = errno;
      ::close(descriptor_);
      descriptor_ = -1;
      throw recording::RecordingCatalogError("recording_control_busy",
                                             "recording control lock cannot be acquired: " +
                                                 std::string(std::strerror(lock_error)));
    }
  }

  ~RootControlLock() {
    if (descriptor_ >= 0) {
      static_cast<void>(::flock(descriptor_, LOCK_UN));
      ::close(descriptor_);
    }
  }

  RootControlLock(const RootControlLock &) = delete;
  RootControlLock &operator=(const RootControlLock &) = delete;

 private:
  int descriptor_{-1};
};

std::string generated_session_id(std::string_view prefix) {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (::gmtime_r(&time, &utc) == nullptr) {
    throw recording::RecordingCatalogError("native_recorder_start_failed",
                                           "UTC session timestamp cannot be generated");
  }
  const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  std::ostringstream output;
  output << prefix << '_' << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << '_' << ::getpid() << '_'
         << std::hex << (static_cast<std::uint64_t>(nanoseconds) & 0xffffffffU);
  return output.str();
}

std::vector<std::string> record_command(const StartOptions &options, const char *self,
                                        const std::filesystem::path &session_directory,
                                        const std::string &session_id) {
  std::vector<std::string> command{recording::recording_executable_path(self).string(),
                                   "record",
                                   "--output-dir",
                                   session_directory.string(),
                                   "--session-id",
                                   session_id};
  command.insert(command.end(), options.record_arguments.begin(), options.record_arguments.end());

  std::vector<char *> arguments;
  arguments.reserve(command.size());
  for (auto &argument : command) {
    arguments.push_back(argument.data());
  }
  const auto parsed = parse_record_options(static_cast<int>(arguments.size()), arguments.data());
  static_cast<void>(build_spec(parsed, self));
  return command;
}

pid_t launch_detached_recording(const std::vector<std::string> &command,
                                const std::filesystem::path &manager_log) {
  const int log_fd =
      ::open(manager_log.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (log_fd < 0) {
    throw recording::RecordingCatalogError("native_recorder_start_failed",
                                           "manager log cannot be created: " +
                                               std::string(std::strerror(errno)));
  }
  const int null_fd = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
  if (null_fd < 0) {
    const int open_error = errno;
    ::close(log_fd);
    throw recording::RecordingCatalogError("native_recorder_start_failed",
                                           "standard input cannot be detached: " +
                                               std::string(std::strerror(open_error)));
  }
  const pid_t child = ::fork();
  if (child < 0) {
    const int fork_error = errno;
    ::close(null_fd);
    ::close(log_fd);
    throw recording::RecordingCatalogError("native_recorder_start_failed",
                                           "recording manager cannot be forked: " +
                                               std::string(std::strerror(fork_error)));
  }
  if (child == 0) {
    if (::setsid() < 0 || ::dup2(null_fd, STDIN_FILENO) < 0 || ::dup2(log_fd, STDOUT_FILENO) < 0 ||
        ::dup2(log_fd, STDERR_FILENO) < 0) {
      ::_exit(126);
    }
    ::close(null_fd);
    ::close(log_fd);
    std::vector<char *> arguments;
    arguments.reserve(command.size() + 1);
    for (const auto &argument : command) {
      arguments.push_back(const_cast<char *>(argument.c_str()));
    }
    arguments.push_back(nullptr);
    ::execv(arguments.front(), arguments.data());
    ::_exit(127);
  }
  ::close(null_fd);
  ::close(log_fd);
  return child;
}

void terminate_starting_manager(pid_t process_id) {
  static_cast<void>(::kill(process_id, SIGTERM));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  int status = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    if (::waitpid(process_id, &status, WNOHANG) == process_id) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  static_cast<void>(::kill(process_id, SIGKILL));
  while (::waitpid(process_id, &status, 0) < 0 && errno == EINTR) {}
}

int start_root_recording(const StartOptions &options, const char *self) {
  RootControlLock lock(options.root);
  const auto existing = recording::inspect_recording_catalog(options.root);
  if (existing.selected && active_state(existing.selected->state)) {
    throw recording::RecordingCatalogError("recording_in_progress",
                                           "recording catalog already contains an active session");
  }

  const auto session_id = generated_session_id(options.prefix);
  const auto session_directory = options.root / session_id;
  if (std::filesystem::exists(session_directory)) {
    throw recording::RecordingCatalogError("native_recorder_start_failed",
                                           "generated recording session already exists");
  }
  const auto command = record_command(options, self, session_directory, session_id);
  const auto temporary_log = options.root / ("." + session_id + ".manager.log");
  const pid_t child = launch_detached_recording(command, temporary_log);
  bool log_promoted = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(options.startup_timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    std::error_code error;
    if (!log_promoted && std::filesystem::is_directory(session_directory / "logs", error)) {
      std::filesystem::rename(temporary_log, session_directory / "logs" / "manager.log", error);
      log_promoted = !error;
    }
    const auto catalog = recording::inspect_recording_catalog(options.root);
    if (catalog.selected && catalog.selected->session_id == session_id) {
      if (catalog.selected->manager_process_id != child) {
        terminate_starting_manager(child);
        throw recording::RecordingCatalogError("native_recorder_start_failed",
                                               "recording manager identity changed during startup");
      }
      if (catalog.selected->state == "recording" && log_promoted) {
        return print_catalog_status(catalog);
      }
      if (recording::recording_state_is_terminal(catalog.selected->state)) {
        if (!log_promoted) {
          throw recording::RecordingCatalogError(
              "native_recorder_start_failed",
              "recording manager terminated before its control log was published");
        }
        return print_catalog_status(catalog);
      }
    } else if (catalog.selected && active_state(catalog.selected->state)) {
      terminate_starting_manager(child);
      throw recording::RecordingCatalogError("recording_in_progress",
                                             "another recording session won the startup race");
    }

    int child_status = 0;
    const pid_t wait_result = ::waitpid(child, &child_status, WNOHANG);
    if (wait_result == child) {
      throw recording::RecordingCatalogError(
          "native_recorder_start_failed",
          "recording manager exited before publishing an active session");
    }
    if (wait_result < 0 && errno != EINTR) {
      terminate_starting_manager(child);
      throw recording::RecordingCatalogError("native_recorder_start_failed",
                                             "recording manager cannot be observed: " +
                                                 std::string(std::strerror(errno)));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  terminate_starting_manager(child);
  throw recording::RecordingCatalogError("native_recorder_start_timeout",
                                         "recording manager did not become ready before timeout");
}

void install_signal_handlers() {
  struct sigaction action{};
  action.sa_handler = request_stop;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (::sigaction(SIGINT, &action, nullptr) != 0 || ::sigaction(SIGTERM, &action, nullptr) != 0) {
    throw std::runtime_error("failed to install recording signal handlers");
  }
}

int record(const Options &options, const char *self) {
  auto spec = build_spec(options, self);
  if (options.dry_run) {
    print_plan(spec);
    return 0;
  }
  install_signal_handlers();
  recording::PosixRecordingProcessFactory process_factory;
  recording::RecordingManager manager(process_factory);
  const auto started = manager.start(spec);
  std::cout << "session=" << started.session_id
            << " state=" << recording::recording_state_name(started.state)
            << " manifest=" << (started.session_directory / "session.json").string() << '\n';

  auto deadline = std::chrono::steady_clock::time_point::max();
  if (options.seconds > 0.0) {
    deadline = std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(options.seconds));
  }
  while (stop_signal_count == 0 && std::chrono::steady_clock::now() < deadline) {
    const auto current = manager.status();
    if (current.state == recording::RecordingState::kFailed) {
      std::cerr << "lingtu_recorder: " << current.error << '\n';
      return 4;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  const auto grace = stop_signal_count > 1 ? std::chrono::milliseconds(0)
                                           : std::chrono::milliseconds(options.stop_grace_ms);
  const auto completed = manager.stop(grace);
  std::cout << "session=" << completed.session_id
            << " state=" << recording::recording_state_name(completed.state) << '\n';
  if (completed.state != recording::RecordingState::kCompleted) {
    std::cerr << "lingtu_recorder: " << completed.error << '\n';
    return 4;
  }
  return 0;
}

int print_status(const std::filesystem::path &session_directory) {
  const auto snapshot = read_manifest(session_directory);
  std::cout << snapshot.manifest_json;
  if (snapshot.state == "failed") {
    return 4;
  }
  if (active_state(snapshot.state)) {
    if (const auto identity_error = manager_identity_error(
            static_cast<pid_t>(snapshot.manager_process_id), session_directory)) {
      std::cerr << "lingtu_recorder: stale session: " << *identity_error << '\n';
      return 4;
    }
    return 0;
  }
  if (snapshot.state != "completed") {
    throw std::runtime_error("recording manifest has an unknown state: " + snapshot.state);
  }
  return 0;
}

int stop_recording(const StopOptions &options, bool print_manifest = true) {
  auto snapshot = read_manifest(options.session_directory);
  if (snapshot.state == "completed") {
    if (print_manifest) {
      std::cout << snapshot.manifest_json;
    }
    return 0;
  }
  if (snapshot.state == "failed") {
    if (print_manifest) {
      std::cout << snapshot.manifest_json;
    }
    std::cerr << "lingtu_recorder: recording session has already failed\n";
    return 4;
  }
  if (!active_state(snapshot.state)) {
    throw std::runtime_error("recording manifest has an unknown state: " + snapshot.state);
  }
  if (const auto identity_error = manager_identity_error(
          static_cast<pid_t>(snapshot.manager_process_id), options.session_directory)) {
    std::cerr << "lingtu_recorder: refusing to signal a stale session: " << *identity_error << '\n';
    return 4;
  }

  if (snapshot.state != "stopping" &&
      ::kill(static_cast<pid_t>(snapshot.manager_process_id), SIGTERM) != 0) {
    if (errno == ESRCH) {
      std::cerr << "lingtu_recorder: recording manager exited before it could be stopped\n";
      return 4;
    }
    throw std::runtime_error("failed to signal recording manager: " +
                             std::string(std::strerror(errno)));
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(options.timeout_ms);
  while (true) {
    snapshot = read_manifest(options.session_directory);
    if (snapshot.state == "completed") {
      if (print_manifest) {
        std::cout << snapshot.manifest_json;
      }
      return 0;
    }
    if (snapshot.state == "failed") {
      if (print_manifest) {
        std::cout << snapshot.manifest_json;
      }
      std::cerr << "lingtu_recorder: recording session failed while stopping\n";
      return 4;
    }
    if (!active_state(snapshot.state)) {
      throw std::runtime_error("recording manifest has an unknown state: " + snapshot.state);
    }
    if (!process_alive(static_cast<pid_t>(snapshot.manager_process_id))) {
      std::cerr << "lingtu_recorder: recording manager exited without a final manifest\n";
      return 4;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      std::cerr << "lingtu_recorder: timed out waiting for the recording session to stop\n";
      return 5;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int stop_root_recording(const RootOptions &options) {
  RootControlLock lock(options.root);
  const auto catalog = recording::inspect_recording_catalog(options.root);
  if (!catalog.selected || !active_state(catalog.selected->state)) {
    throw recording::RecordingCatalogError("not_recording",
                                           "recording catalog has no active session");
  }
  StopOptions stop_options;
  stop_options.session_directory = catalog.selected->session_directory;
  stop_options.timeout_ms = options.timeout_ms;
  const int result = stop_recording(stop_options, false);
  const int status_result = print_root_status(options.root);
  return result != 0 ? result : status_result;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc < 2 || std::string_view(argv[1]) == "-h" || std::string_view(argv[1]) == "--help") {
      print_help(std::cout, argv[0]);
      return argc < 2 ? 2 : 0;
    }
    const std::string_view command(argv[1]);
    if (command == "start") {
      return start_root_recording(parse_start_options(argc, argv), argv[0]);
    }
    if (command == "record") {
      return record(parse_record_options(argc, argv), argv[0]);
    }
    if (command == "status") {
      if (argc >= 3 && std::string_view(argv[2]) == "--root") {
        return print_root_status(parse_root_options(argc, argv, command, false).root);
      }
      if (argc != 3) {
        throw std::invalid_argument("status requires exactly one session directory");
      }
      return print_status(argv[2]);
    }
    if (command == "stop") {
      if (argc >= 3 && std::string_view(argv[2]) == "--root") {
        return stop_root_recording(parse_root_options(argc, argv, command, true));
      }
      return stop_recording(parse_stop_options(argc, argv));
    }
    throw std::invalid_argument("unknown command: " + std::string(command));
  } catch (const recording::RecordingCatalogError &error) {
    print_control_error(error);
    return 4;
  } catch (const std::invalid_argument &error) {
    std::cerr << "lingtu_recorder: " << error.what() << '\n';
    return 2;
  } catch (const std::exception &error) {
    std::cerr << "lingtu_recorder: " << error.what() << '\n';
    return 3;
  }
}
