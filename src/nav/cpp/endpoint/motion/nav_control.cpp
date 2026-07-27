#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "nav/cpp/client/client.hpp"

namespace {

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header &header, double stamp_s, const char *frame_id) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char *>(frame_id);
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

lingtu::dds::UniqueQos qosFor(const lingtu::message::TopicContract &contract) {
  return lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
}

void waitForMatchedReader(dds_entity_t writer, const char *label, double timeout_s = 3.0) {
  const double deadline = nowSeconds() + timeout_s;
  while (nowSeconds() < deadline) {
    const dds_return_t count = dds_get_matched_subscriptions(writer, nullptr, 0);
    if (count > 0) {
      return;
    }
    if (count < 0) {
      throw std::runtime_error(std::string("dds_get_matched_subscriptions(") + label +
                               "): " + dds_strretcode(-count));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  throw std::runtime_error(std::string("no matched DDS reader for ") + label);
}

constexpr const char *kUsage =
    "usage: lingtu_nav_control goal X Y [Z YAW] [--domain-id N] [--timeout-ms N] | "
    "teleop VX VY WZ [--duration-s S] [--rate-hz HZ] [--domain-id N] | "
    "operator-motion VX VY WZ [--duration-s S] [--rate-hz HZ] [--domain-id N] "
    "[--source-id ID] [--source-epoch N] [--lease-ttl-ms N] "
    "[--freshness-budget-ms N] | "
    "teleop-stream [--rate-hz HZ] [--input-timeout-ms N] [--domain-id N] "
    "[--ready-file PATH] | cloud X Y Z [HEIGHT] [--domain-id N] | "
    "trav COST [--domain-id N] | "
    "path X1 Y1 Z1 X2 Y2 Z2 [X Y Z ...] [--domain-id N] | "
    "clear <map|cloud|all> [--domain-id N] | cancel [REASON] [--domain-id N] | "
    "stop [REASON] [--domain-id N] | estop [REASON] [--domain-id N] | "
    "clear-estop [REASON] [--domain-id N] | resume [REASON] [--domain-id N] | "
    "explore start SESSION_ID [REASON] [--request-id ID] [--domain-id N] | "
    "explore <pause|resume|stop> [REASON] [--request-id ID] [--domain-id N] | "
    "instruction TEXT [--domain-id N]";

struct CliConfig {
  std::string command;
  std::string text;
  std::string request_id;
  std::string session_id;
  std::vector<lingtu_dds_Point> path;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double cost = 100.0;
  double resolution = 0.1;
  double origin_x = -2.0;
  double origin_y = -2.0;
  double duration_s = 0.0;
  double rate_hz = 20.0;
  int timeout_ms = 3000;
  int input_timeout_ms = 350;
  int cleanup_settle_ms = 300;
  std::uint32_t lease_ttl_ms = 1000;
  std::uint32_t freshness_budget_ms = 350;
  std::uint64_t source_epoch = 0;
  std::string source_id{"lingtu-nav-control"};
  std::string ready_file;
  std::uint32_t width = 50;
  std::uint32_t height = 50;
  int domain_id = 0;
};

CliConfig parseArgs(int argc, char **argv) {
  if (argc < 2) {
    throw std::runtime_error(kUsage);
  }
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    std::printf("%s\n", kUsage);
    std::exit(0);
  }
  CliConfig cfg;
  cfg.command = argv[1];
  int i = 2;
  if (cfg.command == "explore") {
    if (i >= argc) {
      throw std::runtime_error("explore requires start, pause, resume, or stop");
    }
    const std::string action = argv[i++];
    if (action != "start" && action != "pause" && action != "resume" && action != "stop") {
      throw std::runtime_error("unsupported explore action: " + action);
    }
    cfg.command += "-" + action;
  }
  if (cfg.command == "goal") {
    if (argc < 4) {
      throw std::runtime_error("goal requires X and Y");
    }
    cfg.x = std::stod(argv[i++]);
    cfg.y = std::stod(argv[i++]);
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.z = std::stod(argv[i++]);
    }
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.yaw = std::stod(argv[i++]);
    }
  } else if (cfg.command == "teleop" || cfg.command == "operator-motion") {
    if (argc < 5) {
      throw std::runtime_error("teleop requires VX VY WZ");
    }
    cfg.x = std::stod(argv[i++]);
    cfg.y = std::stod(argv[i++]);
    cfg.yaw = std::stod(argv[i++]);
  } else if (cfg.command == "teleop-stream") {
    // Commands arrive on stdin as `VX VY WZ` lines. `quit [REASON]`
    // performs a typed stop and exits cleanly.
  } else if (cfg.command == "cloud") {
    if (argc < 5) {
      throw std::runtime_error("cloud requires X Y Z");
    }
    cfg.x = std::stod(argv[i++]);
    cfg.y = std::stod(argv[i++]);
    cfg.z = std::stod(argv[i++]);
    cfg.cost = cfg.z;
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.cost = std::stod(argv[i++]);
    }
  } else if (cfg.command == "trav") {
    if (argc < 3) {
      throw std::runtime_error("trav requires COST");
    }
    cfg.cost = std::stod(argv[i++]);
  } else if (cfg.command == "explore-start") {
    if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
      throw std::runtime_error("explore start requires SESSION_ID");
    }
    cfg.session_id = argv[i++];
    cfg.text = "operator_start";
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.text = argv[i++];
    }
  } else if (cfg.command == "explore-pause" || cfg.command == "explore-resume" ||
             cfg.command == "explore-stop") {
    cfg.text = "operator_" + cfg.command.substr(std::string("explore-").size());
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.text = argv[i++];
    }
  } else if (cfg.command == "cancel") {
    cfg.text = "dds_cancel";
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.text = argv[i++];
    }
  } else if (cfg.command == "stop" || cfg.command == "estop" || cfg.command == "clear-estop" ||
             cfg.command == "resume") {
    cfg.text = cfg.command;
    if (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      cfg.text = argv[i++];
    }
  } else if (cfg.command == "path") {
    while (i < argc && std::string(argv[i]).rfind("--", 0) != 0) {
      lingtu_dds_Point point{};
      point.x = std::stod(argv[i++]);
      if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
        throw std::runtime_error("path points must be X Y Z triples");
      }
      point.y = std::stod(argv[i++]);
      if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
        throw std::runtime_error("path points must be X Y Z triples");
      }
      point.z = std::stod(argv[i++]);
      cfg.path.push_back(point);
    }
    if (cfg.path.size() < 2) {
      throw std::runtime_error("path requires at least two X Y Z points");
    }
  } else if (cfg.command == "instruction") {
    if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
      throw std::runtime_error("instruction requires text");
    }
    cfg.text = argv[i++];
  } else if (cfg.command == "clear") {
    if (i >= argc || std::string(argv[i]).rfind("--", 0) == 0) {
      throw std::runtime_error("clear requires map, cloud, or all");
    }
    cfg.text = argv[i++];
    if (cfg.text != "map" && cfg.text != "cloud" && cfg.text != "all") {
      throw std::runtime_error("clear target must be map, cloud, or all");
    }
  } else {
    throw std::runtime_error("unsupported command: " + cfg.command);
  }
  while (i < argc) {
    const std::string arg = argv[i++];
    if (arg == "--domain-id") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --domain-id");
      }
      cfg.domain_id = std::stoi(argv[i++]);
    } else if (arg == "--duration-s") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --duration-s");
      }
      cfg.duration_s = std::stod(argv[i++]);
    } else if (arg == "--rate-hz") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --rate-hz");
      }
      cfg.rate_hz = std::stod(argv[i++]);
    } else if (arg == "--timeout-ms") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --timeout-ms");
      }
      cfg.timeout_ms = std::stoi(argv[i++]);
    } else if (arg == "--input-timeout-ms") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --input-timeout-ms");
      }
      cfg.input_timeout_ms = std::stoi(argv[i++]);
    } else if (arg == "--cleanup-settle-ms") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --cleanup-settle-ms");
      }
      cfg.cleanup_settle_ms = std::stoi(argv[i++]);
    } else if (arg == "--lease-ttl-ms") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --lease-ttl-ms");
      }
      cfg.lease_ttl_ms = static_cast<std::uint32_t>(std::stoul(argv[i++]));
    } else if (arg == "--freshness-budget-ms") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --freshness-budget-ms");
      }
      cfg.freshness_budget_ms = static_cast<std::uint32_t>(std::stoul(argv[i++]));
    } else if (arg == "--source-id") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --source-id");
      }
      cfg.source_id = argv[i++];
    } else if (arg == "--source-epoch") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --source-epoch");
      }
      cfg.source_epoch = std::stoull(argv[i++]);
    } else if (arg == "--ready-file") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --ready-file");
      }
      cfg.ready_file = argv[i++];
    } else if (arg == "--request-id") {
      if (i >= argc) {
        throw std::runtime_error("missing value for --request-id");
      }
      cfg.request_id = argv[i++];
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.duration_s = std::max(0.0, cfg.duration_s);
  cfg.rate_hz = std::max(1.0, cfg.rate_hz);
  cfg.timeout_ms = std::max(1, cfg.timeout_ms);
  cfg.input_timeout_ms = std::max(50, cfg.input_timeout_ms);
  cfg.cleanup_settle_ms = std::max(0, cfg.cleanup_settle_ms);
  if (cfg.command == "operator-motion") {
    if (cfg.source_id.empty() || cfg.source_id.size() > 128U ||
        cfg.source_id.find_first_of(" \t\r\n") != std::string::npos) {
      throw std::runtime_error(
          "operator-motion source ID must be a non-empty token up to 128 bytes");
    }
    if (cfg.lease_ttl_ms == 0U || cfg.lease_ttl_ms > 2000U) {
      throw std::runtime_error("operator-motion lease TTL must be within 1..2000 ms");
    }
    if (cfg.freshness_budget_ms == 0U) {
      throw std::runtime_error("operator-motion freshness budget is required");
    }
  }
  const bool exploration_command = cfg.command == "explore-start" ||
                                   cfg.command == "explore-pause" ||
                                   cfg.command == "explore-resume" || cfg.command == "explore-stop";
  if (!cfg.request_id.empty() && !exploration_command) {
    throw std::runtime_error("--request-id is only supported for explore commands");
  }
  if (cfg.request_id.size() > 128U) {
    throw std::runtime_error("exploration request ID exceeds 128 bytes");
  }
  if (cfg.session_id.size() > 128U) {
    throw std::runtime_error("exploration session ID exceeds 128 bytes");
  }
  if (exploration_command && cfg.text.size() > 256U) {
    throw std::runtime_error("exploration reason exceeds 256 bytes");
  }
  return cfg;
}

void writeReadyFile(const std::string &path) {
  if (path.empty()) {
    return;
  }
  const std::filesystem::path target(path);
  std::error_code ec;
  if (!target.parent_path().empty()) {
    std::filesystem::create_directories(target.parent_path(), ec);
  }
  const std::filesystem::path temporary = target.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("failed to write teleop stream ready file: " + path);
    }
    output << "ready\n";
  }
  std::filesystem::rename(temporary, target, ec);
  if (ec) {
    std::filesystem::remove(target, ec);
    ec.clear();
    std::filesystem::rename(temporary, target, ec);
  }
  if (ec) {
    throw std::runtime_error("failed to publish teleop stream ready file: " + path);
  }
}

struct TeleopStreamLine {
  bool has_update{false};
  bool quit{false};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  std::string reason{"teleop_stream_eof"};
};

TeleopStreamLine parseTeleopStreamLine(const std::string &line) {
  TeleopStreamLine update;
  std::istringstream input(line);
  std::string first;
  if (!(input >> first)) {
    return update;
  }
  if (first == "quit") {
    update.quit = true;
    update.reason = "teleop_stream_quit";
    std::string reason;
    if (input >> reason) {
      update.reason = reason;
    }
    std::string trailing;
    if (input >> trailing) {
      throw std::runtime_error("teleop-stream quit accepts at most one reason token");
    }
    return update;
  }
  try {
    update.vx = std::stod(first);
  } catch (const std::exception &) {
    throw std::runtime_error("teleop-stream line must be `VX VY WZ` or `quit [REASON]`");
  }
  if (!(input >> update.vy >> update.wz)) {
    throw std::runtime_error("teleop-stream line requires VX VY WZ");
  }
  std::string trailing;
  if (input >> trailing) {
    throw std::runtime_error("teleop-stream line has trailing fields");
  }
  if (!std::isfinite(update.vx) || !std::isfinite(update.vy) || !std::isfinite(update.wz)) {
    throw std::runtime_error("teleop-stream values must be finite");
  }
  update.has_update = true;
  return update;
}

int runTeleopStream(lingtu::nav::commands::Client &client, const CliConfig &cfg) {
  using SteadyClock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / cfg.rate_hz));
  const auto input_timeout = std::chrono::milliseconds(cfg.input_timeout_ms);
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  std::uint64_t count = 0;
  std::uint64_t received_updates = 0;
  std::uint64_t superseded_updates = 0;
  std::string stop_reason{"teleop_stream_eof"};
  bool stop_sent = false;
  std::string input_buffer;

  const auto removeReadyFile = [&cfg]() {
    if (!cfg.ready_file.empty()) {
      std::error_code ec;
      std::filesystem::remove(cfg.ready_file, ec);
    }
  };
  const auto bestEffortErrorStop = [&]() {
    bool zero_ok = false;
    bool stop_ok = false;
    try {
      client.navigation().sendTeleop(0.0, 0.0, 0.0, cfg.timeout_ms);
      ++count;
      zero_ok = true;
    } catch (const std::exception &exc) {
      std::fprintf(stderr, "teleop-stream error cleanup zero failed: %s\n", exc.what());
    } catch (...) {
      std::fprintf(stderr, "teleop-stream error cleanup zero failed\n");
    }
    try {
      client.navigation().stop("teleop_stream_error", cfg.timeout_ms);
      stop_ok = true;
    } catch (const std::exception &exc) {
      std::fprintf(stderr, "teleop-stream error cleanup stop failed: %s\n", exc.what());
    } catch (...) {
      std::fprintf(stderr, "teleop-stream error cleanup stop failed\n");
    }
    std::fprintf(stderr, "LT_TELEOP_STREAM_ERROR_CLEANUP_V1 zero=%s stop=%s\n",
                 zero_ok ? "ok" : "failed", stop_ok ? "ok" : "failed");
    std::fflush(stderr);
  };

  try {
    const int input_flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
    if (input_flags < 0 || ::fcntl(STDIN_FILENO, F_SETFL, input_flags | O_NONBLOCK) < 0) {
      throw std::runtime_error("teleop-stream failed to configure nonblocking stdin");
    }

    client.navigation().sendTeleop(vx, vy, wz, cfg.timeout_ms);
    ++count;
    writeReadyFile(cfg.ready_file);
    std::printf("LT_TELEOP_STREAM_READY_V1\n");
    std::fflush(stdout);

    auto last_input = SteadyClock::now();
    auto next_publish = last_input + period;
    while (true) {
      const auto now = SteadyClock::now();
      const bool watchdog_armed = std::abs(vx) > 1e-9 || std::abs(vy) > 1e-9 || std::abs(wz) > 1e-9;
      const auto wake_at =
          watchdog_armed ? std::min(next_publish, last_input + input_timeout) : next_publish;
      const auto remaining =
          wake_at > now
              ? std::chrono::duration_cast<std::chrono::milliseconds>(wake_at - now).count()
              : 0;
      pollfd input_fd{};
      input_fd.fd = STDIN_FILENO;
      input_fd.events = POLLIN | POLLHUP;
      const int poll_result =
          ::poll(&input_fd, 1, static_cast<int>(std::clamp<long long>(remaining, 0, 1000)));
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("teleop-stream stdin poll failed");
      }
      if (poll_result > 0 && (input_fd.revents & (POLLERR | POLLNVAL))) {
        throw std::runtime_error("teleop-stream stdin became invalid");
      }
      bool input_eof = false;
      if (poll_result > 0 && (input_fd.revents & (POLLIN | POLLHUP))) {
        char buffer[4096];
        while (true) {
          const ssize_t read_count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
          if (read_count > 0) {
            input_buffer.append(buffer, static_cast<std::size_t>(read_count));
            continue;
          }
          if (read_count == 0) {
            input_eof = true;
            break;
          }
          if (errno == EINTR) {
            continue;
          }
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
          }
          throw std::runtime_error("teleop-stream stdin read failed");
        }
      }

      TeleopStreamLine latest_update;
      bool has_latest_update = false;
      bool quit = false;
      const auto consumeLine = [&](std::string line) {
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        const TeleopStreamLine update = parseTeleopStreamLine(line);
        if (update.quit) {
          stop_reason = update.reason;
          quit = true;
          return;
        }
        if (update.has_update) {
          ++received_updates;
          if (has_latest_update) {
            ++superseded_updates;
          }
          latest_update = update;
          has_latest_update = true;
        }
      };
      std::size_t newline = std::string::npos;
      while (!quit && (newline = input_buffer.find('\n')) != std::string::npos) {
        consumeLine(input_buffer.substr(0, newline));
        input_buffer.erase(0, newline + 1);
      }
      if (!quit && input_eof && !input_buffer.empty()) {
        consumeLine(input_buffer);
        input_buffer.clear();
      }
      if (quit) {
        break;
      }
      if (has_latest_update) {
        vx = latest_update.vx;
        vy = latest_update.vy;
        wz = latest_update.wz;
        last_input = SteadyClock::now();
        client.navigation().sendTeleop(vx, vy, wz, cfg.timeout_ms);
        ++count;
        next_publish = SteadyClock::now() + period;
      }
      if (input_eof) {
        break;
      }

      const auto after_input = SteadyClock::now();
      const bool input_timed_out =
          (std::abs(vx) > 1e-9 || std::abs(vy) > 1e-9 || std::abs(wz) > 1e-9) &&
          after_input >= last_input + input_timeout;
      if (input_timed_out) {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
        stop_reason = "teleop_stream_input_timeout";
        client.navigation().sendTeleop(vx, vy, wz, cfg.timeout_ms);
        ++count;
        client.navigation().stop(stop_reason, cfg.timeout_ms);
        stop_sent = true;
        std::printf(
            "LT_TELEOP_STREAM_TIMEOUT_STOP_V1\n"
            "teleop-stream input timeout; forced zero+stop; restart required\n");
        std::fflush(stdout);
        break;
      }
      if (SteadyClock::now() >= next_publish) {
        client.navigation().sendTeleop(vx, vy, wz, cfg.timeout_ms);
        ++count;
        next_publish = SteadyClock::now() + period;
      }
    }

    if (!stop_sent) {
      client.navigation().sendTeleop(0.0, 0.0, 0.0, cfg.timeout_ms);
      ++count;
      client.navigation().stop(stop_reason, cfg.timeout_ms);
    }
    removeReadyFile();
    std::printf("accepted teleop-stream stop=%s samples=%llu updates=%llu superseded=%llu\n",
                stop_reason.c_str(), static_cast<unsigned long long>(count),
                static_cast<unsigned long long>(received_updates),
                static_cast<unsigned long long>(superseded_updates));
    return 0;
  } catch (...) {
    const std::exception_ptr failure = std::current_exception();
    bestEffortErrorStop();
    removeReadyFile();
    std::rethrow_exception(failure);
  }
}
volatile std::sig_atomic_t operator_motion_stop_requested = 0;

void requestOperatorMotionStop(int) {
  operator_motion_stop_requested = 1;
}

std::uint64_t generatedSourceEpoch() {
  const auto epoch =
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
  return epoch == 0U ? 1U : epoch;
}

void printOperatorMotionEvent(const char *action, const CliConfig &cfg, std::uint64_t source_epoch,
                              std::uint64_t source_sequence, std::uint64_t sample_count) {
  std::printf(
      "LT_OPERATOR_MOTION_EVENT_V1 action=%s accepted=true source_id=%s "
      "source_epoch=%llu source_sequence=%llu sample_count=%llu\n",
      action, cfg.source_id.c_str(), static_cast<unsigned long long>(source_epoch),
      static_cast<unsigned long long>(source_sequence),
      static_cast<unsigned long long>(sample_count));
  std::fflush(stdout);
}

int runOperatorMotion(lingtu::nav::commands::Client &client, const CliConfig &cfg) {
  using SteadyClock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / cfg.rate_hz));
  const auto settle = std::chrono::milliseconds(cfg.cleanup_settle_ms);
  const std::uint64_t source_epoch =
      cfg.source_epoch == 0U ? generatedSourceEpoch() : cfg.source_epoch;
  std::uint64_t sequence = 1U;
  std::uint64_t sample_count = 0U;
  bool claimed = false;
  operator_motion_stop_requested = 0;
  const auto prior_sigint = std::signal(SIGINT, requestOperatorMotionStop);
  const auto prior_sigterm = std::signal(SIGTERM, requestOperatorMotionStop);

  const auto restoreSignals = [&]() {
    std::signal(SIGINT, prior_sigint);
    std::signal(SIGTERM, prior_sigterm);
  };
  const auto finishAuthority = [&](const char *reason, bool best_effort) {
    std::exception_ptr cleanup_failure;
    if (!claimed) {
      return;
    }
    try {
      ++sequence;
      client.operatorMotion().hold(cfg.source_id, source_epoch, sequence, reason, cfg.timeout_ms);
      printOperatorMotionEvent("hold", cfg, source_epoch, sequence, sample_count);
    } catch (...) {
      cleanup_failure = std::current_exception();
      std::fprintf(stderr, "LT_OPERATOR_MOTION_CLEANUP_V1 action=hold accepted=false\n");
      std::fflush(stderr);
    }
    if (settle.count() > 0) {
      std::this_thread::sleep_for(settle);
    }
    try {
      ++sequence;
      client.operatorMotion().release(cfg.source_id, source_epoch, sequence, reason,
                                      cfg.timeout_ms);
      printOperatorMotionEvent("release", cfg, source_epoch, sequence, sample_count);
      claimed = false;
    } catch (...) {
      if (cleanup_failure == nullptr) {
        cleanup_failure = std::current_exception();
      }
      std::fprintf(stderr, "LT_OPERATOR_MOTION_CLEANUP_V1 action=release accepted=false\n");
      std::fflush(stderr);
    }
    if (settle.count() > 0) {
      std::this_thread::sleep_for(settle);
    }
    if (!best_effort && cleanup_failure != nullptr) {
      std::rethrow_exception(cleanup_failure);
    }
  };

  try {
    client.operatorMotion().claim(cfg.source_id, source_epoch, sequence, cfg.lease_ttl_ms,
                                  cfg.timeout_ms);
    claimed = true;
    printOperatorMotionEvent("claim", cfg, source_epoch, sequence, sample_count);

    const auto start = SteadyClock::now();
    const auto deadline = cfg.duration_s > 0.0
                              ? start + std::chrono::duration_cast<SteadyClock::duration>(
                                            std::chrono::duration<double>(cfg.duration_s))
                              : start;
    auto next_publish = start;
    while (operator_motion_stop_requested == 0) {
      ++sequence;
      client.operatorMotion().sample(cfg.source_id, source_epoch, sequence, cfg.x, cfg.y, cfg.yaw,
                                     true, cfg.freshness_budget_ms, cfg.timeout_ms);
      ++sample_count;
      printOperatorMotionEvent("sample", cfg, source_epoch, sequence, sample_count);
      if (cfg.duration_s <= 0.0) {
        break;
      }
      next_publish += period;
      std::this_thread::sleep_until(next_publish);
      if (SteadyClock::now() >= deadline) {
        break;
      }
    }

    finishAuthority(operator_motion_stop_requested != 0 ? "operator_motion_signal"
                                                        : "operator_motion_complete",
                    false);
    restoreSignals();
    std::printf("accepted operator-motion vx=%.3f vy=%.3f wz=%.3f samples=%llu\n", cfg.x, cfg.y,
                cfg.yaw, static_cast<unsigned long long>(sample_count));
    return 0;
  } catch (...) {
    const std::exception_ptr failure = std::current_exception();
    finishAuthority("operator_motion_error", true);
    restoreSignals();
    std::rethrow_exception(failure);
  }
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const CliConfig cfg = parseArgs(argc, argv);
    if (cfg.command == "goal" || cfg.command == "cancel" || cfg.command == "teleop" ||
        cfg.command == "operator-motion" || cfg.command == "teleop-stream" ||
        cfg.command == "stop" || cfg.command == "estop" || cfg.command == "clear-estop" ||
        cfg.command == "resume" || cfg.command == "explore-start" ||
        cfg.command == "explore-pause" || cfg.command == "explore-resume" ||
        cfg.command == "explore-stop") {
      lingtu::nav::commands::Client client(cfg.domain_id);
      if (cfg.command == "goal") {
        client.navigation().sendGoal(cfg.x, cfg.y, cfg.z, cfg.yaw, cfg.timeout_ms);
        std::printf("accepted goal %.3f %.3f %.3f yaw=%.3f\n", cfg.x, cfg.y, cfg.z, cfg.yaw);
      } else if (cfg.command == "cancel") {
        client.navigation().cancel(cfg.text, cfg.timeout_ms);
        std::printf("accepted cancel: %s\n", cfg.text.c_str());
      } else if (cfg.command == "teleop") {
        const double deadline = nowSeconds() + cfg.duration_s;
        std::uint64_t count = 0;
        do {
          client.navigation().sendTeleop(cfg.x, cfg.y, cfg.yaw, cfg.timeout_ms);
          ++count;
          if (cfg.duration_s <= 0.0) {
            break;
          }
          std::this_thread::sleep_for(
              std::chrono::milliseconds(static_cast<int>(1000.0 / cfg.rate_hz)));
        } while (nowSeconds() < deadline);
        std::printf("accepted teleop vx=%.3f vy=%.3f wz=%.3f samples=%llu\n", cfg.x, cfg.y, cfg.yaw,
                    static_cast<unsigned long long>(count));
      } else if (cfg.command == "operator-motion") {
        return runOperatorMotion(client, cfg);
      } else if (cfg.command == "teleop-stream") {
        return runTeleopStream(client, cfg);
      } else if (cfg.command == "stop") {
        client.navigation().stop(cfg.text, cfg.timeout_ms);
        std::printf("accepted stop: %s\n", cfg.text.c_str());
      } else if (cfg.command == "estop") {
        client.navigation().estop(cfg.text, cfg.timeout_ms);
        std::printf("accepted estop: %s\n", cfg.text.c_str());
      } else if (cfg.command == "clear-estop") {
        client.navigation().clearEstop(cfg.text, cfg.timeout_ms);
        std::printf("accepted clear-estop: %s\n", cfg.text.c_str());
      } else if (cfg.command == "resume") {
        client.navigation().resumeAutonomy(cfg.text, cfg.timeout_ms);
        std::printf("accepted resume: %s\n", cfg.text.c_str());
      } else if (cfg.command == "explore-start") {
        client.exploration().start(cfg.session_id, cfg.text, cfg.timeout_ms, cfg.request_id);
        std::printf("accepted explore start session=%s request_id=%s\n", cfg.session_id.c_str(),
                    cfg.request_id.empty() ? "<generated>" : cfg.request_id.c_str());
      } else if (cfg.command == "explore-pause") {
        client.exploration().pause(cfg.text, cfg.timeout_ms, cfg.request_id);
        std::printf("accepted explore pause: %s\n", cfg.text.c_str());
      } else if (cfg.command == "explore-resume") {
        client.exploration().resume(cfg.text, cfg.timeout_ms, cfg.request_id);
        std::printf("accepted explore resume: %s\n", cfg.text.c_str());
      } else {
        client.exploration().stop(cfg.text, cfg.timeout_ms, cfg.request_id);
        std::printf("accepted explore stop: %s\n", cfg.text.c_str());
      }
      return 0;
    }
    const dds_entity_t participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
        "dds_create_participant");
    const dds_entity_t publisher =
        checked(dds_create_publisher(participant, nullptr, nullptr), "dds_create_publisher");

    if (cfg.command == "cloud") {
      const dds_entity_t topic =
          checked(dds_create_topic(participant, &lingtu_dds_PointCloud2_desc,
                                   lingtu::message::kSlamRegisteredCloud.dds_topic.data(), nullptr,
                                   nullptr),
                  "dds_create_topic(registered_cloud)");
      auto qos = qosFor(lingtu::message::kSlamRegisteredCloud);
      const dds_entity_t writer = checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                                          "dds_create_writer(registered_cloud)");

      lingtu_dds_PointField fields[4]{};
      fields[0].name = const_cast<char *>("x");
      fields[0].offset = 0;
      fields[0].datatype = 7;
      fields[0].count = 1;
      fields[1].name = const_cast<char *>("y");
      fields[1].offset = 4;
      fields[1].datatype = 7;
      fields[1].count = 1;
      fields[2].name = const_cast<char *>("z");
      fields[2].offset = 8;
      fields[2].datatype = 7;
      fields[2].count = 1;
      fields[3].name = const_cast<char *>("height");
      fields[3].offset = 12;
      fields[3].datatype = 7;
      fields[3].count = 1;
      std::vector<std::uint8_t> data(16);
      const float values[4] = {
          static_cast<float>(cfg.x),
          static_cast<float>(cfg.y),
          static_cast<float>(cfg.z),
          static_cast<float>(cfg.cost),
      };
      std::memcpy(data.data(), values, sizeof(values));

      lingtu_dds_PointCloud2 msg{};
      fillHeader(msg.header, nowSeconds(), "body");
      msg.height = 1;
      msg.width = 1;
      msg.fields._maximum = 4;
      msg.fields._length = 4;
      msg.fields._buffer = fields;
      msg.fields._release = false;
      msg.point_step = 16;
      msg.row_step = 16;
      msg.data._maximum = static_cast<std::uint32_t>(data.size());
      msg.data._length = static_cast<std::uint32_t>(data.size());
      msg.data._buffer = data.data();
      msg.data._release = false;
      msg.is_dense = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "registered_cloud");
      checked(dds_write(writer, &msg), "dds_write(registered_cloud)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(registered_cloud)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published cloud point x=%.3f y=%.3f z=%.3f height=%.3f\n", cfg.x, cfg.y, cfg.z,
                  cfg.cost);
    } else if (cfg.command == "trav") {
      const dds_entity_t topic = checked(
          dds_create_topic(participant, &lingtu_dds_OccupancyGrid_desc,
                           lingtu::message::kNavTraversability.dds_topic.data(), nullptr, nullptr),
          "dds_create_topic(traversability)");
      auto qos = qosFor(lingtu::message::kNavTraversability);
      const dds_entity_t writer = checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                                          "dds_create_writer(traversability)");

      std::vector<std::uint8_t> data(static_cast<std::size_t>(cfg.width) *
                                         static_cast<std::size_t>(cfg.height),
                                     static_cast<std::uint8_t>(std::clamp(cfg.cost, 0.0, 100.0)));
      lingtu_dds_OccupancyGrid msg{};
      fillHeader(msg.header, nowSeconds(), "map");
      msg.info.resolution = static_cast<float>(cfg.resolution);
      msg.info.width = cfg.width;
      msg.info.height = cfg.height;
      msg.info.origin.position.x = cfg.origin_x;
      msg.info.origin.position.y = cfg.origin_y;
      msg.info.origin.orientation.w = 1.0;
      msg.data._maximum = static_cast<std::uint32_t>(data.size());
      msg.data._length = static_cast<std::uint32_t>(data.size());
      msg.data._buffer = data.data();
      msg.data._release = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "traversability");
      checked(dds_write(writer, &msg), "dds_write(traversability)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(traversability)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published traversability cost=%.1f cells=%ux%u\n", cfg.cost, cfg.width,
                  cfg.height);
    } else if (cfg.command == "path") {
      std::fprintf(stderr,
                   "nav_control: legacy path injection requires endpoint "
                   "--allow-legacy-motion-inputs true; product control uses typed Goal\n");
      const dds_entity_t topic = checked(
          dds_create_topic(participant, &lingtu_dds_Path_desc,
                           lingtu::message::kNavGlobalPath.dds_topic.data(), nullptr, nullptr),
          "dds_create_topic(global_path)");
      auto qos = qosFor(lingtu::message::kNavGlobalPath);
      const dds_entity_t writer = checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                                          "dds_create_writer(global_path)");
      const double stamp_s = nowSeconds();
      std::vector<lingtu_dds_PoseStamped> poses(cfg.path.size());
      for (std::size_t i = 0; i < cfg.path.size(); ++i) {
        fillHeader(poses[i].header, stamp_s, "map");
        poses[i].pose.position = cfg.path[i];
        poses[i].pose.orientation = quaternionFromYaw(0.0);
      }
      lingtu_dds_Path msg{};
      fillHeader(msg.header, stamp_s, "map");
      msg.poses._maximum = static_cast<std::uint32_t>(poses.size());
      msg.poses._length = static_cast<std::uint32_t>(poses.size());
      msg.poses._buffer = poses.data();
      msg.poses._release = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, "global_path");
      checked(dds_write(writer, &msg), "dds_write(global_path)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(global_path)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published path %zu waypoints\n", cfg.path.size());
    } else if (cfg.command == "clear") {
      auto publish_clear = [&](const lingtu::message::TopicContract &contract, const char *label) {
        const dds_entity_t topic =
            checked(dds_create_topic(participant, &lingtu_dds_Bool_desc, contract.dds_topic.data(),
                                     nullptr, nullptr),
                    (std::string("dds_create_topic(") + label + ")").c_str());
        auto qos = qosFor(contract);
        const dds_entity_t writer =
            checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                    (std::string("dds_create_writer(") + label + ")").c_str());
        lingtu_dds_Bool msg{};
        msg.data = true;
        waitForMatchedReader(writer, label);
        checked(dds_write(writer, &msg), (std::string("dds_write(") + label + ")").c_str());
        checked(dds_wait_for_acks(writer, DDS_SECS(2)),
                (std::string("dds_wait_for_acks(") + label + ")").c_str());
      };
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (cfg.text == "map" || cfg.text == "all") {
        publish_clear(lingtu::message::kNavMapClearing, "map_clearing");
      }
      if (cfg.text == "cloud" || cfg.text == "all") {
        publish_clear(lingtu::message::kNavCloudClearing, "cloud_clearing");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published clear %s\n", cfg.text.c_str());
    } else {
      const auto &contract = lingtu::message::kNavSemanticInstruction;
      const dds_entity_t topic =
          checked(dds_create_topic(participant, &lingtu_dds_Text_desc, contract.dds_topic.data(),
                                   nullptr, nullptr),
                  "dds_create_topic(text)");
      auto qos = qosFor(contract);
      const dds_entity_t writer = checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                                          "dds_create_writer(text)");
      lingtu_dds_Text msg{};
      msg.data = const_cast<char *>(cfg.text.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      waitForMatchedReader(writer, cfg.command.c_str());
      checked(dds_write(writer, &msg), "dds_write(text)");
      checked(dds_wait_for_acks(writer, DDS_SECS(2)), "dds_wait_for_acks(text)");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::printf("published %s: %s\n", cfg.command.c_str(), cfg.text.c_str());
    }
    dds_delete(participant);
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "lingtu_nav_control failed: %s\n", exc.what());
    return 1;
  }
}
