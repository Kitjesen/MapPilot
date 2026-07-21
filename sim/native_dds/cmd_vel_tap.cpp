#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unistd.h>

#include <dds/dds.h>

#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

double sourceWallSeconds(const lingtu_dds_FinalVelocityCommand& msg) {
  return static_cast<double>(msg.source_wall_ns) * 1e-9;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

void writeControlState(dds_entity_t writer, std::uint64_t accepted_sequence) {
  lingtu_dds_DriverControlState msg{};
  fillHeader(msg.header, nowSeconds(), "body");
  msg.connected = true;
  msg.ready = true;
  msg.motors_enabled = true;
  msg.critical_fault = false;
  msg.lease_valid = true;
  msg.lease_remaining_ms = 1000;
  msg.accepted_sequence = accepted_sequence;
  msg.last_command_accepted = true;
  msg.fsm = const_cast<char*>(accepted_sequence == 0 ? "standing" : "walking");
  msg.owner = const_cast<char*>("grpc");
  msg.owner_id = const_cast<char*>("lingtu-driver");
  msg.reason = const_cast<char*>("");
  checked(dds_write(writer, &msg), "dds_write(driver_control_state)");
}

struct Config {
  int domain_id{0};
  std::string ready_file;
};

Config parseArgs(int argc, char** argv) {
  Config cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--ready-file") {
      cfg.ready_file = next();
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: lingtu_mujoco_cmd_vel_tap [--domain-id N] [--ready-file PATH]\n");
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return cfg;
}

void writeReadyFile(const std::string& path) {
  if (path.empty()) {
    return;
  }
  const std::filesystem::path target(path);
  std::error_code ec;
  if (!target.parent_path().empty()) {
    std::filesystem::create_directories(target.parent_path(), ec);
  }
  std::ofstream out(target, std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to write ready file: " + path);
  }
  out << "ready\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    const Config cfg = parseArgs(argc, argv);

    const dds_entity_t participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
        "dds_create_participant");
    const dds_entity_t subscriber = checked(
        dds_create_subscriber(participant, nullptr, nullptr),
        "dds_create_subscriber");
    const dds_entity_t publisher = checked(
        dds_create_publisher(participant, nullptr, nullptr),
        "dds_create_publisher");
    const auto& contract = lingtu::message::kNavCmdVel;
    const dds_entity_t topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_FinalVelocityCommand_desc,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(cmd_vel)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    const dds_entity_t reader = checked(
        dds_create_reader(subscriber, topic, qos.get(), nullptr),
        "dds_create_reader(cmd_vel)");
    const auto& control_contract = lingtu::message::kDriverControlState;
    const dds_entity_t control_state_topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_DriverControlState_desc,
            control_contract.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(driver_control_state)");
    auto control_state_qos =
        lingtu::dds::make_qos(lingtu::dds::qos_for_topic(control_contract.dds_topic));
    const dds_entity_t control_state_writer = checked(
        dds_create_writer(
            publisher, control_state_topic, control_state_qos.get(), nullptr),
        "dds_create_writer(driver_control_state)");

    writeReadyFile(cfg.ready_file);
    std::printf("LT_PID_V1\t%ld\n", static_cast<long>(getpid()));
    std::fflush(stdout);
    std::fprintf(
        stderr,
        "lingtu_mujoco_cmd_vel_tap: domain=%d cmd_topic=%s control_state_topic=%s\n",
        cfg.domain_id,
        contract.dds_topic.data(),
        control_contract.dds_topic.data());

    std::uint64_t sequence = 0;
    auto next_control_state = std::chrono::steady_clock::now();
    while (g_running) {
      void* samples[32]{};
      dds_sample_info_t infos[32]{};
      const dds_return_t count = dds_take(reader, samples, infos, 32, 32);
      if (count < 0) {
        throw std::runtime_error(
            std::string("dds_take(cmd_vel): ") + dds_strretcode(-count));
      }
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data || samples[i] == nullptr) {
          continue;
        }
        const auto* msg =
            static_cast<const lingtu_dds_FinalVelocityCommand*>(samples[i]);
        std::printf(
            "LT_CMD_V1\t%llu\t%.9f\t%.9f\t%.9f\t%.9f\n",
            static_cast<unsigned long long>(++sequence),
            sourceWallSeconds(*msg),
            msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.angular.z);
      }
      if (count > 0) {
        std::fflush(stdout);
        checked(dds_return_loan(reader, samples, count), "dds_return_loan(cmd_vel)");
      }
      const auto heartbeat_now = std::chrono::steady_clock::now();
      if (heartbeat_now >= next_control_state) {
        writeControlState(control_state_writer, sequence);
        next_control_state = heartbeat_now + std::chrono::milliseconds(50);
      }
      if (count == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }

    dds_delete(participant);
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_mujoco_cmd_vel_tap failed: %s\n", exc.what());
    return 1;
  }
}
