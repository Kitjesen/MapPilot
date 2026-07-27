#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "nav_dds_runtime.hpp"

namespace {

using namespace std::chrono_literals;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

std::string hostBootId() {
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string value;
  std::getline(input, value);
  return value;
}

struct Sample {
  std::string host_boot_id;
  std::string producer_boot_id;
  std::uint64_t output_seq{0};
  std::uint64_t source_boottime_ns{0};
  std::uint64_t source_wall_ns{0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

class FinalVelocityReader {
 public:
  explicit FinalVelocityReader(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(test)");
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, &lingtu_dds_FinalVelocityCommand_desc,
                                 lingtu::message::kNavCmdVel.dds_topic.data(), nullptr, nullptr),
                "dds_create_topic(test_cmd_vel)");
    auto qos =
        lingtu::dds::make_qos(lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
    reader_ = checked(dds_create_reader(participant_, topic, qos.get(), nullptr),
                      "dds_create_reader(test_cmd_vel)");
  }

  ~FinalVelocityReader() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  Sample take() {
    void *samples[1]{};
    dds_sample_info_t infos[1]{};
    const dds_return_t count = dds_take(reader_, samples, infos, 1, 1);
    if (count < 0) {
      throw std::runtime_error(std::string("dds_take(test_cmd_vel): ") + dds_strretcode(-count));
    }
    if (count == 0 || !infos[0].valid_data) {
      if (count > 0) {
        checked(dds_return_loan(reader_, samples, count), "dds_return_loan(test_cmd_vel)");
      }
      return {};
    }
    const auto &msg = *static_cast<lingtu_dds_FinalVelocityCommand *>(samples[0]);
    Sample result;
    result.host_boot_id = msg.host_boot_id ? msg.host_boot_id : "";
    result.producer_boot_id = msg.producer_boot_id ? msg.producer_boot_id : "";
    result.output_seq = msg.output_seq;
    result.source_boottime_ns = msg.source_boottime_ns;
    result.source_wall_ns = msg.source_wall_ns;
    result.vx = msg.twist.linear.x;
    result.vy = msg.twist.linear.y;
    result.wz = msg.twist.angular.z;
    checked(dds_return_loan(reader_, samples, count), "dds_return_loan(test_cmd_vel)");
    return result;
  }

 private:
  dds_entity_t participant_{0};
  dds_entity_t reader_{0};
};

Sample writeAndTake(lingtu::nav::endpoint::DdsRuntime &runtime, FinalVelocityReader &reader,
                    const nav_kernel::Twist &command) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto receipt = runtime.writeCmdVelSequenced(command);
    require(receipt.has_value(), "final velocity write must succeed");
    std::this_thread::sleep_for(10ms);
    Sample sample = reader.take();
    if (sample.output_seq != 0) {
      require(sample.output_seq == receipt->output_sequence,
              "returned sequence must match the published final velocity command");
      require(sample.source_wall_ns == receipt->source_wall_ns,
              "returned source wall stamp must match the published final velocity command");
      return sample;
    }
  }
  throw std::runtime_error("final velocity sample was not received");
}
void testWriterAddsFreshnessEnvelope() {
  constexpr int kDomain = 96;
  FinalVelocityReader reader(kDomain);
  lingtu::nav::endpoint::DdsRuntime runtime(kDomain);

  const Sample first = writeAndTake(runtime, reader, nav_kernel::Twist{0.2, -0.1, 0.4});
  require(first.host_boot_id == hostBootId(), "host boot id must match Linux");
  require(!first.producer_boot_id.empty(), "producer boot id must be present");
  require(first.output_seq >= 1, "output sequence must be positive");
  require(first.source_boottime_ns > 0, "boottime timestamp must be present");
  require(first.source_wall_ns > 0, "wall timestamp must be present");
  require(std::abs(first.vx - 0.2) < 1e-9, "vx must be preserved");
  require(std::abs(first.vy + 0.1) < 1e-9, "vy must be preserved");
  require(std::abs(first.wz - 0.4) < 1e-9, "wz must be preserved");

  const Sample second = writeAndTake(runtime, reader, nav_kernel::Twist{});
  require(second.producer_boot_id == first.producer_boot_id,
          "producer id must remain stable for the process");
  require(second.output_seq == first.output_seq + 1, "output sequence must increase monotonically");
  require(second.source_boottime_ns >= first.source_boottime_ns, "boottime must not roll back");
}

}  // namespace

int main() {
  try {
    testWriterAddsFreshnessEnvelope();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_final_velocity_writer: FAIL: %s\n", exc.what());
    return 1;
  }
}
