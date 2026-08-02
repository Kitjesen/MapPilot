#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "dds/dds.h"
#include "lingtu_slam.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: test_imu_subscriber DOMAIN\n";
    return 2;
  }
  const int domain = std::stoi(argv[1]);
  const dds_entity_t participant =
      dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr);
  if (participant < 0) {
    return 2;
  }
  const dds_entity_t topic =
      dds_create_topic(participant, &lingtu_dds_Imu_desc, "rt/imu/raw", nullptr, nullptr);
  auto *qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
  dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 256);
  const dds_entity_t reader = dds_create_reader(participant, topic, qos, nullptr);
  dds_delete_qos(qos);
  if (topic < 0 || reader < 0) {
    dds_delete(participant);
    return 2;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < deadline) {
    void *samples[16]{};
    dds_sample_info_t infos[16]{};
    const dds_return_t count = dds_take(reader, samples, infos, 16, 16);
    if (count < 0) {
      dds_delete(participant);
      return 3;
    }
    bool received_valid_sample = false;
    for (dds_return_t index = 0; index < count; ++index) {
      if (!infos[index].valid_data) {
        continue;
      }
      const auto *sample = static_cast<const lingtu_dds_Imu *>(samples[index]);
      if (sample != nullptr && sample->header.frame_id != nullptr &&
          std::strcmp(sample->header.frame_id, "imu_link") == 0 &&
          infos[index].source_timestamp >= 1000000000LL) {
        received_valid_sample = true;
      }
    }
    if (count > 0 && dds_return_loan(reader, samples, count) != DDS_RETCODE_OK) {
      dds_delete(participant);
      return 4;
    }
    if (received_valid_sample) {
      dds_delete(participant);
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cerr << "timed out waiting for replayed IMU sample\n";
  dds_delete(participant);
  return 5;
}
