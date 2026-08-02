#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "dds/dds.h"
#include "lingtu_slam.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: test_imu_publisher DOMAIN\n";
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
  const dds_entity_t writer = dds_create_writer(participant, topic, nullptr, nullptr);
  if (topic < 0 || writer < 0) {
    dds_delete(participant);
    return 2;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  for (int index = 0; index < 20; ++index) {
    auto *sample = lingtu_dds_Imu__alloc();
    sample->header.frame_id = dds_string_dup("imu_link");
    sample->header.stamp.sec = 100;
    sample->header.stamp.nanosec = static_cast<std::uint32_t>(index);
    sample->angular_velocity.x = static_cast<double>(index);
    const auto timestamp = static_cast<dds_time_t>(1000000000LL + index * 1000000LL);
    const dds_return_t result = dds_write_ts(writer, sample, timestamp);
    lingtu_dds_Imu_free(sample, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
    if (result != DDS_RETCODE_OK) {
      dds_delete(participant);
      return 3;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  dds_delete(participant);
  return 0;
}
