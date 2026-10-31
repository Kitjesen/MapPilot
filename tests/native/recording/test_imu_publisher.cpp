#include "lingtu/recording/topic_catalog.hpp"

#include "dds/dds.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "messages.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: test_imu_publisher DOMAIN [--with-ack] [--with-joints] [--with-odom] "
                 "[--seconds N]\n";
    return 2;
  }
  bool with_ack = false, with_joints = false, with_odom = false;
  double seconds = 0;
  for (int i = 2; i < argc; ++i) {
    const std::string option(argv[i]);
    if (option == "--with-ack")
      with_ack = true;
    else if (option == "--with-joints")
      with_joints = true;
    else if (option == "--with-odom")
      with_odom = true;
    else if (option == "--seconds" && i + 1 < argc)
      seconds = std::stod(argv[++i]);
    else
      return 2;
  }
  if (!std::isfinite(seconds) || seconds < 0 || seconds > 3600)
    return 2;
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

  dds_entity_t ack_writer = DDS_RETCODE_ERROR;
  if (with_ack) {
    const auto *binding = lingtu::recording::find_recording_topic("/nav/command/ack");
    const auto ack_topic = dds_create_topic(participant, binding->descriptor,
                                            binding->contract->dds_topic.data(), nullptr, nullptr);
    auto qos = lingtu::dds::make_qos(binding->qos_profile);
    ack_writer = dds_create_writer(participant, ack_topic, qos.get(), nullptr);
    if (ack_writer < 0) {
      dds_delete(participant);
      return 2;
    }
  }
  dds_entity_t joints_writer = DDS_RETCODE_ERROR;
  if (with_joints) {
    const auto *binding = lingtu::recording::find_recording_topic("/robot/joint_states");
    const auto joints_topic = dds_create_topic(
        participant, binding->descriptor, binding->contract->dds_topic.data(), nullptr, nullptr);
    auto qos = lingtu::dds::make_qos(binding->qos_profile);
    joints_writer = dds_create_writer(participant, joints_topic, qos.get(), nullptr);
    if (joints_writer < 0) {
      dds_delete(participant);
      return 2;
    }
  }
  dds_entity_t odom_writer = DDS_RETCODE_ERROR;
  if (with_odom) {
    const auto *binding = lingtu::recording::find_recording_topic("/slam/odometry");
    const auto odom_topic = dds_create_topic(participant, binding->descriptor,
                                             binding->contract->dds_topic.data(), nullptr, nullptr);
    auto qos = lingtu::dds::make_qos(binding->qos_profile);
    odom_writer = dds_create_writer(participant, odom_topic, qos.get(), nullptr);
    if (odom_writer < 0) {
      dds_delete(participant);
      return 2;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  const auto count = seconds > 0 ? static_cast<int>(std::ceil(seconds * 100)) : 20;
  for (int index = 0; index < count; ++index) {
    auto *sample = lingtu_dds_Imu__alloc();
    sample->header.frame_id = dds_string_dup("imu_link");
    sample->header.stamp.sec = 100;
    sample->header.stamp.nanosec = static_cast<std::uint32_t>(index);
    sample->angular_velocity.x = seconds > 0 ? std::sin(index * 0.02) : static_cast<double>(index);
    const auto timestamp = static_cast<dds_time_t>(1000000000LL + index * 1000000LL);
    const dds_return_t result = dds_write_ts(writer, sample, timestamp);
    lingtu_dds_Imu_free(sample, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
    if (result != DDS_RETCODE_OK) {
      dds_delete(participant);
      return 3;
    }
    if (ack_writer > 0) {
      lingtu_dds_NavigationCommandAck ack{};
      ack.header.frame_id = const_cast<char *>("body");
      ack.task_id = const_cast<char *>("");
      ack.request_id = const_cast<char *>("recording-fixture");
      ack.reason = const_cast<char *>("fixture");
      ack.accepted = true;
      if (dds_write_ts(ack_writer, &ack, timestamp) != DDS_RETCODE_OK) {
        dds_delete(participant);
        return 3;
      }
    }
    if (joints_writer > 0) {
      lingtu_dds_JointState joints{};
      joints.header.frame_id = const_cast<char *>("body");
      joints.robot_model = const_cast<char *>("synthetic-joints");
      char *names[]{const_cast<char *>("hip"), const_cast<char *>("knee")};
      double q[]{0.25 + 0.05 * std::sin(index * 0.1), -1.2};
      double dq[]{0.5 * std::cos(index * 0.1), 0.0};
      double tau[]{3.5, -2.25};
      joints.names = {2, 2, names, false};
      joints.position = {2, 2, q, false};
      joints.velocity = {2, 2, dq, false};
      joints.effort = {2, 2, tau, false};
      if (dds_write_ts(joints_writer, &joints, timestamp) != DDS_RETCODE_OK) {
        dds_delete(participant);
        return 3;
      }
    }
    if (odom_writer > 0) {
      lingtu_dds_Odometry odom{};
      odom.header.frame_id = const_cast<char *>("map");
      odom.child_frame_id = const_cast<char *>("body");
      odom.pose.pose.position.x = index * 0.01;
      odom.pose.pose.orientation.w = 1;
      if (dds_write_ts(odom_writer, &odom, timestamp) != DDS_RETCODE_OK) {
        dds_delete(participant);
        return 3;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  dds_delete(participant);
  return 0;
}
