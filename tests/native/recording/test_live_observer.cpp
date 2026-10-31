#include "lingtu/recording/live_observer.hpp"
#include "lingtu/recording/topic_catalog.hpp"

#include "dds/dds.h"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "messages.h"

namespace {
void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
dds_entity_t writer(dds_entity_t participant, const char *name) {
  const auto *binding = lingtu::recording::find_recording_topic(name);
  const auto topic = dds_create_topic(participant, binding->descriptor,
                                      binding->contract->dds_topic.data(), nullptr, nullptr);
  auto qos = lingtu::dds::make_qos(binding->qos_profile);
  const auto result = dds_create_writer(participant, topic, qos.get(), nullptr);
  require(result > 0, "could not create fixture publisher");
  for (int i = 0; i < 200; ++i) {
    dds_publication_matched_status_t state{};
    require(dds_get_publication_matched_status(result, &state) == DDS_RETCODE_OK,
            "could not inspect fixture match");
    if (state.current_count)
      return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("live observer did not match fixture publisher");
}
}  // namespace

int main() {
  lingtu::recording::LiveObserver observer(
      73, {"/imu/raw", "/robot/joint_states", "/nav/command/ack", "/slam/registered_cloud"});
  const auto idle = observer.snapshot();
  require(idle.find("\"last_sample_age_ms\":null") != std::string::npos,
          "unseen topic incorrectly fresh");
  require(idle.find("\"preview\":null") != std::string::npos, "unseen topic has fabricated value");
  const auto participant = dds_create_participant(73, nullptr, nullptr);
  require(participant > 0, "could not create fixture participant");
  const auto imu_writer = writer(participant, "/imu/raw");
  const auto joints_writer = writer(participant, "/robot/joint_states");
  const auto ack_writer = writer(participant, "/nav/command/ack");
  const auto cloud_writer = writer(participant, "/slam/registered_cloud");
  lingtu_dds_Imu imu{};
  imu.header.frame_id = const_cast<char *>("imu_link");
  imu.angular_velocity.x = 1.25;
  imu.linear_acceleration.z = 9.81;
  lingtu_dds_JointState joints{};
  joints.header.frame_id = const_cast<char *>("body");
  joints.robot_model = const_cast<char *>("fixture");
  char *names[]{const_cast<char *>("hip"), const_cast<char *>("knee")};
  double q[]{0.25, -1.2}, dq[]{1.5, -0.75}, tau[]{3.5, -2.25};
  joints.names = {2, 2, names, false};
  joints.position = {2, 2, q, false};
  joints.velocity = {2, 2, dq, false};
  joints.effort = {2, 2, tau, false};
  lingtu_dds_NavigationCommandAck ack{};
  ack.header.frame_id = const_cast<char *>("body");
  ack.task_id = const_cast<char *>("");
  ack.request_id = const_cast<char *>("read-only-observer");
  ack.reason = const_cast<char *>("line1\nline2");
  ack.accepted = true;
  lingtu_dds_PointCloud2 cloud{};
  cloud.header.frame_id = const_cast<char *>("map");
  cloud.width = 3;
  cloud.height = 2;
  cloud.point_step = 12;
  cloud.row_step = 36;
  std::vector<std::uint8_t> pixels(72, 0);
  cloud.data = {72, 72, pixels.data(), false};
  require(dds_write_ts(imu_writer, &imu, 1500000000LL) == DDS_RETCODE_OK, "IMU publish failed");
  require(dds_write_ts(joints_writer, &joints, 1500000000LL) == DDS_RETCODE_OK,
          "joints publish failed");
  require(dds_write_ts(ack_writer, &ack, 1500000000LL) == DDS_RETCODE_OK, "ACK publish failed");
  require(dds_write_ts(cloud_writer, &cloud, 1500000000LL) == DDS_RETCODE_OK,
          "cloud publish failed");
  std::string observed;
  for (int i = 0; i < 100; ++i) {
    observer.poll();
    observed = observer.snapshot();
    if (observed.find("\"point_count\":6") != std::string::npos &&
        observed.find("\"estimated_torque_nm\":[3.5,-2.25]") != std::string::npos &&
        observed.find("read-only-observer") != std::string::npos)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(observed.find("\"angular_velocity_rad_s\":{\"x\":1.25") != std::string::npos,
          "live IMU value not decoded");
  require(observed.find("\"estimated_torque_nm\":[3.5,-2.25]") != std::string::npos,
          "live joint values changed");
  require(observed.find("\"source_time_ns\":1500000000") != std::string::npos,
          "source time was replaced by receive time");
  require(observed.find("read-only-observer") != std::string::npos,
          "record-only message cannot be observed");
  require(observed.find("\"point_count\":6") != std::string::npos,
          "bounded point cloud metadata missing");
  require(observed.find("\"payload_omitted\":true") != std::string::npos,
          "large data preview is not explicitly summarized");
  require(observed.find("\"preview\":null") == std::string::npos,
          "received topic still has no preview");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  const auto quiet = observer.snapshot();
  require(quiet.find("\"rate_hz\":0") != std::string::npos, "quiet window kept old nonzero rate");
  require(quiet.find("\"preview\":null") == std::string::npos,
          "latest value lost during quiet interval");
  const std::string age_key = "\"last_sample_age_ms\":";
  const auto previous_age = std::stod(observed.substr(observed.find(age_key) + age_key.size()));
  const auto quiet_age = std::stod(quiet.substr(quiet.find(age_key) + age_key.size()));
  require(quiet_age > previous_age + 20, "quiet sample age was refreshed without a new message");
  dds_delete(participant);
}
