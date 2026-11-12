#include "dds.hpp"

#include "transport/dds/qos.hpp"
#include "message/generated/topics.hpp"
#include "message/protocol/navigation.hpp"

#include "dds/dds.h"
#include "messages.h"

#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <stdexcept>
#include <string>

namespace lingtu::driver {
namespace {

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = 0.0;
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

std::string readHostBootId() {
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string value;
  std::getline(input, value);
  if (value.empty()) {
    throw std::runtime_error("failed to read Linux host boot id");
  }
  return value;
}

std::uint64_t boottimeNanoseconds() {
  timespec value{};
  if (clock_gettime(CLOCK_BOOTTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_BOOTTIME) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
      static_cast<std::uint64_t>(value.tv_nsec);
}

}  // namespace

struct DdsReader::Impl {
  explicit Impl(int domain_id) : host_boot_id(readHostBootId()) {
    participant = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber = checked(
        dds_create_subscriber(participant, nullptr, nullptr), "dds_create_subscriber");
    publisher = checked(
        dds_create_publisher(participant, nullptr, nullptr), "dds_create_publisher");
    topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_FinalVelocityCommand_desc,
            lingtu::message::kNavCmdVel.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(cmd_vel)");
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
    reader = checked(
        dds_create_reader(subscriber, topic, qos.get(), nullptr),
        "dds_create_reader(cmd_vel)");

    control_state_topic = checked(
        dds_create_topic(
            participant,
            &lingtu_dds_DriverControlState_desc,
            lingtu::message::kDriverControlState.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(driver_control_state)");
    auto control_qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(
            lingtu::message::kDriverControlState.dds_topic));
    control_state_writer = checked(
        dds_create_writer(
            publisher, control_state_topic, control_qos.get(), nullptr),
        "dds_create_writer(driver_control_state)");

    const auto profile_request_topic = checked(dds_create_topic(participant,
        &lingtu_dds_MotionProfileRequest_desc,
        lingtu::message::kMotionProfileRequest.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(profile_request)");
    auto profile_request_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
        lingtu::message::kMotionProfileRequest.dds_topic));
    profile_reader = checked(dds_create_reader(subscriber, profile_request_topic,
        profile_request_qos.get(), nullptr), "dds_create_reader(profile_request)");
    const auto profile_state_topic = checked(dds_create_topic(participant,
        &lingtu_dds_MotionProfileState_desc,
        lingtu::message::kMotionProfileState.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(profile_state)");
    auto profile_state_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
        lingtu::message::kMotionProfileState.dds_topic));
    profile_writer = checked(dds_create_writer(publisher, profile_state_topic,
        profile_state_qos.get(), nullptr), "dds_create_writer(profile_state)");

    const auto posture_topic = checked(dds_create_topic(participant,
        &lingtu_dds_MotionPostureRequest_desc,
        message::kMotionPostureRequest.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(posture_request)");
    auto posture_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
        message::kMotionPostureRequest.dds_topic));
    posture_reader = checked(dds_create_reader(subscriber, posture_topic,
        posture_qos.get(), nullptr), "dds_create_reader(posture_request)");
    const auto posture_ack_topic = checked(dds_create_topic(participant,
        &lingtu_dds_MotionPostureAck_desc,
        message::kMotionPostureAck.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(posture_ack)");
    auto posture_ack_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
        message::kMotionPostureAck.dds_topic));
    posture_writer = checked(dds_create_writer(publisher, posture_ack_topic,
        posture_ack_qos.get(), nullptr), "dds_create_writer(posture_ack)");
    const auto external_topic = checked(dds_create_topic(participant,
        &lingtu_dds_ExternalCommand_desc, message::kExternalCommand.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(external_command)");
    auto external_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(message::kExternalCommand.dds_topic));
    external_reader = checked(dds_create_reader(subscriber, external_topic,
        external_qos.get(), nullptr), "dds_create_reader(external_command)");
    const auto external_state_topic = checked(dds_create_topic(participant,
        &lingtu_dds_ExternalControlState_desc, message::kExternalControlState.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(external_state)");
    auto external_state_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(message::kExternalControlState.dds_topic));
    external_writer = checked(dds_create_writer(publisher, external_state_topic,
        external_state_qos.get(), nullptr), "dds_create_writer(external_state)");

    joint_state_topic = checked(
        dds_create_topic(participant, &lingtu_dds_JointState_desc,
                         lingtu::message::kRobotJointStates.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(robot_joint_states)");
    auto joint_qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kRobotJointStates.dds_topic));
    joint_state_writer = checked(
        dds_create_writer(publisher, joint_state_topic, joint_qos.get(), nullptr),
        "dds_create_writer(robot_joint_states)");

  }

  ~Impl() {
    if (participant > 0) {
      dds_delete(participant);
    }
  }

  dds_entity_t participant{0};
  dds_entity_t subscriber{0};
  dds_entity_t publisher{0};
  dds_entity_t topic{0};
  dds_entity_t reader{0};
  dds_entity_t control_state_topic{0};
  dds_entity_t control_state_writer{0};
  dds_entity_t profile_reader{0};
  dds_entity_t profile_writer{0};
  dds_entity_t posture_reader{0};
  dds_entity_t posture_writer{0};
  dds_entity_t external_reader{0};
  dds_entity_t external_writer{0};
  dds_entity_t joint_state_topic{0};
  dds_entity_t joint_state_writer{0};
  std::string host_boot_id;
};

DdsReader::DdsReader(int domain_id) : impl_(std::make_unique<Impl>(domain_id)) {}
DdsReader::~DdsReader() = default;

std::optional<message::PostureRequest> DdsReader::takePostureRequest() {
  void *samples[8]{};
  dds_sample_info_t infos[8]{};
  const auto count = dds_take(impl_->posture_reader, samples, infos, 8, 8);
  if (count < 0) throw std::runtime_error("dds_take(posture_request) failed");
  std::optional<message::PostureRequest> latest;
  const bool one_writer = dds_get_matched_publications(impl_->posture_reader, nullptr, 0) == 1;
  const auto now_ns = boottimeNanoseconds();
  for (dds_return_t i = 0; i < count; ++i) {
    if (!infos[i].valid_data || !one_writer) continue;
    const auto &msg = *static_cast<lingtu_dds_MotionPostureRequest *>(samples[i]);
    if (!msg.host_boot_id || impl_->host_boot_id != msg.host_boot_id ||
        msg.source_boottime_ns > now_ns || now_ns - msg.source_boottime_ns > 1000000000ULL ||
        !msg.producer_boot_id || !*msg.producer_boot_id ||
        !msg.client_id || !*msg.client_id || !msg.request_id || !*msg.request_id) continue;
    latest = message::PostureRequest{msg.producer_boot_id, msg.client_id, msg.request_id,
                                     msg.action, msg.phase, msg.profile_name ? msg.profile_name : "",
                                     msg.external_session_id ? msg.external_session_id : ""};
  }
  if (count > 0) dds_return_loan(impl_->posture_reader, samples, count);
  return latest;
}

bool DdsReader::writePostureAck(const message::PostureAck &ack, double stamp_s) {
  lingtu_dds_MotionPostureAck msg{};
  fillHeader(msg.header, stamp_s, "body");
  msg.producer_boot_id = const_cast<char *>(ack.request.producer_boot_id.c_str());
  msg.client_id = const_cast<char *>(ack.request.client_id.c_str());
  msg.request_id = const_cast<char *>(ack.request.request_id.c_str());
  msg.action = ack.request.action;
  msg.phase = ack.request.phase;
  msg.profile_name = const_cast<char *>(ack.request.profile_name.c_str());
  msg.external_session_id = const_cast<char *>(ack.request.external_session_id.c_str());
  msg.final_result = ack.final_result;
  msg.accepted = ack.accepted;
  msg.reason = const_cast<char *>(ack.reason.c_str());
  return dds_write(impl_->posture_writer, &msg) >= 0;
}

DdsReader::ExternalRead DdsReader::takeExternal() {
  ExternalRead result;
  void *samples[32]{};
  dds_sample_info_t infos[32]{};
  const auto count = dds_take(impl_->external_reader, samples, infos, 32, 32);
  if (count < 0) throw std::runtime_error("dds_take(external_command) failed");
  const bool one_writer = dds_get_matched_publications(impl_->external_reader, nullptr, 0) == 1;
  const auto now = boottimeNanoseconds();
  for (dds_return_t i = 0; i < count; ++i) {
    if (!one_writer || !infos[i].valid_data) continue;
    const auto &msg = *static_cast<lingtu_dds_ExternalCommand *>(samples[i]);
    if (!msg.host_boot_id || impl_->host_boot_id != msg.host_boot_id || !msg.session_id ||
        !*msg.session_id || msg.source_boottime_ns > now ||
        now - msg.source_boottime_ns > 300000000ULL) continue;
    if (msg.heartbeat) {
      result.heartbeat_session = msg.session_id;
      continue;
    }
    message::ExternalFrame frame;
    frame.session_id = msg.session_id;
    frame.sequence = msg.frame_sequence;
    frame.input_window = msg.input_window;
    for (std::size_t joint = 0; joint < frame.joints.size(); ++joint) {
      const auto &source = msg.joints[joint];
      frame.joints[joint] = {source.q, source.dq, source.kp, source.kd, source.tau};
    }
    result.frame = std::move(frame);
  }
  if (count > 0) dds_return_loan(impl_->external_reader, samples, count);
  return result;
}

bool DdsReader::writeExternalState(const message::ExternalState &state, double stamp_s) {
  lingtu_dds_ExternalControlState msg{};
  fillHeader(msg.header, stamp_s, "body");
  msg.session_id = const_cast<char *>(state.session_id.c_str());
  msg.selected = state.selected;
  msg.active = state.active;
  msg.motors_enabled = state.motors_enabled;
  msg.last_sequence = state.last_sequence;
  msg.reason = const_cast<char *>(state.reason.c_str());
  msg.input_window = state.input_window;
  msg.joint_count = state.joint_count;
  msg.fresh = state.fresh;
  msg.motor_disable_confirmed = state.motor_disable_confirmed;
  return dds_write(impl_->external_writer, &msg) >= 0;
}

std::optional<ProfileRequest> DdsReader::takeProfileRequest(double wall_s) {
  void *samples[8]{};
  dds_sample_info_t infos[8]{};
  const auto count = dds_take(impl_->profile_reader, samples, infos, 8, 8);
  if (count < 0) throw std::runtime_error("dds_take(profile_request) failed");
  std::optional<ProfileRequest> latest;
  const bool one_writer = dds_get_matched_publications(impl_->profile_reader, nullptr, 0) == 1;
  for (dds_return_t i = 0; i < count; ++i) {
    if (!infos[i].valid_data || !one_writer) continue;
    const auto &msg = *static_cast<lingtu_dds_MotionProfileRequest *>(samples[i]);
    const double stamp = static_cast<double>(msg.header.stamp.sec) +
                         static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    if (wall_s - stamp < -0.5 || wall_s - stamp > 1.0) continue;
    latest = ProfileRequest{msg.task_id ? msg.task_id : "",
                            msg.requirement ? msg.requirement : ""};
  }
  if (count > 0) dds_return_loan(impl_->profile_reader, samples, count);
  return latest;
}

bool DdsReader::writeProfileState(const ProfileStatus &state, double stamp_s) {
  lingtu_dds_MotionProfileState msg{};
  fillHeader(msg.header, stamp_s, "body");
  msg.task_id = const_cast<char *>(state.task_id.c_str());
  msg.requirement = const_cast<char *>(state.requirement.c_str());
  msg.phase = const_cast<char *>(state.phase.c_str());
  msg.requested_profile = const_cast<char *>(state.requested_profile.c_str());
  msg.active_profile = const_cast<char *>(state.active_profile.c_str());
  msg.reason = const_cast<char *>(state.reason.c_str());
  return dds_write(impl_->profile_writer, &msg) == DDS_RETCODE_OK;
}

const std::string& DdsReader::hostBootId() const noexcept {
  return impl_->host_boot_id;
}

std::uint32_t DdsReader::matchedCommandWriters() const {
  const dds_return_t count =
      dds_get_matched_publications(impl_->reader, nullptr, 0);
  if (count < 0) {
    throw std::runtime_error(
        std::string("dds_get_matched_publications(cmd_vel): ") +
        dds_strretcode(-count));
  }
  return static_cast<std::uint32_t>(count);
}

ReadResult DdsReader::takeLatest() {
  constexpr std::size_t kMaxSamples = 8;
  void* samples[kMaxSamples]{};
  dds_sample_info_t infos[kMaxSamples];

  ReadResult result;
  const dds_return_t count =
      dds_take(impl_->reader, samples, infos, kMaxSamples, kMaxSamples);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_take(cmd_vel): ") + dds_strretcode(-count));
  }

  for (dds_return_t i = 0; i < count; ++i) {
    if (!infos[i].valid_data) {
      continue;
    }
    const auto& msg =
        *static_cast<lingtu_dds_FinalVelocityCommand*>(samples[i]);
    TwistSample latest;
    latest.host_boot_id = msg.host_boot_id ? msg.host_boot_id : "";
    latest.producer_boot_id =
        msg.producer_boot_id ? msg.producer_boot_id : "";
    latest.source_boottime_ns = msg.source_boottime_ns;
    latest.received_at = Clock::now();
    latest.receive_boottime_ns = boottimeNanoseconds();
    latest.vx = msg.twist.linear.x;
    latest.vy = msg.twist.linear.y;
    latest.wz = msg.twist.angular.z;
    result.latest = std::move(latest);
    ++result.valid_samples;
  }
  if (count > 0) {
    const dds_return_t returned = dds_return_loan(impl_->reader, samples, count);
    if (returned < 0) {
      throw std::runtime_error(
          std::string("dds_return_loan(cmd_vel): ") + dds_strretcode(-returned));
    }
  }
  return result;
}

bool DdsReader::writeControlState(
    const ControlState& state,
    bool zero_output,
    double stamp_s) {
  lingtu_dds_DriverControlState msg{};
  fillHeader(msg.header, stamp_s, "body");
  msg.connected = state.connected;
  msg.ready = state.ready;
  msg.motors_enabled = state.motors_enabled;
  msg.critical_fault = state.critical_fault;
  msg.control_assured = state.control_assured;
  msg.lease_valid = state.lease_valid;
  msg.lease_remaining_ms = state.lease_remaining_ms;
  msg.accepted_sequence = state.accepted_sequence;
  msg.zero_output = zero_output;
  msg.fsm = const_cast<char*>(state.fsm.c_str());
  msg.owner = const_cast<char*>(state.owner.c_str());
  msg.owner_id = const_cast<char*>(state.owner_id.c_str());
  msg.reason = const_cast<char*>(state.reason.c_str());
  return dds_write(impl_->control_state_writer, &msg) >= 0;
}

bool DdsReader::writeJointState(const JointState& state) {
  lingtu_dds_JointState msg{};
  fillHeader(msg.header, state.stamp_s, "body");
  msg.robot_model = const_cast<char*>(state.robot_model.c_str());
  std::vector<char*> names;
  names.reserve(state.names.size());
  for (const auto& name : state.names) names.push_back(const_cast<char*>(name.c_str()));
  const auto count = static_cast<std::uint32_t>(names.size());
  msg.names = {count, count, names.data(), false};
  msg.position = {count, count, const_cast<double*>(state.position.data()), false};
  msg.velocity = {count, count, const_cast<double*>(state.velocity.data()), false};
  msg.effort = {count, count, const_cast<double*>(state.effort.data()), false};
  return dds_write(impl_->joint_state_writer, &msg) >= 0;
}

}  // namespace lingtu::driver
