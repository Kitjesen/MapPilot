#include "nav_dds_runtime.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

namespace lingtu::nav::endpoint {
namespace {

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

std::uint64_t wallClockNanoseconds() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

std::uint64_t boottimeNanoseconds() {
  timespec value{};
  if (clock_gettime(CLOCK_BOOTTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_BOOTTIME) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
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

std::string makeProducerBootId(const std::string &host_boot_id, std::uint64_t start_boottime_ns) {
  return host_boot_id + ":" + std::to_string(static_cast<long long>(getpid())) + ":" +
         std::to_string(start_boottime_ns);
}

void fillHeader(lingtu_dds_Header &header, double stamp_s, const char *frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
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

struct PathMessage {
  lingtu_dds_Path msg{};
  std::vector<lingtu_dds_PoseStamped> poses;
};

PathMessage toDdsPath(const std::vector<nav_kernel::Vec3> &path, const char *frame_id,
                      double stamp = 0.0) {
  PathMessage out;
  fillHeader(out.msg.header, stamp, frame_id);
  stamp = static_cast<double>(out.msg.header.stamp.sec) +
          static_cast<double>(out.msg.header.stamp.nanosec) * 1e-9;
  out.poses.resize(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    fillHeader(out.poses[i].header, stamp, frame_id);
    out.poses[i].pose.position.x = path[i].x;
    out.poses[i].pose.position.y = path[i].y;
    out.poses[i].pose.position.z = path[i].z;
    out.poses[i].pose.orientation = quaternionFromYaw(0.0);
  }
  out.msg.poses._maximum = static_cast<std::uint32_t>(out.poses.size());
  out.msg.poses._length = static_cast<std::uint32_t>(out.poses.size());
  out.msg.poses._buffer = out.poses.data();
  out.msg.poses._release = false;
  return out;
}

lingtu_dds_FinalVelocityCommand toDdsFinalVelocity(const nav_kernel::Twist &cmd,
                                                   const std::string &host_boot_id,
                                                   const std::string &producer_boot_id,
                                                   std::uint64_t output_seq) {
  lingtu_dds_FinalVelocityCommand out{};
  out.host_boot_id = const_cast<char *>(host_boot_id.c_str());
  out.producer_boot_id = const_cast<char *>(producer_boot_id.c_str());
  out.output_seq = output_seq;
  out.source_boottime_ns = boottimeNanoseconds();
  out.source_wall_ns = wallClockNanoseconds();
  out.twist.linear.x = cmd.vx;
  out.twist.linear.y = cmd.vy;
  out.twist.angular.z = cmd.wz;
  return out;
}

std::string copyDdsString(const char *value) {
  return value == nullptr ? std::string{} : std::string{value};
}

double toSeconds(const lingtu_dds_Time &value) {
  return static_cast<double>(value.sec) + static_cast<double>(value.nanosec) * 1e-9;
}

double headerStampSeconds(const lingtu_dds_Header &header) {
  return toSeconds(header.stamp);
}

double safeHeaderStamp(double stamp_s) {
  constexpr double kOnePastMaxSeconds =
      static_cast<double>(std::numeric_limits<std::int32_t>::max()) + 1.0;
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0 || stamp_s >= kOnePastMaxSeconds) {
    return nowSeconds();
  }
  return stamp_s;
}

std::string frameIdOrMap(const std::string &frame_id) {
  return frame_id.empty() ? std::string{"map"} : frame_id;
}

bool executionCellCount(const lingtu_dds_ExplorationExecutionGrid &message,
                        std::size_t *cell_count) {
  constexpr std::size_t kMaximumExecutionCells = 1'000'000U;
  const std::size_t width = static_cast<std::size_t>(message.info.width);
  const std::size_t height = static_cast<std::size_t>(message.info.height);
  if (width == 0U || height == 0U || width > kMaximumExecutionCells / height) {
    return false;
  }
  *cell_count = width * height;
  return true;
}

template <typename Sequence>
bool hasExactExecutionPayload(const Sequence &sequence, std::size_t cell_count) {
  return sequence._buffer != nullptr && sequence._maximum >= sequence._length &&
         static_cast<std::size_t>(sequence._length) == cell_count;
}

lingtu_dds_PoseStamped toDdsPoseStamped(const nav_kernel::Vec3 &point, const char *frame_id) {
  lingtu_dds_PoseStamped out{};
  fillHeader(out.header, nowSeconds(), frame_id);
  out.pose.position.x = point.x;
  out.pose.position.y = point.y;
  out.pose.position.z = point.z;
  out.pose.orientation = quaternionFromYaw(0.0);
  return out;
}

}  // namespace

ExplorationExecutionGridView
copyExplorationExecutionGrid(const lingtu_dds_ExplorationExecutionGrid &message) {
  ExplorationExecutionGridView view;
  view.stamp_s = headerStampSeconds(message.header);
  view.frame_id = copyDdsString(message.header.frame_id);
  view.width = message.info.width;
  view.height = message.info.height;
  view.resolution = static_cast<double>(message.info.resolution);
  view.origin_x = message.info.origin.position.x;
  view.origin_y = message.info.origin.position.y;
  view.origin_z = message.info.origin.position.z;
  view.origin_qx = message.info.origin.orientation.x;
  view.origin_qy = message.info.origin.orientation.y;
  view.origin_qz = message.info.origin.orientation.z;
  view.origin_qw = message.info.origin.orientation.w;
  view.session_id = copyDdsString(message.session_id);
  view.reset_epoch = message.reset_epoch;
  view.generation = message.generation;
  view.live = message.live;
  view.terrain_risk_stamp_s = toSeconds(message.terrain_risk_stamp);
  view.terrain_risk_ready = message.terrain_risk_ready;

  std::size_t cell_count = 0U;
  if (!executionCellCount(message, &cell_count) ||
      !hasExactExecutionPayload(message.occupancy, cell_count) ||
      !hasExactExecutionPayload(message.terrain_cost, cell_count)) {
    return view;
  }
  view.occupancy.assign(message.occupancy._buffer, message.occupancy._buffer + cell_count);
  view.terrain_cost.assign(message.terrain_cost._buffer, message.terrain_cost._buffer + cell_count);
  view.payload_complete = true;
  return view;
}

ExplorationSegmentRequestView
copyExplorationSegmentRequest(const lingtu_dds_ExplorationSegmentRequest &message) {
  ExplorationSegmentRequestView view;
  view.stamp_s = headerStampSeconds(message.header);
  view.frame_id = copyDdsString(message.header.frame_id);
  view.request_id = copyDdsString(message.request_id);
  view.kind = static_cast<std::int32_t>(message.kind);
  view.session_id = copyDdsString(message.session_id);
  view.reset_epoch = message.reset_epoch;
  view.minimum_generation = message.minimum_generation;
  view.target.x = message.target.position.x;
  view.target.y = message.target.position.y;
  view.target.z = message.target.position.z;
  view.target.qx = message.target.orientation.x;
  view.target.qy = message.target.orientation.y;
  view.target.qz = message.target.orientation.z;
  view.target.qw = message.target.orientation.w;
  view.reason = copyDdsString(message.reason);
  return view;
}

DdsRuntime::DdsRuntime(int domain_id, bool allow_legacy_motion_inputs)
    : host_boot_id_(readHostBootId()),
      producer_boot_id_(makeProducerBootId(host_boot_id_, boottimeNanoseconds())) {
  participant_ =
      checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
              "dds_create_participant");
  subscriber_ =
      checked(dds_create_subscriber(participant_, nullptr, nullptr), "dds_create_subscriber");
  publisher_ =
      checked(dds_create_publisher(participant_, nullptr, nullptr), "dds_create_publisher");

  odom_reader_ =
      reader(lingtu::message::kSlamOdometry.dds_topic.data(), &lingtu_dds_Odometry_desc, "odom");
  tf_reader_ = reader(lingtu::message::kTf.dds_topic.data(), &lingtu_dds_TFMessage_desc, "tf");
  if (allow_legacy_motion_inputs) {
    legacy_goal_reader_ = reader(lingtu::message::kNavGoalPose.dds_topic.data(),
                                 &lingtu_dds_PoseStamped_desc, "goal_pose");
  }
  cloud_reader_ = reader(lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
                         &lingtu_dds_PointCloud2_desc, "registered_cloud");
  terrain_map_reader_ = reader(lingtu::message::kNavTerrainMap.dds_topic.data(),
                               &lingtu_dds_PointCloud2_desc, "terrain_map");
  terrain_map_ext_reader_ = reader(lingtu::message::kNavTerrainMapExt.dds_topic.data(),
                                   &lingtu_dds_PointCloud2_desc, "terrain_map_ext");
  if (allow_legacy_motion_inputs) {
    legacy_global_path_reader_ = reader(lingtu::message::kNavGlobalPath.dds_topic.data(),
                                        &lingtu_dds_Path_desc, "global_path");
  }
  traversability_reader_ = reader(lingtu::message::kNavTraversability.dds_topic.data(),
                                  &lingtu_dds_OccupancyGrid_desc, "traversability");
  localization_health_reader_ = reader(lingtu::message::kSlamLocalizationHealth.dds_topic.data(),
                                       &lingtu_dds_Text_desc, "localization_health");
  driver_control_state_reader_ =
      reader(lingtu::message::kDriverControlState.dds_topic.data(),
             &lingtu_dds_DriverControlState_desc, "driver_control_state");
  map_clearing_reader_ = reader(lingtu::message::kNavMapClearing.dds_topic.data(),
                                &lingtu_dds_Bool_desc, "map_clearing");
  cloud_clearing_reader_ = reader(lingtu::message::kNavCloudClearing.dds_topic.data(),
                                  &lingtu_dds_Bool_desc, "cloud_clearing");
  if (allow_legacy_motion_inputs) {
    legacy_cancel_reader_ =
        reader(lingtu::message::kNavCancel.dds_topic.data(), &lingtu_dds_Text_desc, "cancel");
    legacy_teleop_cmd_reader_ = reader(lingtu::message::kNavTeleopCmdVel.dds_topic.data(),
                                       &lingtu_dds_TwistStamped_desc, "teleop_cmd_vel");
  }
  command_request_reader_ =
      reader(lingtu::message::kNavCommandRequest.dds_topic.data(),
             &lingtu_dds_NavigationCommandRequest_desc, "nav_command_request");
  operator_motion_control_reader_ =
      reader(lingtu::message::kOperatorMotionControl.dds_topic.data(),
             &lingtu_dds_OperatorMotionControl_desc, "operator_motion_control");
  operator_motion_sample_reader_ =
      reader(lingtu::message::kOperatorMotionSample.dds_topic.data(),
             &lingtu_dds_OperatorMotionSample_desc, "operator_motion_sample");
  exploration_execution_grid_reader_ =
      reader(lingtu::message::kNavExplorationExecutionSnapshot.dds_topic.data(),
             &lingtu_dds_ExplorationExecutionGrid_desc, "exploration_execution_snapshot");
  exploration_segment_request_reader_ =
      reader(lingtu::message::kNavExplorationSegmentRequest.dds_topic.data(),
             &lingtu_dds_ExplorationSegmentRequest_desc, "exploration_segment_request");

  inspection_command_reader_ =
      reader(lingtu::message::kNavInspectionCommand.dds_topic.data(),
             &lingtu_dds_InspectionCommandRequest_desc, "inspection_command");
  inspection_evidence_result_reader_ =
      reader(lingtu::message::kNavInspectionEvidenceResult.dds_topic.data(),
             &lingtu_dds_InspectionEvidenceResult_desc, "inspection_evidence_result");
  local_path_writer_ =
      writer(lingtu::message::kNavLocalPath.dds_topic.data(), &lingtu_dds_Path_desc, "local_path");
  global_path_writer_ = writer(lingtu::message::kNavGlobalPath.dds_topic.data(),
                               &lingtu_dds_Path_desc, "global_path");
  way_point_writer_ = writer(lingtu::message::kNavWayPoint.dds_topic.data(),
                             &lingtu_dds_PoseStamped_desc, "way_point");
  cmd_vel_writer_ = writer(lingtu::message::kNavCmdVel.dds_topic.data(),
                           &lingtu_dds_FinalVelocityCommand_desc, "cmd_vel");
  command_ack_writer_ = writer(lingtu::message::kNavCommandAck.dds_topic.data(),
                               &lingtu_dds_NavigationCommandAck_desc, "nav_command_ack");
  goal_status_writer_ = writer(lingtu::message::kNavGoalStatus.dds_topic.data(),
                               &lingtu_dds_NavigationGoalStatus_desc, "nav_goal_status");
  navigation_state_writer_ = writer(lingtu::message::kNavState.dds_topic.data(),
                                    &lingtu_dds_NavigationState_desc, "navigation_state");
  operator_motion_ack_writer_ = writer(lingtu::message::kOperatorMotionAck.dds_topic.data(),
                                       &lingtu_dds_OperatorMotionAck_desc, "operator_motion_ack");
  operator_motion_status_writer_ =
      writer(lingtu::message::kOperatorMotionStatus.dds_topic.data(),
             &lingtu_dds_OperatorMotionStatus_desc, "operator_motion_status");
  exploration_segment_ack_writer_ =
      writer(lingtu::message::kNavExplorationSegmentAck.dds_topic.data(),
             &lingtu_dds_ExplorationSegmentAck_desc, "exploration_segment_ack");
  exploration_segment_status_writer_ =
      writer(lingtu::message::kNavExplorationSegmentStatus.dds_topic.data(),
             &lingtu_dds_ExplorationSegmentStatus_desc, "exploration_segment_status");

  inspection_ack_writer_ = writer(lingtu::message::kNavInspectionAck.dds_topic.data(),
                                  &lingtu_dds_InspectionCommandAck_desc, "inspection_ack");
  inspection_status_writer_ = writer(lingtu::message::kNavInspectionStatus.dds_topic.data(),
                                     &lingtu_dds_InspectionStatus_desc, "inspection_status");
  inspection_evidence_request_writer_ =
      writer(lingtu::message::kNavInspectionEvidenceRequest.dds_topic.data(),
             &lingtu_dds_InspectionEvidenceRequest_desc, "inspection_evidence_request");
}

DdsRuntime::~DdsRuntime() {
  if (participant_ > 0) {
    dds_delete(participant_);
  }
}

void DdsRuntime::writeLocalPath(const std::vector<nav_kernel::Vec3> &path) {
  PathMessage msg = toDdsPath(path, "map");
  logDdsError(dds_write(local_path_writer_, &msg.msg), "dds_write(local_path)");
}

void DdsRuntime::writeGlobalPath(const std::vector<nav_kernel::Vec3> &path, double stamp_s) {
  PathMessage msg = toDdsPath(path, "map", stamp_s);
  logDdsError(dds_write(global_path_writer_, &msg.msg), "dds_write(global_path)");
}

void DdsRuntime::writeWayPoint(const nav_kernel::Vec3 &point) {
  lingtu_dds_PoseStamped msg = toDdsPoseStamped(point, "map");
  logDdsError(dds_write(way_point_writer_, &msg), "dds_write(way_point)");
}

std::optional<FinalVelocityCommandReceipt>
DdsRuntime::writeCmdVelSequenced(const nav_kernel::Twist &cmd) {
  if (output_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    std::fputs("dds_write(cmd_vel): output sequence exhausted\n", stderr);
    return std::nullopt;
  }
  const std::uint64_t output_sequence = output_seq_ + 1U;
  lingtu_dds_FinalVelocityCommand msg =
      toDdsFinalVelocity(cmd, host_boot_id_, producer_boot_id_, output_sequence);
  const dds_return_t result = dds_write(cmd_vel_writer_, &msg);
  logDdsError(result, "dds_write(cmd_vel)");
  if (result < 0) {
    return std::nullopt;
  }
  output_seq_ = output_sequence;
  last_output_command_ = cmd;
  return FinalVelocityCommandReceipt{output_sequence, msg.source_wall_ns};
}

bool DdsRuntime::writeCmdVel(const nav_kernel::Twist &cmd) {
  return writeCmdVelSequenced(cmd).has_value();
}

bool DdsRuntime::writeOperatorMotionAck(const OperatorMotionAckSample &ack) {
  if (operator_motion_ack_writer_ <= 0) {
    return false;
  }
  lingtu_dds_OperatorMotionAck message{};
  fillHeader(message.header, nowSeconds(), "");
  message.source_id = const_cast<char *>(ack.source_id.c_str());
  message.source_epoch = ack.source_epoch;
  message.source_sequence = ack.sequence;
  message.request_id = const_cast<char *>(ack.request_id.c_str());
  message.action = ack.action;
  message.accepted = ack.accepted;
  message.reason = const_cast<char *>(ack.reason.c_str());
  message.accepted_sequence = ack.accepted_sequence;
  message.final_output_sequence = ack.final_output_sequence;
  const dds_return_t result = dds_write(operator_motion_ack_writer_, &message);
  logDdsError(result, "dds_write(operator_motion_ack)");
  return result >= 0;
}

bool DdsRuntime::writeOperatorMotionStatus(const OperatorMotionStatusSample &status) {
  if (operator_motion_status_writer_ <= 0) {
    return false;
  }
  lingtu_dds_OperatorMotionStatus message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.active_source_id = const_cast<char *>(status.active_source_id.c_str());
  message.active_source_epoch = status.active_source_epoch;
  message.has_active_authority = status.has_active_authority;
  message.holding = status.holding;
  message.has_active_sample = status.has_active_sample;
  message.last_sample_sequence = status.last_sample_sequence;
  message.admitted_sequence = status.admitted_sequence;
  message.final_output_sequence = status.final_output_sequence;
  message.authority_reason = const_cast<char *>(status.authority_reason.c_str());
  message.input_gate_reason = const_cast<char *>(status.input_gate_reason.c_str());
  message.teleop_output.linear.x = status.teleop_output.vx;
  message.teleop_output.linear.y = status.teleop_output.vy;
  message.teleop_output.angular.z = status.teleop_output.wz;
  message.final_cmd_vel.linear.x = status.final_cmd_vel.vx;
  message.final_cmd_vel.linear.y = status.final_cmd_vel.vy;
  message.final_cmd_vel.angular.z = status.final_cmd_vel.wz;
  const dds_return_t result = dds_write(operator_motion_status_writer_, &message);
  logDdsError(result, "dds_write(operator_motion_status)");
  return result >= 0;
}

bool DdsRuntime::writeCommandAck(const char *request_id,
                                 lingtu::message::NavigationCommandKind kind, bool accepted,
                                 const char *reason) {
  lingtu_dds_NavigationCommandAck msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.request_id = const_cast<char *>(request_id == nullptr ? "" : request_id);
  msg.kind = static_cast<std::int32_t>(kind);
  msg.accepted = accepted;
  msg.reason = const_cast<char *>(reason == nullptr ? "" : reason);
  const dds_return_t result = dds_write(command_ack_writer_, &msg);
  logDdsError(result, "dds_write(nav_command_ack)");
  return result >= 0;
}

void DdsRuntime::writeNavigationGoalStatus(const char *request_id,
                                           lingtu::message::NavigationGoalState state,
                                           std::uint64_t goal_epoch, const char *reason) {
  if (goal_status_writer_ <= 0 || request_id == nullptr || *request_id == '\0' ||
      goal_status_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    return;
  }
  lingtu_dds_NavigationGoalStatus msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.boot_id = const_cast<char *>(producer_boot_id_.c_str());
  msg.event_sequence = goal_status_seq_ + 1U;
  msg.request_id = const_cast<char *>(request_id);
  msg.state = static_cast<std::int32_t>(state);
  msg.goal_epoch = goal_epoch;
  msg.reason = const_cast<char *>(reason == nullptr ? "" : reason);
  const dds_return_t result = dds_write(goal_status_writer_, &msg);
  logDdsError(result, "dds_write(nav_goal_status)");
  if (result >= 0) {
    goal_status_seq_ = msg.event_sequence;
  }
}

bool DdsRuntime::writeNavigationState(const NavigationStateSample &state) {
  if (navigation_state_writer_ <= 0 ||
      navigation_state_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  lingtu_dds_NavigationState message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.boot_id = const_cast<char *>(producer_boot_id_.c_str());
  message.state_sequence = navigation_state_seq_ + 1U;
  message.control_mode = state.control_mode;
  message.lifecycle_state = state.lifecycle_state;
  message.active_request_id = const_cast<char *>(state.active_request_id.c_str());
  message.goal_epoch = state.goal_epoch;
  message.map_id = const_cast<char *>(state.map_id.c_str());
  message.map_version = state.map_version;
  message.map_hash = const_cast<char *>(state.map_hash.c_str());
  message.planning_state = state.planning_state;
  message.execution_state = state.execution_state;
  message.recovery_state = state.recovery_state;
  message.progress = state.progress;
  message.authority = const_cast<char *>(state.authority.c_str());
  message.hold_reason = const_cast<char *>(state.hold_reason.c_str());
  message.failure_code = const_cast<char *>(state.failure_code.c_str());
  const dds_return_t result = dds_write(navigation_state_writer_, &message);
  logDdsError(result, "dds_write(navigation_state)");
  if (result < 0) {
    return false;
  }
  navigation_state_seq_ = message.state_sequence;
  return true;
}

bool DdsRuntime::writeExplorationSegmentAck(const ExplorationSegmentAck &ack) {
  if (exploration_segment_ack_writer_ <= 0) {
    return false;
  }
  const std::string frame_id = frameIdOrMap(ack.frame_id);
  lingtu_dds_ExplorationSegmentAck message{};
  fillHeader(message.header, safeHeaderStamp(ack.stamp_s), frame_id.c_str());
  message.request_id = const_cast<char *>(ack.request_id.c_str());
  message.kind = ack.kind;
  message.accepted = ack.accepted;
  message.session_id = const_cast<char *>(ack.session_id.c_str());
  message.reset_epoch = ack.reset_epoch;
  message.generation = ack.generation;
  message.live = ack.live;
  message.reason = const_cast<char *>(ack.reason.c_str());
  const dds_return_t result = dds_write(exploration_segment_ack_writer_, &message);
  logDdsError(result, "dds_write(exploration_segment_ack)");
  return result >= 0;
}

bool DdsRuntime::writeExplorationSegmentStatus(const ExplorationSegmentStatus &status) {
  if (exploration_segment_status_writer_ <= 0) {
    return false;
  }
  const std::string frame_id = frameIdOrMap(status.frame_id);
  lingtu_dds_ExplorationSegmentStatus message{};
  fillHeader(message.header, safeHeaderStamp(status.stamp_s), frame_id.c_str());
  message.request_id = const_cast<char *>(status.request_id.c_str());
  message.state = status.state;
  message.session_id = const_cast<char *>(status.session_id.c_str());
  message.reset_epoch = status.reset_epoch;
  message.generation = status.generation;
  message.live = status.live;
  message.reason = const_cast<char *>(status.reason.c_str());
  const dds_return_t result = dds_write(exploration_segment_status_writer_, &message);
  logDdsError(result, "dds_write(exploration_segment_status)");
  return result >= 0;
}

bool DdsRuntime::writeInspectionAck(const char *request_id,
                                    lingtu::nav::inspection::CommandKind kind, bool accepted,
                                    const char *reason, const char *run_id) {
  lingtu_dds_InspectionCommandAck msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.request_id = const_cast<char *>(request_id == nullptr ? "" : request_id);
  msg.kind = static_cast<std::int32_t>(kind);
  msg.accepted = accepted;
  msg.reason = const_cast<char *>(reason == nullptr ? "" : reason);
  msg.run_id = const_cast<char *>(run_id == nullptr ? "" : run_id);
  const dds_return_t result = dds_write(inspection_ack_writer_, &msg);
  logDdsError(result, "dds_write(inspection_ack)");
  return result >= 0;
}

void DdsRuntime::writeInspectionStatus(const lingtu::nav::inspection::RunStatus &status) {
  lingtu_dds_InspectionStatus msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.run_id = const_cast<char *>(status.run_id.c_str());
  msg.route_id = const_cast<char *>(status.route_id.c_str());
  msg.route_revision = status.route_revision;
  msg.state = static_cast<std::int32_t>(status.state);
  msg.point_index = static_cast<std::uint32_t>(status.point_index);
  msg.point_count = static_cast<std::uint32_t>(status.point_count);
  msg.loop_index = status.loop_index;
  msg.retry_count = status.retry_count;
  msg.point_id = const_cast<char *>(status.point_id.c_str());
  msg.action = const_cast<char *>(status.action.c_str());
  msg.action_request_id = const_cast<char *>(status.action_request_id.c_str());
  msg.evidence_id = const_cast<char *>(status.evidence_id.c_str());
  msg.phase_started_at = status.phase_started_at_s;
  msg.stable_since = status.stable_since_s;
  msg.deadline = status.deadline_s;
  msg.reason = const_cast<char *>(status.reason.c_str());
  logDdsError(dds_write(inspection_status_writer_, &msg), "dds_write(inspection_status)");
}

bool DdsRuntime::inspectionEvidenceWorkerMatched() const noexcept {
  if (inspection_evidence_request_writer_ <= 0) {
    return false;
  }
  dds_publication_matched_status_t status{};
  const dds_return_t result =
      dds_get_publication_matched_status(inspection_evidence_request_writer_, &status);
  if (result < 0) {
    logDdsError(result, "dds_get_publication_matched_status(inspection_evidence_request)");
    return false;
  }
  return status.current_count > 0U;
}

bool DdsRuntime::writeInspectionEvidenceRequest(
    const lingtu::nav::inspection::ActionRequest &request, const std::string &map_id,
    std::int64_t map_version, double deadline_s) {
  if (request.point_index > std::numeric_limits<std::uint32_t>::max() ||
      request.request_id.empty() || request.run_id.empty() || request.route_id.empty() ||
      request.point_id.empty() || request.action.empty() || map_id.empty() ||
      !std::isfinite(deadline_s) || deadline_s <= 0.0) {
    return false;
  }
  lingtu_dds_InspectionEvidenceRequest msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.request_id = const_cast<char *>(request.request_id.c_str());
  msg.run_id = const_cast<char *>(request.run_id.c_str());
  msg.route_id = const_cast<char *>(request.route_id.c_str());
  msg.revision = request.route_revision;
  msg.map_id = const_cast<char *>(map_id.c_str());
  msg.map_version = map_version;
  msg.point_index = static_cast<std::uint32_t>(request.point_index);
  msg.point_id = const_cast<char *>(request.point_id.c_str());
  msg.action = const_cast<char *>(request.action.c_str());
  msg.deadline_s = deadline_s;
  const dds_return_t result = dds_write(inspection_evidence_request_writer_, &msg);
  logDdsError(result, "dds_write(inspection_evidence_request)");
  return result >= 0;
}

void DdsRuntime::logDdsError(dds_return_t value, const char *what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

dds_entity_t DdsRuntime::reader(const char *topic_name, const dds_topic_descriptor_t *desc,
                                const char *label) {
  const dds_entity_t topic =
      checked(dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
              (std::string("dds_create_topic(") + label + ")").c_str());
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
  const dds_entity_t entity = checked(dds_create_reader(subscriber_, topic, qos.get(), nullptr),
                                      (std::string("dds_create_reader(") + label + ")").c_str());
  return entity;
}

dds_entity_t DdsRuntime::writer(const char *topic_name, const dds_topic_descriptor_t *desc,
                                const char *label) {
  const dds_entity_t topic =
      checked(dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
              (std::string("dds_create_topic(") + label + ")").c_str());
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
  return checked(dds_create_writer(publisher_, topic, qos.get(), nullptr),
                 (std::string("dds_create_writer(") + label + ")").c_str());
}

}  // namespace lingtu::nav::endpoint
