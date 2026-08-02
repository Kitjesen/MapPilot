#include "lingtu/recording/topic_catalog.hpp"

#include <string_view>

#include "lingtu_slam.h"

namespace lingtu::recording {
namespace {

constexpr dds_free_op_t kFreeAll = static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT);

void *allocate_livox_frame() {
  return lingtu_dds_LivoxFrame__alloc();
}
void free_livox_frame(void *sample) {
  lingtu_dds_LivoxFrame_free(sample, kFreeAll);
}
void *allocate_imu() {
  return lingtu_dds_Imu__alloc();
}
void free_imu(void *sample) {
  lingtu_dds_Imu_free(sample, kFreeAll);
}
void *allocate_odometry() {
  return lingtu_dds_Odometry__alloc();
}
void free_odometry(void *sample) {
  lingtu_dds_Odometry_free(sample, kFreeAll);
}
void *allocate_point_cloud() {
  return lingtu_dds_PointCloud2__alloc();
}
void free_point_cloud(void *sample) {
  lingtu_dds_PointCloud2_free(sample, kFreeAll);
}
void *allocate_tf_message() {
  return lingtu_dds_TFMessage__alloc();
}
void free_tf_message(void *sample) {
  lingtu_dds_TFMessage_free(sample, kFreeAll);
}
void *allocate_map_observation() {
  return lingtu_dds_MapObservation__alloc();
}
void free_map_observation(void *sample) {
  lingtu_dds_MapObservation_free(sample, kFreeAll);
}
void *allocate_gnss_fix() {
  return lingtu_dds_GnssFix__alloc();
}
void free_gnss_fix(void *sample) {
  lingtu_dds_GnssFix_free(sample, kFreeAll);
}
void *allocate_gnss_status() {
  return lingtu_dds_GnssStatus__alloc();
}
void free_gnss_status(void *sample) {
  lingtu_dds_GnssStatus_free(sample, kFreeAll);
}
void *allocate_text() {
  return lingtu_dds_Text__alloc();
}
void free_text(void *sample) {
  lingtu_dds_Text_free(sample, kFreeAll);
}
void *allocate_float32() {
  return lingtu_dds_Float32__alloc();
}
void free_float32(void *sample) {
  lingtu_dds_Float32_free(sample, kFreeAll);
}
void *allocate_navigation_command_request() {
  return lingtu_dds_NavigationCommandRequest__alloc();
}
void free_navigation_command_request(void *sample) {
  lingtu_dds_NavigationCommandRequest_free(sample, kFreeAll);
}
void *allocate_navigation_command_ack() {
  return lingtu_dds_NavigationCommandAck__alloc();
}
void free_navigation_command_ack(void *sample) {
  lingtu_dds_NavigationCommandAck_free(sample, kFreeAll);
}
void *allocate_operator_motion_control() {
  return lingtu_dds_OperatorMotionControl__alloc();
}
void free_operator_motion_control(void *sample) {
  lingtu_dds_OperatorMotionControl_free(sample, kFreeAll);
}
void *allocate_operator_motion_sample() {
  return lingtu_dds_OperatorMotionSample__alloc();
}
void free_operator_motion_sample(void *sample) {
  lingtu_dds_OperatorMotionSample_free(sample, kFreeAll);
}
void *allocate_operator_motion_ack() {
  return lingtu_dds_OperatorMotionAck__alloc();
}
void free_operator_motion_ack(void *sample) {
  lingtu_dds_OperatorMotionAck_free(sample, kFreeAll);
}
void *allocate_inspection_task_request() {
  return lingtu_dds_InspectionTaskRequest__alloc();
}
void free_inspection_task_request(void *sample) {
  lingtu_dds_InspectionTaskRequest_free(sample, kFreeAll);
}
void *allocate_inspection_task_ack() {
  return lingtu_dds_InspectionTaskAck__alloc();
}
void free_inspection_task_ack(void *sample) {
  lingtu_dds_InspectionTaskAck_free(sample, kFreeAll);
}
void *allocate_inspection_status() {
  return lingtu_dds_InspectionStatus__alloc();
}
void free_inspection_status(void *sample) {
  lingtu_dds_InspectionStatus_free(sample, kFreeAll);
}
void *allocate_inspection_evidence_request() {
  return lingtu_dds_InspectionEvidenceRequest__alloc();
}
void free_inspection_evidence_request(void *sample) {
  lingtu_dds_InspectionEvidenceRequest_free(sample, kFreeAll);
}
void *allocate_driver_control_state() {
  return lingtu_dds_DriverControlState__alloc();
}
void free_driver_control_state(void *sample) {
  lingtu_dds_DriverControlState_free(sample, kFreeAll);
}
void *allocate_navigation_goal_status() {
  return lingtu_dds_NavigationGoalStatus__alloc();
}
void free_navigation_goal_status(void *sample) {
  lingtu_dds_NavigationGoalStatus_free(sample, kFreeAll);
}
void *allocate_navigation_state() {
  return lingtu_dds_NavigationState__alloc();
}
void free_navigation_state(void *sample) {
  lingtu_dds_NavigationState_free(sample, kFreeAll);
}
void *allocate_operator_motion_status() {
  return lingtu_dds_OperatorMotionStatus__alloc();
}
void free_operator_motion_status(void *sample) {
  lingtu_dds_OperatorMotionStatus_free(sample, kFreeAll);
}
void *allocate_path() {
  return lingtu_dds_Path__alloc();
}
void free_path(void *sample) {
  lingtu_dds_Path_free(sample, kFreeAll);
}
void *allocate_final_velocity_command() {
  return lingtu_dds_FinalVelocityCommand__alloc();
}
void free_final_velocity_command(void *sample) {
  lingtu_dds_FinalVelocityCommand_free(sample, kFreeAll);
}
void *allocate_inspection_task_event() {
  return lingtu_dds_InspectionTaskEvent__alloc();
}
void free_inspection_task_event(void *sample) {
  lingtu_dds_InspectionTaskEvent_free(sample, kFreeAll);
}
void *allocate_inspection_evidence_result() {
  return lingtu_dds_InspectionEvidenceResult__alloc();
}
void free_inspection_evidence_result(void *sample) {
  lingtu_dds_InspectionEvidenceResult_free(sample, kFreeAll);
}

std::string canonical_topic(std::string topic) {
  if (topic.rfind("rt/", 0) == 0) {
    return "/" + topic.substr(3);
  }
  if (topic.empty() || topic.front() != '/') {
    return "/" + topic;
  }
  return topic;
}

}  // namespace

const std::vector<TopicBinding> &sensor_topic_catalog() {
  static const std::vector<TopicBinding> bindings{
      {&lingtu::message::kTf, &lingtu_dds_TFMessage_desc, lingtu::dds::QosProfile::TfDynamic,
       "tf_dynamic", allocate_tf_message, free_tf_message, ReplayPolicy::Replayable},
      {&lingtu::message::kTfStatic, &lingtu_dds_TFMessage_desc, lingtu::dds::QosProfile::TfStatic,
       "tf_static", allocate_tf_message, free_tf_message, ReplayPolicy::Replayable},
      {&lingtu::message::kLidarRawFrame, &lingtu_dds_LivoxFrame_desc,
       lingtu::dds::QosProfile::RawLidarStream, "raw_lidar_stream", allocate_livox_frame,
       free_livox_frame, ReplayPolicy::Replayable},
      {&lingtu::message::kImuRaw, &lingtu_dds_Imu_desc, lingtu::dds::QosProfile::SensorStream,
       "sensor_stream", allocate_imu, free_imu, ReplayPolicy::Replayable},
      {&lingtu::message::kSlamOdomPrior, &lingtu_dds_Odometry_desc,
       lingtu::dds::QosProfile::SensorStream, "sensor_stream", allocate_odometry, free_odometry,
       ReplayPolicy::Replayable},
      {&lingtu::message::kDriverOdometry, &lingtu_dds_Odometry_desc,
       lingtu::dds::QosProfile::HighFreqState, "high_freq_state", allocate_odometry, free_odometry,
       ReplayPolicy::Replayable},
      {&lingtu::message::kSlamOdometry, &lingtu_dds_Odometry_desc,
       lingtu::dds::QosProfile::HighFreqState, "high_freq_state", allocate_odometry, free_odometry,
       ReplayPolicy::Replayable},
      {&lingtu::message::kSlamStateAtScan, &lingtu_dds_Odometry_desc,
       lingtu::dds::QosProfile::HighFreqState, "high_freq_state", allocate_odometry, free_odometry,
       ReplayPolicy::Replayable},
      {&lingtu::message::kSlamRegisteredCloud, &lingtu_dds_PointCloud2_desc,
       lingtu::dds::QosProfile::LidarPointcloud, "lidar_pointcloud", allocate_point_cloud,
       free_point_cloud, ReplayPolicy::Replayable},
      {&lingtu::message::kSlamMapObservation, &lingtu_dds_MapObservation_desc,
       lingtu::dds::QosProfile::LidarPointcloud, "lidar_pointcloud", allocate_map_observation,
       free_map_observation, ReplayPolicy::Replayable},
      {&lingtu::message::kSlamMapCloud, &lingtu_dds_PointCloud2_desc,
       lingtu::dds::QosProfile::LidarPointcloud, "lidar_pointcloud", allocate_point_cloud,
       free_point_cloud, ReplayPolicy::Replayable},
      {&lingtu::message::kGnssFix, &lingtu_dds_GnssFix_desc,
       lingtu::dds::QosProfile::HighFreqState, "high_freq_state", allocate_gnss_fix, free_gnss_fix,
       ReplayPolicy::Replayable},
      {&lingtu::message::kGnssOdom, &lingtu_dds_Odometry_desc,
       lingtu::dds::QosProfile::HighFreqState, "high_freq_state", allocate_odometry, free_odometry,
       ReplayPolicy::Replayable},
  };
  return bindings;
}

const std::vector<TopicBinding> &recording_topic_catalog() {
  static const std::vector<TopicBinding> bindings = [] {
    auto result = sensor_topic_catalog();
    result.insert(
        result.end(),
        {
            {&lingtu::message::kDriverControlState, &lingtu_dds_DriverControlState_desc,
             lingtu::dds::QosProfile::SystemStatus, "system_status", allocate_driver_control_state,
             free_driver_control_state},
            {&lingtu::message::kNavGoalStatus, &lingtu_dds_NavigationGoalStatus_desc,
             lingtu::dds::QosProfile::CommandAck, "command_ack", allocate_navigation_goal_status,
             free_navigation_goal_status},
            {&lingtu::message::kNavState, &lingtu_dds_NavigationState_desc,
             lingtu::dds::QosProfile::SystemStatus, "system_status", allocate_navigation_state,
             free_navigation_state},
            {&lingtu::message::kOperatorMotionStatus, &lingtu_dds_OperatorMotionStatus_desc,
             lingtu::dds::QosProfile::OperatorMotionStatus, "operator_motion_status",
             allocate_operator_motion_status, free_operator_motion_status},
            {&lingtu::message::kNavGlobalPath, &lingtu_dds_Path_desc,
             lingtu::dds::QosProfile::GlobalPath, "global_path", allocate_path, free_path},
            {&lingtu::message::kNavLocalPath, &lingtu_dds_Path_desc,
             lingtu::dds::QosProfile::GlobalPath, "global_path", allocate_path, free_path},
            {&lingtu::message::kNavCmdVel, &lingtu_dds_FinalVelocityCommand_desc,
             lingtu::dds::QosProfile::FinalVelocityCommand, "final_velocity_command",
             allocate_final_velocity_command, free_final_velocity_command},
            {&lingtu::message::kNavInspectionTaskEvent, &lingtu_dds_InspectionTaskEvent_desc,
             lingtu::dds::QosProfile::TaskEvent, "task_event", allocate_inspection_task_event,
             free_inspection_task_event},
            {&lingtu::message::kNavInspectionEvidenceResult,
             &lingtu_dds_InspectionEvidenceResult_desc, lingtu::dds::QosProfile::InspectionEvidence,
             "inspection_evidence", allocate_inspection_evidence_result,
             free_inspection_evidence_result},
            {&lingtu::message::kGnssStatus, &lingtu_dds_GnssStatus_desc,
             lingtu::dds::QosProfile::SystemStatus, "system_status", allocate_gnss_status,
             free_gnss_status},
            {&lingtu::message::kSlamLocalizationHealth, &lingtu_dds_Text_desc,
             lingtu::dds::QosProfile::Default, "default", allocate_text, free_text},
            {&lingtu::message::kSlamLocalizationQuality, &lingtu_dds_Float32_desc,
             lingtu::dds::QosProfile::Default, "default", allocate_float32, free_float32},
            {&lingtu::message::kNavCommandRequest, &lingtu_dds_NavigationCommandRequest_desc,
             lingtu::dds::QosProfile::CommandRequest, "command_request",
             allocate_navigation_command_request, free_navigation_command_request},
            {&lingtu::message::kNavCommandAck, &lingtu_dds_NavigationCommandAck_desc,
             lingtu::dds::QosProfile::CommandAck, "command_ack", allocate_navigation_command_ack,
             free_navigation_command_ack},
            {&lingtu::message::kOperatorMotionControl, &lingtu_dds_OperatorMotionControl_desc,
             lingtu::dds::QosProfile::OperatorMotionControl, "operator_motion_control",
             allocate_operator_motion_control, free_operator_motion_control},
            {&lingtu::message::kOperatorMotionSample, &lingtu_dds_OperatorMotionSample_desc,
             lingtu::dds::QosProfile::OperatorMotionSample, "operator_motion_sample",
             allocate_operator_motion_sample, free_operator_motion_sample},
            {&lingtu::message::kOperatorMotionAck, &lingtu_dds_OperatorMotionAck_desc,
             lingtu::dds::QosProfile::OperatorMotionAck, "operator_motion_ack",
             allocate_operator_motion_ack, free_operator_motion_ack},
            {&lingtu::message::kNavInspectionTaskRequest, &lingtu_dds_InspectionTaskRequest_desc,
             lingtu::dds::QosProfile::CommandRequest, "command_request",
             allocate_inspection_task_request, free_inspection_task_request},
            {&lingtu::message::kNavInspectionTaskAck, &lingtu_dds_InspectionTaskAck_desc,
             lingtu::dds::QosProfile::CommandAck, "command_ack", allocate_inspection_task_ack,
             free_inspection_task_ack},
            {&lingtu::message::kNavInspectionStatus, &lingtu_dds_InspectionStatus_desc,
             lingtu::dds::QosProfile::SystemStatus, "system_status", allocate_inspection_status,
             free_inspection_status},
            {&lingtu::message::kNavInspectionEvidenceRequest,
             &lingtu_dds_InspectionEvidenceRequest_desc,
             lingtu::dds::QosProfile::InspectionEvidence, "inspection_evidence",
             allocate_inspection_evidence_request, free_inspection_evidence_request},
        });
    return result;
  }();
  return bindings;
}

const TopicBinding *find_sensor_topic(const std::string &topic) {
  const auto *binding = find_recording_topic(topic);
  return binding != nullptr && binding->replay_policy == ReplayPolicy::Replayable ? binding
                                                                                  : nullptr;
}

const TopicBinding *find_recording_topic(const std::string &topic) {
  const std::string canonical = canonical_topic(topic);
  for (const auto &binding : recording_topic_catalog()) {
    if (binding.contract->topic == canonical || binding.contract->dds_topic == topic) {
      return &binding;
    }
  }
  return nullptr;
}

const char *replay_policy_name(ReplayPolicy policy) noexcept {
  switch (policy) {
    case ReplayPolicy::Replayable:
      return "replayable";
    case ReplayPolicy::RecordOnly:
      return "record-only";
  }
  return "record-only";
}

std::vector<const TopicBinding *> default_recording_topics() {
  return {
      find_sensor_topic("/imu/raw"),
      find_sensor_topic("/lidar/raw_frame"),
      find_sensor_topic("/slam/odometry"),
      find_sensor_topic("/slam/registered_cloud"),
  };
}

ChannelDefinition channel_definition(const TopicBinding &binding) {
  return {
      std::string(binding.contract->topic),
      std::string(binding.contract->dds_topic),
      std::string(binding.contract->idl_type),
      binding.qos_name,
  };
}

}  // namespace lingtu::recording
