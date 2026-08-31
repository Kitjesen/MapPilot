#include "dds/runtime.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "dds/codec.hpp"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "nav/cpp/platform/runtime.hpp"
#include "status/navigation_state.hpp"

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
  return lingtu::nav::platform::bootTimeNanoseconds();
}

std::string readHostBootId() {
  return lingtu::nav::platform::hostBootId();
}

std::string makeProducerBootId(const std::string &host_boot_id, std::uint64_t start_boottime_ns) {
  return lingtu::nav::platform::producerBootId(host_boot_id, start_boottime_ns);
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

double wireHeaderStampSeconds(const lingtu_dds_Header &header) {
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

bool applyInspectionEvidenceResult(lingtu::nav::inspection::Executor &executor,
                                   const InspectionEvidenceResultSample &result,
                                   double now_s) {
  const auto &status = executor.status();
  const std::string &request_id = result.request_id;
  if (status.state != lingtu::nav::inspection::RunState::kActionPending ||
      status.action_request_id.empty() || request_id != status.action_request_id) {
    return false;
  }
  const std::string &evidence_id = result.evidence_id;
  std::string reason = result.reason;
  if (!result.persisted && reason.empty()) {
    reason = "evidence_not_persisted";
  }
  return executor.OnActionResult(request_id, result.persisted, evidence_id, reason, now_s);
}

RollingSegmentExecutionGrid
copyExplorationExecutionGrid(const lingtu_dds_ExplorationExecutionGrid &message) {
  RollingSegmentExecutionGrid view;
  view.stamp_s = wireHeaderStampSeconds(message.header);
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

RollingSegmentCommand
copyExplorationSegmentRequest(const lingtu_dds_ExplorationSegmentRequest &message) {
  RollingSegmentCommand view;
  view.stamp_s = wireHeaderStampSeconds(message.header);
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

GeofenceCommandView copyGeofenceCommand(const lingtu_dds_GeofenceCommandRequest &message) {
  GeofenceCommandView view;
  view.request_id = copyDdsString(message.request_id);
  view.action = message.action;
  view.name = copyDdsString(message.name);
  if (message.polygon._buffer == nullptr || message.polygon._maximum < message.polygon._length) {
    return view;
  }
  view.polygon.reserve(message.polygon._length);
  for (std::uint32_t index = 0U; index < message.polygon._length; ++index) {
    const auto &point = message.polygon._buffer[index];
    view.polygon.push_back({point.x, point.y});
  }
  return view;
}

Dds::Dds(int domain_id, DdsStatus *status)
    : host_boot_id_(readHostBootId()),
      producer_boot_id_(makeProducerBootId(host_boot_id_, boottimeNanoseconds())),
      status_(status) {
  if (status_ != nullptr) {
    status_->producer_boot_id = producer_boot_id_;
  }
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
  cloud_reader_ = reader(lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
                         &lingtu_dds_PointCloud2_desc, "registered_cloud");
  terrain_map_reader_ = reader(lingtu::message::kNavTerrainMap.dds_topic.data(),
                               &lingtu_dds_PointCloud2_desc, "terrain_map");
  terrain_map_ext_reader_ = reader(lingtu::message::kNavTerrainMapExt.dds_topic.data(),
                                   &lingtu_dds_PointCloud2_desc, "terrain_map_ext");
  traversability_reader_ = reader(lingtu::message::kNavTraversability.dds_topic.data(),
                                  &lingtu_dds_OccupancyGrid_desc, "traversability");
  local_traversability_reader_ = reader(lingtu::message::kNavLocalTraversability.dds_topic.data(),
                                        &lingtu_dds_OccupancyGrid_desc, "local_traversability");
  local_collision_reader_ = reader(lingtu::message::kMapsLocalCollision.dds_topic.data(),
                                   &lingtu_dds_MapCollisionLayer_desc, "local_collision");
  localization_health_reader_ = reader(lingtu::message::kSlamLocalizationHealth.dds_topic.data(),
                                       &lingtu_dds_Text_desc, "localization_health");
  driver_control_state_reader_ =
      reader(lingtu::message::kDriverControlState.dds_topic.data(),
             &lingtu_dds_DriverControlState_desc, "driver_control_state");
  map_clearing_reader_ = reader(lingtu::message::kNavMapClearing.dds_topic.data(),
                                &lingtu_dds_Bool_desc, "map_clearing");
  cloud_clearing_reader_ = reader(lingtu::message::kNavCloudClearing.dds_topic.data(),
                                  &lingtu_dds_Bool_desc, "cloud_clearing");
  command_request_reader_ =
      reader(lingtu::message::kNavCommandRequest.dds_topic.data(),
             &lingtu_dds_NavigationCommandRequest_desc, "nav_command_request");
  plan_request_reader_ = reader(lingtu::message::kNavPlanRequest.dds_topic.data(),
                                &lingtu_dds_PlanRequest_desc, "nav_plan_request");
  geofence_command_reader_ = reader(lingtu::message::kNavGeofenceCommand.dds_topic.data(),
                                    &lingtu_dds_GeofenceCommandRequest_desc, "geofence_command");
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

  inspection_task_request_reader_ =
      reader(lingtu::message::kNavInspectionTaskRequest.dds_topic.data(),
             &lingtu_dds_InspectionTaskRequest_desc, "inspection_task_request");
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
  plan_result_writer_ = writer(lingtu::message::kNavPlanResult.dds_topic.data(),
                               &lingtu_dds_PlanResult_desc, "nav_plan_result");
  geofence_response_writer_ = writer(lingtu::message::kNavGeofenceResponse.dds_topic.data(),
                                     &lingtu_dds_GeofenceCommandAck_desc, "geofence_response");
  geofence_alert_writer_ = writer(lingtu::message::kNavGeofenceAlert.dds_topic.data(),
                                  &lingtu_dds_GeofenceAlert_desc, "geofence_alert");
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

  inspection_task_ack_writer_ = writer(lingtu::message::kNavInspectionTaskAck.dds_topic.data(),
                                       &lingtu_dds_InspectionTaskAck_desc, "inspection_task_ack");
  inspection_status_writer_ = writer(lingtu::message::kNavInspectionStatus.dds_topic.data(),
                                     &lingtu_dds_InspectionStatus_desc, "inspection_status");
  inspection_task_event_writer_ =
      writer(lingtu::message::kNavInspectionTaskEvent.dds_topic.data(),
             &lingtu_dds_InspectionTaskEvent_desc, "inspection_task_event");
  inspection_evidence_request_writer_ =
      writer(lingtu::message::kNavInspectionEvidenceRequest.dds_topic.data(),
             &lingtu_dds_InspectionEvidenceRequest_desc, "inspection_evidence_request");
}

Dds::~Dds() {
  if (participant_ > 0) {
    dds_delete(participant_);
  }
}

SensorBatch Dds::takeSensors(double now_steady_s) {
  SensorBatch batch;
  batch.receive_steady_s = now_steady_s;
  batch.receive_wall_s = nowSeconds();

  drainReader<lingtu_dds_TFMessage>(
      tf_reader_, lingtu_dds_TFMessage_desc,
      [&](const lingtu_dds_TFMessage &message) {
        batch.transforms.push_back(copyTransformSample(message));
      },
      DdsDrainProfile::kTransform);
  drainReader<lingtu_dds_Odometry>(
      odom_reader_, lingtu_dds_Odometry_desc,
      [&](const lingtu_dds_Odometry &message) {
        batch.odometry.push_back(copyOdometrySample(message));
      });
  drainReader<lingtu_dds_DriverControlState>(
      driver_control_state_reader_, lingtu_dds_DriverControlState_desc,
      [&](const lingtu_dds_DriverControlState &message) {
        batch.driver_control = copyDriverControlSample(message);
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_ExplorationExecutionGrid>(
      exploration_execution_grid_reader_, lingtu_dds_ExplorationExecutionGrid_desc,
      [&](const lingtu_dds_ExplorationExecutionGrid &message) {
        batch.exploration_grid = copyExplorationExecutionGrid(message);
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_PointCloud2>(
      cloud_reader_, lingtu_dds_PointCloud2_desc,
      [&](const lingtu_dds_PointCloud2 &message) {
        batch.obstacles = copyPointCloudSample(message, false);
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_PointCloud2>(
      terrain_map_reader_, lingtu_dds_PointCloud2_desc,
      [&](const lingtu_dds_PointCloud2 &message) {
        batch.terrain = copyPointCloudSample(message, true);
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_PointCloud2>(
      terrain_map_ext_reader_, lingtu_dds_PointCloud2_desc,
      [&](const lingtu_dds_PointCloud2 &message) {
        batch.terrain_extended = copyPointCloudSample(message, true);
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_Bool>(
      map_clearing_reader_, lingtu_dds_Bool_desc,
      [&](const lingtu_dds_Bool &message) {
        batch.clears.push_back({ClearSource::Map, message.data});
      });
  drainReader<lingtu_dds_Bool>(
      cloud_clearing_reader_, lingtu_dds_Bool_desc,
      [&](const lingtu_dds_Bool &message) {
        batch.clears.push_back({ClearSource::Cloud, message.data});
      });
  drainReader<lingtu_dds_OccupancyGrid>(
      traversability_reader_, lingtu_dds_OccupancyGrid_desc,
      [&](const lingtu_dds_OccupancyGrid &message) {
        batch.traversability = copyGridSample(message, "map");
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_OccupancyGrid>(
      local_traversability_reader_, lingtu_dds_OccupancyGrid_desc,
      [&](const lingtu_dds_OccupancyGrid &message) {
        batch.local_traversability = copyGridSample(message, "odom");
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_MapCollisionLayer>(
      local_collision_reader_, lingtu_dds_MapCollisionLayer_desc,
      [&](const lingtu_dds_MapCollisionLayer &message) {
        batch.local_collision = copyLocalCollisionSample(message);
      },
      DdsDrainProfile::kDefault, true);
  drainReader<lingtu_dds_Text>(
      localization_health_reader_, lingtu_dds_Text_desc,
      [&](const lingtu_dds_Text &message) {
        batch.localization_health = textData(message);
      },
      DdsDrainProfile::kDefault, true);
  return batch;
}

CommandBatch Dds::takeCommands(double now_steady_s) {
  CommandBatch batch;
  batch.receive_steady_s = now_steady_s;
  batch.receive_wall_s = nowSeconds();

  drainReader<lingtu_dds_OperatorMotionControl>(
      operator_motion_control_reader_, lingtu_dds_OperatorMotionControl_desc,
      [&](const lingtu_dds_OperatorMotionControl &message) {
        batch.ordered.emplace_back(OperatorMotionControlSample{
            stringValue(message.source_id), message.source_epoch, message.source_sequence,
            stringValue(message.request_id), message.action, message.lease_ttl_ms});
      });
  drainReader<lingtu_dds_OperatorMotionSample>(
      operator_motion_sample_reader_, lingtu_dds_OperatorMotionSample_desc,
      [&](const lingtu_dds_OperatorMotionSample &message) {
        batch.ordered.emplace_back(OperatorMotionInputSample{
            headerStampSeconds(message.header), headerFrameId(message.header),
            stringValue(message.source_id), message.source_epoch, message.source_sequence,
            message.source_stamp_ns, message.freshness_budget_ms, message.deadman,
            message.manual_mode,
            {message.velocity.linear.x, message.velocity.linear.y,
             message.velocity.angular.z}});
      });
  drainReader<lingtu_dds_NavigationCommandRequest>(
      command_request_reader_, lingtu_dds_NavigationCommandRequest_desc,
      [&](const lingtu_dds_NavigationCommandRequest &message) {
        NavigationCommandSample sample;
        sample.ingress = commandIngressRequestFromDds(message);
        sample.goal.stamp_s = headerStampSeconds(message.header);
        sample.goal.frame_id = headerFrameId(message.header);
        sample.goal.position = {message.goal.position.x, message.goal.position.y,
                                message.goal.position.z};
        sample.goal.orientation = quaternionFromDds(message.goal.orientation);
        const auto &q = sample.goal.orientation;
        sample.goal.has_orientation =
            q.x != 0.0 || q.y != 0.0 || q.z != 0.0 || q.w != 0.0;
        batch.ordered.emplace_back(std::move(sample));
      });
  drainReader<lingtu_dds_PlanRequest>(
      plan_request_reader_, lingtu_dds_PlanRequest_desc,
      [&](const lingtu_dds_PlanRequest &message) {
        batch.ordered.emplace_back(PlanPreviewRequest{
            headerStampSeconds(message.header), headerFrameId(message.header),
            stringValue(message.request_id),
            {message.goal.x, message.goal.y, message.goal.z}});
      });
  drainReader<lingtu_dds_InspectionTaskRequest>(
      inspection_task_request_reader_, lingtu_dds_InspectionTaskRequest_desc,
      [&](const lingtu_dds_InspectionTaskRequest &message) {
        batch.ordered.emplace_back(InspectionCommandRequest{
            stringValue(message.task_id), stringValue(message.request_id), message.kind,
            stringValue(message.route_id), message.route_revision, stringValue(message.reason)});
      });
  drainReader<lingtu_dds_GeofenceCommandRequest>(
      geofence_command_reader_, lingtu_dds_GeofenceCommandRequest_desc,
      [&](const lingtu_dds_GeofenceCommandRequest &message) {
        batch.ordered.emplace_back(copyGeofenceCommand(message));
      });
  drainReader<lingtu_dds_ExplorationSegmentRequest>(
      exploration_segment_request_reader_, lingtu_dds_ExplorationSegmentRequest_desc,
      [&](const lingtu_dds_ExplorationSegmentRequest &message) {
        batch.ordered.emplace_back(copyExplorationSegmentRequest(message));
      });
  drainReader<lingtu_dds_InspectionEvidenceResult>(
      inspection_evidence_result_reader_, lingtu_dds_InspectionEvidenceResult_desc,
      [&](const lingtu_dds_InspectionEvidenceResult &message) {
        batch.ordered.emplace_back(InspectionEvidenceResultSample{
            stringValue(message.request_id), message.persisted,
            stringValue(message.evidence_id), stringValue(message.reason)});
      });
  if (status_ != nullptr) {
    status_->inspection_evidence_worker_matched = inspectionEvidenceWorkerMatched();
  }
  return batch;
}

PublishReceipt Dds::publish(const OutputEvent &output) {
  PublishReceipt receipt;
  std::visit(
      [&](const auto &event) {
        using Event = std::decay_t<decltype(event)>;
        if constexpr (std::is_same_v<Event, LocalPathOutput>) {
          receipt.published = writeLocalPath(event.path);
        } else if constexpr (std::is_same_v<Event, GlobalPathOutput>) {
          receipt.published = writeGlobalPath(event.path, event.stamp_s);
        } else if constexpr (std::is_same_v<Event, WaypointOutput>) {
          receipt.published = writeWayPoint(event.point);
        } else if constexpr (std::is_same_v<Event, FinalVelocityOutput>) {
          receipt.final_velocity = writeCmdVelSequenced(event.command);
          receipt.published = receipt.final_velocity.has_value();
        } else if constexpr (std::is_same_v<Event, OperatorMotionAckSample>) {
          receipt.published = writeOperatorMotionAck(event);
        } else if constexpr (std::is_same_v<Event, OperatorMotionStatusSample>) {
          receipt.published = writeOperatorMotionStatus(event);
        } else if constexpr (std::is_same_v<Event, CommandAckOutput>) {
          receipt.published = writeCommandAck(event.task_id.c_str(), event.request_id.c_str(),
                                              event.kind, event.accepted, event.reason.c_str());
        } else if constexpr (std::is_same_v<Event, PlanResultSample>) {
          receipt.published = writePlanResult(event);
        } else if constexpr (std::is_same_v<Event, GeofenceCommandAckSample>) {
          receipt.published = writeGeofenceAck(event);
        } else if constexpr (std::is_same_v<Event, GeofenceAlertSample>) {
          receipt.published = writeGeofenceAlert(event);
        } else if constexpr (std::is_same_v<Event, NavigationGoalStatusOutput>) {
          receipt.published = writeNavigationGoalStatus(
              event.task_id.c_str(), event.request_id.c_str(), event.state, event.goal_epoch,
              event.reason.c_str());
        } else if constexpr (std::is_same_v<Event, NavigationStateSample>) {
          receipt.published = writeNavigationState(event);
        } else if constexpr (std::is_same_v<Event, ExplorationSegmentAck>) {
          receipt.published = writeExplorationSegmentAck(event);
        } else if constexpr (std::is_same_v<Event, ExplorationSegmentStatus>) {
          receipt.published = writeExplorationSegmentStatus(event);
        } else if constexpr (std::is_same_v<Event, InspectionTaskAckOutput>) {
          receipt.published = writeInspectionTaskAck(
              event.task_id.c_str(), event.request_id.c_str(), event.kind, event.accepted,
              event.reason.c_str(), event.run_id.c_str());
        } else if constexpr (std::is_same_v<Event, InspectionStatusOutput>) {
          receipt.published = writeInspectionStatus(event.status);
        } else if constexpr (std::is_same_v<Event, InspectionTaskEventEnvelope>) {
          receipt.published = writeInspectionTaskEvent(event);
        } else if constexpr (std::is_same_v<Event, InspectionEvidenceRequestOutput>) {
          receipt.published = writeInspectionEvidenceRequest(
              event.request, event.map_id, event.map_content_epoch, event.deadline_s);
        }
      },
      output);
  return receipt;
}

bool Dds::writeLocalPath(const std::vector<nav_kernel::Vec3> &path) {
  PathMessage msg = toDdsPath(path, "map");
  const dds_return_t result = dds_write(local_path_writer_, &msg.msg);
  logDdsError(result, "dds_write(local_path)");
  return result >= 0;
}

bool Dds::writeGlobalPath(const std::vector<nav_kernel::Vec3> &path, double stamp_s) {
  PathMessage msg = toDdsPath(path, "map", stamp_s);
  const dds_return_t result = dds_write(global_path_writer_, &msg.msg);
  logDdsError(result, "dds_write(global_path)");
  return result >= 0;
}

bool Dds::writeWayPoint(const nav_kernel::Vec3 &point) {
  lingtu_dds_PoseStamped msg = toDdsPoseStamped(point, "map");
  const dds_return_t result = dds_write(way_point_writer_, &msg);
  logDdsError(result, "dds_write(way_point)");
  return result >= 0;
}

std::optional<FinalVelocityCommandReceipt>
Dds::writeCmdVelSequenced(const nav_kernel::Twist &cmd) {
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
  if (status_ != nullptr) {
    status_->producer_boot_id = producer_boot_id_;
    status_->final_output_sequence = output_seq_;
    status_->final_output_command = last_output_command_;
  }
  return FinalVelocityCommandReceipt{output_sequence, msg.source_wall_ns};
}

bool Dds::writeCmdVel(const nav_kernel::Twist &cmd) {
  return writeCmdVelSequenced(cmd).has_value();
}

bool Dds::writeOperatorMotionAck(const OperatorMotionAckSample &ack) {
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

bool Dds::writeOperatorMotionStatus(const OperatorMotionStatusSample &status) {
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

bool Dds::writeCommandAck(const char *task_id, const char *request_id,
                                 lingtu::message::NavigationCommandKind kind, bool accepted,
                                 const char *reason) {
  lingtu_dds_NavigationCommandAck msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.task_id = const_cast<char *>(task_id == nullptr ? "" : task_id);
  msg.request_id = const_cast<char *>(request_id == nullptr ? "" : request_id);
  msg.kind = static_cast<std::int32_t>(kind);
  msg.accepted = accepted;
  msg.reason = const_cast<char *>(reason == nullptr ? "" : reason);
  const dds_return_t result = dds_write(command_ack_writer_, &msg);
  logDdsError(result, "dds_write(nav_command_ack)");
  return result >= 0;
}

bool Dds::writePlanResult(const PlanResultSample &result) {
  if (plan_result_writer_ <= 0 || result.request_id.empty()) {
    return false;
  }
  std::vector<lingtu_dds_Point> path(result.path.size());
  for (std::size_t index = 0U; index < result.path.size(); ++index) {
    path[index].x = result.path[index].x;
    path[index].y = result.path[index].y;
    path[index].z = result.path[index].z;
  }
  lingtu_dds_PlanResult message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.request_id = const_cast<char *>(result.request_id.c_str());
  message.feasible = result.feasible;
  message.start_valid = result.start_valid;
  message.reason = const_cast<char *>(result.reason.c_str());
  message.elapsed_ms = result.elapsed_ms;
  message.planner = const_cast<char *>(result.planner.c_str());
  message.start.x = result.start.x;
  message.start.y = result.start.y;
  message.start.z = result.start.z;
  message.goal.x = result.goal.x;
  message.goal.y = result.goal.y;
  message.goal.z = result.goal.z;
  message.path._maximum = static_cast<std::uint32_t>(path.size());
  message.path._length = static_cast<std::uint32_t>(path.size());
  message.path._buffer = path.data();
  message.path._release = false;
  const dds_return_t write_result = dds_write(plan_result_writer_, &message);
  logDdsError(write_result, "dds_write(nav_plan_result)");
  return write_result >= 0;
}

bool Dds::writeGeofenceAck(const GeofenceCommandAckSample &ack) {
  if (geofence_response_writer_ <= 0) {
    return false;
  }
  std::vector<lingtu_dds_GeofenceZoneSummary> summaries(ack.zones.size());
  for (std::size_t index = 0U; index < ack.zones.size(); ++index) {
    summaries[index].name = const_cast<char *>(ack.zones[index].name.c_str());
    summaries[index].enabled = ack.zones[index].enabled;
    summaries[index].vertex_count = ack.zones[index].vertex_count;
  }
  lingtu_dds_GeofenceCommandAck message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.request_id = const_cast<char *>(ack.request_id.c_str());
  message.action = ack.action;
  message.accepted = ack.accepted;
  message.reason = const_cast<char *>(ack.reason.c_str());
  message.revision = ack.revision;
  message.zones._maximum = static_cast<std::uint32_t>(summaries.size());
  message.zones._length = static_cast<std::uint32_t>(summaries.size());
  message.zones._buffer = summaries.data();
  message.zones._release = false;
  const dds_return_t result = dds_write(geofence_response_writer_, &message);
  logDdsError(result, "dds_write(geofence_response)");
  return result >= 0;
}

bool Dds::writeGeofenceAlert(const GeofenceAlertSample &alert) {
  if (geofence_alert_writer_ <= 0) {
    return false;
  }
  lingtu_dds_GeofenceAlert message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.active = alert.active;
  message.name = const_cast<char *>(alert.name.c_str());
  message.robot_position.x = alert.robot_x;
  message.robot_position.y = alert.robot_y;
  message.robot_position.z = alert.robot_z;
  message.revision = alert.revision;
  message.reason = const_cast<char *>(alert.reason.c_str());
  const dds_return_t result = dds_write(geofence_alert_writer_, &message);
  logDdsError(result, "dds_write(geofence_alert)");
  return result >= 0;
}

bool Dds::writeNavigationGoalStatus(const char *task_id, const char *request_id,
                                           lingtu::message::NavigationGoalState state,
                                           std::uint64_t goal_epoch, const char *reason) {
  if (goal_status_writer_ <= 0 || task_id == nullptr || *task_id == '\0' || request_id == nullptr ||
      *request_id == '\0' || goal_status_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  lingtu_dds_NavigationGoalStatus msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.boot_id = const_cast<char *>(producer_boot_id_.c_str());
  msg.event_sequence = goal_status_seq_ + 1U;
  msg.task_id = const_cast<char *>(task_id);
  msg.request_id = const_cast<char *>(request_id);
  msg.state = static_cast<std::int32_t>(state);
  msg.goal_epoch = goal_epoch;
  msg.reason = const_cast<char *>(reason == nullptr ? "" : reason);
  const dds_return_t result = dds_write(goal_status_writer_, &msg);
  logDdsError(result, "dds_write(nav_goal_status)");
  if (result >= 0) {
    goal_status_seq_ = msg.event_sequence;
    return true;
  }
  return false;
}

bool Dds::writeNavigationState(const NavigationStateSample &state) {
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
  message.active_task_id = const_cast<char *>(state.active_task_id.c_str());
  message.active_request_id = const_cast<char *>(state.active_request_id.c_str());
  message.goal_epoch = state.goal_epoch;
  message.map_id = const_cast<char *>(state.map_id.c_str());
  message.map_content_epoch = state.map_content_epoch;
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

bool Dds::writeExplorationSegmentAck(const ExplorationSegmentAck &ack) {
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

bool Dds::writeExplorationSegmentStatus(const ExplorationSegmentStatus &status) {
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

bool Dds::writeInspectionTaskAck(const char *task_id, const char *request_id,
                                        lingtu::nav::inspection::CommandKind kind, bool accepted,
                                        const char *reason, const char *run_id) {
  lingtu_dds_InspectionTaskAck msg{};
  fillHeader(msg.header, nowSeconds(), "map");
  msg.task_id = const_cast<char *>(task_id == nullptr ? "" : task_id);
  msg.request_id = const_cast<char *>(request_id == nullptr ? "" : request_id);
  msg.kind = static_cast<std::int32_t>(kind);
  msg.accepted = accepted;
  msg.reason = const_cast<char *>(reason == nullptr ? "" : reason);
  msg.run_id = const_cast<char *>(run_id == nullptr ? "" : run_id);
  const dds_return_t result = dds_write(inspection_task_ack_writer_, &msg);
  logDdsError(result, "dds_write(inspection_task_ack)");
  return result >= 0;
}

bool Dds::writeInspectionStatus(const lingtu::nav::inspection::RunStatus &status) {
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
  const dds_return_t result = dds_write(inspection_status_writer_, &msg);
  logDdsError(result, "dds_write(inspection_status)");
  return result >= 0;
}

bool Dds::writeInspectionTaskEvent(const InspectionTaskEventEnvelope &envelope) {
  if (inspection_task_event_writer_ <= 0 || envelope.boot_id.empty() || envelope.sequence == 0U) {
    return false;
  }
  const auto &event = envelope.event;
  const auto &status = event.status;
  lingtu_dds_InspectionTaskEvent msg{};
  fillHeader(msg.header, event.timestamp_s, "map");
  msg.boot_id = const_cast<char *>(envelope.boot_id.c_str());
  msg.event_sequence = envelope.sequence;
  msg.kind = static_cast<std::int32_t>(event.kind);
  msg.task_id = const_cast<char *>(status.task_id.c_str());
  msg.request_id = const_cast<char *>(event.request_id.c_str());
  msg.command_request_id = const_cast<char *>(status.request_id.c_str());
  msg.state = static_cast<std::int32_t>(status.state);
  msg.map_id = const_cast<char *>(status.map_id.c_str());
  msg.map_content_epoch = status.map_content_epoch;
  msg.route_id = const_cast<char *>(status.route_id.c_str());
  msg.route_revision = status.route_revision;
  msg.point_index = static_cast<std::uint32_t>(status.point_index);
  msg.point_count = static_cast<std::uint32_t>(status.point_count);
  msg.loop_index = status.loop_index;
  msg.retry_count = status.retry_count;
  msg.point_id = const_cast<char *>(status.point_id.c_str());
  msg.action = const_cast<char *>(status.action.c_str());
  msg.action_request_id = const_cast<char *>(status.action_request_id.c_str());
  msg.evidence_id = const_cast<char *>(status.evidence_id.c_str());
  msg.reason = const_cast<char *>(status.reason.c_str());
  const dds_return_t result = dds_write(inspection_task_event_writer_, &msg);
  logDdsError(result, "dds_write(inspection_task_event)");
  return result >= 0;
}

bool Dds::inspectionEvidenceWorkerMatched() const noexcept {
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

bool Dds::writeInspectionEvidenceRequest(
    const lingtu::nav::inspection::ActionRequest &request, const std::string &map_id,
    std::int64_t map_content_epoch, double deadline_s) {
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
  msg.map_content_epoch = map_content_epoch;
  msg.point_index = static_cast<std::uint32_t>(request.point_index);
  msg.point_id = const_cast<char *>(request.point_id.c_str());
  msg.action = const_cast<char *>(request.action.c_str());
  msg.deadline_s = deadline_s;
  const dds_return_t result = dds_write(inspection_evidence_request_writer_, &msg);
  logDdsError(result, "dds_write(inspection_evidence_request)");
  return result >= 0;
}

void Dds::logDdsError(dds_return_t value, const char *what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

dds_entity_t Dds::reader(const char *topic_name, const dds_topic_descriptor_t *desc,
                                const char *label) {
  const dds_entity_t topic =
      checked(dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
              (std::string("dds_create_topic(") + label + ")").c_str());
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
  const dds_entity_t entity = checked(dds_create_reader(subscriber_, topic, qos.get(), nullptr),
                                      (std::string("dds_create_reader(") + label + ")").c_str());
  return entity;
}

dds_entity_t Dds::writer(const char *topic_name, const dds_topic_descriptor_t *desc,
                                const char *label) {
  const dds_entity_t topic =
      checked(dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
              (std::string("dds_create_topic(") + label + ")").c_str());
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
  return checked(dds_create_writer(publisher_, topic, qos.get(), nullptr),
                 (std::string("dds_create_writer(") + label + ")").c_str());
}

}  // namespace lingtu::nav::endpoint
