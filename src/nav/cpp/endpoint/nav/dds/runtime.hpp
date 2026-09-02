#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "dds/dds.h"
#include "command/batch.hpp"
#include "dds/drain.hpp"
#include "input/samples.hpp"
#include "messages.h"
#include "message/cpp/navigation_command.hpp"
#include "message/cpp/operator_motion.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav_kernel/types.hpp"
#include "safety/geofence.hpp"
#include "status/navigation_state.hpp"

namespace lingtu::nav::endpoint {

struct NavigationStateSample;

struct PlanResultSample {
  std::string request_id;
  bool feasible{false};
  bool start_valid{false};
  std::string reason;
  double elapsed_ms{0.0};
  std::string planner;
  nav_kernel::Vec3 start{};
  nav_kernel::Vec3 goal{};
  std::vector<nav_kernel::Vec3> path;
};

// DDS-facing envelope. Reliable ordering and retry ownership live in the
// inspection core; this type only adapts one immutable fact to the wire.
struct InspectionTaskEventEnvelope {
  std::string boot_id;
  std::uint64_t sequence{0U};
  lingtu::nav::inspection::TaskEvent event;
};

bool applyInspectionEvidenceResult(lingtu::nav::inspection::Executor &executor,
                                   const InspectionEvidenceResultSample &result, double now_s);

struct ExplorationSegmentAck {
  double stamp_s{0.0};
  std::string frame_id{"map"};
  std::string request_id;
  std::int32_t kind{0};
  bool accepted{false};
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};
  std::string reason;
};

struct ExplorationSegmentStatus {
  double stamp_s{0.0};
  std::string frame_id{"map"};
  std::string request_id;
  std::int32_t state{0};
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};
  std::string reason;
};

struct OperatorMotionAckSample {
  std::string source_id;
  std::uint64_t source_epoch{0U};
  std::uint64_t sequence{0U};
  std::string request_id;
  std::int32_t action{0};
  bool accepted{false};
  std::string reason;
  std::uint64_t accepted_sequence{0U};
  std::uint64_t final_output_sequence{0U};
};

struct OperatorMotionStatusSample {
  std::string active_source_id;
  std::uint64_t active_source_epoch{0U};
  bool has_active_authority{false};
  bool holding{false};
  bool has_active_sample{false};
  std::uint64_t last_sample_sequence{0U};
  std::uint64_t admitted_sequence{0U};
  std::uint64_t final_output_sequence{0U};
  std::string authority_reason;
  std::string input_gate_reason;
  nav_kernel::Twist teleop_output{};
  nav_kernel::Twist final_cmd_vel{};
};

struct FinalVelocityCommandReceipt {
  std::uint64_t output_sequence{0U};
  std::uint64_t source_wall_ns{0U};
};

struct GeofenceCommandAckSample {
  std::string request_id;
  std::int32_t action{0};
  bool accepted{false};
  std::string reason;
  std::uint64_t revision{0U};
  std::vector<GeofenceSummary> zones;
};

struct GeofenceAlertSample {
  bool active{false};
  std::string name;
  double robot_x{0.0};
  double robot_y{0.0};
  double robot_z{0.0};
  std::uint64_t revision{0U};
  std::string reason;
};

struct LocalPathOutput {
  std::vector<nav_kernel::Vec3> path;
};

struct GlobalPathOutput {
  std::vector<nav_kernel::Vec3> path;
  double stamp_s{0.0};
};

struct WaypointOutput {
  nav_kernel::Vec3 point{};
};

struct FinalVelocityOutput {
  nav_kernel::Twist command{};
};

struct CommandAckOutput {
  std::string task_id;
  std::string request_id;
  lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};
  bool accepted{false};
  std::string reason;
};

struct NavigationGoalStatusOutput {
  std::string task_id;
  std::string request_id;
  lingtu::message::NavigationGoalState state{lingtu::message::NavigationGoalState::Planning};
  std::uint64_t goal_epoch{0U};
  std::string reason;
};

struct InspectionTaskAckOutput {
  std::string task_id;
  std::string request_id;
  lingtu::nav::inspection::CommandKind kind{lingtu::nav::inspection::CommandKind::kStart};
  bool accepted{false};
  std::string reason;
  std::string run_id;
};

struct InspectionStatusOutput {
  lingtu::nav::inspection::RunStatus status;
};

struct InspectionEvidenceRequestOutput {
  lingtu::nav::inspection::ActionRequest request;
  std::string map_id;
  std::int64_t map_content_epoch{0};
  double deadline_s{0.0};
};

using OutputEvent =
    std::variant<LocalPathOutput, GlobalPathOutput, WaypointOutput, FinalVelocityOutput,
                 OperatorMotionAckSample, OperatorMotionStatusSample, CommandAckOutput,
                 PlanResultSample, GeofenceCommandAckSample, GeofenceAlertSample,
                 NavigationGoalStatusOutput, NavigationStateSample, ExplorationSegmentAck,
                 ExplorationSegmentStatus, InspectionTaskAckOutput, InspectionStatusOutput,
                 InspectionTaskEventEnvelope, InspectionEvidenceRequestOutput>;

struct PublishReceipt {
  bool published{false};
  std::optional<FinalVelocityCommandReceipt> final_velocity;
};

// Transport evidence owned by navd rather than queried through topic-specific
// DDS methods. Dds updates this snapshot when commands are taken or published.
struct DdsStatus {
  std::string producer_boot_id;
  std::uint64_t final_output_sequence{0U};
  nav_kernel::Twist final_output_command{};
  bool inspection_evidence_worker_matched{false};
};

[[nodiscard]] RollingSegmentExecutionGrid
copyExplorationExecutionGrid(const lingtu_dds_ExplorationExecutionGrid &message);
[[nodiscard]] RollingSegmentCommand
copyExplorationSegmentRequest(const lingtu_dds_ExplorationSegmentRequest &message);
[[nodiscard]] GeofenceCommandView
copyGeofenceCommand(const lingtu_dds_GeofenceCommandRequest &message);

class Dds {
 public:
  explicit Dds(int domain_id, DdsStatus *status = nullptr,
               bool read_obstacle_cloud = true);
  ~Dds();

  Dds(const Dds &) = delete;
  Dds &operator=(const Dds &) = delete;

  [[nodiscard]] SensorBatch takeSensors(double now_steady_s);
  [[nodiscard]] CommandBatch takeCommands(double now_steady_s);
  [[nodiscard]] PublishReceipt publish(const OutputEvent &output);

 private:
  template <typename Handler>
  void drainOdometry(Handler &&handler) {
    drainReader<lingtu_dds_Odometry>(odom_reader_, lingtu_dds_Odometry_desc,
                                     std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTf(Handler &&handler) {
    drainReader<lingtu_dds_TFMessage>(tf_reader_, lingtu_dds_TFMessage_desc,
                                      std::forward<Handler>(handler), DdsDrainProfile::kTransform);
  }

  template <typename Handler>
  void drainCloud(Handler &&handler) {
    drainReader<lingtu_dds_PointCloud2>(cloud_reader_, lingtu_dds_PointCloud2_desc,
                                        std::forward<Handler>(handler), DdsDrainProfile::kDefault,
                                        true);
  }

  template <typename Handler>
  void drainTerrainMap(Handler &&handler) {
    drainReader<lingtu_dds_PointCloud2>(terrain_map_reader_, lingtu_dds_PointCloud2_desc,
                                        std::forward<Handler>(handler), DdsDrainProfile::kDefault,
                                        true);
  }

  template <typename Handler>
  void drainTerrainMapExt(Handler &&handler) {
    drainReader<lingtu_dds_PointCloud2>(terrain_map_ext_reader_, lingtu_dds_PointCloud2_desc,
                                        std::forward<Handler>(handler), DdsDrainProfile::kDefault,
                                        true);
  }

  template <typename Handler>
  void drainTraversability(Handler &&handler) {
    drainReader<lingtu_dds_OccupancyGrid>(traversability_reader_, lingtu_dds_OccupancyGrid_desc,
                                          std::forward<Handler>(handler), DdsDrainProfile::kDefault,
                                          true);
  }

  template <typename Handler>
  void drainLocalTraversability(Handler &&handler) {
    drainReader<lingtu_dds_OccupancyGrid>(
        local_traversability_reader_, lingtu_dds_OccupancyGrid_desc, std::forward<Handler>(handler),
        DdsDrainProfile::kDefault, true);
  }

  template <typename Handler>
  void drainLocalCollision(Handler &&handler) {
    drainReader<lingtu_dds_MapCollisionLayer>(
        local_collision_reader_, lingtu_dds_MapCollisionLayer_desc, std::forward<Handler>(handler),
        DdsDrainProfile::kDefault, true);
  }

  template <typename Handler>
  void drainLocalizationHealth(Handler &&handler) {
    drainReader<lingtu_dds_Text>(localization_health_reader_, lingtu_dds_Text_desc,
                                 std::forward<Handler>(handler), DdsDrainProfile::kDefault, true);
  }

  template <typename Handler>
  void drainDriverControlState(Handler &&handler) {
    drainReader<lingtu_dds_DriverControlState>(
        driver_control_state_reader_, lingtu_dds_DriverControlState_desc,
        std::forward<Handler>(handler), DdsDrainProfile::kDefault, true);
  }

  template <typename Handler>
  void drainMapClearing(Handler &&handler) {
    drainReader<lingtu_dds_Bool>(map_clearing_reader_, lingtu_dds_Bool_desc,
                                 std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCloudClearing(Handler &&handler) {
    drainReader<lingtu_dds_Bool>(cloud_clearing_reader_, lingtu_dds_Bool_desc,
                                 std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCommandRequests(Handler &&handler) {
    drainReader<lingtu_dds_NavigationCommandRequest>(command_request_reader_,
                                                     lingtu_dds_NavigationCommandRequest_desc,
                                                     std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainPlanRequests(Handler &&handler) {
    drainReader<lingtu_dds_PlanRequest>(plan_request_reader_, lingtu_dds_PlanRequest_desc,
                                        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainGeofenceCommands(Handler &&handler) {
    drainReader<lingtu_dds_GeofenceCommandRequest>(
        geofence_command_reader_, lingtu_dds_GeofenceCommandRequest_desc,
        [&handler](const lingtu_dds_GeofenceCommandRequest &message) {
          handler(copyGeofenceCommand(message));
        });
  }

  template <typename Handler>
  void drainOperatorMotionControls(Handler &&handler) {
    drainReader<lingtu_dds_OperatorMotionControl>(operator_motion_control_reader_,
                                                  lingtu_dds_OperatorMotionControl_desc,
                                                  std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainOperatorMotionSamples(Handler &&handler) {
    drainReader<lingtu_dds_OperatorMotionSample>(operator_motion_sample_reader_,
                                                 lingtu_dds_OperatorMotionSample_desc,
                                                 std::forward<Handler>(handler));
  }

  // Convert inside the DDS drain so no generated sample or sequence buffer can
  // outlive the callback that receives this owning value.
  template <typename Handler>
  void drainExplorationExecutionGrids(Handler &&handler) {
    drainReader<lingtu_dds_ExplorationExecutionGrid>(
        exploration_execution_grid_reader_, lingtu_dds_ExplorationExecutionGrid_desc,
        [&handler](const lingtu_dds_ExplorationExecutionGrid &message) {
          handler(copyExplorationExecutionGrid(message));
        },
        DdsDrainProfile::kDefault, true);
  }

  template <typename Handler>
  void drainExplorationSegmentRequests(Handler &&handler) {
    drainReader<lingtu_dds_ExplorationSegmentRequest>(
        exploration_segment_request_reader_, lingtu_dds_ExplorationSegmentRequest_desc,
        [&handler](const lingtu_dds_ExplorationSegmentRequest &message) {
          handler(copyExplorationSegmentRequest(message));
        });
  }

  template <typename Handler>
  void drainInspectionTaskRequests(Handler &&handler) {
    drainReader<lingtu_dds_InspectionTaskRequest>(inspection_task_request_reader_,
                                                  lingtu_dds_InspectionTaskRequest_desc,
                                                  std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainInspectionEvidenceResults(Handler &&handler) {
    drainReader<lingtu_dds_InspectionEvidenceResult>(inspection_evidence_result_reader_,
                                                     lingtu_dds_InspectionEvidenceResult_desc,
                                                     std::forward<Handler>(handler));
  }

  bool writeLocalPath(const std::vector<nav_kernel::Vec3> &path);
  bool writeGlobalPath(const std::vector<nav_kernel::Vec3> &path, double stamp_s);
  bool writeWayPoint(const nav_kernel::Vec3 &point);
  std::optional<FinalVelocityCommandReceipt> writeCmdVelSequenced(const nav_kernel::Twist &cmd);
  bool writeCmdVel(const nav_kernel::Twist &cmd);
  bool writeOperatorMotionAck(const OperatorMotionAckSample &ack);
  bool writeOperatorMotionStatus(const OperatorMotionStatusSample &status);
  bool writeCommandAck(const char *task_id, const char *request_id,
                       lingtu::message::NavigationCommandKind kind, bool accepted,
                       const char *reason);
  bool writePlanResult(const PlanResultSample &result);
  bool writeGeofenceAck(const GeofenceCommandAckSample &ack);
  bool writeGeofenceAlert(const GeofenceAlertSample &alert);
  bool writeNavigationGoalStatus(const char *task_id, const char *request_id,
                                 lingtu::message::NavigationGoalState state,
                                 std::uint64_t goal_epoch, const char *reason);
  bool writeNavigationState(const NavigationStateSample &state);
  bool writeExplorationSegmentAck(const ExplorationSegmentAck &ack);
  bool writeExplorationSegmentStatus(const ExplorationSegmentStatus &status);

  [[nodiscard]] bool writeInspectionTaskAck(const char *task_id, const char *request_id,
                                            lingtu::nav::inspection::CommandKind kind,
                                            bool accepted, const char *reason, const char *run_id);
  bool writeInspectionStatus(const lingtu::nav::inspection::RunStatus &status);
  [[nodiscard]] bool writeInspectionTaskEvent(const InspectionTaskEventEnvelope &event);
  bool inspectionEvidenceWorkerMatched() const noexcept;
  bool writeInspectionEvidenceRequest(const lingtu::nav::inspection::ActionRequest &request,
                                      const std::string &map_id, std::int64_t map_content_epoch,
                                      double deadline_s);

  template <typename T, typename Handler>
  void drainReader(dds_entity_t reader, const dds_topic_descriptor_t &descriptor, Handler &&handler,
                   DdsDrainProfile profile = DdsDrainProfile::kDefault, bool latest_only = false) {
    if (reader <= 0) {
      return;
    }

    const DdsDrainBudget budget = drainBudget(profile);
    T storage[kDdsReaderBatchSize]{};
    void *samples[kDdsReaderBatchSize];
    dds_sample_info_t infos[kDdsReaderBatchSize]{};
    for (std::size_t index = 0U; index < kDdsReaderBatchSize; ++index) {
      samples[index] = &storage[index];
    }
    drainBatches(budget, [&](std::size_t capacity) -> std::ptrdiff_t {
      const auto dds_capacity = static_cast<std::uint32_t>(capacity);
      const dds_return_t count = dds_take(reader, samples, infos, dds_capacity, dds_capacity);
      if (count >= 0) {
        if (latest_only) {
          for (dds_return_t i = count; i > 0; --i) {
            if (infos[i - 1].valid_data) {
              handler(*static_cast<T *>(samples[i - 1]));
              break;
            }
          }
        } else {
          for (dds_return_t i = 0; i < count; ++i) {
            if (infos[i].valid_data) {
              handler(*static_cast<T *>(samples[i]));
            }
          }
        }
      } else {
        logDdsError(count, "dds_take");
      }
      for (std::size_t index = 0U; index < capacity; ++index) {
        dds_sample_free(samples[index], &descriptor, DDS_FREE_CONTENTS);
        std::memset(&storage[index], 0, sizeof(T));
        samples[index] = &storage[index];
        infos[index] = {};
      }
      return static_cast<std::ptrdiff_t>(count);
    });
  }

  static void logDdsError(dds_return_t value, const char *what);

  dds_entity_t reader(const char *topic_name, const dds_topic_descriptor_t *desc,
                      const char *label);
  dds_entity_t writer(const char *topic_name, const dds_topic_descriptor_t *desc,
                      const char *label);

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t odom_reader_{0};
  dds_entity_t tf_reader_{0};
  dds_entity_t cloud_reader_{0};
  dds_entity_t terrain_map_reader_{0};
  dds_entity_t terrain_map_ext_reader_{0};
  dds_entity_t traversability_reader_{0};
  dds_entity_t local_traversability_reader_{0};
  dds_entity_t local_collision_reader_{0};
  dds_entity_t localization_health_reader_{0};
  dds_entity_t driver_control_state_reader_{0};
  dds_entity_t map_clearing_reader_{0};
  dds_entity_t cloud_clearing_reader_{0};
  dds_entity_t command_request_reader_{0};
  dds_entity_t plan_request_reader_{0};
  dds_entity_t geofence_command_reader_{0};
  dds_entity_t operator_motion_control_reader_{0};
  dds_entity_t operator_motion_sample_reader_{0};
  dds_entity_t exploration_execution_grid_reader_{0};
  dds_entity_t exploration_segment_request_reader_{0};

  dds_entity_t inspection_task_request_reader_{0};
  dds_entity_t inspection_evidence_result_reader_{0};
  dds_entity_t global_path_writer_{0};
  dds_entity_t local_path_writer_{0};
  dds_entity_t way_point_writer_{0};
  dds_entity_t cmd_vel_writer_{0};
  dds_entity_t command_ack_writer_{0};
  dds_entity_t plan_result_writer_{0};
  dds_entity_t geofence_response_writer_{0};
  dds_entity_t geofence_alert_writer_{0};
  dds_entity_t goal_status_writer_{0};
  dds_entity_t navigation_state_writer_{0};
  dds_entity_t operator_motion_ack_writer_{0};
  dds_entity_t operator_motion_status_writer_{0};
  dds_entity_t exploration_segment_ack_writer_{0};
  dds_entity_t exploration_segment_status_writer_{0};

  dds_entity_t inspection_task_ack_writer_{0};
  dds_entity_t inspection_status_writer_{0};
  dds_entity_t inspection_task_event_writer_{0};
  dds_entity_t inspection_evidence_request_writer_{0};
  std::string host_boot_id_;
  std::string producer_boot_id_;
  std::uint64_t output_seq_{0};
  std::uint64_t goal_status_seq_{0};
  std::uint64_t navigation_state_seq_{0};
  nav_kernel::Twist last_output_command_{};
  DdsStatus *status_{nullptr};
};

}  // namespace lingtu::nav::endpoint
