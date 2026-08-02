#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "inspection/inspection_task_event_outbox.hpp"
#include "lingtu_slam.h"
#include "message/cpp/navigation_command.hpp"
#include "message/cpp/operator_motion.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav_kernel/types.hpp"
#include "plan/dds_drain_policy.hpp"
#include "status/navigation_state.hpp"

namespace lingtu::nav::endpoint {

inline bool applyInspectionEvidenceResult(lingtu::nav::inspection::Executor &executor,
                                          const lingtu_dds_InspectionEvidenceResult &result,
                                          double now_s) {
  const auto &status = executor.status();
  const std::string request_id = result.request_id == nullptr ? "" : result.request_id;
  if (status.state != lingtu::nav::inspection::RunState::kActionPending ||
      status.action_request_id.empty() || request_id != status.action_request_id) {
    return false;
  }
  const std::string evidence_id = result.evidence_id == nullptr ? "" : result.evidence_id;
  std::string reason = result.reason == nullptr ? "" : result.reason;
  if (!result.persisted && reason.empty()) {
    reason = "evidence_not_persisted";
  }
  return executor.OnActionResult(request_id, result.persisted, evidence_id, reason, now_s);
}

// Owning boundary types for the exploration DDS contract. These deliberately
// stay independent from the executor's internal map and planning types: raw
// DDS samples are released immediately after a drain callback returns.
struct ExplorationExecutionGridView {
  double stamp_s{0.0};
  std::string frame_id;
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  double origin_qx{0.0};
  double origin_qy{0.0};
  double origin_qz{0.0};
  double origin_qw{1.0};
  std::vector<std::uint8_t> occupancy;
  std::vector<std::uint8_t> terrain_cost;
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};
  double terrain_risk_stamp_s{0.0};
  bool terrain_risk_ready{false};
  bool payload_complete{false};
};

struct ExplorationSegmentTarget {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct ExplorationSegmentRequestView {
  double stamp_s{0.0};
  std::string frame_id;
  std::string request_id;
  std::int32_t kind{0};
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t minimum_generation{0U};
  ExplorationSegmentTarget target;
  std::string reason;
};

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

[[nodiscard]] ExplorationExecutionGridView
copyExplorationExecutionGrid(const lingtu_dds_ExplorationExecutionGrid &message);
[[nodiscard]] ExplorationSegmentRequestView
copyExplorationSegmentRequest(const lingtu_dds_ExplorationSegmentRequest &message);

class DdsRuntime {
 public:
  explicit DdsRuntime(int domain_id);
  ~DdsRuntime();

  DdsRuntime(const DdsRuntime &) = delete;
  DdsRuntime &operator=(const DdsRuntime &) = delete;

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
                                        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTerrainMap(Handler &&handler) {
    drainReader<lingtu_dds_PointCloud2>(terrain_map_reader_, lingtu_dds_PointCloud2_desc,
                                        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTerrainMapExt(Handler &&handler) {
    drainReader<lingtu_dds_PointCloud2>(terrain_map_ext_reader_, lingtu_dds_PointCloud2_desc,
                                        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTraversability(Handler &&handler) {
    drainReader<lingtu_dds_OccupancyGrid>(traversability_reader_, lingtu_dds_OccupancyGrid_desc,
                                          std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainLocalizationHealth(Handler &&handler) {
    drainReader<lingtu_dds_Text>(localization_health_reader_, lingtu_dds_Text_desc,
                                 std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainDriverControlState(Handler &&handler) {
    drainReader<lingtu_dds_DriverControlState>(driver_control_state_reader_,
                                               lingtu_dds_DriverControlState_desc,
                                               std::forward<Handler>(handler));
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
        });
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

  void writeLocalPath(const std::vector<nav_kernel::Vec3> &path);
  void writeGlobalPath(const std::vector<nav_kernel::Vec3> &path, double stamp_s);
  void writeWayPoint(const nav_kernel::Vec3 &point);
  std::optional<FinalVelocityCommandReceipt> writeCmdVelSequenced(const nav_kernel::Twist &cmd);
  bool writeCmdVel(const nav_kernel::Twist &cmd);
  const std::string &producerBootId() const noexcept { return producer_boot_id_; }
  std::uint64_t lastOutputSequence() const noexcept { return output_seq_; }
  const nav_kernel::Twist &lastOutputCommand() const noexcept { return last_output_command_; }
  bool writeOperatorMotionAck(const OperatorMotionAckSample &ack);
  bool writeOperatorMotionStatus(const OperatorMotionStatusSample &status);
  bool writeCommandAck(const char *task_id, const char *request_id,
                       lingtu::message::NavigationCommandKind kind, bool accepted,
                       const char *reason);
  bool writeNavigationGoalStatus(const char *task_id, const char *request_id,
                                 lingtu::message::NavigationGoalState state,
                                 std::uint64_t goal_epoch, const char *reason);
  bool writeNavigationState(const NavigationStateSample &state);
  bool writeExplorationSegmentAck(const ExplorationSegmentAck &ack);
  bool writeExplorationSegmentStatus(const ExplorationSegmentStatus &status);

  [[nodiscard]] bool writeInspectionTaskAck(const char *task_id, const char *request_id,
                                            lingtu::nav::inspection::CommandKind kind,
                                            bool accepted, const char *reason, const char *run_id);
  void writeInspectionStatus(const lingtu::nav::inspection::RunStatus &status);
  [[nodiscard]] bool writeInspectionTaskEvent(const InspectionTaskEventEnvelope &event);
  bool inspectionEvidenceWorkerMatched() const noexcept;
  bool writeInspectionEvidenceRequest(const lingtu::nav::inspection::ActionRequest &request,
                                      const std::string &map_id, std::int64_t map_version,
                                      double deadline_s);

 private:
  template <typename T, typename Handler>
  void drainReader(dds_entity_t reader, const dds_topic_descriptor_t &descriptor, Handler &&handler,
                   DdsDrainProfile profile = DdsDrainProfile::kDefault) {
    if (reader <= 0) {
      return;
    }

    const DdsDrainBudget budget = drainBudget(profile);
    drainBatches(budget, [&](std::size_t capacity) -> std::ptrdiff_t {
      void *samples[kDdsReaderBatchSize];
      dds_sample_info_t infos[kDdsReaderBatchSize];
      for (auto &sample : samples) {
        sample = dds_alloc(sizeof(T));
        std::memset(sample, 0, sizeof(T));
      }

      const dds_return_t count = dds_take(reader, samples, infos, capacity, capacity);
      if (count >= 0) {
        for (dds_return_t i = 0; i < count; ++i) {
          if (infos[i].valid_data) {
            handler(*static_cast<T *>(samples[i]));
          }
        }
      } else {
        logDdsError(count, "dds_take");
      }
      for (auto &sample : samples) {
        dds_sample_free(sample, &descriptor, DDS_FREE_ALL);
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
  dds_entity_t localization_health_reader_{0};
  dds_entity_t driver_control_state_reader_{0};
  dds_entity_t map_clearing_reader_{0};
  dds_entity_t cloud_clearing_reader_{0};
  dds_entity_t command_request_reader_{0};
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
};

}  // namespace lingtu::nav::endpoint
