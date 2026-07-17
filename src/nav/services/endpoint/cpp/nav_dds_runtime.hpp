#pragma once

#include "dds_drain_policy.hpp"
#include "message/cpp/navigation_command.hpp"
#include "nav_kernel/types.hpp"
#include "nav/inspection/inspection.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::nav::endpoint {

inline bool applyInspectionEvidenceResult(
    lingtu::nav::inspection::Executor& executor,
    const lingtu_dds_InspectionEvidenceResult& result,
    double now_s) {
  const auto& status = executor.status();
  const std::string request_id = result.request_id == nullptr ? "" : result.request_id;
  if (status.state != lingtu::nav::inspection::RunState::kActionPending ||
      status.action_request_id.empty() || request_id != status.action_request_id) {
    return false;
  }
  const std::string evidence_id =
      result.evidence_id == nullptr ? "" : result.evidence_id;
  std::string reason = result.reason == nullptr ? "" : result.reason;
  if (!result.persisted && reason.empty()) {
    reason = "evidence_not_persisted";
  }
  return executor.OnActionResult(
      request_id,
      result.persisted,
      evidence_id,
      reason,
      now_s);
}

class DdsRuntime {
 public:
  explicit DdsRuntime(
      int domain_id,
      bool allow_legacy_motion_inputs = false);
  ~DdsRuntime();

  DdsRuntime(const DdsRuntime&) = delete;
  DdsRuntime& operator=(const DdsRuntime&) = delete;

  template <typename Handler>
  void drainOdometry(Handler&& handler) {
    drainReader<lingtu_dds_Odometry>(
        odom_reader_, lingtu_dds_Odometry_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTf(Handler&& handler) {
    drainReader<lingtu_dds_TFMessage>(
        tf_reader_,
        lingtu_dds_TFMessage_desc,
        std::forward<Handler>(handler),
        DdsDrainProfile::kTransform);
  }

  // Explicit compatibility adapter. These readers are not created unless
  // --allow-legacy-motion-inputs=true; product control uses command requests.
  template <typename Handler>
  void drainLegacyGoals(Handler&& handler) {
    drainReader<lingtu_dds_PoseStamped>(
        legacy_goal_reader_,
        lingtu_dds_PoseStamped_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCloud(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        cloud_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTerrainMap(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        terrain_map_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTerrainMapExt(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        terrain_map_ext_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainLegacyGlobalPath(Handler&& handler) {
    drainReader<lingtu_dds_Path>(
        legacy_global_path_reader_,
        lingtu_dds_Path_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTraversability(Handler&& handler) {
    drainReader<lingtu_dds_OccupancyGrid>(
        traversability_reader_, lingtu_dds_OccupancyGrid_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainLocalizationHealth(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        localization_health_reader_,
        lingtu_dds_Text_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainDriverControlState(Handler&& handler) {
    drainReader<lingtu_dds_DriverControlState>(
        driver_control_state_reader_,
        lingtu_dds_DriverControlState_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainMapClearing(Handler&& handler) {
    drainReader<lingtu_dds_Bool>(
        map_clearing_reader_, lingtu_dds_Bool_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCloudClearing(Handler&& handler) {
    drainReader<lingtu_dds_Bool>(
        cloud_clearing_reader_, lingtu_dds_Bool_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainLegacyCancel(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        legacy_cancel_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainLegacyTeleopCmdVel(Handler&& handler) {
    drainReader<lingtu_dds_TwistStamped>(
        legacy_teleop_cmd_reader_,
        lingtu_dds_TwistStamped_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCommandRequests(Handler&& handler) {
    drainReader<lingtu_dds_NavigationCommandRequest>(
        command_request_reader_,
        lingtu_dds_NavigationCommandRequest_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainInspectionCommands(Handler&& handler) {
    drainReader<lingtu_dds_InspectionCommandRequest>(
        inspection_command_reader_,
        lingtu_dds_InspectionCommandRequest_desc,
        std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainInspectionEvidenceResults(Handler&& handler) {
    drainReader<lingtu_dds_InspectionEvidenceResult>(
        inspection_evidence_result_reader_,
        lingtu_dds_InspectionEvidenceResult_desc,
        std::forward<Handler>(handler));
  }

  void writeLocalPath(const std::vector<nav_kernel::Vec3>& path);
  void writeGlobalPath(const std::vector<nav_kernel::Vec3>& path, double stamp_s);
  void writeWayPoint(const nav_kernel::Vec3& point);
  bool writeCmdVel(const nav_kernel::Twist& cmd);
  void writeCommandAck(
      const char* request_id,
      lingtu::message::NavigationCommandKind kind,
      bool accepted,
      const char* reason);
  void writeInspectionAck(
      const char* request_id,
      lingtu::nav::inspection::CommandKind kind,
      bool accepted,
      const char* reason,
      const char* run_id);
  void writeInspectionStatus(const lingtu::nav::inspection::RunStatus& status);
  bool inspectionEvidenceWorkerMatched() const noexcept;
  bool writeInspectionEvidenceRequest(
      const lingtu::nav::inspection::ActionRequest& request,
      const std::string& map_id,
      std::int64_t map_version,
      double deadline_s);

 private:
  template <typename T, typename Handler>
  void drainReader(
      dds_entity_t reader,
      const dds_topic_descriptor_t& descriptor,
      Handler&& handler,
      DdsDrainProfile profile = DdsDrainProfile::kDefault) {
    if (reader <= 0) {
      return;
    }

    const DdsDrainBudget budget = drainBudget(profile);
    drainBatches(budget, [&](std::size_t capacity) -> std::ptrdiff_t {
      void* samples[kDdsReaderBatchSize];
      dds_sample_info_t infos[kDdsReaderBatchSize];
      for (auto& sample : samples) {
        sample = dds_alloc(sizeof(T));
        std::memset(sample, 0, sizeof(T));
      }

      const dds_return_t count = dds_take(
          reader, samples, infos, capacity, capacity);
      if (count >= 0) {
        for (dds_return_t i = 0; i < count; ++i) {
          if (infos[i].valid_data) {
            handler(*static_cast<T*>(samples[i]));
          }
        }
      } else {
        logDdsError(count, "dds_take");
      }
      for (auto& sample : samples) {
        dds_sample_free(sample, &descriptor, DDS_FREE_ALL);
      }
      return static_cast<std::ptrdiff_t>(count);
    });
  }

  static void logDdsError(dds_return_t value, const char* what);

  dds_entity_t reader(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label);
  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label);

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t odom_reader_{0};
  dds_entity_t tf_reader_{0};
  dds_entity_t legacy_goal_reader_{0};
  dds_entity_t cloud_reader_{0};
  dds_entity_t terrain_map_reader_{0};
  dds_entity_t terrain_map_ext_reader_{0};
  dds_entity_t legacy_global_path_reader_{0};
  dds_entity_t traversability_reader_{0};
  dds_entity_t localization_health_reader_{0};
  dds_entity_t driver_control_state_reader_{0};
  dds_entity_t map_clearing_reader_{0};
  dds_entity_t cloud_clearing_reader_{0};
  dds_entity_t legacy_cancel_reader_{0};
  dds_entity_t legacy_teleop_cmd_reader_{0};
  dds_entity_t command_request_reader_{0};
  dds_entity_t inspection_command_reader_{0};
  dds_entity_t inspection_evidence_result_reader_{0};
  dds_entity_t global_path_writer_{0};
  dds_entity_t local_path_writer_{0};
  dds_entity_t way_point_writer_{0};
  dds_entity_t cmd_vel_writer_{0};
  dds_entity_t command_ack_writer_{0};
  dds_entity_t inspection_ack_writer_{0};
  dds_entity_t inspection_status_writer_{0};
  dds_entity_t inspection_evidence_request_writer_{0};
  std::string host_boot_id_;
  std::string producer_boot_id_;
  std::uint64_t output_seq_{0};
};

}  // namespace lingtu::nav::endpoint
