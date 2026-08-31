#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/store.hpp"

namespace lingtu::maps {

enum class SaveJobState {
  kWaitingSnapshot,
  kQueued,
  kRunning,
  kSucceeded,
  kFailed,
  kCancelled,
};

enum class SavePhase {
  kCapture,
  kValidate,
  kOptimizeSource,
  kProcessSource,
  kBuildArtifacts,
  kVerify,
  kCommit,
  kDone,
};

struct SaveRequirements {
  bool occupancy{true};
  bool octomap{true};
  bool esdf{true};
  bool traversability{true};
  bool semantic{false};
};

struct PgoOptions {
  std::string executable;
  std::string constraints_file{"pose_graph.constraints"};
  double timeout_sec{300.0};
};

struct MapSnapshot {
  std::string snapshot_id;
  std::filesystem::path source_dir;
  std::string frame_id{"map"};
  std::int64_t captured_at_ns{0};
  std::uint64_t first_sequence{0};
  std::uint64_t last_sequence{0};
  std::string slam_boot_id;
  std::string product_session_id;
  std::uint64_t reset_epoch{0};
  std::uint64_t observation_sequence{0};
  std::uint64_t source_point_count{0};
  bool slam_healthy{true};
  std::string health_message;
};

struct SaveMapRequest {
  std::string request_id;
  std::string map_id;
  std::string product_session_id;
  SaveRequirements require;
  PgoOptions pgo;
  SourceCommitOptions source;
  OctomapBuildOptions octomap;
  bool activate_on_success{false};
  bool require_slam_healthy{true};
  // Development-only compatibility for snapshot sources without an atomic
  // native SLAM receipt. Field native_dds saves must leave this disabled.
  bool allow_unverified_snapshot{false};
  std::uint64_t minimum_point_count{1};
};

struct SaveMapStatus {
  std::string job_id;
  std::string request_id;
  std::string map_id;
  std::string product_session_id;
  SaveJobState state{SaveJobState::kWaitingSnapshot};
  SavePhase phase{SavePhase::kCapture};
  double progress{0.0};
  std::string message;
  std::string reason_code;
  std::filesystem::path capture_dir;
  std::filesystem::path map_dir;
  std::string source_report_json;
  std::string artifact_report_json;
  std::int64_t created_at_ns{0};
  std::int64_t updated_at_ns{0};
  std::int64_t completed_at_ns{0};
  bool activation_requested{false};
  bool activation_succeeded{false};
  bool cancel_requested{false};
  bool recovered{false};
  bool replayed{false};
};

struct SaveMapResult {
  bool accepted{false};
  bool replayed{false};
  std::string reason_code;
  SaveMapStatus status;
};

struct SaveMapHooks {
  std::function<void(const std::string &job_id)> after_snapshot_copied;
  std::function<void(const std::string &job_id, SavePhase phase)> before_phase;
  std::function<std::optional<std::string>(const std::string &job_id, SavePhase phase)>
      forced_failure;
};

class SaveMapEngine {
 public:
  explicit SaveMapEngine(MapStore &store, SaveMapHooks hooks = {});
  ~SaveMapEngine();

  SaveMapEngine(const SaveMapEngine &) = delete;
  SaveMapEngine &operator=(const SaveMapEngine &) = delete;

  SaveMapResult Begin(const SaveMapRequest &request);
  SaveMapResult ProvideSnapshot(const std::string &job_id, const MapSnapshot &snapshot);
  SaveMapResult RejectSnapshot(const std::string &job_id, const std::string &reason_code,
                               const std::string &message);
  SaveMapResult Cancel(const std::string &job_id);
  SaveMapResult Retry(const std::string &job_id);
  std::optional<SaveMapStatus> GetStatus(const std::string &job_id) const;
  std::vector<SaveMapStatus> ListStatuses(std::size_t limit = 100U) const;
  std::optional<SaveMapStatus> Wait(const std::string &job_id,
                                    std::chrono::milliseconds timeout) const;
  void Recover();

  std::string BeginJson(const SaveMapRequest &request);
  std::string ProvideSnapshotJson(const std::string &job_id, const MapSnapshot &snapshot);
  std::string RejectSnapshotJson(const std::string &job_id, const std::string &reason_code,
                                 const std::string &message);
  std::string CancelJson(const std::string &job_id);
  std::string RetryJson(const std::string &job_id);
  std::string GetStatusJson(const std::string &job_id) const;
  std::string ListStatusesJson(std::size_t limit = 100U) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

const char *SaveJobStateName(SaveJobState state);
const char *SavePhaseName(SavePhase phase);
std::string SaveMapStatusJson(const SaveMapStatus &status);

}  // namespace lingtu::maps
