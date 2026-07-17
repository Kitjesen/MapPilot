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

struct MapSnapshot {
  std::string snapshot_id;
  std::filesystem::path source_dir;
  std::string frame_id{"map"};
  std::int64_t captured_at_ns{0};
  std::uint64_t first_sequence{0};
  std::uint64_t last_sequence{0};
  std::string source_sha256;
  bool slam_healthy{true};
  std::string health_message;
};

struct SaveMapRequest {
  std::string request_id;
  std::string map_id;
  SaveRequirements require;
  SourceCommitOptions source;
  OctomapBuildOptions octomap;
  bool activate_on_success{false};
  bool require_slam_healthy{true};
  std::uint64_t minimum_point_count{1};
};

struct SaveMapStatus {
  std::string job_id;
  std::string request_id;
  std::string map_id;
  SaveJobState state{SaveJobState::kWaitingSnapshot};
  SavePhase phase{SavePhase::kCapture};
  double progress{0.0};
  std::string message;
  std::string reason_code;
  std::filesystem::path capture_dir;
  std::filesystem::path version_dir;
  std::filesystem::path manifest_path;
  std::string source_report_json;
  std::string artifact_report_json;
  std::int64_t created_at_ns{0};
  std::int64_t updated_at_ns{0};
  std::int64_t completed_at_ns{0};
  std::int64_t version{0};
  bool cancel_requested{false};
  bool compatibility_ready{false};
  std::string compatibility_message;
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
  std::function<void(const std::string& job_id, SavePhase phase)> before_phase;
  std::function<std::optional<std::string>(
      const std::string& job_id,
      SavePhase phase)> forced_failure;
};

class SaveMapEngine {
 public:
  explicit SaveMapEngine(MapStore& store, SaveMapHooks hooks = {});
  ~SaveMapEngine();

  SaveMapEngine(const SaveMapEngine&) = delete;
  SaveMapEngine& operator=(const SaveMapEngine&) = delete;

  SaveMapResult Begin(const SaveMapRequest& request);
  SaveMapResult ProvideSnapshot(const std::string& job_id, const MapSnapshot& snapshot);
  SaveMapResult Cancel(const std::string& job_id);
  SaveMapResult Retry(const std::string& job_id);
  std::optional<SaveMapStatus> GetStatus(const std::string& job_id) const;
  std::vector<SaveMapStatus> ListStatuses(std::size_t limit = 100U) const;
  std::optional<SaveMapStatus> Wait(
      const std::string& job_id,
      std::chrono::milliseconds timeout) const;
  void Recover();

  std::string BeginJson(const SaveMapRequest& request);
  std::string ProvideSnapshotJson(const std::string& job_id, const MapSnapshot& snapshot);
  std::string CancelJson(const std::string& job_id);
  std::string RetryJson(const std::string& job_id);
  std::string GetStatusJson(const std::string& job_id) const;
  std::string ListStatusesJson(std::size_t limit = 100U) const;
  std::string ListVersionsJson(const std::string& map_id);
  std::string RollbackVersionJson(const std::string& map_id, std::int64_t version);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

const char* SaveJobStateName(SaveJobState state);
const char* SavePhaseName(SavePhase phase);
std::string SaveMapStatusJson(const SaveMapStatus& status);

}  // namespace lingtu::maps
