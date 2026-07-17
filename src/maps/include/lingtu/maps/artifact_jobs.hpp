#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::maps {

enum class ArtifactJobState {
  kQueued,
  kRunning,
  kSucceeded,
  kFailed,
  kCancelled,
};

struct ArtifactJobRequest {
  std::string request_id;
  std::string map_id;
  std::string artifact_type;
  std::map<std::string, std::string> parameters;
};

struct ArtifactBuildResult {
  bool success{false};
  bool retryable{true};
  std::string message;
  std::string reason_code;
  std::vector<std::filesystem::path> artifacts;
};

struct ArtifactJobStatus {
  std::string job_id;
  std::string request_id;
  std::string map_id;
  std::string artifact_type;
  ArtifactJobState state{ArtifactJobState::kQueued};
  double progress{0.0};
  std::string message;
  std::string reason_code;
  std::vector<std::filesystem::path> artifacts;
  std::int64_t created_at_ns{0};
  std::int64_t updated_at_ns{0};
  std::int64_t started_at_ns{0};
  std::int64_t completed_at_ns{0};
  std::int64_t heartbeat_at_ns{0};
  std::int64_t lease_expires_at_ns{0};
  std::string lease_owner;
  std::uint32_t attempt{0};
  bool cancel_requested{false};
  bool recovered{false};
  bool replayed{false};
};

struct ArtifactJobSubmitResult {
  bool accepted{false};
  bool replayed{false};
  std::string reason_code;
  ArtifactJobStatus status;
};

struct ArtifactJobWorkerConfig {
  std::filesystem::path journal_root;
  std::size_t max_queue_depth{64};
  std::chrono::milliseconds lease_ttl{std::chrono::seconds(30)};
  std::chrono::milliseconds heartbeat_interval{std::chrono::seconds(1)};
  bool start_immediately{true};
};

class ArtifactJobWorkerImpl;

class ArtifactJobContext {
 public:
  ArtifactJobContext(const ArtifactJobContext&) = delete;
  ArtifactJobContext& operator=(const ArtifactJobContext&) = delete;

  bool IsCancelRequested() const;
  bool UpdateProgress(double progress, const std::string& message = "");
  bool Heartbeat(const std::string& message = "");

 private:
  friend class ArtifactJobWorkerImpl;
  ArtifactJobContext(ArtifactJobWorkerImpl* impl, std::string job_id);

  ArtifactJobWorkerImpl* impl_;
  std::string job_id_;
};

using ArtifactBuildFunction =
    std::function<ArtifactBuildResult(const ArtifactJobRequest&, ArtifactJobContext&)>;

class ArtifactJobWorker {
 public:
  ArtifactJobWorker(ArtifactJobWorkerConfig config, ArtifactBuildFunction build);
  ~ArtifactJobWorker();

  ArtifactJobWorker(const ArtifactJobWorker&) = delete;
  ArtifactJobWorker& operator=(const ArtifactJobWorker&) = delete;

  void Start();
  void Stop();
  void Recover();

  ArtifactJobSubmitResult Submit(const ArtifactJobRequest& request);
  ArtifactJobSubmitResult Cancel(const std::string& request_id);
  ArtifactJobSubmitResult Retry(const std::string& request_id);
  std::optional<ArtifactJobStatus> GetStatus(const std::string& request_id) const;
  std::vector<ArtifactJobStatus> ListStatuses(std::size_t limit = 100U) const;
  std::optional<ArtifactJobStatus> Wait(
      const std::string& request_id,
      std::chrono::milliseconds timeout) const;

 private:
  std::unique_ptr<ArtifactJobWorkerImpl> impl_;
};

const char* ArtifactJobStateName(ArtifactJobState state);

}  // namespace lingtu::maps
