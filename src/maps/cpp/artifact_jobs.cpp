#include "lingtu/maps/artifact_jobs.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

bool IsTerminal(ArtifactJobState state) {
  return state == ArtifactJobState::kSucceeded ||
      state == ArtifactJobState::kFailed ||
      state == ArtifactJobState::kCancelled;
}

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string OwnerId() {
#if defined(_WIN32)
  const auto pid = static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  const auto pid = static_cast<std::uint64_t>(getpid());
#endif
  std::ostringstream out;
  out << "pid-" << pid;
  return out.str();
}

std::string Encode(const std::string& value) {
  std::ostringstream out;
  out << std::uppercase << std::hex;
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.' || ch == '/') {
      out << static_cast<char>(ch);
    } else {
      out << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(ch);
    }
  }
  return out.str();
}

int HexValue(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

std::string Decode(const std::string& value) {
  std::string out;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] == '%' && i + 2 < value.size()) {
      const int high = HexValue(value[i + 1]);
      const int low = HexValue(value[i + 2]);
      if (high >= 0 && low >= 0) {
        out.push_back(static_cast<char>((high << 4) | low));
        i += 2;
        continue;
      }
    }
    out.push_back(value[i]);
  }
  return out;
}

ArtifactJobState ParseState(const std::string& value) {
  if (value == "RUNNING") return ArtifactJobState::kRunning;
  if (value == "SUCCEEDED") return ArtifactJobState::kSucceeded;
  if (value == "FAILED") return ArtifactJobState::kFailed;
  if (value == "CANCELLED") return ArtifactJobState::kCancelled;
  return ArtifactJobState::kQueued;
}

std::string JsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string SafeJobId(const std::string& request_id) {
  if (request_id.empty()) {
    return "";
  }
  std::string out;
  for (const unsigned char ch : request_id) {
    if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.') {
      out.push_back(static_cast<char>(ch));
    } else {
      std::ostringstream encoded;
      encoded << '_' << std::uppercase << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(ch);
      out += encoded.str();
    }
  }
  return out;
}

bool SameRequest(const ArtifactJobRequest& lhs, const ArtifactJobRequest& rhs) {
  return lhs.request_id == rhs.request_id && lhs.map_id == rhs.map_id &&
      lhs.artifact_type == rhs.artifact_type && lhs.parameters == rhs.parameters;
}

}  // namespace

const char* ArtifactJobStateName(ArtifactJobState state) {
  switch (state) {
    case ArtifactJobState::kQueued:
      return "QUEUED";
    case ArtifactJobState::kRunning:
      return "RUNNING";
    case ArtifactJobState::kSucceeded:
      return "SUCCEEDED";
    case ArtifactJobState::kFailed:
      return "FAILED";
    case ArtifactJobState::kCancelled:
      return "CANCELLED";
  }
  return "UNKNOWN";
}

class ArtifactJobWorkerImpl {
 public:
  ArtifactJobWorkerImpl(ArtifactJobWorkerConfig config, ArtifactBuildFunction build)
      : config_(std::move(config)), build_(std::move(build)), owner_id_(OwnerId()) {
    if (config_.journal_root.empty()) {
      throw std::invalid_argument("ArtifactJobWorker requires a journal_root");
    }
    if (!build_) {
      throw std::invalid_argument("ArtifactJobWorker requires a build callback");
    }
    std::filesystem::create_directories(config_.journal_root);
    Recover();
    if (config_.start_immediately) {
      Start();
    }
  }

  ~ArtifactJobWorkerImpl() {
    Stop();
  }

  void Start() {
    std::lock_guard<std::mutex> lock(mu_);
    if (running_) {
      return;
    }
    stop_requested_ = false;
    running_ = true;
    worker_ = std::thread([this]() { WorkerLoop(); });
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mu_);
      stop_requested_ = true;
      cv_.notify_all();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    std::lock_guard<std::mutex> lock(mu_);
    running_ = false;
  }

  void Recover() {
    std::lock_guard<std::mutex> lock(mu_);
    LoadFromDiskLocked();
    bool changed = RequeueExpiredLeasesLocked(NowNs());
    for (auto& [id, record] : jobs_) {
      if (!IsTerminal(record.status.state)) {
        record.status.recovered = true;
        PersistLocked(record);
        changed = true;
      }
    }
    if (changed) {
      cv_.notify_all();
    }
  }

  ArtifactJobSubmitResult Submit(const ArtifactJobRequest& request) {
    std::lock_guard<std::mutex> lock(mu_);
    const auto validation = ValidateRequest(request);
    if (!validation.empty()) {
      ArtifactJobSubmitResult result;
      result.reason_code = validation;
      return result;
    }
    const auto job_id = SafeJobId(request.request_id);
    auto existing = jobs_.find(job_id);
    if (existing != jobs_.end()) {
      if (!SameRequest(existing->second.request, request)) {
        ArtifactJobSubmitResult result;
        result.reason_code = "idempotency_conflict";
        result.status = existing->second.status;
        return result;
      }
      existing->second.status.replayed = true;
      ArtifactJobSubmitResult result;
      result.accepted = true;
      result.replayed = true;
      result.status = existing->second.status;
      return result;
    }
    if (ActiveDepthLocked() >= config_.max_queue_depth) {
      ArtifactJobSubmitResult result;
      result.reason_code = "queue_full";
      return result;
    }

    JobRecord record;
    record.request = request;
    record.status.job_id = job_id;
    record.status.request_id = request.request_id;
    record.status.map_id = request.map_id;
    record.status.artifact_type = request.artifact_type;
    record.status.state = ArtifactJobState::kQueued;
    record.status.created_at_ns = NowNs();
    record.status.updated_at_ns = record.status.created_at_ns;
    record.status.message = "Queued";
    record.status.attempt = 0;
    record.job_dir = config_.journal_root / job_id;
    std::filesystem::create_directories(record.job_dir);
    PersistLocked(record);
    AppendEventLocked(record, "SUBMITTED", "Queued artifact build request");
    jobs_.emplace(job_id, record);
    cv_.notify_all();

    ArtifactJobSubmitResult result;
    result.accepted = true;
    result.status = record.status;
    return result;
  }

  ArtifactJobSubmitResult Cancel(const std::string& request_id) {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(SafeJobId(request_id));
    if (found == jobs_.end()) {
      ArtifactJobSubmitResult result;
      result.reason_code = "not_found";
      return result;
    }
    auto& record = found->second;
    if (IsTerminal(record.status.state)) {
      ArtifactJobSubmitResult result;
      result.accepted = true;
      result.replayed = true;
      result.status = record.status;
      return result;
    }
    record.status.cancel_requested = true;
    record.status.updated_at_ns = NowNs();
    record.status.message = "Cancellation requested";
    if (record.status.state == ArtifactJobState::kQueued) {
      record.status.state = ArtifactJobState::kCancelled;
      record.status.progress = 1.0;
      record.status.completed_at_ns = record.status.updated_at_ns;
      record.status.reason_code = "cancelled";
    }
    PersistLocked(record);
    AppendEventLocked(record, "CANCEL_REQUESTED", record.status.message);
    cv_.notify_all();
    ArtifactJobSubmitResult result;
    result.accepted = true;
    result.status = record.status;
    return result;
  }

  ArtifactJobSubmitResult Retry(const std::string& request_id) {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(SafeJobId(request_id));
    if (found == jobs_.end()) {
      ArtifactJobSubmitResult result;
      result.reason_code = "not_found";
      return result;
    }
    auto& record = found->second;
    if (!IsTerminal(record.status.state)) {
      ArtifactJobSubmitResult result;
      result.reason_code = "not_terminal";
      result.status = record.status;
      return result;
    }
    if (record.status.state == ArtifactJobState::kSucceeded) {
      ArtifactJobSubmitResult result;
      result.reason_code = "already_succeeded";
      result.status = record.status;
      return result;
    }
    if (ActiveDepthLocked() >= config_.max_queue_depth) {
      ArtifactJobSubmitResult result;
      result.reason_code = "queue_full";
      result.status = record.status;
      return result;
    }
    const auto now = NowNs();
    record.status.state = ArtifactJobState::kQueued;
    record.status.progress = 0.0;
    record.status.message = "Retry queued";
    record.status.reason_code.clear();
    record.status.cancel_requested = false;
    record.status.started_at_ns = 0;
    record.status.completed_at_ns = 0;
    record.status.heartbeat_at_ns = 0;
    record.status.lease_expires_at_ns = 0;
    record.status.updated_at_ns = now;
    record.status.replayed = false;
    record.status.recovered = false;
    record.status.artifacts.clear();
    PersistLocked(record);
    AppendEventLocked(record, "RETRY_QUEUED", record.status.message);
    cv_.notify_all();
    ArtifactJobSubmitResult result;
    result.accepted = true;
    result.status = record.status;
    return result;
  }

  std::optional<ArtifactJobStatus> GetStatus(const std::string& request_id) const {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(SafeJobId(request_id));
    if (found == jobs_.end()) {
      return std::nullopt;
    }
    return found->second.status;
  }

  std::vector<ArtifactJobStatus> ListStatuses(std::size_t limit) const {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<ArtifactJobStatus> statuses;
    statuses.reserve(jobs_.size());
    for (const auto& [id, record] : jobs_) {
      (void)id;
      statuses.push_back(record.status);
    }
    std::sort(statuses.begin(), statuses.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.created_at_ns != rhs.created_at_ns) {
        return lhs.created_at_ns < rhs.created_at_ns;
      }
      return lhs.request_id < rhs.request_id;
    });
    if (statuses.size() > limit) {
      statuses.resize(limit);
    }
    return statuses;
  }

  std::optional<ArtifactJobStatus> Wait(
      const std::string& request_id,
      std::chrono::milliseconds timeout) const {
    const auto job_id = SafeJobId(request_id);
    std::unique_lock<std::mutex> lock(mu_);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      auto found = jobs_.find(job_id);
      if (found == jobs_.end()) {
        return std::nullopt;
      }
      if (IsTerminal(found->second.status.state)) {
        return found->second.status;
      }
      if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        return found->second.status;
      }
    }
  }

  bool IsCancelRequested(const std::string& job_id) const {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(job_id);
    return found != jobs_.end() && found->second.status.cancel_requested;
  }

  bool TouchProgress(const std::string& job_id, double progress, const std::string& message) {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(job_id);
    if (found == jobs_.end() || IsTerminal(found->second.status.state)) {
      return false;
    }
    auto& status = found->second.status;
    status.progress = std::max(0.0, std::min(0.999, progress));
    status.heartbeat_at_ns = NowNs();
    status.lease_expires_at_ns =
        status.heartbeat_at_ns +
        std::chrono::duration_cast<std::chrono::nanoseconds>(config_.lease_ttl).count();
    status.updated_at_ns = status.heartbeat_at_ns;
    if (!message.empty()) {
      status.message = message;
    }
    PersistLocked(found->second);
    cv_.notify_all();
    return !status.cancel_requested;
  }

  bool TouchHeartbeat(const std::string& job_id, const std::string& message) {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(job_id);
    if (found == jobs_.end() || IsTerminal(found->second.status.state)) {
      return false;
    }
    auto& status = found->second.status;
    status.heartbeat_at_ns = NowNs();
    status.lease_expires_at_ns =
        status.heartbeat_at_ns +
        std::chrono::duration_cast<std::chrono::nanoseconds>(config_.lease_ttl).count();
    status.updated_at_ns = status.heartbeat_at_ns;
    if (!message.empty()) {
      status.message = message;
    }
    PersistLocked(found->second);
    cv_.notify_all();
    return !status.cancel_requested;
  }

 private:
  struct JobRecord {
    ArtifactJobRequest request;
    ArtifactJobStatus status;
    std::filesystem::path job_dir;
  };

  std::string ValidateRequest(const ArtifactJobRequest& request) const {
    if (request.request_id.empty()) return "missing_request_id";
    if (request.map_id.empty()) return "missing_map_id";
    if (request.artifact_type.empty()) return "missing_artifact_type";
    if (SafeJobId(request.request_id).empty()) return "invalid_request_id";
    return "";
  }

  std::size_t ActiveDepthLocked() const {
    std::size_t depth = 0;
    for (const auto& [id, record] : jobs_) {
      (void)id;
      if (!IsTerminal(record.status.state)) {
        ++depth;
      }
    }
    return depth;
  }

  bool RequeueExpiredLeasesLocked(std::int64_t now) {
    bool changed = false;
    for (auto& [id, record] : jobs_) {
      (void)id;
      if (record.status.state != ArtifactJobState::kRunning) {
        continue;
      }
      if (record.status.lease_expires_at_ns > 0 && record.status.lease_expires_at_ns >= now) {
        continue;
      }
      record.status.state = ArtifactJobState::kQueued;
      record.status.progress = std::min(record.status.progress, 0.99);
      record.status.message = "Recovered expired artifact build lease";
      record.status.reason_code.clear();
      record.status.recovered = true;
      record.status.cancel_requested = false;
      record.status.lease_expires_at_ns = 0;
      record.status.lease_owner.clear();
      record.status.updated_at_ns = now;
      PersistLocked(record);
      AppendEventLocked(record, "RECOVERED", record.status.message);
      changed = true;
    }
    return changed;
  }

  void WorkerLoop() {
    while (true) {
      JobRecord record;
      {
        std::unique_lock<std::mutex> lock(mu_);
        cv_.wait_for(lock, config_.heartbeat_interval, [this]() {
          return stop_requested_ || PickNextQueuedLocked().has_value();
        });
        if (stop_requested_) {
          return;
        }
        RequeueExpiredLeasesLocked(NowNs());
        const auto next = PickNextQueuedLocked();
        if (!next.has_value()) {
          continue;
        }
        auto& stored = jobs_.at(*next);
        const auto now = NowNs();
        stored.status.state = ArtifactJobState::kRunning;
        stored.status.attempt += 1;
        stored.status.started_at_ns = now;
        stored.status.updated_at_ns = now;
        stored.status.heartbeat_at_ns = now;
        stored.status.lease_expires_at_ns =
            now + std::chrono::duration_cast<std::chrono::nanoseconds>(config_.lease_ttl).count();
        stored.status.lease_owner = owner_id_;
        stored.status.message = "Running";
        stored.status.reason_code.clear();
        stored.status.progress = std::max(0.01, stored.status.progress);
        PersistLocked(stored);
        AppendEventLocked(stored, "STARTED", "Artifact build started");
        record = stored;
      }
      RunOne(record.status.job_id);
    }
  }

  std::optional<std::string> PickNextQueuedLocked() const {
    std::optional<std::string> best;
    std::int64_t best_created = 0;
    for (const auto& [id, record] : jobs_) {
      if (record.status.state != ArtifactJobState::kQueued) {
        continue;
      }
      if (!best.has_value() || record.status.created_at_ns < best_created ||
          (record.status.created_at_ns == best_created && id < *best)) {
        best = id;
        best_created = record.status.created_at_ns;
      }
    }
    return best;
  }

  void RunOne(const std::string& job_id) {
    ArtifactJobRequest request;
    {
      std::lock_guard<std::mutex> lock(mu_);
      auto found = jobs_.find(job_id);
      if (found == jobs_.end()) {
        return;
      }
      request = found->second.request;
    }

    std::atomic<bool> heartbeat_done{false};
    std::thread heartbeat([&]() {
      while (!heartbeat_done.load()) {
        std::this_thread::sleep_for(config_.heartbeat_interval);
        if (!heartbeat_done.load()) {
          TouchProgress(job_id, CurrentProgress(job_id), "");
        }
      }
    });

    ArtifactBuildResult build_result;
    try {
      ArtifactJobContext context(this, job_id);
      build_result = build_(request, context);
    } catch (const std::exception& error) {
      build_result.success = false;
      build_result.retryable = true;
      build_result.reason_code = "exception";
      build_result.message = error.what();
    } catch (...) {
      build_result.success = false;
      build_result.retryable = true;
      build_result.reason_code = "exception";
      build_result.message = "Unknown artifact build exception";
    }
    heartbeat_done.store(true);
    if (heartbeat.joinable()) {
      heartbeat.join();
    }

    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return;
    }
    auto& record = found->second;
    const auto now = NowNs();
    record.status.updated_at_ns = now;
    record.status.completed_at_ns = now;
    record.status.lease_expires_at_ns = 0;
    record.status.heartbeat_at_ns = now;
    if (record.status.cancel_requested) {
      record.status.state = ArtifactJobState::kCancelled;
      record.status.reason_code = "cancelled";
      record.status.message = "Cancelled";
      record.status.progress = 1.0;
      AppendEventLocked(record, "CANCELLED", record.status.message);
    } else if (build_result.success) {
      record.status.state = ArtifactJobState::kSucceeded;
      record.status.reason_code.clear();
      record.status.message = build_result.message.empty() ? "Succeeded" : build_result.message;
      record.status.progress = 1.0;
      record.status.artifacts = build_result.artifacts;
      AppendEventLocked(record, "SUCCEEDED", record.status.message);
    } else {
      record.status.state = ArtifactJobState::kFailed;
      record.status.reason_code =
          build_result.reason_code.empty() ? "build_failed" : build_result.reason_code;
      record.status.message = build_result.message.empty() ? "Artifact build failed" :
                                                            build_result.message;
      record.status.progress = std::min(record.status.progress, 0.999);
      AppendEventLocked(record, "FAILED", record.status.message);
    }
    PersistLocked(record);
    cv_.notify_all();
  }

  double CurrentProgress(const std::string& job_id) const {
    std::lock_guard<std::mutex> lock(mu_);
    auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return 0.0;
    }
    return found->second.status.progress;
  }

  void LoadFromDiskLocked() {
    jobs_.clear();
    std::error_code ignored;
    if (!std::filesystem::is_directory(config_.journal_root, ignored)) {
      return;
    }
    for (const auto& entry : std::filesystem::directory_iterator(config_.journal_root, ignored)) {
      if (!entry.is_directory()) {
        continue;
      }
      auto record = LoadRecord(entry.path());
      if (!record.has_value()) {
        continue;
      }
      jobs_[record->status.job_id] = *record;
    }
  }

  std::optional<JobRecord> LoadRecord(const std::filesystem::path& job_dir) const {
    std::ifstream file(job_dir / "job.state", std::ios::binary);
    if (!file) {
      return std::nullopt;
    }
    JobRecord record;
    record.job_dir = job_dir;
    record.status.job_id = job_dir.filename().string();
    std::string line;
    while (std::getline(file, line)) {
      const auto split = line.find('=');
      if (split == std::string::npos) {
        continue;
      }
      const auto key = line.substr(0, split);
      const auto value = Decode(line.substr(split + 1));
      if (key == "request_id") {
        record.request.request_id = value;
        record.status.request_id = value;
      } else if (key == "map_id") {
        record.request.map_id = value;
        record.status.map_id = value;
      } else if (key == "artifact_type") {
        record.request.artifact_type = value;
        record.status.artifact_type = value;
      } else if (key.rfind("param.", 0) == 0U) {
        record.request.parameters[Decode(key.substr(6))] = value;
      } else if (key == "state") {
        record.status.state = ParseState(value);
      } else if (key == "progress") {
        record.status.progress = std::stod(value);
      } else if (key == "message") {
        record.status.message = value;
      } else if (key == "reason_code") {
        record.status.reason_code = value;
      } else if (key == "created_at_ns") {
        record.status.created_at_ns = std::stoll(value);
      } else if (key == "updated_at_ns") {
        record.status.updated_at_ns = std::stoll(value);
      } else if (key == "started_at_ns") {
        record.status.started_at_ns = std::stoll(value);
      } else if (key == "completed_at_ns") {
        record.status.completed_at_ns = std::stoll(value);
      } else if (key == "heartbeat_at_ns") {
        record.status.heartbeat_at_ns = std::stoll(value);
      } else if (key == "lease_expires_at_ns") {
        record.status.lease_expires_at_ns = std::stoll(value);
      } else if (key == "lease_owner") {
        record.status.lease_owner = value;
      } else if (key == "attempt") {
        record.status.attempt = static_cast<std::uint32_t>(std::stoul(value));
      } else if (key == "cancel_requested") {
        record.status.cancel_requested = value == "1";
      } else if (key == "recovered") {
        record.status.recovered = value == "1";
      } else if (key.rfind("artifact.", 0) == 0U) {
        record.status.artifacts.push_back(value);
      }
    }
    if (record.request.request_id.empty() || record.request.map_id.empty() ||
        record.request.artifact_type.empty()) {
      return std::nullopt;
    }
    if (record.status.created_at_ns == 0) {
      record.status.created_at_ns = NowNs();
    }
    return record;
  }

  void PersistLocked(const JobRecord& record) const {
    std::filesystem::create_directories(record.job_dir);
    const auto tmp = record.job_dir / "job.state.tmp";
    {
      std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
      if (!file) {
        throw std::runtime_error("failed to write artifact job state");
      }
      file << "request_id=" << Encode(record.request.request_id) << "\n"
           << "map_id=" << Encode(record.request.map_id) << "\n"
           << "artifact_type=" << Encode(record.request.artifact_type) << "\n";
      for (const auto& [key, value] : record.request.parameters) {
        file << "param." << Encode(key) << "=" << Encode(value) << "\n";
      }
      file << "state=" << ArtifactJobStateName(record.status.state) << "\n"
           << "progress=" << record.status.progress << "\n"
           << "message=" << Encode(record.status.message) << "\n"
           << "reason_code=" << Encode(record.status.reason_code) << "\n"
           << "created_at_ns=" << record.status.created_at_ns << "\n"
           << "updated_at_ns=" << record.status.updated_at_ns << "\n"
           << "started_at_ns=" << record.status.started_at_ns << "\n"
           << "completed_at_ns=" << record.status.completed_at_ns << "\n"
           << "heartbeat_at_ns=" << record.status.heartbeat_at_ns << "\n"
           << "lease_expires_at_ns=" << record.status.lease_expires_at_ns << "\n"
           << "lease_owner=" << Encode(record.status.lease_owner) << "\n"
           << "attempt=" << record.status.attempt << "\n"
           << "cancel_requested=" << (record.status.cancel_requested ? "1" : "0") << "\n"
           << "recovered=" << (record.status.recovered ? "1" : "0") << "\n";
      for (std::size_t i = 0; i < record.status.artifacts.size(); ++i) {
        file << "artifact." << i << "=" << Encode(record.status.artifacts[i].string()) << "\n";
      }
    }
    std::error_code ignored;
    std::filesystem::rename(tmp, record.job_dir / "job.state", ignored);
    if (ignored) {
      std::filesystem::remove(record.job_dir / "job.state", ignored);
      std::filesystem::rename(tmp, record.job_dir / "job.state");
    }
  }

  void AppendEventLocked(const JobRecord& record, const std::string& event,
                         const std::string& message) const {
    std::ofstream file(record.job_dir / "events.jsonl", std::ios::binary | std::ios::app);
    file << "{\"ts_ns\":" << NowNs() << ",\"event\":\"" << JsonEscape(event)
         << "\",\"request_id\":\"" << JsonEscape(record.request.request_id)
         << "\",\"map_id\":\"" << JsonEscape(record.request.map_id)
         << "\",\"artifact_type\":\"" << JsonEscape(record.request.artifact_type)
         << "\",\"attempt\":" << record.status.attempt
         << ",\"state\":\"" << ArtifactJobStateName(record.status.state)
         << "\",\"message\":\"" << JsonEscape(message) << "\"}\n";
  }

  ArtifactJobWorkerConfig config_;
  ArtifactBuildFunction build_;
  std::string owner_id_;
  mutable std::mutex mu_;
  mutable std::condition_variable cv_;
  std::unordered_map<std::string, JobRecord> jobs_;
  std::thread worker_;
  bool stop_requested_{false};
  bool running_{false};
};

ArtifactJobContext::ArtifactJobContext(ArtifactJobWorkerImpl* impl, std::string job_id)
    : impl_(impl), job_id_(std::move(job_id)) {}

bool ArtifactJobContext::IsCancelRequested() const {
  return impl_ != nullptr && impl_->IsCancelRequested(job_id_);
}

bool ArtifactJobContext::UpdateProgress(double progress, const std::string& message) {
  return impl_ != nullptr && impl_->TouchProgress(job_id_, progress, message);
}

bool ArtifactJobContext::Heartbeat(const std::string& message) {
  return impl_ != nullptr && impl_->TouchHeartbeat(job_id_, message);
}

ArtifactJobWorker::ArtifactJobWorker(ArtifactJobWorkerConfig config, ArtifactBuildFunction build)
    : impl_(std::make_unique<ArtifactJobWorkerImpl>(std::move(config), std::move(build))) {}

ArtifactJobWorker::~ArtifactJobWorker() = default;

void ArtifactJobWorker::Start() {
  impl_->Start();
}

void ArtifactJobWorker::Stop() {
  impl_->Stop();
}

void ArtifactJobWorker::Recover() {
  impl_->Recover();
}

ArtifactJobSubmitResult ArtifactJobWorker::Submit(const ArtifactJobRequest& request) {
  return impl_->Submit(request);
}

ArtifactJobSubmitResult ArtifactJobWorker::Cancel(const std::string& request_id) {
  return impl_->Cancel(request_id);
}

ArtifactJobSubmitResult ArtifactJobWorker::Retry(const std::string& request_id) {
  return impl_->Retry(request_id);
}

std::optional<ArtifactJobStatus> ArtifactJobWorker::GetStatus(
    const std::string& request_id) const {
  return impl_->GetStatus(request_id);
}

std::vector<ArtifactJobStatus> ArtifactJobWorker::ListStatuses(std::size_t limit) const {
  return impl_->ListStatuses(limit);
}

std::optional<ArtifactJobStatus> ArtifactJobWorker::Wait(
    const std::string& request_id,
    std::chrono::milliseconds timeout) const {
  return impl_->Wait(request_id, timeout);
}

}  // namespace lingtu::maps
