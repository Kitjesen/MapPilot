#include "lingtu/maps/artifact_jobs.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using lingtu::maps::ArtifactBuildResult;
using lingtu::maps::ArtifactJobContext;
using lingtu::maps::ArtifactJobRequest;
using lingtu::maps::ArtifactJobState;
using lingtu::maps::ArtifactJobWorker;
using lingtu::maps::ArtifactJobWorkerConfig;

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << message << "\n";
  std::exit(1);
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    Fail(message);
  }
}

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_artifact_jobs_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

void ReplaceStateValue(
    const std::filesystem::path& path,
    const std::string& key,
    const std::string& value) {
  std::istringstream input(ReadFile(path));
  std::ostringstream output;
  std::string line;
  const std::string prefix = key + "=";
  bool replaced = false;
  while (std::getline(input, line)) {
    if (line.rfind(prefix, 0) == 0U) {
      output << prefix << value << '\n';
      replaced = true;
    } else {
      output << line << '\n';
    }
  }
  Require(replaced, "missing persisted artifact job field: " + key);
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << output.str();
}

ArtifactJobRequest Request(
    const std::string& request_id,
    const std::string& map_id,
    const std::string& artifact_type = "occupancy") {
  ArtifactJobRequest request;
  request.request_id = request_id;
  request.map_id = map_id;
  request.artifact_type = artifact_type;
  request.parameters["source"] = map_id + ".pcd";
  return request;
}

ArtifactJobWorkerConfig Config(const std::filesystem::path& root) {
  ArtifactJobWorkerConfig config;
  config.journal_root = root / ".artifact_jobs";
  config.max_queue_depth = 2;
  config.lease_ttl = std::chrono::milliseconds(80);
  config.heartbeat_interval = std::chrono::milliseconds(10);
  return config;
}

lingtu::maps::ArtifactJobStatus WaitTerminal(
    ArtifactJobWorker& worker,
    const std::string& request_id) {
  auto status = worker.Wait(request_id, std::chrono::seconds(5));
  Require(status.has_value(), "artifact job disappeared: " + request_id);
  Require(
      status->state == ArtifactJobState::kSucceeded ||
          status->state == ArtifactJobState::kFailed ||
          status->state == ArtifactJobState::kCancelled,
      "artifact job did not reach a terminal state: " + request_id);
  return *status;
}

}  // namespace

int main() {
  const auto root = TempRoot();
  std::vector<std::string> build_order;
  std::mutex order_mu;

  auto success_build = [&](const ArtifactJobRequest& request, ArtifactJobContext& context) {
    {
      std::lock_guard<std::mutex> lock(order_mu);
      build_order.push_back(request.request_id);
    }
    context.UpdateProgress(0.25, "reading source");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    context.UpdateProgress(0.75, "writing artifact");
    const auto artifact = root / (request.request_id + ".artifact");
    std::ofstream file(artifact, std::ios::binary);
    file << request.map_id << ":" << request.artifact_type;
    ArtifactBuildResult result;
    result.success = true;
    result.message = "built";
    result.artifacts.push_back(artifact);
    return result;
  };

  {
    ArtifactJobWorker worker(Config(root), success_build);
    const auto first = worker.Submit(Request("req-1", "map-a"));
    Require(first.accepted && !first.replayed, "first submit was not accepted");
    const auto replay = worker.Submit(Request("req-1", "map-a"));
    Require(replay.accepted && replay.replayed, "same request_id was not replayed");
    auto conflict_request = Request("req-1", "map-b");
    const auto conflict = worker.Submit(conflict_request);
    Require(!conflict.accepted && conflict.reason_code == "idempotency_conflict",
            "request_id conflict was not rejected");
    Require(WaitTerminal(worker, "req-1").state == ArtifactJobState::kSucceeded,
            "req-1 did not succeed");
    const auto status = worker.GetStatus("req-1");
    Require(status.has_value() && status->progress == 1.0, "success progress was not complete");
    Require(status->attempt == 1, "first success attempt count is wrong");
    Require(!status->lease_owner.empty(), "running lease owner was not preserved in status");
    Require(std::filesystem::is_regular_file(status->artifacts.front()),
            "artifact result path was not preserved");
    const auto journal = ReadFile(Config(root).journal_root / "req-1" / "events.jsonl");
    Require(journal.find("SUBMITTED") != std::string::npos, "journal missing SUBMITTED");
    Require(journal.find("STARTED") != std::string::npos, "journal missing STARTED");
    Require(journal.find("SUCCEEDED") != std::string::npos, "journal missing SUCCEEDED");
  }

  {
    build_order.clear();
    ArtifactJobWorker worker(Config(root), success_build);
    Require(worker.Submit(Request("fifo-1", "same-map")).accepted, "fifo-1 rejected");
    Require(worker.Submit(Request("fifo-2", "same-map")).accepted, "fifo-2 rejected");
    Require(!worker.Submit(Request("fifo-3", "other-map")).accepted,
            "bounded queue accepted too many active jobs");
    Require(WaitTerminal(worker, "fifo-1").state == ArtifactJobState::kSucceeded,
            "fifo-1 failed");
    Require(WaitTerminal(worker, "fifo-2").state == ArtifactJobState::kSucceeded,
            "fifo-2 failed");
    Require(build_order.size() == 2 && build_order[0] == "fifo-1" &&
                build_order[1] == "fifo-2",
            "worker did not process jobs FIFO for the same map");
  }

  {
    auto fail_once = [&](const ArtifactJobRequest& request, ArtifactJobContext& context) {
      context.UpdateProgress(0.4, "attempting");
      ArtifactBuildResult result;
      if (request.request_id == "retry-me") {
        const auto status_file = root / "retry_marker";
        if (!std::filesystem::exists(status_file)) {
          std::ofstream marker(status_file, std::ios::binary);
          marker << "failed";
          result.success = false;
          result.reason_code = "injected_failure";
          result.message = "first attempt failed";
          return result;
        }
      }
      result.success = true;
      result.message = "retry built";
      return result;
    };
    ArtifactJobWorker worker(Config(root), fail_once);
    Require(worker.Submit(Request("retry-me", "map-retry")).accepted, "retry submit rejected");
    auto failed = WaitTerminal(worker, "retry-me");
    Require(failed.state == ArtifactJobState::kFailed, "injected failure did not fail");
    Require(worker.Retry("retry-me").accepted, "retry was rejected");
    auto retried = WaitTerminal(worker, "retry-me");
    Require(retried.state == ArtifactJobState::kSucceeded, "retry did not succeed");
    Require(retried.attempt == 2, "retry attempt was not persisted");
  }

  {
    std::atomic<bool> entered{false};
    auto cancellable = [&](const ArtifactJobRequest&, ArtifactJobContext& context) {
      entered.store(true);
      while (!context.IsCancelRequested()) {
        context.Heartbeat("waiting for cancellation");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      ArtifactBuildResult result;
      result.success = false;
      result.reason_code = "cancelled";
      result.message = "cancelled by caller";
      return result;
    };
    ArtifactJobWorker worker(Config(root), cancellable);
    Require(worker.Submit(Request("cancel-running", "map-cancel")).accepted,
            "cancel-running submit rejected");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!entered.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Require(entered.load(), "cancellable job never entered build callback");
    Require(worker.Cancel("cancel-running").accepted, "running cancel rejected");
    const auto cancelled = WaitTerminal(worker, "cancel-running");
    Require(cancelled.state == ArtifactJobState::kCancelled, "running job did not cancel");
  }

  {
    ArtifactJobWorkerConfig config = Config(root);
    config.start_immediately = false;
    ArtifactJobWorker worker(config, success_build);
    Require(worker.Submit(Request("cancel-queued", "map-cancel")).accepted,
            "queued cancel submit rejected");
    const auto cancel = worker.Cancel("cancel-queued");
    Require(cancel.accepted && cancel.status.state == ArtifactJobState::kCancelled,
            "queued cancel was not immediate");
  }

  {
    ArtifactJobWorkerConfig config = Config(root);
    config.start_immediately = false;
    ArtifactJobWorker worker(config, success_build);
    Require(worker.Submit(Request("recover-expired", "map-recover")).accepted,
            "recover submit rejected");
    const auto state_file = config.journal_root / "recover-expired" / "job.state";
    ReplaceStateValue(state_file, "state", "RUNNING");
    ReplaceStateValue(state_file, "lease_expires_at_ns", "1");
  }
  {
    ArtifactJobWorker recovered(Config(root), success_build);
    const auto status = WaitTerminal(recovered, "recover-expired");
    Require(status.state == ArtifactJobState::kSucceeded,
            "expired lease recovery did not rerun queued job");
    Require(status.recovered, "recovered status did not expose recovery evidence");
  }

  {
    ArtifactJobWorkerConfig config = Config(root);
    config.start_immediately = false;
    ArtifactJobWorker worker(config, success_build);
    Require(worker.Submit(Request("recover-delayed", "map-recover-delayed")).accepted,
            "delayed recovery submit rejected");
    const auto state_file = config.journal_root / "recover-delayed" / "job.state";
    ReplaceStateValue(state_file, "state", "RUNNING");
    ReplaceStateValue(
        state_file,
        "lease_expires_at_ns",
        std::to_string(NowNs() + 50'000'000));
    ReplaceStateValue(state_file, "lease_owner", "dead-worker");
  }
  {
    ArtifactJobWorker recovered(Config(root), success_build);
    const auto status = WaitTerminal(recovered, "recover-delayed");
    Require(status.state == ArtifactJobState::kSucceeded,
            "delayed lease expiry did not recover a running job");
    Require(status.recovered, "delayed lease recovery lacks recovery evidence");
  }

  std::filesystem::remove_all(root);
  return 0;
}
