#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "native/snapshot_file.hpp"
#include "status/status_snapshot_file_writer.hpp"

namespace {

using Writer = lingtu::nav::endpoint::StatusSnapshotFileWriter;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testSubmitDoesNotBlockAndPendingSnapshotIsLatestWins() {
  std::mutex mutex;
  std::condition_variable cv;
  bool first_started = false;
  bool release_first = false;
  std::vector<std::string> writes;

  Writer writer("ignored.json", [&](const std::filesystem::path &, const std::string &snapshot) {
    std::unique_lock<std::mutex> lock(mutex);
    writes.push_back(snapshot);
    if (writes.size() == 1) {
      first_started = true;
      cv.notify_all();
      cv.wait(lock, [&]() { return release_first; });
    }
    return true;
  });

  writer.submit("first");
  {
    std::unique_lock<std::mutex> lock(mutex);
    require(cv.wait_for(lock, std::chrono::seconds(2), [&]() { return first_started; }),
            "the background sink must start within two seconds");
  }

  const auto submit_start = std::chrono::steady_clock::now();
  writer.submit("second");
  writer.submit("third");
  const auto submit_elapsed =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - submit_start);
  require(submit_elapsed.count() < 50.0, "submit must not block behind a slow snapshot sink");

  {
    std::lock_guard<std::mutex> lock(mutex);
    release_first = true;
  }
  cv.notify_all();
  writer.flush();

  require(writes.size() == 2, "latest-wins must publish exactly two snapshots");
  require(writes[0] == "first", "the in-flight snapshot must complete");
  require(writes[1] == "third", "only the latest pending snapshot must survive");
  const auto diagnostics = writer.diagnostics();
  require(diagnostics.submitted == 3, "submitted diagnostics must count all snapshots");
  require(diagnostics.written == 2, "written diagnostics must count completed snapshots");
  require(diagnostics.dropped == 1, "dropped diagnostics must count replaced pending snapshots");
  require(diagnostics.failures == 0, "successful sinks must not report failures");
}

void testSnapshotFactoryRunsOnWorkerThread() {
  std::mutex mutex;
  std::condition_variable cv;
  bool factory_started = false;
  bool release_factory = false;
  std::thread::id submit_thread_id;
  std::thread::id factory_thread_id;
  std::vector<std::string> writes;

  Writer writer("ignored.json", [&](const std::filesystem::path &, const std::string &snapshot) {
    std::lock_guard<std::mutex> lock(mutex);
    writes.push_back(snapshot);
    return true;
  });

  submit_thread_id = std::this_thread::get_id();
  writer.submitFactory([&]() {
    std::unique_lock<std::mutex> lock(mutex);
    factory_started = true;
    factory_thread_id = std::this_thread::get_id();
    cv.notify_all();
    cv.wait(lock, [&]() { return release_factory; });
    return std::string("from-worker");
  });

  {
    std::unique_lock<std::mutex> lock(mutex);
    require(cv.wait_for(lock, std::chrono::seconds(2), [&]() { return factory_started; }),
            "snapshot factory must start on the background worker");
    require(factory_thread_id != submit_thread_id,
            "submitFactory must not run JSON rendering on the caller thread");
    require(writes.empty(), "sink must wait for the factory result");
    release_factory = true;
  }
  cv.notify_all();
  writer.flush();

  require(writes.size() == 1, "factory snapshot must be written once");
  require(writes[0] == "from-worker", "factory result must be passed to the sink");
  const auto diagnostics = writer.diagnostics();
  require(diagnostics.submitted == 1, "factory submit must increment submitted diagnostics");
  require(diagnostics.written == 1, "factory submit must increment written diagnostics");
  require(diagnostics.failures == 0, "successful factory submit must not report failures");
}

void testSnapshotFactoryFailureIsObservable() {
  Writer writer("ignored.json",
                [](const std::filesystem::path &, const std::string &) { return true; });

  writer.submitFactory([]() -> std::string { throw std::runtime_error("render failed"); });
  writer.flush();

  const auto diagnostics = writer.diagnostics();
  require(diagnostics.submitted == 1, "failed factory must count as submitted");
  require(diagnostics.written == 0, "failed factory must not count as written");
  require(diagnostics.failures == 1, "failed factory must increment failure diagnostics");
  require(!diagnostics.pending, "failed factory must clear the pending state");
  require(!diagnostics.writing, "failed factory must clear the writing state");
}

void testDefaultSinkPublishesCompleteSnapshot() {
  const auto path =
      std::filesystem::temp_directory_path() / "lingtu_status_snapshot_file_writer_test.json";
  std::error_code ec;
  std::filesystem::remove(path, ec);

  {
    Writer writer(path);
    writer.submit("{\"ready\":true}\n");
    writer.flush();
  }

  std::ifstream input(path, std::ios::binary);
  const std::string value((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());
  require(value == "{\"ready\":true}\n", "default sink must atomically publish the full snapshot");
  std::filesystem::remove(path, ec);
}

void testDefaultSinkNeverExposesPartialSnapshotDuringReplacement() {
  const auto path = std::filesystem::temp_directory_path() /
                    "lingtu_status_snapshot_atomic_replacement_test.json";
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path.string() + ".tmp", ec);

  const std::string first = "{\"generation\":1,\"padding\":\"" +
                            std::string(512 * 1024, 'a') + "\"}\n";
  const std::string second = "{\"generation\":2,\"padding\":\"" +
                             std::string(512 * 1024, 'b') + "\"}\n";

  Writer writer(path);
  writer.submit(first);
  writer.flush();

  std::atomic<bool> stop_reader{false};
  std::atomic<bool> invalid_snapshot_seen{false};
  std::thread reader([&]() {
    while (!stop_reader.load(std::memory_order_relaxed)) {
      std::ifstream input(path, std::ios::binary);
      if (!input) {
        continue;
      }
      const std::string value((std::istreambuf_iterator<char>(input)),
                              std::istreambuf_iterator<char>());
      if (value != first && value != second) {
        invalid_snapshot_seen.store(true, std::memory_order_relaxed);
        return;
      }
      input.close();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  for (int generation = 0; generation < 32; ++generation) {
    writer.submit((generation % 2 == 0) ? second : first);
    writer.flush();
  }
  stop_reader.store(true, std::memory_order_relaxed);
  reader.join();

  require(!invalid_snapshot_seen.load(std::memory_order_relaxed),
          "readers must observe only a complete old or complete new snapshot");
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path.string() + ".tmp", ec);
}

void testFailedReplacementPreservesPreviousStableSnapshot() {
  const auto target = std::filesystem::temp_directory_path() /
                      "lingtu_status_snapshot_failed_replacement_test.json";
  const auto missing_temporary = target.string() + ".missing";
  std::error_code ec;
  std::filesystem::remove(target, ec);
  std::filesystem::remove(missing_temporary, ec);

  const std::string stable = "{\"generation\":1}\n";
  {
    std::ofstream output(target, std::ios::binary | std::ios::trunc);
    output << stable;
  }

  std::error_code replace_error;
  require(!lingtu::native::replaceSnapshotFile(
              missing_temporary, target, &replace_error),
          "replacement without a complete temporary snapshot must fail");
  require(static_cast<bool>(replace_error),
          "failed replacement must report its operating-system error");

  std::ifstream input(target, std::ios::binary);
  const std::string value((std::istreambuf_iterator<char>(input)),
                          std::istreambuf_iterator<char>());
  require(value == stable,
          "failed replacement must preserve the previous stable snapshot");
  std::filesystem::remove(target, ec);
}

}  // namespace

int main() {
  testSubmitDoesNotBlockAndPendingSnapshotIsLatestWins();
  testSnapshotFactoryRunsOnWorkerThread();
  testSnapshotFactoryFailureIsObservable();
  testDefaultSinkPublishesCompleteSnapshot();
  testDefaultSinkNeverExposesPartialSnapshotDuringReplacement();
  testFailedReplacementPreservesPreviousStableSnapshot();
  return 0;
}
