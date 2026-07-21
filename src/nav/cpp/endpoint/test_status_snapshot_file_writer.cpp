#include "status_snapshot_file_writer.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace {

using Writer = lingtu::nav::endpoint::StatusSnapshotFileWriter;

void require(bool condition, const char* message) {
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

  Writer writer(
      "ignored.json",
      [&](const std::filesystem::path&, const std::string& snapshot) {
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
    require(cv.wait_for(
        lock,
        std::chrono::seconds(2),
        [&]() { return first_started; }),
        "the background sink must start within two seconds");
  }

  const auto submit_start = std::chrono::steady_clock::now();
  writer.submit("second");
  writer.submit("third");
  const auto submit_elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - submit_start);
  require(
      submit_elapsed.count() < 50.0,
      "submit must not block behind a slow snapshot sink");

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

void testDefaultSinkPublishesCompleteSnapshot() {
  const auto path = std::filesystem::temp_directory_path() /
      "lingtu_status_snapshot_file_writer_test.json";
  std::error_code ec;
  std::filesystem::remove(path, ec);

  {
    Writer writer(path);
    writer.submit("{\"ready\":true}\n");
    writer.flush();
  }

  std::ifstream input(path, std::ios::binary);
  const std::string value(
      (std::istreambuf_iterator<char>(input)),
      std::istreambuf_iterator<char>());
  require(value == "{\"ready\":true}\n", "default sink must atomically publish the full snapshot");
  std::filesystem::remove(path, ec);
}

}  // namespace

int main() {
  testSubmitDoesNotBlockAndPendingSnapshotIsLatestWins();
  testDefaultSinkPublishesCompleteSnapshot();
  return 0;
}
