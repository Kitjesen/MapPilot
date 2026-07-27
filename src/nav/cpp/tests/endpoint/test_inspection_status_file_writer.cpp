#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav/inspection/store.hpp"
#include "status/inspection_status_file_writer.hpp"

namespace {

using Writer = lingtu::nav::endpoint::InspectionStatusFileWriter;
namespace inspection = lingtu::nav::inspection;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testEmptyMapRootDisablesPersistence() {
  std::size_t sink_calls = 0;
  Writer writer({}, [&](const std::filesystem::path &, const std::string &) {
    ++sink_calls;
    return true;
  });

  inspection::RunStatus status;
  status.run_id = "disabled-run";
  writer.submit(status);
  writer.flush();

  const auto diagnostics = writer.diagnostics();
  require(sink_calls == 0, "empty map root must disable the sink");
  require(diagnostics.submitted == 0, "disabled writer must not count submissions");
  require(diagnostics.written == 0, "disabled writer must not report writes");
  require(!diagnostics.writing, "disabled writer must remain idle");
  require(!diagnostics.pending, "disabled writer must have no pending snapshot");
}

void testSubmitIsNonBlockingAndPendingStatusIsLatestWins() {
  std::mutex mutex;
  std::condition_variable cv;
  bool first_started = false;
  bool release_first = false;
  std::vector<std::filesystem::path> paths;
  std::vector<std::string> writes;

  Writer writer("map-root", [&](const std::filesystem::path &path, const std::string &snapshot) {
    std::unique_lock<std::mutex> lock(mutex);
    paths.push_back(path);
    writes.push_back(snapshot);
    if (writes.size() == 1U) {
      first_started = true;
      cv.notify_all();
      cv.wait(lock, [&]() { return release_first; });
    }
    return true;
  });

  inspection::RunStatus first;
  first.run_id = "first";
  writer.submit(first);
  {
    std::unique_lock<std::mutex> lock(mutex);
    require(cv.wait_for(lock, std::chrono::seconds(2), [&]() { return first_started; }),
            "the background sink must start within two seconds");
  }

  inspection::RunStatus second;
  second.run_id = "second";
  inspection::RunStatus third;
  third.run_id = "third";
  const auto submit_start = std::chrono::steady_clock::now();
  writer.submit(second);
  writer.submit(third);
  const auto submit_elapsed =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - submit_start);
  require(submit_elapsed.count() < 50.0,
          "submit must not block behind a slow inspection status sink");

  {
    std::lock_guard<std::mutex> lock(mutex);
    release_first = true;
  }
  cv.notify_all();
  writer.flush();

  require(writes.size() == 2U, "latest-wins must write exactly two statuses");
  require(paths[0] == std::filesystem::path("map-root") / ".inspection" / "run_status.json",
          "the writer must target the inspection run-status path");
  require(writes[0] == inspection::RunStatusToJson(first) + "\n",
          "the in-flight status must complete with one trailing newline");
  require(writes[1] == inspection::RunStatusToJson(third) + "\n",
          "only the latest pending status must survive");

  const auto diagnostics = writer.diagnostics();
  require(diagnostics.submitted == 3, "submitted must count every status");
  require(diagnostics.written == 2, "written must count completed statuses");
  require(diagnostics.dropped == 1, "dropped must count the replaced status");
  require(diagnostics.failures == 0, "successful writes must not fail");
}

void testFlushPublishesExactStatusSnapshot() {
  const auto unique = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto root =
      std::filesystem::temp_directory_path() / ("lingtu_inspection_status_file_writer_" + unique);
  std::error_code error;
  std::filesystem::remove_all(root, error);

  inspection::RunStatus status;
  status.state = inspection::RunState::kNavigating;
  status.run_id = "run-exact";
  status.route_id = "route-exact";
  status.point_index = 4;
  status.reason = "persist this snapshot";

  {
    Writer writer(root);
    writer.submit(status);
    writer.flush();

    const auto path = root / ".inspection" / "run_status.json";
    std::ifstream input(path, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(input)),
                               std::istreambuf_iterator<char>());
    require(contents == inspection::RunStatusToJson(status) + "\n",
            "flush must publish the exact serialized status plus one newline");
  }

  std::filesystem::remove_all(root, error);
}

void testSinkFailuresAreReported() {
  inspection::RunStatus status;
  status.run_id = "failed-run";

  {
    Writer writer("map-root",
                  [](const std::filesystem::path &, const std::string &) { return false; });
    writer.submit(status);
    writer.flush();

    const auto diagnostics = writer.diagnostics();
    require(diagnostics.submitted == 1, "false sink must count its submission");
    require(diagnostics.written == 0, "false sink must not count a write");
    require(diagnostics.failures == 1, "false sink must count one failure");
  }

  {
    Writer writer("map-root", [](const std::filesystem::path &, const std::string &) -> bool {
      throw std::runtime_error("sink failed");
    });
    writer.submit(status);
    writer.flush();

    const auto diagnostics = writer.diagnostics();
    require(diagnostics.submitted == 1, "throwing sink must count its submission");
    require(diagnostics.written == 0, "throwing sink must not count a write");
    require(diagnostics.failures == 1, "throwing sink must count one failure");
  }
}
}  // namespace

int main() {
  testEmptyMapRootDisablesPersistence();
  testSubmitIsNonBlockingAndPendingStatusIsLatestWins();
  testFlushPublishesExactStatusSnapshot();
  testSinkFailuresAreReported();
  return 0;
}
