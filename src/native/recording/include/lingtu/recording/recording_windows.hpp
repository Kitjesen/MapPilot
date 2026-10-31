#pragma once

#if !defined(_WIN32)
#error "Windows recording process supervision requires Windows"
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include "lingtu/recording/recording_manager.hpp"

namespace lingtu::recording {

class WindowsRecordingStopEvent {
 public:
  explicit WindowsRecordingStopEvent(const std::string &session_key = {});
  ~WindowsRecordingStopEvent();
  WindowsRecordingStopEvent(const WindowsRecordingStopEvent &) = delete;
  WindowsRecordingStopEvent &operator=(const WindowsRecordingStopEvent &) = delete;
  bool stop_requested() const;

 private:
  HANDLE event_{nullptr};
};

class WindowsRecordingProcessFactory final : public RecordingProcessFactory {
 public:
  std::unique_ptr<RecordingProcess> start(const RecordingProcessSpec &spec) override;
};

// The root controller relinquishes ownership after startup. The returned
// manager handle observes startup and closes without terminating the session.
std::unique_ptr<RecordingProcess> start_windows_recording_manager(
    const std::vector<std::string> &command, const std::filesystem::path &manager_log,
    const std::string &session_key);
bool windows_recording_process_alive(std::int64_t process_id);
std::optional<std::string> windows_recording_manager_identity_error(
    std::int64_t process_id, const std::filesystem::path &session_directory);
void request_windows_recording_stop(std::int64_t process_id, const std::string &session_key);

}  // namespace lingtu::recording
