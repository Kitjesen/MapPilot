#include "lingtu/maps/lock.hpp"

#include <cstdint>
#include <fstream>
#include <string>
#include <system_error>

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

std::uint64_t CurrentProcessIdValue() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint64_t>(getpid());
#endif
}

bool ProcessAlive(std::uint64_t pid) {
  if (pid == 0U)
    return false;
#if defined(_WIN32)
  HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
  if (process == nullptr)
    return false;
  DWORD exit_code = 0;
  const bool alive = GetExitCodeProcess(process, &exit_code) != 0 && exit_code == STILL_ACTIVE;
  CloseHandle(process);
  return alive;
#else
  if (kill(static_cast<pid_t>(pid), 0) == 0)
    return true;
  return errno == EPERM;
#endif
}

std::uint64_t ReadOwnerPid(const std::filesystem::path &path) {
  std::ifstream file(path / "owner.state", std::ios::binary);
  std::string line;
  while (std::getline(file, line)) {
    if (line.rfind("pid=", 0) != 0U)
      continue;
    try {
      return static_cast<std::uint64_t>(std::stoull(line.substr(4)));
    } catch (...) {
      return 0U;
    }
  }
  return 0U;
}

bool IncompleteOwnerIsRecent(const std::filesystem::path &path, std::chrono::seconds grace) {
  std::error_code error;
  const auto modified = std::filesystem::last_write_time(path, error);
  if (error)
    return true;
  return std::filesystem::file_time_type::clock::now() - modified <= grace;
}

}  // namespace

MapLock::MapLock(std::filesystem::path path) : path_(std::move(path)), owns_(true) {}

MapLock::MapLock(MapLock &&other) noexcept : path_(std::move(other.path_)), owns_(other.owns_) {
  other.owns_ = false;
}

MapLock &MapLock::operator=(MapLock &&other) noexcept {
  if (this == &other)
    return *this;
  Release();
  path_ = std::move(other.path_);
  owns_ = other.owns_;
  other.owns_ = false;
  return *this;
}

MapLock::~MapLock() {
  Release();
}

std::optional<MapLock> MapLock::TryAcquire(const std::filesystem::path &map_root,
                                           const std::string &map_id, const std::string &owner,
                                           std::chrono::seconds incomplete_owner_grace) {
  const auto locks_root = map_root / ".map_locks";
  const auto lock_path = locks_root / map_id;
  std::filesystem::create_directories(locks_root);
  for (int attempt = 0; attempt < 2; ++attempt) {
    std::error_code error;
    if (std::filesystem::create_directory(lock_path, error)) {
      std::ofstream state(lock_path / "owner.state", std::ios::binary | std::ios::trunc);
      if (!state) {
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      state << "owner=" << owner << "\n"
            << "pid=" << CurrentProcessIdValue() << "\n";
      state.flush();
      if (!state) {
        state.close();
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      return MapLock(lock_path);
    }

    const auto pid = ReadOwnerPid(lock_path);
    if (ProcessAlive(pid) ||
        (pid == 0U && IncompleteOwnerIsRecent(lock_path, incomplete_owner_grace))) {
      return std::nullopt;
    }
    std::filesystem::remove_all(lock_path, error);
    if (error)
      return std::nullopt;
  }
  return std::nullopt;
}

void MapLock::Release() noexcept {
  if (!owns_)
    return;
  std::error_code ignored;
  std::filesystem::remove_all(path_, ignored);
  owns_ = false;
}

}  // namespace lingtu::maps
