#include "lingtu/maps/lock.hpp"

#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
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

enum class OwnerPidState {
  kIncomplete,
  kValid,
  kCorrupt,
};

struct OwnerPid {
  OwnerPidState state{OwnerPidState::kIncomplete};
  std::uint64_t pid{0U};
};

OwnerPid ReadOwnerPid(const std::filesystem::path &path) {
  std::ifstream file(path / "owner.state", std::ios::binary);
  if (!file) {
    return {};
  }
  std::string line;
  bool found = false;
  std::uint64_t pid = 0U;
  while (std::getline(file, line)) {
    if (line.rfind("pid=", 0) != 0U) {
      continue;
    }
    if (found) {
      return {OwnerPidState::kCorrupt, 0U};
    }
    const std::string value = line.substr(4);
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), pid);
    if (value.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != value.data() + value.size() || pid == 0U) {
      return {OwnerPidState::kCorrupt, 0U};
    }
    found = true;
  }
  if (file.bad()) {
    return {OwnerPidState::kCorrupt, 0U};
  }
  return found ? OwnerPid{OwnerPidState::kValid, pid} : OwnerPid{};
}

std::string NewNonce() {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto stamp = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  const std::uint64_t suffix =
      (CurrentProcessIdValue() << 32U) ^ sequence.fetch_add(1U, std::memory_order_relaxed);
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << stamp
      << std::setw(16) << suffix;
  return out.str();
}

bool OwnerNonceMatches(const std::filesystem::path &path, const std::string &expected) {
  std::ifstream file(path / "owner.state", std::ios::binary);
  if (!file) {
    return false;
  }
  std::string line;
  bool matched = false;
  while (std::getline(file, line)) {
    if (line.rfind("nonce=", 0) != 0U) {
      continue;
    }
    if (matched || line.substr(6) != expected) {
      return false;
    }
    matched = true;
  }
  return !file.bad() && matched;
}

bool IncompleteOwnerIsRecent(const std::filesystem::path &path, std::chrono::seconds grace) {
  std::error_code error;
  const auto modified = std::filesystem::last_write_time(path, error);
  if (error)
    return true;
  return std::filesystem::file_time_type::clock::now() - modified <= grace;
}

bool PersistentLockIsBusy(
    const std::filesystem::path &path,
    std::chrono::seconds incomplete_owner_grace,
    bool reclaim_stale) {
  const auto state = MapLock::InspectPersistentOwner(path, incomplete_owner_grace);
  if (state == PersistentOwnerState::kMissing) {
    return false;
  }
  if (state != PersistentOwnerState::kStale || !reclaim_stale) {
    return true;
  }
  std::error_code error;
  std::filesystem::remove_all(path, error);
  return static_cast<bool>(error);
}

}  // namespace

PersistentOwnerState MapLock::InspectPersistentOwner(
    const std::filesystem::path &path,
    std::chrono::seconds incomplete_owner_grace) {
  std::error_code error;
  if (!std::filesystem::exists(path, error)) {
    return error ? PersistentOwnerState::kInvalid : PersistentOwnerState::kMissing;
  }
  const auto owner = ReadOwnerPid(path);
  if (owner.state == OwnerPidState::kCorrupt) {
    return PersistentOwnerState::kInvalid;
  }
  if (owner.state == OwnerPidState::kValid) {
    return ProcessAlive(owner.pid) ? PersistentOwnerState::kActive
                                   : PersistentOwnerState::kStale;
  }
  return IncompleteOwnerIsRecent(path, incomplete_owner_grace)
      ? PersistentOwnerState::kActive
      : PersistentOwnerState::kStale;
}

MapLock::MapLock(std::filesystem::path path, std::string nonce)
    : path_(std::move(path)), nonce_(std::move(nonce)), owns_(true) {}

MapLock::MapLock(MapLock &&other) noexcept
    : path_(std::move(other.path_)), nonce_(std::move(other.nonce_)), owns_(other.owns_) {
  other.owns_ = false;
}

MapLock &MapLock::operator=(MapLock &&other) noexcept {
  if (this == &other)
    return *this;
  Release();
  path_ = std::move(other.path_);
  nonce_ = std::move(other.nonce_);
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
  const auto build_lock_path = map_root / map_id / ".build_lock";
  if (PersistentLockIsBusy(build_lock_path, incomplete_owner_grace, false)) {
    return std::nullopt;
  }
  const auto locks_root = map_root / ".map_locks";
  const auto lock_path = locks_root / map_id;
  std::filesystem::create_directories(locks_root);
  for (int attempt = 0; attempt < 2; ++attempt) {
    std::error_code error;
    if (std::filesystem::create_directory(lock_path, error)) {
      if (PersistentLockIsBusy(build_lock_path, incomplete_owner_grace, false)) {
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      const std::string nonce = NewNonce();
      std::ofstream state(lock_path / "owner.state", std::ios::binary | std::ios::trunc);
      if (!state) {
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      state << "owner=" << owner << "\n"
            << "pid=" << CurrentProcessIdValue() << "\n"
            << "nonce=" << nonce << "\n";
      state.flush();
      if (!state) {
        state.close();
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      return MapLock(lock_path, nonce);
    }

    if (PersistentLockIsBusy(lock_path, incomplete_owner_grace, true)) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<MapLock> MapLock::TryAcquireForBuildRecovery(
    const std::filesystem::path &map_root,
    const std::string &map_id,
    const std::string &owner,
    std::chrono::seconds incomplete_owner_grace) {
  const auto build_lock_path = map_root / map_id / ".build_lock";
  if (InspectPersistentOwner(build_lock_path, incomplete_owner_grace) !=
      PersistentOwnerState::kStale) {
    return std::nullopt;
  }

  const auto locks_root = map_root / ".map_locks";
  const auto lock_path = locks_root / map_id;
  std::filesystem::create_directories(locks_root);
  for (int attempt = 0; attempt < 2; ++attempt) {
    std::error_code error;
    if (std::filesystem::create_directory(lock_path, error)) {
      if (InspectPersistentOwner(build_lock_path, incomplete_owner_grace) !=
          PersistentOwnerState::kStale) {
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      const std::string nonce = NewNonce();
      std::ofstream state(lock_path / "owner.state", std::ios::binary | std::ios::trunc);
      if (!state) {
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      state << "owner=" << owner << "\n"
            << "pid=" << CurrentProcessIdValue() << "\n"
            << "nonce=" << nonce << "\n";
      state.flush();
      if (!state) {
        state.close();
        std::filesystem::remove_all(lock_path, error);
        return std::nullopt;
      }
      return MapLock(lock_path, nonce);
    }
    if (PersistentLockIsBusy(lock_path, incomplete_owner_grace, true)) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

void MapLock::Release() noexcept {
  if (!owns_)
    return;
  if (OwnerNonceMatches(path_, nonce_)) {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  owns_ = false;
}

}  // namespace lingtu::maps
