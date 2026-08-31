#pragma once

#include <chrono>
#include <filesystem>
#include <system_error>
#include <thread>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace lingtu::native {

// Publish a complete snapshot atomically. If replacement remains unavailable,
// preserve the previous stable target and report failure to the caller.
inline bool replaceSnapshotFile(
    const std::filesystem::path& temporary,
    const std::filesystem::path& target,
    std::error_code* failure = nullptr) {
  std::error_code ec;
  constexpr int kMaxAttempts = 20;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
#if defined(_WIN32)
    if (::MoveFileExW(
            temporary.c_str(),
            target.c_str(),
            MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != FALSE) {
      ec.clear();
    } else {
      ec = std::error_code(
          static_cast<int>(::GetLastError()), std::system_category());
    }
#else
    ec.clear();
    std::filesystem::rename(temporary, target, ec);
#endif
    if (!ec) {
      if (failure) {
        failure->clear();
      }
      return true;
    }
    if (attempt + 1 < kMaxAttempts) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  if (failure) {
    *failure = ec;
  }
  return false;
}

}  // namespace lingtu::native
