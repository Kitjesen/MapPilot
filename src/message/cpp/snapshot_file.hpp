#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <thread>

namespace lingtu::message {

// DrvFs can temporarily reject rename-over-open while a Windows process reads
// a WSL-produced snapshot. Retry the atomic replace, then fall back to a
// bounded direct copy so diagnostics do not remain permanently stale.
inline bool replaceSnapshotFile(
    const std::filesystem::path& temporary,
    const std::filesystem::path& target,
    std::error_code* failure = nullptr) {
  std::error_code ec;
  for (int attempt = 0; attempt < 8; ++attempt) {
    ec.clear();
    std::filesystem::rename(temporary, target, ec);
    if (!ec) {
      if (failure) {
        failure->clear();
      }
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  for (int attempt = 0; attempt < 8; ++attempt) {
    std::ifstream source(temporary, std::ios::in | std::ios::binary);
    std::ofstream destination(
        target,
        std::ios::out | std::ios::binary | std::ios::trunc);
    if (source && destination) {
      destination << source.rdbuf();
      destination.flush();
      if (source.good() || source.eof()) {
        if (destination.good()) {
          destination.close();
          source.close();
          std::error_code remove_ec;
          std::filesystem::remove(temporary, remove_ec);
          if (failure) {
            failure->clear();
          }
          return true;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (failure) {
    *failure = ec;
    if (!*failure) {
      *failure = std::make_error_code(std::errc::io_error);
    }
  }
  return false;
}

}  // namespace lingtu::message
