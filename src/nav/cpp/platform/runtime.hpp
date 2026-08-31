#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#if defined(_WIN32)
#define NOMINMAX
#include <Windows.h>
#include <process.h>
#elif defined(__linux__)
#include <ctime>
#include <unistd.h>
#else
#include <unistd.h>
#endif

namespace lingtu::nav::platform {

// Keeps periodic control loops on one portable clock contract. On Windows,
// std::this_thread::sleep_until may inherit the coarse system timer and wake
// tens of milliseconds late. A high-resolution waitable timer avoids making
// every endpoint invent its own platform-specific pacing workaround.
class DeadlineSleeper {
 public:
  DeadlineSleeper() {
#if defined(_WIN32)
    constexpr DWORD kHighResolutionTimer = 0x00000002UL;
    timer_ = ::CreateWaitableTimerExW(nullptr, nullptr, kHighResolutionTimer,
                                      TIMER_MODIFY_STATE | SYNCHRONIZE);
    if (timer_ == nullptr) {
      timer_ = ::CreateWaitableTimerW(nullptr, FALSE, nullptr);
    }
#endif
  }

  ~DeadlineSleeper() {
#if defined(_WIN32)
    if (timer_ != nullptr) {
      ::CloseHandle(timer_);
    }
#endif
  }

  DeadlineSleeper(const DeadlineSleeper &) = delete;
  DeadlineSleeper &operator=(const DeadlineSleeper &) = delete;

  void sleepUntil(std::chrono::steady_clock::time_point deadline) const {
    const auto now = std::chrono::steady_clock::now();
    if (deadline <= now) {
      return;
    }
#if defined(_WIN32)
    if (timer_ != nullptr) {
      const auto remaining =
          std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count();
      LARGE_INTEGER due{};
      // Negative values are relative intervals in 100 ns units. Round up so
      // truncation never requests an earlier wake-up than the deadline.
      due.QuadPart = -static_cast<LONGLONG>(
          std::max<std::int64_t>(1, (static_cast<std::int64_t>(remaining) + 99) / 100));
      if (::SetWaitableTimer(timer_, &due, 0, nullptr, nullptr, FALSE) != 0 &&
          ::WaitForSingleObject(timer_, INFINITE) == WAIT_OBJECT_0) {
        return;
      }
    }
#endif
    std::this_thread::sleep_until(deadline);
  }

 private:
#if defined(_WIN32)
  HANDLE timer_{nullptr};
#endif
};

inline std::uint64_t processId() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(::_getpid());
#else
  return static_cast<std::uint64_t>(::getpid());
#endif
}

inline std::uint64_t bootTimeNanoseconds() {
#if defined(_WIN32)
  LARGE_INTEGER counter{};
  LARGE_INTEGER frequency{};
  if (::QueryPerformanceCounter(&counter) == 0 || ::QueryPerformanceFrequency(&frequency) == 0 ||
      counter.QuadPart < 0 || frequency.QuadPart <= 0) {
    throw std::runtime_error("QueryPerformanceCounter failed");
  }
  const auto ticks = static_cast<std::uint64_t>(counter.QuadPart);
  const auto ticks_per_second = static_cast<std::uint64_t>(frequency.QuadPart);
  const auto seconds = ticks / ticks_per_second;
  const auto remainder = ticks % ticks_per_second;
  return seconds * 1'000'000'000ULL + remainder * 1'000'000'000ULL / ticks_per_second;
#elif defined(__linux__)
  timespec value{};
  if (::clock_gettime(CLOCK_BOOTTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_BOOTTIME) failed");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
#else
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
#endif
}

inline bool validBootId(const std::string &value) {
  if (value.empty() || value.size() > 128U) {
    return false;
  }
  for (const unsigned char character : value) {
    const bool alphanumeric = (character >= 'a' && character <= 'z') ||
                              (character >= 'A' && character <= 'Z') ||
                              (character >= '0' && character <= '9');
    if (!alphanumeric && character != '-' && character != '_' && character != '.' &&
        character != ':') {
      return false;
    }
  }
  return true;
}

inline std::string configuredHostBootId() {
#if defined(_WIN32)
  char *value = nullptr;
  std::size_t size = 0U;
  if (::_dupenv_s(&value, &size, "LINGTU_HOST_BOOT_ID") != 0) {
    throw std::runtime_error("failed to read LINGTU_HOST_BOOT_ID");
  }
  std::string result = value == nullptr ? std::string{} : std::string{value};
  std::free(value);
  return result;
#else
  const char *value = std::getenv("LINGTU_HOST_BOOT_ID");
  return value == nullptr ? std::string{} : std::string{value};
#endif
}

inline std::string hostBootId() {
  if (std::string value = configuredHostBootId(); !value.empty()) {
    if (!validBootId(value)) {
      throw std::runtime_error("LINGTU_HOST_BOOT_ID is invalid");
    }
    return value;
  }
#if defined(__linux__)
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string value;
  std::getline(input, value);
  if (!validBootId(value)) {
    throw std::runtime_error("failed to read Linux host boot id");
  }
  return value;
#else
  throw std::runtime_error("LINGTU_HOST_BOOT_ID is required on this platform");
#endif
}

inline std::string producerBootId(const std::string &host_boot_id,
                                  std::uint64_t start_boot_time_ns) {
  if (!validBootId(host_boot_id) || start_boot_time_ns == 0U) {
    throw std::runtime_error("producer boot identity is invalid");
  }
  std::string result =
      host_boot_id + ":" + std::to_string(processId()) + ":" + std::to_string(start_boot_time_ns);
  // C clients expose boot identities through 128-byte fixed buffers. Reject
  // an oversized configured host identity instead of silently truncating it.
  if (result.size() >= 128U) {
    throw std::runtime_error("producer boot identity exceeds 127 bytes");
  }
  return result;
}

}  // namespace lingtu::nav::platform
