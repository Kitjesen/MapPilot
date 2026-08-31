#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <windows.h>
#else
#include <cerrno>
#include <poll.h>
#include <unistd.h>
#endif

#include "../camera_record.hpp"

namespace {

namespace record = lingtu::drivers::camera::record;

int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (condition) {
    return;
  }
  std::cerr << "camera record deadline test failed at line " << line << ": "
            << expression << '\n';
  ++failures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

struct PipePair {
  int reader{-1};
  int writer{-1};
};

PipePair makePipe() {
  int descriptors[2] = {-1, -1};
#ifdef _WIN32
  if (_pipe(descriptors, 4096, _O_BINARY) != 0) {
    throw std::runtime_error("_pipe failed");
  }
#else
  if (::pipe(descriptors) != 0) {
    throw std::runtime_error("pipe failed");
  }
#endif
  return {descriptors[0], descriptors[1]};
}

void closeFd(int descriptor) {
  if (descriptor < 0) {
    return;
  }
#ifdef _WIN32
  _close(descriptor);
#else
  ::close(descriptor);
#endif
}

std::ptrdiff_t readSome(int descriptor, void* output, std::size_t size) {
#ifdef _WIN32
  const int count = _read(descriptor, output, static_cast<unsigned int>(size));
#else
  const ssize_t count = ::read(descriptor, output, size);
#endif
  if (count >= 0) {
    return static_cast<std::ptrdiff_t>(count);
  }
  if (errno == EINTR) {
    return -1;
  }
  throw std::runtime_error("pipe read failed");
}

bool writeByte(int descriptor, std::uint8_t value) {
#ifdef _WIN32
  return _write(descriptor, &value, 1) == 1;
#else
  return ::write(descriptor, &value, 1) == 1;
#endif
}

record::RecordWaitResult waitReadable(int descriptor, int timeout_ms) {
#ifdef _WIN32
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  const auto handle = reinterpret_cast<HANDLE>(_get_osfhandle(descriptor));
  while (std::chrono::steady_clock::now() < deadline) {
    DWORD available = 0;
    if (PeekNamedPipe(handle, nullptr, 0, nullptr, &available, nullptr) == 0) {
      if (GetLastError() == ERROR_BROKEN_PIPE) {
        return record::RecordWaitResult::kEndOfStream;
      }
      throw std::runtime_error("PeekNamedPipe failed");
    }
    if (available > 0) {
      return record::RecordWaitResult::kReady;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return record::RecordWaitResult::kTimeout;
#else
  pollfd poll_descriptor{};
  poll_descriptor.fd = descriptor;
  poll_descriptor.events = POLLIN;
  const int result = ::poll(&poll_descriptor, 1, timeout_ms);
  if (result == 0) {
    return record::RecordWaitResult::kTimeout;
  }
  if (result < 0) {
    if (errno == EINTR) {
      return record::RecordWaitResult::kRetry;
    }
    throw std::runtime_error("poll failed");
  }
  if ((poll_descriptor.revents & (POLLERR | POLLNVAL)) != 0) {
    throw std::runtime_error("pipe poll error");
  }
  if ((poll_descriptor.revents & POLLHUP) != 0 &&
      (poll_descriptor.revents & POLLIN) == 0) {
    return record::RecordWaitResult::kEndOfStream;
  }
  return record::RecordWaitResult::kReady;
#endif
}

record::RecordReadResult readExact(
    int descriptor,
    void* output,
    std::size_t size,
    record::RecordDeadline deadline) {
  return record::readExactUntil(
      output,
      size,
      deadline,
      [&](int timeout_ms) { return waitReadable(descriptor, timeout_ms); },
      [&](void* destination, std::size_t remaining) {
        return readSome(descriptor, destination, remaining);
      });
}

void testCompletePipeRead() {
  PipePair pipe = makePipe();
  const std::array<std::uint8_t, 8> expected{{0, 1, 2, 3, 4, 5, 6, 7}};
  for (const auto value : expected) {
    CHECK(writeByte(pipe.writer, value));
  }
  closeFd(pipe.writer);
  pipe.writer = -1;

  std::array<std::uint8_t, 8> actual{};
  const auto result = readExact(
      pipe.reader,
      actual.data(),
      actual.size(),
      record::makeRecordDeadline(100));
  CHECK(result == record::RecordReadResult::kComplete);
  CHECK(actual == expected);
  closeFd(pipe.reader);
}

void testTrickleBytesCannotExtendRecordDeadline() {
  PipePair pipe = makePipe();
  std::atomic_bool stop{false};
  std::thread writer([&] {
    for (std::uint8_t value = 0; value < 32 && !stop.load(); ++value) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (stop.load() || !writeByte(pipe.writer, value)) {
        break;
      }
    }
  });

  std::array<std::uint8_t, 32> output{};
  const auto started = std::chrono::steady_clock::now();
  const auto result = readExact(
      pipe.reader,
      output.data(),
      output.size(),
      record::makeRecordDeadline(75));
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started);
  stop.store(true);
  writer.join();

  CHECK(result == record::RecordReadResult::kTimeout);
  CHECK(elapsed >= std::chrono::milliseconds(45));
  CHECK(elapsed < std::chrono::milliseconds(200));
  closeFd(pipe.reader);
  closeFd(pipe.writer);
}

}  // namespace

int main() {
  testCompletePipeRead();
  testTrickleBytesCannotExtendRecordDeadline();
  return failures == 0 ? 0 : 1;
}