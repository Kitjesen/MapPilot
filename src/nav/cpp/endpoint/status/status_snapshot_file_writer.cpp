#include "status/status_snapshot_file_writer.hpp"

#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "message/cpp/snapshot_file.hpp"

namespace lingtu::nav::endpoint {
namespace {

bool writeSnapshotFile(const std::filesystem::path &path, const std::string &snapshot) {
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code directory_error;
    std::filesystem::create_directories(parent, directory_error);
  }

  const std::filesystem::path temporary = path.string() + ".tmp";
  std::ofstream output(temporary, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!output) {
    std::fprintf(stderr, "nav_native: failed to open status snapshot %s\n",
                 temporary.string().c_str());
    return false;
  }
  output.write(snapshot.data(), static_cast<std::streamsize>(snapshot.size()));
  output.close();
  if (!output) {
    std::fprintf(stderr, "nav_native: failed to write status snapshot %s\n",
                 temporary.string().c_str());
    return false;
  }

  std::error_code replace_error;
  if (!lingtu::message::replaceSnapshotFile(temporary, path, &replace_error)) {
    std::fprintf(stderr, "nav_native: failed to replace status snapshot %s: %s\n",
                 path.string().c_str(), replace_error.message().c_str());
    return false;
  }
  return true;
}

}  // namespace

struct StatusSnapshotFileWriter::Impl {
  Impl(std::filesystem::path target_path, Sink target_sink)
      : path(std::move(target_path)), sink(std::move(target_sink)) {
    if (!path.empty()) {
      worker = std::thread([this]() { run(); });
    }
  }

  ~Impl() {
    if (!worker.joinable())
      return;
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
    }
    work_ready.notify_all();
    worker.join();
  }

  void submit(std::string snapshot) {
    if (!worker.joinable())
      return;
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++stats.submitted;
      if (pending.has_value()) {
        ++stats.dropped;
      }
      pending = std::move(snapshot);
      stats.pending = true;
    }
    work_ready.notify_one();
  }

  void flush() {
    if (!worker.joinable())
      return;
    std::unique_lock<std::mutex> lock(mutex);
    idle.wait(lock, [this]() { return !pending.has_value() && !stats.writing; });
  }

  StatusSnapshotWriterDiagnostics diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex);
    return stats;
  }

  void run() noexcept {
    std::unique_lock<std::mutex> lock(mutex);
    while (true) {
      work_ready.wait(lock, [this]() { return stopping || pending.has_value(); });
      if (!pending.has_value() && stopping)
        break;

      std::string snapshot = std::move(*pending);
      pending.reset();
      stats.pending = false;
      stats.writing = true;
      lock.unlock();

      bool ok = false;
      try {
        ok = sink(path, snapshot);
      } catch (...) {
        ok = false;
      }

      lock.lock();
      if (ok) {
        ++stats.written;
      } else {
        ++stats.failures;
      }
      stats.writing = false;
      idle.notify_all();
    }
    idle.notify_all();
  }

  std::filesystem::path path;
  Sink sink;
  mutable std::mutex mutex;
  std::condition_variable work_ready;
  std::condition_variable idle;
  std::optional<std::string> pending;
  StatusSnapshotWriterDiagnostics stats;
  bool stopping{false};
  std::thread worker;
};

StatusSnapshotFileWriter::StatusSnapshotFileWriter(std::filesystem::path path)
    : StatusSnapshotFileWriter(std::move(path), writeSnapshotFile) {}

StatusSnapshotFileWriter::StatusSnapshotFileWriter(std::filesystem::path path, Sink sink)
    : impl_(std::make_unique<Impl>(std::move(path), std::move(sink))) {}

StatusSnapshotFileWriter::~StatusSnapshotFileWriter() = default;

void StatusSnapshotFileWriter::submit(std::string snapshot) {
  impl_->submit(std::move(snapshot));
}

void StatusSnapshotFileWriter::flush() {
  impl_->flush();
}

StatusSnapshotWriterDiagnostics StatusSnapshotFileWriter::diagnostics() const {
  return impl_->diagnostics();
}

}  // namespace lingtu::nav::endpoint
