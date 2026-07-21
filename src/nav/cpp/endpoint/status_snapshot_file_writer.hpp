#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>

namespace lingtu::nav::endpoint {

struct StatusSnapshotWriterDiagnostics {
  std::uint64_t submitted{0};
  std::uint64_t written{0};
  std::uint64_t dropped{0};
  std::uint64_t failures{0};
  bool writing{false};
  bool pending{false};
};

class StatusSnapshotFileWriter {
 public:
  using Sink = std::function<bool(
      const std::filesystem::path&,
      const std::string&)>;

  explicit StatusSnapshotFileWriter(std::filesystem::path path);
  StatusSnapshotFileWriter(std::filesystem::path path, Sink sink);
  ~StatusSnapshotFileWriter();

  StatusSnapshotFileWriter(const StatusSnapshotFileWriter&) = delete;
  StatusSnapshotFileWriter& operator=(const StatusSnapshotFileWriter&) = delete;

  void submit(std::string snapshot);
  void flush();
  StatusSnapshotWriterDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::nav::endpoint
