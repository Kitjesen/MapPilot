#pragma once

#include <filesystem>

#include "nav/inspection/inspection.hpp"
#include "status/status_snapshot_file_writer.hpp"

namespace lingtu::nav::endpoint {

class InspectionStatusFileWriter {
 public:
  explicit InspectionStatusFileWriter(std::filesystem::path data_dir);
  InspectionStatusFileWriter(std::filesystem::path data_dir, StatusSnapshotFileWriter::Sink sink);

  InspectionStatusFileWriter(const InspectionStatusFileWriter &) = delete;
  InspectionStatusFileWriter &operator=(const InspectionStatusFileWriter &) = delete;

  void submit(const inspection::RunStatus &status);
  void flush();
  StatusSnapshotWriterDiagnostics diagnostics() const;

 private:
  StatusSnapshotFileWriter writer_;
};

}  // namespace lingtu::nav::endpoint
