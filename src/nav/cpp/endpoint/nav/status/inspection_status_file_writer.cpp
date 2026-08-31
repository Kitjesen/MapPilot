#include "status/inspection_status_file_writer.hpp"

#include <utility>

#include "nav/inspection/store.hpp"

namespace lingtu::nav::endpoint {
namespace {

std::filesystem::path inspectionStatusPath(const std::filesystem::path &data_dir) {
  if (data_dir.empty())
    return {};
  return data_dir / "run_status.json";
}

}  // namespace

InspectionStatusFileWriter::InspectionStatusFileWriter(std::filesystem::path data_dir)
    : writer_(inspectionStatusPath(data_dir)) {}

InspectionStatusFileWriter::InspectionStatusFileWriter(std::filesystem::path data_dir,
                                                       StatusSnapshotFileWriter::Sink sink)
    : writer_(inspectionStatusPath(data_dir), std::move(sink)) {}

void InspectionStatusFileWriter::submit(const inspection::RunStatus &status) {
  writer_.submit(inspection::RunStatusToJson(status) + "\n");
}

void InspectionStatusFileWriter::flush() {
  writer_.flush();
}

StatusSnapshotWriterDiagnostics InspectionStatusFileWriter::diagnostics() const {
  return writer_.diagnostics();
}

}  // namespace lingtu::nav::endpoint
