#include "status/inspection_status_file_writer.hpp"

#include <utility>

#include "nav/inspection/store.hpp"

namespace lingtu::nav::endpoint {
namespace {

std::filesystem::path inspectionStatusPath(const std::filesystem::path &map_root) {
  if (map_root.empty())
    return {};
  return map_root / ".inspection" / "run_status.json";
}

}  // namespace

InspectionStatusFileWriter::InspectionStatusFileWriter(std::filesystem::path map_root)
    : writer_(inspectionStatusPath(map_root)) {}

InspectionStatusFileWriter::InspectionStatusFileWriter(std::filesystem::path map_root,
                                                       StatusSnapshotFileWriter::Sink sink)
    : writer_(inspectionStatusPath(map_root), std::move(sink)) {}

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
