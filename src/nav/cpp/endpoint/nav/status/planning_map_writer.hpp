#pragma once

#include <memory>
#include <optional>
#include <string>

#include "input/active/octomap.hpp"
#include "nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp"
#include "status/status_snapshot_file_writer.hpp"

namespace lingtu::nav::endpoint {

// Diagnostic projection only. Its worker never owns or issues motion commands.
class PlanningMapWriter {
 public:
  PlanningMapWriter(std::string status_path, std::string product_session_id,
                    std::shared_ptr<ActiveOctomapGate> gate, std::string map_path,
                    lingtu::nav::plan::GlobalPlannerOptions options);
  void update(std::optional<lingtu::nav::plan::MapIdentity> identity,
              std::optional<double> reference_z, double stamp_s);
  void flush() { writer_.flush(); }

 private:
  std::string build(const std::optional<lingtu::nav::plan::MapIdentity> &identity,
                    std::optional<double> reference_z, double stamp_s);
  std::string product_session_id_;
  std::shared_ptr<ActiveOctomapGate> gate_;
  std::string map_path_;
  lingtu::nav::plan::GlobalPlannerOptions options_;
  // A separate query session avoids holding the navigation planner's mutex
  // while rendering a whole map. Both sessions run the exact same predicate.
  octoplanner3d::runtime::PlannerSession session_;
  octoplanner3d::runtime::PlanningMapProjection cached_;
  double cached_stamp_s_{0.0};
  StatusSnapshotFileWriter writer_;
};

}  // namespace lingtu::nav::endpoint
