#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "command/ingress.hpp"
#include "input/frame.hpp"
#include "nav_kernel/types.hpp"
#include "runtime/rolling/contract.hpp"
#include "safety/geofence.hpp"

namespace lingtu::nav::endpoint {

struct GoalSample {
  double stamp_s{0.0};
  std::string frame_id;
  nav_kernel::Vec3 position{};
  Quaternion orientation{};
  bool has_orientation{false};
};

struct NavigationCommandSample {
  CommandIngressRequest ingress;
  GoalSample goal;
};

struct PlanPreviewRequest {
  double stamp_s{0.0};
  std::string frame_id;
  std::string request_id;
  nav_kernel::Vec3 goal{};
};

struct OperatorMotionControlSample {
  std::string source_id;
  std::uint64_t source_epoch{0U};
  std::uint64_t source_sequence{0U};
  std::string request_id;
  std::int32_t action{0};
  std::uint32_t lease_ttl_ms{0U};
};

struct OperatorMotionInputSample {
  double stamp_s{0.0};
  std::string frame_id;
  std::string source_id;
  std::uint64_t source_epoch{0U};
  std::uint64_t source_sequence{0U};
  std::uint64_t source_stamp_ns{0U};
  std::uint32_t freshness_budget_ms{0U};
  bool deadman{false};
  bool manual_mode{false};
  nav_kernel::Twist velocity{};
};

struct InspectionCommandRequest {
  std::string task_id;
  std::string request_id;
  std::int32_t raw_kind{0};
  std::string route_id;
  std::uint64_t route_revision{0U};
  std::string reason;
};

struct InspectionEvidenceResultSample {
  std::string request_id;
  bool persisted{false};
  std::string evidence_id;
  std::string reason;
};

struct GeofenceCommandView {
  std::string request_id;
  std::int32_t action{0};
  std::string name;
  std::vector<GeofencePoint> polygon;
};

using CommandEvent =
    std::variant<OperatorMotionControlSample, OperatorMotionInputSample,
                 NavigationCommandSample, PlanPreviewRequest, InspectionCommandRequest,
                 GeofenceCommandView, RollingSegmentCommand, InspectionEvidenceResultSample>;

struct CommandBatch {
  double receive_steady_s{0.0};
  double receive_wall_s{0.0};
  std::vector<CommandEvent> ordered;
};

}  // namespace lingtu::nav::endpoint
