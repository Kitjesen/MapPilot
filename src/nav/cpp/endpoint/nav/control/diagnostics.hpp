#pragma once

#include "navigation/executor.hpp"
#include "safety/command.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

// Projects one native Executor result into the status shape shared by the
// autonomy and teleop control loops.  The control loops may then override the
// few mode-specific fields (for example, teleop's final command) without
// duplicating the common recovery and tracking mapping.
LocalDiagnostics makeLocalDiagnostics(
    const lingtu::nav::navigation::ExecutionOutput &output,
    const CommandSafetyDecision *final_safety,
    bool safety_applied);

LocalDiagnostics makeStoppedLocalDiagnostics(
    LocalDiagnostics local, const std::string &reason,
    const std::string &safety_reason);

}  // namespace lingtu::nav::endpoint
