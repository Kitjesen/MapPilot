#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "plan/rolling_segment_effect_coordinator.hpp"

namespace {

using lingtu::explore::Pose2D;
using lingtu::nav::endpoint::ExplorationSegmentCommandKind;
using lingtu::nav::endpoint::ExplorationSegmentState;
using lingtu::nav::endpoint::RollingSegmentAck;
using lingtu::nav::endpoint::RollingSegmentAckEffect;
using lingtu::nav::endpoint::RollingSegmentActivateAuthorityEffect;
using lingtu::nav::endpoint::RollingSegmentBeginTick;
using lingtu::nav::endpoint::RollingSegmentClearMotionEffect;
using lingtu::nav::endpoint::RollingSegmentCommand;
using lingtu::nav::endpoint::RollingSegmentCommandEvent;
using lingtu::nav::endpoint::RollingSegmentEffectActions;
using lingtu::nav::endpoint::RollingSegmentEffectCoordinator;
using lingtu::nav::endpoint::RollingSegmentEffectFailurePolicy;
using lingtu::nav::endpoint::RollingSegmentExecutionGrid;
using lingtu::nav::endpoint::RollingSegmentInstallPathEffect;
using lingtu::nav::endpoint::RollingSegmentLifecycle;
using lingtu::nav::endpoint::RollingSegmentObserveExecutionGrid;
using lingtu::nav::endpoint::RollingSegmentPublishPathEffect;
using lingtu::nav::endpoint::RollingSegmentRuntimeContext;
using lingtu::nav::endpoint::RollingSegmentStatus;
using lingtu::nav::endpoint::RollingSegmentStatusEffect;
using lingtu::nav::endpoint::RollingSegmentStepResult;
using lingtu::nav::endpoint::RollingSegmentStopAuthorityEffect;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::vector<Pose2D> path() {
  return {{0.5, 1.5, 0.0}, {2.5, 1.5, 0.0}};
}

RollingSegmentStepResult batch(std::vector<lingtu::nav::endpoint::RollingSegmentEffect> effects) {
  RollingSegmentStepResult result;
  result.effects = std::move(effects);
  return result;
}

RollingSegmentEffectActions loggingActions(std::vector<std::string> &log) {
  RollingSegmentEffectActions actions;
  actions.activate_authority = [&]() {
    log.emplace_back("activate");
    return true;
  };
  actions.install_path = [&](const RollingSegmentInstallPathEffect &effect) {
    log.emplace_back("install:" + std::to_string(effect.path.size()));
  };
  actions.publish_path = [&](const RollingSegmentPublishPathEffect &effect) {
    log.emplace_back("publish_path:" + std::to_string(effect.path.size()));
  };
  actions.publish_ack = [&](const RollingSegmentAck &ack) {
    log.emplace_back(std::string{"ack:"} + (ack.accepted ? "accepted:" : "rejected:") + ack.reason);
    return true;
  };
  actions.publish_status = [&](const RollingSegmentStatus &status) {
    log.emplace_back("status:" + std::to_string(static_cast<int>(status.state)) + ":" +
                     status.reason);
    return true;
  };
  actions.stop_authority = [&]() { log.emplace_back("stop"); };
  actions.clear_motion = [&](const std::string &reason) {
    log.emplace_back("clear:" + reason);
    return true;
  };
  return actions;
}

RollingSegmentExecutionGrid freeGrid(std::uint64_t generation = 3U, double stamp_s = 10.0) {
  RollingSegmentExecutionGrid grid;
  grid.stamp_s = stamp_s;
  grid.frame_id = "map";
  grid.width = 10U;
  grid.height = 3U;
  grid.resolution = 1.0;
  grid.origin_qw = 1.0;
  grid.occupancy.assign(30U, 0U);
  grid.terrain_cost.assign(30U, 0U);
  grid.session_id = "rolling-session";
  grid.reset_epoch = 7U;
  grid.generation = generation;
  grid.live = true;
  grid.terrain_risk_stamp_s = stamp_s;
  grid.terrain_risk_ready = true;
  grid.payload_complete = true;
  return grid;
}

RollingSegmentRuntimeContext readyContext(double now_s = 10.0) {
  RollingSegmentRuntimeContext context;
  context.now_s = now_s;
  context.input_ready = true;
  context.autonomy_mode = true;
  context.motion_allowed = true;
  context.driver_control_ready = true;
  context.robot_pose = {0.5, 1.5, 0.0};
  context.map_z = 0.25;
  return context;
}

RollingSegmentCommand executeCommand(double stamp_s = 10.0) {
  RollingSegmentCommand command;
  command.stamp_s = stamp_s;
  command.frame_id = "map";
  command.request_id = "segment-1";
  command.kind = static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute);
  command.session_id = "rolling-session";
  command.reset_epoch = 7U;
  command.minimum_generation = 2U;
  command.target.x = 8.5;
  command.target.y = 1.5;
  command.target.qw = 1.0;
  return command;
}

RollingSegmentStepResult acceptedAdmission(RollingSegmentLifecycle &lifecycle) {
  (void)lifecycle.step(RollingSegmentBeginTick{10.0});
  (void)lifecycle.step(RollingSegmentObserveExecutionGrid{freeGrid()});
  auto result = lifecycle.step(RollingSegmentCommandEvent{executeCommand(), readyContext()});
  require(result.effects.size() == 6U, "test fixture must produce accepted admission effects");
  return result;
}

void testOrder() {
  RollingSegmentLifecycle lifecycle;
  std::vector<std::string> log;
  RollingSegmentEffectCoordinator coordinator(lifecycle, loggingActions(log));

  RollingSegmentAckEffect ack;
  ack.effect_id = 4U;
  ack.ack.accepted = true;
  ack.ack.reason = "accepted";
  RollingSegmentStatusEffect status;
  status.effect_id = 5U;
  status.status.state = ExplorationSegmentState::kExecuting;
  status.status.reason = "executing";
  const bool ok = coordinator.apply(batch({
      RollingSegmentActivateAuthorityEffect{1U},
      RollingSegmentInstallPathEffect{2U, path(), 0.25, 10.0},
      RollingSegmentPublishPathEffect{3U, path(), 0.25, 10.0},
      ack,
      status,
      RollingSegmentStopAuthorityEffect{6U},
      RollingSegmentClearMotionEffect{7U, RollingSegmentEffectFailurePolicy::kAbortBatch, "done"},
  }));

  require(ok, "all successful effects must return true");
  const std::vector<std::string> expected = {
      "activate",           "install:2", "publish_path:2", "ack:accepted:accepted",
      "status:2:executing", "stop",      "clear:done"};
  require(log == expected, "effects must be visited in legacy order");
}

void testAuthorityAbort() {
  RollingSegmentLifecycle lifecycle;
  std::vector<std::string> log;
  auto actions = loggingActions(log);
  actions.activate_authority = [&]() {
    log.emplace_back("activate:false");
    return false;
  };
  RollingSegmentEffectCoordinator coordinator(lifecycle, std::move(actions));

  const bool ok = coordinator.apply(batch({
      RollingSegmentActivateAuthorityEffect{1U},
      RollingSegmentInstallPathEffect{2U, path(), 0.25, 10.0},
  }));

  require(!ok, "authority activation failure must drive the motion return false");
  require(log == std::vector<std::string>{"activate:false"},
          "authority failure must abort the remaining batch");
}

void testMotionReturnTracksOnlyMotionEffects() {
  RollingSegmentLifecycle lifecycle;
  std::vector<std::string> log;
  auto actions = loggingActions(log);
  actions.publish_status = [&](const RollingSegmentStatus &status) {
    log.emplace_back("status:false:" + status.reason);
    return false;
  };
  RollingSegmentEffectCoordinator coordinator(lifecycle, std::move(actions));

  RollingSegmentStatusEffect status;
  status.effect_id = 1U;
  status.failure_policy = RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment;
  status.status.reason = "transport_down";
  require(coordinator.apply(batch({status})),
          "non-motion transport failure must not change the motion return value");

  auto actions_with_clear_failure = loggingActions(log);
  actions_with_clear_failure.clear_motion = [&](const std::string &reason) {
    log.emplace_back("clear:false:" + reason);
    return false;
  };
  RollingSegmentEffectCoordinator clear_coordinator(lifecycle,
                                                    std::move(actions_with_clear_failure));
  require(!clear_coordinator.apply(batch({RollingSegmentClearMotionEffect{
              2U, RollingSegmentEffectFailurePolicy::kAbortBatch, "blocked"}})),
          "clear-motion failure must drive the motion return false");
}

void testIgnoredAckFailureDoesNotAbort() {
  RollingSegmentLifecycle lifecycle;
  std::vector<std::string> log;
  auto actions = loggingActions(log);
  actions.publish_ack = [&](const RollingSegmentAck &ack) {
    log.emplace_back("ack:false:" + ack.reason);
    return false;
  };
  RollingSegmentEffectCoordinator coordinator(lifecycle, std::move(actions));

  RollingSegmentAckEffect ack;
  ack.effect_id = 1U;
  ack.failure_policy = RollingSegmentEffectFailurePolicy::kIgnore;
  ack.ack.reason = "best_effort";
  RollingSegmentStatusEffect status;
  status.effect_id = 2U;
  status.status.state = ExplorationSegmentState::kExecuting;
  status.status.reason = "after_ack";
  require(coordinator.apply(batch({ack, status})),
          "ignored ACK failure must not change the motion return value");
  require(log == std::vector<std::string>({"ack:false:best_effort", "status:2:after_ack"}),
          "ignored ACK failure must not abort the following status effect");
}

void testFailClosedStatusAppliesRecursiveFeedbackBeforeAbort() {
  RollingSegmentLifecycle lifecycle;
  const auto admission = acceptedAdmission(lifecycle);
  std::vector<std::string> log;
  auto actions = loggingActions(log);
  bool failed_first_status = false;
  actions.publish_status = [&](const RollingSegmentStatus &status) {
    log.emplace_back("status:" + std::to_string(static_cast<int>(status.state)) + ":" +
                     status.reason);
    if (!failed_first_status && status.state == ExplorationSegmentState::kAccepted) {
      failed_first_status = true;
      return false;
    }
    return true;
  };
  RollingSegmentEffectCoordinator coordinator(lifecycle, std::move(actions));

  require(coordinator.apply(admission),
          "status transport failure must not make the motion return false when clear succeeds");
  require(!lifecycle.snapshot().active,
          "fail-closed status feedback must release the active segment");

  const std::vector<std::string> expected = {
      "activate",
      "install:6",
      "publish_path:6",
      "ack:accepted:safe_observed_free_prefix",
      "status:1:safe_observed_free_prefix",
      "stop",
      "status:4:segment_status_publish_failed",
      "clear:segment_status_publish_failed",
  };
  if (log != expected) {
    std::cerr << "observed fail-closed log:\n";
    for (const auto &entry : log) {
      std::cerr << entry << '\n';
    }
  }
  require(log == expected,
          "fail-closed status failure must apply recursive stop/status/clear before aborting");
}

void testConstructorValidation() {
  RollingSegmentLifecycle lifecycle;
  bool threw = false;
  try {
    RollingSegmentEffectCoordinator coordinator(lifecycle, RollingSegmentEffectActions{});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  require(threw, "constructor must fail fast when required actions are missing");
}

}  // namespace

int main() {
  try {
    testOrder();
    testAuthorityAbort();
    testMotionReturnTracksOnlyMotionEffects();
    testIgnoredAckFailureDoesNotAbort();
    testFailClosedStatusAppliesRecursiveFeedbackBeforeAbort();
    testConstructorValidation();
  } catch (const std::exception &ex) {
    std::cerr << "FAIL: " << ex.what() << '\n';
    return 1;
  }
  std::cout << "rolling segment effect coordinator tests passed\n";
  return 0;
}
