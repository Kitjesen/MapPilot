#include "safety/stop.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

#if defined(_WIN32)
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace lingtu::nav::endpoint {

StopConfirmation::StopConfirmation(std::string producer_boot_id, std::uint64_t output_sequence,
                                   std::uint64_t zero_published_source_wall_ns,
                                   Clock::time_point started_at,
                                   StopConfirmationConfig config)
    : producer_boot_id_(std::move(producer_boot_id)),
      output_sequence_(output_sequence),
      zero_published_source_wall_ns_(zero_published_source_wall_ns),
      started_at_(started_at),
      config_(config) {
  if (producer_boot_id_.empty() || output_sequence_ == 0U ||
      zero_published_source_wall_ns_ == 0U) {
    throw std::invalid_argument(
        "stop confirmation requires a sequenced producer identity and "
        "zero-publication source wall stamp");
  }
  if (config_.timeout <= std::chrono::milliseconds::zero() ||
      !std::isfinite(config_.linear_speed_threshold_mps) ||
      config_.linear_speed_threshold_mps < 0.0 ||
      !std::isfinite(config_.angular_speed_threshold_radps) ||
      config_.angular_speed_threshold_radps < 0.0 || config_.quiet_odometry_samples == 0U) {
    throw std::invalid_argument("invalid stop confirmation configuration");
  }
  if (config_.evidence_policy != StopConfirmationEvidencePolicy::DriverAckAndOdometry &&
      config_.evidence_policy != StopConfirmationEvidencePolicy::DriverAckOnly) {
    throw std::invalid_argument("invalid stop confirmation evidence policy");
  }
}

void StopConfirmation::observeDriverAck(const std::string &producer_boot_id,
                                        std::uint64_t output_sequence, bool accepted,
                                        std::uint64_t source_stamp_ns) noexcept {
  if (producer_boot_id != producer_boot_id_ || output_sequence != output_sequence_ ||
      source_stamp_ns <= zero_published_source_wall_ns_) {
    return;
  }
  if (!driver_ack_observed_ || driver_accepted_ != accepted) {
    quiet_odometry_samples_ = 0U;
    last_qualified_odometry_source_stamp_ns_ = 0U;
    driver_ack_source_stamp_ns_ = source_stamp_ns;
  }
  driver_ack_observed_ = true;
  driver_accepted_ = accepted;
}

void StopConfirmation::observeQuietOdometry(double source_stamp_s, double linear_speed_mps,
                                            double angular_speed_radps) noexcept {
  ++odometry_samples_observed_;
  last_observed_odometry_source_stamp_ns_ = sourceStampNanoseconds(source_stamp_s);
  last_linear_speed_mps_ = linear_speed_mps;
  last_angular_speed_radps_ = angular_speed_radps;
  if (!driver_ack_observed_ || !driver_accepted_) {
    return;
  }
  ++post_ack_odometry_samples_;
  const std::uint64_t source_stamp_ns = last_observed_odometry_source_stamp_ns_;
  if (source_stamp_ns == 0U || source_stamp_ns <= last_qualified_odometry_source_stamp_ns_) {
    ++stale_odometry_samples_;
    return;
  }
  last_qualified_odometry_source_stamp_ns_ = source_stamp_ns;
  if (!std::isfinite(linear_speed_mps) || !std::isfinite(angular_speed_radps) ||
      linear_speed_mps > config_.linear_speed_threshold_mps ||
      angular_speed_radps > config_.angular_speed_threshold_radps) {
    ++moving_odometry_samples_;
    quiet_odometry_samples_ = 0U;
    return;
  }
  if (quiet_odometry_samples_ < config_.quiet_odometry_samples) {
    ++quiet_odometry_samples_;
  }
}

StopConfirmationState StopConfirmation::state(Clock::time_point now) const noexcept {
  if (driver_ack_observed_ && !driver_accepted_) {
    return StopConfirmationState::DriverRejected;
  }
  if (now - started_at_ >= config_.timeout) {
    return StopConfirmationState::TimedOut;
  }
  const bool odometry_confirmed =
      config_.evidence_policy == StopConfirmationEvidencePolicy::DriverAckOnly ||
      quiet_odometry_samples_ >= config_.quiet_odometry_samples;
  if (driver_ack_observed_ && driver_accepted_ && odometry_confirmed) {
    return StopConfirmationState::Confirmed;
  }
  return StopConfirmationState::Pending;
}

StopConfirmationDiagnostics StopConfirmation::diagnostics() const noexcept {
  return {
      zero_published_source_wall_ns_,
      driver_ack_source_stamp_ns_,
      last_observed_odometry_source_stamp_ns_,
      odometry_samples_observed_,
      post_ack_odometry_samples_,
      stale_odometry_samples_,
      moving_odometry_samples_,
      quiet_odometry_samples_,
      config_.evidence_policy == StopConfirmationEvidencePolicy::DriverAckAndOdometry
          ? config_.quiet_odometry_samples
          : 0U,
      config_.evidence_policy == StopConfirmationEvidencePolicy::DriverAckAndOdometry,
      last_linear_speed_mps_,
      last_angular_speed_radps_,
      driver_ack_observed_,
      driver_accepted_,
  };
}

std::uint64_t StopConfirmation::sourceStampNanoseconds(double source_stamp_s) noexcept {
  if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
    return 0U;
  }
  const long double ns = static_cast<long double>(source_stamp_s) * 1000000000.0L;
  if (ns >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(ns + 0.5L);
}

EstopLatchStore::EstopLatchStore(std::string path) : path_(std::move(path)) {}

std::optional<std::string> EstopLatchStore::load() const {
  if (path_.empty()) {
    return std::nullopt;
  }
  std::error_code error;
  const bool exists = std::filesystem::exists(path_, error);
  if (error) {
    return std::string("estop_latch_state_unreadable");
  }
  if (!exists) {
    return std::nullopt;
  }
  std::ifstream input(path_);
  std::string reason;
  if (input) {
    std::getline(input, reason);
  }
  return reason.empty() ? std::optional<std::string>("persisted_estop")
                        : std::optional<std::string>(reason);
}

bool EstopLatchStore::persist(const std::string &reason) const {
  if (path_.empty()) {
    return true;
  }
  std::error_code error;
  const auto parent = path_.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, error);
    if (error) {
      return false;
    }
  }
  const std::string path_text = path_.string();
  std::FILE *output = std::fopen(path_text.c_str(), "wb");
  if (output == nullptr) {
    return false;
  }
  std::string line = reason.empty() ? "estop" : reason;
  const auto newline = line.find_first_of("\r\n");
  if (newline != std::string::npos) {
    line.resize(newline);
  }
  line.push_back('\n');
  const bool written = std::fwrite(line.data(), 1, line.size(), output) == line.size();
  const bool synced = written && syncFile(output);
  const bool closed = std::fclose(output) == 0;
  return synced && closed && syncParentDirectory();
}

bool EstopLatchStore::clear() const {
  if (path_.empty()) {
    return true;
  }
  std::error_code error;
  std::filesystem::remove(path_, error);
  if (error) {
    return false;
  }
  (void)syncParentDirectory();
  return true;
}

bool EstopLatchStore::syncFile(std::FILE *file) {
  if (std::fflush(file) != 0) {
    return false;
  }
#if defined(_WIN32)
  return ::_commit(::_fileno(file)) == 0;
#else
  return ::fsync(::fileno(file)) == 0;
#endif
}

bool EstopLatchStore::syncParentDirectory() const {
#if defined(_WIN32)
  return true;
#else
  const auto parent =
      path_.parent_path().empty() ? std::filesystem::path(".") : path_.parent_path();
  const int descriptor = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
  if (descriptor < 0) {
    return false;
  }
  const bool synced = ::fsync(descriptor) == 0;
  ::close(descriptor);
  return synced;
#endif
}

MotionStopBarrier::MotionStopBarrier(bool publish_cmd_vel, MotionStopActions actions)
    : publish_cmd_vel_(publish_cmd_vel), actions_(std::move(actions)) {}

bool MotionStopBarrier::clearEndpointMotion(const std::string &reason) const {
  return clearMotionOutputs(reason);
}

bool MotionStopBarrier::holdEndpointMotion(const std::string &reason) const {
  return actions_.suspend_motion_outputs(reason);
}

bool MotionStopBarrier::clearMotionOutputs(const std::string &reason) const {
  if (actions_.rolling_segment_active()) {
    return actions_.preempt_rolling_segment(reason);
  }
  return actions_.clear_motion_outputs(reason);
}

MotionStopTerminalCommit MotionStopBarrier::deferGoalAbort(const std::string &reason) const {
  MotionStopTerminalCommit commit_terminal = actions_.defer_goal_abort(reason);
  actions_.sync_goal_diagnostics();
  return commit_terminal;
}

MotionStopResult MotionStopBarrier::failClosed(MotionStopResult result) const {
  actions_.record_stop_evidence_failure(result.reason);
  return result;
}

MotionStopResult MotionStopBarrier::confirmAndCommitTerminal(
    MotionStopTerminalCommit commit_terminal, const std::string &confirmed_reason,
    const std::string &failure_suffix, bool fail_closed) const {
  const MotionStopResult confirmation = confirmLastZero(confirmed_reason, failure_suffix);
  if (!confirmation.accepted) {
    return fail_closed ? failClosed(confirmation) : confirmation;
  }
  commit_terminal();
  return confirmation;
}

MotionStopResult MotionStopBarrier::confirmLastZero(const std::string &confirmed_reason,
                                                        const std::string &failure_suffix) const {
  if (!publish_cmd_vel_) {
    return {
        false,
        "zero_publish_unavailable_" + failure_suffix,
    };
  }
  const std::uint64_t output_sequence = actions_.last_output_sequence();
  if (output_sequence == 0U) {
    return {
        false,
        "zero_publish_unavailable_" + failure_suffix,
    };
  }
  const StopConfirmationState state = actions_.confirm_zero(output_sequence);
  if (state == StopConfirmationState::Confirmed) {
    return {true, confirmed_reason};
  }
  if (state == StopConfirmationState::DriverRejected) {
    return {
        false,
        "driver_rejected_zero_" + failure_suffix,
    };
  }
  return {
      false,
      "stop_confirmation_timeout_" + failure_suffix,
  };
}

MotionStopResult MotionStopBarrier::cancel() const {
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_cancelled");
  MotionStopTerminalCommit commit_cancelled = deferGoalAbort("cancelled");
  if (!clearMotionOutputs("cancelled")) {
    return failClosed({false, "zero_publish_failed"});
  }
  return confirmAndCommitTerminal(std::move(commit_cancelled), "cancelled",
                                  "cancel_remains_stopped", true);
}

MotionStopTerminalBarrierResult
MotionStopBarrier::cancelPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const {
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_cancelled");
  if (!clearMotionOutputs("cancelled")) {
    const MotionStopResult failure = failClosed({false, "zero_publish_failed"});
    return {failure.accepted, failure.reason, false};
  }
  const MotionStopResult confirmation = confirmLastZero("cancelled", "cancel_remains_stopped");
  if (!confirmation.accepted) {
    const MotionStopResult failure = failClosed(confirmation);
    return {failure.accepted, failure.reason, false};
  }
  commit_terminal();
  return {true, confirmation.reason, true};
}

MotionStopResult MotionStopBarrier::pauseTask(
    MotionStopTerminalCommit commit_paused) const {
  actions_.stop_control();
  if (!actions_.suspend_motion_outputs("task_paused")) {
    return failClosed({false, "zero_publish_failed_pause_remains_stopped"});
  }
  return confirmAndCommitTerminal(std::move(commit_paused), "pause_requested",
                                  "pause_remains_stopped", true);
}

MotionStopResult
MotionStopBarrier::confirmGoalReplanStop(const std::string &reason) const {
  actions_.stop_control();
  if (!clearMotionOutputs(reason)) {
    return failClosed({false, "zero_publish_failed_goal_replan_pending"});
  }
  const MotionStopResult confirmation =
      confirmLastZero("replan_stop_confirmed", "goal_replan_pending");
  if (!confirmation.accepted) {
    return failClosed(confirmation);
  }
  return confirmation;
}

MotionStopResult
MotionStopBarrier::commitGoalTerminalAfterStop(const std::string &reason,
                                                   MotionStopTerminalCommit commit_terminal) const {
  actions_.stop_control();
  if (!clearMotionOutputs(reason)) {
    return failClosed({false, "zero_publish_failed_goal_terminal_pending"});
  }
  return confirmAndCommitTerminal(std::move(commit_terminal), reason, "goal_terminal_pending",
                                  true);
}

MotionStopResult MotionStopBarrier::stop() const {
  actions_.stop_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_stopped");
  MotionStopTerminalCommit commit_stopped = deferGoalAbort("stopped");
  if (!clearMotionOutputs("stopped")) {
    return failClosed({false, "zero_publish_failed"});
  }
  return confirmAndCommitTerminal(std::move(commit_stopped), "stopped", "stop_remains_latched",
                                  true);
}

MotionStopResult MotionStopBarrier::stopWithoutTerminalCommit() const {
  const MotionStopTerminalBarrierResult stopped = runStopPipeline(std::nullopt);
  return {stopped.accepted, stopped.reason};
}

MotionStopTerminalBarrierResult
MotionStopBarrier::stopPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const {
  return runStopPipeline(std::move(commit_terminal));
}

MotionStopTerminalBarrierResult MotionStopBarrier::runStopPipeline(
    std::optional<MotionStopTerminalCommit> commit_terminal) const {
  actions_.stop_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_stopped");
  if (!clearMotionOutputs("stopped")) {
    const MotionStopResult failure = failClosed({false, "zero_publish_failed"});
    return {failure.accepted, failure.reason, false};
  }
  const MotionStopResult confirmation = confirmLastZero("stopped", "stop_remains_latched");
  if (!confirmation.accepted) {
    const MotionStopResult failure = failClosed(confirmation);
    return {failure.accepted, failure.reason, false};
  }
  if (commit_terminal) {
    (*commit_terminal)();
  }
  return {true, confirmation.reason, commit_terminal.has_value()};
}

MotionStopResult MotionStopBarrier::estop(const std::string &reason) const {
  const MotionStopTerminalBarrierResult estopped =
      runEstopPipeline(reason, std::nullopt, true);
  return {estopped.accepted, estopped.reason};
}

MotionStopResult
MotionStopBarrier::estopWithoutTerminalCommit(const std::string &reason) const {
  const MotionStopTerminalBarrierResult estopped =
      runEstopPipeline(reason, std::nullopt, false);
  return {estopped.accepted, estopped.reason};
}

MotionStopTerminalBarrierResult
MotionStopBarrier::estopPreservingGoalTerminal(const std::string &estop_reason,
                                                   MotionStopTerminalCommit commit_terminal) const {
  return runEstopPipeline(estop_reason, std::move(commit_terminal), false);
}

MotionStopTerminalBarrierResult MotionStopBarrier::runEstopPipeline(
    const std::string &estop_reason,
    std::optional<MotionStopTerminalCommit> commit_terminal,
    bool defer_goal_terminal) const {
  actions_.latch_estop(estop_reason);
  const bool persisted = actions_.persist_estop_latch(estop_reason);
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("estop_latched");
  if (!clearMotionOutputs("estop_latched")) {
    return {
        false,
        "zero_publish_failed_estop_remains_latched",
        false,
    };
  }
  const MotionStopResult confirmation = confirmLastZero("estop_latched", "estop_remains_latched");
  if (!confirmation.accepted) {
    return {confirmation.accepted, confirmation.reason, false};
  }
  if (!persisted) {
    return {
        false,
        "estop_latch_persist_failed_estop_remains_latched",
        false,
    };
  }
  if (defer_goal_terminal) {
    commit_terminal = deferGoalAbort("estop_latched");
  }
  if (commit_terminal) {
    (*commit_terminal)();
  }
  return {true, "estop_latched", commit_terminal.has_value()};
}

MotionStopResult MotionStopBarrier::clearEstop(const std::string &precondition_error) const {
  if (!precondition_error.empty()) {
    return {false, precondition_error};
  }
  MotionStopTerminalCommit commit_cleared = deferGoalAbort("estop_cleared");
  if (!clearMotionOutputs("estop_cleared")) {
    return {
        false,
        "zero_publish_failed_estop_remains_latched",
    };
  }
  const MotionStopResult confirmation = confirmAndCommitTerminal(
      std::move(commit_cleared), "zero_confirmed", "estop_remains_latched", false);
  if (!confirmation.accepted) {
    return confirmation;
  }
  if (!actions_.clear_persisted_estop_latch()) {
    return {
        false,
        "estop_latch_clear_failed_estop_remains_latched",
    };
  }
  const bool cleared = actions_.clear_control_estop();
  actions_.clear_operator_resume_required();
  return {
      cleared,
      cleared ? "estop_cleared" : "estop_remains_latched",
  };
}

MotionStopResult MotionStopBarrier::resumeAutonomy(const ResumeAutonomyRequest &request) const {
  if (!request.precondition_error.empty()) {
    return {false, request.precondition_error};
  }
  if (!request.operator_takeover_latched) {
    return {true, "autonomy_already_ready"};
  }
  MotionStopTerminalCommit commit_ready = deferGoalAbort("autonomy_resume_ready");
  if (!clearMotionOutputs("autonomy_resume_ready")) {
    return {
        false,
        "zero_publish_failed_takeover_remains_latched",
    };
  }
  const MotionStopResult confirmation = confirmAndCommitTerminal(
      std::move(commit_ready), "zero_confirmed", "takeover_remains_latched", false);
  if (!confirmation.accepted) {
    return confirmation;
  }
  if (!actions_.resume_control()) {
    return {false, "estop_latched"};
  }
  actions_.set_autonomy_request_not_before(request.source_stamp_s);
  actions_.clear_operator_resume_required();
  return {true, "autonomy_resume_ready_reissue_goal"};
}

MotionStopResult MotionStopBarrier::resumeTeleop(const ResumeTeleopRequest &request) const {
  if (!request.precondition_error.empty()) {
    return {false, request.precondition_error};
  }
  if (!request.motion_hold_latched) {
    return {true, "teleop_already_ready"};
  }
  if (!clearMotionOutputs("teleop_resume_ready")) {
    return {
        false,
        "zero_publish_failed_takeover_remains_latched",
    };
  }
  const MotionStopResult confirmation =
      confirmLastZero("zero_confirmed", "takeover_remains_latched");
  if (!confirmation.accepted) {
    return confirmation;
  }
  actions_.set_teleop_request_not_before(request.source_stamp_s);
  if (!actions_.resume_control()) {
    return {false, "estop_latched"};
  }
  actions_.clear_operator_resume_required();
  return {true, "teleop_resume_ready_reassert_command"};
}

bool MotionStopBarrier::driverAuthorityLost(const std::string &blocker) const {
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("driver_control_lost:" + blocker);
  return clearEndpointMotion(blocker);
}

bool MotionStopBarrier::keepZeroFresh() const {
  return actions_.publish_zero();
}

FinalShutdownResult MotionStopBarrier::finalShutdown() const {
  return finalShutdownWithoutTerminalCommit();
}

FinalShutdownResult MotionStopBarrier::finalShutdownWithoutTerminalCommit() const {
  return runFinalShutdownPipeline(std::nullopt);
}

FinalShutdownResult MotionStopBarrier::finalShutdownPreservingGoalTerminal(
    MotionStopTerminalCommit commit_terminal) const {
  return runFinalShutdownPipeline(std::move(commit_terminal));
}

FinalShutdownResult MotionStopBarrier::runFinalShutdownPipeline(
    std::optional<MotionStopTerminalCommit> commit_terminal) const {
  actions_.stop_control();
  actions_.cancel_inspection("navd_shutdown");
  if (!clearMotionOutputs("navd_shutdown")) {
    return {false, std::nullopt};
  }
  if (!publish_cmd_vel_) {
    return {false, std::nullopt};
  }
  const std::optional<std::uint64_t> output_sequence = actions_.publish_sequenced_zero();
  if (!output_sequence) {
    return {false, std::nullopt};
  }
  const StopConfirmationState state = actions_.confirm_zero(*output_sequence);
  if (state == StopConfirmationState::Confirmed) {
    if (commit_terminal) {
      (*commit_terminal)();
    }
  }
  return {
      state == StopConfirmationState::Confirmed,
      state,
  };
}

}  // namespace lingtu::nav::endpoint
