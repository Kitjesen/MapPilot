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
      zero_published_source_wall_ns_(zero_published_source_wall_ns),
      started_at_(started_at),
      config_(config) {
  if (producer_boot_id_.empty() || output_sequence == 0U ||
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
  observePublishedZero(output_sequence, zero_published_source_wall_ns);
}

void StopConfirmation::observePublishedZero(std::uint64_t output_sequence,
                                            std::uint64_t source_wall_ns) {
  if (output_sequence == 0U || source_wall_ns == 0U ||
      (!zero_publications_.empty() &&
       output_sequence <= zero_publications_.back().output_sequence)) {
    throw std::invalid_argument("stop zero publication requires a fresh sequenced output");
  }
  zero_publications_.push_back({output_sequence, source_wall_ns});
}

void StopConfirmation::observeDriverAck(const std::string &producer_boot_id,
                                        std::uint64_t output_sequence, bool accepted,
                                        std::uint64_t source_stamp_ns) noexcept {
  if (producer_boot_id != producer_boot_id_ || output_sequence < driver_ack_output_sequence_) {
    return;
  }
  const auto publication = std::find_if(
      zero_publications_.rbegin(), zero_publications_.rend(),
      [output_sequence](const ZeroPublication &zero) {
        return zero.output_sequence == output_sequence;
      });
  if (publication == zero_publications_.rend() ||
      source_stamp_ns <= publication->source_wall_ns) {
    return;
  }
  if (!driver_ack_observed_ || driver_accepted_ != accepted) {
    quiet_odometry_samples_ = 0U;
    last_qualified_odometry_source_stamp_ns_ = 0U;
  }
  driver_ack_source_stamp_ns_ = source_stamp_ns;
  driver_ack_output_sequence_ = output_sequence;
  zero_published_source_wall_ns_ = publication->source_wall_ns;
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
      driver_ack_output_sequence_,
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
  if (!result.pending) actions_.record_stop_evidence_failure(result.reason);
  return result;
}

bool MotionStopBarrier::confirmationPending() const noexcept {
  return pending_confirmation_.has_value();
}

std::string MotionStopBarrier::confirmationKey() const {
  return pending_confirmation_ ? pending_confirmation_->key : std::string{};
}

void MotionStopBarrier::resetAfterMotionStart() const {
  completed_confirmation_.reset();
}

void MotionStopBarrier::cancelPendingResume() const {
  if (pending_confirmation_ && pending_confirmation_->key.rfind("resume", 0) == 0) {
    replaceConfirmation();
  }
}

void MotionStopBarrier::replaceConfirmation() const {
  pending_confirmation_.reset();
  completed_confirmation_.reset();
}

std::optional<MotionStopResult> MotionStopBarrier::poll() const {
  if (!pending_confirmation_) return std::nullopt;
  const auto state = actions_.confirm_zero(pending_confirmation_->output_sequence,
                                          pending_confirmation_->evidence_policy);
  if (state == StopConfirmationState::Pending) {
    return MotionStopResult{false, "stop_confirmation_pending", true};
  }
  PendingConfirmation pending = std::move(*pending_confirmation_);
  pending_confirmation_.reset();
  if (state != StopConfirmationState::Confirmed && pending.key.rfind("pause:", 0) == 0) {
    // Preserve the requested pause and its ticket across a reconnect. A fresh
    // zero publication starts the next evidence attempt; no motion is released.
    const std::string failure = (state == StopConfirmationState::DriverRejected
                                    ? "driver_rejected_zero_" : "stop_confirmation_timeout_") +
                                    pending.failure_suffix;
    actions_.record_stop_evidence_failure(failure);
    pending.output_sequence = actions_.last_output_sequence();
    pending_confirmation_ = std::move(pending);
    return MotionStopResult{false, failure, true};
  }
  MotionStopResult result;
  if (state == StopConfirmationState::Confirmed) {
    result = pending.commit ? pending.commit()
                            : MotionStopResult{true, pending.confirmed_reason};
  } else {
    result = {false, (state == StopConfirmationState::DriverRejected
                         ? "driver_rejected_zero_" : "stop_confirmation_timeout_") +
                         pending.failure_suffix};
    if (pending.fail_closed) result = failClosed(std::move(result));
  }
  if (pending.retain_completion) {
    completed_confirmation_ = CompletedConfirmation{std::move(pending.key), result};
  }
  return result;
}

std::optional<MotionStopResult>
MotionStopBarrier::replayConfirmation(const std::string &key) const {
  if (pending_confirmation_ && pending_confirmation_->key == key) {
    auto result = poll();
    if (result && result->pending) return result;
    if (result && !completed_confirmation_) return result;
  }
  if (completed_confirmation_ && completed_confirmation_->key == key) {
    auto result = std::move(completed_confirmation_->result);
    completed_confirmation_.reset();
    return result;
  }
  return std::nullopt;
}

MotionStopResult MotionStopBarrier::confirmTransition(
    const std::string &key, const std::string &confirmed_reason,
    const std::string &failure_suffix, bool fail_closed,
    std::function<MotionStopResult()> commit, std::optional<std::uint64_t> sequence,
    bool retain_completion, std::optional<StopConfirmationEvidencePolicy> evidence_policy) const {
  if (auto replay = replayConfirmation(key)) return *replay;
  replaceConfirmation();
  if (!publish_cmd_vel_) {
    MotionStopResult failure{false, "zero_publish_unavailable_" + failure_suffix};
    return fail_closed ? failClosed(std::move(failure)) : failure;
  }
  const auto output_sequence = sequence ? *sequence : actions_.last_output_sequence();
  if (output_sequence == 0U) {
    MotionStopResult failure{false, "zero_publish_unavailable_" + failure_suffix};
    return fail_closed ? failClosed(std::move(failure)) : failure;
  }
  pending_confirmation_ = PendingConfirmation{key, output_sequence, confirmed_reason,
                                             failure_suffix, fail_closed, retain_completion,
                                             std::move(commit), evidence_policy};
  auto result = *poll();
  if (!result.pending) completed_confirmation_.reset();
  return result;
}

MotionStopResult MotionStopBarrier::confirmAndCommitTerminal(
    MotionStopTerminalCommit commit_terminal, const std::string &confirmed_reason,
    const std::string &failure_suffix, bool fail_closed, bool retain_completion,
    std::optional<StopConfirmationEvidencePolicy> evidence_policy) const {
  return confirmTransition(confirmed_reason + ":" + failure_suffix, confirmed_reason,
                           failure_suffix, fail_closed,
                           [commit_terminal = std::move(commit_terminal), confirmed_reason] {
                             if (commit_terminal) commit_terminal();
                             return MotionStopResult{true, confirmed_reason};
                           }, std::nullopt, retain_completion, evidence_policy);
}

MotionStopResult MotionStopBarrier::confirmLastZero(const std::string &confirmed_reason,
                                                        const std::string &failure_suffix) const {
  return confirmTransition(confirmed_reason + ":" + failure_suffix, confirmed_reason,
                           failure_suffix, false, {}, std::nullopt, true);
}

MotionStopResult MotionStopBarrier::cancel() const {
  const auto result = runCancelPipeline(std::nullopt);
  return {result.accepted, result.reason, result.pending};
}

MotionStopTerminalBarrierResult
MotionStopBarrier::runCancelPipeline(std::optional<MotionStopTerminalCommit> commit_terminal) const {
  if (auto replay = replayConfirmation("cancelled:cancel_remains_stopped")) {
    return {replay->accepted, replay->reason, replay->accepted, replay->pending};
  }
  replaceConfirmation();
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_cancelled");
  if (!commit_terminal) commit_terminal = deferGoalAbort("cancelled");
  if (!clearMotionOutputs("cancelled")) {
    const auto failure = failClosed({false, "zero_publish_failed"});
    return {failure.accepted, failure.reason, false};
  }
  const auto result = confirmAndCommitTerminal(std::move(*commit_terminal), "cancelled",
                                              "cancel_remains_stopped", true);
  return {result.accepted, result.reason, result.accepted, result.pending};
}

MotionStopTerminalBarrierResult
MotionStopBarrier::cancelPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const {
  return runCancelPipeline(std::move(commit_terminal));
}

MotionStopResult MotionStopBarrier::pauseTask(
    MotionStopTerminalCommit commit_paused, const std::string &identity) const {
  const auto key = "pause:" + identity;
  if (auto replay = replayConfirmation(key)) return *replay;
  replaceConfirmation();
  actions_.stop_control();
  if (!actions_.suspend_motion_outputs("task_paused")) {
    return failClosed({false, "zero_publish_failed_pause_remains_stopped"});
  }
  return confirmTransition(key, "pause_requested", "pause_remains_stopped", true,
                           [commit_paused = std::move(commit_paused)] {
                             commit_paused();
                             return MotionStopResult{true, "pause_requested"};
                           });
}

MotionStopResult MotionStopBarrier::resumeTask(MotionStopTerminalCommit commit_resumed,
                                               double source_stamp_s,
                                               const std::string &identity) const {
  const auto key = "resume_task:" + identity;
  if (auto replay = replayConfirmation(key)) return *replay;
  replaceConfirmation();
  if (!actions_.suspend_motion_outputs("task_resuming")) {
    return failClosed({false, "zero_publish_failed_resume_remains_stopped"});
  }
  return confirmTransition(key, "resume_requested", "resume_remains_stopped", true,
                           [this, commit_resumed = std::move(commit_resumed), source_stamp_s] {
                             if (!actions_.resume_control()) return MotionStopResult{false, "estop_latched"};
                             actions_.set_autonomy_request_not_before(source_stamp_s);
                             actions_.clear_operator_resume_required();
                             commit_resumed();
                             return MotionStopResult{true, "resume_requested"};
                           });
}

MotionStopResult
MotionStopBarrier::confirmGoalReplanStop(const std::string &reason) const {
  if (auto replay = replayConfirmation("replan_stop_confirmed:goal_replan_pending")) return *replay;
  replaceConfirmation();
  actions_.stop_control();
  if (!clearMotionOutputs(reason)) {
    return failClosed({false, "zero_publish_failed_goal_replan_pending"});
  }
  const MotionStopResult confirmation =
      confirmLastZero("replan_stop_confirmed", "goal_replan_pending");
  if (!confirmation.accepted && !confirmation.pending) {
    return failClosed(confirmation);
  }
  return confirmation;
}

MotionStopResult
MotionStopBarrier::commitGoalTerminalAfterStop(const std::string &reason,
    MotionStopTerminalCommit commit_terminal,
    std::optional<StopConfirmationEvidencePolicy> evidence_policy) const {
  if (auto replay = replayConfirmation(reason + ":goal_terminal_pending")) return *replay;
  replaceConfirmation();
  actions_.stop_control();
  if (!clearMotionOutputs(reason)) {
    return failClosed({false, "zero_publish_failed_goal_terminal_pending"});
  }
  return confirmAndCommitTerminal(std::move(commit_terminal), reason, "goal_terminal_pending",
                                  true, true, evidence_policy);
}

MotionStopResult MotionStopBarrier::stop() const {
  const auto result = runStopPipeline(std::nullopt, true);
  return {result.accepted, result.reason, result.pending};
}

MotionStopResult MotionStopBarrier::stopWithoutTerminalCommit() const {
  const MotionStopTerminalBarrierResult stopped = runStopPipeline(std::nullopt);
  return {stopped.accepted, stopped.reason, stopped.pending};
}

MotionStopTerminalBarrierResult
MotionStopBarrier::stopPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const {
  return runStopPipeline(std::move(commit_terminal));
}

MotionStopTerminalBarrierResult MotionStopBarrier::runStopPipeline(
    std::optional<MotionStopTerminalCommit> commit_terminal, bool defer_goal_terminal) const {
  if (auto replay = replayConfirmation("stopped:stop_remains_latched")) {
    return {replay->accepted, replay->reason,
            replay->accepted && (defer_goal_terminal || commit_terminal.has_value()),
            replay->pending};
  }
  replaceConfirmation();
  actions_.stop_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_stopped");
  if (defer_goal_terminal) commit_terminal = deferGoalAbort("stopped");
  if (!clearMotionOutputs("stopped")) {
    const MotionStopResult failure = failClosed({false, "zero_publish_failed"});
    return {failure.accepted, failure.reason, false};
  }
  const bool has_terminal = commit_terminal.has_value();
  const auto result = confirmAndCommitTerminal(commit_terminal ? std::move(*commit_terminal)
                                                              : MotionStopTerminalCommit{},
                                              "stopped", "stop_remains_latched", true,
                                              has_terminal);
  return {result.accepted, result.reason, result.accepted && has_terminal, result.pending};
}

MotionStopResult MotionStopBarrier::estop(const std::string &reason) const {
  const MotionStopTerminalBarrierResult estopped =
      runEstopPipeline(reason, std::nullopt, true);
  return {estopped.accepted, estopped.reason, estopped.pending};
}

MotionStopResult
MotionStopBarrier::estopWithoutTerminalCommit(const std::string &reason) const {
  const MotionStopTerminalBarrierResult estopped =
      runEstopPipeline(reason, std::nullopt, false);
  return {estopped.accepted, estopped.reason, estopped.pending};
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
  if (auto replay = replayConfirmation("estop_latched:estop_remains_latched")) {
    return {replay->accepted, replay->reason,
            replay->accepted && (defer_goal_terminal || commit_terminal.has_value()), replay->pending};
  }
  replaceConfirmation();
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
  const bool has_terminal = defer_goal_terminal || commit_terminal.has_value();
  const auto result = confirmTransition(
      "estop_latched:estop_remains_latched", "estop_latched", "estop_remains_latched", false,
      [this, persisted, defer_goal_terminal, commit_terminal = std::move(commit_terminal)]() mutable {
        if (!persisted) return MotionStopResult{false, "estop_latch_persist_failed_estop_remains_latched"};
        if (defer_goal_terminal) commit_terminal = deferGoalAbort("estop_latched");
        if (commit_terminal) (*commit_terminal)();
        return MotionStopResult{true, "estop_latched"};
      }, std::nullopt, has_terminal);
  return {result.accepted, result.reason, result.accepted && has_terminal, result.pending};
}

MotionStopResult MotionStopBarrier::clearEstop(const std::string &precondition_error) const {
  if (!precondition_error.empty()) return {false, precondition_error};
  const std::string key = "clear_estop";
  if (auto replay = replayConfirmation(key)) return *replay;
  replaceConfirmation();
  auto commit_cleared = deferGoalAbort("estop_cleared");
  if (!clearMotionOutputs("estop_cleared")) {
    return {false, "zero_publish_failed_estop_remains_latched"};
  }
  return confirmTransition(key, "estop_cleared", "estop_remains_latched", false,
      [this, commit_cleared = std::move(commit_cleared)] {
        commit_cleared();
        if (!actions_.clear_persisted_estop_latch()) {
          return MotionStopResult{false, "estop_latch_clear_failed_estop_remains_latched"};
        }
        const bool cleared = actions_.clear_control_estop();
        actions_.clear_operator_resume_required();
        return MotionStopResult{cleared, cleared ? "estop_cleared" : "estop_remains_latched"};
      });
}

MotionStopResult MotionStopBarrier::resumeAutonomy(const ResumeAutonomyRequest &request) const {
  if (!request.precondition_error.empty()) return {false, request.precondition_error};
  if (!request.operator_takeover_latched && !request.resume_required) {
    return {true, "autonomy_already_ready"};
  }
  const std::string key = "resume_autonomy:" + std::to_string(request.source_stamp_s);
  if (auto replay = replayConfirmation(key)) return *replay;
  replaceConfirmation();
  if (!actions_.suspend_motion_outputs("autonomy_resume_ready")) {
    return {false, "zero_publish_failed_takeover_remains_latched"};
  }
  return confirmTransition(key, "autonomy_resume_ready", "takeover_remains_latched", false,
      [this, source_stamp_s = request.source_stamp_s] {
        if (!actions_.resume_control()) return MotionStopResult{false, "estop_latched"};
        actions_.set_autonomy_request_not_before(source_stamp_s);
        actions_.clear_operator_resume_required();
        return MotionStopResult{true, "autonomy_resume_ready"};
      });
}

MotionStopResult MotionStopBarrier::resumeTeleop(const ResumeTeleopRequest &request) const {
  if (!request.precondition_error.empty()) return {false, request.precondition_error};
  if (!request.motion_hold_latched && !request.resume_required) return {true, "teleop_already_ready"};
  const std::string key = "resume_teleop:" + std::to_string(request.source_stamp_s);
  if (auto replay = replayConfirmation(key)) return *replay;
  replaceConfirmation();
  if (!clearMotionOutputs("teleop_resume_ready")) {
    return {false, "zero_publish_failed_takeover_remains_latched"};
  }
  return confirmTransition(key, "teleop_resume_ready_reassert_command", "takeover_remains_latched", false,
      [this, source_stamp_s = request.source_stamp_s] {
        actions_.set_teleop_request_not_before(source_stamp_s);
        if (!actions_.resume_control()) return MotionStopResult{false, "estop_latched"};
        actions_.clear_operator_resume_required();
        return MotionStopResult{true, "teleop_resume_ready_reassert_command"};
      });
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
  const std::string key = "shutdown";
  auto result = replayConfirmation(key);
  if (!result) {
    replaceConfirmation();
    actions_.stop_control();
    actions_.cancel_inspection("navd_shutdown");
    if (!clearMotionOutputs("navd_shutdown") || !publish_cmd_vel_) return {false, std::nullopt};
    const auto sequence = actions_.publish_sequenced_zero();
    if (!sequence) return {false, std::nullopt};
    result = confirmTransition(key, "shutdown_confirmed", "shutdown_pending", false,
        [commit_terminal = std::move(commit_terminal)] {
          if (commit_terminal) (*commit_terminal)();
          return MotionStopResult{true, "shutdown_confirmed"};
        }, *sequence, true);
  }
  return {result->accepted,
          result->pending ? StopConfirmationState::Pending :
              result->accepted ? StopConfirmationState::Confirmed :
              result->reason.find("driver_rejected_zero") == 0 ? StopConfirmationState::DriverRejected :
                                                               StopConfirmationState::TimedOut,
          result->pending};
}

}  // namespace lingtu::nav::endpoint
