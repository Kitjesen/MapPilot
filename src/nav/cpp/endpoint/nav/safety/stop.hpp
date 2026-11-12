#pragma once

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <optional>
#include <string>

namespace lingtu::nav::endpoint {

enum class StopConfirmationEvidencePolicy {
  ZeroOutputAndOdometry,
  ZeroOutputOnly,
};

struct StopConfirmationConfig {
  std::chrono::milliseconds timeout{4000};
  double linear_speed_threshold_mps{0.03};
  double angular_speed_threshold_radps{0.08};
  std::size_t quiet_odometry_samples{8};
  StopConfirmationEvidencePolicy evidence_policy{
      StopConfirmationEvidencePolicy::ZeroOutputAndOdometry};
};

enum class StopConfirmationState {
  Pending,
  Confirmed,
  TimedOut,
};

struct StopConfirmationDiagnostics {
  std::uint64_t zero_published_source_wall_ns{0U};
  std::uint64_t driver_state_stamp_ns{0U};
  std::uint64_t last_odometry_source_stamp_ns{0U};
  std::size_t odometry_samples_observed{0U};
  std::size_t post_zero_odometry_samples{0U};
  std::size_t stale_odometry_samples{0U};
  std::size_t moving_odometry_samples{0U};
  std::size_t quiet_odometry_samples{0U};
  std::size_t required_quiet_odometry_samples{0U};
  bool quiet_odometry_required{true};
  double last_linear_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double last_angular_speed_radps{std::numeric_limits<double>::quiet_NaN()};
  bool driver_state_observed{false};
  bool driver_zero_output{false};
};

class StopConfirmation {
 public:
  using Clock = std::chrono::steady_clock;

  StopConfirmation(std::uint64_t zero_published_source_wall_ns, Clock::time_point started_at,
                   StopConfirmationConfig config = {});

  void observeDriverState(bool zero_output, std::uint64_t source_stamp_ns) noexcept;
  void observeQuietOdometry(double source_stamp_s, double linear_speed_mps,
                            double angular_speed_radps) noexcept;
  [[nodiscard]] StopConfirmationState state(Clock::time_point now = Clock::now()) const noexcept;
  [[nodiscard]] StopConfirmationDiagnostics diagnostics() const noexcept;

 private:
  static std::uint64_t sourceStampNanoseconds(double source_stamp_s) noexcept;

  std::uint64_t zero_published_source_wall_ns_{0U};
  Clock::time_point started_at_{};
  StopConfirmationConfig config_;
  std::uint64_t driver_state_stamp_ns_{0U};
  std::uint64_t last_observed_odometry_source_stamp_ns_{0U};
  std::uint64_t last_qualified_odometry_source_stamp_ns_{0U};
  std::size_t odometry_samples_observed_{0U};
  std::size_t post_zero_odometry_samples_{0U};
  std::size_t stale_odometry_samples_{0U};
  std::size_t moving_odometry_samples_{0U};
  std::size_t quiet_odometry_samples_{0};
  double last_linear_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_angular_speed_radps_{std::numeric_limits<double>::quiet_NaN()};
  bool driver_state_observed_{false};
  bool driver_zero_output_{false};
};

class EstopLatchStore {
 public:
  explicit EstopLatchStore(std::string path);

  [[nodiscard]] std::optional<std::string> load() const;
  [[nodiscard]] bool persist(const std::string &reason) const;
  [[nodiscard]] bool clear() const;

 private:
  static bool syncFile(std::FILE *file);
  bool syncParentDirectory() const;

  std::filesystem::path path_;
};

struct MotionStopResult {
  bool accepted{false};
  std::string reason;
  bool pending{false};
};

struct MotionStopTerminalBarrierResult {
  bool accepted{false};
  std::string reason;
  bool terminal_committed{false};
  bool pending{false};
};

struct ResumeRequest {
  std::string precondition_error;
  bool held{false};
  double source_stamp_s{0.0};
};

struct FinalShutdownResult {
  bool success{false};
  std::optional<StopConfirmationState> confirmation_state;
  bool pending{false};
};

using MotionStopTerminalCommit = std::function<void()>;

struct MotionStopActions {
  std::function<MotionStopTerminalCommit(const std::string &)> defer_goal_abort;
  std::function<void(const std::string &)> hold;
  std::function<void()> sync_goal_diagnostics;
  std::function<bool()> rolling_segment_active;
  std::function<bool(const std::string &)> preempt_rolling_segment;
  std::function<bool(const std::string &)> clear_motion_outputs;
  std::function<bool(const std::string &)> suspend_motion_outputs;

  std::function<void()> cancel_control;
  std::function<void()> stop_control;
  std::function<void(const std::string &)> latch_estop;
  std::function<bool()> clear_control_estop;
  std::function<bool()> resume_control;

  std::function<void(const std::string &)> cancel_inspection;
  std::function<void(double)> set_autonomy_request_not_before;
  std::function<void(double)> set_teleop_request_not_before;

  std::function<bool(const std::string &)> persist_estop_latch;
  std::function<bool()> clear_persisted_estop_latch;

  std::function<bool()> publish_zero;
  std::function<std::uint64_t()> last_output_sequence;
  std::function<std::optional<std::uint64_t>()> publish_sequenced_zero;
  std::function<StopConfirmationState(
      std::uint64_t, std::optional<StopConfirmationEvidencePolicy>)> confirm_zero;
  std::function<void()> clear_global_path;
};

class MotionStopBarrier {
 public:
  MotionStopBarrier(bool publish_cmd_vel, MotionStopActions actions);

  bool clearEndpointMotion(const std::string &reason) const;
  bool holdEndpointMotion(const std::string &reason) const;

  MotionStopResult cancel() const;
  // commit_terminal must be non-empty and copy-safe/idempotent across retries.
  MotionStopTerminalBarrierResult
  cancelPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const;
  MotionStopResult pauseTask(MotionStopTerminalCommit commit_paused,
                            const std::string &identity = {}) const;
  MotionStopResult resumeTask(MotionStopTerminalCommit commit_resumed, double source_stamp_s,
                             const std::string &identity) const;
  // Called by the ordinary endpoint frame, never by a blocking wait loop.
  std::optional<MotionStopResult> poll() const;
  bool confirmationPending() const noexcept;
  std::string confirmationKey() const;
  void resetAfterMotionStart() const;
  void cancelPendingResume() const;
  MotionStopResult confirmGoalReplanStop(const std::string &reason) const;
  MotionStopResult commitGoalTerminalAfterStop(
      const std::string &reason,
      MotionStopTerminalCommit commit_terminal,
      std::optional<StopConfirmationEvidencePolicy> evidence_policy = std::nullopt) const;
  MotionStopResult stop() const;
  MotionStopResult stopWithoutTerminalCommit() const;
  // commit_terminal must be non-empty and copy-safe/idempotent across retries.
  MotionStopTerminalBarrierResult
  stopPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const;
  MotionStopResult estop(const std::string &reason) const;
  MotionStopResult estopWithoutTerminalCommit(const std::string &reason) const;
  // commit_terminal must be non-empty and copy-safe/idempotent across retries.
  MotionStopTerminalBarrierResult
  estopPreservingGoalTerminal(const std::string &estop_reason,
                              MotionStopTerminalCommit commit_terminal) const;
  MotionStopResult clearEstop(const std::string &precondition_error) const;
  MotionStopResult resumeAutonomy(const ResumeRequest &request) const;
  MotionStopResult resumeTeleop(const ResumeRequest &request) const;

  bool keepZeroFresh() const;
  FinalShutdownResult finalShutdownWithoutTerminalCommit() const;
  FinalShutdownResult
  finalShutdownPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const;
  // Compatibility alias. Production shutdown uses the ticketed transaction.
  FinalShutdownResult finalShutdown() const;

 private:
  struct PendingConfirmation {
    std::string key;
    std::uint64_t output_sequence{0U};
    std::string confirmed_reason;
    std::string failure_suffix;
    bool fail_closed{false};
    bool retain_completion{false};
    std::function<MotionStopResult()> commit;
    std::optional<StopConfirmationEvidencePolicy> evidence_policy;
  };
  struct CompletedConfirmation {
    std::string key;
    MotionStopResult result;
  };
  std::optional<MotionStopResult> replayConfirmation(const std::string &key) const;
  MotionStopResult confirmTransition(const std::string &key,
                                    const std::string &confirmed_reason,
                                    const std::string &failure_suffix, bool fail_closed,
                                    std::function<MotionStopResult()> commit,
                                    std::optional<std::uint64_t> sequence = std::nullopt,
                                    bool retain_completion = false,
                                    std::optional<StopConfirmationEvidencePolicy> evidence_policy =
                                        std::nullopt) const;
  void replaceConfirmation() const;
  bool clearMotionOutputs(const std::string &reason) const;
  MotionStopTerminalCommit deferGoalAbort(const std::string &reason) const;
  MotionStopResult failClosed(MotionStopResult result) const;
  MotionStopResult confirmAndCommitTerminal(MotionStopTerminalCommit commit_terminal,
                                            const std::string &confirmed_reason,
                                            const std::string &failure_suffix,
                                            bool fail_closed, bool retain_completion = true,
                                            std::optional<StopConfirmationEvidencePolicy> evidence_policy =
                                                std::nullopt) const;
  MotionStopResult confirmLastZero(const std::string &confirmed_reason,
                                   const std::string &failure_suffix) const;
  MotionStopTerminalBarrierResult
  runStopPipeline(std::optional<MotionStopTerminalCommit> commit_terminal,
                  bool defer_goal_terminal = false) const;
  MotionStopTerminalBarrierResult
  runCancelPipeline(std::optional<MotionStopTerminalCommit> commit_terminal) const;
  MotionStopTerminalBarrierResult
  runEstopPipeline(const std::string &estop_reason,
                   std::optional<MotionStopTerminalCommit> commit_terminal,
                   bool defer_goal_terminal) const;
  FinalShutdownResult
  runFinalShutdownPipeline(std::optional<MotionStopTerminalCommit> commit_terminal) const;

  bool publish_cmd_vel_{false};
  MotionStopActions actions_;
  mutable std::optional<PendingConfirmation> pending_confirmation_;
  mutable std::optional<CompletedConfirmation> completed_confirmation_;
};

}  // namespace lingtu::nav::endpoint
