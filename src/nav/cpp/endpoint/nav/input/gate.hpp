#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <string_view>

namespace lingtu::nav::endpoint {

enum class SourceStampDecision {
  kAccept,
  kClockRebase,
  kReject,
};

// Source time orders samples; receiver steady time determines freshness.
SourceStampDecision classifySourceOrder(double previous_stamp_s, double incoming_stamp_s,
                                        double clock_rebase_threshold_s);

struct InputGateConfig {
  double odom_max_age_s{0.25};
  double tf_max_age_s{0.25};
  double cloud_max_age_s{0.35};
  std::uint32_t recovery_frames{3};
  double future_tolerance_s{0.05};
  bool require_odom{true};
  bool require_cloud{true};
  double traversability_max_age_s{1.5};
  double localization_health_max_age_s{0.5};
  double driver_control_max_age_s{0.35};
  bool require_traversability{false};
  bool require_localization_health{false};
  bool require_driver_control{false};
  double odom_max_speed_mps{3.0};
  double local_collision_max_age_s{0.5};
  bool require_local_collision{false};
};

struct InputSnapshot {
  double now_s{0.0};
  double odom_stamp_s{0.0};
  double odom_receive_s{0.0};
  std::uint64_t odom_generation{0};
  double odom_linear_speed_mps{0.0};
  double tf_stamp_s{0.0};
  double tf_receive_s{0.0};
  std::uint64_t tf_generation{0};
  double cloud_stamp_s{0.0};
  double cloud_receive_s{0.0};
  std::uint64_t cloud_generation{0};
  double traversability_stamp_s{0.0};
  double traversability_receive_s{0.0};
  std::uint64_t traversability_generation{0};
  double local_collision_stamp_s{0.0};
  double local_collision_receive_s{0.0};
  std::uint64_t local_collision_sequence{0};
  bool local_collision_complete{false};
  double localization_health_stamp_s{0.0};
  double localization_health_receive_s{0.0};
  std::uint64_t localization_health_generation{0};
  double driver_control_stamp_s{0.0};
  double driver_control_receive_s{0.0};
  std::uint64_t driver_control_generation{0};
  bool odom_requires_tf{true};
  bool localization_healthy{false};
  std::string localization_state;
  std::string localization_reason;
  bool driver_control_ready{false};
  std::string driver_control_reason;
};

struct LocalizationHealthSample {
  bool valid{false};
  bool healthy{false};
  double stamp_s{0.0};
  std::string state;
  std::string reason;
  std::string error;
};

bool isHealthyLocalizationState(std::string_view state);
bool isCatastrophicLocalizationReason(std::string_view reason);
LocalizationHealthSample decodeLocalizationHealth(std::string_view json);

enum class OdometrySpeedEvidence {
  ReportedTwistAndPose,
  PoseDerivedPlanar,
};

inline constexpr double kOdometryMinimumStampAdvanceS = 1e-6;

class OdometrySpeedMonitor {
 public:
  explicit OdometrySpeedMonitor(
      OdometrySpeedEvidence evidence = OdometrySpeedEvidence::ReportedTwistAndPose);

  double observe(double stamp_s, std::string_view frame_id, double x, double y, double z, double vx,
                 double vy, double vz);
  void reset();

 private:
  struct PoseSample {
    double stamp_s{0.0};
    std::array<double, 3> position{};
  };

  static double planarDistance(const std::array<double, 3> &lhs,
                               const std::array<double, 3> &rhs);
  std::array<double, 3> medianPosition() const;
  void resetPoseHistory(std::string_view frame_id);
  void appendSample(const PoseSample &sample);

  OdometrySpeedEvidence evidence_{OdometrySpeedEvidence::ReportedTwistAndPose};
  std::string previous_frame_;
  std::array<PoseSample, 5> samples_{};
  std::size_t sample_count_{0};
  std::deque<PoseSample> filtered_samples_;
};

struct InputGateState {
  bool ready{false};
  bool recovering{false};
  std::string reason{"not_evaluated"};
  double odom_age_s{-1.0};
  double tf_age_s{-1.0};
  double cloud_age_s{-1.0};
  double traversability_age_s{-1.0};
  double local_collision_age_s{-1.0};
  double localization_health_age_s{-1.0};
  double driver_control_age_s{-1.0};
  bool localization_healthy{false};
  std::string localization_state;
  std::string localization_reason;
  bool driver_control_ready{false};
  std::string driver_control_reason;
  double odom_linear_speed_mps{0.0};
  std::uint32_t fresh_frames{0};
};

inline bool manualModeMayBypassInputGate(const InputGateState &state) noexcept {
  const std::string_view reason{state.reason};
  const auto starts_with = [&](std::string_view prefix) {
    return reason.size() >= prefix.size() && reason.substr(0, prefix.size()) == prefix;
  };
  return state.ready || reason == "recovering" || starts_with("odom_") ||
          starts_with("tf_") || starts_with("cloud_") ||
          starts_with("traversability_") || starts_with("local_collision_") ||
          starts_with("localization_");
}

class InputGate {
 public:
  explicit InputGate(InputGateConfig config = {});

  InputGateState evaluate(const InputSnapshot &inputs);
  void reset();
  void beginRecoveryFrom(const InputSnapshot &inputs);

 private:
  struct InputGenerations {
    std::uint64_t odom{0};
    std::uint64_t tf{0};
    std::uint64_t cloud{0};
    std::uint64_t traversability{0};
    std::uint64_t local_collision{0};
    std::uint64_t localization_health{0};
    std::uint64_t driver_control{0};
  };

  static InputGenerations generations(const InputSnapshot &inputs);
  bool allRequiredInputsAdvanced(const InputSnapshot &inputs) const;
  static double age(double now_s, double last_s);
  static double receiveOrSource(double receive_s, double source_s);

  InputGateConfig config_{};
  std::uint32_t fresh_frames_{0};
  InputGenerations recovery_generations_{};
};

}  // namespace lingtu::nav::endpoint
