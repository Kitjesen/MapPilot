#pragma once

#include "body.hpp"

namespace lingtu::driver {

struct ProfileRequest {
  std::string task_id;
  std::string requirement;
};

struct ProfileStatus {
  std::string task_id;
  std::string requirement;
  std::string phase{"idle"};
  std::string requested_profile;
  std::string active_profile;
  std::string reason;
};

// Runs only in the existing driver's loop, while nav supplies a fresh zero command.
class ProfileSwitch {
 public:
  ProfileSwitch(std::string up, std::string down);
  void request(const ProfileRequest &request, Clock::time_point now);
  void step(Body &body, Clock::time_point now, bool zero_command);
  void cancel(std::string reason);
  bool needsCommand() const noexcept {
    return stage_ == Stage::Failed || stage_ == Stage::AwaitEnable;
  }
  void resetBlocked();
  bool holdsMotion() const noexcept;
  const ProfileStatus &status() const noexcept { return status_; }

 private:
  enum class Stage {
    Idle, Inspect, Stop, Ground, Disable, ConfirmDisabled, Switch, Confirm,
    Enable, ConfirmEnabled, Stand, Ready, Unsupported, Failed, AwaitEnable,
  };
  void wait(std::string reason, Clock::time_point now);
  void fail(std::string reason);
  std::string up_;
  std::string down_;
  std::string previous_profile_;
  ProfileStatus status_;
  Stage stage_{Stage::Idle};
  Clock::time_point last_request_{};
  Clock::time_point next_step_{};
  bool posture_requested_{false};
  bool stopped_{false};
  bool motion_changed_{false};
  bool paused_{false};
  std::optional<bool> initial_motors_enabled_;
};

}  // namespace lingtu::driver
