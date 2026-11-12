#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "control/authority.hpp"
#include "safety/stop.hpp"

namespace {

using lingtu::nav::endpoint::ControlAuthority;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testCancelClearsEveryResumableCommand() {
  ControlAuthority authority;
  authority.activatePath();
  authority.acceptTeleop({0.2, 0.0, 0.1});
  authority.setTeleopManualMode(true);

  authority.cancel();

  require(!authority.pathActive(), "cancel must clear the active path");
  require(!authority.teleopRequest().has_value(), "cancel must clear the teleop request");
  require(!authority.teleopManualMode(), "cancel must clear manual mode");
}

void testStopDoesNotLatchButCannotResumeOldMotion() {
  ControlAuthority authority;
  authority.activatePath();
  authority.acceptTeleop({0.2, 0.0, 0.1});

  authority.stop();

  require(!authority.estopLatched(), "ordinary stop must not latch estop");
  require(authority.motionAllowed(), "ordinary stop must allow a new command");
  require(!authority.pathActive(), "ordinary stop must clear the path");
  require(!authority.teleopRequest().has_value(), "ordinary stop must clear teleop");
}

void testOperatorTakeoverBlocksOldAndNewPathsUntilExplicitResume() {
  ControlAuthority authority;
  require(authority.activatePath(), "initial autonomous path must activate");
  require(authority.takeover({0.2, 0.0, 0.1}),
          "operator takeover must be accepted when estop is clear");
  require(authority.operatorHeld(), "takeover must latch control ownership");
  require(!authority.pathActive(), "takeover must preempt the active path");
  require(!authority.activatePath(),
          "a stale planner/path callback must not reactivate autonomy during takeover");

  authority.holdOperator();
  require(authority.operatorHeld(), "manual hold must retain operator ownership");
  require(!authority.teleopRequest().has_value(),
          "manual hold must clear the last velocity sample");
  require(authority.resume(),
          "explicit resume must release the takeover latch when estop is clear");
  require(!authority.operatorHeld(), "explicit resume must release operator ownership");
  require(authority.activatePath(),
          "a fresh autonomous path may activate only after explicit resume");
}

void testEstopRequiresExplicitClearAndNeverResumesOldMotion() {
  ControlAuthority authority;
  authority.activatePath();
  authority.acceptTeleop({0.2, 0.0, 0.1});

  authority.latchEstop("operator_estop");

  require(authority.estopLatched(), "estop must latch");
  require(!authority.motionAllowed(), "latched estop must reject motion");
  require(authority.estopReason() == "operator_estop", "estop reason must be retained");
  require(!authority.pathActive(), "estop must clear the path");
  require(!authority.teleopRequest().has_value(), "estop must clear teleop");

  require(!authority.clearEstop(false), "failed zero publication must not release estop");
  require(authority.estopLatched(), "failed clear must remain latched");

  require(authority.clearEstop(true), "safe clear must release estop");

  require(!authority.estopLatched(), "clear estop must release the latch");
  require(authority.motionAllowed(), "motion may be commanded after clear estop");
  require(!authority.pathActive(), "clear estop must not restore an old path");
  require(!authority.teleopRequest().has_value(), "clear estop must not restore teleop");
}

void testFaultHold() {
  ControlAuthority authority;
  require(authority.activatePath(), "path must activate before the fault");
  authority.hold("stop_confirmation_timeout");
  require(authority.held() && !authority.operatorHeld(), "a fault is not operator takeover");
  require(authority.holdReason() == "stop_confirmation_timeout", "fault cause must survive");
  require(!authority.pathActive() && !authority.activatePath(), "fault must block path activation");
  require(!authority.acceptTeleop({0.2, 0, 0}) && !authority.takeover({0.2, 0, 0}),
          "teleop must not bypass a fault hold");
  authority.hold("zero_publish_failed");
  require(authority.holdReason() == "zero_publish_failed", "a new fault must update the held reason");
  require(authority.resume() && !authority.held(), "confirmed resume must release the fault");
  require(!authority.pathActive() && !authority.teleopRequest(), "resume must not replay old motion");
  require(authority.takeover({0.2, 0, 0}), "operator takeover must still work");
  authority.hold("stop_timeout");
  require(authority.operatorHeld() && authority.holdReason() == "stop_timeout",
          "a fault during takeover must retain the operator and the real fault");
  authority.latchEstop("estop");
  require(!authority.resume(), "resume must not clear estop");
  require(authority.clearEstop(true) && !authority.held(), "confirmed estop clear must leave no stale hold");
}

void testStopTimeoutAndResume() {
  using namespace lingtu::nav::endpoint;
  ControlAuthority authority;
  authority.activatePath();
  auto evidence = StopConfirmationState::TimedOut;
  MotionStopActions actions;
  actions.hold = [&](const std::string &reason) { authority.hold(reason); };
  actions.stop_control = [&] { authority.stop(); };
  actions.cancel_inspection = [](const std::string &) {};
  actions.rolling_segment_active = [] { return false; };
  actions.clear_motion_outputs = [](const std::string &) { return true; };
  actions.suspend_motion_outputs = actions.clear_motion_outputs;
  actions.last_output_sequence = [] { return 1U; };
  actions.confirm_zero = [&](auto, auto) { return evidence; };
  actions.resume_control = [&] { return authority.resume(); };
  double resumed_at = 0.0;
  actions.set_autonomy_request_not_before = [&](double value) { resumed_at = value; };
  MotionStopBarrier stop(true, std::move(actions));
  const auto failed = stop.stopWithoutTerminalCommit();
  require(!failed.accepted && authority.held() && !authority.operatorHeld(),
          "an actual stop timeout must hold motion without inventing an operator");
  require(authority.holdReason() == failed.reason && !authority.activatePath(),
          "stop failure must preserve its reason and block another route");
  require(!stop.resumeAutonomy({{}, authority.held(), 2.0}).accepted && authority.held(),
          "unconfirmed resume must keep the hold");
  evidence = StopConfirmationState::Confirmed;
  require(stop.resumeAutonomy({{}, authority.held(), 3.0}).accepted && !authority.held(),
          "confirmed resume must clear the hold");
  require(resumed_at == 3.0 && !authority.pathActive() && !authority.teleopRequest(),
          "resume must require fresh commands instead of replaying old motion");
}

void testEstopLatchPersistsAcrossAuthorityRestart() {
  const auto path = std::filesystem::temp_directory_path() / "lingtu-test-estop-latch";
  std::error_code ignored;
  std::filesystem::remove(path, ignored);
  lingtu::nav::endpoint::EstopLatchStore store(path.string());

  require(store.persist("operator_estop"), "estop latch must persist");
  const auto loaded = store.load();
  require(loaded.has_value(), "new process must observe persisted estop");
  require(*loaded == "operator_estop", "persisted reason must survive restart");

  ControlAuthority restarted;
  restarted.latchEstop(*loaded);
  require(restarted.estopLatched(), "restart must restore the estop latch");
  require(store.clear(), "explicit clear must remove persisted estop");
  require(!store.load().has_value(), "cleared latch must stay clear on restart");
}

}  // namespace

int main() {
  testStopTimeoutAndResume();
  testFaultHold();
  testCancelClearsEveryResumableCommand();
  testStopDoesNotLatchButCannotResumeOldMotion();
  testOperatorTakeoverBlocksOldAndNewPathsUntilExplicitResume();
  testEstopRequiresExplicitClearAndNeverResumesOldMotion();
  testEstopLatchPersistsAcrossAuthorityRestart();
  std::cout << "test_control_authority passed\n";
  return 0;
}
