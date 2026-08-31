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
  authority.acceptTeleop({0.2, 0.0, 0.1}, 10.0);
  authority.setTeleopManualMode(true);

  authority.cancel();

  require(!authority.pathActive(), "cancel must clear the active path");
  require(!authority.teleopRequest().has_value(), "cancel must clear the teleop request");
  require(authority.teleopStampSeconds() == 0.0, "cancel must clear teleop freshness");
  require(!authority.teleopManualMode(), "cancel must clear manual mode");
}

void testStopDoesNotLatchButCannotResumeOldMotion() {
  ControlAuthority authority;
  authority.activatePath();
  authority.acceptTeleop({0.2, 0.0, 0.1}, 10.0);

  authority.stop();

  require(!authority.estopLatched(), "ordinary stop must not latch estop");
  require(authority.motionAllowed(), "ordinary stop must allow a new command");
  require(!authority.pathActive(), "ordinary stop must clear the path");
  require(!authority.teleopRequest().has_value(), "ordinary stop must clear teleop");
}

void testOperatorTakeoverBlocksOldAndNewPathsUntilExplicitResume() {
  ControlAuthority authority;
  require(authority.activatePath(), "initial autonomous path must activate");
  require(authority.beginOperatorTakeover({0.2, 0.0, 0.1}, 10.0),
          "operator takeover must be accepted when estop is clear");
  require(authority.operatorTakeoverLatched(), "takeover must latch control ownership");
  require(!authority.pathActive(), "takeover must preempt the active path");
  require(!authority.activatePath(),
          "a stale planner/path callback must not reactivate autonomy during takeover");

  authority.holdOperatorTakeover();
  require(authority.operatorTakeoverLatched(), "manual hold must retain operator ownership");
  require(!authority.teleopRequest().has_value(),
          "manual hold must clear the last velocity sample");
  require(authority.resumeMotion(),
          "explicit resume must release the takeover latch when estop is clear");
  require(!authority.operatorTakeoverLatched(), "explicit resume must release operator ownership");
  require(authority.activatePath(),
          "a fresh autonomous path may activate only after explicit resume");
}

void testEstopRequiresExplicitClearAndNeverResumesOldMotion() {
  ControlAuthority authority;
  authority.activatePath();
  authority.acceptTeleop({0.2, 0.0, 0.1}, 10.0);

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
  testCancelClearsEveryResumableCommand();
  testStopDoesNotLatchButCannotResumeOldMotion();
  testOperatorTakeoverBlocksOldAndNewPathsUntilExplicitResume();
  testEstopRequiresExplicitClearAndNeverResumesOldMotion();
  testEstopLatchPersistsAcrossAuthorityRestart();
  std::cout << "test_control_authority passed\n";
  return 0;
}
