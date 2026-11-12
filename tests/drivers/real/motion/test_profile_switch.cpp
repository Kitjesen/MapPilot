#include "profile_switch.hpp"
#include "posture.hpp"
#include "message/protocol/navigation.hpp"

#include <cstdio>
#include <stdexcept>
#include <vector>

using namespace lingtu::driver;
using namespace std::chrono_literals;
namespace message = lingtu::message;

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

class Robot final : public Body {
 public:
  Robot() {
    body.fresh = true;
    body.posture = Posture::Standing;
    observed.supported = observed.fresh = observed.requires_grounded = true;
    observed.requires_disabled = true;
    observed.motors_enabled = true;
    observed.current = "walk-custom";
    observed.available = {"walk-custom", "stairs-v2"};
  }
  Result refresh() override {
    Result result;
    result.ok = result.accepted = result.transport_ok = result.state.ready =
        body.posture == Posture::Standing;
    return result;
  }
  Result move(const Velocity &) override { return refresh(); }
  Result stop() noexcept override {
    ++stops;
    calls.push_back("stop");
    observed.control_free = true;
    Result result;
    result.transport_ok = true;
    result.ok = result.accepted = result.stop_confirmed = stop_ok;
    return result;
  }
  Result act(BodyAction action) override {
    if (action == BodyAction::Sit) { ++sits; calls.push_back("sit"); }
    if (action == BodyAction::Stand) { ++stands; calls.push_back("stand"); }
    body.posture = Posture::Transitioning;
    Result result;
    result.ok = result.accepted = result.transport_ok = true;
    return result;
  }
  Result motorPower(bool enabled) override {
    if (enabled) { ++enables; calls.push_back("enable"); }
    else { ++disables; calls.push_back("disable"); }
    Result result;
    result.transport_ok = power_transport;
    result.ok = result.accepted = enabled ? enable_ok : disable_ok;
    result.error = result.state.reason = result.ok ? "accepted" : "motor_power_failed";
    if (result.ok && power_readback) observed.motors_enabled = enabled;
    return result;
  }
  Result inspectControl() override {
    Result result;
    result.ok = result.transport_ok = true;
    result.state.motors_enabled = observed.motors_enabled.value_or(false);
    result.state.fsm = observed.grounded ? "grounded" : "standing";
    result.state.owner = observed.control_free ? "none" : "yunzhuo";
    return result;
  }
  Capabilities capabilities() const noexcept override { return {true, true, false, false, true}; }
  BodyState state() const override { return body; }
  HealthState health() const override { return {}; }
  AdapterDiagnostics diagnostics() const override { return {}; }
  ProfileState readProfile() override { ++reads; return observed; }
  Result switchProfile(const std::string &name) override {
    ++switches;
    calls.push_back("switch");
    requested = name;
    check(!observed.requires_grounded || observed.grounded, "switch requires actual Grounded");
    check(!observed.requires_disabled ||
        (observed.motors_enabled == false && observed.control_free), "switch requires disabled and unowned");
    Result result;
    result.ok = result.accepted = result.transport_ok = !reject_switch;
    result.error = result.state.reason = reject_switch ? "profile_failure" : "accepted";
    if (!reject_switch && profile_readback) observed.current = name;
    return result;
  }
  void grounded() { observed.grounded = true; body.posture = Posture::Sitting; }
  void standing() { observed.grounded = false; body.posture = Posture::Standing; }
  BodyState body;
  ProfileState observed;
  std::vector<std::string> calls;
  std::string requested;
  int stops{0}, sits{0}, stands{0}, switches{0}, reads{0}, enables{0}, disables{0};
  bool reject_switch{false}, profile_readback{true};
  bool enable_ok{true}, disable_ok{true}, power_transport{true}, power_readback{true};
  bool stop_ok{true};
};

struct Flow {
  Robot robot;
  ProfileSwitch transition{"stairs-v2", "stairs-v2"};
  Clock::time_point now{Clock::now()};
  void tick(const char *requirement = "stairs_up", const char *task = "task") {
    now += 200ms;
    transition.request({task, requirement}, now);
    transition.step(robot, now, true);
  }
  void switched() {
    tick();
    robot.grounded();
    tick(); // Disable, then wait for independent gate feedback.
    tick(); // Switch, then wait for independent profile feedback.
  }
};

void handoffRestoreAndReadback() {
  Flow flow;
  flow.tick();
  check(flow.robot.stops == 1 && flow.robot.sits == 1 && flow.robot.switches == 0,
        "stop and sit precede profile switching");
  flow.tick();
  check(flow.robot.disables == 0, "Sit ACK alone is not Grounded feedback");
  flow.robot.grounded();
  flow.tick();
  check(flow.robot.disables == 1 && flow.robot.switches == 0, "disable requires independent readback");
  flow.tick();
  check(flow.robot.switches == 1 && flow.robot.enables == 0, "profile ACK requires independent readback");
  flow.tick();
  check(flow.robot.enables == 1 && flow.robot.stands == 0, "enable requires independent readback");
  flow.tick();
  check(flow.robot.stands == 1 && flow.transition.holdsMotion(), "Stand ACK does not resume walking");
  flow.robot.standing();
  flow.tick();
  check(flow.transition.status().phase == "ready" && !flow.transition.holdsMotion(), "readiness is confirmed");
  check(flow.robot.calls == std::vector<std::string>{"stop", "sit", "disable", "switch", "enable", "stand"},
        "internal profile sequence preserves gate ordering");
  flow.tick("stairs_down");
  check(flow.transition.status().phase == "ready" && flow.robot.sits == 1,
        "same active profile avoids redundant body transitions");
  flow.tick("restore");
  check(flow.transition.status().requested_profile == "walk-custom", "restore captures the actual previous profile");
  flow.robot.grounded();
  flow.tick("restore");
  flow.tick("restore");
  check(flow.robot.requested == "walk-custom", "restore uses the captured profile");
  flow.tick("restore");
  flow.tick("restore");
  flow.robot.standing();
  flow.tick("restore");
  check(flow.transition.status().phase == "ready", "restore waits for profile, enable and standing");
}

void groundedFreeSkipsWalkingStop() {
  Flow flow;
  flow.robot.grounded();
  flow.robot.observed.control_free = true;
  flow.tick();
  check(flow.robot.stops == 0 && flow.robot.sits == 0 && flow.robot.disables == 1,
        "Grounded enabled body has no walking lease to stop");
  flow.tick();
  check(flow.robot.switches == 1, "Grounded free body reaches profile switch");
}

void initiallyDisabledStaysDisabled() {
  for (const bool same_profile : {false, true}) {
    Flow flow;
    flow.robot.grounded();
    flow.robot.observed.control_free = true;
    flow.robot.observed.motors_enabled = false;
    if (same_profile) flow.robot.observed.current = "stairs-v2";
    flow.tick();
    flow.tick();
    check(flow.transition.holdsMotion() && flow.transition.needsCommand() &&
        flow.transition.status().phase == "waiting" &&
        flow.transition.status().reason == "motors_initially_disabled", "disabled body awaits explicit enable");
    flow.tick("stairs_down", "new-task");
    check(flow.robot.enables == 0 && flow.robot.stands == 0 && flow.robot.disables == 0 && flow.robot.stops == 0,
          "nav requests cannot open an initially disabled gate");
  }
}

void faultsLatchWithoutReopening() {
  for (int scenario = 0; scenario < 8; ++scenario) {
    Flow flow;
    if (scenario == 0) flow.robot.observed.motors_enabled.reset();
    if (scenario == 1) flow.robot.disable_ok = false;
    if (scenario == 2) flow.robot.profile_readback = false;
    if (scenario == 3) flow.robot.enable_ok = false;
    if (scenario == 4) flow.robot.power_readback = false;
    if (scenario == 5) flow.robot.reject_switch = true;
    if (scenario == 6) flow.robot.power_transport = false;
    flow.switched();
    if (scenario == 7) flow.robot.observed.critical_fault = true;
    flow.tick();
    flow.tick();
    check(flow.transition.status().phase == "failed" && flow.transition.needsCommand() &&
              flow.transition.holdsMotion() == (scenario != 0),
          "only failures after a body operation retain the hold");
    const auto before = flow.robot.calls;
    flow.robot.enable_ok = flow.robot.disable_ok = flow.robot.profile_readback = flow.robot.power_readback = true;
    flow.robot.observed.critical_fault = false;
    flow.tick("restore", "another-task");
    flow.tick();
    check(flow.robot.calls == before, "new or repeated nav profile requests cannot retry failed actions");
  }
}

void disableCancelsPendingEnable() {
  Flow flow;
  flow.switched();
  check(flow.robot.switches == 1 && flow.robot.enables == 0, "cancel occurs before pending enable");
  PostureControl control;
  message::PostureRequest disable{"boot", "sdk", "disable", static_cast<int>(message::NavigationCommandKind::DisableMotors), 0, ""};
  const auto reply = control.request(disable, flow.robot, "", false, flow.transition.holdsMotion());
  check(reply.ack.accepted && reply.result && reply.result->calls > 0, "explicit disable bypasses profile hold");
  flow.transition.cancel("operator_motor_disable");
  flow.tick();
  flow.tick("restore", "another-task");
  check(flow.robot.enables == 0 && flow.robot.stands == 0 && flow.robot.observed.motors_enabled == false,
        "external disable cannot be overridden by queued internal enable");
}

void explicitRecoveryAndReplay() {
  Flow flow;
  flow.robot.reject_switch = true;
  flow.switched();
  check(flow.transition.needsCommand(), "automatic profile failure latches");
  flow.robot.reject_switch = false;
  PostureControl control;
  message::PostureRequest enable{"boot", "sdk", "enable", static_cast<int>(message::NavigationCommandKind::EnableMotors), 0, ""};
  const auto reply = control.request(enable, flow.robot, "", false,
      flow.transition.holdsMotion() && !flow.transition.needsCommand());
  check(reply.new_request && reply.ack.accepted && flow.robot.enables == 1 && flow.robot.stands == 0,
        "fresh explicit enable recovers only the motor gate");
  if (reply.new_request && (!reply.ack.final_result || reply.ack.accepted)) flow.transition.resetBlocked();
  flow.tick();
  check(!flow.transition.holdsMotion() && flow.robot.enables == 1, "same old profile heartbeat cannot restart after recovery");
  flow.transition.cancel("later_failure");
  const auto replay = control.request(enable, flow.robot, "", false, false);
  if (replay.new_request && (!replay.ack.final_result || replay.ack.accepted)) flow.transition.resetBlocked();
  check(!replay.new_request && replay.ack.accepted && flow.robot.enables == 1,
        "completed public request replay does not re-enable");
  // Re-enter a different flow, then fail it to verify a replay cannot clear its hold.
  flow.tick("stairs_up", "next-task");
  flow.transition.cancel("later_failure");
  const auto again = control.request(enable, flow.robot, "", false, false);
  if (again.new_request && (!again.ack.final_result || again.ack.accepted)) flow.transition.resetBlocked();
  check(flow.transition.needsCommand(), "cached public ACK cannot clear a later failure");
}

void staleRequestAndPause() {
  Flow nonzero;
  nonzero.transition.request({"task", "stairs_up"}, nonzero.now);
  nonzero.transition.step(nonzero.robot, nonzero.now, false);
  check(nonzero.robot.stops == 1 && nonzero.robot.sits == 0,
        "nonzero nav input must stop before body preparation");
  Flow unconfirmed;
  unconfirmed.robot.stop_ok = false;
  unconfirmed.transition.request({"task", "stairs_up"}, unconfirmed.now);
  unconfirmed.transition.step(unconfirmed.robot, unconfirmed.now, false);
  check(unconfirmed.transition.needsCommand() && unconfirmed.robot.sits == 0,
        "unconfirmed stop latches failure without proceeding to body preparation");
  Flow flow;
  flow.tick();
  flow.robot.grounded();
  flow.transition.step(flow.robot, flow.now + 2s, true);
  check(flow.robot.disables == 0 && flow.transition.holdsMotion(), "stale nav profile request holds preparation");
  flow.now += 4s;
  flow.tick();
  check(flow.robot.disables == 1, "fresh same-task heartbeat resumes preparation");
  flow.transition.request({"task", "hold"}, flow.now + 1ms);
  flow.now += 500ms;
  flow.transition.step(flow.robot, flow.now, true);
  check(flow.robot.switches == 0 && flow.transition.status().reason == "profile_paused", "pause suspends body actions");
  flow.tick();
  check(flow.robot.switches == 1, "same-task explicit resume continues retained preparation");
}

void unsupportedAndOtherAdapters() {
  Flow flow;
  flow.tick("restore");
  check(flow.transition.status().reason == "previous_profile_unknown", "fresh driver cannot invent previous profile");
  ProfileSwitch absent("", "");
  absent.request({"task", "stairs_up"}, flow.now);
  absent.step(flow.robot, flow.now, true);
  check(absent.status().phase == "unsupported" && flow.robot.sits == 0, "unconfigured stairs do not change body");
  ProfileSwitch unsupported("stairs-v2", "stairs-v2");
  flow.robot.observed = {};
  unsupported.request({"task", "stairs_up"}, flow.now);
  unsupported.step(flow.robot, flow.now, true);
  check(unsupported.status().phase == "unsupported", "unsupported profile remains unsupported");
  Flow other;
  other.robot.observed.requires_disabled = false;
  other.robot.observed.requires_grounded = false;
  other.tick();
  other.tick();
  check(other.transition.status().phase == "ready" && other.robot.disables == 0 && other.robot.enables == 0,
        "adapters without motor-gate prerequisites preserve direct switching");
}

void rejectedProfileDoesNotHold() {
  Robot robot;
  ProfileSwitch flow("stairs-v2", "stairs-v2");
  auto now = Clock::now();
  robot.observed.fresh = false;
  flow.request({"bad", "stairs_up"}, now);
  flow.step(robot, now, false);
  check(flow.status().phase == "failed" && !flow.holdsMotion(), "read failure before action must not hold");
  check(robot.calls.empty(), "read failure must not stop or alter the body");
  robot.observed.fresh = true;
  flow.request({"retry", "stairs_up"}, now + 1s);
  flow.step(robot, now + 1s, true);
  check(robot.stops > 0 && flow.holdsMotion(), "new request can start an actual transition");
}
}  // namespace

int main() {
  try {
    rejectedProfileDoesNotHold();
    handoffRestoreAndReadback();
    groundedFreeSkipsWalkingStop();
    initiallyDisabledStaysDisabled();
    faultsLatchWithoutReopening();
    disableCancelsPendingEnable();
    explicitRecoveryAndReplay();
    staleRequestAndPause();
    unsupportedAndOtherAdapters();
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_profile_switch: %s\n", error.what());
    return 1;
  }
}
