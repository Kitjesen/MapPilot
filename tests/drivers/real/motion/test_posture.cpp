#include "posture.hpp"
#include "message/protocol/navigation.hpp"

#include <cstdio>
#include <stdexcept>

namespace {

void check(bool value, const char *reason) {
  if (!value) throw std::runtime_error(reason);
}

class FakeBody final : public lingtu::driver::Body {
 public:
  using Result = lingtu::driver::Result;
  Result refresh() override { throw std::runtime_error("posture must not acquire walking control"); }
  Result move(const lingtu::driver::Velocity &) override {
    throw std::runtime_error("posture must not send walking commands");
  }
  Result stop() noexcept override {
    ++stops;
    Result result;
    result.ok = stop_ok;
    result.transport_ok = true;
    result.accepted = stop_ok;
    result.stop_confirmed = stop_ok;
    return result;
  }
  Result act(lingtu::driver::BodyAction action) override {
    ++actions;
    last_action = action;
    Result result;
    result.ok = action_ok;
    result.transport_ok = action_transport;
    result.accepted = action_ok;
    result.state.reason = action_ok ? "accepted" : "control_busy";
    return result;
  }
  Result inspectControl() override {
    ++inspections;
    Result result;
    result.ok = inspection_ok;
    result.transport_ok = inspection_ok;
    result.state.connected = inspection_ok;
    result.state.motors_enabled = motors;
    result.state.critical_fault = fault;
    result.state.fsm = fsm;
    result.state.lease_valid = lease;
    result.state.owner = owner;
    result.state.owner_id = owner == "grpc" ? lingtu::driver::kDriverMotionPrincipal : "";
    result.state.reason = inspection_ok ? "observed" : "transport_error";
    return result;
  }
  Result motorPower(bool enabled) override {
    if (enabled) ++enables;
    else ++disables;
    Result result;
    result.ok = result.accepted = power_ok;
    result.transport_ok = power_transport;
    result.state.reason = power_ok ? "accepted" : "motor_power_failed";
    if (power_ok && power_readback) {
      motors = enabled;
      lease = false;
    }
    return result;
  }
  lingtu::driver::ProfileState readProfile() override {
    ++profile_reads;
    lingtu::driver::ProfileState result;
    result.supported = true;
    result.fresh = profile_fresh;
    result.current = current_profile;
    result.available = {"ordinary", "stairs"};
    return result;
  }
  Result switchProfile(const std::string &profile) override {
    ++switches;
    Result result;
    result.ok = result.accepted = action_ok;
    result.transport_ok = action_transport;
    result.state.reason = action_ok ? "accepted" : "profile_failed";
    if (action_ok && profile_readback) current_profile = profile;
    return result;
  }
  lingtu::driver::Capabilities capabilities() const noexcept override { return {true, true, false, false, true, true}; }
  lingtu::message::ExternalState externalState() const override {
    lingtu::message::ExternalState state;
    state.session_id = external_session;
    state.selected = !external_session.empty();
    state.active = external_active;
    state.fresh = true;
    state.motors_enabled = motors;
    state.motor_disable_confirmed = !motors;
    return state;
  }
  bool externalSelected() const override { return !external_session.empty(); }
  Result enterExternal(const std::string &session) override {
    ++external_enters;
    external_session = session;
    Result result;
    result.ok = result.transport_ok = result.accepted = true;
    result.state.reason = "external_entered";
    return result;
  }
  Result enableExternal() override {
    ++external_enables;
    motors = true;
    Result result;
    result.ok = result.transport_ok = result.accepted = true;
    return result;
  }
  Result exitExternal() override {
    ++external_exits;
    external_session.clear();
    Result result;
    result.ok = result.transport_ok = result.accepted = true;
    return result;
  }
  lingtu::driver::BodyState state() const override { return {}; }
  lingtu::driver::HealthState health() const override { return {}; }
  lingtu::driver::AdapterDiagnostics diagnostics() const override { return {}; }

  std::string fsm{"grounded"};
  std::string owner{"none"};
  bool lease{false};
  bool motors{true};
  bool fault{false};
  bool stop_ok{true};
  bool action_ok{true};
  bool action_transport{true};
  bool inspection_ok{true};
  bool power_ok{true}, power_transport{true}, power_readback{true};
  bool profile_fresh{true}, profile_readback{true};
  std::string current_profile{"ordinary"};
  unsigned enables{0}, disables{0}, switches{0}, profile_reads{0};
  unsigned actions{0};
  unsigned stops{0};
  unsigned inspections{0};
  unsigned external_enters{0}, external_enables{0}, external_exits{0};
  std::string external_session;
  bool external_active{true};
  lingtu::driver::BodyAction last_action{lingtu::driver::BodyAction::Damp};
};

lingtu::message::PostureRequest stand() {
  return {"nav-boot", "sdk-client", "stand-1",
          static_cast<int>(lingtu::message::NavigationCommandKind::Stand), 0, "", ""};
}

void groundedStandAndReplay() {
  FakeBody body;
  lingtu::driver::PostureControl control;
  const auto reply = control.request(stand(), body, "", false, false);
  check(reply.ack.final_result && reply.ack.accepted, "Grounded stand must be accepted without walking lease");
  check(body.inspections == 1 && body.stops == 0 && body.actions == 1,
        "Grounded action must use one real status query and no walking stop");
  check(control.holdsMotion(), "posture action must hold velocity and lease acquisition");
  lingtu::driver::BodyState standing;
  standing.fresh = true;
  standing.posture = lingtu::driver::Posture::Standing;
  control.observeBody(standing);
  check(control.holdsMotion(), "pre-action standing sample cannot release posture hold");
  standing.posture = lingtu::driver::Posture::Transitioning;
  standing.observed_at = lingtu::driver::Clock::now();
  control.observeBody(standing);
  check(control.holdsMotion(), "actual transitioning sample must retain posture hold");
  standing.posture = lingtu::driver::Posture::Standing;
  standing.observed_at = lingtu::driver::Clock::now();
  control.observeBody(standing);
  check(!control.holdsMotion(), "actual standing after stand ACK must permit walking acquisition");
  check(control.request(stand(), body, "", false, false).ack.accepted, "retry must replay ACK");
  check(body.actions == 1 && body.inspections == 1, "retry must not repeat body action");
  auto conflict = stand();
  conflict.action = static_cast<int>(lingtu::message::NavigationCommandKind::Sit);
  check(control.request(conflict, body, "", false, false).ack.reason == "request_id_conflict",
        "request identity cannot change action");
}

void standingSitRequiresStop() {
  FakeBody body;
  body.fsm = "walking";
  body.owner = "grpc";
  body.lease = true;
  lingtu::driver::PostureControl control;
  auto request = stand();
  request.action = static_cast<int>(lingtu::message::NavigationCommandKind::Sit);
  const auto inspection = control.request(request, body, "nav-boot", true, false);
  check(!inspection.ack.final_result && inspection.ack.reason == "stop_required" && body.actions == 0,
        "walking body must wait for native stop barrier");
  request.phase = 1;
  const auto result = control.request(request, body, "nav-boot", true, false);
  check(result.ack.accepted && body.stops == 1 && body.actions == 1,
        "stopped phase must stop the vendor endpoint before the action");
  check(body.last_action == lingtu::driver::BodyAction::Sit, "sit must map to vendor sit action");
}

void lostActionAckStaysUnknown() {
  FakeBody body;
  body.action_transport = false;
  lingtu::driver::PostureControl control;
  const auto reply = control.request(stand(), body, "", false, false);
  check(!reply.ack.accepted && reply.ack.reason == "posture_action_ack_unknown" && body.actions == 1,
        "lost action ACK cannot be reported as a definite Brainstem rejection");
  check(control.holdsMotion(), "unknown action outcome must hold motion");
  auto cancellation = stand();
  cancellation.phase = 2;
  check(control.request(cancellation, body, "", false, false).ack.reason == "posture_action_ack_unknown",
        "cancel cannot turn a lost Brainstem action ACK into a confirmed cancellation");
  body.action_transport = true;
  body.fsm = "standing";
  auto recovery = stand();
  recovery.request_id = "stand-recovery";
  const auto prepared = control.request(recovery, body, "", false, false);
  check(!prepared.ack.final_result && prepared.ack.reason == "stop_required" && control.holdsMotion(),
        "preparation must preserve an earlier unknown-action hold");
  recovery.phase = 1;
  check(control.request(recovery, body, "nav-boot", true, false).ack.accepted,
        "explicit recovery can execute while retaining the earlier hold");
  lingtu::driver::BodyState standing;
  standing.fresh = true;
  standing.posture = lingtu::driver::Posture::Standing;
  standing.observed_at = lingtu::driver::Clock::now();
  control.observeBody(standing);
  check(!control.holdsMotion(), "successful explicit recovery releases the hold");
}

void cancelNeverExecutesQueuedAction() {
  for (const bool prepared : {false, true}) {
    FakeBody body;
    body.fsm = "walking";
    lingtu::driver::PostureControl control;
    auto request = stand();
    if (prepared) (void)control.request(request, body, "nav-boot", true, false);
    request.phase = 2;
    const auto cancelled = control.request(request, body, "nav-boot", true, false);
    check(!cancelled.ack.accepted && cancelled.ack.reason == "posture_cancelled",
          "cancel must acknowledge only the request cancellation");
    request.phase = prepared ? 1 : 0;
    check(!control.request(request, body, "nav-boot", true, false).ack.accepted && body.actions == 0,
          "queued inspect/execute after cancel must not act");
    check(!control.holdsMotion(), "cancel before execution cannot hold motion");
  }
  FakeBody body;
  lingtu::driver::PostureControl control;
  auto request = stand();
  check(control.request(request, body, "", false, false).ack.accepted, "stand must be accepted");
  request.phase = 2;
  check(control.request(request, body, "", false, false).ack.reason == "posture_already_accepted",
        "cancel cannot claim to undo an already submitted Brainstem action");
  lingtu::driver::BodyState standing;
  standing.fresh = true;
  standing.posture = lingtu::driver::Posture::Standing;
  standing.observed_at = lingtu::driver::Clock::now();
  control.observeBody(standing);
  check(!control.holdsMotion() && body.actions == 1, "late cancellation must preserve standing recovery");
}

void failedPrerequisitesNeverAct() {
  for (int scenario = 0; scenario < 8; ++scenario) {
    FakeBody body;
    lingtu::driver::PostureControl control;
    auto request = stand();
    if (scenario == 0) body.owner = "yunzhuo";
    if (scenario == 1) body.motors = false;
    if (scenario == 2) body.fault = true;
    if (scenario == 3) body.inspection_ok = false;
    if (scenario == 4) body.fsm = "transitioning";
    if (scenario >= 5) {
      body.fsm = "standing";
      (void)control.request(request, body, "", false, false);
      request.phase = 1;
    }
    if (scenario == 7) body.stop_ok = false;
    const auto result = control.request(request, body,
        scenario == 5 ? "other-nav-boot" : "nav-boot", scenario != 6, false);
    check(result.ack.final_result && !result.ack.accepted && body.actions == 0,
          "failed prerequisite must prevent physical action");
    check(control.holdsMotion() == (scenario == 7), "only attempted stop may add a hold");
    lingtu::driver::BodyState standing;
    standing.fresh = true;
    standing.posture = lingtu::driver::Posture::Standing;
    standing.observed_at = lingtu::driver::Clock::now();
    control.observeBody(standing);
    check(control.holdsMotion() == (scenario == 7), "unexecuted rejection must not latch motion");
  }
  FakeBody body;
  lingtu::driver::PostureControl control;
  check(!control.request(stand(), body, "nav-boot", true, true).ack.accepted && body.actions == 0,
        "profile transition must exclude posture action");
}

void motorGateCommands() {
  using Kind = lingtu::message::NavigationCommandKind;
  FakeBody body;
  body.fsm = "recovering";
  body.fault = true;
  body.lease = true;
  body.owner = "grpc";
  lingtu::driver::PostureControl control;
  auto disable = stand();
  disable.action = static_cast<int>(Kind::DisableMotors);
  disable.request_id = "disable";
  const auto disabled = control.request(disable, body, "", false, true);
  check(disabled.ack.accepted && body.disables == 1 && !body.motors &&
      body.stops == 0 && body.actions == 0, "disable bypasses stop, fault, recovery and profile barriers");
  check(control.request(disable, body, "", false, true).ack.accepted && body.disables == 1,
        "disable retry replays ACK without another hardware command");
  auto enable = disable;
  enable.request_id = "enable";
  enable.action = static_cast<int>(Kind::EnableMotors);
  body.fsm = "grounded";
  body.fault = false;
  body.owner = "none";
  const auto enabled = control.request(enable, body, "", false, false);
  check(enabled.ack.accepted && body.enables == 1 && body.motors && body.actions == 0 && body.stops == 0,
        "enable opens only the motor gate without stand or walking acquisition");
  check(control.holdsMotion(), "enable keeps walking held until a fresh standing observation");
  lingtu::driver::BodyState standing;
  standing.fresh = true;
  standing.posture = lingtu::driver::Posture::Standing;
  control.observeBody(standing);
  check(control.holdsMotion(), "old standing cannot release enable hold");
  standing.observed_at = lingtu::driver::Clock::now();
  control.observeBody(standing);
  check(!control.holdsMotion(), "fresh actual standing releases enable hold");
}

void motorGateOwnerAndUnknown() {
  using Kind = lingtu::message::NavigationCommandKind;
  for (const bool unavailable : {false, true}) {
    FakeBody body;
    body.owner = "yunzhuo";
    body.inspection_ok = !unavailable;
    lingtu::driver::PostureControl control;
    auto disable = stand();
    disable.action = static_cast<int>(Kind::DisableMotors);
    const auto reply = control.request(disable, body, "", false, false);
    check(!reply.ack.accepted && body.disables == 0 && body.stops == 0,
          "foreign physical owner and failed owner inspection prevent motor disable");
    check(reply.ack.reason == (unavailable ? "transport_error" : "control_busy"),
          "pre-command inspection failure reports a definite rejection");
  }
  for (const bool enable : {false, true}) {
    for (const bool lost_ack : {false, true}) {
      FakeBody body;
      body.motors = !enable;
      body.power_readback = false;
      body.power_transport = !lost_ack;
      lingtu::driver::PostureControl control;
      auto request = stand();
      request.action = static_cast<int>(enable ? Kind::EnableMotors : Kind::DisableMotors);
      const auto reply = control.request(request, body, "", false, false);
      check(!reply.ack.accepted && reply.ack.reason == "posture_action_ack_unknown" && control.holdsMotion(),
            "lost ACK or unconfirmed gate stays unknown and holds walking");
      check(body.enables + body.disables == 1, "unknown outcome sends exactly one motor command");
      (void)control.request(request, body, "", false, false);
      check(body.enables + body.disables == 1, "unknown outcome is cached without automatic retry");
    }
  }
}

void publicProfileSwitch() {
  using Kind = lingtu::message::NavigationCommandKind;
  FakeBody body;
  body.motors = false;
  lingtu::driver::PostureControl control;
  auto request = stand();
  request.action = static_cast<int>(Kind::SwitchProfile);
  request.profile_name = "stairs";
  const auto reply = control.request(request, body, "", false, false);
  check(reply.ack.accepted && body.switches == 1 && body.profile_reads == 2 &&
      body.actions == 0 && body.enables == 0 && body.stops == 0,
      "public switch only changes profile and confirms through independent readback");
  check(control.request(request, body, "", false, false).ack.accepted && body.switches == 1,
        "same public profile request replays without switching again");
  auto conflict = request;
  conflict.profile_name = "ordinary";
  check(control.request(conflict, body, "", false, false).ack.reason == "request_id_conflict",
        "profile target participates in request identity");
  request.request_id = "same-target";
  check(control.request(request, body, "", false, false).ack.accepted && body.switches == 1,
        "already active profile uses readback without another switch");
}

void publicProfilePrerequisitesAndUnknown() {
  using Kind = lingtu::message::NavigationCommandKind;
  for (int scenario = 0; scenario < 7; ++scenario) {
    FakeBody body;
    body.motors = false;
    lingtu::driver::PostureControl control;
    auto request = stand();
    request.action = static_cast<int>(Kind::SwitchProfile);
    request.profile_name = "stairs";
    if (scenario == 0) body.motors = true;
    if (scenario == 1) body.fsm = "standing";
    if (scenario == 2) body.owner = "yunzhuo";
    if (scenario == 3) body.lease = true;
    if (scenario == 4) body.fault = true;
    if (scenario == 5) body.profile_fresh = false;
    if (scenario == 6) request.profile_name = "missing";
    check(!control.request(request, body, "", false, false).ack.accepted && body.switches == 0 &&
        body.actions == 0 && body.enables == 0, "profile prerequisites prevent hardware action");
  }
  FakeBody body;
  body.motors = false;
  body.profile_readback = false;
  lingtu::driver::PostureControl control;
  auto request = stand();
  request.action = static_cast<int>(Kind::SwitchProfile);
  request.profile_name = "stairs";
  const auto reply = control.request(request, body, "", false, false);
  check(reply.ack.reason == "posture_action_ack_unknown" && body.switches == 1 && !reply.ack.accepted,
        "accepted switch without matching independent readback stays unknown");
  (void)control.request(request, body, "", false, false);
  check(body.switches == 1, "unknown switch cannot repeat the hardware change");
}

void externalControlLifecycle() {
  using Kind = lingtu::message::NavigationCommandKind;
  FakeBody body;
  body.motors = false;
  lingtu::driver::PostureControl control;
  auto enter = stand();
  enter.request_id = "external-session";
  enter.action = static_cast<int>(Kind::EnterExternal);
  const auto entered = control.request(enter, body, "", false, false);
  check(entered.ack.accepted && body.external_enters == 1 && body.actions == 0 && body.stops == 0,
        "external entry must preserve Grounded disabled without builtin commands");
  check(control.request(enter, body, "", false, false).ack.accepted && body.external_enters == 1,
        "external entry retry must replay");
  auto enable = enter;
  enable.action = static_cast<int>(Kind::EnableExternal);
  enable.request_id = "external-enable";
  enable.external_session_id = "other-session";
  check(control.request(enable, body, "", false, false).ack.reason == "external_session_mismatch" &&
        body.external_enables == 0, "other session must not enable external output");
  enable.request_id = "external-enable-correct";
  enable.external_session_id = "external-session";
  check(control.request(enable, body, "", false, false).ack.accepted && body.external_enables == 1,
        "matching session may enable external output");
  check(control.request(stand(), body, "", false, false).ack.reason == "external_control_selected" &&
        body.actions == 0, "builtin posture cannot run while external selected");
  auto disable = enable;
  disable.action = static_cast<int>(Kind::DisableMotors);
  disable.request_id = "external-disable";
  check(control.request(disable, body, "", false, false).ack.accepted && body.disables == 1 &&
        !body.motors, "external disable must use its session and confirm the closed motor gate");
  auto exit = enable;
  exit.action = static_cast<int>(Kind::ExitExternal);
  exit.request_id = "external-exit";
  check(control.request(exit, body, "", false, false).ack.accepted && body.external_exits == 1 &&
        body.actions == 0 && body.enables == 0, "exit must not stand or enable builtin control");
}

void externalEntryPrerequisites() {
  using Kind = lingtu::message::NavigationCommandKind;
  for (int scenario = 0; scenario < 5; ++scenario) {
    FakeBody body;
    body.motors = false;
    if (scenario == 0) body.motors = true;
    if (scenario == 1) body.fsm = "standing";
    if (scenario == 2) body.owner = "yunzhuo";
    if (scenario == 3) body.lease = true;
    if (scenario == 4) body.fault = true;
    auto request = stand();
    request.action = static_cast<int>(Kind::EnterExternal);
    lingtu::driver::PostureControl control;
    check(!control.request(request, body, "", false, false).ack.accepted && body.external_enters == 0,
          "invalid physical state must prevent external entry");
  }
  FakeBody expired;
  expired.external_session = "expired-session";
  expired.external_active = false;
  expired.motors = false;
  expired.owner = "external";
  expired.fsm = "standing";
  auto request = stand();
  request.action = static_cast<int>(Kind::EnterExternal);
  request.request_id = "recovered-session";
  lingtu::driver::PostureControl control;
  check(control.request(request, expired, "", false, false).ack.accepted &&
        expired.external_session == "recovered-session" && expired.external_enters == 1,
        "expired external selection with confirmed closed gate may acquire a new session");
}

}  // namespace

void rejectedRequestPreservesHold() {
  FakeBody body;
  lingtu::driver::PostureControl control;
  auto accepted = stand();
  check(control.request(accepted, body, "", false, false).ack.accepted, "stand accepted");
  auto rejected = accepted;
  rejected.request_id += "-bad";
  rejected.action = -1;
  check(!control.request(rejected, body, "", false, false).ack.accepted, "unknown rejected");
  check(control.holdsMotion(), "rejection preserves existing hold");
  rejected.phase = 2;
  (void)control.request(rejected, body, "", false, false);
  lingtu::driver::BodyState standing;
  standing.fresh = true;
  standing.posture = lingtu::driver::Posture::Standing;
  standing.observed_at = lingtu::driver::Clock::now();
  control.observeBody(standing);
  check(!control.holdsMotion(), "unrelated rejection/cancel preserves pending recovery");
  accepted.phase = 2;
  (void)control.request(accepted, body, "", false, false);
  check(!control.holdsMotion(), "late cancel cannot relatch completed posture");
  check(body.actions == 1, "replays never execute again");
}

int main() {
  try {
    rejectedRequestPreservesHold();
    groundedStandAndReplay();
    standingSitRequiresStop();
    failedPrerequisitesNeverAct();
    cancelNeverExecutesQueuedAction();
    lostActionAckStaysUnknown();
    motorGateCommands();
    motorGateOwnerAndUnknown();
    publicProfileSwitch();
    publicProfilePrerequisitesAndUnknown();
    externalControlLifecycle();
    externalEntryPrerequisites();
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_posture: %s\n", error.what());
    return 1;
  }
}
