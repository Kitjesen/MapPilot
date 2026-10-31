#include "profile_switch.hpp"

#include <cstdio>
#include <stdexcept>

using namespace lingtu::driver;
using namespace std::chrono_literals;

namespace {
void check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

class Robot final : public Body {
 public:
  Robot() { body.fresh = true; body.posture = Posture::Standing; }
  Result refresh() override {
    Result r;
    r.ok = r.accepted = r.transport_ok = r.state.ready = body.posture == Posture::Standing;
    return r;
  }
  Result move(const Velocity &) override { return refresh(); }
  Result stop() noexcept override {
    ++stops;
    Result r;
    r.ok = r.accepted = r.transport_ok = r.stop_confirmed = true;
    return r;
  }
  Result act(BodyAction action) override {
    if (action == BodyAction::Sit) ++sits;
    if (action == BodyAction::Stand) ++stands;
    body.posture = Posture::Transitioning;
    Result r;
    r.ok = r.accepted = r.transport_ok = true;
    return r;
  }
  Capabilities capabilities() const noexcept override { return {true, true, false, false}; }
  BodyState state() const override { return body; }
  HealthState health() const override { return {}; }
  AdapterDiagnostics diagnostics() const override { return {}; }
  ProfileState readProfile() override { ++reads; return observed; }
  Result switchProfile(const std::string &name) override {
    ++switches;
    requested = name;
    check(observed.grounded, "profile RPC must run only after actual Grounded feedback");
    Result r;
    r.ok = r.accepted = r.transport_ok = !reject_switch;
    if (reject_switch) r.error = "temporary_profile_failure";
    return r; // An ACK deliberately leaves actual profile unchanged.
  }
  void grounded() { observed.grounded = true; body.posture = Posture::Sitting; }
  void standing() { observed.grounded = false; body.posture = Posture::Standing; }
  BodyState body;
  ProfileState observed{true, true, false, true, "walk-custom", {"walk-custom", "stairs-v2"}, ""};
  std::string requested;
  int stops{0}, sits{0}, stands{0}, switches{0}, reads{0};
  bool reject_switch{false};
};

void handoffRestoreAndReadback() {
  Robot robot;
  ProfileSwitch transition("stairs-v2", "stairs-v2");
  auto now = Clock::now();
  auto tick = [&] (const char *requirement) {
    now += 200ms;
    transition.request({"task-a", requirement}, now);
    transition.step(robot, now, true);
  };
  tick("stairs_up");
  check(robot.stops == 1 && robot.sits == 1 && robot.switches == 0, "prepare the landing first");
  tick("stairs_up");
  check(robot.switches == 0, "an accepted Sit is not Grounded feedback");
  robot.grounded();
  tick("stairs_up");
  check(robot.switches == 1 && robot.stands == 0, "switch before a separate readback");
  tick("stairs_up");
  check(transition.status().phase == "waiting" && robot.stands == 0,
        "accepted switch with old profile must stay waiting");
  now += 1s;
  robot.observed.current = "stairs-v2";
  tick("stairs_up"); // Retry command, then read independently.
  tick("stairs_up");
  check(robot.stands == 1, "readback must permit return to standing");
  tick("stairs_up");
  check(transition.holdsMotion(), "stand ACK alone must not restore motion");
  robot.standing();
  tick("stairs_up");
  check(transition.status().phase == "ready" && !transition.holdsMotion(), "profile and readiness confirmed");
  const int before = robot.sits;
  tick("stairs_down");
  check(transition.status().phase == "ready" && robot.sits == before,
        "connected flight with the same profile must avoid redundant body transition");
  tick("restore");
  check(transition.status().requested_profile == "walk-custom", "restore the actual previous profile");
  robot.grounded();
  tick("restore");
  check(robot.requested == "walk-custom", "restore uses real captured profile, no default name");
  robot.observed.current = "walk-custom";
  tick("restore");
  robot.standing();
  tick("restore");
  check(transition.status().phase == "ready", "exit restoration waits for readback and standing");
}

void staleRequestDoesNotAdvance() {
  Robot robot;
  ProfileSwitch transition("stairs-v2", "stairs-v2");
  auto now = Clock::now();
  transition.request({"task", "stairs_up"}, now);
  transition.step(robot, now, true);
  robot.grounded();
  transition.step(robot, now + 2s, true);
  check(robot.switches == 0 && transition.holdsMotion(), "missing nav request must hold without switching");
  transition.request({"task", "stairs_up"}, now + 4s);
  transition.step(robot, now + 4s, true);
  check(robot.switches == 1, "fresh same-task request resumes the existing preparation");
}

void unavailableProfilesDoNotInventSupport() {
  Robot robot;
  auto now = Clock::now();
  ProfileSwitch no_prior("stairs-v2", "stairs-v2");
  no_prior.request({"task", "restore"}, now);
  no_prior.step(robot, now, true);
  check(no_prior.status().phase == "unsupported" &&
        no_prior.status().reason == "previous_profile_unknown",
        "a fresh driver must not claim restoration to a profile it never observed");
  ProfileSwitch absent("", "");
  absent.request({"task", "stairs_up"}, now);
  absent.step(robot, now, true);
  check(absent.status().phase == "unsupported" && robot.sits == 0,
        "unconfigured stairs do not change the robot body or ordinary nav");
  ProfileSwitch unsupported("stairs-v2", "stairs-v2");
  robot.observed = {};
  unsupported.request({"task", "stairs_up"}, now);
  unsupported.step(robot, now, true);
  check(unsupported.status().phase == "unsupported" && robot.switches == 0,
        "no gait number may replace unavailable model readback");
}

void commandAndReadbackFailuresWaitForRetry() {
  Robot robot;
  ProfileSwitch transition("stairs-v2", "stairs-v2");
  auto now = Clock::now();
  transition.request({"task", "stairs_up"}, now);
  transition.step(robot, now, false);
  check(robot.stops == 1 && robot.sits == 0, "nonzero nav input must stop before body preparation");
  now += 2s;
  transition.request({"task", "stairs_up"}, now);
  transition.step(robot, now, true);
  robot.grounded();
  robot.reject_switch = true;
  now += 200ms;
  transition.request({"task", "stairs_up"}, now);
  transition.step(robot, now, true);
  check(transition.status().phase == "waiting" && transition.holdsMotion(), "switch failure retains request");
  robot.reject_switch = false;
  now += 2s;
  transition.request({"task", "stairs_up"}, now);
  transition.step(robot, now, true);
  check(robot.switches == 2, "recoverable failure retries without exhausting nav task");
}

void holdSuspendsBodyActionsUntilExplicitResume() {
  Robot robot;
  ProfileSwitch transition("stairs-v2", "stairs-v2");
  auto now = Clock::now();
  transition.request({"task", "stairs_up"}, now);
  transition.step(robot, now, true);
  robot.grounded();
  transition.request({"task", "hold"}, now + 1ms);
  transition.step(robot, now + 200ms, true);
  transition.step(robot, now + 800ms, true);
  check(robot.switches == 0 && robot.stands == 0 && transition.holdsMotion(),
        "hold must suspend queued switch/stand immediately, before request TTL expires");
  check(transition.status().reason == "profile_paused", "hold keeps a visible waiting state");
  transition.request({"other-task", "hold"}, now + 900ms);
  check(transition.status().task_id == "task", "foreign hold cannot create or replace a handoff");
  transition.request({"task", "stairs_up"}, now + 1s);
  transition.step(robot, now + 1s, true);
  check(robot.switches == 1 && robot.requested == "stairs-v2",
        "resuming the same requirement continues retained handoff without restoring ordinary profile");
}
}

int main() {
  try {
    handoffRestoreAndReadback();
    staleRequestDoesNotAdvance();
    unavailableProfilesDoNotInventSupport();
    commandAndReadbackFailuresWaitForRetry();
    holdSuspendsBodyActionsUntilExplicitResume();
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_profile_switch: %s\n", error.what());
    return 1;
  }
}
