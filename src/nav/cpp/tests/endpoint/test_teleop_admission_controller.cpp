#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "control/admission.hpp"

namespace {
using lingtu::nav::endpoint::TeleopAdmissionActions;
using lingtu::nav::endpoint::TeleopAdmissionContext;
using lingtu::nav::endpoint::TeleopAdmissionController;
using lingtu::nav::endpoint::TeleopAdmissionRequest;

void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}

struct Fixture {
  std::vector<std::string> order;
  bool begin_ok{true};
  bool clear_ok{true};
  bool publish_zero_ok{true};
  bool accept_ok{true};
  TeleopAdmissionActions actions;

  Fixture() {
    actions.hold_operator_takeover = [&] { order.emplace_back("hold"); };
    actions.stop_control = [&] { order.emplace_back("stop"); };
    actions.begin_operator_takeover = [&](const nav_kernel::Twist &, double) {
      order.emplace_back("begin");
      return begin_ok;
    };
    actions.mark_operator_resume_required = [&] { order.emplace_back("mark_resume"); };
    actions.clear_motion = [&](const std::string &) {
      order.emplace_back("clear_motion");
      return clear_ok;
    };
    actions.pause_inspection = [&](const std::string &) { order.emplace_back("pause"); };
    actions.accept_teleop = [&](const nav_kernel::Twist &, double) {
      order.emplace_back("accept");
      return accept_ok;
    };
    actions.publish_zero = [&] {
      order.emplace_back("zero");
      return publish_zero_ok;
    };
  }
};

TeleopAdmissionContext context() {
  TeleopAdmissionContext out;
  out.receive_s = 100.0;
  out.motion_allowed = true;
  out.autonomy_mode = true;
  out.allow_takeover = true;
  out.max_age_s = 0.5;
  out.future_tolerance_s = 0.1;
  out.min_motion = 0.05;
  out.publish_cmd_vel = true;
  return out;
}

TeleopAdmissionRequest request(nav_kernel::Twist twist = {0.2, 0.0, 0.0}) {
  return {twist, 99.8, {}};
}

void orderIs(const Fixture &fixture, std::initializer_list<const char *> expected) {
  std::vector<std::string> wanted;
  for (const char *item : expected)
    wanted.emplace_back(item);
  require(fixture.order == wanted, "action order mismatch");
}

void testEstopBeforeDecode() {
  Fixture fixture;
  TeleopAdmissionController controller(fixture.actions);
  auto input = request();
  input.decode_error = "decode_error";
  auto ctx = context();
  ctx.motion_allowed = false;
  const auto result = controller.admit(input, ctx);
  require(!result.accepted && result.reason == "estop_latched", "estop ordering mismatch");
  require(result.delta.command_count == 1 && result.delta.frame_rejected == 1,
          "estop deltas mismatch");
  require(result.observation.reason == "estop_latched" && result.observation.limited == false,
          "estop observation mismatch");
  require(!result.update_receive_timestamp && fixture.order.empty(),
          "estop must not update or act");
}

void testDecodeOwnership() {
  for (int mode = 0; mode < 4; ++mode) {
    Fixture fixture;
    TeleopAdmissionController controller(fixture.actions);
    auto input = request();
    input.decode_error = "decode_error";
    auto ctx = context();
    ctx.autonomy_mode = mode != 0;
    ctx.operator_takeover_latched = mode == 2;
    ctx.has_active_teleop = mode == 3;
    const bool owns = !ctx.autonomy_mode || ctx.operator_takeover_latched || ctx.has_active_teleop;
    const auto result = controller.admit(input, ctx);
    require(!result.accepted && result.delta.frame_rejected == 1, "decode must reject");
    require(result.delta.stop_count == (owns ? 1u : 0u), "decode stop delta mismatch");
    require(result.delta.limited_count == (owns ? 1u : 0u), "decode limit delta mismatch");
    require(result.delta.output_count == (owns ? 1u : 0u), "decode output delta mismatch");
    if (ctx.operator_takeover_latched)
      orderIs(fixture, {"hold", "zero"});
    else if (owns)
      orderIs(fixture, {"stop", "zero"});
    else
      orderIs(fixture, {});
  }

  Fixture fixture;
  TeleopAdmissionController controller(fixture.actions);
  auto input = request();
  input.decode_error = "decode_error";
  auto ctx = context();
  ctx.operator_takeover_latched = true;
  ctx.publish_cmd_vel = false;
  const auto result = controller.admit(input, ctx);
  require(result.delta.output_count == 0, "disabled writer output delta mismatch");
  orderIs(fixture, {"hold"});
}

void testStampFailClosed() {
  for (const auto &sample :
       std::vector<std::pair<double, std::string>>{{0.0, "teleop_source_stamp_invalid"},
                                                   {100.2, "teleop_source_stamp_future"},
                                                   {99.0, "teleop_source_stamp_stale"}}) {
    Fixture fixture;
    TeleopAdmissionController controller(fixture.actions);
    auto input = request();
    input.source_stamp_s = sample.first;
    auto ctx = context();
    ctx.operator_takeover_latched = true;
    const auto result = controller.admit(input, ctx);
    require(!result.accepted && result.reason == sample.second, "stamp reason mismatch");
    require(result.observation.age_s.has_value() && result.observation.published == true,
            "stamp observation mismatch");
    require(result.delta.stop_count == 1 && result.delta.limited_count == 1 &&
                result.delta.output_count == 1,
            "stamp deltas mismatch");
    orderIs(fixture, {"hold", "zero"});
  }

  Fixture fixture;
  TeleopAdmissionController controller(fixture.actions);
  auto input = request();
  input.source_stamp_s = 99.0;
  auto ctx = context();
  ctx.operator_takeover_latched = true;
  ctx.publish_cmd_vel = false;
  const auto result = controller.admit(input, ctx);
  require(result.observation.published == false && result.delta.output_count == 0,
          "no-wire stamp observation mismatch");
  orderIs(fixture, {"hold", "zero"});

  Fixture publish_failure;
  publish_failure.publish_zero_ok = false;
  TeleopAdmissionController publish_failure_controller(publish_failure.actions);
  auto failed_input = request();
  failed_input.source_stamp_s = 99.0;
  auto failed_ctx = context();
  failed_ctx.operator_takeover_latched = true;
  const auto failed = publish_failure_controller.admit(failed_input, failed_ctx);
  require(!failed.accepted && failed.reason == "zero_publish_failed_manual_hold_latched" &&
              failed.observation.published == false && failed.delta.output_count == 0,
          "failed zero publication must not be reported as output");
  require(failed.delta.frame_error.has_value() &&
              *failed.delta.frame_error == "zero_publish_failed_manual_hold_latched",
          "failed zero publication must be observable");
}

void testResumeBoundaryRejectsQueuedPreResumeSamples() {
  Fixture queued;
  TeleopAdmissionController queued_controller(queued.actions);
  auto queued_ctx = context();
  queued_ctx.autonomy_mode = false;
  queued_ctx.request_not_before_s = 99.85;
  const auto queued_result = queued_controller.admit(request(), queued_ctx);
  require(!queued_result.accepted &&
              queued_result.reason == "teleop_source_predates_resume",
          "a queued pre-resume teleop sample must be rejected");
  orderIs(queued, {"stop", "zero"});

  Fixture fresh;
  TeleopAdmissionController fresh_controller(fresh.actions);
  auto fresh_ctx = context();
  fresh_ctx.autonomy_mode = false;
  fresh_ctx.request_not_before_s = 99.85;
  auto fresh_request = request();
  fresh_request.source_stamp_s = 99.9;
  const auto fresh_result = fresh_controller.admit(fresh_request, fresh_ctx);
  require(fresh_result.accepted && fresh_result.reason == "accepted_for_safety_arbitration",
          "a fresh post-resume teleop sample must be admitted");
  orderIs(fresh, {"accept"});
}

void testAutonomyGates() {
  Fixture disabled;
  TeleopAdmissionController disabled_controller(disabled.actions);
  auto disabled_ctx = context();
  disabled_ctx.allow_takeover = false;
  disabled_ctx.operator_takeover_latched = true;
  const auto rejected = disabled_controller.admit(request(), disabled_ctx);
  require(!rejected.accepted && rejected.reason == "teleop_takeover_disabled",
          "takeover disabled mismatch");
  require(rejected.delta.frame_rejected == 1 && rejected.delta.output_count == 0,
          "takeover disabled deltas mismatch");
  orderIs(disabled, {});

  Fixture zero;
  TeleopAdmissionController zero_controller(zero.actions);
  const auto accepted = zero_controller.admit(request({0.0, 0.0, 0.0}), context());
  require(accepted.accepted && accepted.reason == "zero_teleop_did_not_take_over",
          "zero input mismatch");
  require(!accepted.update_receive_timestamp && accepted.observation.request.has_value(),
          "zero input observation mismatch");
  orderIs(zero, {});
}

void testTakeoverOrderAndFailures() {
  Fixture fixture;
  TeleopAdmissionController controller(fixture.actions);
  const auto result = controller.admit(request(), context());
  require(result.accepted && result.reason == "takeover_active_for_safety_arbitration",
          "takeover result mismatch");
  require(result.update_receive_timestamp, "takeover must update receive timestamp");
  orderIs(fixture, {"begin", "mark_resume", "clear_motion", "pause"});

  Fixture begin_failure;
  begin_failure.begin_ok = false;
  TeleopAdmissionController begin_controller(begin_failure.actions);
  require(begin_controller.admit(request(), context()).reason == "estop_latched",
          "begin failure mismatch");
  orderIs(begin_failure, {"begin"});

  Fixture clear_failure;
  clear_failure.clear_ok = false;
  TeleopAdmissionController clear_controller(clear_failure.actions);
  const auto clear = clear_controller.admit(request(), context());
  require(!clear.accepted && clear.reason == "zero_publish_failed_manual_hold_latched",
          "clear failure mismatch");
  orderIs(clear_failure, {"begin", "mark_resume", "clear_motion", "hold"});
}

void testOwnedAndManualOnlyQueue() {
  Fixture owned;
  TeleopAdmissionController owned_controller(owned.actions);
  auto owned_ctx = context();
  owned_ctx.has_active_teleop = true;
  const auto result = owned_controller.admit(request(), owned_ctx);
  require(result.accepted && result.reason == "accepted_for_safety_arbitration" &&
              result.update_receive_timestamp,
          "owned admission mismatch");
  orderIs(owned, {"accept"});

  Fixture manual;
  TeleopAdmissionController manual_controller(manual.actions);
  auto manual_ctx = context();
  manual_ctx.autonomy_mode = false;
  manual_ctx.allow_takeover = false;
  require(manual_controller.admit(request(), manual_ctx).accepted, "manual admission mismatch");
  orderIs(manual, {"accept"});

  Fixture failed;
  failed.accept_ok = false;
  TeleopAdmissionController failed_controller(failed.actions);
  auto failed_ctx = context();
  failed_ctx.has_active_teleop = true;
  const auto rejected = failed_controller.admit(request(), failed_ctx);
  require(!rejected.accepted && rejected.reason == "estop_latched" &&
              !rejected.update_receive_timestamp,
          "accept failure mismatch");
  orderIs(failed, {"accept"});
}

}  // namespace

int main() {
  try {
    testEstopBeforeDecode();
    testDecodeOwnership();
    testStampFailClosed();
    testResumeBoundaryRejectsQueuedPreResumeSamples();
    testAutonomyGates();
    testTakeoverOrderAndFailures();
    testOwnedAndManualOnlyQueue();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_teleop_admission_controller: FAIL: %s\n", exc.what());
    return 1;
  }
  return 0;
}
