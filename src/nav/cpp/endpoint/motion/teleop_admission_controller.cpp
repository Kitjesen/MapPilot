#include "motion/teleop_admission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

bool ownsControl(const TeleopAdmissionContext &context) {
  return !context.autonomy_mode || context.operator_takeover_latched || context.has_active_teleop;
}

void setStoppedObservation(TeleopAdmissionResult &result, const std::string &reason, bool limited,
                           bool published) {
  result.observation.seen = true;
  result.observation.fresh = false;
  result.observation.published = published;
  result.observation.stopped = true;
  result.observation.limited = limited;
  result.observation.reason = reason;
  result.observation.output = nav_kernel::Twist{};
}

void setZeroPublishFailure(TeleopAdmissionResult &result) {
  result.reason = "zero_publish_failed_manual_hold_latched";
  result.delta.frame_error = result.reason;
  result.observation.published = false;
  result.observation.reason = result.reason;
}

std::string stampError(const TeleopAdmissionRequest &request, const TeleopAdmissionContext &context,
                       double source_age_s) {
  if (!std::isfinite(request.source_stamp_s) || request.source_stamp_s <= 0.0) {
    return "teleop_source_stamp_invalid";
  }
  if (source_age_s < -context.future_tolerance_s) {
    return "teleop_source_stamp_future";
  }
  if (context.max_age_s > 0.0 && source_age_s > context.max_age_s) {
    return "teleop_source_stamp_stale";
  }
  return {};
}

}  // namespace

TeleopAdmissionController::TeleopAdmissionController(TeleopAdmissionActions actions)
    : actions_(std::move(actions)) {}

TeleopAdmissionResult
TeleopAdmissionController::admit(const TeleopAdmissionRequest &request,
                                 const TeleopAdmissionContext &context) const {
  TeleopAdmissionResult result;

  if (!context.motion_allowed) {
    result.reason = "estop_latched";
    result.delta.frame_rejected = 1;
    result.delta.frame_error = result.reason;
    setStoppedObservation(result, result.reason, false, false);
    return result;
  }

  if (!request.decode_error.empty()) {
    result.reason = request.decode_error;
    result.delta.frame_rejected = 1;
    result.delta.frame_error = result.reason;
    const bool owns = ownsControl(context);
    if (owns) {
      if (context.operator_takeover_latched) {
        if (actions_.hold_operator_takeover) {
          actions_.hold_operator_takeover();
        }
      } else if (actions_.stop_control) {
        actions_.stop_control();
      }
    }
    setStoppedObservation(result, result.reason, true, false);
    if (owns && context.publish_cmd_vel) {
      const bool zero_published = actions_.publish_zero && actions_.publish_zero();
      result.observation.published = zero_published;
      if (zero_published) {
        result.delta.output_count = 1;
      } else {
        setZeroPublishFailure(result);
      }
    }
    if (owns) {
      result.delta.stop_count = 1;
      result.delta.limited_count = 1;
    }
    return result;
  }

  const double source_age_s = context.receive_s - request.source_stamp_s;
  result.observation.age_s = source_age_s;
  const std::string stamp_error = stampError(request, context, source_age_s);
  if (!stamp_error.empty()) {
    result.reason = stamp_error;
    result.delta.frame_rejected = 1;
    result.delta.frame_error = result.reason;
    const bool owns = ownsControl(context);
    if (owns) {
      if (context.operator_takeover_latched) {
        if (actions_.hold_operator_takeover) {
          actions_.hold_operator_takeover();
        }
      } else if (actions_.stop_control) {
        actions_.stop_control();
      }
    }
    setStoppedObservation(result, result.reason, true, false);
    if (owns) {
      const bool zero_published = actions_.publish_zero && actions_.publish_zero();
      result.delta.stop_count = 1;
      result.delta.limited_count = 1;
      if (context.publish_cmd_vel) {
        result.observation.published = zero_published;
        if (zero_published) {
          result.delta.output_count = 1;
        } else {
          setZeroPublishFailure(result);
        }
      }
    }
    return result;
  }

  bool takeover_started = false;
  if (context.autonomy_mode) {
    if (!context.allow_takeover) {
      result.reason = "teleop_takeover_disabled";
      result.delta.frame_rejected = 1;
      result.delta.frame_error = result.reason;
      setStoppedObservation(result, result.reason, false, false);
      return result;
    }

    const bool already_owned = context.operator_takeover_latched || context.has_active_teleop;
    const double takeover_threshold = std::max(1e-6, context.min_motion);
    const bool deliberate_input =
        std::hypot(request.command.vx, request.command.vy) >= takeover_threshold ||
        std::abs(request.command.wz) >= takeover_threshold;
    if (!already_owned && !deliberate_input) {
      result.accepted = true;
      result.reason = "zero_teleop_did_not_take_over";
      setStoppedObservation(result, result.reason, false, false);
      result.observation.request = request.command;
      return result;
    }

    if (!already_owned) {
      if (!actions_.begin_operator_takeover ||
          !actions_.begin_operator_takeover(request.command, request.source_stamp_s)) {
        result.reason = "estop_latched";
        return result;
      }
      if (actions_.mark_operator_resume_required) {
        actions_.mark_operator_resume_required();
      }
      if (!actions_.clear_motion || !actions_.clear_motion("operator_takeover")) {
        if (actions_.hold_operator_takeover) {
          actions_.hold_operator_takeover();
        }
        result.reason = "zero_publish_failed_manual_hold_latched";
        return result;
      }
      if (actions_.pause_inspection) {
        actions_.pause_inspection("operator_takeover");
      }
      takeover_started = true;
    }
  }

  if (!takeover_started && (!actions_.accept_teleop ||
                            !actions_.accept_teleop(request.command, request.source_stamp_s))) {
    result.reason = "estop_latched";
    return result;
  }

  result.accepted = true;
  result.reason = takeover_started ? "takeover_active_for_safety_arbitration"
                                   : "accepted_for_safety_arbitration";
  result.update_receive_timestamp = true;
  result.observation.seen = true;
  result.observation.request = request.command;
  return result;
}

}  // namespace lingtu::nav::endpoint
