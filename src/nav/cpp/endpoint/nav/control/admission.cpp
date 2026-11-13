#include "control/admission.hpp"

namespace lingtu::nav::endpoint {

TeleopAdmissionResult admitTeleop(const TeleopAdmissionRequest &request) {
  TeleopAdmissionResult result;
  if (!request.decode_error.empty()) {
    result.reason = request.decode_error;
    result.delta.frame_rejected = 1;
    result.delta.frame_error = result.reason;
    result.observation.seen = true;
    result.observation.fresh = false;
    result.observation.stopped = true;
    result.observation.reason = result.reason;
    return result;
  }
  result.accepted = true;
  result.reason = "accepted";
  result.update_receive_timestamp = true;
  result.observation.seen = true;
  result.observation.fresh = true;
  result.observation.request = request.command;
  return result;
}

}  // namespace lingtu::nav::endpoint
