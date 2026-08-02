#include "cmd_vel_writer_gate.hpp"

namespace lingtu::driver {

const char* cmdVelWriterReason(std::uint32_t matched_writers) noexcept {
  if (matched_writers == 1) {
    return "single_cmd_vel_writer";
  }
  if (matched_writers == 0) {
    return "missing_cmd_vel_writer";
  }
  return "ambiguous_cmd_vel_writers";
}

CmdVelWriterDecision CmdVelWriterGate::update(
    std::uint32_t matched_writers) noexcept {
  CmdVelWriterDecision decision;
  decision.matched_writers = matched_writers;
  decision.reason = cmdVelWriterReason(matched_writers);
  decision.ready = matched_writers == 1;
  decision.requires_stop = ready_ && !decision.ready;
  ready_ = decision.ready;
  return decision;
}

void CmdVelWriterGate::reset() noexcept {
  ready_ = false;
}

}  // namespace lingtu::driver
