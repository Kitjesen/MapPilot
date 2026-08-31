#pragma once

#include <cstdint>
#include <string>

namespace lingtu::driver {

struct CmdVelWriterDecision {
  bool ready{false};
  bool requires_stop{false};
  std::uint32_t matched_writers{0};
  std::string reason{"missing_cmd_vel_writer"};
};

class CmdVelWriterGate {
 public:
  CmdVelWriterDecision update(std::uint32_t matched_writers) noexcept;
  void reset() noexcept;

  bool ready() const noexcept {
    return ready_;
  }

 private:
  bool ready_{false};
};

const char* cmdVelWriterReason(std::uint32_t matched_writers) noexcept;

}  // namespace lingtu::driver
