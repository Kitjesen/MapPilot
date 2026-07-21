#pragma once

namespace lingtu::nav::commands {

// ACK time is sampled before the client receives it, so this offset is a
// conservative lower bound on endpoint time.  Re-evaluating it against the
// local wall clock keeps source timestamps aligned when CLOCK_REALTIME slews.
inline double endpointClockOffset(
    double endpoint_ack_stamp_s,
    double local_receive_wall_s) noexcept {
  return endpoint_ack_stamp_s - local_receive_wall_s;
}

inline double endpointSourceTime(
    double local_wall_s,
    double endpoint_clock_offset_s) noexcept {
  return local_wall_s + endpoint_clock_offset_s;
}

}  // namespace lingtu::nav::commands
