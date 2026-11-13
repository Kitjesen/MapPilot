#include <cstdio>
#include <stdexcept>
#include "control/admission.hpp"
using namespace lingtu::nav::endpoint;
void check(bool ok, const char *msg) { if (!ok) throw std::runtime_error(msg); }
int main() {
  try {
    TeleopAdmissionRequest good; good.command = {0.2, 0.0, 0.0};
    auto accepted = admitTeleop(good);
    check(accepted.accepted && accepted.update_receive_timestamp, "valid teleop rejected");
    check(accepted.observation.seen && accepted.observation.fresh, "valid observation missing");
    TeleopAdmissionRequest bad; bad.decode_error = "decode_error";
    auto rejected = admitTeleop(bad);
    check(!rejected.accepted && rejected.delta.frame_rejected == 1, "invalid teleop accepted");
    check(rejected.observation.fresh.has_value() && !*rejected.observation.fresh && rejected.observation.stopped.value_or(false), "invalid observation mismatch");
    std::puts("test_teleop_admission_controller: PASS");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_teleop_admission_controller: %s\n", error.what());
    return 1;
  }
}
