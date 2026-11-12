#include <chrono>
#include <cstdio>
#include <stdexcept>
#include "safety/stop.hpp"
using namespace lingtu::nav::endpoint;
using namespace std::chrono_literals;
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
int main() {
  try {
    const auto start = StopConfirmation::Clock::time_point{10s};
    StopConfirmationConfig config{1000ms, 0.03, 0.08, 3};
    StopConfirmation stop(100000000000ULL, start, config);
    stop.observeDriverState(true, 99000000000ULL);
    stop.observeQuietOdometry(1.0, 0, 0);
    require(stop.state(start) == StopConfirmationState::Pending, "old zero must not confirm");
    stop.observeDriverState(false, 101000000000ULL);
    require(stop.state(start) == StopConfirmationState::Pending, "wait for zero output");
    stop.observeDriverState(true, 102000000000ULL);
    stop.observeQuietOdometry(2.0, 0, 0);
    stop.observeQuietOdometry(2.0, 0, 0);
    require(stop.diagnostics().quiet_odometry_samples == 1, "duplicates must not count twice");
    stop.observeDriverState(true, 103000000000ULL);
    stop.observeQuietOdometry(2.1, 0, 0);
    stop.observeQuietOdometry(2.2, 0.2, 0);
    require(stop.diagnostics().quiet_odometry_samples == 0, "motion resets settling");
    stop.observeQuietOdometry(2.3, 0, 0);
    stop.observeDriverState(false, 104000000000ULL);
    stop.observeQuietOdometry(2.4, 0, 0);
    require(stop.diagnostics().quiet_odometry_samples == 0, "loss of zero resets settling");
    stop.observeDriverState(true, 105000000000ULL);
    stop.observeDriverState(false, 104000000000ULL);
    for (double stamp : {2.5, 2.6, 2.7}) stop.observeQuietOdometry(stamp, 0, 0);
    require(stop.state(start + 500ms) == StopConfirmationState::Confirmed,
            "fresh zero plus quiet odometry confirms stop");
    config.evidence_policy = StopConfirmationEvidencePolicy::ZeroOutputOnly;
    StopConfirmation output_only(100000000000ULL, start, config);
    output_only.observeDriverState(true, 101000000000ULL);
    require(output_only.state(start) == StopConfirmationState::Confirmed &&
                !output_only.diagnostics().quiet_odometry_required,
            "output-only policy must not claim physical settling");
    StopConfirmation unavailable(100000000000ULL, start, config);
    unavailable.observeDriverState(false, 101000000000ULL);
    require(unavailable.state(start + 1s) == StopConfirmationState::TimedOut,
            "missing zero output times out");
    std::puts("test_stop_confirmation: PASS");
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
}
