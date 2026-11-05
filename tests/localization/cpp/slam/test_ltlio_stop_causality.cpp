#include "ltlio/frontend.hpp"
#include "ltlio/detail/ElevatorSelfExit.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
namespace lio = lingtu::ltlio;
namespace detail = lingtu::ltlio::detail;
constexpr double kGravity = 9.80665;
constexpr double kImuDt = 0.005;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

lio::Config config() {
  lio::Config settings;
  settings.imu_init_samples = 20;
  settings.point_filter_num = 1;
  settings.blind_m = 0.1;
  settings.scan_resolution = 0.1;
  settings.elevator_enabled = true;
  settings.automatic_door = false;
  return settings;
}

lio::TimedCloud cabin(double end) {
  lio::TimedCloud cloud;
  cloud.stamp_s = end - 0.02;
  cloud.end_stamp_s = end;
  for (int plane = 0; plane < 3; ++plane) {
    for (int a = -6; a <= 6; ++a) {
      for (int b = -6; b <= 6; ++b) {
        lio::TimedPoint point;
        if (plane == 0) {
          point.x = 1.1F; point.y = a * 0.1F; point.z = b * 0.1F;
        } else if (plane == 1) {
          point.x = a * 0.1F; point.y = 1.4F; point.z = b * 0.1F;
        } else {
          point.x = a * 0.1F; point.y = b * 0.1F; point.z = -0.8F;
        }
        cloud.points.push_back(point);
      }
    }
  }
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
    cloud.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return cloud;
}

double acceleration(double time) {
  const double elapsed = time - 1.1;
  if (elapsed < 0.0) return 0.0;
  if (elapsed < 1.5) return 0.7;
  if (elapsed < 4.5) return 0.0;
  if (elapsed < 6.0) return -0.7;
  return 0.0;
}

struct Stream {
  int imu_index = 0;
  int frame_index = 0;

  static int endIndex(int frame) { return 44 + frame * 10; }

  void feedThrough(lio::Frontend& frontend, int last_imu_index, double extra_acceleration = 0.0) {
    while (imu_index <= last_imu_index) {
      lio::ImuSample sample;
      sample.stamp_s = imu_index++ * kImuDt;
      sample.acceleration = {0.0, 0.0, kGravity + acceleration(sample.stamp_s) + extra_acceleration};
      frontend.feedImu(sample);
    }
  }

  lio::Result observe(lio::Frontend& frontend, double end) {
    frontend.feedCloud(cabin(end));
    auto result = frontend.processNext();
    require(result.has_value(), "covered stop-causality scan did not return a result");
    if (frame_index > 5)
      require(result->accepted, "stop-causality cabin failed actual matching: " + result->rejection_reason);
    return *result;
  }

  lio::Result step(lio::Frontend& frontend) {
    const int end_index = endIndex(frame_index++);
    feedThrough(frontend, end_index);
    return observe(frontend, end_index * kImuDt);
  }
};

void initialize(lio::Frontend& frontend, Stream& stream) {
  for (int i = 0; i < 8; ++i) stream.step(frontend);
  frontend.requestElevator(true);
}

int firstStopFrame() {
  int first_stop_frame = -1;
  {
    lio::Frontend baseline(config());
    Stream stream;
    initialize(baseline, stream);
    while (stream.frame_index < 260) {
      const auto result = stream.step(baseline);
      if (result.elevator.stop_updates > 0) {
        require(result.state.in_elevator && result.elevator.stop_confirmed,
                "baseline correction was not a confirmed transport stop");
        first_stop_frame = stream.frame_index - 1;
        break;
      }
    }
  }
  require(first_stop_frame >= 9, "physical ride did not produce a confirmed stop");
  return first_stop_frame;
}

void checkStopDoesNotUseLaterImu() {
  const int first_stop_frame = firstStopFrame();

  lio::Frontend delayed(config());
  Stream stream;
  initialize(delayed, stream);
  lio::Result before;
  while (stream.frame_index < first_stop_frame) before = stream.step(delayed);
  require(before.state.in_elevator && !before.elevator.stop_confirmed &&
              before.elevator.stop_updates == 0,
          "stop-causality prefix already consumed the stop confirmation");

  // The next scan ends before the first new IMU. Its delayed arrival still
  // falls inside retained IMU history, which is a supported frontend input.
  // Later IMUs can cover interpolation, but their stop evidence cannot be
  // applied to this earlier scan's posterior.
  stream.feedThrough(delayed, Stream::endIndex(first_stop_frame));
  const double early_end = before.scan_end_s + 0.002;
  const auto early = stream.observe(delayed, early_end);
  require(early.accepted && early.state.in_elevator,
          "delayed covered scan was rejected instead of preserving its estimate");
  require(early.elevator.stop_updates == 0,
          "future IMU stop confirmation corrected an earlier scan");
  require((early.world_from_odom.matrix() - before.world_from_odom.matrix()).norm() < 1e-11,
          "future stop evidence changed the earlier scan correction frame");

  ++stream.frame_index;
  const auto after = stream.step(delayed);
  require(after.accepted && after.elevator.stop_updates == 1 &&
              after.elevator.stop_confirmed && after.state.in_elevator,
          "causal scan did not consume the deferred stop update exactly once");
  std::cout << "Delayed scan preserved its posterior; stop update consumed after confirmation\n";
}

void checkLaterMotionDoesNotCancelEarlierStop() {
  const int first_stop_frame = firstStopFrame();
  lio::Frontend delayed(config());
  Stream stream;
  initialize(delayed, stream);
  lio::Result before;
  while (stream.frame_index <= first_stop_frame) before = stream.step(delayed);
  require(before.state.in_elevator && before.elevator.stop_confirmed &&
              before.elevator.stop_updates == 1,
          "cancellation prefix did not commit exactly one confirmed stop update");

  // A later acceleration may revoke the stop for later scans, but must not
  // rewrite a delayed observation that still precedes that physical motion.
  stream.feedThrough(delayed, Stream::endIndex(first_stop_frame) + 100, 2.0);
  const auto early = stream.observe(delayed, before.scan_end_s + 0.002);
  require(early.accepted && early.state.in_elevator,
          "covered scan before later acceleration did not preserve its estimate");
  require(early.elevator.stop_confirmed && early.elevator.stop_updates == 2,
          "future IMU motion cancelled an earlier scan's confirmed stop");
  const auto later = stream.observe(delayed,
      (Stream::endIndex(first_stop_frame) + 100) * kImuDt);
  require(later.accepted && later.state.in_elevator && !later.elevator.stop_confirmed &&
              later.elevator.stop_updates == 0,
          "later acceleration was ignored instead of revoking the later scan's stop");
  std::cout << "Later acceleration did not revoke stop evidence for an earlier scan\n";
}

void checkLostDetectorHistoryDoesNotPreserveStop() {
  const int first_stop_frame = firstStopFrame();
  lio::Frontend delayed(config());
  Stream stream;
  initialize(delayed, stream);
  lio::Result before;
  while (stream.frame_index <= first_stop_frame) before = stream.step(delayed);
  require(before.state.in_elevator && before.elevator.stop_confirmed &&
              before.elevator.stop_updates == 1,
          "history-gap prefix did not establish a confirmed stop");

  // The retained IMU history is 1000 samples. With no LiDAR observations,
  // 1203 new samples necessarily discard detector inputs that have not yet
  // been consumed, unlike normal eviction of already processed samples.
  const int end_index = Stream::endIndex(first_stop_frame) + 1203;
  stream.feedThrough(delayed, end_index);
  const auto current = stream.observe(delayed, end_index * kImuDt);
  require(current.accepted && current.state.in_elevator &&
              current.state.covariance.allFinite(),
          "current covered scan did not recover after delayed LiDAR input");
  require(current.elevator.phase == 4 && current.elevator.stop_confirmed &&
              current.elevator.stop_updates == 1 && current.elevator.exits == 0,
          "retained post-gap IMUs did not rebuild fresh stop evidence without exiting");

  // More than 3 seconds of retained, contiguous post-gap IMUs are available,
  // so the detector may confirm again. The gap must still clear the previous
  // LiDAR ZUPT count: this first recovering scan cannot finish the trip.
  lio::Result recovered = current;
  for (int i = 0; i < 5 && recovered.state.in_elevator; ++i) {
    stream.feedThrough(delayed, end_index + (i + 1) * 10);
    recovered = stream.observe(delayed, (end_index + (i + 1) * 10) * kImuDt);
  }
  require(!recovered.state.in_elevator && recovered.elevator.exits == 1,
          "history-gap recovery did not finish transport after fresh stop evidence");
  std::cout << "Discarded detector history rebuilt stop evidence before exiting\n";
}

void checkEntryBetweenScanAndCoveringImu(double offset) {
  lio::Frontend frontend(config());
  Stream stream;
  lio::Result before;
  for (int i = 0; i < 8; ++i) before = stream.step(frontend);
  require(before.accepted && !before.state.in_elevator &&
              before.elevator.entries == 0 && before.elevator.exits == 0,
          "entry-boundary prefix did not establish ordinary matched odometry");

  // The next scan has already begun when the live operator requests entry.
  // It ends 2 or 4 ms after the last IMU, while its first covering IMU arrives
  // 5 ms after it. This is the normal nonaligned scan/IMU intake contract.
  const double early_end = before.scan_end_s + offset;
  frontend.requestElevator(true);
  frontend.feedCloud(cabin(early_end));
  stream.feedThrough(frontend, stream.imu_index);
  const auto early = frontend.processNext();
  require(early && early->accepted,
          "covered entry-boundary scan did not complete real cabin matching");
  require(early->elevator.exits == 0,
          "entry request fabricated an exit while committing the earlier scan: entries=" +
              std::to_string(early->elevator.entries) + " exits=" +
              std::to_string(early->elevator.exits));
  require(!early->state.in_elevator && early->elevator.entries == 0,
          "an earlier scan borrowed the covering IMU's future transport entry");

  // No new IMU is supplied here: the already observed entry at the covering
  // sample must survive correction/reintegration of the earlier scan.
  const double entry_time = (stream.imu_index - 1) * kImuDt;
  const auto at_entry = stream.observe(frontend, entry_time);
  require(at_entry.accepted && at_entry.state.in_elevator &&
              at_entry.elevator.entries == 1 && at_entry.elevator.exits == 0,
          "earlier posterior commit erased or duplicated the existing future entry");

  // A subsequent complete scan is strictly after the requested entry and
  // must retain exactly one trip, regardless of the earlier interpolation.
  const double next_end = before.scan_end_s + 0.052;
  frontend.feedCloud(cabin(next_end));
  stream.feedThrough(frontend, static_cast<int>(std::ceil(next_end / kImuDt)));
  const auto active = frontend.processNext();
  require(active && active->accepted && active->state.in_elevator &&
              active->elevator.entries == 1 && active->elevator.exits == 0,
          "posterior reintegration erased or duplicated the requested transport entry");
  std::cout << "Entry across an IMU bracket preserved one trip without a fabricated exit\n";
}

void checkRejectedEntryAndExitTail() {
  lio::Frontend frontend(config());
  Stream stream;
  lio::Result before;
  for (int i = 0; i < 8; ++i) before = stream.step(frontend);
  frontend.requestElevator(true);
  const double rejected_end = before.scan_end_s + 0.05;
  auto sparse = cabin(rejected_end);
  sparse.points.resize(2);
  frontend.feedCloud(sparse);
  stream.feedThrough(frontend, static_cast<int>(std::ceil(rejected_end / kImuDt)));
  const auto rejected = frontend.processNext();
  require(rejected && rejected->attempted && !rejected->accepted && rejected->state.in_elevator,
          "rejected-entry fixture did not reject actual matching during transport");
  require(rejected->elevator.entries == 1 && rejected->elevator.exits == 0,
          "rejecting the only transport scan hid the observed trip entry");

  // The only accepted scan of this trip is its exit. The covering IMU still
  // contains the old active tail, which must not become a new entry edge.
  frontend.requestElevator(false);
  const double exit_end = (stream.imu_index - 1) * kImuDt + 0.002;
  frontend.feedCloud(cabin(exit_end));
  stream.feedThrough(frontend, static_cast<int>(std::ceil(exit_end / kImuDt)));
  const auto exited = frontend.processNext();
  require(exited && exited->accepted && !exited->state.in_elevator &&
              exited->elevator.entries == 1 && exited->elevator.exits == 1,
          "accepted exit lost the rejected-scan trip or invented another entry");
  const double tail_time = (stream.imu_index - 1) * kImuDt;
  const auto tail = stream.observe(frontend, tail_time);
  require(tail.accepted && !tail.state.in_elevator &&
              tail.elevator.entries == 1 && tail.elevator.exits == 1,
          "exit posterior replayed an old active IMU tail as a new entry");
  std::cout << "Rejected scan retained its trip; explicit exit retired the existing active tail\n";
}

void checkQueuedImuGapBreaksStopEvidenceCausally() {
  const int first_stop_frame = firstStopFrame();
  auto settings = config();
  settings.max_imu_buffer = 350;
  lio::Frontend delayed(settings);
  Stream stream;
  initialize(delayed, stream);
  lio::Result before;
  while (stream.frame_index <= first_stop_frame) before = stream.step(delayed);
  require(before.state.in_elevator && before.elevator.stop_confirmed &&
              before.elevator.stop_updates == 1,
          "queued-gap prefix did not establish exactly one confirmed stop update");

  // A queued scan stops integration at its covering IMU. Further incoming
  // samples fill the bounded intake queue and discard its oldest entries.
  // The loss is in the future of this pending scan, not in its stop evidence.
  delayed.feedCloud(cabin(before.scan_end_s + 0.002));
  const int base_index = Stream::endIndex(first_stop_frame);
  stream.feedThrough(delayed, base_index + 400);
  require(delayed.droppedImus() > 0, "queued-gap fixture did not overflow the real IMU intake");
  const auto early = delayed.processNext();
  require(early && early->accepted && early->state.in_elevator &&
              early->elevator.stop_confirmed && early->elevator.stop_updates == 2,
          "future queued-IMU loss revoked the earlier scan's confirmed stop");

  // This scan ends inside the missing interval, before the first surviving
  // IMU that covers it. Loss already breaks its evidence even though that
  // covering sample must remain in the detector's future.
  const auto middle = stream.observe(delayed, before.scan_end_s + 0.100);
  require(middle.accepted && middle.state.in_elevator &&
              !middle.elevator.stop_confirmed &&
              middle.elevator.stop_updates == 0,
          "scan inside the dropped IMU interval retained a continuous stop confirmation");

  // The 350 retained samples fill the 300-sample variance window, but provide
  // only 250 ms of the new 1.5 second hold. The missing interval and the
  // pre-gap confirmation must not count toward that hold.
  const auto after_gap = stream.observe(delayed, (base_index + 400) * kImuDt);
  require(after_gap.accepted && after_gap.state.in_elevator &&
              !after_gap.elevator.stop_confirmed &&
              after_gap.elevator.stop_updates == 0,
          "queued IMU loss preserved a stop confirmation across missing observations");

  stream.feedThrough(delayed, base_index + 651);
  const auto reconfirmed = stream.observe(delayed, (base_index + 651) * kImuDt);
  require(reconfirmed.accepted && reconfirmed.state.in_elevator &&
              reconfirmed.elevator.stop_confirmed &&
              reconfirmed.elevator.stop_updates == 1 && reconfirmed.elevator.exits == 0,
          "queued-gap detector did not rebuild a full window and fresh stop hold");

  lio::Result recovered = reconfirmed;
  for (int i = 0; i < 5 && recovered.state.in_elevator; ++i) {
    stream.feedThrough(delayed, base_index + 661 + i * 10);
    recovered = stream.observe(delayed, (base_index + 661 + i * 10) * kImuDt);
  }
  require(!recovered.state.in_elevator && recovered.elevator.exits == 1,
          "queued-gap recovery did not finish transport after fresh stop evidence");
  std::cout << "Queued IMU loss preserved phase and rebuilt stop evidence before exiting\n";
}

void checkIdleQueuedImuGapDoesNotInventExit() {
  auto settings = config();
  settings.max_imu_buffer = 50;
  lio::Frontend delayed(settings);
  Stream stream;
  initialize(delayed, stream);
  const auto active = stream.step(delayed);
  require(active.accepted && active.state.in_elevator &&
              active.elevator.phase == 0 && active.elevator.exits == 0,
          "idle-gap prefix did not establish an active, not-yet-moving trip");

  // Keep the whole loss before the ride's acceleration begins. Invalidating
  // continuity in IDLE must clear detector evidence without manufacturing a
  // completed ride or an exit edge.
  delayed.feedCloud(cabin(active.scan_end_s + 0.002));
  const int base_index = Stream::endIndex(stream.frame_index - 1);
  stream.feedThrough(delayed, base_index + 80);
  require(delayed.droppedImus() > 0, "idle-gap fixture did not overflow the IMU intake");
  const auto early = delayed.processNext();
  require(early && early->accepted && early->state.in_elevator &&
              early->elevator.phase == 0 && !early->elevator.stop_confirmed &&
              early->elevator.stop_updates == 0 && early->elevator.exits == 0,
          "future IDLE loss changed the earlier scan's active trip");

  const auto middle = stream.observe(delayed, active.scan_end_s + 0.100);
  require(middle.accepted && middle.state.in_elevator &&
              middle.elevator.phase == 0 && !middle.elevator.stop_confirmed &&
              middle.elevator.stop_updates == 0 && middle.elevator.exits == 0,
          "IDLE continuity loss fabricated stop evidence or an exit");

  const auto after_gap = stream.observe(delayed, (base_index + 80) * kImuDt);
  require(after_gap.accepted && after_gap.state.in_elevator &&
              after_gap.elevator.phase == 0 && !after_gap.elevator.stop_confirmed &&
              after_gap.elevator.stop_updates == 0 && after_gap.elevator.exits == 0,
          "IDLE detector did not remain active and unconfirmed after recovery");
  std::cout << "Queued IMU loss in IDLE did not invent a stop or exit\n";
}

detail::ElevatorSelfExit::ProcessResult detectorSample(
    detail::ElevatorSelfExit& detector, detail::State& state,
    double& timestamp, double velocity, double acceleration = 0.0) {
  timestamp += kImuDt;
  state.az = acceleration;
  detail::ElevatorSelfExit::ImuFrame frame;
  frame.timestamp = timestamp;
  frame.acc_z = acceleration;
  frame.vel_z = velocity;
  frame.in_elevator = true;
  return detector.process(frame, state);
}

void driveDetectorToPhase(detail::ElevatorSelfExit& detector,
                          detail::State& state, double& timestamp,
                          int target_phase) {
  for (int i = 0; i < 300; ++i)
    detectorSample(detector, state, timestamp, i % 2 == 0 ? 0.0 : 1.0);
  require(detector.eleState() == 1,
          "detector fixture did not reach ACCELERATING naturally");
  if (target_phase == 1) return;
  for (int i = 0; i < 300 && detector.eleState() != 2; ++i)
    detectorSample(detector, state, timestamp, 1.0);
  require(detector.eleState() == 2,
          "detector fixture did not reach CONSTANT_SPEED naturally");
}

void checkPhaseGapCanRecoverAfterHiddenDeceleration() {
  for (const int starting_phase : {1, 2}) {
    auto context = std::make_unique<detail::Context>(config());
    detail::ElevatorSelfExit detector(*context);
    detail::State state;
    state.in_elevator = true;
    double timestamp = 0.0;
    driveDetectorToPhase(detector, state, timestamp, starting_phase);

    detector.invalidateImuContinuity();
    detail::ElevatorSelfExit::ProcessResult result;
    for (int i = 0; i < 300; ++i)
      result = detectorSample(detector, state, timestamp, 0.0);
    require(detector.eleState() == 4 && !result.stop_confirmed &&
                !detector.stopCheckConfirmed(),
            "first rebuilt stationary window did not start a fresh stop check");

    bool confirmed = false;
    for (int i = 0; i < 299; ++i) {
      result = detectorSample(detector, state, timestamp, 0.0);
      require(!result.stop_confirmed && !detector.stopCheckConfirmed(),
              "phase-gap recovery counted the gap or refill window toward the stop hold");
    }
    for (int i = 0; i < 2; ++i)
      confirmed = detectorSample(detector, state, timestamp, 0.0).stop_confirmed || confirmed;
    require(confirmed && detector.eleState() == 4 && detector.stopCheckConfirmed(),
            "phase-gap recovery did not confirm after a fresh 1.5 second hold");
  }
  std::cout << "Active phases recovered when an IMU gap hid the complete deceleration\n";
}

void checkPhaseGapDoesNotTreatMotionAsStopped() {
  const struct {
    int starting_phase;
    double recovery_velocity;
  } cases[] = {{1, 0.3}, {2, 1.0}};

  for (const auto& test_case : cases) {
    auto context = std::make_unique<detail::Context>(config());
    detail::ElevatorSelfExit detector(*context);
    detail::State state;
    state.in_elevator = true;
    double timestamp = 0.0;
    driveDetectorToPhase(detector, state, timestamp, test_case.starting_phase);

    detector.invalidateImuContinuity();
    bool confirmed = false;
    for (int i = 0; i < 601; ++i) {
      confirmed = detectorSample(
          detector, state, timestamp, test_case.recovery_velocity).stop_confirmed || confirmed;
    }
    require(detector.eleState() != 4 && !confirmed && !detector.stopCheckConfirmed(),
            "phase-gap recovery treated continuing constant motion as a stop");
  }
  std::cout << "Rebuilt moving windows did not fabricate a stop after an IMU gap\n";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string selected = argc > 1 ? argv[1] : "";
    if (selected.empty() || selected == "--future-confirmation") checkStopDoesNotUseLaterImu();
    if (selected.empty() || selected == "--future-cancellation") checkLaterMotionDoesNotCancelEarlierStop();
    if (selected.empty() || selected == "--history-gap") checkLostDetectorHistoryDoesNotPreserveStop();
    if (selected.empty() || selected == "--entry-boundary" || selected == "--entry-boundary-left")
      checkEntryBetweenScanAndCoveringImu(0.002);
    if (selected.empty() || selected == "--entry-boundary" || selected == "--entry-boundary-right")
      checkEntryBetweenScanAndCoveringImu(0.004);
    if (selected.empty() || selected == "--rejected-entry-exit") checkRejectedEntryAndExitTail();
    if (selected.empty() || selected == "--queued-gap") checkQueuedImuGapBreaksStopEvidenceCausally();
    if (selected.empty() || selected == "--idle-gap") checkIdleQueuedImuGapDoesNotInventExit();
    if (selected.empty() || selected == "--phase-gap-stop")
      checkPhaseGapCanRecoverAfterHiddenDeceleration();
    if (selected.empty() || selected == "--phase-gap-moving")
      checkPhaseGapDoesNotTreatMotionAsStopped();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
