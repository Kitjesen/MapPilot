#include "packet_timestamp_clock.hpp"

#include <cassert>
#include <cstdint>
#include <limits>

namespace {

using lingtu::drivers::lidar::PacketTimestampAction;
using lingtu::drivers::lidar::PacketTimestampClock;
using lingtu::drivers::lidar::PacketTimestampSource;
using lingtu::drivers::lidar::PacketTimestampStream;

constexpr std::uint64_t kMs = 1000000ULL;
constexpr std::uint64_t kSec = 1000000000ULL;
constexpr std::uint64_t kFallbackStart = 1720000000ULL * kSec;
constexpr std::uint64_t kObservedSourceJump = 8864ULL * kSec + 975ULL * kMs;

void expect_publish(
    const lingtu::drivers::lidar::PacketTimestampResult& result,
    std::uint64_t expected) {
  assert(result.action == PacketTimestampAction::Publish);
  assert(result.stamp_ns == expected);
}

void expect_drop(
    const lingtu::drivers::lidar::PacketTimestampResult& result) {
  assert(result.action == PacketTimestampAction::DropStale);
}

void expect_fatal(
    const lingtu::drivers::lidar::PacketTimestampResult& result) {
  assert(result.action == PacketTimestampAction::Fatal);
}

}  // namespace

int main() {
  const auto lidar = PacketTimestampStream::Lidar;
  const auto imu = PacketTimestampStream::Imu;
  const auto fallback = PacketTimestampSource::Fallback;
  const auto ptp = PacketTimestampSource::Ptp;

  // Field failure: fallback -> PTP differed by +8864.975 seconds. The output
  // must remain on the steady fallback timeline.
  PacketTimestampClock field_clock;
  expect_publish(field_clock.map(lidar, fallback, 0U, kFallbackStart), kFallbackStart);
  const std::uint64_t first_ptp_raw = kFallbackStart + kObservedSourceJump;
  expect_publish(
      field_clock.map(lidar, ptp, first_ptp_raw, kFallbackStart + 5ULL * kMs),
      kFallbackStart + 5ULL * kMs);
  expect_publish(
      field_clock.map(imu, ptp, first_ptp_raw + 1580ULL * 1000ULL,
                      kFallbackStart + 7ULL * kMs),
      kFallbackStart + 5ULL * kMs + 1580ULL * 1000ULL);
  expect_publish(
      field_clock.map(lidar, ptp, first_ptp_raw + 100ULL * kMs,
                      kFallbackStart + 105ULL * kMs),
      kFallbackStart + 105ULL * kMs);
  // Cross-stream callback order may differ; IMU keeps its own monotonic state.
  expect_publish(
      field_clock.map(imu, ptp, first_ptp_raw + 90ULL * kMs,
                      kFallbackStart + 106ULL * kMs),
      kFallbackStart + 95ULL * kMs);
  expect_publish(
      field_clock.map(lidar, fallback, 0U, kFallbackStart + 107ULL * kMs),
      kFallbackStart + 107ULL * kMs);

  // PTP-first -> fallback -> PTP is continuous even with different raw epochs.
  PacketTimestampClock ptp_first;
  expect_publish(ptp_first.map(lidar, ptp, first_ptp_raw, kFallbackStart),
                 kFallbackStart);
  expect_publish(ptp_first.map(lidar, fallback, 0U, kFallbackStart + 10ULL * kMs),
                 kFallbackStart + 10ULL * kMs);
  expect_publish(
      ptp_first.map(lidar, ptp, first_ptp_raw + 20ULL * kMs,
                    kFallbackStart + 20ULL * kMs),
      kFallbackStart + 20ULL * kMs);

  // One future outlier is dropped and does not poison the next good sample.
  PacketTimestampClock isolated_outlier;
  expect_publish(isolated_outlier.map(imu, ptp, first_ptp_raw, kFallbackStart),
                 kFallbackStart);
  expect_drop(isolated_outlier.map(
      imu, ptp, first_ptp_raw + 10ULL * kSec, kFallbackStart + 5ULL * kMs));
  expect_publish(isolated_outlier.map(
      imu, ptp, first_ptp_raw + 10ULL * kMs, kFallbackStart + 10ULL * kMs),
      kFallbackStart + 10ULL * kMs);

  // A persistent forward jump is fatal on the third monotonic packet.
  PacketTimestampClock forward_jump;
  expect_publish(forward_jump.map(lidar, ptp, first_ptp_raw, kFallbackStart),
                 kFallbackStart);
  expect_drop(forward_jump.map(
      lidar, ptp, first_ptp_raw + 10ULL * kSec, kFallbackStart + 5ULL * kMs));
  expect_drop(forward_jump.map(
      lidar, ptp, first_ptp_raw + 10ULL * kSec + 5ULL * kMs,
      kFallbackStart + 10ULL * kMs));
  expect_fatal(forward_jump.map(
      lidar, ptp, first_ptp_raw + 10ULL * kSec + 10ULL * kMs,
      kFallbackStart + 15ULL * kMs));
  assert(forward_jump.faulted());

  // A device restart/large rollback is fatal after two confirming packets.
  PacketTimestampClock rollback;
  expect_publish(rollback.map(imu, ptp, first_ptp_raw, kFallbackStart),
                 kFallbackStart);
  expect_publish(rollback.map(
      imu, ptp, first_ptp_raw + 10ULL * kSec, kFallbackStart + 10ULL * kSec),
      kFallbackStart + 10ULL * kSec);
  expect_drop(rollback.map(
      imu, ptp, 50ULL * kMs, kFallbackStart + 10ULL * kSec + 5ULL * kMs));
  expect_fatal(rollback.map(
      imu, ptp, 55ULL * kMs, kFallbackStart + 10ULL * kSec + 10ULL * kMs));

  // A real ten-second input gap advances device and fallback equally and is OK.
  PacketTimestampClock real_gap;
  expect_publish(real_gap.map(lidar, ptp, first_ptp_raw, kFallbackStart),
                 kFallbackStart);
  expect_publish(real_gap.map(
      lidar, ptp, first_ptp_raw + 10ULL * kSec, kFallbackStart + 10ULL * kSec),
      kFallbackStart + 10ULL * kSec);

  // PTP <-> GPS changes epoch semantics and must restart rather than reuse an anchor.
  PacketTimestampClock source_change;
  expect_publish(source_change.map(lidar, ptp, first_ptp_raw, kFallbackStart),
                 kFallbackStart);
  expect_fatal(source_change.map(
      lidar, PacketTimestampSource::Gps, first_ptp_raw + 5ULL * kMs,
      kFallbackStart + 5ULL * kMs));

  // Checked arithmetic rejects overflow; it never emits UINT64_MAX.
  PacketTimestampClock overflow;
  const std::uint64_t near_max = std::numeric_limits<std::uint64_t>::max() - 2ULL;
  expect_publish(overflow.map(lidar, ptp, 1ULL, near_max), near_max);
  const auto overflowed = overflow.map(lidar, ptp, 10ULL, near_max);
  expect_fatal(overflowed);
  assert(overflowed.stamp_ns != std::numeric_limits<std::uint64_t>::max());

  return 0;
}
