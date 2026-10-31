#define main camera_video_main
#include "../../../../../src/drivers/real/camera/native/video/stream_main.cpp"
#undef main

#include <cassert>

int main() {
  using namespace std::chrono_literals;
  const Options options;
  assert(options.max_age_ms == 250);
  const auto max_age_ns = static_cast<std::uint64_t>(options.max_age_ms) * 1000000ULL;
  const auto accepted_at = Clock::time_point(10s);
  const auto last_fresh = frameFreshTime(accepted_at, 50'000'000);

  // The wall clock moves back one hour after accepting a frame aged 50 ms.
  const std::uint64_t capture_wall_ns = 7'200'000'000'000;
  const std::uint64_t regressed_wall_ns = capture_wall_ns - 3'600'000'000'000;
  const auto old_wall_age_ns = regressed_wall_ns > capture_wall_ns
                                   ? regressed_wall_ns - capture_wall_ns : 0;
  assert(old_wall_age_ns == 0);
  assert(frameAgeNs(accepted_at, last_fresh) == 50'000'000);
  assert(frameAgeNs(accepted_at + 199ms, last_fresh) < max_age_ns);
  assert(frameAgeNs(accepted_at + 200ms, last_fresh) == max_age_ns);
  assert(frameAgeNs(accepted_at + 200ms + 1ns, last_fresh) > max_age_ns);
  assert(frameAgeNs(accepted_at + 1h, last_fresh) > max_age_ns);

  const auto fresh = frameFreshTime(accepted_at, 0);
  assert(frameAgeNs(accepted_at + 250ms, fresh) == max_age_ns);
  assert(frameAgeNs(accepted_at + 250ms + 1ns, fresh) > max_age_ns);
  std::cout << "video_clock backward_wall_step=3600s expires_after=250ms initial_age=preserved\n";
}
