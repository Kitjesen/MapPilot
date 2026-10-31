#include "lingtu/recording/history_series.hpp"
#include "lingtu/recording/message_preview.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "messages.h"
#include "typed_cdr_fixture.hpp"
using lingtu::recording::test::serialize;
#if __has_include("dds/cdr/dds_cdrstream.h")
#include "dds/cdr/dds_cdrstream.h"
#define LINGTU_CDR_STREAM_ALLOCATOR 1
#else
#include "dds/ddsi/ddsi_cdrstream.h"
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
namespace rec = lingtu::recording;
void require(bool condition, const char *message) { if (!condition) throw std::runtime_error(message); }

const rec::TopicBinding &topic(const char *name) {
  const auto *binding = rec::find_recording_topic(name);
  require(binding, "fixture topic absent");
  return *binding;
}
constexpr std::uint64_t origin_a = 1792902000000000000ULL;
void write_session(const std::filesystem::path &root, const std::string &name,
                   const std::string &idl, bool comparison_b) {
  const auto directory = std::filesystem::absolute(root / name);
  std::filesystem::create_directories(directory / "dds");
  const auto &imu_binding = topic("/imu/raw");
  const auto &joint_binding = topic("/robot/joint_states");
  const auto &health_binding = topic("/slam/localization_health");
  rec::McapSessionWriter writer(directory / "dds/sensors.mcap", idl,
    {rec::channel_definition(imu_binding), rec::channel_definition(joint_binding), rec::channel_definition(health_binding)}, 32768);
  const auto origin = origin_a + (comparison_b ? 60000000000ULL : 0ULL);
  const double shift = comparison_b ? 0.75 : 0;
  const double amplitude = comparison_b ? 1.2 : 1;
  for (int index = 0; index <= 6000; ++index) {
    const double t = index / 500.0;
    if (t >= 5 + shift && t < 6 + shift) continue;
    double value = amplitude * std::sin((t - shift) * 2 * 3.14159265358979323846 / 3);
    if (index == (comparison_b ? 4937 : 4562)) value = 8 * amplitude;
    // Two separate invalid intervals inside a single coarse min/max bucket.
    if ((index >= 700 && index < 705) || (index >= 720 && index < 725)) value = std::numeric_limits<double>::quiet_NaN();
    const auto log_time = origin + static_cast<std::uint64_t>(index) * 2000000ULL;
    lingtu_dds_Imu imu{};
    imu.header.frame_id = const_cast<char *>("synthetic/body");
    imu.header.stamp.sec = static_cast<std::int32_t>(log_time / 1000000000);
    imu.header.stamp.nanosec = static_cast<std::uint32_t>(log_time % 1000000000);
    imu.orientation.w = 1;
    imu.angular_velocity.x = value;
    imu.linear_acceleration.z = 9.81;
    writer.write({std::string(imu_binding.contract->dds_topic), log_time, log_time - 2000000, static_cast<std::uint32_t>(index), serialize(imu_binding, &imu)});
    lingtu_dds_JointState joint{};
    joint.header = imu.header;
    joint.robot_model = const_cast<char *>("SYNTHETIC_TEST_ONLY");
    char *names[]{const_cast<char *>("joint_0"), const_cast<char *>("joint_1")};
    double positions[]{value, 0.5 * value}, velocities[]{0.2, -0.2}, torques[]{1, 2};
    joint.names = {2, 2, names, false};
    joint.position = {2, 2, positions, false};
    joint.velocity = {2, 2, velocities, false};
    joint.effort = {2, 2, torques, false};
    writer.write({std::string(joint_binding.contract->dds_topic), log_time, log_time - 2000000, static_cast<std::uint32_t>(index), serialize(joint_binding, &joint)});
    // Distinct messages at the same timestamp must not collapse.
    if (index == 250) writer.write({std::string(joint_binding.contract->dds_topic), log_time, log_time, 7000, serialize(joint_binding, &joint)});
  }
  lingtu_dds_Text health{};
  health.data = const_cast<char *>("Synthetic fixture; not a robot recording");
  writer.write({std::string(health_binding.contract->dds_topic), origin, origin, 0, serialize(health_binding, &health)});
  writer.commit();
  std::ofstream manifest(directory / "session.json");
  manifest << "{\"version\":1,\"session_id\":" << rec::json_quote(name)
    << ",\"state\":\"completed\",\"session_directory\":" << rec::json_quote(directory.generic_string())
    << ",\"context\":{\"product\":\"synthetic_test\",\"robot_id\":\"SYNTHETIC_TEST_ONLY\"},\"manager_pid\":12345"
    << ",\"created_at_unix_ns\":" << origin << ",\"started_at_unix_ns\":" << origin
    << ",\"ended_at_unix_ns\":" << origin + 12000000000ULL
    << ",\"children\":[{\"name\":\"dds\",\"required\":true,\"pid\":0,\"state\":\"exited\",\"exit_code\":0,\"argv\":[],\"artifacts\":[\"dds/sensors.mcap\"],\"selected_topics\":[\"/imu/raw\",\"/robot/joint_states\",\"/slam/localization_health\"],\"required_topics\":[]}]}\n";
  std::ofstream(directory / "SYNTHETIC.txt") << "Native typed CDR/MCAP test fixture. No robot input.\n500 Hz; 12 s; B wall origin +60 s, signal phase +0.75 s, amplitude x1.2.\nOne 1 s gap, two nonfinite intervals and narrow one-sample peak.\n";
}
void verify(const std::filesystem::path &root) {
  const auto flattened = rec::numeric_preview("{\"enabled\":true,\"text\":\"42\",\"values\":[1,null,3]}");
  require(flattened.values.size() == 3 && flattened.values[1].path == "values[1]" &&
          !flattened.values[1].value, "numeric field discovery converted bool/text/null incorrectly");
  const std::byte truncated[]{std::byte{0}, std::byte{1}, std::byte{0}, std::byte{0}, std::byte{0}};
  bool malformed_rejected = false;
  try { rec::message_preview(topic("/robot/joint_states"), truncated, sizeof(truncated)); }
  catch (const std::runtime_error &) { malformed_rejected = true; }
  require(malformed_rejected, "truncated typed recording was accepted");
  const auto file_a = root / "synthetic_compare_a/dds/sensors.mcap";
  rec::SeriesQuery query{"/robot/joint_states"};
  auto discovery = rec::read_history_series(file_a, query);
  require(discovery.field == "position_rad[0]", "discovery chose a counter instead of joint position");
  require(discovery.start_time_ns == origin_a && discovery.duration_s == 12, "record origin or duration changed");
  require(discovery.original_count == 5502, "record topic count is wrong");
  require(discovery.frame_id == "synthetic/body", "frame metadata missing");
  require(discovery.samples.size() <= 2000, "output bound exceeded");
  query.max_points = 16;
  auto reduced = rec::read_history_series(file_a, query);
  require(reduced.downsampled && reduced.samples.size() <= 16, "coarse sampling exceeded requested cap");
  require(std::any_of(reduced.samples.begin(), reduced.samples.end(), [](const auto &p) { return p.value && *p.value == 8; }), "minmax lost the narrow spike");
  require(std::any_of(reduced.samples.begin(), reduced.samples.end(), [](const auto &p) { return p.gap_before && p.t >= 6; }), "downsampling bridged recorded gap");
  require(std::any_of(reduced.samples.begin(), reduced.samples.end(), [](const auto &p) { return p.gap_before && p.value && p.t > 1.448 && p.t < 5; }), "later nonfinite interval in same bucket became a connecting line");
  query.start_s = 0.498; query.end_s = 0.502; query.max_points = 2000;
  auto duplicates = rec::read_history_series(file_a, query);
  require(duplicates.read_count == 4 && duplicates.samples.size() == 4, "equal-time messages collapsed");
  require(duplicates.samples[1].t == duplicates.samples[2].t, "duplicate timestamps changed");
  query.start_s = 5.1; query.end_s = 5.9;
  require(rec::read_history_series(file_a, query).read_count == 0, "empty recorded interval filled with fake samples");
  query.start_s = 99; query.end_s = 100;
  auto outside = rec::read_history_series(file_a, query);
  require(outside.samples.empty() && outside.duration_s == 12 && outside.original_count == 5502, "nonoverlap lost recording metadata");
  query = {"/slam/localization_health"};
  auto text = rec::read_history_series(file_a, query);
  require(text.fields.empty() && text.samples.empty(), "text turned into a numeric curve");
  query.field = "data";
  bool rejected = false;
  try { rec::read_history_series(file_a, query); } catch (const rec::SeriesError &error) { rejected = error.code == "recording_series_unsupported_field"; }
  require(rejected, "nonnumeric field accepted");
  query = {"/imu/raw", "angular_velocity_rad_s.x"};
  query.start_s = 1.38; query.end_s = 1.46;
  auto nonfinite = rec::read_history_series(file_a, query);
  require(std::count_if(nonfinite.samples.begin(), nonfinite.samples.end(), [](const auto &p) { return !p.value; }) == 10, "nonfinite samples became zero or disappeared");
  query = {"/robot/joint_states", "position_rad[0]"};
  auto compared = rec::read_history_series(root / "synthetic_compare_b/dds/sensors.mcap", query);
  require(compared.start_time_ns - origin_a == 60000000000ULL, "source timestamp offset lost");
  const auto peak = *std::max_element(compared.samples.begin(), compared.samples.end(), [](const auto &a, const auto &b) { return a.value.value_or(-100) < b.value.value_or(-100); });
  require(peak.value && std::abs(*peak.value - 9.6) < 1e-8 && std::abs(peak.t - 9.874) < 1e-8, "B known phase/amplitude changed");
  std::cout << "native history: indexed window, typed arrays, units, timestamps, duplicate, gap, null and minmax peak checks passed\n";
}
}
int main(int argc, char **argv) {
  try {
    const bool fixtures = argc == 4 && std::string(argv[1]) == "--fixtures";
    const auto root = fixtures ? std::filesystem::path(argv[2]) : std::filesystem::temp_directory_path() /
        ("lingtu-history-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto idl = rec::read_text_file(fixtures ? argv[3] : LINGTU_RECORDING_DEFAULT_IDL);
    write_session(root, "synthetic_compare_a", idl, false);
    write_session(root, "synthetic_compare_b", idl, true);
    verify(root);
    if (fixtures) std::cout << root.string() << '\n';
    else std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
