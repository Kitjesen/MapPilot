#include "slam.hpp"
#include "frames.hpp"
#include "ltlio_geometry_json.hpp"
#include "../../src/drivers/real/lidar/native/module.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace slam = lingtu::slam;
namespace wire = lingtu::drivers::lidar;

namespace {
#pragma pack(push, 1)
struct Header {
  char magic[4];
  std::uint8_t type;
  std::uint8_t reserved[3];
  std::uint64_t stamp_ns;
  std::uint32_t sequence;
  std::uint32_t count;
  std::uint32_t bytes;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 28);

void readExact(std::istream& source, void* data, std::size_t bytes) {
  if (!source.read(static_cast<char*>(data), static_cast<std::streamsize>(bytes)))
    throw std::runtime_error("truncated LTU1 record");
}

void number(std::ostream& stream, double value) {
  if (std::isfinite(value)) stream << value;
  else stream << "null";
}

void pose(std::ostream& stream, double stamp, const slam::Pose3d& value) {
  stream << stamp << ' ' << value.x << ' ' << value.y << ' ' << value.z << ' '
         << value.qx << ' ' << value.qy << ' ' << value.qz << ' ' << value.qw << '\n';
}

void status(std::ostream& stream, const slam::SlamOutputs& out, double tick_ms) {
  const auto& elevator = out.elevator;
  stream << "{\"stamp_s\":" << out.stamp_s
         << ",\"scan_end_s\":" << out.scan_end_s << ",\"observation_sequence\":"
         << out.observation_sequence << ",\"state\":" << std::quoted(slam::toString(out.state))
         << ",\"reason\":" << std::quoted(out.reason)
         << ",\"tick_ms\":" << tick_ms << ",\"elevator\":{\"mode\":" << std::quoted(elevator.mode)
         << ",\"phase\":" << std::quoted(elevator.phase) << ",\"active\":"
         << (elevator.active ? "true" : "false") << ",\"height_m\":";
  number(stream, elevator.height_m);
  stream << ",\"velocity_mps\":"; number(stream, elevator.velocity_mps);
  stream << ",\"acceleration_mps2\":"; number(stream, elevator.acceleration_mps2);
  stream << ",\"range_quantile_m\":"; number(stream, elevator.range_quantile_m);
  stream << ",\"velocity_variance\":"; number(stream, elevator.velocity_variance);
  stream << ",\"entries\":" << elevator.entries << ",\"exits\":" << elevator.exits
         << "},\"lidar_update\":{\"attempt_sequence\":" << out.ltlio_lidar_update.attempt_sequence
         << ",\"attempted\":" << (out.ltlio_lidar_update.attempted ? "true" : "false")
         << ",\"accepted\":" << (out.ltlio_lidar_update.accepted ? "true" : "false")
         << ",\"rejection_reason\":" << std::quoted(out.ltlio_lidar_update.rejection_reason)
         << ",\"consecutive_rejections\":" << out.ltlio_lidar_update.consecutive_rejections
         << ",\"sampled_points\":" << out.ltlio_lidar_update.downsampled_points
         << ",\"plane_matches\":" << out.ltlio_lidar_update.plane_matches
         << ",\"point_matches\":" << out.ltlio_lidar_update.point_matches
         << ",\"mapping_points\":" << out.ltlio_lidar_update.mapping_points
         << ",\"sampling_leaf_size_m\":" << out.ltlio_lidar_update.sampling_leaf_size_m
         << ",\"sampling_scale_m\":" << out.ltlio_lidar_update.sampling_scale_m
         << "},\"condition_number\":";
  number(stream, out.ltlio_condition_number);
  stream << ",\"position_covariance_trace\":"; number(stream, out.ltlio_pos_cov_trace);
  stream << ",\"odom_z\":";
  if (out.odometry_odom_body) number(stream, out.odometry_odom_body->z); else stream << "null";
  stream << ",\"world_z\":";
  if (out.odometry_odom_body && out.map_odom_tf)
    number(stream, slam::composePoses(out.map_odom_tf->pose, *out.odometry_odom_body).z);
  else stream << "null";
  stream << ",\"geometry\":" << slam::ltlioGeometryJson(out) << "}\n";
}

void usage(std::ostream& stream) {
  stream << "Usage: ltlio_replay INPUT.ltu CONFIG.yaml OUTPUT_PREFIX [RECORDED_EVENTS.tsv]"
            " [--save-map MAP_DIRECTORY]\n"
         << "Events contain original bag_timestamp_ns and 0/1; no time restamping.\n"
         << "--save-map exports a native SLAM snapshot into a new directory.\n"
         << "This is offline component evidence, not robot or motion acceptance.\n";
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    usage(std::cout);
    return 0;
  }
  if (argc < 4) {
    usage(std::cerr);
    return 2;
  }
  try {
    std::string events_path;
    std::optional<std::filesystem::path> map_directory;
    for (int argument = 4; argument < argc; ++argument) {
      const std::string value(argv[argument]);
      if (value == "--save-map") {
        if (map_directory || argument + 1 >= argc || std::string(argv[argument + 1]).empty()
            || std::string(argv[argument + 1]).rfind("--", 0) == 0)
          throw std::runtime_error("--save-map requires one new map directory");
        map_directory = std::filesystem::absolute(argv[++argument]).lexically_normal();
      } else if (!value.empty() && value.front() != '-' && events_path.empty()) {
        events_path = value;
      } else {
        throw std::runtime_error("unexpected replay argument: " + value);
      }
    }
    if (map_directory && std::filesystem::exists(*map_directory))
      throw std::runtime_error("map output directory already exists: " + map_directory->string());
    const std::filesystem::path prefix(argv[3]);
    const std::vector<std::string> suffixes{".odom.tum", ".map.tum", ".status.jsonl", ".events.jsonl", ".summary.json"};
    for (const auto& suffix : suffixes)
      if (std::filesystem::exists(prefix.string() + suffix))
        throw std::runtime_error("replay output already exists: " + prefix.string() + suffix);
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) throw std::runtime_error("cannot open LTU1 input");
    std::vector<std::pair<std::uint64_t, bool>> manual;
    if (!events_path.empty()) {
      std::ifstream source(events_path);
      if (!source) throw std::runtime_error("cannot open recorded events");
      std::uint64_t stamp; int active;
      while (source >> stamp >> active) {
        if (active != 0 && active != 1) throw std::runtime_error("manual event must be 0 or 1");
        if (!manual.empty() && stamp < manual.back().first) throw std::runtime_error("manual event order regressed");
        manual.emplace_back(stamp, active != 0);
      }
      if (!source.eof()) throw std::runtime_error("invalid recorded event row");
    }
    auto backend = slam::makeLtlioBackend();
    slam::SlamConfig config;
    config.config_path = argv[2];
    auto configured = backend->configure(config);
    if (!configured.ok) throw std::runtime_error(configured.message);
    configured = backend->setMode(slam::SlamMode::Mapping, "");
    if (!configured.ok) throw std::runtime_error(configured.message);
    if (map_directory && !std::filesystem::create_directories(*map_directory))
      throw std::runtime_error("map output directory already exists: " + map_directory->string());
    if (!prefix.parent_path().empty()) std::filesystem::create_directories(prefix.parent_path());
    std::ofstream odom(prefix.string() + ".odom.tum"), map(prefix.string() + ".map.tum"),
        states(prefix.string() + ".status.jsonl"), events(prefix.string() + ".events.jsonl");
    if (!odom || !map || !states || !events) throw std::runtime_error("cannot open replay outputs");
    for (auto* stream : {&odom, &map, &states, &events}) *stream << std::setprecision(17);
    std::uint64_t records = 0, clouds = 0, imus = 0, poses = 0, feed_errors = 0;
    std::uint64_t last_observation = 0, last_attempt = 0, points = 0;
    std::uint64_t observations = 0, attempts = 0, accepted_updates = 0, rejected_attempts = 0;
    std::uint16_t point_flags = 0;
    std::size_t manual_index = 0;
    std::vector<double> timings;
    std::map<std::string, std::size_t> observation_state_counts, cloud_arrival_state_counts;
    std::tuple<std::string, std::string, std::string, std::string, bool> previous;
    bool have_previous = false;
    double first_z = 0, last_z = 0, min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    std::uint64_t world_poses = 0;
    double first_world_z = 0, last_world_z = 0;
    double world_min_z = std::numeric_limits<double>::infinity();
    double world_max_z = -std::numeric_limits<double>::infinity();
    double first_stamp = 0, last_stamp = 0;
    const auto start = std::chrono::steady_clock::now();
    auto tick = [&]() {
      const auto started = std::chrono::steady_clock::now();
      const auto result = backend->tick();
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      const auto out = backend->outputs();
      if (!result.ok) ++feed_errors;
      auto identity = std::make_tuple(slam::toString(out.state), out.reason, out.elevator.mode, out.elevator.phase, out.elevator.active);
      if (!have_previous || identity != previous) {
        status(events, out, ms);
        previous = std::move(identity);
        have_previous = true;
      }
      const bool new_observation = out.observation_sequence != last_observation;
      const bool new_attempt = out.ltlio_lidar_update.attempt_sequence != last_attempt;
      if (!new_observation && !new_attempt) return;
      if (new_attempt) {
        last_attempt = out.ltlio_lidar_update.attempt_sequence;
        if (last_attempt != 0) {
          ++attempts;
          if (out.ltlio_lidar_update.accepted) ++accepted_updates;
          else ++rejected_attempts;
          timings.push_back(ms);
        }
      }
      status(states, out, ms);
      if (!new_observation) return;
      last_observation = out.observation_sequence;
      ++observations;
      ++observation_state_counts[slam::toString(out.state)];
      if (!out.odometry_odom_body) return;
      pose(odom, out.stamp_s, *out.odometry_odom_body);
      if (out.map_odom_tf) {
        const auto world = slam::composePoses(out.map_odom_tf->pose, *out.odometry_odom_body);
        pose(map, out.stamp_s, world);
        last_world_z = world.z;
        if (!world_poses) first_world_z = world.z;
        world_min_z = std::min(world_min_z, world.z);
        world_max_z = std::max(world_max_z, world.z);
        ++world_poses;
      }
      last_z = out.odometry_odom_body->z;
      if (poses == 0) first_z = last_z;
      min_z = std::min(min_z, last_z); max_z = std::max(max_z, last_z);
      ++poses;
    };
    while (input.peek() != std::char_traits<char>::eof()) {
      Header header{};
      readExact(input, &header, sizeof(header));
      if (std::memcmp(header.magic, "LTU1", 4) != 0) throw std::runtime_error("invalid LTU1 magic");
      const double stamp = static_cast<double>(header.stamp_ns) * 1e-9;
      first_stamp = records == 0 ? stamp : std::min(first_stamp, stamp);
      last_stamp = std::max(last_stamp, stamp);
      while (manual_index < manual.size() && manual[manual_index].first <= header.stamp_ns) {
        const auto applied = backend->setElevator(manual[manual_index].second);
        if (!applied.ok) throw std::runtime_error("recorded elevator event rejected: " + applied.message);
        ++manual_index;
      }
      slam::Status accepted;
      if (header.type == 1) {
        if (static_cast<std::uint64_t>(header.count) * sizeof(wire::Point) != header.bytes)
          throw std::runtime_error("LTU1 cloud size mismatch");
        std::vector<wire::Point> source(header.count);
        readExact(input, source.data(), header.bytes);
        slam::LidarFrame frame;
        frame.stamp_s = stamp;
        frame.frame_id = "livox_frame";
        frame.points.reserve(source.size());
        for (const auto& point : source) {
          frame.points.push_back({point.x, point.y, point.z, point.intensity,
                                  point.offset_time_ns, point.line, point.tag});
          point_flags |= point.flags;
        }
        accepted = backend->feedLidar(frame);
        ++clouds; points += header.count;
      } else if (header.type == 2) {
        if (header.count != 1 || header.bytes != sizeof(wire::ImuSample))
          throw std::runtime_error("LTU1 IMU size mismatch");
        wire::ImuSample source{};
        readExact(input, &source, sizeof(source));
        slam::ImuSample sample;
        sample.stamp_s = stamp;
        sample.gx = source.gyro_x; sample.gy = source.gyro_y; sample.gz = source.gyro_z;
        sample.ax = source.acc_x; sample.ay = source.acc_y; sample.az = source.acc_z;
        accepted = backend->feedImu(sample);
        ++imus;
      } else throw std::runtime_error("replay accepts LiDAR/IMU records only");
      if (!accepted.ok) ++feed_errors;
      ++records;
      tick();
      if (header.type == 1) ++cloud_arrival_state_counts[slam::toString(backend->outputs().state)];
      if (clouds && clouds % 500 == 0 && header.type == 1)
        std::cout << "clouds=" << clouds << " poses=" << poses << " state=" << slam::toString(backend->outputs().state) << std::endl;
    }
    // Do not invent IMU samples after the final scan merely to complete deskew.
    tick();
    const auto final = backend->outputs();
    const double replay_elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::optional<slam::Status> map_save;
    double map_save_elapsed_s = 0.0;
    if (map_directory) {
      const auto save_started = std::chrono::steady_clock::now();
      std::cout << "saving_map=" << map_directory->generic_string() << std::endl;
      try {
        map_save = backend->saveMap((*map_directory / "map.pcd").string());
      } catch (const std::exception& error) {
        map_save = slam::Status::Error(error.what());
      }
      map_save_elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - save_started).count();
    }
    std::sort(timings.begin(), timings.end());
    std::ofstream report(prefix.string() + ".summary.json");
    report << std::setprecision(17) << "{\"records\":" << records << ",\"clouds\":" << clouds
           << ",\"imus\":" << imus << ",\"points\":" << points << ",\"poses\":" << poses
           << ",\"lidar_update_attempts\":" << attempts << ",\"accepted_updates\":" << accepted_updates
           << ",\"rejected_attempts\":" << rejected_attempts << ",\"published_observations\":" << observations
           << ",\"clouds_without_lidar_attempt\":" << (clouds >= attempts ? clouds - attempts : 0)
           << ",\"clouds_without_published_observation\":" << (clouds >= observations ? clouds - observations : 0)
           << ",\"input_point_flags_or\":" << point_flags << ",\"feed_or_tick_errors\":" << feed_errors
           << ",\"dropped_imu_frames\":" << final.dropped_imu_frames
           << ",\"dropped_lidar_frames\":" << final.dropped_lidar_frames
           << ",\"manual_events_applied\":" << manual_index << ",\"manual_events_total\":" << manual.size()
           << ",\"sensor_duration_s\":" << last_stamp - first_stamp << ",\"first_z\":" << first_z
           << ",\"last_z\":" << last_z << ",\"min_z\":";
    number(report, min_z); report << ",\"max_z\":"; number(report, max_z);
    report << ",\"world_poses\":" << world_poses << ",\"first_world_z\":" << first_world_z
           << ",\"last_world_z\":" << last_world_z << ",\"world_min_z\":";
    number(report, world_min_z); report << ",\"world_max_z\":"; number(report, world_max_z);
    report << ",\"elapsed_s\":" << replay_elapsed_s
           << ",\"lidar_attempt_tick_p95_ms\":" << (timings.empty() ? 0.0 : timings[static_cast<std::size_t>(0.95 * (timings.size()-1))])
           << ",\"final_state\":" << std::quoted(slam::toString(final.state))
           << ",\"final_reason\":" << std::quoted(final.reason) << ",\"elevator_entries\":" << final.elevator.entries
           << ",\"elevator_exits\":" << final.elevator.exits << ",\"published_observation_state_counts\":{";
    bool separator = false;
    for (const auto& [name, count] : observation_state_counts) {
      if (separator) report << ',';
      report << std::quoted(name) << ':' << count; separator = true;
    }
    report << "},\"input_cloud_arrival_state_counts\":{";
    separator = false;
    for (const auto& [name, count] : cloud_arrival_state_counts) {
      if (separator) report << ',';
      report << std::quoted(name) << ':' << count; separator = true;
    }
    report << '}';
    if (map_save) {
      report << ",\"map_save\":{\"ok\":" << (map_save->ok ? "true" : "false")
             << ",\"directory\":" << std::quoted(map_directory->generic_string())
             << ",\"pcd\":" << std::quoted((*map_directory / "map.pcd").generic_string())
             << ",\"message\":" << std::quoted(map_save->message)
             << ",\"elapsed_s\":" << map_save_elapsed_s << '}';
    }
    report << "}\n";
    for (auto* stream : {&odom, &map, &states, &events, &report}) {
      stream->close();
      if (!*stream) throw std::runtime_error("cannot finish replay outputs");
    }
    if (map_save && !map_save->ok) {
      std::cerr << "map export failed: " << map_save->message << '\n';
      return 3;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
