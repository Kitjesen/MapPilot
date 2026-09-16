// Read-only native DDS acquisition before calibrating supplementary Go2 LiDAR.
#include <unitree/idl/go2/LidarState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

using Cloud = sensor_msgs::msg::dds_::PointCloud2_;
using State = unitree_go::msg::dds_::LidarState_;
using Clock = std::chrono::steady_clock;

struct Capture {
  std::mutex mutex;
  Cloud cloud;
  std::size_t count{0};
  bool frozen{false};
  Clock::time_point first{}, last{};
  double received_wall_s{0.0};
  void receive(const void* sample) {
    std::lock_guard<std::mutex> lock(mutex);
    if (frozen) return;
    cloud = *static_cast<const Cloud*>(sample);
    last = Clock::now();
    received_wall_s = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (++count == 1) first = last;
  }
};

std::size_t writePcd(const std::filesystem::path& path, const Cloud& cloud) {
  std::array<std::uint32_t, 3> offsets{};
  for (std::size_t axis = 0; axis < offsets.size(); ++axis) {
    const std::string name(1, "xyz"[axis]);
    bool found = false;
    for (const auto& field : cloud.fields()) {
      if (field.name() != name) continue;
      if (field.datatype() != 7 || field.count() != 1 ||
          field.offset() > cloud.point_step() || cloud.point_step() - field.offset() < 4)
        throw std::runtime_error("unsupported XYZ field layout");
      offsets[axis] = field.offset();
      found = true;
    }
    if (!found) throw std::runtime_error("missing XYZ field");
  }
  if (cloud.is_bigendian() || cloud.point_step() == 0 ||
      static_cast<std::uint64_t>(cloud.width()) * cloud.point_step() > cloud.row_step() ||
      static_cast<std::uint64_t>(cloud.height()) * cloud.row_step() > cloud.data().size())
    throw std::runtime_error("unsupported byte order or truncated cloud");
  std::vector<std::array<float, 3>> points;
  for (std::uint32_t row = 0; row < cloud.height(); ++row) {
    for (std::uint32_t col = 0; col < cloud.width(); ++col) {
      const auto base = static_cast<std::size_t>(row) * cloud.row_step() +
                        static_cast<std::size_t>(col) * cloud.point_step();
      std::array<float, 3> point{};
      for (std::size_t axis = 0; axis < point.size(); ++axis)
        std::memcpy(&point[axis], cloud.data().data() + base + offsets[axis], sizeof(float));
      if (std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]))
        points.push_back(point);
    }
  }
  std::ofstream file(path, std::ios::binary);
  file.exceptions(std::ios::failbit | std::ios::badbit);
  file << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
       << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
       << points.size() << "\nDATA binary\n";
  for (const auto& point : points)
    file.write(reinterpret_cast<const char*>(point.data()), 3 * sizeof(float));
  return points.size();
}

int main(int argc, char** argv) {
  try {
    if (argc != 4 && argc != 5)
      throw std::runtime_error("usage: go2_lidar_capture INTERFACE SECONDS NEW_OUTPUT_DIR [MID360_SNAPSHOT]");
    const double seconds = std::stod(argv[2]);
    if (!std::isfinite(seconds) || seconds < 1 || seconds > 60)
      throw std::runtime_error("capture duration must be 1..60 seconds");
    const std::filesystem::path output(argv[3]);
    if (!std::filesystem::create_directory(output))
      throw std::runtime_error("output directory must be new");
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    std::array<Capture, 3> captures;
    const std::array<std::string, 3> names{{"cloud", "cloud_base", "cloud_deskewed"}};
    std::vector<std::unique_ptr<unitree::robot::ChannelSubscriber<Cloud>>> readers;
    for (std::size_t i = 0; i < captures.size(); ++i) {
      auto reader = std::make_unique<unitree::robot::ChannelSubscriber<Cloud>>("rt/utlidar/" + names[i]);
      reader->InitChannel([&, i](const void* sample) { captures[i].receive(sample); }, 1);
      readers.push_back(std::move(reader));
    }
    std::mutex state_mutex;
    State state;
    std::size_t state_count = 0;
    unitree::robot::ChannelSubscriber<State> state_reader("rt/utlidar/lidar_state");
    state_reader.InitChannel([&](const void* sample) {
      std::lock_guard<std::mutex> lock(state_mutex);
      state = *static_cast<const State*>(sample);
      ++state_count;
    }, 1);
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    for (auto& capture : captures) {
      std::lock_guard<std::mutex> lock(capture.mutex);
      capture.frozen = true;
    }
    if (argc == 5)
      std::filesystem::copy_file(argv[4], output / "mid360_registered.bin");
    state_reader.CloseChannel();
    for (auto& reader : readers) reader->CloseChannel();
    std::ofstream report(output / "summary.txt");
    report.exceptions(std::ios::failbit | std::ios::badbit);
    report << std::setprecision(17) << "read_only=1\n";
    bool complete = true;
    for (std::size_t i = 0; i < captures.size(); ++i) {
      auto& capture = captures[i];
      std::lock_guard<std::mutex> lock(capture.mutex);
      report << names[i] << " samples=" << capture.count;
      if (capture.count == 0) {
        complete = false;
      } else {
        const double span = std::chrono::duration<double>(capture.last - capture.first).count();
        const auto& header = capture.cloud.header();
        const double stamp = header.stamp().sec() + header.stamp().nanosec() * 1e-9;
        report << " hz=" << (span > 0 ? (capture.count - 1) / span : 0)
               << " frame=" << std::quoted(header.frame_id())
               << " stamp=" << stamp
               << " received_wall_s=" << capture.received_wall_s
               << " receive_minus_stamp_s=" << capture.received_wall_s - stamp
               << " finite_points=" << writePcd(output / (names[i] + ".pcd"), capture.cloud);
      }
      report << '\n';
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      report << "lidar_state samples=" << state_count;
      if (state_count) report << " firmware=" << std::quoted(state.firmware_version())
                             << " software=" << std::quoted(state.software_version())
                             << " sdk=" << std::quoted(state.sdk_version())
                             << " error=" << static_cast<unsigned>(state.error_state())
                             << " cloud_hz=" << state.cloud_frequency();
      report << '\n';
    }
    std::cout << output / "summary.txt" << '\n';
    return complete ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
