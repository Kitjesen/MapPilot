#include "components.hpp"

#include <array>

namespace lingtu::native_runtime {
namespace {

constexpr std::array<Component, 9> kComponents{{
    {
        "lidar",
        RuntimeLane::SensorInput,
        State::Ready,
        "src/drivers/real/lidar/sdk2_stream",
        "livox_sdk2_stream",
        "systemctl start lingtu-livox-dds.service",
        "Livox MID-360 packet ingestion and native DDS publication.",
        true,
        false,
    },
    {
        "slam",
        RuntimeLane::Localization,
        State::Ready,
        "src/localization/slam/cpp",
        "lingtu_slam_cyclone_runtime",
        "systemctl start lingtu-slam-dds.service",
        "Fast-LIO/native SLAM runtime consuming lidar/IMU DDS and publishing odometry, registered cloud, map cloud, and TF.",
        true,
        false,
    },
    {
        "slamctl",
        RuntimeLane::Localization,
        State::Ready,
        "src/localization/slam/cpp",
        "lingtu_slam_control",
        "use Gateway restart/relocalize controls or call lingtu_slam_control directly on the robot",
        "Native SLAM control command entrypoint for restart, mode, and map operations.",
        true,
        false,
    },
    {
        "relocalize",
        RuntimeLane::Relocalization,
        State::Ready,
        "src/localization/slam/cpp",
        "lingtu_slam_contract",
        "use scene relocalize controls through Gateway",
        "Native relocalization support linked into the SLAM contract when BBS3D/small_gicp are available.",
        true,
        false,
    },
    {
        "nav",
        RuntimeLane::Navigation,
        State::Ready,
        "src/nav/services/endpoint/cpp",
        "lingtu_nav_native_endpoint",
        "systemctl start lingtu-nav-dds.service",
        "Native navigation endpoint for goal intake, OctoPlanner3D planning, local target selection, path following, and cmd_vel publication.",
        true,
        false,
    },
    {
        "navctl",
        RuntimeLane::Navigation,
        State::Ready,
        "src/nav/services/endpoint/cpp",
        "lingtu_nav_control",
        "use Gateway navigation controls or call lingtu_nav_control directly on the robot",
        "Native navigation command utility for endpoint control.",
        true,
        false,
    },
    {
        "trav",
        RuntimeLane::Navigation,
        State::Ready,
        "src/nav/services/endpoint/cpp",
        "lingtu_traversability_dds",
        "systemctl start lingtu-traversability-dds.service",
        "Native traversability DDS producer for near-field navigation cost input.",
        true,
        false,
    },
    {
        "pgo",
        RuntimeLane::MapOptimization,
        State::Ready,
        "src/localization/opt",
        "lt_pgo",
        "lt_pgo --map MAP_DIR --out MAP_DIR",
        "Product-native save-time pose graph optimization using saved patches, poses.txt, and the portable pose_graph_opt kernel.",
        true,
        false,
    },
    {
        "hba",
        RuntimeLane::MapOptimization,
        State::Ready,
        "src/localization/opt",
        "lt_hba",
        "lt_hba --map MAP_DIR --out MAP_DIR",
        "Product-native high-quality save-time map refinement using saved patches, poses.txt, and the portable pose_graph_opt kernel.",
        false,
        false,
    },
}};

}  // namespace

const Component* components(std::size_t& count) noexcept {
  count = kComponents.size();
  return kComponents.data();
}

const Component* find(std::string_view name) noexcept {
  for (const auto& component : kComponents) {
    if (name == component.name) {
      return &component;
    }
  }
  return nullptr;
}

bool ready(std::string_view name) noexcept {
  const auto* component = find(name);
  return component != nullptr && component->state == State::Ready &&
         component->product_default && !component->requires_ros2;
}

const char* to_string(RuntimeLane lane) noexcept {
  switch (lane) {
    case RuntimeLane::SensorInput:
      return "sensor_input";
    case RuntimeLane::Localization:
      return "localization";
    case RuntimeLane::Relocalization:
      return "relocalization";
    case RuntimeLane::MapOptimization:
      return "map_optimization";
    case RuntimeLane::Navigation:
      return "navigation";
  }
  return "unknown";
}

const char* to_string(State state) noexcept {
  switch (state) {
    case State::Ready:
      return "ready";
    case State::Wip:
      return "wip";
    case State::Legacy:
      return "legacy";
    case State::Exp:
      return "exp";
  }
  return "unknown";
}

}  // namespace lingtu::native_runtime
