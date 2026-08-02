#pragma once

#include <memory>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::nav::commands {

struct NavigationStateSnapshot {
  double timestamp_s{0.0};
  std::string frame_id;
  std::string boot_id;
  std::uint64_t sequence{0U};
  std::int32_t control_mode{0};
  std::int32_t lifecycle_state{0};
  std::string active_task_id;
  std::string active_request_id;
  std::uint64_t goal_epoch{0U};
  std::string map_id;
  std::int64_t map_version{0};
  std::string map_hash;
  std::int32_t planning_state{0};
  std::int32_t execution_state{0};
  std::int32_t recovery_state{0};
  float progress{-1.0F};
  std::string authority;
  std::string hold_reason;
  std::string failure_code;
};

struct NavigationGoalStatusSnapshot {
  double timestamp_s{0.0};
  std::string frame_id;
  std::string boot_id;
  std::uint64_t sequence{0U};
  std::string task_id;
  std::string request_id;
  std::int32_t state{0};
  std::uint64_t goal_epoch{0U};
  std::string reason;
};

struct NavigationCommandReceipt {
  std::string task_id;
  std::string request_id;
  bool accepted{false};
  std::int32_t kind{0};
  std::string reason;
  double endpoint_timestamp_s{0.0};
  std::string diagnostic;
};

struct InspectionTaskCommandReceipt {
  std::string task_id;
  std::string request_id;
  bool accepted{false};
  std::int32_t kind{0};
  std::string reason;
  std::string run_id;
  double endpoint_timestamp_s{0.0};
  std::string diagnostic;
};

struct ExplorationCommandReceipt {
  bool accepted{false};
  std::string request_id;
  std::string exploration_run_id;
  std::string reason;
  bool duplicate{false};
};

// Immutable native inspection task fact. This carries the endpoint replay
// cursor so the Host can surface a delivery gap instead of inferring progress
// from command ACKs or snapshots.
struct InspectionTaskEventSnapshot {
  double timestamp_s{0.0};
  std::string frame_id;
  std::string boot_id;
  std::uint64_t event_sequence{0U};
  std::int32_t kind{0};
  std::string task_id;
  std::string request_id;
  std::string command_request_id;
  std::int32_t state{0};
  std::string map_id;
  std::int64_t map_version{0};
  std::string route_id;
  std::uint64_t route_revision{0U};
  std::uint32_t point_index{0U};
  std::uint32_t point_count{0U};
  std::uint32_t loop_index{0U};
  std::uint32_t retry_count{0U};
  std::string point_id;
  std::string action;
  std::string action_request_id;
  std::string evidence_id;
  std::string reason;
};

struct ExplorationRunEventSnapshot {
  double timestamp_s{0.0};
  std::string frame_id;
  std::string boot_id;
  std::uint64_t event_sequence{0U};
  std::int32_t kind{0};
  std::string exploration_run_id;
  std::string start_request_id;
  std::string command_request_id;
  std::string product_session_id;
  std::int32_t state{0};
  std::string route;
  std::string map_id;
  std::int64_t map_version{0};
  std::string artifact_hash;
  std::string reason;
  bool motion_stop_confirmed{false};
  std::string motion_stop_reason;
};

struct OperatorMotionCommandReceipt {
  bool accepted{false};
  std::int32_t action{0};
  std::string request_id;
  std::string source_id;
  std::uint64_t source_epoch{0U};
  std::uint64_t source_sequence{0U};
  std::uint64_t accepted_sequence{0U};
  std::uint64_t final_output_sequence{0U};
  double endpoint_timestamp_s{0.0};
  std::string reason;
};

struct PathPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PathSnapshot {
  double timestamp_s{0.0};
  std::string frame_id;
  std::uint64_t receive_sequence{0U};
  std::vector<PathPoint> points;
};

struct MapScenePoint {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct MapSceneGridSnapshot {
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  float resolution{0.0F};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};
  double origin_qx{0.0};
  double origin_qy{0.0};
  double origin_qz{0.0};
  double origin_qw{1.0};
  std::vector<float> cells;
};

struct MapSceneSnapshot {
  double timestamp_s{0.0};
  std::string frame_id;
  std::string producer_boot_id;
  std::uint64_t receive_sequence{0U};
  std::uint64_t reset_epoch{0U};
  std::uint64_t observation_sequence{0U};
  std::uint64_t generation{0U};
  bool live{false};
  double sensor_x{0.0};
  double sensor_y{0.0};
  double sensor_z{0.0};
  double sensor_qx{0.0};
  double sensor_qy{0.0};
  double sensor_qz{0.0};
  double sensor_qw{1.0};
  std::size_t payload_bytes{0U};
  std::vector<MapScenePoint> live_points;
  std::vector<MapScenePoint> voxel_points;
  std::vector<MapScenePoint> accumulated_points;
  MapSceneGridSnapshot occupancy;
  MapSceneGridSnapshot elevation;
  MapSceneGridSnapshot esdf;
};

struct MapSceneHealthSnapshot {
  std::uint64_t received_samples{0U};
  std::uint64_t valid_samples{0U};
  std::uint64_t stale_samples{0U};
  std::uint64_t invalid_samples{0U};
  std::uint64_t capacity_rejections{0U};
  std::uint64_t replaced_samples{0U};
  std::uint64_t last_receive_sequence{0U};
  std::uint64_t last_generation{0U};
  double last_sample_timestamp_s{0.0};
  bool pending{false};
  std::string last_error;
  std::uint64_t state_received_samples{0U};
  std::uint64_t state_valid_samples{0U};
  std::uint64_t state_stale_samples{0U};
  std::uint64_t state_invalid_samples{0U};
  double state_timestamp_s{0.0};
  std::string state_producer_boot_id;
  bool state_received{false};
  bool state_running{false};
  bool state_live{false};
  bool state_required_publications_ready{false};
  bool state_current_generation_published{false};
  bool state_capacity_limited{false};
  std::uint64_t state_reset_epoch{0U};
  std::uint64_t state_observation_sequence{0U};
  std::uint64_t state_generation{0U};
  std::uint64_t state_scene_published_generation{0U};
  std::string state_error;
};

class Client {
 public:
  class NavigationCommands {
   public:
    [[nodiscard]] NavigationCommandReceipt startTask(
        double x,
        double y,
        double z,
        double yaw,
        int timeout_ms = 1000,
        const std::string& task_id = {},
        const std::string& request_id = {});
    [[nodiscard]] NavigationCommandReceipt cancelTask(
        const std::string& task_id,
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] NavigationCommandReceipt pauseTask(
        const std::string& task_id,
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] NavigationCommandReceipt resumeTask(
        const std::string& task_id,
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void stop(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void estop(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void clearEstop(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void resumeAutonomy(
        const std::string& reason,
        int timeout_ms = 1000,
        const std::string& request_id = {});

   private:
    friend class Client;
    explicit NavigationCommands(Client& owner) : owner_(owner) {}
    Client& owner_;
  };

  class ExplorationCommands {
   public:
    ExplorationCommandReceipt start(
        const std::string& exploration_run_id,
        const std::string& session_id,
        const std::string& reason = "operator_start",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    ExplorationCommandReceipt pause(
        const std::string& exploration_run_id,
        const std::string& session_id,
        const std::string& reason = "operator_pause",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    ExplorationCommandReceipt resume(
        const std::string& exploration_run_id,
        const std::string& session_id,
        const std::string& reason = "operator_resume",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    ExplorationCommandReceipt stop(
        const std::string& exploration_run_id,
        const std::string& session_id,
        const std::string& reason = "operator_stop",
        int timeout_ms = 1000,
        const std::string& request_id = {});

    ExplorationCommandReceipt setDirectedTarget(
        double x,
        double y,
        double ttl_s,
        const std::string& exploration_run_id,
        const std::string& session_id,
        const std::string& reason = "operator_directed_explore",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    ExplorationCommandReceipt clearDirectedTarget(
        const std::string& exploration_run_id,
        const std::string& session_id,
        const std::string& reason = "operator_clear_directed_explore",
        int timeout_ms = 1000,
        const std::string& request_id = {});

   private:
    friend class Client;
    explicit ExplorationCommands(Client& owner) : owner_(owner) {}
    Client& owner_;
  };

  class InspectionCommands {
   public:
    [[nodiscard]] InspectionTaskCommandReceipt startTask(
        const std::string& task_id,
        const std::string& route_id,
        std::uint64_t route_revision = 0,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] InspectionTaskCommandReceipt pauseTask(
        const std::string& task_id,
        const std::string& reason = "operator_pause",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] InspectionTaskCommandReceipt resumeTask(
        const std::string& task_id,
        const std::string& reason = "operator_resume",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] InspectionTaskCommandReceipt cancelTask(
        const std::string& task_id,
        const std::string& reason = "operator_cancel",
        int timeout_ms = 1000,
        const std::string& request_id = {});
   private:
    friend class Client;
    explicit InspectionCommands(Client& owner) : owner_(owner) {}
    Client& owner_;
  };

  class OperatorMotionCommands {
   public:
    [[nodiscard]] OperatorMotionCommandReceipt claimWithReceipt(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        std::uint32_t lease_ttl_ms,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void claim(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        std::uint32_t lease_ttl_ms,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void sample(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        double vx,
        double vy,
        double wz,
        bool deadman = true,
        std::uint32_t freshness_budget_ms = 350,
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] OperatorMotionCommandReceipt holdWithReceipt(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        const std::string& reason = "operator_hold",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void hold(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        const std::string& reason = "operator_hold",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    [[nodiscard]] OperatorMotionCommandReceipt releaseWithReceipt(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        const std::string& reason = "operator_release",
        int timeout_ms = 1000,
        const std::string& request_id = {});
    void release(
        const std::string& source_id,
        std::uint64_t source_epoch,
        std::uint64_t sequence,
        const std::string& reason = "operator_release",
        int timeout_ms = 1000,
        const std::string& request_id = {});

   private:
    friend class Client;
    explicit OperatorMotionCommands(Client& owner) : owner_(owner) {}
    Client& owner_;
  };

  explicit Client(int domain_id);
  ~Client();

  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  Client(Client&&) = delete;
  Client& operator=(Client&&) = delete;

  NavigationCommands& navigation() noexcept { return navigation_; }
  ExplorationCommands& exploration() noexcept { return exploration_; }
  InspectionCommands& inspection() noexcept { return inspection_; }
  OperatorMotionCommands& operatorMotion() noexcept { return operator_motion_; }
  [[nodiscard]] std::optional<NavigationStateSnapshot>
  latestNavigationState() const;
  [[nodiscard]] bool takeNavigationGoalStatus(
      NavigationGoalStatusSnapshot* status);
  [[nodiscard]] bool takeInspectionTaskEvent(
      InspectionTaskEventSnapshot* event);
  [[nodiscard]] bool takeExplorationRunEvent(
      ExplorationRunEventSnapshot* event);
  [[nodiscard]] std::optional<NavigationGoalStatusSnapshot>
  navigationGoalStatus(const std::string& request_id) const;
  [[nodiscard]] std::optional<NavigationGoalStatusSnapshot>
  navigationTaskStatus(const std::string& task_id) const;
  [[nodiscard]] bool takeGlobalPath(PathSnapshot* path);
  [[nodiscard]] bool takeLocalPath(PathSnapshot* path);
  [[nodiscard]] bool takeMapScene(MapSceneSnapshot* scene);
  [[nodiscard]] MapSceneHealthSnapshot mapSceneHealth() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  NavigationCommands navigation_;
  ExplorationCommands exploration_;
  InspectionCommands inspection_;
  OperatorMotionCommands operator_motion_;
};

}  // namespace lingtu::nav::commands
