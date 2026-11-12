#pragma once

#include <chrono>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::sdk {

using Query = std::vector<std::pair<std::string, std::string>>;

struct Event {
  std::string event;
  std::string id;
  std::string data;
};

class Error : public std::runtime_error {
 public:
  Error(int status, std::string body);
  int status() const noexcept { return status_; }
  const std::string& body() const noexcept { return body_; }

 private:
  int status_;
  std::string body_;
};

class Navigation;
class Map;
class Tasks;
class Observations;
class System;
class Feed;
class Teleop;

struct Frame {
  bool binary;
  std::string data;
};

class Client {
 public:
  explicit Client(const std::string& base_url = "http://127.0.0.1:5050",
                  const std::string& api_key = "",
                  std::chrono::milliseconds timeout = std::chrono::seconds(10));
  Client(const std::string& base_url, const std::string& api_key,
          std::chrono::milliseconds timeout, const std::string& media_token);
  ~Client();
  Client(const Client&) = delete;
  Client& operator=(const Client&) = delete;
  Client(Client&& other) noexcept;
  Client& operator=(Client&& other) noexcept;

  Navigation navigation() const;
  Map map() const;
  Tasks tasks() const;
  Observations observations() const;
  System system() const;
  Feed watch_cloud() const;
  Feed watch_scan() const;
  Feed watch_camera(const std::string& viewer_token = "") const;
  Teleop teleop(const std::string& client_id = "sdk") const;

  // JSON request/response bodies retain the Gateway's complete owner evidence.
  std::string request(const std::string& method, const std::string& path,
                      const std::string& body = "", const Query& query = {}) const;
  std::string command(const std::string& method, const std::string& path,
                      const std::string& body = "{}") const;
  std::string bytes(const std::string& path, const Query& query = {}) const;
  void download(const std::string& path, const std::filesystem::path& target) const;
  void events(const std::function<bool(const Event&)>& receive,
              const Query& query = {}, const std::string& last_event_id = "") const;

 private:
  friend class Navigation;
  friend class Map;
  void* handle_ = nullptr;
};

class Feed {
 public:
  ~Feed();
  Feed(const Feed&) = delete;
  Feed& operator=(const Feed&) = delete;
  Feed(Feed&& other) noexcept;
  Feed& operator=(Feed&& other) noexcept;
  std::optional<Frame> receive();
  void close();

 private:
  friend class Client;
  explicit Feed(void* handle) : handle_(handle) {}
  void* handle_;
};

class Teleop {
 public:
  ~Teleop();
  Teleop(const Teleop&) = delete;
  Teleop& operator=(const Teleop&) = delete;
  Teleop(Teleop&& other) noexcept;
  Teleop& operator=(Teleop&& other) noexcept;
  std::string move(double vx_mps, double vy_mps, double yaw_rps,
                   bool manual_mode = false, const std::string& request_id = "");
  std::string hold(const std::string& request_id = "");
  void close();

 private:
  friend class Client;
  explicit Teleop(void* handle) : handle_(handle) {}
  void* handle_;
};

class Navigation {
 public:
  explicit Navigation(const Client& client) : client_(client) {}
  std::string go(double x, double y, double z = 0, const std::string& options = "{}") const;
  std::string go_to_click(double x, double y, double z = 0, const std::string& options = "{}") const;
  std::string select_planner(const std::string& name, const std::string& options = "{}") const;
  std::string cancel(const std::string& task_id = "", const std::string& options = "{}") const;
  std::string pause(const std::string& task_id, const std::string& options = "{}") const;
  std::string resume(const std::string& task_id, const std::string& options = "{}") const;
  std::string set_speed_limit(const std::string& task_id, double speed,
                              const std::string& options = "{}") const;
  std::string status() const;
  std::string task(const std::string& task_id) const;
  std::string goal_status(const std::string& request_id) const;
  std::string path() const;
  std::string start_exploration() const;
  std::string stop_exploration() const;
  std::string exploration_status() const;
  std::string localization_status() const;
  std::string position() const;
  std::string relocalize(const std::string& map_name, const std::string& initial_pose,
                         const std::string& options = "{}") const;
  std::string global_relocalize(const std::string& map_name, const std::string& options = "{}") const;
  std::string start_map_tracking(const std::string& map_name, const std::string& options = "{}") const;
  std::string wait_until_arrived(const std::string& task_id,
      std::chrono::milliseconds timeout = std::chrono::seconds(120),
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(500)) const;

 private:
  std::string control(const std::string& action, const std::string& task_id,
                       const std::string& options) const;
  const Client& client_;
};

class Map {
 public:
  explicit Map(const Client& client) : client_(client) {}
  std::string list() const;
  std::string save(const std::string& options = "{}") const;
  std::string save_and_wait(const std::string& options = "{}",
      std::chrono::milliseconds timeout = std::chrono::seconds(300),
      std::chrono::milliseconds poll_interval = std::chrono::seconds(1),
      bool wait_for_source = false) const;
  std::string operations(int limit = 100) const;
  std::string operation(const std::string& operation_id) const;
  std::string cancel_operation(const std::string& operation_id) const;
  std::string retry_operation(const std::string& operation_id) const;
  std::string process_operation(const std::string& operation_id) const;
  std::string reoptimize_operation(const std::string& operation_id, const std::string& options = "{}") const;
  void download_pcd(const std::string& name, const std::filesystem::path& target) const;
  void download_octomap(const std::string& name, const std::filesystem::path& target) const;
  void download_source(const std::string& operation_id, const std::filesystem::path& target) const;
  std::string rename(const std::string& old_name, const std::string& new_name) const;
  std::string delete_map(const std::string& name) const;
  std::string import_pcd(const std::string& name, const std::string& source_path,
                        const std::string& options = "{}") const;
  std::string reset_cloud() const;
  std::string points() const;
  std::string saved_points(const std::string& name) const;
  std::string surface(const std::string& name, const Query& query = {}) const;
  std::string no_go(const std::string& name) const;
  std::string set_no_go(const std::string& name, const std::string& region,
                        long long expected_content_epoch) const;
  std::string delete_no_go(const std::string& name, const std::string& region_id,
                           long long expected_content_epoch) const;
  std::string preview_edits(const std::string& name, const std::string& options) const;
  std::string save_as(const std::string& name, const std::string& options) const;
  std::string crop(const std::string& name, const std::string& options) const;
  std::string mark_zone(const std::string& name, const std::string& options) const;
  std::string build_occupancy(const std::string& name) const;
  std::string build_octomap(const std::string& name, const std::string& options = "{}") const;
  std::string validate_plan(const std::string& name, const std::string& options) const;
  std::string edit_voxels(const std::string& name, const std::string& options) const;
  std::string voxel_edits(const std::string& name) const;
  std::string locations() const;
  std::string tag_location(const std::string& name, const std::string& options = "{}") const;
  std::string delete_location(const std::string& name) const;

 private:
  std::string mutate(const std::string& name, const std::string& action,
                      const std::string& options = "{}") const;
  const Client& client_;
};

class Tasks {
 public:
  explicit Tasks(const Client& client) : client_(client) {}
  std::string send_instruction(const std::string& text, const std::string& options = "{}") const;
  std::string get_agent_run(const std::string& run_id) const;
  std::string cancel_agent_run(const std::string& run_id, const std::string& options = "{}") const;
  std::string start_follow(const std::string& options) const;
  std::string get_follow_task(const std::string& task_id, const std::string& product_session_id) const;
  std::string cancel_follow_task(const std::string& task_id, const std::string& product_session_id,
                                  const std::string& options = "{}") const;
  std::string inspection_routes(const std::string& map_id = "") const;
  std::string inspection_route(const std::string& route_id, const std::string& map_id = "") const;
  std::string save_inspection_route(const std::string& route) const;
  std::string delete_inspection_route(const std::string& route_id, const std::string& map_id = "") const;
  std::string inspection_tasks(const Query& query = {}) const;
  std::string list_inspection_evidence(int limit = 20) const;
  std::string inspection_evidence(const std::string& evidence_id) const;
  std::string review_inspection_evidence(const std::string& evidence_id, const std::string& verdict,
                                         const std::string& note = "") const;
  void download_inspection_artifact(const std::string& evidence_id, const std::string& kind,
                                     const std::filesystem::path& target) const;
  void download_inspection_measurement(const std::string& evidence_id, const std::string& sensor_id,
                                        const std::filesystem::path& target) const;
  std::string start_inspection(const std::string& route_id, const std::string& options = "{}") const;
  std::string inspection_task(const std::string& task_id) const;
  std::string pause_inspection(const std::string& task_id, const std::string& options = "{}") const;
  std::string resume_inspection(const std::string& task_id, const std::string& options = "{}") const;
  std::string cancel_inspection(const std::string& task_id, const std::string& options = "{}") const;
  std::string inspection_report(const std::string& task_id) const;

 private:
  std::string control(const std::string& action, const std::string& task_id,
                       const std::string& options) const;
  const Client& client_;
};

class Observations {
 public:
  explicit Observations(const Client& client) : client_(client) {}
  std::string scene() const;
  std::string camera_snapshot() const;
  std::string camera_status() const;
  std::string media_status() const;
  std::string camera_diagnostics() const;
  std::string temporal_memory(const Query& query = {}) const;
  std::string search_memory(const std::string& text) const;
  std::string search_memory_embedding(const std::string& embedding,
                                      const std::string& options = "{}") const;
  std::string start_recording(const std::string& options = "{}") const;
  std::string stop_recording() const;
  std::string stop_recording(const std::string& expected_session_id) const;
  std::string recording_status() const;
  std::string recordings(int limit = 100) const;
  std::string recording(const std::string& session_id) const;
  std::string recording_catalog() const;
  std::string recording_streams() const;
  void download_recording_file(const std::string& session_id, const std::string& artifact_path,
                                const std::filesystem::path& target) const;
  void download_recording_archive(const std::string& session_id, const std::filesystem::path& target) const;
  std::string recording_summary(const std::string& session_id) const;
  std::string recording_series(const std::string& session_id, const std::string& topic, const Query& query = {}) const;
  std::string recording_scene_info(const std::string& session_id) const;
  std::string recording_scene(const std::string& session_id, const Query& query = {}) const;
  std::string recording_camera_frame(const std::string& session_id, double time_s, int segment,
                                      const Query& query = {}) const;
  std::string validate_recording(const std::string& session_id) const;
  std::string delete_recording(const std::string& session_id) const;

 private:
  const Client& client_;
};

class System {
 public:
  explicit System(const Client& client) : client_(client) {}
  std::string state() const;
  std::string health() const;
  std::string session() const;
  std::string product_status() const;
  std::string product_operation(const std::string& request_id) const;
  std::string stop_product(const std::string& expected_product_session_id, const std::string& options = "{}") const;
  std::string switch_product(const std::string& product, const std::string& expected_product_session_id,
                              const std::string& options = "{}") const;
  std::string set_mode(const std::string& mode) const;
  std::string capabilities() const;
  std::string bootstrap() const;
  std::string readiness() const;
  std::string field_check() const;
  std::string runtime_contract() const;
  std::string slam_status() const;
  std::string emergency_stop(const std::string& options = "{}") const;
  std::string stop_motion(const std::string& options = "{}") const;
  std::string resume_motion(const std::string& options = "{}") const;
  std::string clear_emergency_stop(const std::string& options = "{}") const;
  std::string stand_up(const std::string& options = "{}") const;
  std::string sit_down(const std::string& options = "{}") const;
  std::string enable_motors(const std::string& options = "{}") const;
  std::string disable_motors(const std::string& options = "{}") const;
  std::string switch_profile(const std::string& name, const std::string& options = "{}") const;
  std::string acquire_lease(const std::string& client_id, double ttl = 30) const;
  std::string renew_lease(const std::string& client_id, double ttl = 30) const;
  std::string release_lease(const std::string& client_id) const;

 private:
  const Client& client_;
};

}  // namespace lingtu::sdk
