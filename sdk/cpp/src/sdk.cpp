#include <lingtu/sdk.hpp>

#include "core.h"

#include <cmath>
#include <exception>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <sstream>

namespace lingtu::sdk {
namespace {

struct ReplyOwner {
  LingtuSdkReply reply;
  ~ReplyOwner() { lingtu_sdk_buffer_free(reply.data, reply.len); }
};

std::string take(LingtuSdkReply reply) {
  ReplyOwner owner{reply};
  std::string body;
  if (reply.len != 0) body.assign(reinterpret_cast<const char*>(reply.data), reply.len);
  if (reply.error != 0) throw Error(reply.status, std::move(body));
  return body;
}

std::string quote(const std::string& value) {
  static constexpr char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (unsigned char byte : value) {
    if (byte == '"' || byte == '\\') {
      result += '\\';
      result += static_cast<char>(byte);
    } else if (byte < 0x20) {
      result += "\\u00";
      result += hex[byte >> 4];
      result += hex[byte & 15];
    } else {
      result += static_cast<char>(byte);
    }
  }
  result += '"';
  return result;
}

std::string segment(const std::string& value) {
  static constexpr char hex[] = "0123456789ABCDEF";
  std::string result;
  for (unsigned char byte : value) {
    if ((byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z') ||
        (byte >= '0' && byte <= '9') || byte == '-' || byte == '_' || byte == '~') {
      result += static_cast<char>(byte);
    } else {
      result += '%';
      result += hex[byte >> 4];
      result += hex[byte & 15];
    }
  }
  return result;
}

std::string query_path(std::string path, const Query& query) {
  for (const auto& item : query) {
    path += path.find('?') == std::string::npos ? '?' : '&';
    path += segment(item.first) + "=" + segment(item.second);
  }
  return path;
}

std::string object(std::initializer_list<std::pair<std::string, std::string>> fields) {
  std::string result = "{";
  for (const auto& field : fields) {
    if (result.size() > 1) result += ',';
    result += quote(field.first) + ":" + field.second;
  }
  return result + "}";
}

std::string merge(const std::string& options,
                  std::initializer_list<std::pair<std::string, std::string>> fields) {
  const std::string required = object(fields);
  return take(lingtu_sdk_merge_json(options.c_str(), required.c_str()));
}

std::string number(double value) {
  if (!std::isfinite(value)) throw std::invalid_argument("JSON numbers must be finite");
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return output.str();
}

uint64_t milliseconds(std::chrono::milliseconds value) {
  if (value.count() < 0) throw std::invalid_argument("duration must be nonnegative");
  return static_cast<uint64_t>(value.count());
}

std::string map_path(const std::string& name) {
  return "/api/v1/maps/" + segment(name);
}

std::string operation_path(const std::string& id) {
  return "/api/v1/maps/operations/" + segment(id);
}

std::string inspection_path(const std::string& id) {
  return "/api/v1/inspection/tasks/" + segment(id);
}

std::string evidence_path(const std::string& id) {
  return "/api/v1/inspection/evidence/" + segment(id);
}

std::string recording_path(const std::string& id) {
  return "/api/v1/recordings/" + segment(id);
}

std::string text(const uint8_t* data, size_t size) {
  return size == 0 ? std::string() : std::string(reinterpret_cast<const char*>(data), size);
}

struct Receiver {
  const std::function<bool(const Event&)>* callback;
  std::exception_ptr failure;
};

int32_t receive_event(const LingtuSdkEvent* event, void* context) noexcept {
  auto& receiver = *static_cast<Receiver*>(context);
  try {
    return (*receiver.callback)({text(event->event, event->event_len), text(event->id, event->id_len),
                                text(event->data, event->data_len)}) ? 1 : 0;
  } catch (...) {
    receiver.failure = std::current_exception();
    return 0;
  }
}

}  // namespace

Error::Error(int status, std::string body)
    : std::runtime_error((status != 0 ? "Gateway HTTP " + std::to_string(status) : "LingTu SDK error") +
                         ": " + body), status_(status), body_(std::move(body)) {}

Client::Client(const std::string& base_url, const std::string& api_key, std::chrono::milliseconds timeout)
    : Client(base_url, api_key, timeout, "") {}

Client::Client(const std::string& base_url, const std::string& api_key,
                 std::chrono::milliseconds timeout, const std::string& media_token) {
  take(lingtu_sdk_client_create_with_media_token(
      base_url.c_str(), api_key.empty() ? nullptr : api_key.c_str(), milliseconds(timeout),
      media_token.empty() ? nullptr : media_token.c_str(), &handle_));
}

Client::~Client() { lingtu_sdk_client_free(handle_); }

Client::Client(Client&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}

Client& Client::operator=(Client&& other) noexcept {
  if (this != &other) {
    lingtu_sdk_client_free(handle_);
    handle_ = std::exchange(other.handle_, nullptr);
  }
  return *this;
}

Navigation Client::navigation() const { return Navigation(*this); }
Map Client::map() const { return Map(*this); }
Tasks Client::tasks() const { return Tasks(*this); }
Observations Client::observations() const { return Observations(*this); }
System Client::system() const { return System(*this); }

Feed Client::watch_cloud() const {
  void* handle = nullptr;
  take(lingtu_sdk_websocket_open(handle_, "cloud", nullptr, &handle));
  return Feed(handle);
}

Feed Client::watch_scan() const {
  void* handle = nullptr;
  take(lingtu_sdk_websocket_open(handle_, "scan", nullptr, &handle));
  return Feed(handle);
}

Feed Client::watch_camera(const std::string& viewer_token) const {
  void* handle = nullptr;
  take(lingtu_sdk_websocket_open(handle_, "camera", viewer_token.empty() ? nullptr : viewer_token.c_str(), &handle));
  return Feed(handle);
}

Teleop Client::teleop(const std::string& client_id) const {
  void* handle = nullptr;
  take(lingtu_sdk_teleop_open(handle_, client_id.c_str(), &handle));
  return Teleop(handle);
}

Feed::~Feed() { lingtu_sdk_websocket_free(handle_); }
Feed::Feed(Feed&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}

Feed& Feed::operator=(Feed&& other) noexcept {
  if (this != &other) {
    lingtu_sdk_websocket_free(handle_);
    handle_ = std::exchange(other.handle_, nullptr);
  }
  return *this;
}

std::optional<Frame> Feed::receive() {
  int32_t kind = 0;
  std::string data = take(lingtu_sdk_websocket_receive(handle_, &kind));
  if (kind == 0) return std::nullopt;
  return Frame{kind == 2, std::move(data)};
}

void Feed::close() { take(lingtu_sdk_websocket_close(handle_)); }

Teleop::~Teleop() { lingtu_sdk_teleop_free(handle_); }
Teleop::Teleop(Teleop&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}

Teleop& Teleop::operator=(Teleop&& other) noexcept {
  if (this != &other) {
    lingtu_sdk_teleop_free(handle_);
    handle_ = std::exchange(other.handle_, nullptr);
  }
  return *this;
}

std::string Teleop::move(double vx, double vy, double yaw, bool manual_mode, const std::string& request_id) {
  return take(lingtu_sdk_teleop_move(handle_, vx, vy, yaw, manual_mode ? 1 : 0,
                                    request_id.empty() ? nullptr : request_id.c_str()));
}

std::string Teleop::hold(const std::string& request_id) {
  return take(lingtu_sdk_teleop_hold(handle_, request_id.empty() ? nullptr : request_id.c_str()));
}

void Teleop::close() { take(lingtu_sdk_teleop_close(handle_)); }

std::string Client::request(const std::string& method, const std::string& path,
                             const std::string& body, const Query& query) const {
  const std::string target = query_path(path, query);
  return take(lingtu_sdk_request(handle_, method.c_str(), target.c_str(), body.empty() ? nullptr : body.c_str()));
}

std::string Client::command(const std::string& method, const std::string& path, const std::string& body) const {
  return take(lingtu_sdk_command(handle_, method.c_str(), path.c_str(), body.c_str()));
}

std::string Client::bytes(const std::string& path, const Query& query) const {
  const std::string target = query_path(path, query);
  return take(lingtu_sdk_bytes(handle_, target.c_str()));
}

void Client::download(const std::string& path, const std::filesystem::path& target) const {
  const std::string filename = target.u8string();
  take(lingtu_sdk_download(handle_, path.c_str(), filename.c_str()));
}

void Client::events(const std::function<bool(const Event&)>& receive, const Query& query,
                     const std::string& last_event_id) const {
  std::string options = "{";
  for (const auto& item : query) {
    if (options.size() > 1) options += ',';
    options += quote(item.first) + ":" + quote(item.second);
  }
  options += '}';
  Receiver receiver{&receive, {}};
  auto reply = lingtu_sdk_events(handle_, options.c_str(),
                                 last_event_id.empty() ? nullptr : last_event_id.c_str(), receive_event, &receiver);
  take(reply);
  if (receiver.failure) std::rethrow_exception(receiver.failure);
}

std::string Navigation::go(double x, double y, double z, const std::string& options) const {
  return take(lingtu_sdk_goal(client_.handle_, x, y, z, options.c_str(), 0));
}

std::string Navigation::go_to_click(double x, double y, double z, const std::string& options) const {
  return take(lingtu_sdk_goal(client_.handle_, x, y, z, options.c_str(), 1));
}

std::string Navigation::select_planner(const std::string& name, const std::string& options) const {
  return client_.command("POST", "/api/v1/navigation/planner", merge(options, {{"name", quote(name)}}));
}

std::string Navigation::control(const std::string& action, const std::string& task_id,
                                const std::string& options) const {
  return client_.command("POST", "/api/v1/navigation/tasks/" + segment(task_id) + "/" + action,
                         merge(options, {{"task_id", quote(task_id)}}));
}

std::string Navigation::cancel(const std::string& task_id, const std::string& options) const {
  return task_id.empty() ? client_.command("POST", "/api/v1/navigation/cancel", options)
                         : control("cancel", task_id, options);
}

std::string Navigation::pause(const std::string& task_id, const std::string& options) const {
  return control("pause", task_id, options);
}

std::string Navigation::resume(const std::string& task_id, const std::string& options) const {
  return control("resume", task_id, options);
}

std::string Navigation::set_speed_limit(const std::string& task_id, double speed, const std::string& options) const {
  return control("speed-limit", task_id, merge(options, {{"max_speed_mps", number(speed)}}));
}

std::string Navigation::status() const { return client_.request("GET", "/api/v1/navigation/status"); }
std::string Navigation::task(const std::string& id) const {
  return client_.request("GET", "/api/v1/navigation/tasks/" + segment(id));
}
std::string Navigation::goal_status(const std::string& id) const {
  return client_.request("GET", "/api/v1/navigation/goals/" + segment(id));
}
std::string Navigation::path() const { return client_.request("GET", "/api/v1/path"); }
std::string Navigation::start_exploration() const { return client_.request("POST", "/api/v1/explore/start", "{}"); }
std::string Navigation::stop_exploration() const { return client_.request("POST", "/api/v1/explore/stop", "{}"); }
std::string Navigation::exploration_status() const { return client_.request("GET", "/api/v1/explore/status"); }
std::string Navigation::localization_status() const { return client_.request("GET", "/api/v1/localization/status"); }
std::string Navigation::position() const { return take(lingtu_sdk_position(client_.handle_)); }

std::string Navigation::relocalize(const std::string& name, const std::string& pose, const std::string& options) const {
  return client_.command("POST", "/api/v1/localization/relocalizations",
      merge(options, {{"map_name", quote(name)}, {"mode", quote("seeded")}, {"initial_pose", pose}}));
}

std::string Navigation::global_relocalize(const std::string& name, const std::string& options) const {
  return client_.command("POST", "/api/v1/localization/relocalizations",
                         merge(options, {{"map_name", quote(name)}, {"mode", quote("global")}}));
}

std::string Navigation::start_map_tracking(const std::string& name, const std::string& options) const {
  return client_.command("POST", "/api/v1/localization/map-tracking", merge(options, {{"map_name", quote(name)}}));
}

std::string Navigation::wait_until_arrived(const std::string& task_id, std::chrono::milliseconds timeout,
                                           std::chrono::milliseconds poll_interval) const {
  return take(lingtu_sdk_wait_navigation(client_.handle_, task_id.c_str(), milliseconds(timeout), milliseconds(poll_interval)));
}

std::string Map::list() const { return client_.request("GET", "/api/v1/slam/maps"); }
std::string Map::save(const std::string& options) const { return client_.command("POST", "/api/v1/map/save", options); }
std::string Map::save_and_wait(const std::string& options, std::chrono::milliseconds timeout,
                                std::chrono::milliseconds poll_interval, bool wait_for_source) const {
  return take(lingtu_sdk_save_and_wait(client_.handle_, options.c_str(), milliseconds(timeout),
                                      milliseconds(poll_interval), wait_for_source ? 1 : 0));
}
std::string Map::operations(int limit) const {
  return client_.request("GET", "/api/v1/maps/operations", "", {{"limit", std::to_string(limit)}});
}
std::string Map::operation(const std::string& id) const { return client_.request("GET", operation_path(id)); }
std::string Map::cancel_operation(const std::string& id) const {
  return client_.request("POST", operation_path(id) + "/cancel", "{}");
}
std::string Map::retry_operation(const std::string& id) const {
  return client_.request("POST", operation_path(id) + "/retry", "{}");
}
std::string Map::process_operation(const std::string& id) const {
  return client_.request("POST", operation_path(id) + "/process", "{}");
}
std::string Map::reoptimize_operation(const std::string& id, const std::string& options) const {
  return client_.command("POST", operation_path(id) + "/reoptimize", options);
}
void Map::download_pcd(const std::string& name, const std::filesystem::path& target) const {
  client_.download(map_path(name) + "/pcd", target);
}
void Map::download_octomap(const std::string& name, const std::filesystem::path& target) const {
  client_.download(map_path(name) + "/octomap", target);
}
void Map::download_source(const std::string& id, const std::filesystem::path& target) const {
  client_.download(operation_path(id) + "/source/pcd", target);
}
std::string Map::rename(const std::string& old_name, const std::string& new_name) const {
  return client_.request("POST", "/api/v1/map/rename", object({{"old_name", quote(old_name)}, {"new_name", quote(new_name)}}));
}
std::string Map::delete_map(const std::string& name) const { return client_.request("DELETE", map_path(name)); }
std::string Map::import_pcd(const std::string& name, const std::string& source, const std::string& options) const {
  return client_.request("POST", "/api/v1/maps/import_pcd", merge(options, {{"name", quote(name)}, {"source_path", quote(source)}}));
}
std::string Map::reset_cloud() const { return client_.request("POST", "/api/v1/map_cloud/reset", "{}"); }
std::string Map::points() const { return client_.request("GET", "/api/v1/map/points"); }
std::string Map::saved_points(const std::string& name) const { return client_.request("GET", map_path(name) + "/points"); }
std::string Map::surface(const std::string& name, const Query& query) const {
  return client_.request("GET", map_path(name) + "/surface", "", query);
}
std::string Map::no_go(const std::string& name) const { return client_.request("GET", map_path(name) + "/no_go"); }
std::string Map::set_no_go(const std::string& name, const std::string& region, long long epoch) const {
  return client_.request("PUT", map_path(name) + "/no_go",
                         object({{"region", region}, {"expected_content_epoch", std::to_string(epoch)}}));
}
std::string Map::delete_no_go(const std::string& name, const std::string& id, long long epoch) const {
  return client_.request("DELETE", map_path(name) + "/no_go/" + segment(id), "",
                         {{"expected_content_epoch", std::to_string(epoch)}});
}
std::string Map::mutate(const std::string& name, const std::string& action, const std::string& options) const {
  return client_.request("POST", map_path(name) + "/" + action, options);
}
std::string Map::preview_edits(const std::string& name, const std::string& options) const { return mutate(name, "edit_preview", options); }
std::string Map::save_as(const std::string& name, const std::string& options) const { return mutate(name, "save_as", options); }
std::string Map::crop(const std::string& name, const std::string& options) const { return mutate(name, "crop", options); }
std::string Map::mark_zone(const std::string& name, const std::string& options) const { return mutate(name, "mark_zone", options); }
std::string Map::build_occupancy(const std::string& name) const { return mutate(name, "build_occupancy"); }
std::string Map::build_octomap(const std::string& name, const std::string& options) const { return mutate(name, "build_octomap", options); }
std::string Map::validate_plan(const std::string& name, const std::string& options) const { return mutate(name, "validate_plan", options); }
std::string Map::edit_voxels(const std::string& name, const std::string& options) const { return mutate(name, "voxels/edit", options); }
std::string Map::voxel_edits(const std::string& name) const { return client_.request("GET", map_path(name) + "/voxels/edits"); }
std::string Map::locations() const { return client_.request("GET", "/api/v1/locations"); }
std::string Map::tag_location(const std::string& name, const std::string& options) const {
  return client_.request("POST", "/api/v1/locations", merge(options, {{"name", quote(name)}}));
}
std::string Map::delete_location(const std::string& name) const {
  return client_.request("DELETE", "/api/v1/locations/" + segment(name));
}

std::string Tasks::send_instruction(const std::string& text_value, const std::string& options) const {
  return client_.command("POST", "/api/v1/instruction", merge(options, {{"text", quote(text_value)}}));
}
std::string Tasks::get_agent_run(const std::string& id) const { return client_.request("GET", "/api/v1/agent/runs/" + segment(id)); }
std::string Tasks::cancel_agent_run(const std::string& id, const std::string& options) const {
  return client_.command("POST", "/api/v1/agent/runs/" + segment(id) + "/cancel", options);
}
std::string Tasks::start_follow(const std::string& options) const {
  return client_.command("POST", "/api/v1/visual_servo", merge(options, {{"mode", quote("follow")}}));
}
std::string Tasks::get_follow_task(const std::string& id, const std::string& session) const {
  return client_.request("GET", "/api/v1/follow/" + segment(id), "", {{"product_session_id", session}});
}
std::string Tasks::cancel_follow_task(const std::string& id, const std::string& session, const std::string& options) const {
  return client_.command("POST", "/api/v1/follow/" + segment(id) + "/cancel",
                         merge(options, {{"product_session_id", quote(session)}}));
}
std::string Tasks::inspection_routes(const std::string& map_id) const {
  return client_.request("GET", "/api/v1/inspection/routes", "", map_id.empty() ? Query{} : Query{{"map_id", map_id}});
}
std::string Tasks::inspection_route(const std::string& id, const std::string& map_id) const {
  return client_.request("GET", "/api/v1/inspection/routes/" + segment(id), "",
                         map_id.empty() ? Query{} : Query{{"map_id", map_id}});
}
std::string Tasks::save_inspection_route(const std::string& route) const {
  return client_.request("POST", "/api/v1/inspection/routes", route);
}
std::string Tasks::delete_inspection_route(const std::string& id, const std::string& map_id) const {
  return client_.request("DELETE", "/api/v1/inspection/routes/" + segment(id), "",
                         map_id.empty() ? Query{} : Query{{"map_id", map_id}});
}
std::string Tasks::inspection_tasks(const Query& query) const {
  return client_.request("GET", "/api/v1/inspection/tasks", "", query);
}
std::string Tasks::list_inspection_evidence(int limit) const {
  return client_.request("GET", "/api/v1/inspection/evidence", "", {{"limit", std::to_string(limit)}});
}
std::string Tasks::inspection_evidence(const std::string& id) const {
  return client_.request("GET", evidence_path(id));
}
std::string Tasks::review_inspection_evidence(const std::string& id, const std::string& verdict,
                                              const std::string& note) const {
  return client_.request("POST", evidence_path(id) + "/review", object({{"verdict", quote(verdict)}, {"note", quote(note)}}));
}
void Tasks::download_inspection_artifact(const std::string& id, const std::string& kind,
                                          const std::filesystem::path& target) const {
  client_.download(evidence_path(id) + "/artifacts/" + segment(kind), target);
}
void Tasks::download_inspection_measurement(const std::string& id, const std::string& sensor,
                                             const std::filesystem::path& target) const {
  client_.download(evidence_path(id) + "/measurements/" + segment(sensor), target);
}
std::string Tasks::start_inspection(const std::string& id, const std::string& options) const {
  return client_.command("POST", "/api/v1/inspection/tasks", merge(options, {{"route_id", quote(id)}}));
}
std::string Tasks::inspection_task(const std::string& id) const { return client_.request("GET", inspection_path(id)); }
std::string Tasks::control(const std::string& action, const std::string& id, const std::string& options) const {
  return client_.command("POST", inspection_path(id) + "/" + action, options);
}
std::string Tasks::pause_inspection(const std::string& id, const std::string& options) const { return control("pause", id, options); }
std::string Tasks::resume_inspection(const std::string& id, const std::string& options) const { return control("resume", id, options); }
std::string Tasks::cancel_inspection(const std::string& id, const std::string& options) const { return control("cancel", id, options); }
std::string Tasks::inspection_report(const std::string& id) const { return client_.request("GET", inspection_path(id) + "/report"); }

std::string Observations::scene() const { return client_.request("GET", "/api/v1/scene_graph"); }
std::string Observations::camera_snapshot() const { return client_.bytes("/api/v1/camera/snapshot"); }
std::string Observations::camera_status() const { return client_.request("GET", "/api/v1/camera/status"); }
std::string Observations::media_status() const { return client_.request("GET", "/api/v1/webrtc/status"); }
std::string Observations::camera_diagnostics() const { return client_.request("GET", "/api/v1/camera/diagnostics"); }
std::string Observations::temporal_memory(const Query& query) const { return client_.request("GET", "/api/v1/memory/temporal", "", query); }
std::string Observations::search_memory(const std::string&) const {
  throw Error(0, R"({"kind":"unsupported","message":"Text search needs the observation store's encoder. Use search_memory_embedding."})");
}
std::string Observations::search_memory_embedding(const std::string& embedding, const std::string& options) const {
  return client_.request("POST", "/api/v1/memory/temporal/semantic", merge(options, {{"embedding", embedding}}));
}
std::string Observations::start_recording(const std::string& options) const { return client_.request("POST", "/api/v1/recordings/start", options); }
std::string Observations::stop_recording() const { return client_.request("POST", "/api/v1/recordings/stop", "{}"); }
std::string Observations::stop_recording(const std::string& session_id) const {
  return client_.request("POST", "/api/v1/recordings/stop", object({{"expected_session_id", quote(session_id)}}));
}
std::string Observations::recording_status() const { return client_.request("GET", "/api/v1/recordings/status"); }
std::string Observations::recordings(int limit) const {
  return client_.request("GET", "/api/v1/recordings", "", {{"limit", std::to_string(limit)}});
}
std::string Observations::recording(const std::string& id) const { return client_.request("GET", recording_path(id)); }
std::string Observations::recording_catalog() const { return client_.request("GET", "/api/v1/recordings/catalog"); }
std::string Observations::recording_streams() const { return client_.request("GET", "/api/v1/recordings/streams"); }
void Observations::download_recording_file(const std::string& id, const std::string& artifact,
                                            const std::filesystem::path& target) const {
  std::string path;
  std::istringstream parts(artifact);
  std::string part;
  while (std::getline(parts, part, '/')) {
    if (!path.empty()) path += '/';
    path += segment(part);
  }
  client_.download(recording_path(id) + "/files/" + path, target);
}
void Observations::download_recording_archive(const std::string& id, const std::filesystem::path& target) const {
  client_.download(recording_path(id) + "/archive", target);
}
std::string Observations::recording_summary(const std::string& id) const {
  return client_.request("GET", recording_path(id) + "/summary");
}
std::string Observations::recording_series(const std::string& id, const std::string& topic, const Query& query) const {
  Query options = query;
  options.emplace_back("topic", topic);
  return client_.request("GET", recording_path(id) + "/series", "", options);
}
std::string Observations::recording_scene_info(const std::string& id) const {
  return client_.request("GET", recording_path(id) + "/scene-info");
}
std::string Observations::recording_scene(const std::string& id, const Query& query) const {
  return client_.request("GET", recording_path(id) + "/scene", "", query);
}
std::string Observations::recording_camera_frame(const std::string& id, double time_s, int index, const Query& query) const {
  Query options = query;
  options.emplace_back("t", number(time_s));
  options.emplace_back("segment", std::to_string(index));
  return client_.bytes(recording_path(id) + "/camera-frame", options);
}
std::string Observations::validate_recording(const std::string& id) const {
  return client_.request("POST", recording_path(id) + "/validate");
}
std::string Observations::delete_recording(const std::string& id) const { return client_.request("DELETE", recording_path(id)); }

std::string System::state() const { return client_.request("GET", "/api/v1/state"); }
std::string System::health() const { return client_.request("GET", "/api/v1/health"); }
std::string System::session() const { return client_.request("GET", "/api/v1/session"); }
std::string System::product_status() const { return client_.request("GET", "/api/v1/product-control"); }
std::string System::product_operation(const std::string& id) const {
  return client_.request("GET", "/api/v1/product-control/operations/" + segment(id));
}
std::string System::stop_product(const std::string& session, const std::string& options) const {
  return client_.command("POST", "/api/v1/product-control/stop", merge(options, {{"expected_product_session_id", quote(session)}}));
}
std::string System::switch_product(const std::string& product, const std::string& session, const std::string& options) const {
  return client_.command("POST", "/api/v1/product-control/switch",
                         merge(options, {{"product", quote(product)}, {"expected_product_session_id", quote(session)}}));
}
std::string System::set_mode(const std::string& mode) const { return client_.command("POST", "/api/v1/mode", object({{"mode", quote(mode)}})); }
std::string System::capabilities() const { return client_.request("GET", "/api/v1/app/capabilities"); }
std::string System::bootstrap() const { return client_.request("GET", "/api/v1/app/bootstrap"); }
std::string System::readiness() const { return client_.request("GET", "/api/v1/readiness"); }
std::string System::field_check() const { return client_.request("POST", "/api/v1/diagnostics/field-check", "{}"); }
std::string System::runtime_contract() const { return client_.request("GET", "/api/v1/diagnostics/runtime-contract"); }
std::string System::slam_status() const { return client_.request("GET", "/api/v1/slam/status"); }
std::string System::emergency_stop(const std::string& options) const { return client_.command("POST", "/api/v1/stop", options); }
std::string System::stop_motion(const std::string& options) const { return client_.command("POST", "/api/v1/motion/stop", options); }
std::string System::resume_motion(const std::string& options) const { return client_.command("POST", "/api/v1/navigation/resume", options); }
std::string System::clear_emergency_stop(const std::string& options) const { return client_.command("POST", "/api/v1/estop/reset", options); }
std::string System::stand_up(const std::string& options) const { return client_.command("POST", "/api/v1/motion/stand", options); }
std::string System::sit_down(const std::string& options) const { return client_.command("POST", "/api/v1/motion/sit", options); }
std::string System::enable_motors(const std::string& options) const { return client_.command("POST", "/api/v1/motion/motors/enable", options); }
std::string System::disable_motors(const std::string& options) const { return client_.command("POST", "/api/v1/motion/motors/disable", options); }
std::string System::switch_profile(const std::string& name, const std::string& options) const {
  return client_.command("POST", "/api/v1/motion/profile", merge(options, {{"name", quote(name)}}));
}
std::string System::acquire_lease(const std::string& id, double ttl) const {
  return client_.command("POST", "/api/v1/lease", object({{"action", quote("acquire")}, {"client_id", quote(id)}, {"ttl", number(ttl)}}));
}
std::string System::renew_lease(const std::string& id, double ttl) const {
  return client_.command("POST", "/api/v1/lease", object({{"action", quote("renew")}, {"client_id", quote(id)}, {"ttl", number(ttl)}}));
}
std::string System::release_lease(const std::string& id) const {
  return client_.command("POST", "/api/v1/lease", object({{"action", quote("release")}, {"client_id", quote(id)}}));
}

}  // namespace lingtu::sdk
