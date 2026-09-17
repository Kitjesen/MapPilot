#include "runtime/semantic/view_query.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "semantic_views.hpp"

namespace lingtu::nav::endpoint {

SemanticViewQuery::SemanticViewQuery(std::shared_ptr<ActiveOctomapGate> gate,
    std::string map_path, plan::GlobalPlannerOptions options)
    : gate_(std::move(gate)), map_path_(std::move(map_path)), options_(options) {}

SemanticViewQuery::~SemanticViewQuery() {
  cancel();
  if (future_.valid()) future_.wait();
}

bool SemanticViewQuery::start(SemanticViewContext context) {
  if (busy()) return false;
  cancelled_.store(false);
  future_ = std::async(std::launch::async, [this, context = std::move(context)] {
    return run(context);
  });
  return true;
}

std::optional<semantic::ViewResult> SemanticViewQuery::poll() {
  if (!future_.valid() || future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return std::nullopt;
  auto result = future_.get();
  // A command may cancel after the worker finishes but before it is polled.
  if (cancelled_.load()) {
    result.available = false;
    result.geometry_exhausted = false;
    result.candidates.clear();
    result.reason = "semantic_search_query_cancelled";
  }
  return result;
}

void SemanticViewQuery::cancel() { cancelled_.store(true); }

semantic::ViewResult SemanticViewQuery::run(const SemanticViewContext& context) {
  semantic::ViewResult result;
  result.request_id = context.query.request_id;
  result.boot_id = context.boot_id;
  result.map = context.map;
  result.frame_epoch = context.frame_epoch;
  result.timestamp_s = context.timestamp_s;
  const auto reject = [&](const std::string& reason) {
    result.reason = reason;
    return result;
  };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  const auto cancel = [&] {
    return cancelled_.load() || std::chrono::steady_clock::now() >= deadline;
  };
  try {
    const auto& request = context.query;
    if (!gate_ || !context.map.valid() || !options_.require_ground_support)
      return reject("semantic_search_saved_grounded_map_required");
    if (request.boot_id != context.boot_id || !plan::sameMapIdentity(request.map, context.map)
        || (request.frame_epoch != 0U && request.frame_epoch != context.frame_epoch)
        || (!request.views.empty() && request.frame_epoch == 0U))
      return reject("semantic_search_context_mismatch");
    if (!std::isfinite(context.robot.x) || !std::isfinite(context.robot.y)
        || !std::isfinite(context.robot.z) || !std::isfinite(context.yaw)
        || request.views.size() > 256U || !std::isfinite(request.camera_range_m)
        || request.camera_range_m <= 0.0 || !std::isfinite(request.camera_horizontal_fov_rad)
        || request.camera_horizontal_fov_rad <= 0.0
        || request.camera_horizontal_fov_rad > 6.283185307179586)
      return reject("invalid_semantic_search_request");
    auto prepared = gate_->prepare(map_path_);
    if (!prepared.ok()) return reject(prepared.reason);
    if (!plan::sameMapIdentity(prepared.artifact->identity(), context.map))
      return reject("semantic_search_map_changed");
    auto projection = session_.project(prepared.artifact->loadPath(), context.map,
                                      options_, context.robot.z, cancel);
    if (!projection.available) return reject(projection.reason);
    // The queried cell center is the tested robot reference height, not the
    // unsnapped input height, nor a ground point or display-layer offset.
    result.reference_z = projection.origin.z;
    if (!request.views.empty() && (!std::isfinite(request.reference_z)
        || std::abs(request.reference_z - result.reference_z) > projection.resolution * 0.25))
      return reject("semantic_search_height_layer_changed");
    if (cancel()) return reject("semantic_search_query_cancelled");

    explore::ExploreInput input;
    input.map = {context.map.frame_id, "", context.map.map_id, context.map.content_epoch,
                 context.frame_epoch, 1U, false};
    input.map_frame = context.map.frame_id;
    input.stamp_s = context.timestamp_s;
    input.robot_pose = {context.robot.x, context.robot.y, context.yaw};
    auto& grid = input.exploration_grid;
    grid.width = projection.cols;
    grid.height = projection.rows;
    grid.resolution = projection.resolution;
    grid.origin_x = projection.origin.x;
    grid.origin_y = projection.origin.y;
    grid.cells.reserve(projection.cells.size());
    std::vector<double> heights;
    heights.reserve(projection.cells.size());
    for (const auto cell : projection.cells) {
      grid.cells.push_back(cell == 1U ? explore::kFree : cell == 2U ? explore::kOccupied : explore::kUnknown);
      heights.push_back(cell == 1U ? result.reference_z : std::numeric_limits<double>::quiet_NaN());
    }
    explore::SemanticSearchHistory history;
    history.map = input.map;
    for (const auto& view : request.views)
      history.views.push_back({{view.x, view.y, view.yaw}, view.range_m, view.horizontal_fov_rad});
    explore::TarePolicyConfig config;
    config.sensor_range_m = request.camera_range_m;
    config.sensor_horizontal_fov_rad = request.camera_horizontal_fov_rad;
    config.max_plan_time_ms = 200.0;
    const auto proposals = explore::ProposeSemanticViews(input, heights, history, config, cancel);
    if (!proposals.map.valid()) return reject(proposals.reason);
    if (cancel()) return reject("semantic_search_query_cancelled");
    result.available = true;
    result.geometry_exhausted = proposals.geometry_exhausted;
    result.reason = proposals.reason;
    for (const auto& candidate : proposals.candidates)
      result.candidates.push_back({{candidate.x, candidate.y, candidate.z}, candidate.yaw,
          candidate.score, candidate.route_cost_m, static_cast<std::uint32_t>(candidate.frontier_size)});
    return result;
  } catch (const std::exception& error) {
    return reject(std::string("semantic_search_query_failed: ") + error.what());
  }
}

}  // namespace lingtu::nav::endpoint
