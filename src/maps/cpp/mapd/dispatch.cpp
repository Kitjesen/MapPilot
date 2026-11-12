#include "dispatch.hpp"
#include "lingtu/maps/service.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/mapd/save.hpp"

namespace lingtu::maps::mapd::query {
namespace {

std::string Escape(const std::string &value) {
  std::ostringstream out;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          constexpr char hex[] = "0123456789abcdef";
          out << "\\u00" << hex[(ch >> 4U) & 0x0fU] << hex[ch & 0x0fU];
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

std::string Failure(const std::string &action, const std::string &reason,
                    const std::string &message) {
  return "{\"action\":\"" + Escape(action) + "\",\"success\":false,\"reason_code\":\"" +
         Escape(reason) + "\",\"message\":\"" + Escape(message) + "\"}";
}

std::optional<std::string_view> TopLevelScalar(std::string_view json, std::string_view wanted_key) {
  auto skip_space = [&json](std::size_t *cursor) {
    while (*cursor < json.size() && (json[*cursor] == ' ' || json[*cursor] == '\t' ||
                                     json[*cursor] == '\r' || json[*cursor] == '\n')) {
      ++*cursor;
    }
  };
  auto string_end = [&json](std::size_t cursor) {
    ++cursor;
    while (cursor < json.size()) {
      if (json[cursor] == '\\') {
        cursor += 2U;
      } else if (json[cursor++] == '"') {
        return cursor;
      }
    }
    return json.size();
  };

  std::size_t cursor = 1U;
  while (cursor < json.size()) {
    skip_space(&cursor);
    if (cursor >= json.size() || json[cursor] == '}')
      return std::nullopt;
    if (json[cursor] != '"')
      return std::nullopt;
    const std::size_t key_begin = cursor + 1U;
    const std::size_t key_end = string_end(cursor);
    if (key_end == 0U || key_end > json.size())
      return std::nullopt;
    const bool matches = json.substr(key_begin, key_end - key_begin - 1U) == wanted_key;
    cursor = key_end;
    skip_space(&cursor);
    if (cursor >= json.size() || json[cursor] != ':')
      return std::nullopt;
    ++cursor;
    skip_space(&cursor);
    const std::size_t value_begin = cursor;
    bool in_string = false;
    int depth = 0;
    while (cursor < json.size()) {
      const char ch = json[cursor];
      if (in_string) {
        if (ch == '\\') {
          cursor += 2U;
          continue;
        }
        in_string = ch != '"';
      } else if (ch == '"') {
        in_string = true;
      } else if (ch == '{' || ch == '[') {
        ++depth;
      } else if (ch == '}' || ch == ']') {
        if (depth == 0)
          break;
        --depth;
      } else if (ch == ',' && depth == 0) {
        break;
      }
      ++cursor;
    }
    std::size_t value_end = cursor;
    while (value_end > value_begin &&
           (json[value_end - 1U] == ' ' || json[value_end - 1U] == '\t' ||
            json[value_end - 1U] == '\r' || json[value_end - 1U] == '\n')) {
      --value_end;
    }
    if (matches)
      return json.substr(value_begin, value_end - value_begin);
    if (cursor < json.size() && json[cursor] == ',')
      ++cursor;
  }
  return std::nullopt;
}

class Fields {
 public:
  explicit Fields(const std::string &json) : json_(json) {}

  bool Has(const char *key) const { return JsonObjectHasPath(json_, {key}); }
  std::vector<SourceEditOperation> Edits() const try {
    const auto doc = ParseJson(json_);
    const auto &object = doc.AsObject("request");
    const auto item = object.find("operations");
    if (item == object.end())
      throw std::invalid_argument("operations is required");
    std::vector<SourceEditOperation> result;
    for (const auto &value : item->second.AsArray("operations")) {
      const auto &edit = value.AsObject("operation");
      SourceEditOperation parsed;
      parsed.operation = edit.at("operation").AsString("operation");
      auto number = [&](const JsonValue::Object &fields, const char *key, double fallback) {
        const auto field = fields.find(key);
        if (field == fields.end()) return fallback;
        const auto *number = std::get_if<double>(&field->second.value);
        if (number == nullptr || !std::isfinite(*number))
          throw std::invalid_argument(std::string("invalid edit field: ") + key);
        return *number;
      };
      const auto integer = [&](const char *key, int fallback) {
        const double value = number(edit, key, fallback);
        if (std::floor(value) != value || value < 0 ||
            value > std::numeric_limits<int>::max())
          throw std::invalid_argument(std::string("invalid integer edit field: ") + key);
        return static_cast<int>(value);
      };
      if (parsed.operation == "delete_inside" || parsed.operation == "keep_inside" || parsed.operation == "add_plane") {
        const auto &bounds = edit.at("bounds").AsObject("bounds");
        const auto coordinate = [&](const char *key) {
          if (bounds.find(key) == bounds.end())
            throw std::invalid_argument(std::string("missing bounds field: ") + key);
          return number(bounds, key, 0);
        };
        parsed.bounds = {true, coordinate("min_x"), coordinate("min_y"), coordinate("min_z"),
                        coordinate("max_x"), coordinate("max_y"), coordinate("max_z")};
        if (parsed.operation == "add_plane") {
          parsed.axis = edit.at("axis").AsString("axis");
          parsed.offset = number(edit, "offset", parsed.offset);
          parsed.spacing = number(edit, "spacing", parsed.spacing);
        }
      } else if (parsed.operation == "voxel") {
        parsed.voxel_size = number(edit, "voxel_size", parsed.voxel_size);
      } else if (parsed.operation == "statistical") {
        parsed.neighbors = integer("neighbors", parsed.neighbors);
        parsed.std_ratio = number(edit, "std_ratio", parsed.std_ratio);
      } else if (parsed.operation == "radius") {
        parsed.radius = number(edit, "radius", parsed.radius);
        parsed.min_points = integer("min_points", parsed.min_points);
      } else if (parsed.operation == "clusters") {
        parsed.radius = number(edit, "radius", parsed.radius);
        parsed.neighbors = integer("neighbors", parsed.neighbors);
        parsed.min_cluster_points = integer("min_cluster_points", parsed.min_cluster_points);
      } else if (parsed.operation == "plane_crop") {
        parsed.distance_threshold = number(edit, "distance_threshold", parsed.distance_threshold);
        parsed.margin = number(edit, "margin", parsed.margin);
      }
      result.push_back(std::move(parsed));
    }
    return result;
  } catch (const std::out_of_range &) {
    throw std::invalid_argument("each edit requires an operation and its parameters");
  }
  std::string String(const char *key) const {
    const auto value = JsonObjectStringAtPath(json_, {key});
    if (!value.has_value())
      throw std::invalid_argument(std::string("missing or invalid string field: ") + key);
    return *value;
  }
  std::string StringOr(const char *key, std::string fallback) const {
    if (!JsonObjectHasPath(json_, {key}))
      return fallback;
    return String(key);
  }
  bool Bool(const char *key) const {
    const auto value = JsonObjectBoolAtPath(json_, {key});
    if (!value.has_value())
      throw std::invalid_argument(std::string("missing or invalid bool field: ") + key);
    return *value;
  }
  bool BoolOr(const char *key, bool fallback) const {
    if (!JsonObjectHasPath(json_, {key}))
      return fallback;
    return Bool(key);
  }
  double Number(const char *key) const {
    const auto value = JsonObjectNumberAtPath(json_, {key});
    if (!value.has_value() || !std::isfinite(*value)) {
      throw std::invalid_argument(std::string("missing or invalid number field: ") + key);
    }
    return *value;
  }
  double NumberOr(const char *key, double fallback) const {
    if (!JsonObjectHasPath(json_, {key}))
      return fallback;
    return Number(key);
  }
  std::uint64_t U64Or(const char *key, std::uint64_t fallback) const {
    if (!JsonObjectHasPath(json_, {key}))
      return fallback;
    std::string value;
    if (const auto text = JsonObjectStringAtPath(json_, {key}); text.has_value()) {
      value = *text;
    } else {
      const auto scalar = TopLevelScalar(json_, key);
      if (!scalar.has_value()) {
        throw std::invalid_argument(std::string("invalid unsigned integer field: ") + key);
      }
      value.assign(scalar->data(), scalar->size());
    }
    std::uint64_t parsed = 0U;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
      throw std::invalid_argument(std::string("invalid unsigned integer field: ") + key);
    }
    return parsed;
  }
  int IntOr(const char *key, int fallback) const {
    const double value = NumberOr(key, static_cast<double>(fallback));
    if (std::floor(value) != value || value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max()) {
      throw std::invalid_argument(std::string("invalid integer field: ") + key);
    }
    return static_cast<int>(value);
  }

 private:
  const std::string &json_;
};

PcdBounds Bounds(const Fields &fields) {
  PcdBounds bounds;
  bounds.enabled = fields.BoolOr("has_bounds", false);
  bounds.min_x = fields.NumberOr("min_x", 0.0);
  bounds.min_y = fields.NumberOr("min_y", 0.0);
  bounds.min_z = fields.NumberOr("min_z", 0.0);
  bounds.max_x = fields.NumberOr("max_x", 0.0);
  bounds.max_y = fields.NumberOr("max_y", 0.0);
  bounds.max_z = fields.NumberOr("max_z", 0.0);
  return bounds;
}

OctomapBuildOptions OctomapOptions(const Fields &fields) {
  OctomapBuildOptions options;
  const std::string build_mode = fields.StringOr("octomap_build_mode", "native_octomap");
  if (build_mode != "native_octomap") {
    throw std::invalid_argument("unsupported octomap_build_mode: " + build_mode +
                                "; maps are built by the embedded OctoMap builder");
  }
  options.resolution = fields.NumberOr("octomap_resolution", options.resolution);
  options.support_dilation_cells = fields.IntOr("octomap_support_dilation_cells", 0);
  options.free_layers_above = fields.IntOr("octomap_free_layers_above", 0);
  options.free_dilation_cells = fields.IntOr("octomap_free_dilation_cells", 0);
  options.frame_id = fields.StringOr("octomap_frame_id", options.frame_id);
  options.source_profile = fields.StringOr("octomap_source_profile", options.source_profile);
  options.data_source = fields.StringOr("octomap_data_source", options.source_profile);
  options.slam_source = fields.StringOr("octomap_slam_source", "unknown");
  options.localization_source = fields.StringOr("octomap_localization_source", options.slam_source);
  options.mapping_source = fields.StringOr("octomap_mapping_source", "lingtu_maps_pipeline");
  return options;
}

std::string Invoke(MapsServiceCore &service, mapd::SaveCoordinator *save_coordinator,
                   const std::string &action, const Fields &f) {
  if (action == "list_maps")
    return service.ListMapsJson();
  if (action == "get_map_types")
    return service.GetMapTypesJson();
  if (action == "get_record")
    return service.GetRecordJson(f.String("map_id"));
  if (action == "get_active_map")
    return service.GetActiveMapJson();
  if (action == "get_health")
    return service.GetHealthJson(f.StringOr("map_id", ""));
  if (action == "validate_artifacts")
    return service.ValidateArtifactsJson(
        f.String("map_id"), f.BoolOr("require_octomap", false),
        f.BoolOr("require_occupancy", false), f.StringOr("expected_frame_id", ""),
        f.StringOr("expected_data_source", ""), f.StringOr("expected_source_profile", ""));
  if (action == "get_bundle")
    return service.GetBundleJson(f.StringOr("map_id", ""), f.String("capability"));
  if (action == "get_map_points")
    return service.GetMapPointsJson(f.StringOr("map_id", ""), f.U64Or("max_points", 0U));
  if (action == "save_map") {
    if (save_coordinator == nullptr) {
      throw std::runtime_error("native SaveMap coordinator is unavailable");
    }
    return save_coordinator->SaveMapJson(f.String("request_id"), f.String("map_id"),
                                         f.BoolOr("auto_process", true));
  }
  if (action == "process_save_map")
    return service.ProcessSaveMapJson(f.String("job_id"));
  if (action == "reoptimize_save_map")
    return service.ReoptimizeSaveMapJson(f.String("job_id"), f.String("request_id"));
  if (action == "get_save_map_status")
    return service.GetSaveMapStatusJson(f.String("job_id"));
  if (action == "list_save_map_jobs")
    return service.ListSaveMapJobsJson(static_cast<std::size_t>(f.U64Or("limit", 100U)));
  if (action == "cancel_save_map") {
    if (save_coordinator == nullptr) {
      throw std::runtime_error("native SaveMap coordinator is unavailable");
    }
    return save_coordinator->CancelSaveMapJson(f.String("job_id"));
  }
  if (action == "retry_save_map") {
    if (save_coordinator == nullptr) {
      throw std::runtime_error("native SaveMap coordinator is unavailable");
    }
    return save_coordinator->RetrySaveMapJson(f.String("job_id"));
  }
  if (action == "list_poi")
    return service.ListPoiJson(f.StringOr("map_id", ""));
  if (action == "set_poi")
    return service.SetPoiJson(f.StringOr("map_id", ""), f.String("name"), f.Number("x_m"),
                              f.Number("y_m"), f.Number("z_m"), f.NumberOr("yaw_rad", 0.0),
                              f.BoolOr("has_yaw", false), f.StringOr("frame_id", "map"),
                              f.StringOr("tags_json", "{}"));
  if (action == "delete_poi")
    return service.DeletePoiJson(f.StringOr("map_id", ""), f.String("name"));
  if (action == "list_stairs")
    return service.ListStairsJson(f.String("map_id"));
  if (action == "set_stair")
    return service.SetStairJson(f.String("map_id"), f.String("stair_id"),
                                f.String("annotation_json"));
  if (action == "delete_stair")
    return service.DeleteStairJson(f.String("map_id"), f.String("stair_id"),
                                   f.U64Or("content_epoch", 0U));
  if (action == "list_map_graph")
    return service.ListMapGraphJson();
  if (action == "set_map_edge")
    return service.SetMapEdgeJson(f.String("from_map_id"), f.String("to_map_id"),
                                  f.StringOr("edge_type", "link"),
                                  f.BoolOr("bidirectional", false));
  if (action == "delete_map_edge")
    return service.DeleteMapEdgeJson(f.String("from_map_id"), f.String("to_map_id"));
  if (action == "shortest_route")
    return service.ShortestRouteJson(f.String("start_map_id"), f.String("goal_map_id"));
  if (action == "audit_maps")
    return service.AuditMapsJson(f.BoolOr("dry_run", true));
  if (action == "quarantine_corrupt_maps")
    return service.QuarantineCorruptMapsJson(f.BoolOr("dry_run", true));
  if (action == "export_map_package")
    return service.ExportMapPackageJson(f.String("map_id"), f.String("package_dir"),
                                        f.BoolOr("dry_run", true));
  if (action == "import_map_package")
    return service.ImportMapPackageJson(f.String("package_dir"), f.StringOr("requested_map_id", ""),
                                        f.BoolOr("dry_run", true));
  if (action == "create_map")
    return service.CreateMapJson(f.String("map_id"));
  if (action == "delete_map")
    return service.DeleteMapJson(f.String("map_id"));
  if (action == "rename_map")
    return service.RenameMapJson(f.String("map_id"), f.String("new_map_id"));
  if (action == "retire_map")
    return service.RetireMapJson(f.String("map_id"));
  if (action == "import_pcd")
    return service.ImportPcdJson(f.String("map_id"), f.String("source_path"),
                                 f.NumberOr("voxel_size", 0.0), Bounds(f));
  if (action == "commit_saved_source") {
    SourceCommitOptions options;
    options.voxel_size = f.NumberOr("voxel_size", 0.0);
    options.dynamic_filter_enabled = f.BoolOr("dynamic_filter_enabled", options.dynamic_filter_enabled);
    options.dynamic_filter_required = f.BoolOr("dynamic_filter_required", options.dynamic_filter_required);
    options.dynamic_filter_command = f.StringOr("dynamic_filter_command", options.dynamic_filter_command);
    options.dynamic_filter_timeout_sec = f.NumberOr("dynamic_filter_timeout_sec", options.dynamic_filter_timeout_sec);
    return service.CommitSavedSourceJson(f.String("map_id"), f.String("source_dir"), options);
  }
  if (action == "preview_source_edits")
    return service.PreviewSourceEditsJson(f.String("map_id"),
        f.U64Or("expected_source_content_epoch", 0U), f.Edits());
  if (action == "save_edited_source")
    return service.SaveEditedSourceJson(f.String("map_id"), f.String("new_map_id"),
        f.U64Or("expected_source_content_epoch", 0U), f.Edits());
  if (action == "crop_pcd")
    return service.CropPcdJson(f.String("map_id"), Bounds(f), f.BoolOr("invert", false),
                               f.NumberOr("voxel_size", 0.0));
  if (action == "build_occupancy_snapshot")
    return service.BuildOccupancySnapshotJson(f.String("map_id"));
  if (action == "build_octomap_artifact") {
    auto options = OctomapOptions(f);
    options.inherit_saved_metadata = f.BoolOr("inherit_saved_metadata",
        !f.Has("octomap_resolution") && !f.Has("octomap_frame_id") &&
        !f.Has("octomap_source_profile") && !f.Has("octomap_data_source") &&
        !f.Has("octomap_slam_source") && !f.Has("octomap_localization_source") &&
        !f.Has("octomap_mapping_source"));
    return service.BuildOctomapArtifactJson(f.String("map_id"), options);
  }
  if (action == "get_voxel_edits")
    return service.GetVoxelEditsJson(f.String("map_id"));
  if (action == "get_no_go_regions")
    return service.GetNoGoRegionsJson(f.String("map_id"));
  if (action == "get_surface_map") {
    SurfaceMapOptions options;
    options.headroom_m = f.NumberOr("headroom_m", options.headroom_m);
    options.radius_m = f.NumberOr("radius_m", options.radius_m);
    options.max_hole_area_m2 = f.NumberOr("max_hole_area_m2", options.max_hole_area_m2);
    options.max_slope = f.NumberOr("max_slope", options.max_slope);
    return service.GetSurfaceMapJson(f.String("map_id"), options);
  }
  if (action == "set_no_go_region") {
    NoGoRegion region{f.String("region_id"),
        {f.Number("min_x"), f.Number("min_y"), f.Number("min_z")},
        {f.Number("max_x"), f.Number("max_y"), f.Number("max_z")}};
    region.label = f.StringOr("region_label", "");
    region.kind = f.StringOr("region_kind", "no_go");
    return service.SetNoGoRegionJson(f.String("map_id"), region,
                                     f.U64Or("expected_content_epoch", 0U));
  }
  if (action == "delete_no_go_region")
    return service.DeleteNoGoRegionJson(f.String("map_id"), f.String("region_id"),
                                        f.U64Or("expected_content_epoch", 0U));
  if (action == "edit_octomap_voxels") {
    OctomapEditOptions options;
    options.state = f.String("state");
    options.shape = f.StringOr("shape", "sphere");
    options.x_m = f.NumberOr("x_m", 0.0);
    options.y_m = f.NumberOr("y_m", 0.0);
    options.z_m = f.NumberOr("z_m", 0.0);
    options.radius_m = f.NumberOr("radius_m", 0.20);
    options.timeout_sec = f.NumberOr("timeout_sec", 15.0);
    return service.EditOctomapVoxelsJson(f.String("map_id"), options);
  }
  if (action == "build_navigation_package")
    return service.BuildNavigationPackageJson(f.String("map_id"), OctomapOptions(f),
                                              f.BoolOr("include_esdf", false),
                                              f.BoolOr("include_traversability", false));
  if (action == "build_esdf_artifact")
    return service.BuildEsdfArtifactJson(f.String("map_id"));
  if (action == "build_traversability_artifact")
    return service.BuildTraversabilityArtifactJson(f.String("map_id"));
  if (action == "build_semantic_artifact")
    return service.BuildSemanticArtifactJson(f.String("map_id"));
  if (action == "ingest_localization_health")
    return service.IngestLocalizationHealthJson(f.StringOr("map_id", ""), f.Number("timestamp_s"),
                                                f.Bool("localized"), f.Number("position_error_m"),
                                                f.Number("covariance_trace"), f.Number("quality"),
                                                f.StringOr("source", "runtime.localization"));
  if (action == "ingest_planning_outcome")
    return service.IngestPlanningOutcomeJson(f.StringOr("map_id", ""), f.Number("timestamp_s"),
                                             f.Bool("success"), f.StringOr("planner", "unknown"),
                                             f.StringOr("reason", ""));
  if (action == "ingest_collision_event")
    return service.IngestCollisionEventJson(
        f.StringOr("map_id", ""), f.Number("timestamp_s"), f.Number("severity"),
        f.StringOr("source", "runtime.safety"), f.StringOr("reason", ""));
  throw std::domain_error("unknown service action");
}

}  // namespace

DispatchResult DispatchServiceJson(MapsServiceCore &service,
                                   mapd::SaveCoordinator *save_coordinator,
                                   const std::string &request_json) {
  if (!IsValidJsonObject(request_json)) {
    return {false, Failure("service", "invalid_request", "request JSON must be an object")};
  }
  const Fields fields(request_json);
  const auto action_value = JsonObjectStringAtPath(request_json, {"action"});
  if (!action_value.has_value() || action_value->empty()) {
    return {false,
            Failure("service", "invalid_request", "missing or invalid string field: action")};
  }
  const std::string &action = *action_value;
  try {
    std::string result = Invoke(service, save_coordinator, action, fields);
    bool ok = true;
    if (JsonObjectHasPath(result, {"success"})) {
      ok = JsonObjectBoolAtPath(result, {"success"}) == true;
    } else if (JsonObjectHasPath(result, {"accepted"})) {
      ok = JsonObjectBoolAtPath(result, {"accepted"}) == true;
    }
    return {ok, std::move(result)};
  } catch (const std::domain_error &) {
    return {false, Failure(action, "unknown_action", "unknown service action: " + action)};
  } catch (const std::invalid_argument &exc) {
    const bool source_edit = action == "save_edited_source" || action == "preview_source_edits";
    return {false, Failure(action, source_edit ? "invalid_operations" : "invalid_request", exc.what())};
  } catch (const std::exception &exc) {
    return {false, Failure(action, "internal_error", exc.what())};
  }
}

}  // namespace lingtu::maps::mapd::query
