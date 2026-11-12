#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/io/pcd.hpp"
#include "lingtu/maps/io/patch_bundle.hpp"
#include "lingtu/maps/storage/lock.hpp"
#include "mapd/dispatch.hpp"
#include "lingtu/maps/mapd/protocol.hpp"
#include "lingtu/maps/service.hpp"
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
#include <octomap/OcTree.h>
#endif
#include "lingtu/maps/storage/stairs.hpp"

namespace {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
void TestSurfacePreview(const std::filesystem::path& root) {
  using namespace lingtu::maps;
  using mapd::query::DispatchServiceJson;
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  assert(service.Store().CreateMap("surface").ok);
  const auto dir = service.Store().MapPath("surface");
  { std::ofstream metadata(dir / "metadata.json"); metadata << R"({"frame_id":"map"})"; }
  octomap::OcTree tree(.1);
  for (int z : {-1, 19}) for (int x=-5; x<=5; ++x) for (int y=-5; y<=5; ++y)
    if (x || y) tree.updateNode((x+.5)*.1, (y+.5)*.1, (z+.5)*.1, true, true);
  tree.updateInnerOccupancy();
  assert(tree.write((dir / "octomap.ot").string()));
  const auto epoch = service.Store().ContentEpoch("surface");
  auto result = DispatchServiceJson(service, R"({"action":"get_surface_map","map_id":"surface"})");
  assert(result.ok);
  assert(JsonObjectNumberAtPath(result.json, {"inferred_count"}) == 2);
  assert(JsonObjectNumberAtPath(result.json, {"observed_count"}) == 240);
  assert(JsonObjectNumberAtPath(result.json, {"content_epoch"}) == epoch);
  const auto parsed = ParseJson(result.json).AsObject("surface");
  const auto& xyz = parsed.at("inferred_xyz").AsArray("points");
  assert(xyz.size() == 6);
  tree.updateNode(.05,.05,-.05,false,true);
  tree.updateInnerOccupancy();
  assert(tree.write((dir / "octomap.ot").string()));
  result = DispatchServiceJson(service, R"({"action":"get_surface_map","map_id":"surface"})");
  assert(result.ok);
  assert(JsonObjectNumberAtPath(result.json, {"inferred_count"}) == 1);
  assert(service.Store().ContentEpoch("surface") == epoch);
  assert(!DispatchServiceJson(service, R"({"action":"get_surface_map","map_id":"surface","radius_m":-1})").ok);
}
#endif

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("lingtu_mapd_dispatch_test_" + std::to_string(stamp));
  std::filesystem::create_directories(root);
  return root;
}

std::string StairBody(std::int64_t epoch, int base_z, const std::string& name = "east stairs") {
  return "{\"name\":" + lingtu::maps::JsonString(name) +
      ",\"frame_id\":\"map\",\"content_epoch\":" + std::to_string(epoch) +
      ",\"bounds\":{\"min\":{\"x\":0,\"y\":0,\"z\":" + std::to_string(base_z) +
      "},\"max\":{\"x\":4,\"y\":4,\"z\":" + std::to_string(base_z + 4) +
      "}},\"lower\":{\"x\":1,\"y\":1,\"z\":" + std::to_string(base_z) +
      "},\"upper\":{\"x\":3,\"y\":1,\"z\":" + std::to_string(base_z + 3) + "}}";
}

void TestStairAnnotations(const std::filesystem::path& root) {
  using namespace lingtu::maps;
  using lingtu::maps::mapd::query::DispatchServiceJson;
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  assert(service.Store().CreateMap("building").ok);
  {
    std::ofstream metadata(root / "building" / "metadata.json");
    metadata << R"({"frame_id":"map"})";
  }
  const auto epoch = service.Store().ContentEpoch("building");
  assert(JsonObjectBoolAtPath(service.SetPoiJson("building", "ordinary", 9, 8, 7, 0,
      false, "map", R"({"custom":{"keep":true}})"), {"success"}) == true);
  auto set_stair = [&](MapsServiceCore& target, const std::string& id, const std::string& body) {
    return DispatchServiceJson(target, "{\"action\":\"set_stair\",\"map_id\":\"building\","
        "\"stair_id\":" + JsonString(id) + ",\"annotation_json\":" + JsonString(body) + "}");
  };
  auto list_stairs = [&](MapsServiceCore& target) {
    return DispatchServiceJson(target, R"({"action":"list_stairs","map_id":"building"})");
  };
  const auto first_stair = set_stair(service, "east-lower", StairBody(epoch, 0));
  if (!first_stair.ok) std::cerr << first_stair.json << std::endl;
  assert(first_stair.ok);
  assert(set_stair(service, "east-upper", StairBody(epoch, 6)).ok);
  assert(service.Store().ContentEpoch("building") == epoch);
  assert(!std::filesystem::exists(root / "map_graph.ltg"));

  MapsServiceCore reloaded(MapsServiceConfig{MapStoreConfig{root}});
  const auto persisted = list_stairs(reloaded);
  assert(persisted.ok);
  const auto document = ParseJson(persisted.json);
  const auto& stairs = document.AsObject("response").at("stairs").AsArray("stairs");
  assert(stairs.size() == 2U);
  const auto& first = stairs[0].AsObject("stair");
  const auto& second = stairs[1].AsObject("stair");
  assert(first.at("id").AsString("id") == "east-lower");
  assert(second.at("id").AsString("id") == "east-upper");
  assert(std::get<double>(first.at("lower").AsObject("lower").at("z").value) == 0.0);
  assert(std::get<double>(second.at("lower").AsObject("lower").at("z").value) == 6.0);
  const auto ordinary = reloaded.ListPoiJson("building");
  assert(ordinary.find("east-lower") == std::string::npos);
  assert(JsonObjectBoolAtPath(ordinary, {"pois", "ordinary", "tags", "custom", "keep"}) == true);
  assert(JsonObjectBoolAtPath(reloaded.SetPoiJson("building", "stair:east-lower", 0, 0, 0,
      0, false, "map", "{}"), {"success"}) == false);
  assert(JsonObjectBoolAtPath(reloaded.DeletePoiJson("building", "stair:east-lower"),
      {"success"}) == false);

  const auto stale_write = set_stair(reloaded, "east-lower", StairBody(epoch + 1, 0, "wrong"));
  assert(!stale_write.ok);
  assert(JsonObjectStringAtPath(stale_write.json, {"reason_code"}) == "content_epoch_conflict");
  assert(list_stairs(reloaded).json == persisted.json);
  auto invalid_body = StairBody(epoch, 0);
  invalid_body.replace(invalid_body.find("\"x\":4"), 5, "\"x\":0");
  assert(!set_stair(reloaded, "east-lower", invalid_body).ok);
  invalid_body = StairBody(epoch, 0);
  invalid_body.replace(invalid_body.find("\"z\":3"), 5, "\"z\":0");
  assert(!set_stair(reloaded, "east-lower", invalid_body).ok);
  invalid_body = StairBody(epoch, 0);
  invalid_body.replace(invalid_body.find("\"x\":0"), 5, "\"x\":1e309");
  assert(!set_stair(reloaded, "east-lower", invalid_body).ok);
  invalid_body = StairBody(epoch, 0);
  invalid_body.replace(invalid_body.find("\"frame_id\":\"map\""), 16, "\"frame_id\":\"odom\"");
  const auto wrong_frame = set_stair(reloaded, "east-lower", invalid_body);
  assert(!wrong_frame.ok);
  assert(JsonObjectStringAtPath(wrong_frame.json, {"reason_code"}) == "frame_id_conflict");
  assert(set_stair(reloaded, "east-lower", StairBody(epoch, 1, "edited")).ok);
  assert(list_stairs(reloaded).json.find("edited") != std::string::npos);

  auto delete_stair = [&](std::int64_t expected) {
    return DispatchServiceJson(reloaded,
        "{\"action\":\"delete_stair\",\"map_id\":\"building\",\"stair_id\":\"east-lower\","
        "\"content_epoch\":" + std::to_string(expected) + "}");
  };
  assert(!delete_stair(epoch + 1).ok);
  assert(delete_stair(epoch).ok);
  assert(!delete_stair(epoch).ok);
  assert(list_stairs(reloaded).json.find("east-lower") == std::string::npos);
  assert(JsonObjectBoolAtPath(reloaded.ListPoiJson("building"),
      {"pois", "ordinary", "tags", "custom", "keep"}) == true);

  {
    std::ofstream marker(root / "building" / ".content_epoch");
    marker << epoch + 10 << '\n';
  }
  const auto changed_map = list_stairs(reloaded);
  assert(changed_map.ok);
  assert(JsonObjectNumberAtPath(changed_map.json, {"content_epoch"}) == epoch + 10);
  const auto changed_document = ParseJson(changed_map.json);
  const auto& old_stair = changed_document.AsObject("response").at("stairs")
                              .AsArray("stairs").front().AsObject("stair");
  assert(old_stair.at("content_epoch").AsUnsigned("epoch") == static_cast<std::uint64_t>(epoch));
  assert(!set_stair(reloaded, "east-upper", StairBody(epoch, 6)).ok);
}

void TestStairRecognitionPersistence(const std::filesystem::path& root) {
  using namespace lingtu::maps;
  using lingtu::maps::mapd::query::DispatchServiceJson;
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  assert(service.Store().CreateMap("stairs").ok);
  {
    std::ofstream metadata(root / "stairs" / "metadata.json");
    metadata << R"({"frame_id":"map"})";
  }
  const auto epoch = service.Store().ContentEpoch("stairs");
  assert(JsonObjectBoolAtPath(service.SetPoiJson("stairs", "desk", 3, 4, 0,
      0, false, "map", R"({"label":"keep me"})"), {"success"}) == true);
  const auto ordinary = service.ListPoiJson("stairs");
  const auto source = service.Store().ContentPath("stairs") / "map.pcd";
  std::vector<PointXyz> points;
  auto surface = [&](double begin, double end, double z, double slope) {
    for (int x = 0; begin+x*0.025 <= end+1e-6; ++x)
      for (int y = -20; y <= 20; ++y)
        points.push_back({static_cast<float>(begin+x*0.025),y*0.025F,
                          static_cast<float>(z+slope*x*0.025)});
  };
  surface(-0.6,-0.025,0,0);
  for (int step = 0; step < 5; ++step) surface(step*0.3,step*0.3+0.275,(step+1)*0.15,0);
  surface(1.5,2.1,0.9,0);
  std::string error;
  assert(WriteBinaryXyzPcd(source,points,&error));
  const auto body = "{\"name\":\"flight\",\"frame_id\":\"map\",\"content_epoch\":" + std::to_string(epoch) +
      R"(,"bounds":{"min":{"x":-0.7,"y":-0.6,"z":-0.1},"max":{"x":2.2,"y":0.6,"z":1}},"lower":{"x":-0.3,"y":0,"z":0},"upper":{"x":1.8,"y":0,"z":0.9},"geometry":{"recognized":true,"reason":"client claim","surfaces":"not real geometry"}})";
  auto save = [&](MapsServiceCore& target, const std::string& id) {
    return DispatchServiceJson(target,"{\"action\":\"set_stair\",\"map_id\":\"stairs\",\"stair_id\":" +
        JsonString(id) + ",\"annotation_json\":" + JsonString(body) + "}");
  };
  const auto saved = save(service,"observed");
  if (!saved.ok) std::cerr << saved.json << '\n';
  assert(saved.ok);
  auto loaded = ReadStairAnnotations(root / "stairs" / "pois.tsv");
  assert(loaded.size() == 1U && loaded.front().geometry.recognized);
  assert(loaded.front().geometry.reason == "stairs");
  assert(loaded.front().geometry.surfaces.size() == 7U);
  assert(std::abs(loaded.front().geometry.mean_rise_m-0.15) < 0.01);
  assert(loaded.front().geometry.surfaces[1].observed_points > 0U);
  assert(service.Store().ContentEpoch("stairs") == epoch);
  assert(service.ListPoiJson("stairs") == ordinary);
  MapsServiceCore reloaded(MapsServiceConfig{MapStoreConfig{root}});
  const auto listed = ParseJson(reloaded.ListStairsJson("stairs"));
  const auto& rows = listed.AsObject("response").at("stairs").AsArray("stairs");
  assert(std::get<bool>(rows.front().AsObject("stair").at("geometry").AsObject("geometry").at("recognized").value));
  assert(StairJson(loaded.front()) == StairJson(ReadStairAnnotations(root / "stairs" / "pois.tsv").front()));

  // Replace only this fixture's source with a ramp, then submit the exact same
  // client's recognized=true claim. The map service must derive a new result.
  points.clear();
  surface(-0.6,-0.025,0,0); surface(0,1.5,0,0.6); surface(1.525,2.1,0.9,0);
  assert(WriteBinaryXyzPcd(source,points,&error));
  assert(save(reloaded,"ramp").ok);
  loaded = ReadStairAnnotations(root / "stairs" / "pois.tsv");
  assert(loaded.size() == 2U);
  bool found_ramp = false;
  for (const auto& annotation : loaded) {
    if (annotation.id != "ramp") continue;
    found_ramp = true;
    assert(!annotation.geometry.recognized);
    assert(annotation.geometry.reason != "client claim");
    assert(annotation.geometry.surfaces.empty());
  }
  assert(found_ramp);
  assert(reloaded.Store().ContentEpoch("stairs") == epoch);
  assert(reloaded.ListPoiJson("stairs") == ordinary);
}

}  // namespace

void TestEditedSource(const std::filesystem::path &root) {
  using namespace lingtu::maps;
  using lingtu::maps::mapd::query::DispatchServiceJson;
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  assert(service.Store().CreateMap("original").ok);
  const auto dir = root / "original";
  std::string error;
  const std::vector<PointXyz> points{{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  assert(WriteBinaryXyzPcd(dir / "map.pcd", points, &error));
  std::filesystem::create_directory(dir / "patches");
  assert(WriteBinaryXyzPcd(dir / "patches/0.pcd", points, &error));
  { std::ofstream f(dir / "poses.txt"); f << "0.pcd 0 0 0 1 0 0 0\n"; }
  { std::ofstream f(dir / "scan_origin.txt"); f << "lidar_origin_in_patch 0 0 0\n"; }
  { std::ofstream f(dir / "patch_bundle.manifest"); f <<
      "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\nfirst_sequence 0\nlast_sequence 0\npatch_count 1\n"; }
  { std::ofstream f(dir / "metadata.json"); f <<
      R"({"frame_id":"floor_903","resolution":0.1,"source_profile":"ltlio","data_source":"field"})"; }
  { std::ofstream f(dir / "map_optimization.json"); f << "stale report"; }
  { std::ofstream f(dir / "octomap.ot"); f << "old derived artifact"; }
  const auto epoch = service.Store().ContentEpoch("original");
  const auto request = [&](const std::string &name, std::int64_t version, const std::string &edits) {
    return "{\"action\":\"save_edited_source\",\"map_id\":\"original\",\"new_map_id\":" +
        JsonString(name) + ",\"expected_source_content_epoch\":" + std::to_string(version) +
        ",\"operations\":" + edits + "}";
  };
  const std::string edits = R"([{"operation":"delete_inside","bounds":{"min_x":1,"max_x":1,"min_y":0,"max_y":0,"min_z":0,"max_z":0}},{"operation":"keep_inside","bounds":{"min_x":0,"max_x":2,"min_y":0,"max_y":0,"min_z":0,"max_z":0}}])";
  const auto preview_request = [&](std::int64_t version, const std::string &edits) {
    return "{\"action\":\"preview_source_edits\",\"map_id\":\"original\","
        "\"expected_source_content_epoch\":" + std::to_string(version) +
        ",\"operations\":" + edits + "}";
  };
  const std::string voxel_edits = R"([{"operation":"voxel","voxel_size":2}])";
  const auto preview = DispatchServiceJson(service, preview_request(epoch, voxel_edits));
  if (!preview.ok) std::cerr << preview.json << std::endl;
  assert(preview.ok);
  assert(JsonObjectNumberAtPath(preview.json, {"point_count"}) == 2.0);
  assert(JsonObjectNumberAtPath(preview.json, {"removed_points"}) == 2.0);
  assert(JsonObjectNumberAtPath(preview.json, {"source_content_epoch"}) == static_cast<double>(epoch));
  assert(JsonObjectStringAtPath(preview.json, {"frame_id"}) == "floor_903");
  assert(JsonObjectBoolAtPath(preview.json, {"preview_sample"}) == false);
  const auto preview_json = ParseJson(preview.json);
  const auto &preview_points = preview_json.AsObject("preview").at("preview_points").AsArray("points");
  const auto &preview_removed = preview_json.AsObject("preview").at("preview_removed").AsArray("removed");
  assert(preview_points.size() == 6U && preview_removed.size() == 6U);
  assert(std::get<double>(preview_points[3].value) == 2.0);
  assert(std::get<double>(preview_removed[0].value) == 1.0);
  assert(service.Store().ContentEpoch("original") == epoch);
  assert(LoadPcdXyz(dir / "map.pcd").points.size() == points.size());
  assert(!DispatchServiceJson(service, preview_request(epoch + 1, voxel_edits)).ok);
  const auto invalid_preview = DispatchServiceJson(service,
      preview_request(epoch, R"([{"operation":"radius","radius":0}])"));
  assert(!invalid_preview.ok);
  assert(JsonObjectStringAtPath(invalid_preview.json, {"reason_code"}) == "invalid_operations");
  const auto missing_bounds = DispatchServiceJson(service,
      preview_request(epoch, R"([{"operation":"keep_inside"}])"));
  assert(!missing_bounds.ok);
  assert(JsonObjectStringAtPath(missing_bounds.json, {"reason_code"}) == "invalid_operations");
  for (const std::string operation : {
           R"([{"operation":"statistical","neighbors":2,"std_ratio":1.5}])",
           R"([{"operation":"radius","radius":1.1,"min_points":2}])",
           R"([{"operation":"clusters","radius":1.1,"neighbors":2,"min_cluster_points":3}])"}) {
    const auto filtered = DispatchServiceJson(service, preview_request(epoch, operation));
    if (!filtered.ok) std::cerr << filtered.json << std::endl;
    assert(filtered.ok);
    assert(JsonObjectNumberAtPath(filtered.json, {"point_count"}) == 4.0);
  }
  const auto no_plane = DispatchServiceJson(service,
      preview_request(epoch, R"([{"operation":"plane_crop","distance_threshold":0.05,"margin":0.5}])"));
  assert(!no_plane.ok);
  assert(JsonObjectStringAtPath(no_plane.json, {"reason_code"}) == "edit_failed");
  const auto filtered_save = DispatchServiceJson(service, request("filtered", epoch, voxel_edits));
  assert(filtered_save.ok);
  const auto filtered_points = LoadPcdXyz(root / "filtered/map.pcd");
  assert(filtered_points.ok && filtered_points.points.size() == 2U);
  assert(filtered_points.points[0].x == 0 && filtered_points.points[1].x == 2);
  assert(HasSavedRays(root / "filtered"));
  const auto filtered_retry = DispatchServiceJson(service, request("filtered", epoch, voxel_edits));
  assert(filtered_retry.ok && JsonObjectBoolAtPath(filtered_retry.json, {"replayed"}) == true);
  const auto changed_parameters = DispatchServiceJson(service,
      request("filtered", epoch, R"([{"operation":"voxel","voxel_size":3}])"));
  assert(!changed_parameters.ok);
  assert(JsonObjectStringAtPath(changed_parameters.json, {"reason_code"}) == "target_exists");
  const auto copied = DispatchServiceJson(service, request("resolution_copy", epoch, "[]"));
  assert(copied.ok);
  assert(JsonObjectNumberAtPath(copied.json, {"removed_points"}) == 0.0);
  assert(LoadPcdXyz(root / "resolution_copy/map.pcd").points.size() == points.size());
  assert(HasSavedRays(root / "resolution_copy"));
  assert(!std::filesystem::exists(root / "resolution_copy/octomap.ot"));
  assert(DispatchServiceJson(service, request("resolution_copy", epoch, "[]")).ok);
  assert(!DispatchServiceJson(service, request("stale", epoch + 1, edits)).ok);
  assert(!std::filesystem::exists(root / "stale"));
  const auto result = DispatchServiceJson(service, request("edited", epoch, edits));
  if (!result.ok) std::cerr << result.json << std::endl;
  assert(result.ok);
  assert(JsonObjectBoolAtPath(result.json, {"activation_ready"}) == false);
  assert(JsonObjectBoolAtPath(result.json, {"saved_rays_preserved"}) == true);
  assert(JsonObjectStringAtPath(result.json, {"build_options", "frame_id"}) == "floor_903");
  assert(service.Store().ContentEpoch("original") == epoch);
  assert(LoadPcdXyz(dir / "map.pcd").points.size() == 4U);
  const auto edited = LoadPcdXyz(root / "edited/map.pcd");
  assert(edited.ok && edited.points.size() == 2U);
  assert(edited.points[0].x == 0.0F && edited.points[1].x == 2.0F);
  assert(HasSavedRays(root / "edited"));
  assert(!std::filesystem::exists(root / "edited/octomap.ot"));
  assert(!std::filesystem::exists(root / "edited/map_optimization.json"));
  const auto saved_epoch = service.Store().ContentEpoch("edited");
  const auto saved_write_time = std::filesystem::last_write_time(root / "edited/map.pcd");
  const auto repeated = DispatchServiceJson(service, request("edited", epoch, edits));
  assert(repeated.ok);
  assert(JsonObjectBoolAtPath(repeated.json, {"replayed"}) == true);
  assert(JsonObjectNumberAtPath(repeated.json, {"point_count"}) == 2.0);
  assert(JsonObjectNumberAtPath(repeated.json, {"removed_points"}) == 2.0);
  assert(service.Store().ContentEpoch("edited") == saved_epoch);
  assert(std::filesystem::last_write_time(root / "edited/map.pcd") == saved_write_time);
  assert(service.Store().ContentEpoch("original") == epoch);
  auto changed_edits = edits;
  changed_edits.replace(changed_edits.find("delete_inside"), 13, "keep_inside");
  assert(!DispatchServiceJson(service, request("edited", epoch, changed_edits)).ok);
  const std::string reversed_edits = R"([{"operation":"keep_inside","bounds":{"min_x":0,"max_x":2,"min_y":0,"max_y":0,"min_z":0,"max_z":0}},{"operation":"delete_inside","bounds":{"min_x":1,"max_x":1,"min_y":0,"max_y":0,"min_z":0,"max_z":0}}])";
  assert(!DispatchServiceJson(service, request("edited", epoch, reversed_edits)).ok);
  assert(!DispatchServiceJson(service, request("edited", epoch + 1, edits)).ok);
  assert(service.Store().CreateMap("unrelated").ok);
  const auto unrelated = DispatchServiceJson(service, request("unrelated", epoch, edits));
  assert(!unrelated.ok);
  assert(JsonObjectStringAtPath(unrelated.json, {"reason_code"}) == "target_exists");
  // The already committed snapshot can be confirmed even if its source moved on.
  { std::ofstream f(dir / MapStore::ContentEpochFilename()); f << epoch + 1 << "\n"; }
  assert(DispatchServiceJson(service, request("edited", epoch, edits)).ok);
  { std::ofstream f(dir / MapStore::ContentEpochFilename()); f << epoch << "\n"; }
  std::filesystem::rename(dir / "map.pcd", dir / "source_temporarily_unavailable.pcd");
  assert(DispatchServiceJson(service, request("edited", epoch, edits)).ok);
  std::filesystem::rename(dir / "source_temporarily_unavailable.pcd", dir / "map.pcd");
  assert(!DispatchServiceJson(service, request("original", epoch, edits)).ok);
  {
    const auto source_lock = MapLock::TryAcquire(root, "original", "test-editor-source");
    assert(source_lock.has_value());
    assert(!DispatchServiceJson(service, request("busy", epoch, edits)).ok);
  }
  const std::string all_points = R"([{"operation":"delete_inside","bounds":{"min_x":-1,"max_x":4,"min_y":-1,"max_y":1,"min_z":-1,"max_z":1}}])";
  const auto empty_preview = DispatchServiceJson(service, preview_request(epoch, all_points));
  assert(empty_preview.ok);
  assert(JsonObjectNumberAtPath(empty_preview.json, {"point_count"}) == 0.0);
  assert(JsonObjectNumberAtPath(empty_preview.json, {"removed_points"}) == 4.0);
  assert(!DispatchServiceJson(service, request("empty", epoch, all_points)).ok);
  assert(!std::filesystem::exists(root / "empty"));
#if defined(_WIN32)
  _putenv_s("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
#else
  setenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1", 1);
#endif
  const auto failed = DispatchServiceJson(service, request("failed", epoch, edits));
#if defined(_WIN32)
  _putenv_s("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "");
#else
  unsetenv("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");
#endif
  assert(!failed.ok);
  assert(!std::filesystem::exists(root / "failed"));
  assert(service.Store().ContentEpoch("original") == epoch);
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  const auto rebuilt = DispatchServiceJson(service,
      R"({"action":"build_octomap_artifact","map_id":"edited"})");
  if (!rebuilt.ok) std::cerr << rebuilt.json << std::endl;
  assert(rebuilt.ok);
  std::ifstream metadata_file(root / "edited/metadata.json");
  const std::string metadata{std::istreambuf_iterator<char>(metadata_file), {}};
  assert(JsonObjectStringAtPath(metadata, {"frame_id"}) == "floor_903");
  assert(JsonObjectNumberAtPath(metadata, {"resolution"}) == 0.1);
  assert(JsonObjectStringAtPath(metadata, {"artifacts", "octomap", "evidence_source"}) == "saved_rays");
  assert(!std::filesystem::exists(root / "edited/map_optimization.json"));
  assert(!DispatchServiceJson(service, request("edited", epoch, edits)).ok);
#endif
  // A source without a complete measured bundle remains limited after editing.
  std::filesystem::remove(dir / "patch_bundle.manifest");
  const auto limited = DispatchServiceJson(service, request("limited", epoch, edits));
  assert(limited.ok);
  assert(JsonObjectBoolAtPath(limited.json, {"saved_rays_preserved"}) == false);
  assert(!std::filesystem::exists(root / "limited/poses.txt"));
}

int main() {
#if defined(_MSC_VER)
  _set_error_mode(_OUT_TO_STDERR);
  _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
  const auto root = TempRoot();
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  const auto surface_root = TempRoot();
  TestSurfacePreview(surface_root);
  std::filesystem::remove_all(surface_root);
#endif
  lingtu::maps::MapsServiceCore service(
      lingtu::maps::MapsServiceConfig{lingtu::maps::MapStoreConfig{root}});
  using lingtu::maps::mapd::query::DispatchServiceJson;

  auto listed = DispatchServiceJson(service, R"({"action":"list_maps"})");
  assert(listed.ok);
  assert(lingtu::maps::JsonObjectBoolAtPath(listed.json, {"success"}) == true);

  auto created = DispatchServiceJson(service, R"({"action":"create_map","map_id":"dispatch_map"})");
  assert(created.ok);
  assert(created.json.find("\"map_dir\"") == std::string::npos);
  const auto no_go_epoch = service.Store().ContentEpoch("dispatch_map");
  const auto no_go_set = DispatchServiceJson(service,
      "{\"action\":\"set_no_go_region\",\"map_id\":\"dispatch_map\",\"region_id\":\"door\","
      "\"expected_content_epoch\":" + std::to_string(no_go_epoch) +
      ",\"min_x\":0,\"min_y\":1,\"min_z\":2,\"max_x\":1,\"max_y\":2,\"max_z\":3}");
  assert(no_go_set.ok);
  assert(DispatchServiceJson(service,
      R"({"action":"get_no_go_regions","map_id":"dispatch_map"})").ok);
  const auto no_go_delete = DispatchServiceJson(service,
      "{\"action\":\"delete_no_go_region\",\"map_id\":\"dispatch_map\",\"region_id\":\"door\","
      "\"expected_content_epoch\":" + std::to_string(service.Store().ContentEpoch("dispatch_map")) + "}");
  assert(no_go_delete.ok);
  assert(!DispatchServiceJson(service,
      R"({"action":"set_no_go_region","map_id":"dispatch_map","region_id":"bad"})").ok);
  auto relisted = DispatchServiceJson(service, R"({"action":"list_maps"})");
  assert(relisted.ok);
  assert(relisted.json.find("\"map_dir\"") == std::string::npos);
  const auto activation = service.Store().CheckMapActivation("dispatch_map");
  const auto parsed_relisted = lingtu::maps::ParseJson(relisted.json);
  const auto& listed_maps =
      parsed_relisted.AsObject("list response").at("maps").AsArray("maps");
  assert(listed_maps.size() == 1U);
  const auto& listed_blockers = listed_maps.front()
                                    .AsObject("listed map")
                                    .at("activation_blockers")
                                    .AsArray("activation blockers");
  assert(listed_blockers.size() == activation.blockers.size());
  for (std::size_t i = 0; i < activation.blockers.size(); ++i) {
    assert(listed_blockers[i].AsString("activation blocker") == activation.blockers[i]);
  }
  auto record = DispatchServiceJson(service, R"({"action":"get_record","map_id":"dispatch_map"})");
  assert(record.ok);
  assert(record.json.find("dispatch_map") != std::string::npos);
  assert(record.json.find("\"map_dir\"") == std::string::npos);
  assert(service.Store().SetActiveMap("dispatch_map", false).ok);
  for (const std::string request : {
           R"({"action":"delete_map","map_id":"dispatch_map"})",
           R"({"action":"rename_map","map_id":"dispatch_map","new_map_id":"renamed"})",
           R"({"action":"retire_map","map_id":"dispatch_map"})",
       }) {
    const auto conflict = DispatchServiceJson(service, request);
    assert(!conflict.ok);
    assert(lingtu::maps::JsonObjectStringAtPath(conflict.json, {"reason_code"}) ==
           "active_map_conflict");
    assert(service.Store().ActiveMapId() == "dispatch_map");
  }
  assert(service.Store().ClearActiveMap("dispatch_map").ok);
  {
    std::ofstream active_state(root / "active_map.txt", std::ios::binary | std::ios::trunc);
    active_state << "../corrupt\n";
  }
  for (const std::string request : {
           R"({"action":"delete_map","map_id":"dispatch_map"})",
           R"({"action":"rename_map","map_id":"dispatch_map","new_map_id":"renamed"})",
           R"({"action":"retire_map","map_id":"dispatch_map"})",
       }) {
    const auto invalid = DispatchServiceJson(service, request);
    assert(!invalid.ok);
    assert(lingtu::maps::JsonObjectStringAtPath(invalid.json, {"reason_code"}) ==
           "active_map_state_invalid");
  }
  {
    std::ofstream active_state(root / "active_map.txt", std::ios::binary | std::ios::trunc);
    active_state << "\n";
  }
  auto deleted = DispatchServiceJson(service, R"({"action":"delete_map","map_id":"dispatch_map"})");
  assert(deleted.ok);

  auto save_without_coordinator = DispatchServiceJson(
      service, R"({"action":"save_map","request_id":"dispatch-save","map_id":"saved_map"})");
  assert(!save_without_coordinator.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(save_without_coordinator.json, {"reason_code"}) ==
         "internal_error");
  auto legacy_begin = DispatchServiceJson(
      service, R"({"action":"begin_save_map","request_id":"legacy","map_id":"saved_map"})");
  assert(!legacy_begin.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(legacy_begin.json, {"reason_code"}) ==
         "unknown_action");

  const auto reoptimize_missing = DispatchServiceJson(
      service, R"({"action":"reoptimize_save_map","job_id":"missing","request_id":"reoptimize"})");
  assert(lingtu::maps::JsonObjectStringAtPath(reoptimize_missing.json, {"reason_code"}) ==
         "job_not_found");
  for (const std::string request : {
           R"({"action":"reoptimize_save_map","job_id":"missing"})",
           R"({"action":"reoptimize_save_map","request_id":"reoptimize"})",
           R"({"action":"reoptimize_save_map","job_id":"missing","request_id":12})",
       }) {
    const auto invalid = DispatchServiceJson(service, request);
    assert(lingtu::maps::JsonObjectStringAtPath(invalid.json, {"reason_code"}) == "invalid_request");
  }

  auto natural_u64 = DispatchServiceJson(service, R"({"action":"get_map_points","max_points":1})");
  assert(lingtu::maps::JsonObjectStringAtPath(natural_u64.json, {"reason_code"}) !=
         "invalid_request");
  auto compatible_string_u64 =
      DispatchServiceJson(service, R"({"action":"get_map_points","max_points":"1"})");
  assert(lingtu::maps::JsonObjectStringAtPath(compatible_string_u64.json, {"reason_code"}) !=
         "invalid_request");
  for (const std::string invalid : {
           R"({"action":"get_map_points","max_points":-1})",
           R"({"action":"get_map_points","max_points":1.5})",
           R"({"action":"get_map_points","max_points":18446744073709551616})",
       }) {
    const auto result = DispatchServiceJson(service, invalid);
    assert(!result.ok);
    assert(lingtu::maps::JsonObjectStringAtPath(result.json, {"reason_code"}) == "invalid_request");
  }
  for (const std::string action : {"set_active_map", "clear_active_map"}) {
    const auto result =
        DispatchServiceJson(service, "{\"action\":\"" + action + "\",\"map_id\":\"dispatch_map\"}");
    assert(!result.ok);
    assert(lingtu::maps::JsonObjectStringAtPath(result.json, {"reason_code"}) == "unknown_action");
  }
  auto missing = DispatchServiceJson(service, R"({"action":"create_map"})");
  assert(!missing.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(missing.json, {"reason_code"}) == "invalid_request");
  auto unknown = DispatchServiceJson(service, R"({"action":"not_an_action"})");
  assert(!unknown.ok);
  assert(lingtu::maps::JsonObjectStringAtPath(unknown.json, {"reason_code"}) == "unknown_action");

  TestEditedSource(root / "edited-source");
  TestStairAnnotations(root / "stair-annotations");
  TestStairRecognitionPersistence(root / "stair-recognition");

  // Match the saved-map preview size requested by the Web client.
  const auto source = root / "preview.pcd";
  {
    std::ofstream points(source);
    points << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              "WIDTH 80000\nHEIGHT 1\nPOINTS 80000\nDATA ascii\n";
    for (int i = 0; i < 80000; ++i) {
      points << "12.34567 -9.876543 0.456789\n";
    }
  }
  const auto imported = service.ImportPcdJson("preview", source, 0.0, {});
  assert(lingtu::maps::JsonObjectBoolAtPath(imported, {"success"}) == true);
  const auto preview_metadata = service.Store().ContentPath("preview") / "metadata.json";
  const auto check_preview_stamp = [&service](const std::string& expected) {
    const auto result = DispatchServiceJson(
        service, R"({"action":"get_map_points","map_id":"preview","max_points":1})");
    assert(result.ok);
    assert(lingtu::maps::IsValidJsonObject(result.json));
    assert(result.json.find("\"built_at_ms\":" + expected + ",") != std::string::npos);
    assert(lingtu::maps::JsonObjectNumberAtPath(result.json, {"returned"}) == 1);
  };
  std::filesystem::remove(preview_metadata);
  check_preview_stamp("null");
  for (const auto* metadata : {
           R"({})", R"({"created_at":1760000000000})", R"({"created_at":"0"})",
           R"({"created_at":"-1"})", R"({"created_at":"1760000000000ms"})",
           R"({"created_at":"99999999999999999999999999999"})", "{broken"}) {
    { std::ofstream file(preview_metadata); file << metadata; }
    check_preview_stamp("null");
  }
  { std::ofstream file(preview_metadata); file << R"({"created_at":"1760000000000"})"; }
  check_preview_stamp("1760000000000");
  { std::ofstream file(preview_metadata); file << R"({"created_at":"1760000001234"})"; }
  check_preview_stamp("1760000001234");
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  const auto build_source = root / "build-defaults.pcd";
  {
    std::ofstream points(build_source);
    points << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              "WIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA ascii\n"
              "0 0 0\n1 0 0\n0 1 0\n1 1 0\n";
  }
  const auto build_imported = service.ImportPcdJson("build_defaults", build_source, 0.0, {});
  assert(lingtu::maps::JsonObjectBoolAtPath(build_imported, {"success"}) == true);
  const auto default_package = DispatchServiceJson(
      service, R"({"action":"build_navigation_package","map_id":"build_defaults"})");
  assert(default_package.ok);
  assert(lingtu::maps::JsonObjectBoolAtPath(default_package.json, {"success"}) == true);
  const auto built_map = service.Store().MapPath("build_defaults");
  assert(!std::filesystem::exists(built_map / "occupancy.npz"));
  assert(lingtu::maps::JsonObjectBoolAtPath(
             default_package.json, {"steps", "occupancy", "requested"}) == false);
  assert(std::filesystem::is_regular_file(built_map / "octomap.ot"));
  assert(!std::filesystem::exists(built_map / "esdf.npz"));
  assert(!std::filesystem::exists(built_map / "traversability.npz"));
  assert(lingtu::maps::JsonObjectBoolAtPath(
             default_package.json, {"steps", "esdf", "requested"}) == false);
  assert(lingtu::maps::JsonObjectBoolAtPath(
             default_package.json, {"steps", "traversability", "requested"}) == false);

  const auto explicit_occupancy = DispatchServiceJson(
      service, R"({"action":"build_occupancy_snapshot","map_id":"build_defaults"})");
  assert(explicit_occupancy.ok);
  assert(std::filesystem::is_regular_file(built_map / "occupancy.npz"));
  const auto explicit_esdf = DispatchServiceJson(
      service, R"({"action":"build_esdf_artifact","map_id":"build_defaults"})");
  assert(explicit_esdf.ok);
  assert(std::filesystem::is_regular_file(built_map / "esdf.npz"));
  const auto explicit_traversability = DispatchServiceJson(
      service, R"({"action":"build_traversability_artifact","map_id":"build_defaults"})");
  assert(explicit_traversability.ok);
  assert(std::filesystem::is_regular_file(built_map / "traversability.npz"));
#endif
  const auto preview = DispatchServiceJson(
      service, R"({"action":"get_map_points","map_id":"preview","max_points":80000})");
  assert(preview.ok);
  assert(preview.json.find("\"returned\":80000") != std::string::npos);
  std::cout << "80k preview bytes: " << preview.json.size() << std::endl;
  const bool preview_fits = preview.json.size() <= lingtu::maps::mapd::query::kDefaultMaxJsonBytes;
  std::filesystem::remove_all(root);
  return preview_fits ? 0 : 1;
}
