#include "cleaner.hpp"
#include "core/io.hpp"
#include "core/evidence.hpp"
#include "core/visibility.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace lingtu::map_cleaning;

namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::string Read(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

struct Fixture {
  fs::path root = fs::temp_directory_path() / ("lingtu_prune_test_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  StaticCleanerOptions options;
  Fixture() {
    fs::create_directories(root / "patches");
    options.map_dir = root;
    const PointXYZI ground{.1F, .1F, -.6F};
    const PointXYZI repeated{1.1F, .1F, .2F};
    const PointXYZI dense{2.1F, .1F, .2F};
    const PointXYZI sparse{3.1F, .1F, .2F};
    const PointXYZI unmatched{4.1F, .1F, .2F};
    const PointXYZI ghost{.9F, 1.1F, .3F};
    writePcd(root / "map.pcd", {ground, repeated, dense, sparse, unmatched, ghost});
    writePcd(root / "patches/a.pcd", {ground, repeated, dense, dense, dense, sparse, ghost, ghost, ghost});
    writePcd(root / "patches/b.pcd", {repeated, ghost});
    for (const auto* name : {"c.pcd", "d.pcd", "e.pcd"})
      writePcd(root / "patches" / name, {{1.8F, 2.2F, .6F}});
    writePcd(root / "patches/unmatched.pcd", {unmatched});
    std::ofstream(root / "poses.txt") << "a.pcd 0 0 0 1 0 0 0\nb.pcd 0 0 0 1 0 0 0\n"
        << "c.pcd 0 0 0 1 0 0 0\nd.pcd 0 0 0 1 0 0 0\ne.pcd 0 0 0 1 0 0 0\n";
    std::ofstream(root / "scan_origin.txt") << "lidar_origin_in_patch 0 0 0\n";
  }
  ~Fixture() { std::error_code ec; fs::remove_all(root, ec); }
};

void DecisionReport() {
  Fixture f;
  const auto original = Read(f.root / "map.pcd");
  const auto r = cleanStaticMap(f.options);
  Require(r.success && !r.applied && !r.dry_run, "preview should write comparison outputs only");
  Require(r.source_points == 6 && r.kept_points == 5 && r.removed_points == 1, "wrong point split");
  Require(r.kept_ground_points == 1 && r.kept_multi_frame_points == 1 &&
      r.kept_hit_support_points == 1 && r.kept_without_evidence_points == 1 && r.kept_unconfirmed_points == 1,
      "decision counts must partition the kept points");
  Require(r.patch_count == 5 && r.unmatched_patch_count == 1, "missing trajectory coverage report");
  Require(Read(f.root / "map.pcd") == original, "preview changed source");
  Require(readPcd(r.removed_pcd).front().x == .9F, "sparse surface removed instead of multi-frame ghost");
  Require(toJson(r).find("multi_frame_observed_free_space_not_semantic_motion") != std::string::npos,
      "report must not claim confirmed moving objects");
}

void DryRun() {
  Fixture f;
  f.options.dry_run = true;
  f.options.output_clean_pcd = f.root / "preview/clean.pcd";
  const auto original = Read(f.root / "map.pcd");
  auto r = cleanStaticMap(f.options);
  Require(r.success && r.dry_run && !r.applied && r.removed_points == 1, "dry-run analysis failed");
  Require(r.clean_pcd.empty() && r.removed_pcd.empty() && r.backup_pcd.empty(), "dry-run claims artifacts");
  Require(!fs::exists(f.root / "preview") && !fs::exists(f.root / "map.removed.pcd") &&
      !fs::exists(f.root / "map.pcd.preclean"), "dry-run wrote files or directories");
  Require(Read(f.root / "map.pcd") == original, "dry-run changed source");
  f.options.apply_to_map = true;
  r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "conflicting_mode", "ambiguous mode must fail");
}

void EmptyApply() {
  Fixture f;
  writePcd(f.root / "map.pcd", {{.9F, 1.1F, .3F}});
  const auto original = Read(f.root / "map.pcd");
  f.options.apply_to_map = true;
  const auto r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "empty_clean_map" && !r.applied, "empty result was applied");
  Require(r.removed_points == 1 && r.kept_points == 0, "rejected apply lost analysis counts");
  Require(Read(f.root / "map.pcd") == original && !fs::exists(f.root / "map.pcd.preclean") &&
      !fs::exists(f.root / "map.clean.pcd"), "empty apply changed artifacts");
}

void RepeatApply() {
  Fixture f;
  const auto original = Read(f.root / "map.pcd");
  f.options.apply_to_map = true;
  f.options.overwrite = true;
  const auto first = cleanStaticMap(f.options);
  Require(first.success && first.applied && readPcd(f.root / "map.pcd").size() == 5, "first apply failed");
  const auto second = cleanStaticMap(f.options);
  Require(second.success && second.applied, "repeated apply failed");
  Require(Read(f.root / "map.pcd.preclean") == original, "repeated apply replaced the original backup");
}

void OutputAliases() {
  Fixture f;
  f.options.overwrite = true;
  const auto original = Read(f.root / "map.pcd");
  for (const auto& path : {f.root / "map.pcd", f.root / "patches/../map.pcd",
      f.root / "poses.txt", f.root / "patches/a.pcd", f.root / "map.pcd.preclean",
      f.root / "map.pcd.tmpclean", f.root / "map.removed.pcd"}) {
    f.options.output_clean_pcd = path;
    const auto r = cleanStaticMap(f.options);
    Require(!r.success && r.reason_code == "bad_output_path", "unsafe output alias accepted");
  }
  Require(Read(f.root / "map.pcd") == original, "output alias damaged source");
}

void TruncatedSource() {
  Fixture f;
  fs::resize_file(f.root / "map.pcd", fs::file_size(f.root / "map.pcd") - 8);
  const auto original = Read(f.root / "map.pcd");
  f.options.apply_to_map = true;
  const auto r = cleanStaticMap(f.options);
  Require(!r.success && r.message.find("truncated PCD") != std::string::npos, "partial input accepted");
  Require(Read(f.root / "map.pcd") == original && !fs::exists(f.root / "map.pcd.preclean"),
      "truncated input was applied");
  std::ofstream(f.root / "map.pcd")
      << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 0 0\n";
  const auto ascii = Read(f.root / "map.pcd");
  const auto ascii_result = cleanStaticMap(f.options);
  Require(!ascii_result.success && Read(f.root / "map.pcd") == ascii &&
              !fs::exists(f.root / "map.pcd.preclean"),
          "truncated ASCII source was applied");
}

void BadThreshold() {
  Fixture f;
  f.options.voxel_size_m = std::numeric_limits<float>::quiet_NaN();
  const auto r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "bad_voxel_size", "NaN voxel size reached voxel indexing");
}

void VisibilityRules() {
  StaticCleanerOptions options;
  options.sensor_origin = std::array<float, 3>{0, 0, 0};
  const std::vector<PointXYZI> map{{2, 0, 0}, {5, 1, 0}};
  VisibilityEvidence v(map, options);
  v.observe({{2, 0, 0}}, {});
  v.observe({{4, 0, 0}, {4, 0, 0}, {4, 0, 0}}, {});
  Require(!v.contradicted(0), "multiple rays in one scan counted as multiple frames");
  v.observe({{4, 0, 0}}, {});
  Require(!v.contradicted(0), "two free frames should not meet the default three");
  v.observe({{4, 0, 0}}, {});
  Require(v.contradicted(0) && !v.contradicted(1), "free ray or unseen wall handling failed");
  v.observe({{4, 0, 0}, {2, .1F, 0}}, {});
  Require(!v.contradicted(0), "same-frame nearby surface did not override free ray");
  for (int i = 0; i < 3; ++i) { v.observe({{1, 0, 0}}, {}); v.observe({}, {}); }
  Require(!v.contradicted(0), "occlusion or absent returns were counted as free");
  VisibilityEvidence new_object(map, options);
  for (int i = 0; i < 3; ++i) new_object.observe({{4, 0, 0}}, {});
  new_object.observe({{2, 0, 0}}, {});
  Require(!new_object.contradicted(0), "earlier free space deleted a later object");
}

void CalibratedOrigin() {
  StaticCleanerOptions options;
  options.sensor_origin = std::array<float, 3>{1, 0, 0};
  Pose pose{10, 3, 1, .7071067811865476, 0, 0, .7071067811865476};
  const PointXYZI point{2, 1, 0};
  const std::vector<PointXYZI> map{transformPoint(point, pose)};
  VisibilityEvidence calibrated(map, options);
  calibrated.observe({point}, pose);
  for (int i = 0; i < 3; ++i) calibrated.observe({{3, 2, 0}}, pose);
  Require(calibrated.contradicted(0), "ray origin and map pose were not composed correctly");
  auto wrong = options;
  wrong.sensor_origin = std::array<float, 3>{0, 0, 0};
  VisibilityEvidence body_center(map, wrong);
  body_center.observe({point}, pose);
  for (int i = 0; i < 3; ++i) body_center.observe({{3, 2, 0}}, pose);
  Require(!body_center.contradicted(0), "test did not distinguish sensor and body origin");
}

void GroundProtection() {
  Fixture f;
  const PointXYZI ground{1, 0, -.6F};
  writePcd(f.root / "map.pcd", {ground});
  writePcd(f.root / "patches/a.pcd", {ground});
  writePcd(f.root / "patches/b.pcd", {ground});
  for (const auto* name : {"c.pcd", "d.pcd", "e.pcd"})
    writePcd(f.root / "patches" / name, {{2, 0, -1.2F}});
  f.options.dry_run = true;
  const auto r = cleanStaticMap(f.options);
  Require(r.success && r.free_space_candidate_points == 1 && r.kept_ground_points == 1 &&
      r.removed_points == 0, "low ground safeguard was overridden by free rays");
}

void PoseOrder() {
  Fixture f;
  // The object appeared after the free observations, despite its earlier filename.
  std::ofstream(f.root / "poses.txt") << "c.pcd 0 0 0 1 0 0 0\nd.pcd 0 0 0 1 0 0 0\n"
      << "e.pcd 0 0 0 1 0 0 0\na.pcd 0 0 0 1 0 0 0\nb.pcd 0 0 0 1 0 0 0\n";
  f.options.dry_run = true;
  const auto r = cleanStaticMap(f.options);
  Require(r.success && r.removed_points == 0, "filename sorting overrode recorded pose order");
}

void SurfaceProtection() {
  for (int orientation = 0; orientation < 4; ++orientation) {
    Fixture f;
    const PointXYZI center{2, 0, -.30F};
    std::vector<PointXYZI> surface;
    for (int u = -1; u <= 1; ++u)
      for (int v = -1; v <= 1; ++v) {
        if (u == 0 && v == 0) continue;
        const float a = .30F * u, b = .30F * v;
        if (orientation == 0) surface.push_back({center.x + a, b, center.z});
        else if (orientation == 2) surface.push_back({center.x + a, b, center.z + a * .4F});
        else surface.push_back({center.x + (orientation == 3 ? .30F : 0), a, center.z + b});
      }
    auto initial = surface;
    initial.push_back(center);
    writePcd(f.root / "map.pcd", initial);
    writePcd(f.root / "patches/a.pcd", initial);
    writePcd(f.root / "patches/b.pcd", initial);
    auto later = surface;
    later.push_back({4, 0, -.60F});
    for (const auto* name : {"c.pcd", "d.pcd", "e.pcd"})
      writePcd(f.root / "patches" / name, later);
    f.options.dry_run = true;
    const auto result = cleanStaticMap(f.options);
    Require(result.success && result.free_space_candidate_points > 0, "surface fixture missed free ray");
    if (orientation == 3)
      Require(result.removed_points == 1 && result.kept_supported_surface_points == 0,
          "wall protected a detached ghost 30 cm in front of it");
    else
      Require(result.removed_points == 0 && result.kept_supported_surface_points == 1,
          "persistent floor, wall or ramp plane was erased by grazing rays");
  }
}

void MissingOrigin() {
  Fixture f;
  std::ofstream(f.root / "scan_origin.txt") << "";
  Require(cleanStaticMap(f.options).reason_code == "missing_sensor_origin", "missing calibration silently guessed");
  f.options.sensor_origin = std::array<float, 3>{0, 0, 0};
  f.options.dry_run = true;
  Require(cleanStaticMap(f.options).success, "explicit origin cannot process older maps");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string name = argc > 1 ? argv[1] : "";
    if (name == "report") DecisionReport();
    else if (name == "dry_run") DryRun();
    else if (name == "empty_apply") EmptyApply();
    else if (name == "repeat_apply") RepeatApply();
    else if (name == "output_aliases") OutputAliases();
    else if (name == "truncated_source") TruncatedSource();
    else if (name == "bad_threshold") BadThreshold();
    else if (name == "visibility") VisibilityRules();
    else if (name == "calibrated_origin") CalibratedOrigin();
    else if (name == "ground") GroundProtection();
    else if (name == "pose_order") PoseOrder();
    else if (name == "missing_origin") MissingOrigin();
    else if (name == "surface") SurfaceProtection();
    else throw std::runtime_error("unknown test case");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
