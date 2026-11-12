#include "cleaner.hpp"
#include "core/io.hpp"
#include "core/evidence.hpp"

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
    const PointXYZI ghost{2.025F, .025F, .025F};
    const PointXYZI wall{4, 0, 0};
    const PointXYZI unseen{10, 10, 10};
    writePcd(root / "map.pcd", {ghost, wall, unseen});
    writePcd(root / "patches/a.pcd", {ghost, wall});
    std::vector<PointXYZI> plane;
    for (int y=-20;y<=20;++y)
      for (int z=-20;z<=20;++z) plane.push_back({4, .05F*y, .05F*z});
    for (const auto* name : {"b.pcd", "c.pcd", "d.pcd", "e.pcd"})
      writePcd(root / "patches" / name, plane);
    writePcd(root / "patches/unmatched.pcd", {unseen});
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
  Require(r.source_points == 3 && r.kept_points == 2 && r.removed_points == 1, "wrong point split");
  Require(r.source_points == r.kept_points+r.removed_points, "point partition lost data");
  Require(r.patch_count == 5 && r.unmatched_patch_count == 1, "missing trajectory coverage report");
  Require(Read(f.root / "map.pcd") == original, "preview changed source");
  Require(readPcd(r.removed_pcd).front().x == 2.025F, "removed point is not the observed ghost");
  Require(toJson(r).find("dufomap") != std::string::npos,
      "report must identify the DUFOMap backend");
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
      !fs::exists(f.root / "map.before_clean.pcd") && !fs::exists(f.root / "map.pcd.preclean"),
      "dry-run wrote files or directories");
  Require(Read(f.root / "map.pcd") == original, "dry-run changed source");
  f.options.apply_to_map = true;
  r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "conflicting_mode", "ambiguous mode must fail");
}

void EmptyApply() {
  Fixture f;
  writePcd(f.root / "map.pcd", {{2.025F, .025F, .025F}});
  const auto original = Read(f.root / "map.pcd");
  f.options.apply_to_map = true;
  const auto r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "empty_clean_map" && !r.applied, "empty result was applied");
  Require(r.removed_points == 1 && r.kept_points == 0, "rejected apply lost analysis counts");
  Require(Read(f.root / "map.pcd") == original && !fs::exists(f.root / "map.before_clean.pcd") &&
      !fs::exists(f.root / "map.pcd.preclean") &&
      !fs::exists(f.root / "map.clean.pcd"), "empty apply changed artifacts");
}

void RepeatApply() {
  for (const bool legacy : {false, true}) {
    Fixture f;
    const auto original = Read(f.root / "map.pcd");
    const auto canonical_backup = f.root / "map.before_clean.pcd";
    const auto legacy_backup = f.root / "map.pcd.preclean";
    f.options.apply_to_map = true;
    f.options.overwrite = true;
    const auto first = cleanStaticMap(f.options);
    Require(first.success && first.applied && readPcd(f.root / "map.pcd").size() == 2,
        "first apply failed");
    Require(first.backup_pcd == canonical_backup && Read(canonical_backup) == original &&
        !fs::exists(legacy_backup), "first apply did not create the canonical backup");
    if (legacy) fs::rename(canonical_backup, legacy_backup);
    else writePcd(legacy_backup, {{9, 9, 9}});
    const auto legacy_contents = Read(legacy_backup);
    const auto second = cleanStaticMap(f.options);
    const auto expected_backup = legacy ? legacy_backup : canonical_backup;
    Require(second.success && second.applied && second.backup_pcd == expected_backup,
        "repeated apply selected the wrong backup");
    Require(Read(expected_backup) == original && Read(legacy_backup) == legacy_contents,
        "repeated apply replaced an existing backup");
    Require(!legacy || !fs::exists(canonical_backup),
        "legacy re-cleaning backed up the already cleaned map");
  }
}

void OutputAliases() {
  Fixture f;
  f.options.overwrite = true;
  const auto original = Read(f.root / "map.pcd");
  for (const auto& path : {f.root / "map.pcd", f.root / "patches/../map.pcd",
      f.root / "poses.txt", f.root / "patches/a.pcd", f.root / "map.before_clean.pcd",
      f.root / "map.pcd.preclean",
      f.root / "map.pcd.tmpclean", f.root / "map.removed.pcd"}) {
    f.options.output_clean_pcd = path;
    const auto r = cleanStaticMap(f.options);
    Require(!r.success && r.reason_code == "bad_output_path", "unsafe output alias accepted");
  }
  f.options.output_clean_pcd.clear();
  for (const auto* filename : {"map.before_clean.pcd", "map.pcd.preclean"}) {
    f.options.output_removed_pcd = f.root / filename;
    const auto r = cleanStaticMap(f.options);
    Require(!r.success && r.reason_code == "bad_output_path", "removed output overwrote backup");
  }
  Require(Read(f.root / "map.pcd") == original, "output alias damaged source");
}

void TruncatedSource() {
  Fixture f;
  fs::resize_file(f.root / "map.pcd", fs::file_size(f.root / "map.pcd") - 8);
  const auto original = Read(f.root / "map.pcd");
  f.options.apply_to_map = true;
  const auto r = cleanStaticMap(f.options);
  Require(!r.success && r.message.find("truncated") != std::string::npos, "partial input accepted");
  Require(Read(f.root / "map.pcd") == original && !fs::exists(f.root / "map.before_clean.pcd") &&
      !fs::exists(f.root / "map.pcd.preclean"),
      "truncated input was applied");
  std::ofstream(f.root / "map.pcd")
      << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 0 0\n";
  const auto ascii = Read(f.root / "map.pcd");
  const auto ascii_result = cleanStaticMap(f.options);
  Require(!ascii_result.success && Read(f.root / "map.pcd") == ascii &&
              !fs::exists(f.root / "map.before_clean.pcd") && !fs::exists(f.root / "map.pcd.preclean"),
          "truncated ASCII source was applied");
}

void BadThreshold() {
  Fixture f;
  f.options.voxel_size_m = std::numeric_limits<float>::quiet_NaN();
  auto r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "bad_voxel_size", "NaN voxel size reached voxel indexing");
  f.options.voxel_size_m = 0;
  r = cleanStaticMap(f.options);
  Require(!r.success && r.reason_code == "bad_voxel_size", "zero resolution accepted");
}

void CalibratedOrigin() {
  Fixture f;
  f.options.dry_run=true;
  f.options.sensor_origin=std::array<float,3>{3,0,0};
  const auto calibrated=cleanStaticMap(f.options);
  Require(calibrated.success && calibrated.removed_points==0,
      "rays from calibrated origin incorrectly crossed the point behind sensor");
  f.options.sensor_origin=std::array<float,3>{0,0,0};
  const auto wrong=cleanStaticMap(f.options);
  Require(wrong.success && wrong.removed_points==1, "fixture does not distinguish sensor origins");
}

void MissingOrigin() {
  Fixture f;
  fs::remove(f.root/"scan_origin.txt");
  f.options.dry_run=true;
  auto result=cleanStaticMap(f.options);
  Require(!result.success && result.reason_code=="missing_sensor_origin", "missing calibration silently assumed zero");
  f.options.sensor_origin=std::array<float,3>{0,0,0};
  result=cleanStaticMap(f.options);
  Require(result.success, "explicit calibrated origin rejected");
}

void InvalidScan() {
  Fixture f;
  f.options.dry_run=true;
  std::ofstream(f.root/"poses.txt") << "a.pcd 0 0 0 0 0 0 0\n";
  auto result=cleanStaticMap(f.options);
  Require(!result.success && result.reason_code=="invalid_scan_pose", "zero quaternion accepted");
  std::ofstream(f.root/"poses.txt") << "a.pcd 0 0 0 1 0 0 0\n";
  writePcd(f.root/"patches/a.pcd", {{std::numeric_limits<float>::quiet_NaN(),0,0}});
  result=cleanStaticMap(f.options);
  Require(!result.success && result.reason_code=="invalid_scan_point", "nonfinite scan accepted");
  writePcd(f.root/"patches/a.pcd", {});
  result=cleanStaticMap(f.options);
  Require(!result.success && result.reason_code=="no_matched_patch_evidence", "empty scan accepted");
}

void Transform() {
  Fixture f;
  f.options.dry_run=true;
  const auto base=cleanStaticMap(f.options);
  const Pose pose{10,5,1,.7071067811865476,0,0,.7071067811865476};
  auto points=readPcd(f.root/"map.pcd");
  for(auto& p:points)p=transformPoint(p,pose);
  writePcd(f.root/"map.pcd",points);
  std::ofstream poses(f.root/"poses.txt");
  for(const auto* name:{"a","b","c","d","e"})
    poses<<name<<".pcd 10 5 1 0.7071067811865476 0 0 0.7071067811865476\n";
  poses.close();
  const auto rotated=cleanStaticMap(f.options);
  Require(base.success && rotated.success && base.removed_points==rotated.removed_points,
      "pose transform changed classified geometry");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const std::string name=argc>1?argv[1]:"";
    if(name=="report") DecisionReport();
    else if(name=="dry_run") DryRun();
    else if(name=="empty_apply") EmptyApply();
    else if(name=="repeat_apply") RepeatApply();
    else if(name=="output_aliases") OutputAliases();
    else if(name=="truncated_source") TruncatedSource();
    else if(name=="bad_threshold") BadThreshold();
    else if(name=="calibrated_origin") CalibratedOrigin();
    else if(name=="missing_origin") MissingOrigin();
    else if(name=="invalid_scan") InvalidScan();
    else if(name=="transform") Transform();
    else throw std::runtime_error("unknown test case");
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
