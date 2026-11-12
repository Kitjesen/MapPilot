#include "core/io.hpp"
#include "core/save.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using namespace lingtu::map_cleaning;

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

std::string contents(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

struct Fixture {
  fs::path root = fs::temp_directory_path() / ("lingtu_prune_io_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  SaveOptions options;
  Fixture() {
    fs::create_directories(root);
    options = {root / "map.pcd", root / "map.clean.pcd", root / "map.removed.pcd",
               root / "map.before_clean.pcd", root / "map.pcd.tmpclean", true, false};
  }
  ~Fixture() { std::error_code ec; fs::remove_all(root, ec); }
};

template <typename Action>
void mustThrow(Action action) {
  bool threw = false;
  try { action(); } catch (const std::exception &) { threw = true; }
  require(threw, "invalid partition input was accepted");
}

std::string payload(const fs::path &path) {
  const auto data = contents(path);
  const auto offset = data.find("DATA binary\n");
  require(offset != std::string::npos, "binary format was changed");
  return data.substr(offset + std::string("DATA binary\n").size());
}

void binaryFields() {
  Fixture fixture;
  const std::array<std::array<float, 8>, 3> rows{{
      {{1, 2, 3, 4, .1F, .2F, .3F, .4F}},
      {{5, 6, 7, 8, .5F, .6F, .7F, .8F}},
      {{9, 10, 11, 12, .9F, 1.0F, 1.1F, 1.2F}}}};
  {
    std::ofstream out(fixture.options.map_pcd, std::ios::binary);
    out << "# preserved comment\nVERSION 0.7\nFIELDS x y z intensity normal_x normal_y normal_z curvature\n"
        << "SIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\nCOUNT 1 1 1 1 1 1 1 1\n"
        << "WIDTH 3\nHEIGHT 1\nVIEWPOINT 1 2 3 1 0 0 0\nPOINTS 3\nDATA binary\n";
    out.write(reinterpret_cast<const char *>(rows.data()), sizeof(rows));
  }
  const auto original = contents(fixture.options.map_pcd);
  const auto saved = writeCleanedMap(fixture.options, {0, 1, 0});
  require(saved.success, "binary partition failed");
  const auto clean = payload(fixture.options.clean_pcd);
  const auto removed = payload(fixture.options.removed_pcd);
  const std::string first(reinterpret_cast<const char *>(rows[0].data()), sizeof(rows[0]));
  const std::string middle(reinterpret_cast<const char *>(rows[1].data()), sizeof(rows[1]));
  const std::string last(reinterpret_cast<const char *>(rows[2].data()), sizeof(rows[2]));
  require(clean == first + last && removed == middle, "original fields or record order changed");
  const auto header = contents(fixture.options.clean_pcd);
  require(header.find("FIELDS x y z intensity normal_x normal_y normal_z curvature") != std::string::npos &&
          header.find("VIEWPOINT 1 2 3 1 0 0 0") != std::string::npos &&
          header.find("# preserved comment") != std::string::npos &&
          header.find("POINTS 2\n") != std::string::npos, "source header metadata lost");
  require(contents(fixture.options.map_pcd) == original, "preview changed source");
  mustThrow([&] { partitionPcd(fixture.options.map_pcd, fixture.options.clean_pcd,
                             fixture.options.removed_pcd, {0, 1}); });
  mustThrow([&] { partitionPcd(fixture.options.map_pcd, fixture.options.clean_pcd,
                             fixture.options.removed_pcd, {0, 2, 0}); });
  fs::resize_file(fixture.options.map_pcd, fs::file_size(fixture.options.map_pcd) - 1);
  mustThrow([&] { readPcd(fixture.options.map_pcd); });
  mustThrow([&] { partitionPcd(fixture.options.map_pcd, fixture.options.clean_pcd,
                             fixture.options.removed_pcd, {0, 1, 0}); });
}

void asciiCounts() {
  Fixture fixture;
  const std::string row1 = "1 2 3 10 20 30 4";
  const std::string row2 = "5 6 7 40 50 60 8";
  std::ofstream(fixture.options.map_pcd)
      << "VERSION 0.7\nFIELDS x y z descriptor intensity\nSIZE 4 4 4 4 4\n"
      << "TYPE F F F F F\nCOUNT 1 1 1 3 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
      << row1 << '\n' << row2 << '\n';
  const auto points = readPcd(fixture.options.map_pcd);
  require(points.size() == 2 && points[0].intensity == 4 && points[1].intensity == 8,
          "ASCII COUNT offsets were ignored");
  const auto saved = writeCleanedMap(fixture.options, {1, 0});
  require(saved.success, "ASCII partition failed");
  require(contents(fixture.options.clean_pcd).find(row2) != std::string::npos &&
          contents(fixture.options.removed_pcd).find(row1) != std::string::npos &&
          contents(fixture.options.clean_pcd).find("COUNT 1 1 1 3 1") != std::string::npos,
          "ASCII values or COUNT metadata changed");
  std::ofstream(fixture.options.map_pcd, std::ios::app) << row1 << '\n';
  mustThrow([&] { readPcd(fixture.options.map_pcd); });
  mustThrow([&] { partitionPcd(fixture.options.map_pcd, fixture.options.clean_pcd,
                             fixture.options.removed_pcd, {1, 0}); });
}

void applyAndBackup() {
  Fixture fixture;
  writePcd(fixture.options.map_pcd, {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}});
  const auto original = contents(fixture.options.map_pcd);
  fixture.options.apply_to_map = true;
  const auto empty = writeCleanedMap(fixture.options, {1, 1, 1});
  require(!empty.success && empty.reason_code == "empty_clean_map", "empty apply was accepted");
  require(contents(fixture.options.map_pcd) == original && !fs::exists(fixture.options.backup_pcd) &&
          !fs::exists(fixture.options.clean_pcd), "empty apply changed source artifacts");
  const auto first = writeCleanedMap(fixture.options, {0, 1, 0});
  require(first.success && contents(fixture.options.backup_pcd) == original, "first backup missing");
  const auto second = writeCleanedMap(fixture.options, {0, 1});
  require(second.success && contents(fixture.options.backup_pcd) == original &&
          readPcd(fixture.options.map_pcd).size() == 1, "repeat apply replaced the first backup");
}

}  // namespace

int main() {
  try {
    binaryFields();
    asciiCounts();
    applyAndBackup();
    std::cout << "prune PCD partition tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
