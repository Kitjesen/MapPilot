#include "lingtu/maps/build/import_check.hpp"
#include "lingtu/maps/build/pcd.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using lingtu::maps::CheckPointCloudForNavigation;
using lingtu::maps::PointXyz;

namespace {

// A 6 m x 4 m room: floor sampled every `spacing` metres, two 2 m walls.
std::vector<PointXyz> Room(double spacing) {
  std::vector<PointXyz> points;
  for (double x = 0.0; x < 6.0; x += spacing)
    for (double y = 0.0; y < 4.0; y += spacing)
      points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
  for (double x = 0.0; x < 6.0; x += 0.05)
    for (double z = 0.0; z < 2.0; z += 0.05) {
      points.push_back({static_cast<float>(x), 0.0F, static_cast<float>(z)});
      points.push_back({static_cast<float>(x), 4.0F, static_cast<float>(z)});
    }
  return points;
}

std::vector<PointXyz> RotatedAboutX(std::vector<PointXyz> points, double degrees) {
  const double a = degrees * 3.14159265358979323846 / 180.0;
  for (auto& p : points) {
    const double y = p.y * std::cos(a) - p.z * std::sin(a);
    const double z = p.y * std::sin(a) + p.z * std::cos(a);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
  }
  return points;
}

bool Mentions(const lingtu::maps::PointCloudNavigationCheck& check, const std::string& text) {
  for (const auto& blocker : check.blockers)
    if (blocker.find(text) != std::string::npos) return true;
  return false;
}

void TestNavigationCheck() {
  const auto level = CheckPointCloudForNavigation(Room(0.02), 0.05);
  assert(level.ok());
  assert(level.floor_tilt_deg < 0.1);
  assert(level.floor_fill > 0.9);

  assert(Mentions(CheckPointCloudForNavigation(RotatedAboutX(Room(0.02), 8.0), 0.05), "tilted"));
  assert(!CheckPointCloudForNavigation(RotatedAboutX(Room(0.02), 30.0), 0.05).ok());
  std::vector<PointXyz> walls;
  for (const auto& p : Room(0.02))
    if (p.z > 0.0F) walls.push_back(p);
  assert(!CheckPointCloudForNavigation(walls, 0.05).ok());

  auto millimetres = Room(0.02);
  for (auto& p : millimetres) { p.x *= 1000.0F; p.y *= 1000.0F; p.z *= 1000.0F; }
  assert(Mentions(CheckPointCloudForNavigation(millimetres, 0.05), "check the units"));

  const auto sparse = CheckPointCloudForNavigation(Room(0.2), 0.05);
  assert(Mentions(sparse, "too sparse"));
  assert(CheckPointCloudForNavigation(Room(0.2), 0.25).ok());
}

void Append(std::string& out, const void* data, std::size_t size) {
  out.append(static_cast<const char*>(data), size);
}

// LZF literal runs (at most 32 bytes each) for `bytes`.
void AppendLiterals(std::string& lzf, const std::string& bytes) {
  for (std::size_t i = 0; i < bytes.size(); i += 32U) {
    const std::size_t n = std::min<std::size_t>(32U, bytes.size() - i);
    lzf.push_back(static_cast<char>(n - 1U));
    lzf.append(bytes, i, n);
  }
}

void TestBinaryCompressedPcd() {
  // Five points; x is constant so it is encoded as one literal float plus a
  // back reference, y and z as literals. The intensity field sits between.
  const float xs[5] = {1.5F, 1.5F, 1.5F, 1.5F, 1.5F};
  const float ys[5] = {0.0F, 1.0F, 2.0F, 3.0F, 4.0F};
  const float intensity[5] = {9.0F, 9.0F, 9.0F, 9.0F, 9.0F};
  const float zs[5] = {-1.0F, -0.5F, 0.0F, 0.5F, 1.0F};
  std::string lzf;
  lzf.push_back(3);                                   // literal: 4 bytes
  Append(lzf, &xs[0], sizeof(float));
  lzf += std::string{static_cast<char>(224), 7, 3};  // copy 16 bytes from 4 back
  AppendLiterals(lzf, std::string(reinterpret_cast<const char*>(ys), sizeof(ys)));
  AppendLiterals(lzf, std::string(reinterpret_cast<const char*>(intensity), sizeof(intensity)));
  AppendLiterals(lzf, std::string(reinterpret_cast<const char*>(zs), sizeof(zs)));

  const auto path = std::filesystem::temp_directory_path() /
      ("lingtu_compressed_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".pcd");
  const auto write = [&](const std::string& payload) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file << "VERSION 0.7\nFIELDS x y intensity z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
            "WIDTH 5\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 5\nDATA binary_compressed\n";
    const std::uint32_t sizes[2] = {static_cast<std::uint32_t>(payload.size()), 80U};
    file.write(reinterpret_cast<const char*>(sizes), sizeof(sizes));
    file.write(payload.data(), static_cast<std::streamsize>(payload.size()));
  };
  write(lzf);
  const auto loaded = lingtu::maps::LoadPcdXyz(path);
  assert(loaded.ok);
  assert(loaded.points.size() == 5U);
  for (std::size_t i = 0; i < 5U; ++i) {
    assert(loaded.points[i].x == xs[i]);
    assert(loaded.points[i].y == ys[i]);
    assert(loaded.points[i].z == zs[i]);
  }

  std::string reaching_back = lzf;
  reaching_back[7] = 40;  // back reference before the start of the output
  write(reaching_back);
  assert(!lingtu::maps::LoadPcdXyz(path).ok);
  write(lzf.substr(0, lzf.size() - 3U));
  assert(!lingtu::maps::LoadPcdXyz(path).ok);
  std::filesystem::remove(path);
}

}  // namespace

int main() {
  TestNavigationCheck();
  TestBinaryCompressedPcd();
  return 0;
}
