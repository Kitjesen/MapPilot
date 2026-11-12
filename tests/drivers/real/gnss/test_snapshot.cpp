#include "native/module.hpp"
#include "wtrtk980/nmea.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <thread>

namespace {

std::string read(const std::filesystem::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

int main() {
  using lingtu::drivers::gnss::Status;
  using lingtu::drivers::gnss::write_status_file;
  lingtu::drivers::wtrtk980::NmeaParser parser;
  const std::string fixed =
      "$GNGGA,123519,4807.038,N,01131.000,E,4,12,0.7,545.4,M,46.9,M,,";
  assert(parser.parse_line(fixed));
  Status status;
  status.device = "/dev/test";
  status.update(parser.state(), "", 1794110400.125);
  assert(status.location);
  assert(std::abs(status.location->lat - 48.1173) < 1e-10);
  assert(std::abs(status.location->lon - 11.516666666666667) < 1e-10);
  assert(status.location->fix_type == 4);
  assert(!status.location->accuracy_m);  // HDOP and nominal RTK accuracy are not measurements.
  status.last_ts += 1.0;
  assert(status.location->ts == 1794110400.125);

  const auto path = std::filesystem::temp_directory_path() /
      ("lingtu_gnss_snapshot_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  write_status_file(path.string(), status);
  const auto valid = read(path);
  assert(std::stod(valid.substr(valid.find("\"lat\":") + 6)) == status.location->lat);
  assert(std::stod(valid.substr(valid.find("\"lon\":") + 6)) == status.location->lon);
  assert(valid.find("\"ts\":1794110400.125") != std::string::npos);
  assert(valid.find("\"accuracy_m\":null") != std::string::npos);

  assert(parser.parse_line("$GNGGA,123520,,,,,0,0,99.9,,,,,,"));
  status.update(parser.state(), "no_fix", 1794110401.0);
  assert(!status.location);
  assert(parser.parse_line(fixed));
  status.update(parser.state(), "", 1794110402.0);
  assert(parser.parse_line(
      "$GNRMC,123521,V,4807.038,N,01131.000,E,0,0,081126,,,"));
  assert(!parser.state().has_fix);
  status.update(parser.state(), "no_fix", 1794110403.0);
  assert(!status.location);

  assert(parser.parse_line(fixed));
  status.update(parser.state(), "", 1794110404.0);
  status.update({}, "nmea_timeout", 1794110405.0);
  assert(!status.location);
  status.update(parser.state(), "", 1794110406.0);
  status.update({}, "serial \"error\"\n", 1794110407.0);
  assert(!status.location);
  write_status_file(path.string(), status);
  const auto invalid = read(path);
  assert(invalid.find("\"location\":null") != std::string::npos);
  assert(invalid.find("serial \\\"error\\\"\\u000a") != std::string::npos);
  assert(!std::filesystem::exists(path.string() + ".tmp"));

  auto invalid_fix = parser.state();
  invalid_fix.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  status.update(invalid_fix, "", 1794110408.0);
  assert(!status.location);
  invalid_fix.latitude_deg = 91.0;
  status.update(invalid_fix, "", 1794110408.0);
  assert(!status.location);

  // Readers must see either complete snapshot while the sole writer replaces it.
  Status valid_status;
  valid_status.device = "/dev/test";
  valid_status.update(parser.state(), "", 1794110400.125);
  valid_status.last_ts += 1.0;
  Status invalid_status;
  invalid_status.device = "/dev/test";
  invalid_status.update({}, "serial \"error\"\n", 1794110407.0);
  write_status_file(path.string(), valid_status);
  assert(read(path) == valid);
  write_status_file(path.string(), invalid_status);
  assert(read(path) == invalid);
#ifndef _WIN32
  // The field contract uses POSIX rename; Windows sharing rules differ.
  std::atomic_bool done{false};
  std::atomic_int reads{0};
  std::thread reader([&] {
    while (!done) {
      const auto snapshot = read(path);
      assert(snapshot == valid || snapshot == invalid);
      ++reads;
    }
  });
  for (int i = 0; i < 200; ++i) {
    write_status_file(path.string(), i % 2 ? valid_status : invalid_status);
  }
  done = true;
  reader.join();
  assert(reads > 0);
#endif
  assert(!std::filesystem::exists(path.string() + ".tmp"));

  const auto before_failure = read(path);
  std::filesystem::create_directory(path.string() + ".tmp");
  write_status_file(path.string(), invalid_status);
  assert(read(path) == before_failure);
  std::filesystem::remove(path);
  std::filesystem::remove(path.string() + ".tmp");
}
