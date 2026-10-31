#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "gnss_protocol.hpp"

namespace {
namespace a = lingtu::sim::dds_adapter;
void require(bool value, const char* error) { if (!value) throw std::runtime_error(error); }
void integer(std::string& out, std::uint64_t value, unsigned size) {
  for (unsigned i = 0; i < size; ++i) out.push_back(static_cast<char>(value >> (i*8)));
}
void set_integer(std::string& out, std::size_t at, std::uint64_t value, unsigned size) {
  for (unsigned i = 0; i < size; ++i) out[at+i] = static_cast<char>(value >> (i*8));
}
void set_real(std::string& out, std::size_t at, double value) {
  std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); set_integer(out, at, bits, 8);
}
std::string bytes(bool fix = true) {
  std::string out("LTGN", 4);
  integer(out, 1, 2); integer(out, 7, 2); integer(out, 176, 4); integer(out, 124, 4);
  out += "sim-session"; out.append(64-11, '\0');
  integer(out, 2, 8); integer(out, 3, 8); integer(out, 0, 8); integer(out, 1234567890, 8);
  out += "robot/gnss"; out.append(64-10, '\0');
  integer(out, fix ? 8 : 0, 4); integer(out, 0, 4); integer(out, 0, 4);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double values[14] = {fix ? 31.2 : nan, fix ? 121.4 : nan, fix ? 15.0 : nan,
      fix ? 1.0 : nan, fix ? 0.1 : nan, fix ? 0.0 : nan,
      fix ? 0.1 : nan, fix ? 2.0 : nan, fix ? 0.0 : nan,
      fix ? 0.0 : nan, fix ? 0.0 : nan, fix ? 3.0 : nan, nan, nan};
  for (double value : values) {
    std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); integer(out, bits, 8);
  }
  return out;
}
a::GnssRecord parse(const std::string& value) {
  std::istringstream input(value); a::GnssRecord r; std::string error;
  const auto result = a::read_gnss_record(input, r, error);
  require(result == a::GnssReadStatus::Ok, error.c_str()); return r;
}
void reject(const std::string& value) {
  std::istringstream input(value); a::GnssRecord r; std::string error;
  require(a::read_gnss_record(input, r, error) == a::GnssReadStatus::Error && !error.empty(),
          "invalid record accepted");
}
} // namespace
int main() {
  try {
    const auto good = bytes();
    require(good.size() == 300, "LTGN wire size mismatch");
    auto r = parse(good);
    require(r.session_id == "sim-session" && r.frame_id == "robot/gnss" && r.timestamp_ns == 1234567890 &&
        r.fix_type == 8 && r.latitude == 31.2 && r.longitude == 121.4 && r.altitude == 15 &&
        r.position_covariance[4] == 2 && std::isnan(r.hdop), "roundtrip failed");
    const auto nofix = parse(bytes(false));
    require(nofix.fix_type == 0 && std::isnan(nofix.latitude) && std::isnan(nofix.position_covariance[8]), "NO_FIX failed");
    std::string error;
    std::istringstream empty;
    require(a::read_gnss_record(empty, r, error) == a::GnssReadStatus::Eof, "empty EOF failed");
    std::istringstream stream(good + bytes(false));
    require(a::read_gnss_record(stream, r, error) == a::GnssReadStatus::Ok &&
        a::read_gnss_record(stream, r, error) == a::GnssReadStatus::Ok &&
        a::read_gnss_record(stream, r, error) == a::GnssReadStatus::Eof, "concatenation/EOF failed");
    for (std::size_t size : {std::size_t{1}, std::size_t{175}, std::size_t{176}, std::size_t{299}}) reject(good.substr(0, size));
    for (std::size_t at : {std::size_t{0}, std::size_t{4}, std::size_t{6}, std::size_t{8}, std::size_t{12}}) {
      auto bad = good; bad[at] = static_cast<char>(bad[at] + 1); reject(bad);
    }
    for (const auto at : {16U, 112U}) {
      auto bad = good; bad[at] = '\0'; reject(bad);
      bad = good; bad[at+63] = 'x'; reject(bad);
      bad = good; bad.replace(at, 64, 64, 'a'); reject(bad);
      bad = good; bad[at] = static_cast<char>(0x80); reject(bad);
      bad = good; bad[at] = ' '; reject(bad);
    }
    { auto bad = good; bad[16] = '-'; reject(bad); }
    { auto bad = good; set_integer(bad, 104, std::numeric_limits<std::uint64_t>::max(), 8); reject(bad); }
    for (auto fix : {1U, 4U, 9U, 0xffffffffU}) { auto bad = good; set_integer(bad, 176, fix, 4); reject(bad); }
    for (auto at : {180U, 184U}) { auto bad = good; set_integer(bad, at, 1, 4); reject(bad); }
    for (auto at : {284U, 292U}) { auto bad = good; set_real(bad, at, 0); reject(bad); }
    for (auto at : {188U, 196U, 204U, 212U}) {
      auto bad = good; set_real(bad, at, std::numeric_limits<double>::infinity()); reject(bad);
      bad = good; set_real(bad, at, std::numeric_limits<double>::quiet_NaN()); reject(bad);
      bad = bytes(false); set_real(bad, at, 0); reject(bad);
    }
    { auto bad = good; set_real(bad, 188, 90.01); reject(bad); }
    { auto bad = good; set_real(bad, 196, -180.01); reject(bad); }
    { auto bad = good; set_real(bad, 212, -1); reject(bad); }
    { auto bad = good; set_real(bad, 220, 1); reject(bad); } // asymmetric
    { auto bad = good; set_real(bad, 220, 2); set_real(bad, 236, 2); reject(bad); } // indefinite
    { auto bad = good; for (auto at : {220U, 228U, 236U, 252U, 260U, 268U}) set_real(bad, at, -1.5); reject(bad); }
    { auto edge = good; set_real(edge, 188, -90); set_real(edge, 196, 180); parse(edge); }

    r = parse(good);
    a::GnssStreamValidator validator("sim-session");
    auto bad = r; bad.sequence = 1; require(!validator.accept(bad, error), "first sequence nonzero accepted");
    bad = r; bad.session_id = "wrong"; require(!validator.accept(bad, error), "wrong session accepted");
    require(validator.accept(r, error), "first record rejected");
    require(!validator.accept(r, error), "duplicate accepted");
    bad = r; ++bad.sequence; require(!validator.accept(bad, error), "equal time accepted");
    bad = r; ++bad.timestamp_ns; require(!validator.accept(bad, error), "equal sequence accepted");
    bad = r; ++bad.sequence; --bad.timestamp_ns; require(!validator.accept(bad, error), "time rollback accepted");
    ++r.sequence; ++r.timestamp_ns;
    bad = r; bad.frame_id = "other"; require(!validator.accept(bad, error), "frame mismatch accepted");
    require(validator.accept(r, error), "valid continuation rejected");
    bad = r; --bad.reset_generation; require(!validator.accept(bad, error), "reset rollback accepted");
    bad = r; --bad.model_generation; ++bad.reset_generation; require(!validator.accept(bad, error), "model rollback accepted");
    bad = r; ++bad.reset_generation; require(!validator.accept(bad, error), "new generation nonzero sequence accepted");
    ++r.reset_generation; r.sequence = 0; r.timestamp_ns = 0;
    require(validator.accept(r, error), "reset time zero rejected");
    ++r.model_generation; r.reset_generation = 0;
    require(validator.accept(r, error), "model generation restart rejected");
    std::puts("GNSS protocol validation passed"); return 0;
  } catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 1; }
}
