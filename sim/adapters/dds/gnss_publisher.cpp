#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "dds_domain.hpp"
#include "gnss_protocol.hpp"
#include "gnss_dds_writer.hpp"
#include "message/generated/topics.hpp"

namespace {
namespace adapter = lingtu::sim::dds_adapter;
struct Config {
  int domain{-1};
  std::string session;
  std::filesystem::path ready;
};
Config parse(int argc, const char* const argv[]) {
  Config c;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" && argc == 2) {
      std::cout << "usage: lingtu_gnss_publisher --dds-domain N --session-id ID --ready-file PATH\n";
      std::exit(0);
    }
    if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
    const std::string value = argv[++i];
    if (arg == "--dds-domain" && c.domain == -1) c.domain = adapter::parse_supported_dds_domain_id(value);
    else if (arg == "--session-id" && c.session.empty()) c.session = value;
    else if (arg == "--ready-file" && c.ready.empty()) c.ready = value;
    else throw std::runtime_error("unexpected or duplicate argument: " + arg);
  }
  if (c.domain < 0 || !adapter::valid_gnss_session_id(c.session) || c.ready.empty())
    throw std::runtime_error("all GNSS publisher arguments are required and valid");
  return c;
}

class Readiness final {
 public:
  explicit Readiness(const Config& c) : path_(c.ready) {
    if (path_.has_parent_path()) std::filesystem::create_directories(path_.parent_path());
    try {
      std::ofstream out(path_, std::ios::binary | std::ios::trunc);
      out << "{\"schema\":\"lingtu.sim.gnss-publisher.ready.v1\",\"ready\":true,\"dds_domain\":"
          << c.domain << ",\"session_id\":\"" << c.session << "\",\"topic\":\""
          << lingtu::message::kGnssFix.dds_topic << "\",\"status_topic\":\""
          << lingtu::message::kGnssStatus.dds_topic << "\"}\n";
      out.close();
      if (!out) throw std::runtime_error("cannot write GNSS readiness file");
    } catch (...) { remove(); throw; }
  }
  Readiness(const Readiness&) = delete;
  Readiness& operator=(const Readiness&) = delete;
  ~Readiness() { remove(); }
 private:
  void remove() noexcept { std::error_code error; std::filesystem::remove(path_, error); }
  std::filesystem::path path_;
};
int run(const Config& c) {
  adapter::GnssDdsWriter writer(c.domain);
  Readiness ready(c);
  adapter::GnssStreamValidator validator(c.session);
  for (;;) {
    adapter::GnssRecord record;
    std::string error;
    const auto result = adapter::read_gnss_record(std::cin, record, error);
    if (result == adapter::GnssReadStatus::Eof) return 0;
    if (result == adapter::GnssReadStatus::Error || !validator.accept(record, error))
      throw std::runtime_error("GNSS input rejected: " + error);
    writer.write(record);
  }
}
} // namespace
int main(int argc, const char* const argv[]) {
  try {
#ifdef _WIN32
    // Binary LTGN may contain SUB and CRLF bytes; text stdin corrupts them.
    if (_setmode(_fileno(stdin), _O_BINARY) == -1) throw std::runtime_error("cannot set GNSS stdin binary mode");
#endif
    return run(parse(argc, argv));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "lingtu_gnss_publisher failed: %s\n", e.what());
    return 1;
  }
}
