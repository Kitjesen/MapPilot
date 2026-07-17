#include "native/dds_module.hpp"
#include "native/module.hpp"
#include "wtrtk980/nmea.hpp"
#include "wtrtk980/serial.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <exception>
#include <string>

namespace {

std::atomic_bool g_running{true};

void stop_signal(int) {
  g_running = false;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);

  try {
    auto module = lingtu::drivers::gnss::Module(
        lingtu::drivers::gnss::config_from_args(argc, argv));
    const auto& cfg = module.config();
    auto& status = module.status();

    lingtu::drivers::gnss::DdsModule dds(cfg);
    lingtu::drivers::wtrtk980::SerialReader reader(cfg.device, cfg.baud, cfg.timeout_ms);
    lingtu::drivers::wtrtk980::NmeaParser parser;
    reader.open();

    status.device = reader.device();
    double next_status_s = 0.0;

    std::fprintf(stderr,
                 "lingtu_gnss_dds: domain=%d device=%s fix=%s status=%s odom=%s\n",
                 cfg.domain_id,
                 reader.device().c_str(),
                 cfg.fix_topic.c_str(),
                 cfg.status_topic.c_str(),
                 cfg.odom_topic.c_str());

    while (g_running) {
      const auto line = reader.read_line();
      const double stamp_s = lingtu::drivers::gnss::now_seconds();
      if (!line) {
        if (stamp_s >= next_status_s) {
          status.last_error = "nmea_timeout";
          status.last_ts = stamp_s;
          lingtu::drivers::gnss::write_status_file(cfg.status_file, status);
          next_status_s = stamp_s + 1.0;
        }
        continue;
      }

      status.nmea_lines += 1;
      if (!parser.parse_line(*line)) {
        continue;
      }

      const auto& fix = parser.state();
      const std::string error = fix.has_fix ? "" : "no_fix";
      dds.publish_status(fix, reader.device(), error, stamp_s, status);

      if (fix.has_fix) {
        const auto covariance = dds.publish_fix(fix, stamp_s, status);
        if (cfg.publish_odom && cfg.origin) {
          dds.publish_odom(fix, covariance, stamp_s, status);
        }
      }

      if (stamp_s >= next_status_s) {
        lingtu::drivers::gnss::write_status_file(cfg.status_file, status);
        next_status_s = stamp_s + 1.0;
      }
      if (cfg.max_samples > 0 && status.nmea_lines >= cfg.max_samples) {
        break;
      }
    }

    lingtu::drivers::gnss::write_status_file(cfg.status_file, status);
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_gnss_dds failed: %s\n", exc.what());
    return 1;
  }
}
