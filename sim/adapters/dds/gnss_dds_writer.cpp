#include "gnss_dds_writer.hpp"

#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "messages.h"

namespace lingtu::sim::dds_adapter {
namespace {
dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0)
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return static_cast<dds_entity_t>(value);
}
}  // namespace

GnssDdsWriter::GnssDdsWriter(int domain) {
  participant_ = checked(
      dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr), "participant");
  try {
    const auto fix_topic =
        checked(dds_create_topic(participant_, &lingtu_dds_GnssFix_desc,
                                 lingtu::message::kGnssFix.dds_topic.data(), nullptr, nullptr),
                "fix topic");
    const auto status_topic =
        checked(dds_create_topic(participant_, &lingtu_dds_GnssStatus_desc,
                                 lingtu::message::kGnssStatus.dds_topic.data(), nullptr, nullptr),
                "status topic");
    auto fix_qos = lingtu::dds::make_qos(lingtu::message::kGnssFix.qos_profile);
    auto status_qos = lingtu::dds::make_qos(lingtu::message::kGnssStatus.qos_profile);
    fix_writer_ =
        checked(dds_create_writer(participant_, fix_topic, fix_qos.get(), nullptr), "fix writer");
    status_writer_ = checked(
        dds_create_writer(participant_, status_topic, status_qos.get(), nullptr), "status writer");
  } catch (...) {
    dds_delete(participant_);
    participant_ = 0;
    throw;
  }
}

GnssDdsWriter::~GnssDdsWriter() {
  if (participant_ > 0)
    dds_delete(participant_);
}

void GnssDdsWriter::write(const GnssRecord &r) {
  lingtu_dds_GnssFix fix{};
  fix.header.stamp.sec = static_cast<std::int32_t>(r.timestamp_ns / 1000000000ULL);
  fix.header.stamp.nanosec = static_cast<std::uint32_t>(r.timestamp_ns % 1000000000ULL);
  fix.header.frame_id = const_cast<char *>(r.frame_id.c_str());
  fix.latitude = r.latitude;
  fix.longitude = r.longitude;
  fix.altitude = r.altitude;
  fix.fix_type = r.fix_type;
  std::copy(r.position_covariance.begin(), r.position_covariance.end(), fix.position_covariance);
  fix.num_sat = r.num_sat;
  fix.num_sat_used = r.num_sat_used;
  fix.hdop = r.hdop;
  fix.rtcm_age_s = r.rtcm_age_s;
  lingtu_dds_GnssStatus status{};
  status.header = fix.header;
  status.device = const_cast<char *>("simulation");
  status.fix_type = fix.fix_type;
  status.link_ok = true;
  status.rtk = false;
  status.num_sat = fix.num_sat;
  status.num_sat_used = fix.num_sat_used;
  status.hdop = fix.hdop;
  status.rtcm_age_s = fix.rtcm_age_s;
  status.error = const_cast<char *>(r.fix_type == 0 ? "simulation_no_fix" : "");
  checked(dds_write(fix_writer_, &fix), "dds_write(gnss fix)");
  checked(dds_write(status_writer_, &status), "dds_write(gnss status)");
}

}  // namespace lingtu::sim::dds_adapter
