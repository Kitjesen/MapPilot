#include "native/dds_module.hpp"

#include "message/cpp/qos.hpp"

#include "dds/dds.h"
#include "messages.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

namespace lingtu::drivers::gnss {
namespace {

void fill_header(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (stamp_s <= 0.0) {
    stamp_s = now_seconds();
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void log_dds_error(dds_return_t value, const char* what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

lingtu_dds_GnssStatus to_status_msg(
    const lingtu::gnss::FixSample& fix,
    const Config& cfg,
    const std::string& device,
    const std::string& error,
    double stamp_s) {
  lingtu_dds_GnssStatus msg{};
  fill_header(msg.header, stamp_s, cfg.frame_id.c_str());
  msg.device = const_cast<char*>(device.c_str());
  msg.fix_type = fix.fix_type;
  msg.link_ok = true;
  msg.rtk = lingtu::gnss::is_rtk_fix(fix.fix_type);
  msg.num_sat = static_cast<std::uint32_t>(std::max(0, fix.num_sat));
  msg.num_sat_used = static_cast<std::uint32_t>(std::max(0, fix.num_sat_used));
  msg.hdop = fix.hdop.value_or(99.9);
  msg.rtcm_age_s = fix.rtcm_age_s.value_or(99.9);
  msg.error = const_cast<char*>(error.c_str());
  return msg;
}

lingtu_dds_GnssFix to_fix_msg(
    const lingtu::gnss::FixSample& fix,
    const Config& cfg,
    double stamp_s,
    std::array<double, 9>& covariance) {
  lingtu_dds_GnssFix msg{};
  fill_header(msg.header, stamp_s, cfg.frame_id.c_str());
  msg.latitude = fix.latitude_deg.value_or(0.0);
  msg.longitude = fix.longitude_deg.value_or(0.0);
  msg.altitude = fix.altitude_m.value_or(0.0);
  msg.fix_type = fix.fix_type;
  covariance = covariance_for(fix);
  std::copy(covariance.begin(), covariance.end(), msg.position_covariance);
  msg.num_sat = static_cast<std::uint32_t>(std::max(0, fix.num_sat));
  msg.num_sat_used = static_cast<std::uint32_t>(std::max(0, fix.num_sat_used));
  msg.hdop = fix.hdop.value_or(99.9);
  msg.rtcm_age_s = fix.rtcm_age_s.value_or(99.9);
  return msg;
}

lingtu_dds_Odometry to_odom_msg(
    const lingtu::gnss::FixSample& fix,
    const Config& cfg,
    const std::array<double, 9>& covariance,
    double stamp_s) {
  lingtu_dds_Odometry msg{};
  fill_header(msg.header, stamp_s, cfg.map_frame_id.c_str());
  msg.child_frame_id = const_cast<char*>(cfg.frame_id.c_str());
  const Enu enu = lla_to_enu(fix, *cfg.origin);
  msg.pose.pose.position.x = enu.east;
  msg.pose.pose.position.y = enu.north;
  msg.pose.pose.position.z = enu.up;
  msg.pose.pose.orientation.w = 1.0;
  msg.pose.covariance[0] = covariance[0];
  msg.pose.covariance[7] = covariance[4];
  msg.pose.covariance[14] = covariance[8];
  msg.twist.twist.linear.x = 0.0;
  msg.twist.twist.linear.y = 0.0;
  msg.twist.twist.linear.z = 0.0;
  return msg;
}

}  // namespace

class DdsModule::Impl {
 public:
  explicit Impl(const Config& cfg)
      : config_(cfg),
        participant_(checked(dds_create_participant(
            static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
            "dds_create_participant")),
        publisher_(checked(dds_create_publisher(participant_, nullptr, nullptr),
            "dds_create_publisher")) {
    fix_writer_ = create_writer(
        cfg.fix_topic.c_str(), &lingtu_dds_GnssFix_desc, "gnss_fix");
    status_writer_ = create_writer(
        cfg.status_topic.c_str(), &lingtu_dds_GnssStatus_desc, "gnss_status");
    odom_writer_ = create_writer(
        cfg.odom_topic.c_str(), &lingtu_dds_Odometry_desc, "gnss_odom");
  }

  ~Impl() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  void publish_status(
      const lingtu::gnss::FixSample& fix,
      const std::string& device,
      const std::string& error,
      double stamp_s,
      Status& status) {
    auto msg = to_status_msg(fix, config_, device, error, stamp_s);
    log_dds_error(dds_write(status_writer_, &msg), "dds_write(gnss_status)");
    status.statuses += 1;
    status.last_fix_type = fix.fix_type;
    status.last_ts = stamp_s;
    status.last_error = error;
  }

  std::array<double, 9> publish_fix(
      const lingtu::gnss::FixSample& fix,
      double stamp_s,
      Status& status) {
    std::array<double, 9> covariance{};
    auto msg = to_fix_msg(fix, config_, stamp_s, covariance);
    log_dds_error(dds_write(fix_writer_, &msg), "dds_write(gnss_fix)");
    status.fixes += 1;
    return covariance;
  }

  void publish_odom(
      const lingtu::gnss::FixSample& fix,
      const std::array<double, 9>& covariance,
      double stamp_s,
      Status& status) {
    auto msg = to_odom_msg(fix, config_, covariance, stamp_s);
    log_dds_error(dds_write(odom_writer_, &msg), "dds_write(gnss_odom)");
    status.odometry += 1;
  }

 private:
  dds_entity_t create_writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  Config config_;
  dds_entity_t participant_{0};
  dds_entity_t publisher_{0};
  dds_entity_t fix_writer_{0};
  dds_entity_t status_writer_{0};
  dds_entity_t odom_writer_{0};
};

DdsModule::DdsModule(const Config& config) : impl_(new Impl(config)) {}

DdsModule::~DdsModule() {
  delete impl_;
}

void DdsModule::publish_status(
    const lingtu::gnss::FixSample& fix,
    const std::string& device,
    const std::string& error,
    double stamp_s,
    Status& status) {
  impl_->publish_status(fix, device, error, stamp_s, status);
}

std::array<double, 9> DdsModule::publish_fix(
    const lingtu::gnss::FixSample& fix,
    double stamp_s,
    Status& status) {
  return impl_->publish_fix(fix, stamp_s, status);
}

void DdsModule::publish_odom(
    const lingtu::gnss::FixSample& fix,
    const std::array<double, 9>& covariance,
    double stamp_s,
    Status& status) {
  impl_->publish_odom(fix, covariance, stamp_s, status);
}

}  // namespace lingtu::drivers::gnss
