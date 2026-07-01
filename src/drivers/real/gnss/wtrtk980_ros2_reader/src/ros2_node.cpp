#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/string.hpp"

#include "wtrtk980/nmea_parser.hpp"
#include "wtrtk980/serial_reader.hpp"

class WTRTK980Ros2Node : public rclcpp::Node {
 public:
  WTRTK980Ros2Node() : Node("wtrtk980_node") {
    device_ = declare_parameter<std::string>("device", "auto");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "wtrtk980");
    fix_topic_ = declare_parameter<std::string>("fix_topic", "/gps/fix");
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/gps/nmea");
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    double timer_period = declare_parameter<double>("timer_period", 0.1);
    int serial_timeout_ms = declare_parameter<int>("serial_timeout_ms", 100);
    log_period_ = declare_parameter<double>("log_period", 5.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, qos);
    if (publish_raw_) raw_pub_ = create_publisher<std_msgs::msg::String>(raw_topic_, 10);

    reader_ = std::make_unique<wtrtk980::SerialReader>(device_, baud_, serial_timeout_ms);
    reader_->open();
    last_log_ = now();
    RCLCPP_INFO(get_logger(), "WTRTK-980 opened: device=%s baud=%d fix_topic=%s raw_topic=%s",
                reader_->device().c_str(), baud_, fix_topic_.c_str(), publish_raw_ ? raw_topic_.c_str() : "disabled");

    timer_ = create_wall_timer(std::chrono::duration<double>(timer_period), [this]() { pollOnce(); });
  }

 private:
  void pollOnce() {
    auto raw = reader_->readRawLine();
    if (!raw) return;

    if (raw_pub_) {
      std_msgs::msg::String msg;
      msg.data = *raw;
      raw_pub_->publish(msg);
    }

    auto& state = reader_->state();
    if (!wtrtk980::parseNmeaLine(*raw, state)) return;

    sensor_msgs::msg::NavSatFix nav;
    nav.header.stamp = now();
    nav.header.frame_id = frame_id_;
    nav.status.status = state.fix ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                  : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    nav.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    nav.latitude = state.lat.value_or(std::numeric_limits<double>::quiet_NaN());
    nav.longitude = state.lon.value_or(std::numeric_limits<double>::quiet_NaN());
    nav.altitude = state.altitude_m.value_or(std::numeric_limits<double>::quiet_NaN());

    if (state.fix && state.hdop && *state.hdop > 0.0 && *state.hdop < 99.0) {
      double variance = (*state.hdop) * (*state.hdop);
      nav.position_covariance = {variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance * 2.0};
      nav.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    } else {
      nav.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    fix_pub_->publish(nav);
    logPeriodically(state);
  }

  void logPeriodically(const wtrtk980::GpsFix& state) {
    if (log_period_ <= 0.0) return;
    auto current = now();
    if ((current - last_log_).seconds() < log_period_) return;
    last_log_ = current;
    RCLCPP_INFO(get_logger(), "%s", state.summary().c_str());
  }

  std::string device_;
  int baud_ = 115200;
  std::string frame_id_;
  std::string fix_topic_;
  std::string raw_topic_;
  bool publish_raw_ = true;
  double log_period_ = 5.0;
  rclcpp::Time last_log_;
  std::unique_ptr<wtrtk980::SerialReader> reader_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WTRTK980Ros2Node>());
  rclcpp::shutdown();
  return 0;
}
