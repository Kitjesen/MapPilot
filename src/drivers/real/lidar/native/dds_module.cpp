#include "native/dds_module.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::drivers::lidar {

class DdsModule::Impl {
 public:
  Impl(
      int domain_id,
      std::string lidar_frame,
      std::string imu_frame,
      bool navigation_fixture)
      : lidar_frame_(std::move(lidar_frame)),
        imu_frame_(std::move(imu_frame)),
        navigation_fixture_(navigation_fixture),
        diagnostics_enabled_(diagnostics_enabled_from_environment()) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    publisher_ = checked(
        dds_create_publisher(participant_, nullptr, nullptr),
        "dds_create_publisher");
    lidar_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_LivoxFrame_desc,
            lingtu::message::kLidarRawFrame.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(lidar)");
    raw_packet_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_LivoxFrame_desc,
            lingtu::message::kLidarRawPacket.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(raw_packet)");
    imu_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_Imu_desc,
            lingtu::message::kImuRaw.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(imu)");
    odom_prior_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_Odometry_desc,
            lingtu::message::kSlamOdomPrior.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(odom_prior)");

    auto lidar_qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kLidarRawFrame.dds_topic));
    auto raw_packet_qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kLidarRawPacket.dds_topic));
    auto sensor_qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::SensorStream);
    lidar_writer_ = checked(
        dds_create_writer(publisher_, lidar_topic_, lidar_qos.get(), nullptr),
        "dds_create_writer(lidar)");
    raw_packet_writer_ = checked(
        dds_create_writer(publisher_, raw_packet_topic_, raw_packet_qos.get(), nullptr),
        "dds_create_writer(raw_packet)");
    imu_writer_ = checked(
        dds_create_writer(publisher_, imu_topic_, sensor_qos.get(), nullptr),
        "dds_create_writer(imu)");
    odom_prior_writer_ = checked(
        dds_create_writer(publisher_, odom_prior_topic_, sensor_qos.get(), nullptr),
        "dds_create_writer(odom_prior)");

    log_writer_qos(lidar_writer_, "raw_frame");
    log_writer_qos(raw_packet_writer_, "raw_packet");

    if (navigation_fixture_) {
      slam_odom_topic_ = create_topic(
          lingtu::message::kSlamOdometry,
          &lingtu_dds_Odometry_desc,
          "slam_odom");
      tf_topic_ = create_topic(
          lingtu::message::kTf,
          &lingtu_dds_TFMessage_desc,
          "tf");
      registered_cloud_topic_ = create_topic(
          lingtu::message::kSlamRegisteredCloud,
          &lingtu_dds_PointCloud2_desc,
          "registered_cloud");
      localization_health_topic_ = create_topic(
          lingtu::message::kSlamLocalizationHealth,
          &lingtu_dds_Text_desc,
          "localization_health");
      slam_odom_writer_ = create_writer(
          slam_odom_topic_, lingtu::message::kSlamOdometry, "slam_odom");
      tf_writer_ = create_writer(tf_topic_, lingtu::message::kTf, "tf");
      registered_cloud_writer_ = create_writer(
          registered_cloud_topic_,
          lingtu::message::kSlamRegisteredCloud,
          "registered_cloud");
      localization_health_writer_ = create_writer(
          localization_health_topic_,
          lingtu::message::kSlamLocalizationHealth,
          "localization_health");
    }
  }

  ~Impl() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  void publish_cloud(
      std::uint8_t lidar_id,
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points) {
    publish_cloud_to(lidar_writer_, "raw_frame", lidar_id, timestamp_ns, points);
  }

  void publish_raw_packet(
      std::uint8_t lidar_id,
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points) {
    publish_cloud_to(
        raw_packet_writer_, "raw_packet", lidar_id, timestamp_ns, points);
  }

  void publish_imu(std::uint64_t timestamp_ns, const ImuSample& imu) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    lingtu_dds_Imu msg{};
    fill_header(msg.header, timestamp_ns, imu_frame_);
    msg.orientation.w = 1.0;
    msg.angular_velocity.x = imu.gyro_x;
    msg.angular_velocity.y = imu.gyro_y;
    msg.angular_velocity.z = imu.gyro_z;
    msg.linear_acceleration.x = imu.acc_x;
    msg.linear_acceleration.y = imu.acc_y;
    msg.linear_acceleration.z = imu.acc_z;
    checked(dds_write(imu_writer_, &msg), "dds_write(imu)");
  }

  void publish_odom_prior(std::uint64_t timestamp_ns, const OdomPrior& prior) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    lingtu_dds_Odometry msg{};
    fill_odometry(msg, timestamp_ns, prior);
    checked(dds_write(odom_prior_writer_, &msg), "dds_write(odom_prior)");

    if (!navigation_fixture_) {
      return;
    }

    checked(dds_write(slam_odom_writer_, &msg), "dds_write(slam_odom)");

    lingtu_dds_TransformStamped transform{};
    fill_header(transform.header, timestamp_ns, map_frame_);
    transform.child_frame_id = const_cast<char*>(odom_frame_.c_str());
    transform.transform.rotation.w = 1.0;
    lingtu_dds_TFMessage tf{};
    tf.transforms._maximum = 1;
    tf.transforms._length = 1;
    tf.transforms._buffer = &transform;
    tf.transforms._release = false;
    checked(dds_write(tf_writer_, &tf), "dds_write(tf)");

    std::string health_json =
        "{\"state\":\"TRACKING\",\"ts\":" +
        std::to_string(static_cast<double>(timestamp_ns) / 1000000000.0) +
        ",\"source\":\"mujoco_navigation_fixture\"}";
    lingtu_dds_Text health{};
    health.data = const_cast<char*>(health_json.c_str());
    checked(
        dds_write(localization_health_writer_, &health),
        "dds_write(localization_health)");
  }

  void publish_registered_cloud(
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points) {
    if (!navigation_fixture_) {
      throw std::runtime_error(
          "registered cloud publishing requires navigation fixture mode");
    }

    std::array<lingtu_dds_PointField, 4> fields{};
    fill_point_fields(fields);
    constexpr std::uint32_t point_step = 16;
    const auto width = static_cast<std::uint32_t>(points.size());
    const auto row_step = point_step * width;
    std::vector<std::uint8_t> data(static_cast<std::size_t>(row_step));
    for (std::size_t i = 0; i < points.size(); ++i) {
      const std::size_t base = i * point_step;
      write_float(data, base + 0, points[i].x);
      write_float(data, base + 4, points[i].y);
      write_float(data, base + 8, points[i].z);
      write_float(data, base + 12, points[i].intensity);
    }

    std::lock_guard<std::mutex> lock(write_mutex_);
    lingtu_dds_PointCloud2 msg{};
    fill_header(msg.header, timestamp_ns, body_frame_);
    msg.height = 1;
    msg.width = width;
    msg.fields._maximum = static_cast<std::uint32_t>(fields.size());
    msg.fields._length = static_cast<std::uint32_t>(fields.size());
    msg.fields._buffer = fields.data();
    msg.fields._release = false;
    msg.is_bigendian = false;
    msg.point_step = point_step;
    msg.row_step = row_step;
    msg.data._maximum = static_cast<std::uint32_t>(data.size());
    msg.data._length = static_cast<std::uint32_t>(data.size());
    msg.data._buffer = data.data();
    msg.data._release = false;
    msg.is_dense = false;
    checked(
        dds_write(registered_cloud_writer_, &msg),
        "dds_write(registered_cloud)");
  }

 private:
  dds_entity_t create_topic(
      const lingtu::message::TopicContract& contract,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    return checked(
        dds_create_topic(
            participant_, descriptor, contract.dds_topic.data(), nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
  }

  dds_entity_t create_writer(
      dds_entity_t topic,
      const lingtu::message::TopicContract& contract,
      const char* label) {
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  void fill_odometry(
      lingtu_dds_Odometry& msg,
      std::uint64_t timestamp_ns,
      const OdomPrior& prior) const {
    fill_header(msg.header, timestamp_ns, odom_frame_);
    msg.child_frame_id = const_cast<char*>(body_frame_.c_str());
    msg.pose.pose.position.x = prior.x;
    msg.pose.pose.position.y = prior.y;
    msg.pose.pose.position.z = prior.z;
    msg.pose.pose.orientation.x = prior.qx;
    msg.pose.pose.orientation.y = prior.qy;
    msg.pose.pose.orientation.z = prior.qz;
    msg.pose.pose.orientation.w = prior.qw;
    if (prior.has_velocity != 0) {
      msg.twist.twist.linear.x = prior.vx;
      msg.twist.twist.linear.y = prior.vy;
      msg.twist.twist.linear.z = prior.vz;
    }
  }

  static void write_float(
      std::vector<std::uint8_t>& data,
      std::size_t offset,
      float value) {
    std::memcpy(data.data() + offset, &value, sizeof(value));
  }

  static void fill_point_fields(
      std::array<lingtu_dds_PointField, 4>& fields) {
    fields[0].name = const_cast<char*>("x");
    fields[0].offset = 0;
    fields[0].datatype = 7;
    fields[0].count = 1;
    fields[1].name = const_cast<char*>("y");
    fields[1].offset = 4;
    fields[1].datatype = 7;
    fields[1].count = 1;
    fields[2].name = const_cast<char*>("z");
    fields[2].offset = 8;
    fields[2].datatype = 7;
    fields[2].count = 1;
    fields[3].name = const_cast<char*>("intensity");
    fields[3].offset = 12;
    fields[3].datatype = 7;
    fields[3].count = 1;
  }

  void publish_cloud_to(
      dds_entity_t writer,
      const char* stream,
      std::uint8_t lidar_id,
      std::uint64_t timestamp_ns,
      const std::vector<Point>& points) {
    std::vector<lingtu_dds_LivoxPoint> dds_points;
    dds_points.reserve(points.size());
    for (const auto& point : points) {
      lingtu_dds_LivoxPoint out{};
      out.offset_time = point.offset_time_ns;
      out.x = point.x;
      out.y = point.y;
      out.z = point.z;
      out.reflectivity = static_cast<std::uint8_t>(point.intensity);
      out.tag = point.tag;
      out.line = point.line;
      dds_points.push_back(out);
    }

    std::lock_guard<std::mutex> lock(write_mutex_);
    lingtu_dds_LivoxFrame msg{};
    fill_header(msg.header, timestamp_ns, lidar_frame_);
    msg.timebase = timestamp_ns;
    msg.point_num = static_cast<std::uint32_t>(dds_points.size());
    msg.lidar_id = lidar_id;
    msg.points._maximum = msg.point_num;
    msg.points._length = msg.point_num;
    msg.points._buffer = dds_points.data();
    msg.points._release = false;
    if (!diagnostics_enabled_) {
      checked(dds_write(writer, &msg), "dds_write(livox)");
      return;
    }

    const auto write_started = std::chrono::steady_clock::now();
    const dds_return_t write_rc = dds_write(writer, &msg);
    const auto write_finished = std::chrono::steady_clock::now();
    const double write_duration_ms =
        std::chrono::duration<double, std::milli>(
            write_finished - write_started)
            .count();

    dds_publication_matched_status_t matched_status{};
    const dds_return_t matched_query_rc =
        dds_get_publication_matched_status(writer, &matched_status);
    constexpr std::uint64_t kLivoxPointPayloadBytes = 24;
    const std::uint64_t payload_bytes =
        static_cast<std::uint64_t>(dds_points.size()) *
        kLivoxPointPayloadBytes;
    std::fprintf(
        stderr,
        "{\"event\":\"livox_dds_write\",\"stream\":\"%s\","
        "\"rc\":%d,\"rc_name\":\"%s\",\"duration_ms\":%.6f,"
        "\"point_count\":%u,\"payload_bytes\":%llu,"
        "\"publication_matched_query_rc\":%d,"
        "\"publication_matched_query_rc_name\":\"%s\","
        "\"publication_matched_current_count\":%d}\n",
        stream,
        static_cast<int>(write_rc),
        return_code_name(write_rc),
        write_duration_ms,
        msg.point_num,
        static_cast<unsigned long long>(payload_bytes),
        static_cast<int>(matched_query_rc),
        return_code_name(matched_query_rc),
        static_cast<int>(matched_status.current_count));
    checked(write_rc, "dds_write(livox)");
  }

  static bool diagnostics_enabled_from_environment() {
    const char* value =
        std::getenv("LINGTU_LIVOX_DDS_WRITE_DIAGNOSTICS");
    return value != nullptr && std::strcmp(value, "1") == 0;
  }

  static const char* return_code_name(dds_return_t value) {
    return dds_strretcode(value < 0 ? -value : value);
  }

  static const char* reliability_name(
      dds_reliability_kind_t reliability,
      bool present) {
    if (!present) {
      return "UNSET";
    }
    switch (reliability) {
      case DDS_RELIABILITY_BEST_EFFORT:
        return "BEST_EFFORT";
      case DDS_RELIABILITY_RELIABLE:
        return "RELIABLE";
    }
    return "UNKNOWN";
  }

  static const char* durability_name(
      dds_durability_kind_t durability,
      bool present) {
    if (!present) {
      return "UNSET";
    }
    switch (durability) {
      case DDS_DURABILITY_VOLATILE:
        return "VOLATILE";
      case DDS_DURABILITY_TRANSIENT_LOCAL:
        return "TRANSIENT_LOCAL";
      case DDS_DURABILITY_TRANSIENT:
        return "TRANSIENT";
      case DDS_DURABILITY_PERSISTENT:
        return "PERSISTENT";
    }
    return "UNKNOWN";
  }

  static const char* history_name(dds_history_kind_t history, bool present) {
    if (!present) {
      return "UNSET";
    }
    switch (history) {
      case DDS_HISTORY_KEEP_LAST:
        return "KEEP_LAST";
      case DDS_HISTORY_KEEP_ALL:
        return "KEEP_ALL";
    }
    return "UNKNOWN";
  }

  void log_writer_qos(dds_entity_t writer, const char* stream) const {
    if (!diagnostics_enabled_) {
      return;
    }

    auto qos = lingtu::dds::make_qos(lingtu::dds::QosProfile::Default);
    const dds_return_t qos_rc = dds_get_qos(writer, qos.get());
    dds_reliability_kind_t reliability{};
    dds_duration_t max_blocking_time{};
    dds_durability_kind_t durability{};
    dds_history_kind_t history{};
    std::int32_t depth = -1;
    dds_duration_t lifespan{};
    std::int32_t max_samples = -1;
    std::int32_t max_instances = -1;
    std::int32_t max_samples_per_instance = -1;
    bool has_reliability = false;
    bool has_durability = false;
    bool has_history = false;
    bool has_lifespan = false;
    bool has_resource_limits = false;
    if (qos_rc >= 0) {
      has_reliability = dds_qget_reliability(
          qos.get(), &reliability, &max_blocking_time);
      has_durability = dds_qget_durability(qos.get(), &durability);
      has_history = dds_qget_history(qos.get(), &history, &depth);
      has_lifespan = dds_qget_lifespan(qos.get(), &lifespan);
      has_resource_limits = dds_qget_resource_limits(
          qos.get(),
          &max_samples,
          &max_instances,
          &max_samples_per_instance);
    }
    const long long lifespan_ms = has_lifespan
        ? static_cast<long long>(lifespan / DDS_MSECS(1))
        : -1;
    std::fprintf(
        stderr,
        "{\"event\":\"livox_dds_writer_qos\",\"stream\":\"%s\","
        "\"profile\":\"RawLidarStream\",\"get_qos_rc\":%d,"
        "\"get_qos_rc_name\":\"%s\",\"reliability\":\"%s\","
        "\"durability\":\"%s\",\"history\":\"%s\",\"depth\":%d,"
        "\"lifespan_ms\":%lld,\"resource_limits\":{"
        "\"present\":%s,\"max_samples\":%d,\"max_instances\":%d,"
        "\"max_samples_per_instance\":%d}}\n",
        stream,
        static_cast<int>(qos_rc),
        return_code_name(qos_rc),
        reliability_name(reliability, has_reliability),
        durability_name(durability, has_durability),
        history_name(history, has_history),
        depth,
        lifespan_ms,
        has_resource_limits ? "true" : "false",
        max_samples,
        max_instances,
        max_samples_per_instance);
  }

  static dds_entity_t checked(dds_return_t value, const char* what) {
    if (value < 0) {
      throw std::runtime_error(
          std::string(what) + ": " + dds_strretcode(-value));
    }
    return static_cast<dds_entity_t>(value);
  }

  static void fill_header(
      lingtu_dds_Header& header,
      std::uint64_t timestamp_ns,
      const std::string& frame_id) {
    header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000ULL);
    header.stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000ULL);
    header.frame_id = const_cast<char*>(frame_id.c_str());
  }

  std::string lidar_frame_;
  std::string imu_frame_;
  bool navigation_fixture_{false};
  bool diagnostics_enabled_{false};
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string body_frame_{"body"};
  dds_entity_t participant_{DDS_RETCODE_ERROR};
  dds_entity_t publisher_{DDS_RETCODE_ERROR};
  dds_entity_t lidar_topic_{DDS_RETCODE_ERROR};
  dds_entity_t raw_packet_topic_{DDS_RETCODE_ERROR};
  dds_entity_t imu_topic_{DDS_RETCODE_ERROR};
  dds_entity_t odom_prior_topic_{DDS_RETCODE_ERROR};
  dds_entity_t slam_odom_topic_{DDS_RETCODE_ERROR};
  dds_entity_t tf_topic_{DDS_RETCODE_ERROR};
  dds_entity_t registered_cloud_topic_{DDS_RETCODE_ERROR};
  dds_entity_t localization_health_topic_{DDS_RETCODE_ERROR};
  dds_entity_t lidar_writer_{DDS_RETCODE_ERROR};
  dds_entity_t raw_packet_writer_{DDS_RETCODE_ERROR};
  dds_entity_t imu_writer_{DDS_RETCODE_ERROR};
  dds_entity_t odom_prior_writer_{DDS_RETCODE_ERROR};
  dds_entity_t slam_odom_writer_{DDS_RETCODE_ERROR};
  dds_entity_t tf_writer_{DDS_RETCODE_ERROR};
  dds_entity_t registered_cloud_writer_{DDS_RETCODE_ERROR};
  dds_entity_t localization_health_writer_{DDS_RETCODE_ERROR};
  std::mutex write_mutex_;
};

DdsModule::DdsModule(
    int domain_id,
    std::string lidar_frame,
    std::string imu_frame,
    bool navigation_fixture)
    : impl_(std::make_unique<Impl>(
          domain_id,
          std::move(lidar_frame),
          std::move(imu_frame),
          navigation_fixture)) {}

DdsModule::~DdsModule() = default;

void DdsModule::publish_cloud(
    std::uint8_t lidar_id,
    std::uint64_t timestamp_ns,
    const std::vector<Point>& points) {
  impl_->publish_cloud(lidar_id, timestamp_ns, points);
}

void DdsModule::publish_raw_packet(
    std::uint8_t lidar_id,
    std::uint64_t timestamp_ns,
    const std::vector<Point>& points) {
  impl_->publish_raw_packet(lidar_id, timestamp_ns, points);
}

void DdsModule::publish_imu(std::uint64_t timestamp_ns, const ImuSample& imu) {
  impl_->publish_imu(timestamp_ns, imu);
}

void DdsModule::publish_odom_prior(
    std::uint64_t timestamp_ns,
    const OdomPrior& prior) {
  impl_->publish_odom_prior(timestamp_ns, prior);
}

void DdsModule::publish_registered_cloud(
    std::uint64_t timestamp_ns,
    const std::vector<Point>& points) {
  impl_->publish_registered_cloud(timestamp_ns, points);
}

}  // namespace lingtu::drivers::lidar
