#include "lingtu/recording/message_preview.hpp"
#include "dds/ddsi/ddsi_serdata.h"
#if __has_include("dds/cdr/dds_cdrstream.h")
#include "dds/cdr/dds_cdrstream.h"
#define LINGTU_CDR_STREAM_ALLOCATOR 1
#else
#include "dds/ddsi/ddsi_cdrstream.h"
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include "messages.h"
namespace lingtu::recording {
std::string json_quote(std::string_view value) {
  // Preserve a whole UTF-8 prefix; frame/task names can contain Chinese text.
  auto length = std::min<std::size_t>(value.size(), 1024);
  if (length < value.size()) {
    while (length > 0 && (static_cast<unsigned char>(value[length]) & 0xc0U) == 0x80U)
      --length;
  }
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value.substr(0, length)) {
    if (c == '"' || c == '\\')
      out << '\\' << static_cast<char>(c);
    else if (c < 0x20U)
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c)
          << std::dec;
    else
      out << static_cast<char>(c);
  }
  out << '"';
  return out.str();
}
std::string text(const char *value) {
  return json_quote(value ? std::string_view(value) : std::string_view());
}
std::string json_number(double value) {
  if (!std::isfinite(value))
    return "null";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(12) << value;
  return out.str();
}
template <typename V>
std::string vector3(const V &v) {
  return "{\"x\":" + json_number(v.x) + ",\"y\":" + json_number(v.y) + ",\"z\":" + json_number(v.z) + "}";
}
std::string quaternion(const lingtu_dds_Quaternion &q) {
  return "{\"x\":" + json_number(q.x) + ",\"y\":" + json_number(q.y) + ",\"z\":" + json_number(q.z) +
         ",\"w\":" + json_number(q.w) + "}";
}
std::string header(const lingtu_dds_Header &h) {
  return "\"frame_id\":" + text(h.frame_id) + ",\"stamp_sec\":" + std::to_string(h.stamp.sec) +
         ",\"stamp_nanosec\":" + std::to_string(h.stamp.nanosec);
}
std::string pose(const lingtu_dds_Pose &p) {
  return "{\"position\":" + vector3(p.position) + ",\"orientation\":" + quaternion(p.orientation) +
         "}";
}
std::string twist(const lingtu_dds_Twist &t) {
  return "{\"linear\":" + vector3(t.linear) + ",\"angular\":" + vector3(t.angular) + "}";
}
template <typename S>
std::string numeric_sequence(const S &values) {
  std::string out = "[";
  for (std::uint32_t i = 0; i < std::min<std::uint32_t>(values._length, 32); ++i) {
    if (i)
      out += ',';
    out += json_number(values._buffer[i]);
  }
  return out + ']';
}

// Read only a small serialized prefix for clouds/images. Do not allocate or
// copy the actual point/pixel arrays just to show metadata in the browser.
class Prefix {
 public:
  explicit Prefix(const ddsi_serdata *data) {
    const auto length = std::min<std::size_t>(ddsi_serdata_size(data), bytes_.size());
    ddsi_serdata_to_ser(data, 0, length, bytes_.data());
    initialize(length);
  }
  Prefix(const std::byte *data, std::size_t size) {
    const auto length = std::min<std::size_t>(size, bytes_.size());
    std::memcpy(bytes_.data(), data, length);
    initialize(length);
  }
  void initialize(std::size_t length) {
    size_ = length;
    if (length < 4 || bytes_[0] != 0 || bytes_[1] != 1)
      throw std::runtime_error("preview encoding unsupported");
  }
  template <typename T>
  T read() {
    position_ += (sizeof(T) - (position_ - 4) % sizeof(T)) % sizeof(T);
    if (position_ + sizeof(T) > size_)
      throw std::runtime_error("preview prefix exhausted");
    T value{};
    std::memcpy(&value, bytes_.data() + position_, sizeof(T));
    position_ += sizeof(T);
    return value;
  }
  std::string string() {
    const auto length = read<std::uint32_t>();
    if (length == 0 || position_ + length > size_)
      throw std::runtime_error("preview string exceeds prefix");
    auto result = json_quote(
        std::string_view(reinterpret_cast<const char *>(bytes_.data() + position_), length - 1));
    position_ += length;
    return result;
  }
  std::string header() {
    const auto sec = read<std::int32_t>();
    const auto nsec = read<std::uint32_t>();
    return "\"frame_id\":" + string() + ",\"stamp_sec\":" + std::to_string(sec) +
           ",\"stamp_nanosec\":" + std::to_string(nsec);
  }

 private:
  std::array<unsigned char, 512> bytes_{};
  std::size_t position_{4}, size_{0};
};

std::uint32_t recorded_path_pose_count(const std::byte *data, std::size_t size) {
  Prefix prefix(data,size);
  (void)prefix.header();
  return prefix.read<std::uint32_t>();
}

bool metadata_type(std::string_view type) {
  return type == "lingtu.dds.PointCloud2" || type == "lingtu.dds.Image" ||
         type == "lingtu.dds.LivoxFrame" || type == "lingtu.dds.MapObservation";
}
std::string prefix_preview(const TopicBinding &binding, Prefix &prefix) {
  const auto type = binding.contract->idl_type;
      auto out = "{" + prefix.header();
      if (type == "lingtu.dds.LivoxFrame") {
        out += ",\"timebase\":" + std::to_string(prefix.read<std::uint64_t>());
        out += ",\"point_count\":" + std::to_string(prefix.read<std::uint32_t>());
      } else if (type == "lingtu.dds.MapObservation") {
        out += ",\"observation_sequence\":" + std::to_string(prefix.read<std::uint64_t>());
        out += ",\"reset_epoch\":" + std::to_string(prefix.read<std::uint64_t>());
        out += ",\"reference_map_id\":" + prefix.string();
      } else {
        const auto height = prefix.read<std::uint32_t>();
        const auto width = prefix.read<std::uint32_t>();
        out += ",\"height\":" + std::to_string(height) + ",\"width\":" + std::to_string(width);
        if (type == "lingtu.dds.PointCloud2")
          out += ",\"point_count\":" + std::to_string(static_cast<std::uint64_t>(height) * width);
        else
          out += ",\"encoding\":" + prefix.string();
      }
      return out + ",\"payload_omitted\":true}";
 }
std::string typed_preview(const TopicBinding &binding, void *sample) {
  const auto type = binding.contract->idl_type;
  if (type == "lingtu.dds.Imu") {
    const auto &s = *static_cast<lingtu_dds_Imu *>(sample);
    return "{" + header(s.header) + ",\"orientation\":" + quaternion(s.orientation) +
           ",\"angular_velocity_rad_s\":" + vector3(s.angular_velocity) +
           ",\"linear_acceleration_m_s2\":" + vector3(s.linear_acceleration) + "}";
  }
  if (type == "lingtu.dds.Odometry") {
    const auto &s = *static_cast<lingtu_dds_Odometry *>(sample);
    return "{" + header(s.header) + ",\"child_frame_id\":" + text(s.child_frame_id) +
           ",\"pose\":" + pose(s.pose.pose) + ",\"twist\":" + twist(s.twist.twist) + "}";
  }
  if (type == "lingtu.dds.JointState") {
    const auto &s = *static_cast<lingtu_dds_JointState *>(sample);
    auto out = "{" + header(s.header) + ",\"robot_model\":" + text(s.robot_model) +
               ",\"joint_count\":" + std::to_string(s.names._length) + ",\"names\":[";
    for (std::uint32_t i = 0; i < std::min<std::uint32_t>(s.names._length, 32); ++i) {
      if (i)
        out += ',';
      out += text(s.names._buffer[i]);
    }
    return out + "],\"position_rad\":" + numeric_sequence(s.position) +
           ",\"velocity_rad_s\":" + numeric_sequence(s.velocity) +
           ",\"estimated_torque_nm\":" + numeric_sequence(s.effort) + ",\"arrays_truncated\":" +
           (s.names._length > 32 || s.position._length > 32 || s.velocity._length > 32 ||
                    s.effort._length > 32
                ? "true"
                : "false") +
           "}";
  }
  if (type == "lingtu.dds.FinalVelocityCommand") {
    const auto &s = *static_cast<lingtu_dds_FinalVelocityCommand *>(sample);
    return "{\"output_seq\":" + std::to_string(s.output_seq) +
           ",\"source_wall_ns\":" + std::to_string(s.source_wall_ns) +
           ",\"twist\":" + twist(s.twist) + "}";
  }
  if (type == "lingtu.dds.DriverControlState") {
    const auto &s = *static_cast<lingtu_dds_DriverControlState *>(sample);
    return "{" + header(s.header) + ",\"connected\":" + (s.connected ? "true" : "false") +
           ",\"ready\":" + (s.ready ? "true" : "false") +
           ",\"motors_enabled\":" + (s.motors_enabled ? "true" : "false") +
           ",\"control_assured\":" + (s.control_assured ? "true" : "false") +
           ",\"zero_output\":" + (s.zero_output ? "true" : "false") +
           ",\"owner\":" + text(s.owner) + ",\"reason\":" + text(s.reason) + "}";
  }
  if (type == "lingtu.dds.NavigationState") {
    const auto &s = *static_cast<lingtu_dds_NavigationState *>(sample);
    return "{" + header(s.header) + ",\"control_mode\":" + std::to_string(s.control_mode) +
           ",\"lifecycle_state\":" + std::to_string(s.lifecycle_state) +
           ",\"planning_state\":" + std::to_string(s.planning_state) +
           ",\"execution_state\":" + std::to_string(s.execution_state) +
           ",\"recovery_state\":" + std::to_string(s.recovery_state) +
           ",\"progress\":" + json_number(s.progress) + ",\"active_task_id\":" + text(s.active_task_id) +
           ",\"hold_reason\":" + text(s.hold_reason) + ",\"failure_code\":" + text(s.failure_code) +
           "}";
  }
  if (type == "lingtu.dds.Float32")
    return "{\"data\":" + json_number(static_cast<lingtu_dds_Float32 *>(sample)->data) + "}";
  if (type == "lingtu.dds.Text")
    return "{\"data\":" + text(static_cast<lingtu_dds_Text *>(sample)->data) + "}";
  if (type == "lingtu.dds.Path") {
    const auto &s = *static_cast<lingtu_dds_Path *>(sample);
    auto out = "{" + header(s.header) + ",\"pose_count\":" + std::to_string(s.poses._length);
    if (s.poses._length)
      out += ",\"start\":" + pose(s.poses._buffer[0].pose) +
             ",\"end\":" + pose(s.poses._buffer[s.poses._length - 1].pose);
    return out + ",\"payload_omitted\":true}";
  }
  if (type == "lingtu.dds.GnssFix") {
    const auto &s = *static_cast<lingtu_dds_GnssFix *>(sample);
    return "{" + header(s.header) + ",\"latitude\":" + json_number(s.latitude) +
           ",\"longitude\":" + json_number(s.longitude) + ",\"altitude_m\":" + json_number(s.altitude) +
           ",\"fix_type\":" + std::to_string(s.fix_type) +
           ",\"satellites\":" + std::to_string(s.num_sat) + "}";
  }
  if (type == "lingtu.dds.CameraInfo") {
    const auto &s = *static_cast<lingtu_dds_CameraInfo *>(sample);
    return "{" + header(s.header) + ",\"width\":" + std::to_string(s.width) +
           ",\"height\":" + std::to_string(s.height) + ",\"depth_scale\":" + json_number(s.depth_scale) +
           ",\"fx\":" + json_number(s.k[0]) + ",\"fy\":" + json_number(s.k[4]) + ",\"cx\":" + json_number(s.k[2]) +
           ",\"cy\":" + json_number(s.k[5]) + "}";
  }
  return "{}";
}

std::string message_preview(const TopicBinding &binding, const ddsi_serdata *data) {
  if (metadata_type(binding.contract->idl_type)) {
    try { Prefix prefix(data); return prefix_preview(binding, prefix); }
    catch (const std::exception &) { return "{\"payload_omitted\":true}"; }
  }
  void *sample = binding.allocate_sample();
  if (!sample) throw std::runtime_error("DDS preview sample allocation failed");
  std::memset(sample, 0, binding.descriptor->m_size);
  const auto release = [&binding](void *value) { binding.free_sample(value); };
  std::unique_ptr<void, decltype(release)> owned(sample, release);
  if (!ddsi_serdata_to_sample(data, sample, nullptr, nullptr)) return "{\"decode_error\":true}";
  auto mapped = typed_preview(binding, sample);
  if (mapped != "{}") return mapped;
  std::array<char, 1024> printed{};
  ddsi_serdata_print(data, printed.data(), printed.size());
  printed.back() = '\0';
  return "{\"sample\":" + text(printed.data()) + ",\"text_preview\":true}";
}

std::string message_preview(const TopicBinding &binding, const std::byte *data, std::size_t size) {
  if (size <= 4 || size > std::numeric_limits<std::uint32_t>::max() ||
      data[0] != std::byte{0} || data[1] != std::byte{1})
    throw std::runtime_error("unsupported or invalid CDR envelope");
  if (metadata_type(binding.contract->idl_type)) {
    Prefix prefix(data, size);
    return prefix_preview(binding, prefix);
  }
  // These are the same mapped types as live preview. Unmapped events remain
  // visible in live text inspection, but have no numeric chart contract yet.
  const std::string_view type(binding.contract->idl_type);
  if (type != "lingtu.dds.Imu" && type != "lingtu.dds.Odometry" &&
      type != "lingtu.dds.JointState" && type != "lingtu.dds.FinalVelocityCommand" &&
      type != "lingtu.dds.DriverControlState" && type != "lingtu.dds.NavigationState" &&
      type != "lingtu.dds.Float32" && type != "lingtu.dds.Text" &&
      type != "lingtu.dds.Path" && type != "lingtu.dds.GnssFix" && type != "lingtu.dds.CameraInfo")
    return "{}";
  auto sample = decode_recorded_sample(binding, data, size);
  return typed_preview(binding, sample.get());
}

RecordedSample decode_recorded_sample(const TopicBinding &binding, const std::byte *data, std::size_t size) {
  if (size <= 4 || size > std::numeric_limits<std::uint32_t>::max() ||
      data[0] != std::byte{0} || data[1] != std::byte{1})
    throw std::runtime_error("unsupported or invalid CDR envelope");
  std::vector<std::byte> normalized(data + 4, data + size);
  RecordedSample sample(binding.allocate_sample(), binding.free_sample);
  if (!sample) throw std::runtime_error("sample allocation failed");
  std::memset(sample.get(), 0, binding.descriptor->m_size);
  dds_istream_t stream;
#ifdef LINGTU_CDR_STREAM_ALLOCATOR
  const dds_cdrstream_allocator allocator{dds_alloc, dds_realloc, dds_free};
  dds_cdrstream_desc descriptor{};
  dds_cdrstream_desc_from_topic_desc(&descriptor, binding.descriptor);
  std::uint32_t actual_size{};
  const bool valid = dds_stream_normalize(normalized.data(), static_cast<std::uint32_t>(normalized.size()),
                                         false, 1, &descriptor, false, &actual_size);
  if (!valid) {
    dds_cdrstream_desc_fini(&descriptor, &allocator);
    throw std::runtime_error("invalid typed CDR sample");
  }
  dds_istream_init(&stream, actual_size, normalized.data(), 1);
  dds_stream_read_sample(&stream, sample.get(), &allocator, &descriptor);
  dds_istream_fini(&stream);
  dds_cdrstream_desc_fini(&descriptor, &allocator);
#else
  std::uint32_t offset{};
  if (!dds_stream_normalize_data(reinterpret_cast<char *>(normalized.data()), &offset,
      static_cast<std::uint32_t>(normalized.size()), false, 1, binding.descriptor->m_ops))
    throw std::runtime_error("invalid typed CDR sample");
  dds_istream_init(&stream, static_cast<std::uint32_t>(normalized.size()), normalized.data(), 1);
  dds_stream_read(&stream, static_cast<char *>(sample.get()), binding.descriptor->m_ops);
  dds_istream_fini(&stream);
#endif
  return sample;
}

namespace {
// This small walker consumes JSON generated above, not arbitrary user JSON.
// It keeps numeric nulls and array indices without turning booleans into 0/1.
class NumericWalker {
 public:
  explicit NumericWalker(const std::string &json) : json_(json) {}
  NumericPreview read() { value(""); return std::move(result_); }
 private:
  void space() { while (position_ < json_.size() && json_[position_] <= ' ') ++position_; }
  char peek() { space(); return position_ < json_.size() ? json_[position_] : '\0'; }
  std::string string() {
    if (peek() != '"') throw std::runtime_error("invalid generated preview");
    ++position_;
    std::string out;
    while (position_ < json_.size()) {
      char c = json_[position_++];
      if (c == '"') return out;
      if (c == '\\') {
        if (position_ == json_.size()) break;
        c = json_[position_++];
        if (c == 'u') { position_ += 4; out += '?'; continue; }
      }
      out += c;
    }
    throw std::runtime_error("invalid generated preview string");
  }
  void value(const std::string &path) {
    const auto token = peek();
    if (token == '{') {
      ++position_;
      while (peek() != '}') {
        auto key = string();
        if (peek() != ':') throw std::runtime_error("invalid generated preview object");
        ++position_;
        value(path.empty() ? key : path + '.' + key);
        if (peek() != ',') break;
        ++position_;
      }
      if (peek() != '}') throw std::runtime_error("invalid generated preview object end");
      ++position_;
    } else if (token == '[') {
      ++position_;
      std::size_t index = 0;
      while (peek() != ']') {
        value(path + '[' + std::to_string(index++) + ']');
        if (peek() != ',') break;
        ++position_;
      }
      if (peek() != ']') throw std::runtime_error("invalid generated preview array end");
      ++position_;
    } else if (token == '"') {
      auto content = string();
      if (path == "frame_id") result_.frame_id = std::move(content);
    } else {
      const auto begin = position_;
      while (position_ < json_.size() && json_[position_] != ',' && json_[position_] != '}' &&
             json_[position_] != ']' && json_[position_] > ' ') ++position_;
      const auto literal = json_.substr(begin, position_ - begin);
      if (literal == "null") result_.values.push_back({path, std::nullopt});
      else if (!literal.empty() && (literal[0] == '-' || (literal[0] >= '0' && literal[0] <= '9'))) {
        std::istringstream input(literal);
        input.imbue(std::locale::classic());
        double number{};
        if (!(input >> number)) throw std::runtime_error("invalid generated preview number");
        result_.values.push_back({path, std::isfinite(number) ? std::optional<double>(number) : std::nullopt});
      } else if (literal != "true" && literal != "false")
        throw std::runtime_error("invalid generated preview value");
    }
  }
  const std::string &json_;
  std::size_t position_{};
  NumericPreview result_;
};
}  // namespace
NumericPreview numeric_preview(const std::string &preview) { return NumericWalker(preview).read(); }
std::string numeric_field_unit(std::string_view path) {
  if (path.find("linear_acceleration_m_s2") == 0) return "m/s²";
  if (path.find("angular_velocity_rad_s") == 0 || path.find("velocity_rad_s[") == 0 ||
      path.find("twist.angular.") == 0) return "rad/s";
  if (path.find("position_rad[") == 0) return "rad";
  if (path.find("estimated_torque_nm[") == 0) return "N·m";
  if (path.find("twist.linear.") == 0) return "m/s";
  if (path.find("pose.position.") == 0 || path.find("start.position.") == 0 ||
      path.find("end.position.") == 0 || path == "altitude_m") return "m";
  if (path == "latitude" || path == "longitude") return "deg";
  if (path == "stamp_sec") return "s";
  if (path == "stamp_nanosec" || path == "source_wall_ns") return "ns";
  return "";
}
}  // namespace lingtu::recording
