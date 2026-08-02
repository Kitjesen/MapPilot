#include "lingtu/recording/camera_recording.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "mcap/reader.hpp"
#include "mcap/writer.hpp"

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

namespace lingtu::recording {
namespace {

inline constexpr const char *kCameraSchemaName = "lingtu.recording.CameraSegment";
inline constexpr const char *kCameraSchema = R"json({
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "LingTu Camera Segment",
  "type": "object",
  "required": ["version", "index", "camera_id", "relative_path", "codec", "source_encoding", "width", "height", "start_time_ns", "end_time_ns", "first_sequence", "last_sequence", "frame_count", "dropped_frames", "byte_size", "crc32", "frame_id", "fx", "fy", "cx", "cy", "depth_scale", "dist_k1", "dist_k2", "dist_p1", "dist_p2", "dist_k3"],
  "properties": {
    "version": {"const": 1},
    "index": {"type": "integer", "minimum": 0},
    "camera_id": {"type": "string"},
    "relative_path": {"type": "string"},
    "codec": {"type": "string"},
    "source_encoding": {"type": "string"},
    "width": {"type": "integer", "minimum": 1},
    "height": {"type": "integer", "minimum": 1},
    "start_time_ns": {"type": "integer", "minimum": 1},
    "end_time_ns": {"type": "integer", "minimum": 1},
    "first_sequence": {"type": "integer", "minimum": 1},
    "last_sequence": {"type": "integer", "minimum": 1},
    "frame_count": {"type": "integer", "minimum": 1},
    "dropped_frames": {"type": "integer", "minimum": 0},
    "byte_size": {"type": "integer", "minimum": 1},
    "crc32": {"type": "integer", "minimum": 0, "maximum": 4294967295},
    "frame_id": {"type": "string"},
    "fx": {"type": "number"},
    "fy": {"type": "number"},
    "cx": {"type": "number"},
    "cy": {"type": "number"},
    "depth_scale": {"type": "number", "exclusiveMinimum": 0},
    "dist_k1": {"type": "number"},
    "dist_k2": {"type": "number"},
    "dist_p1": {"type": "number"},
    "dist_p2": {"type": "number"},
    "dist_k3": {"type": "number"}
  },
  "additionalProperties": false
})json";

void validate_camera_id(const std::string &camera_id) {
  if (camera_id.empty()) {
    throw std::invalid_argument("camera id must not be empty");
  }
  for (const char character : camera_id) {
    const bool valid =
        (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
        (character >= '0' && character <= '9') || character == '_' || character == '-';
    if (!valid) {
      throw std::invalid_argument("camera id may contain only letters, digits, '_' and '-'");
    }
  }
}

void validate_relative_segment_path(const std::string &value) {
  const std::filesystem::path path(value);
  if (value.empty() || path.is_absolute() || path.has_root_path() ||
      path.lexically_normal().generic_string() != value) {
    throw std::runtime_error("camera index contains an unsafe segment path: " + value);
  }
  for (const auto &component : path) {
    if (component == ".." || component == ".") {
      throw std::runtime_error("camera index contains an unsafe segment path: " + value);
    }
  }
}

std::uint32_t bytes_per_pixel(const std::string &encoding) {
  if (encoding == "rgb8" || encoding == "bgr8") {
    return 3;
  }
  return 0;
}

void validate_frame(const CameraFrame &frame) {
  const auto bpp = bytes_per_pixel(frame.encoding);
  if (frame.sequence == 0 || frame.timestamp_ns == 0 || frame.width == 0 || frame.height == 0 ||
      bpp == 0 || frame.stride < frame.width * bpp ||
      frame.payload.size() != static_cast<std::size_t>(frame.stride) * frame.height ||
      !std::isfinite(frame.fx) || !std::isfinite(frame.fy) || !std::isfinite(frame.cx) ||
      !std::isfinite(frame.cy) || !std::isfinite(frame.depth_scale) || frame.depth_scale <= 0.0 ||
      !std::isfinite(frame.dist_k1) || !std::isfinite(frame.dist_k2) ||
      !std::isfinite(frame.dist_p1) || !std::isfinite(frame.dist_p2) ||
      !std::isfinite(frame.dist_k3)) {
    throw std::runtime_error("camera frame is not a valid rgb8/bgr8 image");
  }
}

bool same_stream(const CameraStreamDescription &stream, const CameraFrame &frame) {
  return stream.width == frame.width && stream.height == frame.height &&
         stream.stride == frame.stride && stream.encoding == frame.encoding &&
         stream.frame_id == frame.frame_id && stream.fx == frame.fx && stream.fy == frame.fy &&
         stream.cx == frame.cx && stream.cy == frame.cy &&
         stream.depth_scale == frame.depth_scale && stream.dist_k1 == frame.dist_k1 &&
         stream.dist_k2 == frame.dist_k2 && stream.dist_p1 == frame.dist_p1 &&
         stream.dist_p2 == frame.dist_p2 && stream.dist_k3 == frame.dist_k3;
}

CameraStreamDescription describe(const CameraFrame &frame, double nominal_fps) {
  return CameraStreamDescription{frame.width,    frame.height,      frame.stride,  frame.encoding,
                                 frame.frame_id, frame.fx,          frame.fy,      frame.cx,
                                 frame.cy,       frame.depth_scale, frame.dist_k1, frame.dist_k2,
                                 frame.dist_p1,  frame.dist_p2,     frame.dist_k3, nominal_fps};
}

std::string json_escape(std::string_view value) {
  std::string output;
  output.reserve(value.size() + 8);
  for (const unsigned char character : value) {
    switch (character) {
      case '\"':
        output += "\\\"";
        break;
      case '\\':
        output += "\\\\";
        break;
      case '\b':
        output += "\\b";
        break;
      case '\f':
        output += "\\f";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        if (character < 0x20) {
          throw std::runtime_error("camera index string contains a control character");
        }
        output.push_back(static_cast<char>(character));
    }
  }
  return output;
}

std::string segment_json(const CameraSegment &segment) {
  const auto number = [](double value) {
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return output.str();
  };
  return "{\"version\":1,\"index\":" + std::to_string(segment.index) + ",\"camera_id\":\"" +
         json_escape(segment.camera_id) + "\",\"relative_path\":\"" +
         json_escape(segment.relative_path) + "\",\"codec\":\"" + json_escape(segment.codec) +
         "\",\"source_encoding\":\"" + json_escape(segment.source_encoding) +
         "\",\"width\":" + std::to_string(segment.width) +
         ",\"height\":" + std::to_string(segment.height) +
         ",\"start_time_ns\":" + std::to_string(segment.start_time_ns) +
         ",\"end_time_ns\":" + std::to_string(segment.end_time_ns) +
         ",\"first_sequence\":" + std::to_string(segment.first_sequence) +
         ",\"last_sequence\":" + std::to_string(segment.last_sequence) +
         ",\"frame_count\":" + std::to_string(segment.frame_count) +
         ",\"dropped_frames\":" + std::to_string(segment.dropped_frames) +
         ",\"byte_size\":" + std::to_string(segment.byte_size) +
         ",\"crc32\":" + std::to_string(segment.crc32) + ",\"frame_id\":\"" +
         json_escape(segment.frame_id) + "\",\"fx\":" + number(segment.fx) +
         ",\"fy\":" + number(segment.fy) + ",\"cx\":" + number(segment.cx) +
         ",\"cy\":" + number(segment.cy) + ",\"depth_scale\":" + number(segment.depth_scale) +
         ",\"dist_k1\":" + number(segment.dist_k1) + ",\"dist_k2\":" + number(segment.dist_k2) +
         ",\"dist_p1\":" + number(segment.dist_p1) + ",\"dist_p2\":" + number(segment.dist_p2) +
         ",\"dist_k3\":" + number(segment.dist_k3) + "}";
}

class JsonCursor {
 public:
  explicit JsonCursor(std::string_view input) : input_(input) {}

  void literal(std::string_view expected) {
    if (input_.substr(position_, expected.size()) != expected) {
      throw std::runtime_error("camera index JSON is malformed");
    }
    position_ += expected.size();
  }

  std::string string_value() {
    literal("\"");
    std::string output;
    while (position_ < input_.size()) {
      const char character = input_[position_++];
      if (character == '\"') {
        return output;
      }
      if (character != '\\') {
        if (static_cast<unsigned char>(character) < 0x20) {
          throw std::runtime_error("camera index JSON string contains a control character");
        }
        output.push_back(character);
        continue;
      }
      if (position_ == input_.size()) {
        throw std::runtime_error("camera index JSON has a truncated escape");
      }
      switch (input_[position_++]) {
        case '\"':
          output.push_back('\"');
          break;
        case '\\':
          output.push_back('\\');
          break;
        case 'b':
          output.push_back('\b');
          break;
        case 'f':
          output.push_back('\f');
          break;
        case 'n':
          output.push_back('\n');
          break;
        case 'r':
          output.push_back('\r');
          break;
        case 't':
          output.push_back('\t');
          break;
        default:
          throw std::runtime_error("camera index JSON uses an unsupported escape");
      }
    }
    throw std::runtime_error("camera index JSON has an unterminated string");
  }

  std::uint64_t unsigned_value() {
    const auto start = position_;
    while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
      ++position_;
    }
    if (start == position_) {
      throw std::runtime_error("camera index JSON integer is missing");
    }
    std::uint64_t result = 0;
    for (std::size_t index = start; index < position_; ++index) {
      const auto digit = static_cast<std::uint64_t>(input_[index] - '0');
      if (result > (std::numeric_limits<std::uint64_t>::max() - digit) / 10) {
        throw std::runtime_error("camera index JSON integer overflows uint64");
      }
      result = result * 10 + digit;
    }
    return result;
  }

  double double_value() {
    const auto start = position_;
    if (position_ < input_.size() && (input_[position_] == '-' || input_[position_] == '+')) {
      ++position_;
    }
    while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
      ++position_;
    }
    if (position_ < input_.size() && input_[position_] == '.') {
      ++position_;
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
      }
    }
    if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
      ++position_;
      if (position_ < input_.size() && (input_[position_] == '-' || input_[position_] == '+')) {
        ++position_;
      }
      while (position_ < input_.size() && input_[position_] >= '0' && input_[position_] <= '9') {
        ++position_;
      }
    }
    if (start == position_) {
      throw std::runtime_error("camera index JSON number is missing");
    }
    std::istringstream input(std::string(input_.substr(start, position_ - start)));
    input.imbue(std::locale::classic());
    input >> std::noskipws;
    double result = 0.0;
    input >> result;
    if (!input || !input.eof() || !std::isfinite(result)) {
      throw std::runtime_error("camera index JSON number is invalid");
    }
    return result;
  }

  bool at_end() const noexcept { return position_ == input_.size(); }

 private:
  std::string_view input_;
  std::size_t position_{0};
};

template <typename T>
T narrow_integer(std::uint64_t value, const char *field) {
  if (value > std::numeric_limits<T>::max()) {
    throw std::runtime_error(std::string("camera index ") + field + " is out of range");
  }
  return static_cast<T>(value);
}

CameraSegment parse_segment_json(std::string_view json) {
  JsonCursor cursor(json);
  CameraSegment segment;
  cursor.literal("{\"version\":");
  if (cursor.unsigned_value() != 1) {
    throw std::runtime_error("unsupported camera index version");
  }
  cursor.literal(",\"index\":");
  segment.index = narrow_integer<std::uint32_t>(cursor.unsigned_value(), "index");
  cursor.literal(",\"camera_id\":");
  segment.camera_id = cursor.string_value();
  cursor.literal(",\"relative_path\":");
  segment.relative_path = cursor.string_value();
  cursor.literal(",\"codec\":");
  segment.codec = cursor.string_value();
  cursor.literal(",\"source_encoding\":");
  segment.source_encoding = cursor.string_value();
  cursor.literal(",\"width\":");
  segment.width = narrow_integer<std::uint32_t>(cursor.unsigned_value(), "width");
  cursor.literal(",\"height\":");
  segment.height = narrow_integer<std::uint32_t>(cursor.unsigned_value(), "height");
  cursor.literal(",\"start_time_ns\":");
  segment.start_time_ns = cursor.unsigned_value();
  cursor.literal(",\"end_time_ns\":");
  segment.end_time_ns = cursor.unsigned_value();
  cursor.literal(",\"first_sequence\":");
  segment.first_sequence = cursor.unsigned_value();
  cursor.literal(",\"last_sequence\":");
  segment.last_sequence = cursor.unsigned_value();
  cursor.literal(",\"frame_count\":");
  segment.frame_count = cursor.unsigned_value();
  cursor.literal(",\"dropped_frames\":");
  segment.dropped_frames = cursor.unsigned_value();
  cursor.literal(",\"byte_size\":");
  segment.byte_size = cursor.unsigned_value();
  cursor.literal(",\"crc32\":");
  segment.crc32 = narrow_integer<std::uint32_t>(cursor.unsigned_value(), "crc32");
  cursor.literal(",\"frame_id\":");
  segment.frame_id = cursor.string_value();
  cursor.literal(",\"fx\":");
  segment.fx = cursor.double_value();
  cursor.literal(",\"fy\":");
  segment.fy = cursor.double_value();
  cursor.literal(",\"cx\":");
  segment.cx = cursor.double_value();
  cursor.literal(",\"cy\":");
  segment.cy = cursor.double_value();
  cursor.literal(",\"depth_scale\":");
  segment.depth_scale = cursor.double_value();
  cursor.literal(",\"dist_k1\":");
  segment.dist_k1 = cursor.double_value();
  cursor.literal(",\"dist_k2\":");
  segment.dist_k2 = cursor.double_value();
  cursor.literal(",\"dist_p1\":");
  segment.dist_p1 = cursor.double_value();
  cursor.literal(",\"dist_p2\":");
  segment.dist_p2 = cursor.double_value();
  cursor.literal(",\"dist_k3\":");
  segment.dist_k3 = cursor.double_value();
  cursor.literal("}");
  if (!cursor.at_end()) {
    throw std::runtime_error("camera index JSON has trailing data");
  }
  validate_camera_id(segment.camera_id);
  validate_relative_segment_path(segment.relative_path);
  if (segment.codec.empty() || bytes_per_pixel(segment.source_encoding) == 0 ||
      segment.width == 0 || segment.height == 0 || segment.start_time_ns == 0 ||
      segment.end_time_ns < segment.start_time_ns || segment.frame_count == 0 ||
      segment.byte_size == 0 || segment.first_sequence == 0 ||
      segment.last_sequence < segment.first_sequence || segment.frame_id.empty() ||
      segment.depth_scale <= 0.0) {
    throw std::runtime_error("camera index segment metadata is invalid");
  }
  return segment;
}

std::uint32_t crc32_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open camera segment for checksum: " + path.string());
  }
  std::uint32_t crc = 0xFFFFFFFFu;
  char buffer[64 * 1024];
  while (input) {
    input.read(buffer, sizeof(buffer));
    const auto count = input.gcount();
    for (std::streamsize index = 0; index < count; ++index) {
      crc ^= static_cast<unsigned char>(buffer[index]);
      for (int bit = 0; bit < 8; ++bit) {
        const std::uint32_t mask = 0u - (crc & 1u);
        crc = (crc >> 1u) ^ (0xEDB88320u & mask);
      }
    }
  }
  if (!input.eof()) {
    throw std::runtime_error("failed to read camera segment for checksum: " + path.string());
  }
  return crc ^ 0xFFFFFFFFu;
}

void durable_sync_file(const std::filesystem::path &path) {
#if defined(_WIN32)
  const int descriptor = _wopen(path.c_str(), _O_RDWR | _O_BINARY);
  if (descriptor < 0) {
    throw std::runtime_error("failed to open camera segment for durable sync: " + path.string());
  }
  const int status = _commit(descriptor);
  const int saved_errno = errno;
  _close(descriptor);
#else
  const int descriptor = ::open(path.c_str(), O_RDONLY);
  if (descriptor < 0) {
    throw std::runtime_error("failed to open camera segment for durable sync: " + path.string());
  }
  const int status = ::fdatasync(descriptor);
  const int saved_errno = errno;
  ::close(descriptor);
#endif
  if (status != 0) {
    throw std::runtime_error("failed to durably sync camera segment: " + path.string() + ": " +
                             std::strerror(saved_errno));
  }
}

void durable_sync_directory(const std::filesystem::path &directory) {
#if !defined(_WIN32)
  const int descriptor = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY);
  if (descriptor < 0) {
    throw std::runtime_error("failed to open camera directory for durable sync: " +
                             directory.string());
  }
  const int status = ::fsync(descriptor);
  const int saved_errno = errno;
  ::close(descriptor);
  if (status != 0) {
    throw std::runtime_error("failed to durably sync camera directory: " + directory.string() +
                             ": " + std::strerror(saved_errno));
  }
#else
  (void)directory;
#endif
}

class CameraDurableFileWriter final : public mcap::IWritable {
 public:
  explicit CameraDurableFileWriter(const std::filesystem::path &path) {
#if defined(_WIN32)
    file_ = _wfopen(path.c_str(), L"wb");
#else
    file_ = std::fopen(path.c_str(), "wb");
#endif
    if (file_ == nullptr) {
      throw std::runtime_error("failed to open camera MCAP temporary file: " + path.string() +
                               ": " + std::strerror(errno));
    }
  }

  ~CameraDurableFileWriter() override { close_noexcept(); }

  void handleWrite(const std::byte *data, std::uint64_t size) override {
    if (file_ == nullptr || std::fwrite(data, 1, static_cast<std::size_t>(size), file_) != size) {
      throw std::runtime_error("failed to write camera MCAP index");
    }
    size_ += size;
  }

  void end() override {
    durable_flush();
    if (std::fclose(file_) != 0) {
      file_ = nullptr;
      throw std::runtime_error("failed to close camera MCAP index");
    }
    file_ = nullptr;
  }

  void flush() override { durable_flush(); }
  std::uint64_t size() const override { return size_; }

  void close_noexcept() noexcept {
    if (file_ != nullptr) {
      std::fflush(file_);
      std::fclose(file_);
      file_ = nullptr;
    }
  }

 private:
  void durable_flush() {
    if (file_ == nullptr || std::fflush(file_) != 0) {
      throw std::runtime_error("failed to flush camera MCAP index");
    }
#if defined(_WIN32)
    if (_commit(_fileno(file_)) != 0) {
      throw std::runtime_error("failed to commit camera MCAP index");
    }
#else
    if (::fdatasync(fileno(file_)) != 0) {
      throw std::runtime_error("failed to sync camera MCAP index");
    }
#endif
  }

  std::FILE *file_{nullptr};
  std::uint64_t size_{0};
};

void write_camera_segment_index(const std::filesystem::path &index_path,
                                const std::vector<CameraSegment> &segments) {
  if (segments.empty()) {
    throw std::runtime_error("cannot write an empty camera segment index");
  }
  const auto temporary_path = std::filesystem::path(index_path.string() + ".tmp");
  if (std::filesystem::exists(index_path) || std::filesystem::exists(temporary_path)) {
    throw std::runtime_error("camera MCAP index or temporary file already exists: " +
                             index_path.string());
  }
  CameraDurableFileWriter sink(temporary_path);
  mcap::McapWriter writer;
  mcap::McapWriterOptions options(kCameraMcapProfile);
  options.library = "lingtu-camera-recorder/0.1";
  options.compression = mcap::Compression::None;
  options.chunkSize = 64 * 1024;
  options.noChunkCRC = false;
  options.enableDataCRC = true;
  writer.open(sink, options);

  mcap::Schema schema(kCameraSchemaName, "jsonschema", kCameraSchema);
  writer.addSchema(schema);
  const std::string topic = "/camera/" + segments.front().camera_id + "/segments";
  mcap::Channel channel(topic, "json", schema.id,
                        {{"lingtu.camera_id", segments.front().camera_id},
                         {"lingtu.format_version", "1"},
                         {"lingtu.payload_location", "external-segment"}});
  writer.addChannel(channel);
  const auto metadata_status = writer.write(
      mcap::Metadata{"lingtu.camera.session",
                     {{"format", "segmented-video-index"}, {"profile", kCameraMcapProfile}}});
  if (!metadata_status.ok()) {
    throw std::runtime_error("failed to write camera MCAP metadata: " + metadata_status.message);
  }
  std::uint32_t sequence = 1;
  for (const auto &segment : segments) {
    const std::string json = segment_json(segment);
    mcap::Message message;
    message.channelId = channel.id;
    message.sequence = sequence++;
    message.logTime = segment.end_time_ns;
    message.publishTime = segment.start_time_ns;
    message.data = reinterpret_cast<const std::byte *>(json.data());
    message.dataSize = json.size();
    const auto status = writer.write(message);
    if (!status.ok()) {
      throw std::runtime_error("failed to write camera MCAP message: " + status.message);
    }
  }
  writer.close();
  std::filesystem::rename(temporary_path, index_path);
  durable_sync_directory(index_path.parent_path());
}

}  // namespace

bool CameraSegment::operator==(const CameraSegment &other) const noexcept {
  return index == other.index && camera_id == other.camera_id &&
         relative_path == other.relative_path && codec == other.codec &&
         source_encoding == other.source_encoding && width == other.width &&
         height == other.height && start_time_ns == other.start_time_ns &&
         end_time_ns == other.end_time_ns && first_sequence == other.first_sequence &&
         last_sequence == other.last_sequence && frame_count == other.frame_count &&
         dropped_frames == other.dropped_frames && byte_size == other.byte_size &&
         crc32 == other.crc32 && frame_id == other.frame_id && fx == other.fx && fy == other.fy &&
         cx == other.cx && cy == other.cy && depth_scale == other.depth_scale &&
         dist_k1 == other.dist_k1 && dist_k2 == other.dist_k2 && dist_p1 == other.dist_p1 &&
         dist_p2 == other.dist_p2 && dist_k3 == other.dist_k3;
}

std::filesystem::path camera_index_path(const std::filesystem::path &session_directory,
                                        const std::string &camera_id) {
  validate_camera_id(camera_id);
  if (session_directory.empty()) {
    throw std::invalid_argument("camera session directory must not be empty");
  }
  return session_directory / ("camera_" + camera_id + ".mcap");
}

std::filesystem::path resolve_camera_segment_path(const std::filesystem::path &index_path,
                                                  const std::string &relative_path) {
  validate_relative_segment_path(relative_path);
  const auto parent =
      index_path.parent_path().empty() ? std::filesystem::current_path() : index_path.parent_path();
  const auto root = std::filesystem::weakly_canonical(parent);
  const auto candidate =
      std::filesystem::weakly_canonical(parent / std::filesystem::path(relative_path));
  auto root_component = root.begin();
  auto candidate_component = candidate.begin();
  for (; root_component != root.end(); ++root_component, ++candidate_component) {
    if (candidate_component == candidate.end() || *root_component != *candidate_component) {
      throw std::runtime_error("camera segment path escapes the session directory: " +
                               relative_path);
    }
  }
  if (candidate_component == candidate.end()) {
    throw std::runtime_error("camera segment path resolves to the session directory");
  }
  return candidate;
}

CameraSegmentRecorder::CameraSegmentRecorder(CameraRecordingOptions options,
                                             CameraFrameSource &source,
                                             CameraSegmentEncoder &encoder)
    : options_(std::move(options)), source_(source), encoder_(encoder) {
  validate_camera_id(options_.camera_id);
  if (options_.session_directory.empty() || options_.segment_duration_ns == 0 ||
      !std::isfinite(options_.nominal_fps) || options_.nominal_fps <= 0.0) {
    throw std::invalid_argument("camera recording options are invalid");
  }
  const auto extension = encoder_.container_extension();
  if (encoder_.codec().empty() || extension.size() < 2 || extension.front() != '.' ||
      extension.find('/', 1) != std::string::npos || extension.find('\\', 1) != std::string::npos) {
    throw std::invalid_argument("camera encoder codec or container extension is invalid");
  }
}

std::vector<CameraSegment> CameraSegmentRecorder::record() {
  const auto index_path = camera_index_path(options_.session_directory, options_.camera_id);
  if (std::filesystem::exists(index_path) ||
      std::filesystem::exists(index_path.string() + ".tmp")) {
    throw std::runtime_error("camera recording index already exists: " + index_path.string());
  }
  const auto segment_directory = options_.session_directory / ("camera_" + options_.camera_id);
  std::filesystem::create_directories(segment_directory);

  std::vector<CameraSegment> segments;
  CameraSegment active;
  CameraStreamDescription active_stream;
  std::filesystem::path active_final_path;
  std::filesystem::path active_temporary_path;
  bool encoder_open = false;
  std::uint64_t previous_timestamp = 0;
  std::uint64_t previous_sequence = 0;

  const auto finish_segment = [&]() {
    if (!encoder_open) {
      return;
    }
    encoder_.finish();
    encoder_open = false;
    if (!std::filesystem::exists(active_temporary_path)) {
      throw std::runtime_error("camera encoder did not create its temporary segment");
    }
    active.byte_size = std::filesystem::file_size(active_temporary_path);
    if (active.byte_size == 0) {
      throw std::runtime_error("camera encoder produced an empty segment");
    }
    active.crc32 = crc32_file(active_temporary_path);
    durable_sync_file(active_temporary_path);
    std::filesystem::rename(active_temporary_path, active_final_path);
    durable_sync_directory(active_final_path.parent_path());
    segments.push_back(active);
  };

  try {
    CameraFrame frame;
    while (source_.next(frame)) {
      validate_frame(frame);
      if (previous_timestamp != 0 && frame.timestamp_ns < previous_timestamp) {
        throw std::runtime_error("camera frame timestamp regressed");
      }
      if (previous_sequence != 0 && frame.sequence <= previous_sequence) {
        throw std::runtime_error("camera frame sequence did not increase");
      }

      const bool elapsed =
          encoder_open && frame.timestamp_ns - active.start_time_ns >= options_.segment_duration_ns;
      const bool changed = encoder_open && !same_stream(active_stream, frame);
      if (elapsed || changed) {
        finish_segment();
      }

      if (!encoder_open) {
        active = CameraSegment{};
        active.index = static_cast<std::uint32_t>(segments.size());
        active.camera_id = options_.camera_id;
        active.codec = encoder_.codec();
        active.source_encoding = frame.encoding;
        active.width = frame.width;
        active.height = frame.height;
        active.start_time_ns = frame.timestamp_ns;
        active.end_time_ns = frame.timestamp_ns;
        active.first_sequence = frame.sequence;
        active.last_sequence = frame.sequence;
        active.frame_id = frame.frame_id;
        active.fx = frame.fx;
        active.fy = frame.fy;
        active.cx = frame.cx;
        active.cy = frame.cy;
        active.depth_scale = frame.depth_scale;
        active.dist_k1 = frame.dist_k1;
        active.dist_k2 = frame.dist_k2;
        active.dist_p1 = frame.dist_p1;
        active.dist_p2 = frame.dist_p2;
        active.dist_k3 = frame.dist_k3;
        const auto stem = [&]() {
          char buffer[32];
          std::snprintf(buffer, sizeof(buffer), "%06u", active.index);
          return std::string(buffer);
        }();
        active.relative_path = (std::filesystem::path("camera_" + options_.camera_id) /
                                (stem + encoder_.container_extension()))
                                   .generic_string();
        active_final_path = resolve_camera_segment_path(index_path, active.relative_path);
        active_temporary_path = active_final_path.string() + ".tmp";
        if (std::filesystem::exists(active_final_path) ||
            std::filesystem::exists(active_temporary_path)) {
          throw std::runtime_error("camera segment output already exists: " +
                                   active_final_path.string());
        }
        active_stream = describe(frame, options_.nominal_fps);
        encoder_.begin(active_temporary_path, active_stream);
        encoder_open = true;
      }

      if (previous_sequence != 0 && frame.sequence > previous_sequence + 1) {
        active.dropped_frames += frame.sequence - previous_sequence - 1;
      }
      encoder_.write(frame);
      ++active.frame_count;
      active.end_time_ns = frame.timestamp_ns;
      active.last_sequence = frame.sequence;
      previous_timestamp = frame.timestamp_ns;
      previous_sequence = frame.sequence;
    }
    finish_segment();
  } catch (...) {
    if (encoder_open) {
      encoder_.abort();
    }
    throw;
  }

  if (segments.empty()) {
    throw std::runtime_error("camera source produced no frames");
  }
  write_camera_segment_index(index_path, segments);
  return segments;
}

std::vector<CameraSegment> read_camera_segment_index(const std::filesystem::path &index_path) {
  mcap::McapReader reader;
  const auto open_status = reader.open(index_path.string());
  if (!open_status.ok()) {
    throw std::runtime_error("failed to open camera MCAP index: " + open_status.message);
  }
  if (!reader.header() || reader.header()->profile != kCameraMcapProfile) {
    throw std::runtime_error("MCAP profile is not lingtu.camera.v1");
  }

  std::string parse_error;
  std::vector<CameraSegment> segments;
  std::unordered_set<std::string> paths;
  std::string camera_id;
  for (const auto &view :
       reader.readMessages([&](const mcap::Status &status) { parse_error = status.message; })) {
    if (view.channel == nullptr || view.schema == nullptr ||
        view.channel->messageEncoding != "json" || view.schema->name != kCameraSchemaName ||
        view.schema->encoding != "jsonschema") {
      throw std::runtime_error("camera MCAP index has an unexpected channel or schema");
    }
    const auto metadata = view.channel->metadata.find("lingtu.camera_id");
    if (metadata == view.channel->metadata.end()) {
      throw std::runtime_error("camera MCAP channel is missing camera id metadata");
    }
    if (camera_id.empty()) {
      camera_id = metadata->second;
      validate_camera_id(camera_id);
    } else if (camera_id != metadata->second) {
      throw std::runtime_error("camera MCAP index contains multiple camera ids");
    }
    const std::string expected_topic = "/camera/" + camera_id + "/segments";
    if (view.channel->topic != expected_topic) {
      throw std::runtime_error("camera MCAP topic does not match camera id");
    }
    const std::string json(reinterpret_cast<const char *>(view.message.data),
                           static_cast<std::size_t>(view.message.dataSize));
    auto segment = parse_segment_json(json);
    if (segment.camera_id != camera_id || segment.index != segments.size() ||
        !paths.insert(segment.relative_path).second ||
        (!segments.empty() && segment.start_time_ns < segments.back().start_time_ns)) {
      throw std::runtime_error("camera MCAP segment ordering or identity is invalid");
    }
    segments.push_back(std::move(segment));
  }
  reader.close();
  if (!parse_error.empty()) {
    throw std::runtime_error("camera MCAP parse error: " + parse_error);
  }
  if (segments.empty()) {
    throw std::runtime_error("camera MCAP index contains no segments");
  }
  return segments;
}

std::vector<CameraVerificationIssue>
verify_camera_segments(const std::filesystem::path &index_path) {
  std::vector<CameraVerificationIssue> issues;
  for (const auto &segment : read_camera_segment_index(index_path)) {
    const auto path = resolve_camera_segment_path(index_path, segment.relative_path);
    try {
      if (!std::filesystem::is_regular_file(path)) {
        issues.push_back({path, "segment file is missing or is not a regular file"});
        continue;
      }
      if (std::filesystem::file_size(path) != segment.byte_size) {
        issues.push_back({path, "segment byte size differs from the MCAP index"});
        continue;
      }
      if (crc32_file(path) != segment.crc32) {
        issues.push_back({path, "segment CRC32 differs from the MCAP index"});
      }
    } catch (const std::exception &error) {
      issues.push_back({path, error.what()});
    }
  }
  return issues;
}

std::vector<std::filesystem::path>
extract_camera_window(const std::filesystem::path &index_path,
                      const std::filesystem::path &output_directory, std::uint64_t start_time_ns,
                      std::uint64_t end_time_ns) {
  if (output_directory.empty() || start_time_ns > end_time_ns) {
    throw std::invalid_argument("camera extraction window or output directory is invalid");
  }
  std::filesystem::create_directories(output_directory);
  std::vector<std::filesystem::path> extracted;
  for (const auto &segment : read_camera_segment_index(index_path)) {
    if (segment.end_time_ns < start_time_ns || segment.start_time_ns > end_time_ns) {
      continue;
    }
    const auto source = resolve_camera_segment_path(index_path, segment.relative_path);
    if (!std::filesystem::is_regular_file(source) ||
        std::filesystem::file_size(source) != segment.byte_size ||
        crc32_file(source) != segment.crc32) {
      throw std::runtime_error("camera segment failed verification before extraction: " +
                               source.string());
    }
    const auto destination = output_directory / source.filename();
    if (std::filesystem::exists(destination)) {
      throw std::runtime_error("camera extraction output already exists: " + destination.string());
    }
    std::filesystem::copy_file(source, destination);
    extracted.push_back(destination);
  }
  return extracted;
}

}  // namespace lingtu::recording
