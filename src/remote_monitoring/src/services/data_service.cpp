#include "remote_monitoring/services/data_service.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace remote_monitoring {
namespace services {

namespace {

constexpr double kDefaultCameraHz = 15.0;
constexpr double kDefaultMapHz = 1.0;
constexpr double kDefaultPointCloudHz = 10.0;
constexpr double kMinStreamHz = 0.1;
constexpr double kMaxStreamHz = 60.0;

bool IsTopicName(const std::string &name) {
  return !name.empty() && name.front() == '/';
}

std::string ReplaceAll(std::string input, const std::string &from,
                       const std::string &to) {
  if (from.empty()) {
    return input;
  }

  size_t start_pos = 0;
  while ((start_pos = input.find(from, start_pos)) != std::string::npos) {
    input.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
  return input;
}

std::string ApplyTemplate(
    const std::string &template_str,
    const std::unordered_map<std::string, std::string> &vars) {
  std::string result = template_str;
  for (const auto &pair : vars) {
    result = ReplaceAll(result, "{" + pair.first + "}", pair.second);
  }
  return result;
}

bool ReadFileToString(const std::string &path, std::string *output) {
  if (output == nullptr) {
    return false;
  }

  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::ostringstream ss;
  ss << file.rdbuf();
  *output = ss.str();
  return true;
}

std::vector<std::string> ReadLines(const std::string &path) {
  std::vector<std::string> lines;
  std::ifstream file(path);
  if (!file.is_open()) {
    return lines;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

bool FileExists(const std::string &path) {
  std::ifstream file(path);
  return file.good();
}

bool WaitForFile(const std::string &path, int timeout_ms) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (FileExists(path)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

// 将 PointCloud2 序列化为 [header(20B) + data]
// header 格式（小端）: width(4) height(4) point_step(4) row_step(4) num_fields(4)
// 客户端可通过 header 还原点云结构
std::string SerializePointCloud2WithMeta(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  if (!msg) return {};

  const uint32_t width = msg->width;
  const uint32_t height = msg->height;
  const uint32_t point_step = msg->point_step;
  const uint32_t row_step = msg->row_step;
  const uint32_t num_fields = static_cast<uint32_t>(msg->fields.size());

  // 20 字节元数据头 + 原始数据
  std::string result;
  result.resize(20 + msg->data.size());
  char *p = result.data();

  std::memcpy(p +  0, &width, 4);
  std::memcpy(p +  4, &height, 4);
  std::memcpy(p +  8, &point_step, 4);
  std::memcpy(p + 12, &row_step, 4);
  std::memcpy(p + 16, &num_fields, 4);
  std::memcpy(p + 20, msg->data.data(), msg->data.size());

  return result;
}

void AddDefaultProfiles(robot::v1::ResourceInfo *resource) {
  if (resource == nullptr) {
    return;
  }

  auto *low = resource->add_profiles();
  low->set_profile_name("low");
  low->set_frequency(5.0);
  low->set_compression(robot::v1::COMPRESSION_TYPE_NONE);
  low->set_max_bitrate_kbps(1024);

  auto *medium = resource->add_profiles();
  medium->set_profile_name("medium");
  medium->set_frequency(15.0);
  medium->set_compression(robot::v1::COMPRESSION_TYPE_NONE);
  medium->set_max_bitrate_kbps(4096);

  auto *high = resource->add_profiles();
  high->set_profile_name("high");
  high->set_frequency(30.0);
  high->set_compression(robot::v1::COMPRESSION_TYPE_NONE);
  high->set_max_bitrate_kbps(8192);
}

template <typename MsgT, typename PayloadExtractor>
grpc::Status StreamResource(
    rclcpp::Node *node, const std::string &topic, const rclcpp::QoS &qos,
    const robot::v1::ResourceId &resource_id, double frequency_hz,
    PayloadExtractor payload_extractor,
    const std::shared_ptr<std::atomic_bool> &cancel_flag,
    grpc::ServerContext *context,
    grpc::ServerWriter<robot::v1::DataChunk> *writer) {
  std::mutex mutex;
  std::condition_variable cv;
  typename MsgT::ConstSharedPtr latest_msg;
  bool has_new_msg = false;
  uint64_t sequence = 1;

  auto sub = node->create_subscription<MsgT>(
      topic, qos, [&](const typename MsgT::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        latest_msg = msg;
        has_new_msg = true;
        cv.notify_one();
      });
  (void)sub;

  const double hz = std::max(kMinStreamHz, std::min(kMaxStreamHz, frequency_hz));
  const auto interval = std::chrono::duration<double>(1.0 / hz);
  auto last_sent = std::chrono::steady_clock::time_point{};
  bool sent_once = false;

  while (!context->IsCancelled() && !cancel_flag->load()) {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::milliseconds(200), [&]() {
      return has_new_msg || context->IsCancelled() || cancel_flag->load();
    });

    if (context->IsCancelled() || cancel_flag->load()) {
      break;
    }
    if (!has_new_msg || latest_msg == nullptr) {
      continue;
    }

    const auto msg = latest_msg;
    has_new_msg = false;
    lock.unlock();

    const auto now = std::chrono::steady_clock::now();
    if (sent_once && (now - last_sent) < interval) {
      continue;
    }

    std::string payload;
    if (!payload_extractor(msg, &payload)) {
      continue;
    }

    robot::v1::DataChunk chunk;
    chunk.mutable_resource_id()->CopyFrom(resource_id);
    chunk.mutable_header()->mutable_timestamp()->set_seconds(msg->header.stamp.sec);
    chunk.mutable_header()->mutable_timestamp()->set_nanos(msg->header.stamp.nanosec);
    chunk.mutable_header()->set_frame_id(msg->header.frame_id);
    chunk.mutable_header()->set_sequence(sequence++);
    chunk.set_data(payload);
    chunk.set_compression(robot::v1::COMPRESSION_TYPE_NONE);
    chunk.set_uncompressed_size(static_cast<uint32_t>(
        std::min(payload.size(),
                 static_cast<size_t>(std::numeric_limits<uint32_t>::max()))));

    if (!writer->Write(chunk)) {
      return grpc::Status::OK;
    }

    last_sent = now;
    sent_once = true;
  }

  return grpc::Status::OK;
}

} // namespace

DataServiceImpl::DataServiceImpl(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<std::string>("data_camera_topic",
                                        "/camera/color/image_raw/compressed");
  node_->declare_parameter<std::string>("data_camera_fallback_topic",
                                        "/camera/color/compressed");
  node_->declare_parameter<std::string>("data_map_topic", "/overall_map");
  node_->declare_parameter<std::string>("data_pointcloud_topic",
                                        "/cloud_registered");
  node_->declare_parameter<std::string>("data_terrain_topic",
                                        "/terrain_map_ext");
  node_->declare_parameter<std::string>("data_file_root", "");
  node_->declare_parameter<std::string>("apply_firmware_script",
                                        "/usr/local/bin/apply_firmware.sh");
  node_->declare_parameter<bool>("webrtc_enabled", false);
  node_->declare_parameter<int>("webrtc_offer_timeout_ms", 3000);
  node_->declare_parameter<std::string>("webrtc_start_command", "");
  node_->declare_parameter<std::string>("webrtc_stop_command", "");
  node_->declare_parameter<std::string>("webrtc_offer_path",
                                        "/tmp/webrtc_offer_{session_id}.sdp");
  node_->declare_parameter<std::string>("webrtc_ice_path",
                                        "/tmp/webrtc_ice_{session_id}.txt");

  camera_topic_ = node_->get_parameter("data_camera_topic").as_string();
  camera_fallback_topic_ =
      node_->get_parameter("data_camera_fallback_topic").as_string();
  map_topic_ = node_->get_parameter("data_map_topic").as_string();
  pointcloud_topic_ = node_->get_parameter("data_pointcloud_topic").as_string();
  terrain_topic_ = node_->get_parameter("data_terrain_topic").as_string();
  file_root_ = node_->get_parameter("data_file_root").as_string();
  apply_firmware_script_ =
      node_->get_parameter("apply_firmware_script").as_string();
  webrtc_enabled_ = node_->get_parameter("webrtc_enabled").as_bool();
  webrtc_offer_timeout_ms_ =
      node_->get_parameter("webrtc_offer_timeout_ms").as_int();
  webrtc_start_command_ = node_->get_parameter("webrtc_start_command").as_string();
  webrtc_stop_command_ = node_->get_parameter("webrtc_stop_command").as_string();
  webrtc_offer_path_ = node_->get_parameter("webrtc_offer_path").as_string();
  webrtc_ice_path_ = node_->get_parameter("webrtc_ice_path").as_string();

#ifdef WEBRTC_ENABLED
  // 初始化 WebRTC Bridge
  InitializeWebRTCBridge();
#endif
}

grpc::Status
DataServiceImpl::ListResources(grpc::ServerContext *,
                               const google::protobuf::Empty *,
                               robot::v1::ListResourcesResponse *response) {
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  auto *camera = response->add_resources();
  camera->mutable_id()->set_type(robot::v1::RESOURCE_TYPE_CAMERA);
  camera->mutable_id()->set_name("front");
  camera->set_description("Compressed image stream, default topic: " + camera_topic_);
  camera->set_available(true);
  AddDefaultProfiles(camera);

  auto *map = response->add_resources();
  map->mutable_id()->set_type(robot::v1::RESOURCE_TYPE_MAP);
  map->mutable_id()->set_name("global_map");
  map->set_description("PointCloud2 map stream, default topic: " + map_topic_);
  map->set_available(true);
  AddDefaultProfiles(map);

  auto *pointcloud = response->add_resources();
  pointcloud->mutable_id()->set_type(robot::v1::RESOURCE_TYPE_POINTCLOUD);
  pointcloud->mutable_id()->set_name("local_cloud");
  pointcloud->set_description("PointCloud2 live stream, default topic: " +
                              pointcloud_topic_);
  pointcloud->set_available(true);
  AddDefaultProfiles(pointcloud);

  auto *terrain = response->add_resources();
  terrain->mutable_id()->set_type(robot::v1::RESOURCE_TYPE_POINTCLOUD);
  terrain->mutable_id()->set_name("terrain");
  terrain->set_description("Terrain analysis PointCloud2 stream, default topic: " +
                           terrain_topic_);
  terrain->set_available(true);
  AddDefaultProfiles(terrain);

  return grpc::Status::OK;
}

grpc::Status
DataServiceImpl::Subscribe(grpc::ServerContext *context,
                           const robot::v1::SubscribeRequest *request,
                           grpc::ServerWriter<robot::v1::DataChunk> *writer) {
  if (request->base().request_id().empty()) {
    return grpc::Status(grpc::INVALID_ARGUMENT,
                        "request.base.request_id is required and used as "
                        "subscription_id");
  }

  if (!IsCompressionSupported(request->profile().compression())) {
    return grpc::Status(grpc::UNIMPLEMENTED,
                        "Only COMPRESSION_TYPE_NONE is supported currently");
  }

  std::string topic = ResolveTopic(request);
  if (topic.empty()) {
    return grpc::Status(grpc::INVALID_ARGUMENT,
                        "Unsupported resource type for subscription");
  }

  const bool has_topic_override = IsTopicName(request->resource_id().name());
  if (!has_topic_override &&
      request->resource_id().type() == robot::v1::RESOURCE_TYPE_CAMERA &&
      node_->count_publishers(topic) == 0 &&
      !camera_fallback_topic_.empty() &&
      node_->count_publishers(camera_fallback_topic_) > 0) {
    topic = camera_fallback_topic_;
  }

  const std::string subscription_id = request->base().request_id();
  std::shared_ptr<std::atomic_bool> cancel_flag;
  if (!RegisterSubscription(subscription_id, &cancel_flag)) {
    return grpc::Status(grpc::ALREADY_EXISTS,
                        "subscription_id already active");
  }

  struct SubscriptionCleanup {
    DataServiceImpl *service;
    std::string subscription_id;
    ~SubscriptionCleanup() { service->RemoveSubscription(subscription_id); }
  } cleanup{this, subscription_id};

  const double hz = ResolveFrequency(request);
  switch (request->resource_id().type()) {
  case robot::v1::RESOURCE_TYPE_CAMERA:
    return StreamResource<sensor_msgs::msg::CompressedImage>(
        node_, topic, rclcpp::SensorDataQoS(), request->resource_id(), hz,
        [](const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg,
           std::string *output) {
          if (msg == nullptr || output == nullptr) {
            return false;
          }
          output->assign(reinterpret_cast<const char *>(msg->data.data()),
                         msg->data.size());
          return true;
        },
        cancel_flag, context, writer);
  case robot::v1::RESOURCE_TYPE_MAP:
    return StreamResource<sensor_msgs::msg::PointCloud2>(
        node_, topic, rclcpp::QoS(1).transient_local(), request->resource_id(),
        hz,
        [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
           std::string *output) {
          if (msg == nullptr || output == nullptr) {
            return false;
          }
          *output = SerializePointCloud2WithMeta(msg);
          return !output->empty();
        },
        cancel_flag, context, writer);
  case robot::v1::RESOURCE_TYPE_POINTCLOUD:
    return StreamResource<sensor_msgs::msg::PointCloud2>(
        node_, topic, rclcpp::SensorDataQoS(), request->resource_id(), hz,
        [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg,
           std::string *output) {
          if (msg == nullptr || output == nullptr) {
            return false;
          }
          *output = SerializePointCloud2WithMeta(msg);
          return !output->empty();
        },
        cancel_flag, context, writer);
  default:
    return grpc::Status(grpc::UNIMPLEMENTED, "Resource type not supported");
  }
}

grpc::Status
DataServiceImpl::Unsubscribe(grpc::ServerContext *,
                             const robot::v1::UnsubscribeRequest *request,
                             robot::v1::UnsubscribeResponse *response) {
  response->mutable_base()->set_request_id(request->base().request_id());

  if (request->subscription_id().empty()) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message("subscription_id is required");
    return grpc::Status::OK;
  }

  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  const auto it = active_subscriptions_.find(request->subscription_id());
  if (it == active_subscriptions_.end()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
    response->mutable_base()->set_error_message("subscription_id not found");
    return grpc::Status::OK;
  }

  it->second->store(true);
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  return grpc::Status::OK;
}

grpc::Status
DataServiceImpl::DownloadFile(grpc::ServerContext *context,
                              const robot::v1::DownloadFileRequest *request,
                              grpc::ServerWriter<robot::v1::FileChunk> *writer) {
  if (request->file_path().empty()) {
    return grpc::Status(grpc::INVALID_ARGUMENT, "file_path is required");
  }

  // 路径遍历防护：禁止 ".." 和绝对路径（当 file_root_ 已配置时）
  const std::string &raw_path = request->file_path();
  if (raw_path.find("..") != std::string::npos) {
    return grpc::Status(grpc::PERMISSION_DENIED,
                        "Path traversal not allowed (contains '..')");
  }

  std::string path = raw_path;
  if (!file_root_.empty()) {
    // 有 file_root_ 时，禁止绝对路径，强制拼接
    if (!path.empty() && path.front() == '/') {
      return grpc::Status(grpc::PERMISSION_DENIED,
                          "Absolute paths not allowed when file_root is set");
    }
    path = file_root_ + "/" + path;
  }

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    return grpc::Status(grpc::NOT_FOUND, "file not found");
  }

  const auto end_pos = file.tellg();
  if (end_pos < 0) {
    return grpc::Status(grpc::INTERNAL, "failed to read file size");
  }
  const uint64_t total_size = static_cast<uint64_t>(end_pos);
  file.seekg(0, std::ios::beg);

  const size_t chunk_size =
      request->chunk_size() > 0 ? request->chunk_size() : 64 * 1024;
  std::vector<char> buffer(chunk_size);
  uint64_t offset = 0;

  while (!context->IsCancelled() && file.good()) {
    file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize bytes_read = file.gcount();
    if (bytes_read <= 0) {
      break;
    }

    robot::v1::FileChunk chunk;
    chunk.set_offset(offset);
    chunk.set_data(buffer.data(), static_cast<size_t>(bytes_read));
    chunk.set_total_size(total_size);
    offset += static_cast<uint64_t>(bytes_read);
    chunk.set_is_last(offset >= total_size);

    if (!writer->Write(chunk)) {
      break;
    }
  }

  return grpc::Status::OK;
}

// ==================== File Upload (OTA) ====================

grpc::Status
DataServiceImpl::UploadFile(grpc::ServerContext *context,
                            grpc::ServerReader<robot::v1::UploadFileChunk> *reader,
                            robot::v1::UploadFileResponse *response) {
  robot::v1::UploadFileChunk chunk;
  std::ofstream output_file;
  std::string remote_path;
  uint64_t total_expected = 0;
  uint64_t bytes_received = 0;
  bool first_chunk = true;

  while (reader->Read(&chunk)) {
    if (context->IsCancelled()) {
      return grpc::Status(grpc::CANCELLED, "Upload cancelled by client");
    }

    if (first_chunk) {
      first_chunk = false;
      const auto &meta = chunk.metadata();
      remote_path = meta.remote_path();

      if (remote_path.empty()) {
        response->mutable_base()->set_error_code(
            robot::v1::ERROR_CODE_INVALID_REQUEST);
        response->set_success(false);
        response->set_message("remote_path is required in first chunk metadata");
        return grpc::Status::OK;
      }

      // 路径安全检查
      if (remote_path.find("..") != std::string::npos) {
        response->mutable_base()->set_error_code(
            robot::v1::ERROR_CODE_INVALID_REQUEST);
        response->set_success(false);
        response->set_message("Path traversal not allowed");
        return grpc::Status::OK;
      }

      total_expected = meta.total_size();

      // 检查文件是否已存在
      if (!meta.overwrite() && FileExists(remote_path)) {
        response->mutable_base()->set_error_code(
            robot::v1::ERROR_CODE_INVALID_REQUEST);
        response->set_success(false);
        response->set_message("File already exists and overwrite=false");
        return grpc::Status::OK;
      }

      // 确保目录存在
      {
        auto last_slash = remote_path.rfind('/');
        if (last_slash != std::string::npos) {
          std::string dir = remote_path.substr(0, last_slash);
          std::string mkdir_cmd = "mkdir -p '" + dir + "'";
          std::system(mkdir_cmd.c_str());
        }
      }

      // 打开输出文件
      output_file.open(remote_path, std::ios::binary | std::ios::trunc);
      if (!output_file.is_open()) {
        response->mutable_base()->set_error_code(
            robot::v1::ERROR_CODE_INTERNAL_ERROR);
        response->set_success(false);
        response->set_message("Failed to create file: " + remote_path);
        return grpc::Status::OK;
      }

      RCLCPP_INFO(node_->get_logger(),
                  "Upload started: %s (%s, %lu bytes, category=%s)",
                  meta.filename().c_str(), remote_path.c_str(),
                  static_cast<unsigned long>(total_expected),
                  meta.category().c_str());
    }

    if (!output_file.is_open()) {
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_INTERNAL_ERROR);
      response->set_success(false);
      response->set_message("File not open (missing metadata in first chunk?)");
      return grpc::Status::OK;
    }

    // 写入数据
    const auto &data = chunk.data();
    if (!data.empty()) {
      output_file.write(data.data(), static_cast<std::streamsize>(data.size()));
      bytes_received += data.size();
    }

    if (chunk.is_last()) {
      break;
    }
  }

  if (output_file.is_open()) {
    output_file.close();
  }

  RCLCPP_INFO(node_->get_logger(),
              "Upload complete: %s (%lu bytes received)",
              remote_path.c_str(),
              static_cast<unsigned long>(bytes_received));

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_success(true);
  response->set_remote_path(remote_path);
  response->set_bytes_received(bytes_received);
  response->set_message("Upload complete");
  return grpc::Status::OK;
}

// ==================== List Remote Files ====================

grpc::Status
DataServiceImpl::ListRemoteFiles(
    grpc::ServerContext *,
    const robot::v1::ListRemoteFilesRequest *request,
    robot::v1::ListRemoteFilesResponse *response) {
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  const std::string &directory = request->directory();
  if (directory.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message("directory is required");
    return grpc::Status::OK;
  }

  // 路径安全检查
  if (directory.find("..") != std::string::npos) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message("Path traversal not allowed");
    return grpc::Status::OK;
  }

  // 使用 ls -la 获取文件列表 (简单实现)
  // 更好的方式是直接用 std::filesystem，但 ROS2 Humble 默认 C++17
  std::string cmd = "find '" + directory + "' -maxdepth 1 -type f -printf '%f\\t%s\\t%T+\\n' 2>/dev/null";
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message("Failed to list directory");
    return grpc::Status::OK;
  }

  char line[1024];
  uint64_t total_size = 0;
  const std::string &category_filter = request->category();

  while (fgets(line, sizeof(line), pipe)) {
    std::string line_str(line);
    // 移除尾部换行
    while (!line_str.empty() && (line_str.back() == '\n' || line_str.back() == '\r')) {
      line_str.pop_back();
    }
    if (line_str.empty()) continue;

    // 解析: filename\tsize\tmodified_time
    size_t tab1 = line_str.find('\t');
    size_t tab2 = line_str.find('\t', tab1 + 1);
    if (tab1 == std::string::npos || tab2 == std::string::npos) continue;

    std::string filename = line_str.substr(0, tab1);
    std::string size_str = line_str.substr(tab1 + 1, tab2 - tab1 - 1);
    std::string mod_time = line_str.substr(tab2 + 1);

    uint64_t file_size = 0;
    try {
      file_size = std::stoull(size_str);
    } catch (...) {
      continue;
    }

    // 推断 category
    std::string ext;
    auto dot_pos = filename.rfind('.');
    if (dot_pos != std::string::npos) {
      ext = filename.substr(dot_pos + 1);
    }

    std::string file_category = "other";
    if (ext == "pt" || ext == "pth" || ext == "onnx" || ext == "tflite" || ext == "engine") {
      file_category = "model";
    } else if (ext == "pcd" || ext == "ply" || ext == "pickle" || ext == "pgm") {
      file_category = "map";
    } else if (ext == "yaml" || ext == "yml" || ext == "json" || ext == "xml" || ext == "cfg" || ext == "ini") {
      file_category = "config";
    } else if (ext == "bin" || ext == "hex" || ext == "img" || ext == "deb") {
      file_category = "firmware";
    }

    // 过滤
    if (!category_filter.empty() && file_category != category_filter) {
      continue;
    }

    auto *info = response->add_files();
    info->set_path(directory + "/" + filename);
    info->set_filename(filename);
    info->set_size(file_size);
    info->set_modified_time(mod_time);
    info->set_category(file_category);

    total_size += file_size;
  }

  pclose(pipe);

  response->set_total_size(total_size);

  // 获取磁盘剩余空间
  std::string df_cmd = "df -B1 '" + directory + "' 2>/dev/null | tail -1 | awk '{print $4}'";
  FILE *df_pipe = popen(df_cmd.c_str(), "r");
  if (df_pipe) {
    char buf[64];
    if (fgets(buf, sizeof(buf), df_pipe)) {
      try {
        response->set_free_space(std::stoull(std::string(buf)));
      } catch (...) {}
    }
    pclose(df_pipe);
  }

  return grpc::Status::OK;
}

// ==================== Delete Remote File ====================

grpc::Status
DataServiceImpl::DeleteRemoteFile(
    grpc::ServerContext *,
    const robot::v1::DeleteRemoteFileRequest *request,
    robot::v1::DeleteRemoteFileResponse *response) {
  const std::string &path = request->remote_path();

  if (path.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->set_success(false);
    response->set_message("remote_path is required");
    return grpc::Status::OK;
  }

  // 路径安全检查
  if (path.find("..") != std::string::npos) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->set_success(false);
    response->set_message("Path traversal not allowed");
    return grpc::Status::OK;
  }

  // 只允许删除文件，不允许删除目录
  if (!FileExists(path)) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
    response->set_success(false);
    response->set_message("File not found: " + path);
    return grpc::Status::OK;
  }

  if (std::remove(path.c_str()) != 0) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->set_success(false);
    response->set_message("Failed to delete: " + path);
    return grpc::Status::OK;
  }

  RCLCPP_INFO(node_->get_logger(), "Deleted file: %s", path.c_str());

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_success(true);
  response->set_message("File deleted: " + path);
  return grpc::Status::OK;
}

// ==========================================================================
// ApplyFirmware - 触发固件刷写（fork 外部脚本）
// ==========================================================================
grpc::Status
DataServiceImpl::ApplyFirmware(
    grpc::ServerContext * /*context*/,
    const robot::v1::ApplyFirmwareRequest *request,
    robot::v1::ApplyFirmwareResponse *response) {

  const std::string firmware_path = request->firmware_path();

  // 1. 路径安全检查
  if (firmware_path.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->set_success(false);
    response->set_message("firmware_path is required");
    return grpc::Status::OK;
  }
  if (firmware_path.find("..") != std::string::npos) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->set_success(false);
    response->set_message("Path traversal not allowed");
    return grpc::Status::OK;
  }

  // 2. 检查固件文件是否存在
  {
    std::ifstream probe(firmware_path);
    if (!probe.good()) {
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_NOT_FOUND);
      response->set_success(false);
      response->set_message("Firmware file not found: " + firmware_path);
      return grpc::Status::OK;
    }
  }

  // 3. 检查刷写脚本是否存在
  if (apply_firmware_script_.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->set_success(false);
    response->set_message("apply_firmware_script parameter not configured");
    return grpc::Status::OK;
  }
  {
    std::ifstream probe(apply_firmware_script_);
    if (!probe.good()) {
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
      response->set_success(false);
      response->set_message("Apply script not found: " +
                            apply_firmware_script_);
      return grpc::Status::OK;
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "ApplyFirmware: path=%s script=%s",
              firmware_path.c_str(),
              apply_firmware_script_.c_str());

  // 4. Fork 脚本执行（非阻塞）
  //    脚本接收参数: $1 = firmware_path
  //    脚本负责: 校验 -> 刷写 -> 重启
  const std::string command =
      "nohup " + apply_firmware_script_ + " '" + firmware_path +
      "' >> /tmp/apply_firmware.log 2>&1 &";

  std::thread([command, logger = node_->get_logger()]() {
    RCLCPP_INFO(logger, "ApplyFirmware: executing script in background");
    int ret = std::system(command.c_str());
    RCLCPP_INFO(logger, "ApplyFirmware: script launch returned %d", ret);
  }).detach();

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_success(true);
  response->set_message(
      "Firmware apply started. The robot may reboot shortly.");
  return grpc::Status::OK;
}

grpc::Status
DataServiceImpl::StartCamera(grpc::ServerContext *,
                             const robot::v1::StartCameraRequest *request,
                             robot::v1::StartCameraResponse *response) {
  response->mutable_base()->set_request_id(request->base().request_id());
  if (!webrtc_enabled_ || webrtc_start_command_.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->mutable_base()->set_error_message(
        "WebRTC not enabled. Configure webrtc_* parameters.");
    return grpc::Status::OK;
  }

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  const std::string session_id =
      "camera_session_" + request->camera_id() + "_" + std::to_string(now_ns);

  const std::string topic = camera_topic_;
  const auto &profile = request->profile();

  std::unordered_map<std::string, std::string> vars;
  vars["session_id"] = session_id;
  vars["camera_id"] = request->camera_id();
  vars["topic"] = topic;
  vars["width"] = std::to_string(profile.width());
  vars["height"] = std::to_string(profile.height());
  vars["fps"] = std::to_string(profile.fps());
  vars["bitrate_kbps"] = std::to_string(profile.bitrate_kbps());
  vars["codec"] = profile.codec();

  const std::string offer_path = ApplyTemplate(webrtc_offer_path_, vars);
  const std::string ice_path = ApplyTemplate(webrtc_ice_path_, vars);
  vars["offer_path"] = offer_path;
  vars["ice_path"] = ice_path;

  const std::string command = ApplyTemplate(webrtc_start_command_, vars);

  {
    std::lock_guard<std::mutex> lock(webrtc_mutex_);
    webrtc_sessions_[session_id] = WebrtcSessionInfo{
        request->camera_id(), topic, offer_path, ice_path};
  }

  std::thread([command]() {
    if (!command.empty()) {
      std::system(command.c_str());
    }
  }).detach();

  if (!WaitForFile(offer_path, webrtc_offer_timeout_ms_)) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->mutable_base()->set_error_message(
        "Timed out waiting for WebRTC offer");
    return grpc::Status::OK;
  }

  std::string offer;
  if (!ReadFileToString(offer_path, &offer) || offer.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message(
        "Failed to read WebRTC offer");
    return grpc::Status::OK;
  }

  {
    std::lock_guard<std::mutex> lock(camera_sessions_mutex_);
    active_camera_sessions_.insert(session_id);
  }

  response->set_session_id(session_id);
  response->set_sdp_offer(offer);

  const auto ice_candidates = ReadLines(ice_path);
  for (const auto &candidate : ice_candidates) {
    response->add_ice_candidates(candidate);
  }

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  return grpc::Status::OK;
}

grpc::Status
DataServiceImpl::StopCamera(grpc::ServerContext *,
                            const robot::v1::StopCameraRequest *request,
                            robot::v1::StopCameraResponse *response) {
  response->mutable_base()->set_request_id(request->base().request_id());
  WebrtcSessionInfo session_info;
  bool has_session_info = false;
  {
    std::lock_guard<std::mutex> lock(webrtc_mutex_);
    const auto it = webrtc_sessions_.find(request->session_id());
    if (it != webrtc_sessions_.end()) {
      session_info = it->second;
      has_session_info = true;
      webrtc_sessions_.erase(it);
    }
  }

  if (has_session_info && !webrtc_stop_command_.empty()) {
    std::unordered_map<std::string, std::string> vars;
    vars["session_id"] = request->session_id();
    vars["camera_id"] = session_info.camera_id;
    vars["topic"] = session_info.topic;
    vars["offer_path"] = session_info.offer_path;
    vars["ice_path"] = session_info.ice_path;
    const std::string command = ApplyTemplate(webrtc_stop_command_, vars);
    if (!command.empty()) {
      std::thread([command]() { std::system(command.c_str()); }).detach();
    }
  }

  {
    std::lock_guard<std::mutex> lock(camera_sessions_mutex_);
    const auto it = active_camera_sessions_.find(request->session_id());
    if (it == active_camera_sessions_.end()) {
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
      response->mutable_base()->set_error_message("session_id not found");
      return grpc::Status::OK;
    }
    active_camera_sessions_.erase(it);
  }

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  return grpc::Status::OK;
}

bool DataServiceImpl::IsCompressionSupported(
    robot::v1::CompressionType compression) const {
  return compression == robot::v1::COMPRESSION_TYPE_UNSPECIFIED ||
         compression == robot::v1::COMPRESSION_TYPE_NONE;
}

double DataServiceImpl::ResolveFrequency(
    const robot::v1::SubscribeRequest *request) const {
  if (request == nullptr) {
    return kDefaultPointCloudHz;
  }

  if (request->profile().frequency() > 0.0) {
    return request->profile().frequency();
  }

  switch (request->resource_id().type()) {
  case robot::v1::RESOURCE_TYPE_CAMERA:
    return kDefaultCameraHz;
  case robot::v1::RESOURCE_TYPE_MAP:
    return kDefaultMapHz;
  case robot::v1::RESOURCE_TYPE_POINTCLOUD:
    return kDefaultPointCloudHz;
  default:
    return kDefaultPointCloudHz;
  }
}

std::string DataServiceImpl::ResolveTopic(
    const robot::v1::SubscribeRequest *request) const {
  if (request == nullptr) {
    return {};
  }

  // 如果 resource_id.name 是完整 ROS topic（以 / 开头），直接作为覆盖配置。
  const std::string &name = request->resource_id().name();
  if (IsTopicName(name)) {
    return name;
  }

  switch (request->resource_id().type()) {
  case robot::v1::RESOURCE_TYPE_CAMERA:
    return camera_topic_;
  case robot::v1::RESOURCE_TYPE_MAP:
    return map_topic_;
  case robot::v1::RESOURCE_TYPE_POINTCLOUD:
    // Differentiate between lidar cloud and terrain by resource name
    if (name == "terrain" || name == "terrain_map" || name == "terrain_map_ext") {
      return terrain_topic_;
    }
    return pointcloud_topic_;
  default:
    return {};
  }
}

bool DataServiceImpl::RegisterSubscription(
    const std::string &subscription_id,
    std::shared_ptr<std::atomic_bool> *cancel_flag) {
  if (subscription_id.empty() || cancel_flag == nullptr) {
    return false;
  }

  auto flag = std::make_shared<std::atomic_bool>(false);
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  const auto result = active_subscriptions_.emplace(subscription_id, flag);
  if (!result.second) {
    return false;
  }

  *cancel_flag = result.first->second;
  return true;
}

void DataServiceImpl::RemoveSubscription(const std::string &subscription_id) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  active_subscriptions_.erase(subscription_id);
}

// ==================== WebRTC 信令实现 ====================

std::shared_ptr<DataServiceImpl::WebRTCSession>
DataServiceImpl::GetOrCreateWebRTCSession(const std::string &session_id) {
  std::lock_guard<std::mutex> lock(webrtc_signaling_mutex_);
  auto it = webrtc_signaling_sessions_.find(session_id);
  if (it != webrtc_signaling_sessions_.end()) {
    return it->second;
  }
  
  auto session = std::make_shared<WebRTCSession>();
  session->session_id = session_id;
  webrtc_signaling_sessions_[session_id] = session;
  
  RCLCPP_INFO(node_->get_logger(), "Created WebRTC session: %s", session_id.c_str());
  return session;
}

void DataServiceImpl::RemoveWebRTCSession(const std::string &session_id) {
  std::lock_guard<std::mutex> lock(webrtc_signaling_mutex_);
  auto it = webrtc_signaling_sessions_.find(session_id);
  if (it != webrtc_signaling_sessions_.end()) {
    it->second->active.store(false);
    it->second->cv.notify_all();
    webrtc_signaling_sessions_.erase(it);
    RCLCPP_INFO(node_->get_logger(), "Removed WebRTC session: %s", session_id.c_str());
  }
  
#ifdef WEBRTC_ENABLED
  // 移除 Bridge 中对应的 peer 连接
  if (webrtc_bridge_) {
    webrtc_bridge_->RemovePeer(session_id);
  }
#endif
}

#ifdef WEBRTC_ENABLED
void DataServiceImpl::InitializeWebRTCBridge() {
  RCLCPP_INFO(node_->get_logger(), "Initializing WebRTC Bridge...");
  
  webrtc_bridge_ = std::make_unique<WebRTCBridge>(node_);
  
  if (!webrtc_bridge_->Initialize()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize WebRTC Bridge");
    webrtc_bridge_.reset();
    return;
  }
  
  // 设置回调：本地 SDP 描述生成时（Answer）
  webrtc_bridge_->SetOnLocalDescription(
      [this](const std::string &session_id, const std::string &sdp, const std::string &type) {
        robot::v1::WebRTCSignal signal;
        signal.set_session_id(session_id);
        signal.set_sdp(sdp);
        
        if (type == "answer") {
          signal.set_type(robot::v1::WEBRTC_SIGNAL_TYPE_ANSWER);
        } else if (type == "offer") {
          signal.set_type(robot::v1::WEBRTC_SIGNAL_TYPE_OFFER);
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                    "WebRTC Bridge generated local %s for session %s",
                    type.c_str(), session_id.c_str());
        
        SendWebRTCSignalToSession(session_id, std::move(signal));
      });
  
  // 设置回调：本地 ICE candidate 生成时
  webrtc_bridge_->SetOnLocalCandidate(
      [this](const std::string &session_id, const std::string &candidate, const std::string &mid) {
        robot::v1::WebRTCSignal signal;
        signal.set_session_id(session_id);
        signal.set_type(robot::v1::WEBRTC_SIGNAL_TYPE_ICE_CANDIDATE);
        signal.set_ice_candidate(candidate);
        signal.set_ice_mid(mid);
        
        RCLCPP_DEBUG(node_->get_logger(),
                     "WebRTC Bridge generated ICE candidate for session %s",
                     session_id.c_str());
        
        SendWebRTCSignalToSession(session_id, std::move(signal));
      });
  
  // 设置回调：连接状态变化时
  webrtc_bridge_->SetOnConnectionState(
      [this](const std::string &session_id, const std::string &state) {
        RCLCPP_INFO(node_->get_logger(),
                    "WebRTC connection state for session %s: %s",
                    session_id.c_str(), state.c_str());
        
        // 更新 session 的 peer_connected 状态
        std::lock_guard<std::mutex> lock(webrtc_signaling_mutex_);
        auto it = webrtc_signaling_sessions_.find(session_id);
        if (it != webrtc_signaling_sessions_.end()) {
          it->second->peer_connected.store(state == "connected");
        }
      });
  
  // 设置回调：ICE 收集完成时
  webrtc_bridge_->SetOnIceGatheringDone(
      [this](const std::string &session_id) {
        robot::v1::WebRTCSignal signal;
        signal.set_session_id(session_id);
        signal.set_type(robot::v1::WEBRTC_SIGNAL_TYPE_ICE_DONE);
        
        RCLCPP_INFO(node_->get_logger(),
                    "WebRTC ICE gathering done for session %s",
                    session_id.c_str());
        
        SendWebRTCSignalToSession(session_id, std::move(signal));
      });
  
  RCLCPP_INFO(node_->get_logger(), "WebRTC Bridge initialized successfully");
}

void DataServiceImpl::SendWebRTCSignalToSession(
    const std::string &session_id, robot::v1::WebRTCSignal signal) {
  std::lock_guard<std::mutex> lock(webrtc_signaling_mutex_);
  
  auto it = webrtc_signaling_sessions_.find(session_id);
  if (it == webrtc_signaling_sessions_.end()) {
    RCLCPP_WARN(node_->get_logger(),
                "Cannot send signal: WebRTC session %s not found",
                session_id.c_str());
    return;
  }
  
  auto session = it->second;
  {
    std::lock_guard<std::mutex> session_lock(session->mutex);
    session->outgoing_signals.push(std::move(signal));
  }
  session->cv.notify_one();
}
#endif

void DataServiceImpl::HandleWebRTCOffer(
    std::shared_ptr<WebRTCSession> session,
    const robot::v1::WebRTCSignal &signal) {
  if (!session) return;
  
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    session->remote_sdp = signal.sdp();
    session->config = signal.config();
  }
  
  RCLCPP_INFO(node_->get_logger(), 
              "WebRTC Offer received for session %s: video=%s, audio=%s, camera=%s",
              session->session_id.c_str(),
              session->config.video_enabled() ? "true" : "false",
              session->config.audio_enabled() ? "true" : "false",
              session->config.camera_id().c_str());

#ifdef WEBRTC_ENABLED
  if (webrtc_bridge_) {
    // 使用 WebRTC Bridge 处理 Offer
    // Bridge 会创建 PeerConnection、设置远程 SDP、生成 Answer
    // Answer 和 ICE candidates 会通过回调自动发送到信令队列
    webrtc_bridge_->HandleOffer(session->session_id, signal.sdp());
    
    // 订阅 camera topic 以获取视频帧
    std::string camera_topic = camera_topic_;
    if (!session->config.camera_id().empty() && 
        session->config.camera_id().front() == '/') {
      // 如果 camera_id 是完整的 topic 路径，直接使用
      camera_topic = session->config.camera_id();
    }
    webrtc_bridge_->SubscribeCameraTopic(camera_topic);
    
    RCLCPP_INFO(node_->get_logger(), 
                "WebRTC Bridge handling offer, subscribed to camera topic: %s",
                camera_topic.c_str());
    return;
  }
#endif

  // Fallback: 当 WebRTC Bridge 不可用时，发送占位 Answer（仅用于测试信令通道）
  RCLCPP_WARN(node_->get_logger(), 
              "WebRTC Bridge not available, sending placeholder answer");
  
  robot::v1::WebRTCSignal answer;
  answer.set_session_id(session->session_id);
  answer.set_type(robot::v1::WEBRTC_SIGNAL_TYPE_ANSWER);
  
  // 生成一个简单的 SDP Answer（仅用于测试）
  std::string test_answer = 
      "v=0\r\n"
      "o=- " + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + 
      " 0 IN IP4 0.0.0.0\r\n"
      "s=-\r\n"
      "t=0 0\r\n"
      "a=group:BUNDLE 0\r\n"
      "m=video 9 UDP/TLS/RTP/SAVPF 96\r\n"
      "c=IN IP4 0.0.0.0\r\n"
      "a=rtcp:9 IN IP4 0.0.0.0\r\n"
      "a=ice-ufrag:testufrag\r\n"
      "a=ice-pwd:testpwd123456789012345678\r\n"
      "a=fingerprint:sha-256 00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00\r\n"
      "a=setup:passive\r\n"
      "a=mid:0\r\n"
      "a=recvonly\r\n"
      "a=rtcp-mux\r\n"
      "a=rtpmap:96 H264/90000\r\n";
  
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    answer.set_sdp(test_answer);
    session->local_sdp = test_answer;
    session->outgoing_signals.push(answer);
  }
  session->cv.notify_one();
  
  RCLCPP_INFO(node_->get_logger(), "Queued placeholder WebRTC Answer for session %s",
              session->session_id.c_str());
}

void DataServiceImpl::HandleWebRTCAnswer(
    std::shared_ptr<WebRTCSession> session,
    const robot::v1::WebRTCSignal &signal) {
  if (!session) return;
  
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    session->remote_sdp = signal.sdp();
  }
  
  RCLCPP_INFO(node_->get_logger(), "WebRTC Answer received for session %s",
              session->session_id.c_str());

#ifdef WEBRTC_ENABLED
  if (webrtc_bridge_) {
    auto peer = webrtc_bridge_->GetPeer(session->session_id);
    if (peer) {
      peer->SetRemoteDescription(signal.sdp(), "answer");
      RCLCPP_INFO(node_->get_logger(), 
                  "Set remote answer description for session %s",
                  session->session_id.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "No peer found for session %s to set answer",
                  session->session_id.c_str());
    }
  }
#endif
}

void DataServiceImpl::HandleWebRTCIceCandidate(
    std::shared_ptr<WebRTCSession> session,
    const robot::v1::WebRTCSignal &signal) {
  if (!session) return;
  
  {
    std::lock_guard<std::mutex> lock(session->mutex);
    session->remote_ice_candidates.push_back(signal.ice_candidate());
  }
  
  RCLCPP_DEBUG(node_->get_logger(), 
               "WebRTC ICE candidate received for session %s: mid=%s, index=%d",
               session->session_id.c_str(),
               signal.ice_mid().c_str(),
               signal.ice_mline_index());

#ifdef WEBRTC_ENABLED
  if (webrtc_bridge_) {
    webrtc_bridge_->HandleIceCandidate(
        session->session_id, 
        signal.ice_candidate(), 
        signal.ice_mid());
  }
#endif
}

grpc::Status DataServiceImpl::WebRTCSignaling(
    grpc::ServerContext *context,
    grpc::ServerReaderWriter<robot::v1::WebRTCSignal,
                             robot::v1::WebRTCSignal> *stream) {
  
  std::string session_id;
  std::shared_ptr<WebRTCSession> session;
  
  RCLCPP_INFO(node_->get_logger(), "WebRTC signaling stream opened");
  
  // 启动发送线程：将 outgoing_signals 队列中的消息发送到客户端
  std::atomic_bool writer_running{true};
  std::thread writer_thread([&]() {
    while (writer_running.load() && !context->IsCancelled()) {
      if (!session) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      
      robot::v1::WebRTCSignal outgoing;
      bool has_signal = false;
      
      {
        std::unique_lock<std::mutex> lock(session->mutex);
        if (session->cv.wait_for(lock, std::chrono::milliseconds(100), [&]() {
              return !session->outgoing_signals.empty() || 
                     !session->active.load() ||
                     context->IsCancelled();
            })) {
          if (!session->outgoing_signals.empty()) {
            outgoing = session->outgoing_signals.front();
            session->outgoing_signals.pop();
            has_signal = true;
          }
        }
      }
      
      if (has_signal) {
        if (!stream->Write(outgoing)) {
          RCLCPP_WARN(node_->get_logger(), 
                      "Failed to write WebRTC signal to client");
          break;
        }
        RCLCPP_DEBUG(node_->get_logger(),
                     "Sent WebRTC signal type %d to client",
                     static_cast<int>(outgoing.type()));
      }
      
      if (!session->active.load()) {
        break;
      }
    }
  });
  
  // 主循环：读取客户端发送的信令消息
  robot::v1::WebRTCSignal incoming;
  bool hangup_received = false;
  while (!hangup_received && !context->IsCancelled() && stream->Read(&incoming)) {
    // 第一条消息确定 session_id
    if (session_id.empty()) {
      session_id = incoming.session_id();
      if (session_id.empty()) {
        // 自动生成 session_id
        session_id = "webrtc_" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
      }
      session = GetOrCreateWebRTCSession(session_id);
    }
    
    // 根据信令类型处理
    switch (incoming.type()) {
      case robot::v1::WEBRTC_SIGNAL_TYPE_OFFER:
        HandleWebRTCOffer(session, incoming);
        break;
        
      case robot::v1::WEBRTC_SIGNAL_TYPE_ANSWER:
        HandleWebRTCAnswer(session, incoming);
        break;
        
      case robot::v1::WEBRTC_SIGNAL_TYPE_ICE_CANDIDATE:
        HandleWebRTCIceCandidate(session, incoming);
        break;
        
      case robot::v1::WEBRTC_SIGNAL_TYPE_ICE_DONE:
        RCLCPP_INFO(node_->get_logger(), 
                    "WebRTC ICE gathering done for session %s",
                    session_id.c_str());
        // 客户端 ICE 收集完成，无需额外处理
        // 服务端 ICE 收集完成由 Bridge 的 OnIceGatheringDone 回调处理
        break;
        
      case robot::v1::WEBRTC_SIGNAL_TYPE_HANGUP:
        RCLCPP_INFO(node_->get_logger(),
                    "WebRTC hangup received for session %s",
                    session_id.c_str());
        // 发送确认挂断
        {
          robot::v1::WebRTCSignal hangup_ack;
          hangup_ack.set_session_id(session_id);
          hangup_ack.set_type(robot::v1::WEBRTC_SIGNAL_TYPE_HANGUP);
          std::lock_guard<std::mutex> lock(session->mutex);
          session->outgoing_signals.push(hangup_ack);
          session->cv.notify_one();
        }
        hangup_received = true;
        break;
        
      default:
        RCLCPP_WARN(node_->get_logger(),
                    "Unknown WebRTC signal type: %d",
                    static_cast<int>(incoming.type()));
        break;
    }
  }
  
  // 清理
  writer_running.store(false);
  if (session) {
    session->active.store(false);
    session->cv.notify_all();
  }
  
  if (writer_thread.joinable()) {
    writer_thread.join();
  }
  
  if (!session_id.empty()) {
    RemoveWebRTCSession(session_id);
  }
  
  RCLCPP_INFO(node_->get_logger(), "WebRTC signaling stream closed for session %s",
              session_id.c_str());
  
  return grpc::Status::OK;
}

} // namespace services
} // namespace remote_monitoring
