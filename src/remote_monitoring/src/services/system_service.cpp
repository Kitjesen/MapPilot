#include "remote_monitoring/services/system_service.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/task_manager.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace remote_monitoring {
namespace services {

SystemServiceImpl::SystemServiceImpl(rclcpp::Node *node,
                                     const std::string &robot_id,
                                     const std::string &firmware_version)
    : node_(node), robot_id_(robot_id), firmware_version_(firmware_version) {
  // 服务名通过参数配置 (标准接口契约)
  std::string relocalize_srv = "/nav/relocalize";
  if (node_->has_parameter("relocalize_service")) {
    relocalize_srv = node_->get_parameter("relocalize_service").as_string();
  } else {
    node_->declare_parameter<std::string>("relocalize_service", relocalize_srv);
  }
  relocalize_client_ =
      node_->create_client<interface::srv::Relocalize>(relocalize_srv);

  std::string save_map_srv = "/nav/save_map";
  if (node_->has_parameter("save_map_service")) {
    save_map_srv = node_->get_parameter("save_map_service").as_string();
  } else {
    node_->declare_parameter<std::string>("save_map_service", save_map_srv);
  }
  save_map_client_ =
      node_->create_client<interface::srv::SaveMaps>(save_map_srv);

  // 地图存储目录 (可通过 yaml 配置)
  node_->declare_parameter<std::string>("map_directory", "/maps");
  map_directory_ = node_->get_parameter("map_directory").as_string();

  // 确保地图目录存在
  namespace fs = std::filesystem;
  try {
    if (!fs::exists(map_directory_)) {
      fs::create_directories(map_directory_);
      RCLCPP_INFO(node_->get_logger(), "Created map directory: %s",
                  map_directory_.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Cannot create map directory %s: %s",
                map_directory_.c_str(), e.what());
  }

  // ── RuntimeConfig 初始化 ──
  node_->declare_parameter<std::string>(
      "runtime_config_path", "/opt/robot/config/robot_config.yaml");
  config_persist_path_ =
      node_->get_parameter("runtime_config_path").as_string();

  // 初始化 parameter client 到导航节点
  static const std::vector<std::string> kNavNodes = {
      "pathFollower", "localPlanner", "terrainAnalysis",
  };
  for (const auto &name : kNavNodes) {
    param_clients_[name] =
        std::make_shared<AsyncParamClient>(node_, name);
  }

  // 从磁盘加载配置 (如果有)
  LoadConfigFromYaml();
  // 并从活跃节点收集当前参数作为基线
  auto live = CollectCurrentParams();
  if (!live.empty()) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    // 合并: 磁盘配置覆盖活跃节点参数
    for (auto &[k, v] : current_config_.items()) {
      live[k] = v;
    }
    current_config_ = live;
  }
  RCLCPP_INFO(node_->get_logger(),
              "RuntimeConfig initialized with %zu params (version=%lu)",
              current_config_.size(), config_version_.load());
}

// ================================================================
//  安全辅助
// ================================================================

bool SystemServiceImpl::IsPathUnderMapDir(const std::string &path) const {
  namespace fs = std::filesystem;
  try {
    // canonical 解析 symlinks 和 "..", 防止路径遍历
    const auto map_root = fs::canonical(map_directory_);
    // 对于不存在的文件 (如新保存), 检查父目录
    const fs::path target(path);
    const auto parent = fs::exists(target)
        ? fs::canonical(target).parent_path()
        : fs::canonical(target.parent_path());
    // 目标的父目录必须以 map_root 开头
    const auto root_str = map_root.string();
    const auto parent_str = parent.string();
    return parent_str.compare(0, root_str.size(), root_str) == 0;
  } catch (...) {
    return false;
  }
}

bool SystemServiceImpl::RequireLease(robot::v1::BaseResponse *base) {
  if (!lease_mgr_ || lease_mgr_->HasActiveLease()) {
    return true;  // 无 lease_mgr 时放行; 有活跃租约时放行
  }
  base->set_error_code(robot::v1::ERROR_CODE_LEASE_CONFLICT);
  base->set_error_message("Operation requires an active lease");
  return false;
}

grpc::Status SystemServiceImpl::Login(
  grpc::ServerContext *,
  const robot::v1::LoginRequest *request,
  robot::v1::LoginResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  
  // 简化实现：接受任何用户（实际应校验）
  if (request->username().empty()) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message("Username required");
    return grpc::Status::OK;
  }
  
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_session_token("session_" + request->username());
  response->set_granted_role(request->requested_role());
  response->mutable_session_ttl()->set_seconds(3600);  // 1小时
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::Logout(
  grpc::ServerContext *,
  const robot::v1::LogoutRequest *request,
  robot::v1::LogoutResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());

  // 如果客户端要求释放租约
  if (request->release_lease() && lease_mgr_) {
    // 尝试释放当前 session 关联的租约
    // 注: 简化实现 — 释放所有活跃租约
    RCLCPP_INFO(node_->get_logger(),
                "Logout: releasing lease per client request");
    lease_mgr_->ForceRelease();
  }

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::Heartbeat(
  grpc::ServerContext *,
  const robot::v1::HeartbeatRequest *request,
  robot::v1::HeartbeatResponse *response) {
  
  // 记录心跳时间 (用于断联检测)
  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_heartbeat_ = std::chrono::steady_clock::now();
    heartbeat_received_ = true;
  }

  const auto now = std::chrono::system_clock::now();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now.time_since_epoch()).count();
  response->mutable_server_timestamp()->set_seconds(now_ns / 1000000000LL);
  response->mutable_server_timestamp()->set_nanos(
      static_cast<int32_t>(now_ns % 1000000000LL));
  
  // 计算 RTT（纳秒级精度）
  if (request->has_client_timestamp()) {
    const int64_t client_ns = request->client_timestamp().seconds() * 1000000000LL
                            + request->client_timestamp().nanos();
    const int64_t rtt_ns = std::max<int64_t>(0, now_ns - client_ns);
    const double rtt_ms = static_cast<double>(rtt_ns) / 1000000.0;
    response->mutable_server_quality()->set_rtt_ms(rtt_ms);
  }
  
  response->set_active_sessions(1);  // 简化
  
  return grpc::Status::OK;
}

double SystemServiceImpl::SecondsSinceLastHeartbeat() const {
  std::lock_guard<std::mutex> lock(heartbeat_mutex_);
  if (!heartbeat_received_) {
    return 0.0;  // 从未收到心跳则不触发降级
  }
  const auto elapsed = std::chrono::steady_clock::now() - last_heartbeat_;
  return std::chrono::duration<double>(elapsed).count();
}

grpc::Status SystemServiceImpl::GetRobotInfo(
  grpc::ServerContext *,
  const google::protobuf::Empty *,
  robot::v1::RobotInfoResponse *response) {
  
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_robot_id(robot_id_);
  response->set_display_name("Navigation Robot");
  response->set_firmware_version(firmware_version_);
  response->set_software_version("1.0.0");
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::GetCapabilities(
  grpc::ServerContext *,
  const google::protobuf::Empty *,
  robot::v1::CapabilitiesResponse *response) {
  
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->add_supported_resources("camera/front");
  response->add_supported_resources("camera/rear");
  response->add_supported_resources("pointcloud/lidar");
  response->add_supported_tasks(robot::v1::TASK_TYPE_NAVIGATION);
  response->add_supported_tasks(robot::v1::TASK_TYPE_MAPPING);
  response->set_teleop_supported(true);
  response->set_mapping_supported(true);
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::Relocalize(
  grpc::ServerContext *,
  const robot::v1::RelocalizeRequest *request,
  robot::v1::RelocalizeResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());

  // 租约保护: TF 突变属于破坏性操作
  if (!RequireLease(response->mutable_base())) {
    response->set_success(false);
    return grpc::Status::OK;
  }

  // 安全检查: 有活跃导航任务时拒绝重定位 (TF 突变会导致路径跟踪失败)
  if (task_mgr_ && task_mgr_->HasActiveTask()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_MODE_CONFLICT);
    response->mutable_base()->set_error_message(
        "Cannot relocalize during active task — cancel the task first");
    response->set_success(false);
    return grpc::Status::OK;
  }

  // 检查 PCD 文件是否存在
  if (!request->pcd_path().empty() &&
      !std::filesystem::exists(request->pcd_path())) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
    response->mutable_base()->set_error_message(
        "PCD file not found: " + request->pcd_path());
    response->set_success(false);
    return grpc::Status::OK;
  }

  if (!relocalize_client_->wait_for_service(std::chrono::seconds(2))) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->mutable_base()->set_error_message(
        "Relocalize service not available");
    response->set_success(false);
    return grpc::Status::OK;
  }

  auto ros_req = std::make_shared<interface::srv::Relocalize::Request>();
  ros_req->pcd_path = request->pcd_path();
  ros_req->x = static_cast<float>(request->x());
  ros_req->y = static_cast<float>(request->y());
  ros_req->z = static_cast<float>(request->z());
  ros_req->yaw = static_cast<float>(request->yaw());
  ros_req->pitch = static_cast<float>(request->pitch());
  ros_req->roll = static_cast<float>(request->roll());

  auto future = relocalize_client_->async_send_request(ros_req);
  if (future.wait_for(kServiceTimeout) != std::future_status::ready) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_TIMEOUT);
    response->mutable_base()->set_error_message("Relocalize service timed out");
    response->set_success(false);
    return grpc::Status::OK;
  }

  auto ros_resp = future.get();
  response->set_success(ros_resp->success);
  response->set_message(ros_resp->message);
  response->mutable_base()->set_error_code(
      ros_resp->success ? robot::v1::ERROR_CODE_OK
                        : robot::v1::ERROR_CODE_INTERNAL_ERROR);

  // 成功后跟踪当前加载的地图
  if (ros_resp->success) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    active_map_path_ = request->pcd_path();
    active_map_name_ = std::filesystem::path(request->pcd_path())
                           .filename().string();
    RCLCPP_INFO(node_->get_logger(), "Active map set to: %s",
                active_map_name_.c_str());
  }
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::SaveMap(
  grpc::ServerContext *,
  const robot::v1::SaveMapRequest *request,
  robot::v1::SaveMapResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());

  // 租约保护
  if (!RequireLease(response->mutable_base())) {
    response->set_success(false);
    return grpc::Status::OK;
  }

  // 路径规范化: 如果客户端只传了文件名 (无目录), 自动加上地图目录前缀
  namespace fs = std::filesystem;
  std::string file_path = request->file_path();
  const fs::path p(file_path);
  if (!p.has_parent_path() || p.parent_path().empty() ||
      p.parent_path() == ".") {
    file_path = (fs::path(map_directory_) / p.filename()).string();
  }

  // 安全检查: 目标路径必须在地图目录内
  if (!IsPathUnderMapDir(file_path)) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message(
        "Save path must be under map directory: " + map_directory_);
    response->set_success(false);
    return grpc::Status::OK;
  }

  // 覆盖保护: 目标文件已存在时拒绝 (防止意外数据丢失)
  if (fs::exists(file_path)) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_RESOURCE_CONFLICT);
    response->mutable_base()->set_error_message(
        "Map already exists: " +
        fs::path(file_path).filename().string() +
        " — delete it first or choose a different name");
    response->set_success(false);
    return grpc::Status::OK;
  }

  // 确保目标目录存在
  try {
    const auto parent = fs::path(file_path).parent_path();
    if (!fs::exists(parent)) {
      fs::create_directories(parent);
    }
  } catch (...) {}

  if (!save_map_client_->wait_for_service(std::chrono::seconds(2))) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->mutable_base()->set_error_message(
        "SaveMap service not available");
    response->set_success(false);
    return grpc::Status::OK;
  }

  auto ros_req = std::make_shared<interface::srv::SaveMaps::Request>();
  ros_req->file_path = file_path;  // 使用规范化后的完整路径
  ros_req->save_patches = request->save_patches();

  auto future = save_map_client_->async_send_request(ros_req);
  if (future.wait_for(kServiceTimeout) != std::future_status::ready) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_TIMEOUT);
    response->mutable_base()->set_error_message("SaveMap service timed out");
    response->set_success(false);
    return grpc::Status::OK;
  }

  auto ros_resp = future.get();
  response->set_success(ros_resp->success);
  response->set_message(ros_resp->message);
  response->mutable_base()->set_error_code(
      ros_resp->success ? robot::v1::ERROR_CODE_OK
                        : robot::v1::ERROR_CODE_INTERNAL_ERROR);

  // 验证文件确实被创建
  if (ros_resp->success && !fs::exists(file_path)) {
    response->set_success(false);
    response->set_message("Save service reported success but file not found");
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
  }

  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::ListMaps(
  grpc::ServerContext *,
  const robot::v1::ListMapsRequest *request,
  robot::v1::ListMapsResponse *response) {

  response->mutable_base()->set_request_id(request->base().request_id());

  // 使用可配置的地图目录; 忽略客户端的 directory 参数 (防止目录遍历)
  const std::string &directory = map_directory_;

  namespace fs = std::filesystem;
  try {
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
      response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
      return grpc::Status::OK;  // Empty list
    }

    for (const auto &entry : fs::directory_iterator(directory)) {
      if (!entry.is_regular_file()) continue;

      const auto ext = entry.path().extension().string();
      // 仅列出有意义的地图格式; .pickle 是 Python 序列化不可用于定位
      if (ext != ".pcd" && ext != ".ply" && ext != ".pgm") {
        continue;
      }

      auto *map_info = response->add_maps();
      map_info->set_name(entry.path().filename().string());
      map_info->set_path(entry.path().string());
      map_info->set_size_bytes(static_cast<int64_t>(entry.file_size()));

      const auto mod_time = entry.last_write_time();
      // C++17 compatible: convert file_clock to system_clock via time_t
      const auto file_duration = mod_time.time_since_epoch();
      const auto sys_duration = std::chrono::duration_cast<std::chrono::system_clock::duration>(
          file_duration - fs::file_time_type::clock::now().time_since_epoch()
          + std::chrono::system_clock::now().time_since_epoch());
      const auto time_t = std::chrono::system_clock::to_time_t(
          std::chrono::system_clock::time_point(sys_duration));
      char buf[64];
      std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&time_t));
      map_info->set_modified_at(buf);
      // 注: ext4 不支持独立 creation time，使用 modification time 作为近似值
      map_info->set_created_at(buf);

      // Try to read PCD point count from header
      if (ext == ".pcd") {
        std::ifstream pcd(entry.path());
        std::string line;
        while (std::getline(pcd, line)) {
          if (line.rfind("POINTS ", 0) == 0) {
            try {
              map_info->set_point_count(std::stoi(line.substr(7)));
            } catch (...) {}
            break;
          }
          if (line == "DATA ascii" || line == "DATA binary" ||
              line == "DATA binary_compressed") {
            break;
          }
        }
      }
    }

    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } catch (const std::exception &e) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message(
        std::string("Failed to list maps: ") + e.what());
  }

  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::DeleteMap(
  grpc::ServerContext *,
  const robot::v1::DeleteMapRequest *request,
  robot::v1::DeleteMapResponse *response) {

  response->mutable_base()->set_request_id(request->base().request_id());

  // 租约保护
  if (!RequireLease(response->mutable_base())) {
    response->set_success(false);
    return grpc::Status::OK;
  }

  namespace fs = std::filesystem;

  // 路径安全检查: 必须在地图目录内
  if (!IsPathUnderMapDir(request->path())) {
    response->set_success(false);
    response->set_message("Path is not under map directory");
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    return grpc::Status::OK;
  }

  try {
    const fs::path path(request->path());
    if (!fs::exists(path)) {
      response->set_success(false);
      response->set_message("File not found: " + request->path());
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
      return grpc::Status::OK;
    }

    // 禁止删除当前加载的地图 (定位器仍在使用它)
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!active_map_path_.empty() &&
          active_map_path_ == request->path()) {
        response->set_success(false);
        response->set_message(
            "Cannot delete the currently loaded map — load a different map first");
        response->mutable_base()->set_error_code(
            robot::v1::ERROR_CODE_MODE_CONFLICT);
        return grpc::Status::OK;
      }
    }

    fs::remove(path);

    // 清理关联文件: .pcd 可能有配套的 .yaml + .pgm (2D 占栅格地图)
    const auto stem = path.stem().string();
    const auto parent = path.parent_path();
    static const std::vector<std::string> assoc_exts = {
        ".yaml", ".pgm", ".png", ".jpg"};
    for (const auto &ext : assoc_exts) {
      const auto assoc = parent / (stem + ext);
      if (fs::exists(assoc)) {
        fs::remove(assoc);
        RCLCPP_INFO(node_->get_logger(), "Deleted associated file: %s",
                    assoc.string().c_str());
      }
    }

    response->set_success(true);
    response->set_message("Deleted: " + request->path());
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } catch (const std::exception &e) {
    response->set_success(false);
    response->set_message(std::string("Delete failed: ") + e.what());
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
  }

  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::RenameMap(
  grpc::ServerContext *,
  const robot::v1::RenameMapRequest *request,
  robot::v1::RenameMapResponse *response) {

  response->mutable_base()->set_request_id(request->base().request_id());

  // 租约保护
  if (!RequireLease(response->mutable_base())) {
    response->set_success(false);
    return grpc::Status::OK;
  }

  namespace fs = std::filesystem;

  // 路径安全检查
  if (!IsPathUnderMapDir(request->old_path())) {
    response->set_success(false);
    response->set_message("Path is not under map directory");
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    return grpc::Status::OK;
  }

  // 新名称不能包含路径分隔符或 ".."
  const std::string &new_name = request->new_name();
  if (new_name.find('/') != std::string::npos ||
      new_name.find('\\') != std::string::npos ||
      new_name.find("..") != std::string::npos) {
    response->set_success(false);
    response->set_message("Invalid new name");
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    return grpc::Status::OK;
  }

  try {
    const fs::path old_path(request->old_path());
    if (!fs::exists(old_path)) {
      response->set_success(false);
      response->set_message("File not found: " + request->old_path());
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
      return grpc::Status::OK;
    }

    const fs::path new_path = old_path.parent_path() / new_name;
    if (fs::exists(new_path)) {
      response->set_success(false);
      response->set_message("Target already exists: " + new_path.string());
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_CONFLICT);
      return grpc::Status::OK;
    }

    fs::rename(old_path, new_path);

    // 重命名关联文件
    const auto old_stem = old_path.stem().string();
    const auto new_stem = fs::path(new_name).stem().string();
    const auto parent = old_path.parent_path();
    static const std::vector<std::string> assoc_exts = {
        ".yaml", ".pgm", ".png", ".jpg"};
    for (const auto &ext : assoc_exts) {
      const auto old_assoc = parent / (old_stem + ext);
      if (fs::exists(old_assoc)) {
        const auto new_assoc = parent / (new_stem + ext);
        fs::rename(old_assoc, new_assoc);
      }
    }

    // 如果重命名的是当前加载的地图，同步更新跟踪
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (active_map_path_ == request->old_path()) {
        active_map_path_ = new_path.string();
        active_map_name_ = new_name;
        RCLCPP_INFO(node_->get_logger(), "Active map renamed to: %s",
                    new_name.c_str());
      }
    }

    response->set_success(true);
    response->set_new_path(new_path.string());
    response->set_message("Renamed to: " + new_path.string());
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } catch (const std::exception &e) {
    response->set_success(false);
    response->set_message(std::string("Rename failed: ") + e.what());
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
  }

  return grpc::Status::OK;
}

// ================================================================
//  RuntimeConfig RPC Implementation
// ================================================================

grpc::Status SystemServiceImpl::GetRuntimeConfig(
    grpc::ServerContext *,
    const robot::v1::GetRuntimeConfigRequest *request,
    robot::v1::GetRuntimeConfigResponse *response) {

  response->mutable_base()->set_request_id(request->base().request_id());

  try {
    std::lock_guard<std::mutex> lock(config_mutex_);
    response->set_config_json(current_config_.dump());
    response->set_config_version(config_version_.load());
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } catch (const std::exception &e) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message(
        std::string("Failed to serialize config: ") + e.what());
  }

  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::SetRuntimeConfig(
    grpc::ServerContext *,
    const robot::v1::SetRuntimeConfigRequest *request,
    robot::v1::SetRuntimeConfigResponse *response) {

  response->mutable_base()->set_request_id(request->base().request_id());

  // 租约保护
  if (!RequireLease(response->mutable_base())) {
    return grpc::Status::OK;
  }

  try {
    auto incoming = nlohmann::json::parse(request->config_json());

    // 乐观锁检查
    const uint64_t expected = request->expected_version();
    const uint64_t current = config_version_.load();
    if (expected != 0 && expected != current) {
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_CONFLICT);
      response->mutable_base()->set_error_message(
          "Version conflict: expected " + std::to_string(expected) +
          " but current is " + std::to_string(current));
      return grpc::Status::OK;
    }

    // 收集变更字段
    std::vector<std::string> changed_fields(
        request->changed_fields().begin(),
        request->changed_fields().end());

    // 如果未指定变更字段，则计算差异
    if (changed_fields.empty()) {
      std::lock_guard<std::mutex> lock(config_mutex_);
      for (auto &[key, val] : incoming.items()) {
        if (!current_config_.contains(key) || current_config_[key] != val) {
          changed_fields.push_back(key);
        }
      }
    }

    // 应用配置更新
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      for (auto &[key, val] : incoming.items()) {
        current_config_[key] = val;
      }
      config_version_++;
    }

    // 转发变更的参数到 ROS 2 节点
    ForwardConfigToNodes(incoming, changed_fields);

    // 持久化到磁盘
    PersistConfigToYaml();

    // 构建响应
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      response->set_applied_config_json(current_config_.dump());
      response->set_new_version(config_version_.load());
    }
    // TODO: 实现参数 clamp 逻辑后填充 clamped_fields
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

    RCLCPP_INFO(node_->get_logger(),
                "RuntimeConfig updated: %zu fields changed, version=%lu",
                changed_fields.size(), config_version_.load());
  } catch (const nlohmann::json::parse_error &e) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message(
        std::string("Invalid JSON: ") + e.what());
  } catch (const std::exception &e) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message(
        std::string("SetRuntimeConfig failed: ") + e.what());
  }

  return grpc::Status::OK;
}

std::string SystemServiceImpl::GetCurrentConfigJson() const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  return current_config_.dump();
}

// ================================================================
//  RuntimeConfig 辅助方法
// ================================================================

/// JSON key → (目标节点, ROS2参数名) 的映射
static const std::unordered_map<std::string,
                                std::pair<std::string, std::string>> &
ParamRouteMap() {
  static const std::unordered_map<std::string,
                                  std::pair<std::string, std::string>>
      m = {
          // pathFollower 参数
          {"yaw_rate_gain", {"pathFollower", "yawRateGain"}},
          {"stop_yaw_rate_gain", {"pathFollower", "stopYawRateGain"}},
          {"max_yaw_rate", {"pathFollower", "maxYawRate"}},
          {"max_speed", {"pathFollower", "maxSpeed"}},
          {"max_accel", {"pathFollower", "maxAccel"}},
          {"look_ahead_dis", {"pathFollower", "lookAheadDis"}},
          {"min_look_ahead_dis", {"pathFollower", "minLookAheadDis"}},
          {"max_look_ahead_dis", {"pathFollower", "maxLookAheadDis"}},
          {"look_ahead_ratio", {"pathFollower", "lookAheadRatio"}},
          {"base_look_ahead_dis", {"pathFollower", "baseLookAheadDis"}},
          {"switch_time_thre", {"pathFollower", "switchTimeThre"}},
          {"dir_diff_thre", {"pathFollower", "dirDiffThre"}},
          {"incl_rate_thre", {"pathFollower", "inclRateThre"}},
          {"slow_rate1", {"pathFollower", "slowRate1"}},
          {"slow_rate2", {"pathFollower", "slowRate2"}},
          {"slow_rate3", {"pathFollower", "slowRate3"}},
          {"slow_time1", {"pathFollower", "slowTime1"}},
          {"slow_time2", {"pathFollower", "slowTime2"}},
          {"incl_thre", {"pathFollower", "inclThre"}},
          {"stop_time", {"pathFollower", "stopTime"}},
          {"pub_skip_num", {"pathFollower", "pubSkipNum"}},
          {"use_incl_rate_to_slow", {"pathFollower", "useInclRateToSlow"}},
          {"use_incl_to_stop", {"pathFollower", "useInclToStop"}},
          {"no_rot_at_stop", {"pathFollower", "noRotAtStop"}},
          {"no_rot_at_goal", {"pathFollower", "noRotAtGoal"}},
          {"autonomy_speed", {"pathFollower", "autonomySpeed"}},
          {"stop_dis_thre", {"pathFollower", "stopDisThre"}},
          {"slow_dwn_dis_thre", {"pathFollower", "slowDwnDisThre"}},

          // terrainAnalysis 参数
          {"terrain_voxel_size", {"terrainAnalysis", "terrainVoxelSize"}},
          {"obstacle_height_thre", {"terrainAnalysis", "obstacleHeightThre"}},
          {"ground_height_thre", {"terrainAnalysis", "groundHeightThre"}},
          {"quantile_z", {"terrainAnalysis", "quantileZ"}},
          {"max_ground_lift", {"terrainAnalysis", "maxGroundLift"}},
          {"use_sorting", {"terrainAnalysis", "useSorting"}},
          {"consider_drop", {"terrainAnalysis", "considerDrop"}},
          {"limit_ground_lift", {"terrainAnalysis", "limitGroundLift"}},
          {"clear_dy_obs", {"terrainAnalysis", "clearDyObs"}},
          {"no_data_obstacle", {"terrainAnalysis", "noDataObstacle"}},
          {"check_collision", {"terrainAnalysis", "checkCollision"}},

          // localPlanner 参数
          {"two_way_drive", {"localPlanner", "twoWayDrive"}},
          {"check_obstacle", {"localPlanner", "checkObstacle"}},
          {"path_scale", {"localPlanner", "pathScale"}},
          {"min_path_scale", {"localPlanner", "minPathScale"}},
          {"path_scale_step", {"localPlanner", "pathScaleStep"}},
          {"path_scale_by_speed", {"localPlanner", "pathScaleBySpeed"}},
      };
  return m;
}

void SystemServiceImpl::ForwardConfigToNodes(
    const nlohmann::json &config,
    const std::vector<std::string> &changed_fields) {

  const auto &route = ParamRouteMap();

  // 按目标节点分组
  std::unordered_map<std::string, std::vector<rclcpp::Parameter>> by_node;

  for (const auto &field : changed_fields) {
    auto it = route.find(field);
    if (it == route.end()) continue;

    const auto &[node_name, param_name] = it->second;
    if (!config.contains(field)) continue;

    const auto &val = config[field];
    if (val.is_number_float()) {
      by_node[node_name].emplace_back(param_name, val.get<double>());
    } else if (val.is_number_integer()) {
      by_node[node_name].emplace_back(param_name, val.get<int>());
    } else if (val.is_boolean()) {
      by_node[node_name].emplace_back(param_name, val.get<bool>());
    } else if (val.is_string()) {
      by_node[node_name].emplace_back(param_name, val.get<std::string>());
    }
  }

  // 批量推送到各节点
  for (auto &[node_name, params] : by_node) {
    auto client_it = param_clients_.find(node_name);
    if (client_it == param_clients_.end()) continue;

    auto &client = client_it->second;
    if (!client->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(),
                  "Parameter service for '%s' not ready, skipping %zu params",
                  node_name.c_str(), params.size());
      continue;
    }

    client->set_parameters(params,
        [this, node_name, count = params.size()](
            std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
          try {
            auto results = future.get();
            int ok_count = 0;
            for (const auto &r : results) {
              if (r.successful) ok_count++;
            }
            RCLCPP_INFO(node_->get_logger(),
                        "Forwarded %d/%zu params to '%s'",
                        ok_count, count, node_name.c_str());
          } catch (const std::exception &e) {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to forward params to '%s': %s",
                        node_name.c_str(), e.what());
          }
        });
  }
}

nlohmann::json SystemServiceImpl::CollectCurrentParams() {
  nlohmann::json result;

  for (auto &[node_name, client] : param_clients_) {
    if (!client->service_is_ready()) continue;

    try {
      // 异步获取所有参数名
      auto names_future = client->list_parameters({}, 0);
      if (names_future.wait_for(std::chrono::seconds(2)) !=
          std::future_status::ready)
        continue;

      auto param_names = names_future.get().names;
      if (param_names.empty()) continue;

      auto params_future = client->get_parameters(param_names);
      if (params_future.wait_for(std::chrono::seconds(2)) !=
          std::future_status::ready)
        continue;

      auto params = params_future.get();
      // 反向查找: ROS param name → JSON key
      const auto &route = ParamRouteMap();
      for (const auto &param : params) {
        // 查找匹配的 JSON key
        for (const auto &[json_key, target] : route) {
          if (target.first == node_name && target.second == param.get_name()) {
            switch (param.get_type()) {
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
              result[json_key] = param.as_double();
              break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
              result[json_key] = param.as_int();
              break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
              result[json_key] = param.as_bool();
              break;
            case rclcpp::ParameterType::PARAMETER_STRING:
              result[json_key] = param.as_string();
              break;
            default:
              break;
            }
            break;
          }
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to collect params from '%s': %s",
                  node_name.c_str(), e.what());
    }
  }

  return result;
}

void SystemServiceImpl::LoadConfigFromYaml() {
  namespace fs = std::filesystem;
  if (config_persist_path_.empty() || !fs::exists(config_persist_path_)) {
    RCLCPP_INFO(node_->get_logger(),
                "No persisted runtime config found at: %s",
                config_persist_path_.c_str());
    return;
  }

  try {
    std::ifstream ifs(config_persist_path_);
    if (!ifs.is_open()) return;

    // 简易 YAML → JSON 转换 (只支持顶层 key: value 格式)
    // 生产环境应使用 yaml-cpp
    std::string line;
    std::lock_guard<std::mutex> lock(config_mutex_);
    while (std::getline(ifs, line)) {
      // 跳过注释和空行
      if (line.empty() || line[0] == '#') continue;
      auto colon = line.find(':');
      if (colon == std::string::npos) continue;

      std::string key = line.substr(0, colon);
      std::string val = line.substr(colon + 1);

      // 去除首尾空格
      auto trim = [](std::string &s) {
        s.erase(0, s.find_first_not_of(" \t"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
      };
      trim(key);
      trim(val);

      if (key.empty() || val.empty()) continue;

      // 尝试解析为数字/布尔
      if (val == "true") {
        current_config_[key] = true;
      } else if (val == "false") {
        current_config_[key] = false;
      } else {
        try {
          if (val.find('.') != std::string::npos) {
            current_config_[key] = std::stod(val);
          } else {
            current_config_[key] = std::stoi(val);
          }
        } catch (...) {
          current_config_[key] = val;  // 保持为字符串
        }
      }
    }
    RCLCPP_INFO(node_->get_logger(),
                "Loaded %zu config entries from %s",
                current_config_.size(), config_persist_path_.c_str());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(),
                "Failed to load config from %s: %s",
                config_persist_path_.c_str(), e.what());
  }
}

void SystemServiceImpl::PersistConfigToYaml() {
  if (config_persist_path_.empty()) return;

  try {
    namespace fs = std::filesystem;
    // 确保父目录存在
    const auto parent = fs::path(config_persist_path_).parent_path();
    if (!parent.empty() && !fs::exists(parent)) {
      fs::create_directories(parent);
    }

    // 写入临时文件后原子替换
    const std::string tmp_path = config_persist_path_ + ".tmp";
    {
      std::ofstream ofs(tmp_path);
      if (!ofs.is_open()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Cannot open config persist file: %s", tmp_path.c_str());
        return;
      }

      ofs << "# RuntimeConfig — auto-generated by grpc_gateway\n";
      ofs << "# version: " << config_version_.load() << "\n\n";

      std::lock_guard<std::mutex> lock(config_mutex_);
      for (auto &[key, val] : current_config_.items()) {
        if (val.is_number_float()) {
          ofs << key << ": " << val.get<double>() << "\n";
        } else if (val.is_number_integer()) {
          ofs << key << ": " << val.get<int>() << "\n";
        } else if (val.is_boolean()) {
          ofs << key << ": " << (val.get<bool>() ? "true" : "false") << "\n";
        } else if (val.is_string()) {
          ofs << key << ": " << val.get<std::string>() << "\n";
        }
      }
    }

    fs::rename(tmp_path, config_persist_path_);
    RCLCPP_DEBUG(node_->get_logger(),
                 "Config persisted to %s (version=%lu)",
                 config_persist_path_.c_str(), config_version_.load());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(),
                "Failed to persist config: %s", e.what());
  }
}

}  // namespace services
}  // namespace remote_monitoring
