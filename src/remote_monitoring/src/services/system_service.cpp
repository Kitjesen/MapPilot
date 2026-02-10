#include "remote_monitoring/services/system_service.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

namespace remote_monitoring {
namespace services {

SystemServiceImpl::SystemServiceImpl(rclcpp::Node *node,
                                     const std::string &robot_id,
                                     const std::string &firmware_version)
    : node_(node), robot_id_(robot_id), firmware_version_(firmware_version) {
  relocalize_client_ =
      node_->create_client<interface::srv::Relocalize>("/relocalize");

  // save_map 服务名：lio_node 可能带命名空间 (如 /fastlio2/save_map)
  std::string save_map_srv = "/save_map";
  if (node_->has_parameter("save_map_service")) {
    save_map_srv = node_->get_parameter("save_map_service").as_string();
  }
  save_map_client_ =
      node_->create_client<interface::srv::SaveMaps>(save_map_srv);
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
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::SaveMap(
  grpc::ServerContext *,
  const robot::v1::SaveMapRequest *request,
  robot::v1::SaveMapResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());

  if (!save_map_client_->wait_for_service(std::chrono::seconds(2))) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->mutable_base()->set_error_message(
        "SaveMap service not available");
    response->set_success(false);
    return grpc::Status::OK;
  }

  auto ros_req = std::make_shared<interface::srv::SaveMaps::Request>();
  ros_req->file_path = request->file_path();
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
  
  return grpc::Status::OK;
}

grpc::Status SystemServiceImpl::ListMaps(
  grpc::ServerContext *,
  const robot::v1::ListMapsRequest *request,
  robot::v1::ListMapsResponse *response) {

  response->mutable_base()->set_request_id(request->base().request_id());

  std::string directory = request->directory();
  if (directory.empty()) {
    directory = "/maps";
  }

  namespace fs = std::filesystem;
  try {
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
      response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
      return grpc::Status::OK;  // Empty list
    }

    for (const auto &entry : fs::directory_iterator(directory)) {
      if (!entry.is_regular_file()) continue;

      const auto ext = entry.path().extension().string();
      // Accept PCD, PLY, and other map formats
      if (ext != ".pcd" && ext != ".ply" && ext != ".pickle" && ext != ".pgm") {
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

  namespace fs = std::filesystem;
  try {
    const fs::path path(request->path());
    if (!fs::exists(path)) {
      response->set_success(false);
      response->set_message("File not found: " + request->path());
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
      return grpc::Status::OK;
    }

    fs::remove(path);
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

  namespace fs = std::filesystem;
  try {
    const fs::path old_path(request->old_path());
    if (!fs::exists(old_path)) {
      response->set_success(false);
      response->set_message("File not found: " + request->old_path());
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
      return grpc::Status::OK;
    }

    const fs::path new_path = old_path.parent_path() / request->new_name();
    if (fs::exists(new_path)) {
      response->set_success(false);
      response->set_message("Target already exists: " + new_path.string());
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_RESOURCE_CONFLICT);
      return grpc::Status::OK;
    }

    fs::rename(old_path, new_path);
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

}  // namespace services
}  // namespace remote_monitoring
