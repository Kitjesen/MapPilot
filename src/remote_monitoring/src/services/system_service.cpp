#include "remote_monitoring/services/system_service.hpp"

#include <chrono>

namespace remote_monitoring {
namespace services {

SystemServiceImpl::SystemServiceImpl(rclcpp::Node *node,
                                     const std::string &robot_id,
                                     const std::string &firmware_version)
    : node_(node), robot_id_(robot_id), firmware_version_(firmware_version) {
  relocalize_client_ =
      node_->create_client<interface::srv::Relocalize>("/relocalize");
  save_map_client_ =
      node_->create_client<interface::srv::SaveMaps>("/save_map");
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

}  // namespace services
}  // namespace remote_monitoring
