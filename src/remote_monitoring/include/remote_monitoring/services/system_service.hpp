#pragma once

#include <memory>
#include <string>

#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"
#include "system.grpc.pb.h"

#include "interface/srv/relocalize.hpp"
#include "interface/srv/save_maps.hpp"

namespace remote_monitoring {
namespace services {

class SystemServiceImpl final : public robot::v1::SystemService::Service {
public:
  /// 构造函数，接受 ROS2 node 用于调用 service client。
  SystemServiceImpl(rclcpp::Node *node,
                    const std::string &robot_id = "robot_001",
                    const std::string &firmware_version = "1.0.0");
  
  grpc::Status Login(grpc::ServerContext *context,
                     const robot::v1::LoginRequest *request,
                     robot::v1::LoginResponse *response) override;
  
  grpc::Status Logout(grpc::ServerContext *context,
                      const robot::v1::LogoutRequest *request,
                      robot::v1::LogoutResponse *response) override;
  
  grpc::Status Heartbeat(grpc::ServerContext *context,
                         const robot::v1::HeartbeatRequest *request,
                         robot::v1::HeartbeatResponse *response) override;
  
  grpc::Status GetRobotInfo(grpc::ServerContext *context,
                            const google::protobuf::Empty *request,
                            robot::v1::RobotInfoResponse *response) override;
  
  grpc::Status GetCapabilities(grpc::ServerContext *context,
                               const google::protobuf::Empty *request,
                               robot::v1::CapabilitiesResponse *response) override;

  grpc::Status Relocalize(grpc::ServerContext *context,
                          const robot::v1::RelocalizeRequest *request,
                          robot::v1::RelocalizeResponse *response) override;

  grpc::Status SaveMap(grpc::ServerContext *context,
                       const robot::v1::SaveMapRequest *request,
                       robot::v1::SaveMapResponse *response) override;

  /// 返回自上次心跳以来的秒数 (供 GrpcGateway 检查断联)
  double SecondsSinceLastHeartbeat() const;

private:
  rclcpp::Node *node_;
  std::string robot_id_;
  std::string firmware_version_;

  // ROS 2 Service Clients
  rclcpp::Client<interface::srv::Relocalize>::SharedPtr relocalize_client_;
  rclcpp::Client<interface::srv::SaveMaps>::SharedPtr save_map_client_;

  // 心跳时间追踪 (用于断联检测)
  mutable std::mutex heartbeat_mutex_;
  std::chrono::steady_clock::time_point last_heartbeat_{
      std::chrono::steady_clock::now()};
  bool heartbeat_received_{false};

  static constexpr std::chrono::seconds kServiceTimeout{10};
};

}  // namespace services
}  // namespace remote_monitoring
