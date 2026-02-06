#pragma once

#include <memory>
#include <string>

#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"
#include "system.grpc.pb.h"

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

private:
  rclcpp::Node *node_;
  std::string robot_id_;
  std::string firmware_version_;
};

}  // namespace services
}  // namespace remote_monitoring
