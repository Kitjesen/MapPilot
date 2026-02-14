#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>

#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"
#include "system.grpc.pb.h"

#include "interface/srv/relocalize.hpp"
#include "interface/srv/save_maps.hpp"

#include <nlohmann/json.hpp>

namespace remote_monitoring {
namespace core { class LeaseManager; class TaskManager; }
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

  grpc::Status ListMaps(grpc::ServerContext *context,
                        const robot::v1::ListMapsRequest *request,
                        robot::v1::ListMapsResponse *response) override;

  grpc::Status DeleteMap(grpc::ServerContext *context,
                         const robot::v1::DeleteMapRequest *request,
                         robot::v1::DeleteMapResponse *response) override;

  grpc::Status RenameMap(grpc::ServerContext *context,
                         const robot::v1::RenameMapRequest *request,
                         robot::v1::RenameMapResponse *response) override;

  grpc::Status GetRuntimeConfig(grpc::ServerContext *context,
                                const robot::v1::GetRuntimeConfigRequest *request,
                                robot::v1::GetRuntimeConfigResponse *response) override;

  grpc::Status SetRuntimeConfig(grpc::ServerContext *context,
                                const robot::v1::SetRuntimeConfigRequest *request,
                                robot::v1::SetRuntimeConfigResponse *response) override;

  /// 获取当前配置 JSON (供 StatusAggregator 填充 SlowState)
  std::string GetCurrentConfigJson() const;
  uint64_t GetConfigVersion() const { return config_version_.load(); }

  /// 注入 LeaseManager (可选, 用于 Logout 时释放租约 + 写操作保护)
  void SetLeaseManager(std::shared_ptr<core::LeaseManager> lease_mgr) {
    lease_mgr_ = std::move(lease_mgr);
  }

  /// 注入 TaskManager (可选, 用于 Relocalize 安全检查)
  void SetTaskManager(std::shared_ptr<core::TaskManager> task_mgr) {
    task_mgr_ = std::move(task_mgr);
  }

  /// 获取当前加载的地图名 (Relocalize 成功后设置)
  std::string GetActiveMapName() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return active_map_name_;
  }

  /// 返回自上次心跳以来的秒数 (供 GrpcGateway 检查断联)
  double SecondsSinceLastHeartbeat() const;

private:
  /// 检查路径是否在允许的地图目录内 (防止目录遍历)
  bool IsPathUnderMapDir(const std::string &path) const;
  /// 确保写操作有租约保护
  bool RequireLease(robot::v1::BaseResponse *base);

  std::shared_ptr<core::LeaseManager> lease_mgr_;
  std::shared_ptr<core::TaskManager> task_mgr_;
  rclcpp::Node *node_;
  std::string robot_id_;
  std::string firmware_version_;
  std::string map_directory_;  // 可配置的地图存储目录

  // ROS 2 Service Clients
  rclcpp::Client<interface::srv::Relocalize>::SharedPtr relocalize_client_;
  rclcpp::Client<interface::srv::SaveMaps>::SharedPtr save_map_client_;

  // 心跳时间追踪 (用于断联检测)
  mutable std::mutex heartbeat_mutex_;
  std::chrono::steady_clock::time_point last_heartbeat_{
      std::chrono::steady_clock::now()};
  bool heartbeat_received_{false};

  // 当前加载的地图
  mutable std::mutex map_mutex_;
  std::string active_map_name_;
  std::string active_map_path_;

  // ── RuntimeConfig 状态 ──
  mutable std::mutex config_mutex_;
  nlohmann::json current_config_;         // 当前生效的配置 (JSON)
  std::atomic<uint64_t> config_version_{0};
  std::string config_persist_path_;       // 持久化路径 (robot_config.yaml)

  /// 从 robot_config.yaml 加载初始配置
  void LoadConfigFromYaml();
  /// 持久化配置到 robot_config.yaml
  void PersistConfigToYaml();
  /// 将 JSON 配置推送到 ROS 2 节点参数
  void ForwardConfigToNodes(const nlohmann::json &config,
                            const std::vector<std::string> &changed_fields);
  /// 收集所有导航节点的当前参数到 JSON
  nlohmann::json CollectCurrentParams();

  // ROS 2 parameter client (用于向其他节点推送参数)
  using AsyncParamClient = rclcpp::AsyncParametersClient;
  std::unordered_map<std::string, std::shared_ptr<AsyncParamClient>> param_clients_;

  static constexpr std::chrono::seconds kServiceTimeout{10};
};

}  // namespace services
}  // namespace remote_monitoring
