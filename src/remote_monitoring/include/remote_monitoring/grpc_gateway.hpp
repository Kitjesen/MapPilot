#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <thread>

#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

namespace remote_monitoring {

class StatusAggregator;

namespace core {
class LeaseManager;
class EventBuffer;
class SafetyGate;
class IdempotencyCache;
class GeofenceMonitor;
class HealthMonitor;
class ModeManager;
class TaskManager;
class ServiceOrchestrator;
class LocalizationScorer;
class FlightRecorder;
}

namespace services {
class SystemServiceImpl;
class ControlServiceImpl;
class TelemetryServiceImpl;
class DataServiceImpl;
}

class GrpcGateway {
public:
  explicit GrpcGateway(rclcpp::Node *node);
  ~GrpcGateway();
  
  void Start();
  void Stop();

private:
  void Run();
  
  rclcpp::Node *node_;
  int port_{50051};
  
  // 核心组件
  std::shared_ptr<StatusAggregator> aggregator_;
  std::shared_ptr<core::LeaseManager> lease_mgr_;
  std::shared_ptr<core::EventBuffer> event_buffer_;
  std::shared_ptr<core::SafetyGate> safety_gate_;
  std::shared_ptr<core::IdempotencyCache> idempotency_cache_;
  std::shared_ptr<core::ModeManager> mode_manager_;
  std::shared_ptr<core::GeofenceMonitor> geofence_monitor_;
  std::shared_ptr<core::HealthMonitor> health_monitor_;
  std::shared_ptr<core::TaskManager> task_manager_;
  std::shared_ptr<core::ServiceOrchestrator> service_orchestrator_;
  std::shared_ptr<core::LocalizationScorer> localization_scorer_;
  std::shared_ptr<core::FlightRecorder> flight_recorder_;
  
  // 服务实现
  std::shared_ptr<services::SystemServiceImpl> system_service_;
  std::shared_ptr<services::ControlServiceImpl> control_service_;
  std::shared_ptr<services::TelemetryServiceImpl> telemetry_service_;
  std::shared_ptr<services::DataServiceImpl> data_service_;
  
  std::unique_ptr<grpc::Server> server_;
  std::thread thread_;
  std::atomic<bool> stop_{false};

  // TLS 配置 (Phase 3.3)
  std::string tls_cert_path_;
  std::string tls_key_path_;

  // 断联降级
  void CheckDisconnection();
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_slow_down_;
  int last_disconnect_level_{0};  // 0=正常, 1=减速, 2=停车

  // 崩溃恢复: 启动时检查上次状态, 清理残留服务
  void RecoverFromCrash();

  // 状态持久化文件路径
  static constexpr const char *kStateFilePath = "/tmp/nav_gateway_state";
};

}  // namespace remote_monitoring
