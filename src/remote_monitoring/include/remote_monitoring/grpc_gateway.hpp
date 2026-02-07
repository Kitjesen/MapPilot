#pragma once

#include <memory>
#include <atomic>
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
  
  // 服务实现
  std::shared_ptr<services::SystemServiceImpl> system_service_;
  std::shared_ptr<services::ControlServiceImpl> control_service_;
  std::shared_ptr<services::TelemetryServiceImpl> telemetry_service_;
  std::shared_ptr<services::DataServiceImpl> data_service_;
  
  std::unique_ptr<grpc::Server> server_;
  std::thread thread_;
  std::atomic<bool> stop_{false};

  // 断联降级
  void CheckDisconnection();
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_slow_down_;
  int last_disconnect_level_{0};  // 0=正常, 1=减速, 2=停车
};

}  // namespace remote_monitoring
