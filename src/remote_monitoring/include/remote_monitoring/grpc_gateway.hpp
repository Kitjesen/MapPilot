#pragma once

#include <memory>
#include <atomic>
#include <thread>

#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"

namespace remote_monitoring {

class StatusAggregator;

namespace core {
class LeaseManager;
class EventBuffer;
class SafetyGate;
class IdempotencyCache;
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
  
  // 服务实现
  std::shared_ptr<services::SystemServiceImpl> system_service_;
  std::shared_ptr<services::ControlServiceImpl> control_service_;
  std::shared_ptr<services::TelemetryServiceImpl> telemetry_service_;
  std::shared_ptr<services::DataServiceImpl> data_service_;
  
  std::unique_ptr<grpc::Server> server_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace remote_monitoring
