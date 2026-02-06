#include "remote_monitoring/grpc_gateway.hpp"
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/idempotency_cache.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/safety_gate.hpp"
#include "remote_monitoring/services/control_service.hpp"
#include "remote_monitoring/services/data_service.hpp"
#include "remote_monitoring/services/system_service.hpp"
#include "remote_monitoring/services/telemetry_service.hpp"
#include "remote_monitoring/status_aggregator.hpp"


namespace remote_monitoring {

GrpcGateway::GrpcGateway(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<int>("grpc_port", 50051);
  port_ = node_->get_parameter("grpc_port").as_int();

  // 创建核心组件
  aggregator_ = std::make_shared<StatusAggregator>(node_);
  lease_mgr_ = std::make_shared<core::LeaseManager>();
  event_buffer_ = std::make_shared<core::EventBuffer>(1000);
  safety_gate_ = std::make_shared<core::SafetyGate>(node_);
  idempotency_cache_ = std::make_shared<core::IdempotencyCache>();

  // 创建服务实现
  system_service_ = std::make_shared<services::SystemServiceImpl>();
  control_service_ =
      std::make_shared<services::ControlServiceImpl>(lease_mgr_, safety_gate_,
                                                     event_buffer_,
                                                     idempotency_cache_);
  telemetry_service_ = std::make_shared<services::TelemetryServiceImpl>(
      aggregator_, event_buffer_);
  data_service_ = std::make_shared<services::DataServiceImpl>(node_);

  // 让 StatusAggregator 可以获取真实模式
  aggregator_->SetModeProvider([this]() {
    return control_service_->GetCurrentMode();
  });
}

GrpcGateway::~GrpcGateway() { Stop(); }

void GrpcGateway::Start() {
  thread_ = std::thread([this]() { Run(); });
}

void GrpcGateway::Stop() {
  stop_.store(true);
  if (server_) {
    server_->Shutdown();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GrpcGateway::Run() {
  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:" + std::to_string(port_),
                           grpc::InsecureServerCredentials());

  // 注册所有服务
  builder.RegisterService(system_service_.get());
  builder.RegisterService(control_service_.get());
  builder.RegisterService(telemetry_service_.get());
  builder.RegisterService(data_service_.get());

  server_ = builder.BuildAndStart();
  RCLCPP_INFO(rclcpp::get_logger("grpc_gateway"),
              "gRPC Gateway listening on :%d", port_);

  // 后台循环：Safety Gate deadman 必须高频检查（≤100ms），
  // Lease 超时和幂等缓存清理频率可以较低（1s）。
  auto last_slow_check = std::chrono::steady_clock::now();
  while (!stop_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 每次循环都检查 deadman（~50ms 间隔，远小于 300ms 超时）
    safety_gate_->CheckDeadman();

    const auto now = std::chrono::steady_clock::now();
    if (now - last_slow_check > std::chrono::seconds(1)) {
      lease_mgr_->CheckTimeout();
      idempotency_cache_->Cleanup();
      last_slow_check = now;
    }
  }
}

} // namespace remote_monitoring
