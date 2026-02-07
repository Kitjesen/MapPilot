#include "remote_monitoring/grpc_gateway.hpp"
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/geofence_monitor.hpp"
#include "remote_monitoring/core/health_monitor.hpp"
#include "remote_monitoring/core/idempotency_cache.hpp"
#include "remote_monitoring/core/task_manager.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/mode_manager.hpp"
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

  // 创建核心组件 (解耦: 每个模块独立构造)
  aggregator_ = std::make_shared<StatusAggregator>(node_);
  lease_mgr_ = std::make_shared<core::LeaseManager>();
  event_buffer_ = std::make_shared<core::EventBuffer>(1000);
  safety_gate_ = std::make_shared<core::SafetyGate>(node_);
  idempotency_cache_ = std::make_shared<core::IdempotencyCache>();

  // Layer 2: 围栏监控 (独立模块, 不依赖 SafetyGate/ModeManager)
  geofence_monitor_ = std::make_shared<core::GeofenceMonitor>(node_);
  geofence_monitor_->SetEventBuffer(event_buffer_);

  // Layer 3: 形式化状态机
  mode_manager_ = std::make_shared<core::ModeManager>(node_);
  mode_manager_->SetSafetyGate(safety_gate_);
  mode_manager_->SetEventBuffer(event_buffer_);
  mode_manager_->SetGeofenceMonitor(geofence_monitor_);

  // 注入转换守卫: 将各独立模块的状态查询接线到 ModeManager
  mode_manager_->SetTransitionGuards({
      .tf_ok =
          [this]() { return aggregator_->GetFastState().tf_ok(); },
      .localization_valid =
          [this]() {
            // map→odom TF 由 Localizer/PGO 发布，有效即定位可用
            return aggregator_->GetFastState().tf_ok();
          },
      .has_lease =
          [this]() { return lease_mgr_->HasActiveLease(); },
      .slam_running =
          [this]() {
            return aggregator_->GetSlowState().topic_rates().odom_hz() > 20.0f;
          },
      .estop_clear =
          [this]() {
            return !safety_gate_->GetSafetyStatus().estop_active();
          },
      .tilt_safe =
          [this]() {
            return !safety_gate_->GetSafetyStatus().tilt_limit_active();
          },
      .fence_safe =
          [this]() {
            return geofence_monitor_->GetState() !=
                   core::GeofenceState::VIOLATION;
          },
  });

  // Layer 4: 健康监控 (独立模块)
  health_monitor_ = std::make_shared<core::HealthMonitor>(node_);
  health_monitor_->SetEventBuffer(event_buffer_);

  // 健康监控降级回调 → ModeManager 急停 (回调为 best-effort, 不是唯一停车路径)
  health_monitor_->SetDegradeCallback(
      [this](core::HealthLevel level, const std::string &reason) {
        if (level == core::HealthLevel::FAULT && mode_manager_) {
          mode_manager_->EmergencyStop("health: " + reason);
        }
      });

  // 任务管理器
  task_manager_ = std::make_shared<core::TaskManager>(node_);
  task_manager_->SetEventBuffer(event_buffer_);

  // 创建服务实现
  system_service_ = std::make_shared<services::SystemServiceImpl>(node_);
  control_service_ =
      std::make_shared<services::ControlServiceImpl>(
          lease_mgr_, safety_gate_, mode_manager_,
          event_buffer_, idempotency_cache_, task_manager_);
  telemetry_service_ = std::make_shared<services::TelemetryServiceImpl>(
      aggregator_, event_buffer_);
  data_service_ = std::make_shared<services::DataServiceImpl>(node_);

  // 让 StatusAggregator 可以获取真实模式
  aggregator_->SetModeProvider([this]() {
    return control_service_->GetCurrentMode();
  });

  // 注入健康和围栏监控到 StatusAggregator (用于 SlowState 推送)
  aggregator_->SetHealthMonitor(health_monitor_);
  aggregator_->SetGeofenceMonitor(geofence_monitor_);

  // 断联降级: 发布 /slow_down
  pub_slow_down_ = node_->create_publisher<std_msgs::msg::Int8>("/slow_down", 5);
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
      CheckDisconnection();
      last_slow_check = now;
    }
  }
}

void GrpcGateway::CheckDisconnection() {
  // 仅在 AUTONOMOUS 模式下检查断联
  const auto mode = mode_manager_->GetCurrentMode();
  if (mode != robot::v1::ROBOT_MODE_AUTONOMOUS) {
    last_disconnect_level_ = 0;
    return;
  }

  const double secs = system_service_->SecondsSinceLastHeartbeat();
  if (secs <= 0.0) {
    // 从未收到心跳 — 不触发降级
    return;
  }

  int new_level = 0;

  if (secs > 300.0) {
    // > 5min: 切换 IDLE, 原地停车
    new_level = 2;
    if (last_disconnect_level_ < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("grpc_gateway"),
                   "Client disconnected for %.0fs — switching to IDLE", secs);
      mode_manager_->SwitchMode(robot::v1::ROBOT_MODE_IDLE);
      if (event_buffer_) {
        event_buffer_->AddEvent(
            robot::v1::EVENT_TYPE_NETWORK_DISCONNECTED,
            robot::v1::EVENT_SEVERITY_CRITICAL,
            "Client disconnected > 5min — stopping",
            "Elapsed: " + std::to_string(static_cast<int>(secs)) + "s");
      }
    }
  } else if (secs > 30.0) {
    // 30s-5min: 减速 50%
    new_level = 1;
    if (last_disconnect_level_ < 1) {
      RCLCPP_WARN(rclcpp::get_logger("grpc_gateway"),
                  "Client disconnected for %.0fs — reducing speed 50%%", secs);
      std_msgs::msg::Int8 msg;
      msg.data = 2;  // slow_down level 2 = 50%
      pub_slow_down_->publish(msg);
      if (event_buffer_) {
        event_buffer_->AddEvent(
            robot::v1::EVENT_TYPE_NETWORK_DISCONNECTED,
            robot::v1::EVENT_SEVERITY_WARNING,
            "Client disconnected > 30s — reducing speed",
            "Elapsed: " + std::to_string(static_cast<int>(secs)) + "s");
      }
    }
  } else {
    // < 30s: 正常
    if (last_disconnect_level_ > 0) {
      RCLCPP_INFO(rclcpp::get_logger("grpc_gateway"),
                  "Client reconnected — restoring normal speed");
      std_msgs::msg::Int8 msg;
      msg.data = 0;
      pub_slow_down_->publish(msg);
      if (event_buffer_) {
        event_buffer_->AddEvent(
            robot::v1::EVENT_TYPE_NETWORK_RECONNECTED,
            robot::v1::EVENT_SEVERITY_INFO,
            "Client reconnected — speed restored", "");
      }
    }
  }

  last_disconnect_level_ = new_level;
}

} // namespace remote_monitoring
