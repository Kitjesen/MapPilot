#include "remote_monitoring/grpc_gateway.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/security/server_credentials.h>
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/geofence_monitor.hpp"
#include "remote_monitoring/core/health_monitor.hpp"
#include "remote_monitoring/core/idempotency_cache.hpp"
#include "remote_monitoring/core/task_manager.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/mode_manager.hpp"
#include "remote_monitoring/core/safety_gate.hpp"
#include "remote_monitoring/core/service_orchestrator.hpp"
#include "remote_monitoring/services/control_service.hpp"
#include "remote_monitoring/services/data_service.hpp"
#include "remote_monitoring/services/system_service.hpp"
#include "remote_monitoring/services/telemetry_service.hpp"
#include "remote_monitoring/status_aggregator.hpp"


namespace remote_monitoring {

GrpcGateway::GrpcGateway(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<int>("grpc_port", 50051);
  node_->declare_parameter<std::string>("tls_cert_path", "");
  node_->declare_parameter<std::string>("tls_key_path", "");
  port_ = node_->get_parameter("grpc_port").as_int();
  tls_cert_path_ = node_->get_parameter("tls_cert_path").as_string();
  tls_key_path_ = node_->get_parameter("tls_key_path").as_string();

  // 创建核心组件 (解耦: 每个模块独立构造)
  aggregator_ = std::make_shared<StatusAggregator>(node_);
  lease_mgr_ = std::make_shared<core::LeaseManager>();
  event_buffer_ = std::make_shared<core::EventBuffer>(1000);
  event_buffer_->EnablePersistence("/opt/robot/logs/events.jsonl",
                                   10 * 1024 * 1024);  // 10 MB, 自动轮转
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
  mode_manager_->SetStatePersistPath(kStateFilePath);

  // Layer 3.5: 服务编排器 — 根据模式切换自动启停 systemd 服务
  service_orchestrator_ = std::make_shared<core::ServiceOrchestrator>(node_);
  service_orchestrator_->SetEventBuffer(event_buffer_);
  service_orchestrator_->SetModeProvider(
      [this]() { return mode_manager_->GetCurrentMode(); });
  service_orchestrator_->SetServiceCrashCallback(
      [this](const std::string &svc, robot::v1::RobotMode /*expected_mode*/) {
        // 服务崩溃 → 触发 ESTOP (安全优先)
        if (mode_manager_) {
          mode_manager_->EmergencyStop("service_crash: " + svc);
        }
      });
  mode_manager_->SetServiceOrchestrator(service_orchestrator_);

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

  // 健康监控: 注入模式查询 (IDLE 模式下不触发 ESTOP)
  health_monitor_->SetModeProvider([this]() {
    return static_cast<int>(mode_manager_->GetCurrentMode());
  });

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

  // 创建服务实现 (Phase 1.5: 传入配置化的 robot_id 和 firmware_version)
  {
    if (!node_->has_parameter("robot_id"))
      node_->declare_parameter<std::string>("robot_id", "robot_001");
    if (!node_->has_parameter("firmware_version"))
      node_->declare_parameter<std::string>("firmware_version", "1.0.0");
    if (!node_->has_parameter("save_map_service"))
      node_->declare_parameter<std::string>("save_map_service", "/save_map");
    std::string rid = node_->get_parameter("robot_id").as_string();
    std::string fwv = node_->get_parameter("firmware_version").as_string();
    system_service_ = std::make_shared<services::SystemServiceImpl>(node_, rid, fwv);
  }
  control_service_ =
      std::make_shared<services::ControlServiceImpl>(
          lease_mgr_, safety_gate_, mode_manager_,
          event_buffer_, idempotency_cache_, task_manager_);
  telemetry_service_ = std::make_shared<services::TelemetryServiceImpl>(
      aggregator_, event_buffer_);
  data_service_ = std::make_shared<services::DataServiceImpl>(node_);
  // OTA 已迁移到独立的 ota_daemon (:50052), DataService 只负责数据转发

  // 让 StatusAggregator 可以获取真实模式
  aggregator_->SetModeProvider([this]() {
    return control_service_->GetCurrentMode();
  });

  // 注入健康和围栏监控到 StatusAggregator (用于 SlowState 推送)
  aggregator_->SetHealthMonitor(health_monitor_);
  aggregator_->SetGeofenceMonitor(geofence_monitor_);

  // 断联降级: 发布 /slow_down
  pub_slow_down_ = node_->create_publisher<std_msgs::msg::Int8>("/slow_down", 5);

  // 崩溃恢复: 检查上次运行状态, 清理残留 nav 服务
  RecoverFromCrash();

  // 启动服务巡检 (每 10 秒检查一次 systemctl is-active)
  service_orchestrator_->StartPeriodicCheck(10);
}

// ================================================================
//  崩溃恢复: 读取上次模式, 如果非 IDLE 则停止所有导航服务
// ================================================================
void GrpcGateway::RecoverFromCrash() {
  int prev_mode = -1;
  long prev_timestamp = 0;

  // 1. 尝试读取上次状态文件
  try {
    std::ifstream ifs(kStateFilePath);
    if (ifs.is_open()) {
      std::string line;
      while (std::getline(ifs, line)) {
        if (line.rfind("mode=", 0) == 0) {
          prev_mode = std::stoi(line.substr(5));
        } else if (line.rfind("timestamp=", 0) == 0) {
          prev_timestamp = std::stol(line.substr(10));
        }
      }
    }
  } catch (...) {
    // 文件不存在或格式错误 — 首次启动, 无需恢复
    prev_mode = -1;
  }

  // 2. 写入当前 IDLE 状态 (标记干净启动)
  mode_manager_->PersistState(robot::v1::ROBOT_MODE_IDLE);

  if (prev_mode < 0) {
    RCLCPP_INFO(node_->get_logger(),
                "CrashRecovery: no previous state file, clean start");
    return;
  }

  // 3. 如果上次不是 IDLE, 说明可能崩溃了 — 保守处理
  if (prev_mode != static_cast<int>(robot::v1::ROBOT_MODE_IDLE)) {
    RCLCPP_WARN(node_->get_logger(),
                "CrashRecovery: previous mode was %d (timestamp=%ld), "
                "stopping all nav services for safety",
                prev_mode, prev_timestamp);

    if (service_orchestrator_) {
      service_orchestrator_->StopAllNavServices();
    }

    if (event_buffer_) {
      event_buffer_->AddEvent(
          robot::v1::EVENT_TYPE_SYSTEM_BOOT,
          robot::v1::EVENT_SEVERITY_WARNING,
          "Crash recovery: stopped nav services",
          "Previous mode was " + std::to_string(prev_mode) +
              " (not IDLE). All nav services stopped for safety. "
              "Previous timestamp: " + std::to_string(prev_timestamp));
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "CrashRecovery: previous shutdown was clean (IDLE)");
  }
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
  // 启用 gRPC reflection，让 grpcurl 等工具可以直接发现服务
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();

  grpc::ServerBuilder builder;

  // Phase 3.3: TLS 支持 — 当 tls_cert_path 和 tls_key_path 都配置时启用
  bool use_tls = !tls_cert_path_.empty() && !tls_key_path_.empty();
  if (use_tls) {
    std::string cert_contents, key_contents;
    {
      std::ifstream cert_file(tls_cert_path_);
      std::ifstream key_file(tls_key_path_);
      if (cert_file.is_open() && key_file.is_open()) {
        std::ostringstream cert_ss, key_ss;
        cert_ss << cert_file.rdbuf();
        key_ss << key_file.rdbuf();
        cert_contents = cert_ss.str();
        key_contents = key_ss.str();
      } else {
        RCLCPP_WARN(rclcpp::get_logger("grpc_gateway"),
                    "TLS cert/key files not readable, falling back to insecure");
        use_tls = false;
      }
    }
    if (use_tls && !cert_contents.empty() && !key_contents.empty()) {
      grpc::SslServerCredentialsOptions ssl_opts;
      ssl_opts.pem_key_cert_pairs.push_back({key_contents, cert_contents});
      // No client cert required (trust-on-first-use model, like SSH)
      ssl_opts.client_certificate_request =
          GRPC_SSL_DONT_REQUEST_CLIENT_CERTIFICATE;
      builder.AddListeningPort("0.0.0.0:" + std::to_string(port_),
                               grpc::SslServerCredentials(ssl_opts));
    }
  }
  if (!use_tls) {
    builder.AddListeningPort("0.0.0.0:" + std::to_string(port_),
                             grpc::InsecureServerCredentials());
  }

  // 增大 HTTP/2 流控窗口和消息大小，避免视频流帧率被限制
  builder.SetMaxSendMessageSize(4 * 1024 * 1024);     // 4 MB per message
  builder.SetMaxReceiveMessageSize(4 * 1024 * 1024);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_STREAM_LOOKAHEAD_BYTES,
                             2 * 1024 * 1024);          // 2 MB stream window
  builder.AddChannelArgument(GRPC_ARG_HTTP2_BDP_PROBE, 1);  // 自动探测带宽

  // 注册所有服务
  builder.RegisterService(system_service_.get());
  builder.RegisterService(control_service_.get());
  builder.RegisterService(telemetry_service_.get());
  builder.RegisterService(data_service_.get());

  server_ = builder.BuildAndStart();
  RCLCPP_INFO(rclcpp::get_logger("grpc_gateway"),
              "gRPC Gateway listening on :%d (%s)",
              port_, use_tls ? "TLS" : "insecure");

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
