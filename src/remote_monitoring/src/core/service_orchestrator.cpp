#include "remote_monitoring/core/service_orchestrator.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>

namespace remote_monitoring {
namespace core {

// ── 静态常量定义 ──
constexpr const char *ServiceOrchestrator::kManagedServices[];

// ── 辅助: 执行 shell 命令并获取 stdout ──
static std::string ExecCommand(const std::string &cmd) {
  std::array<char, 256> buf;
  std::string result;
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) return "";
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    result += buf.data();
  }
  pclose(pipe);
  // 去掉末尾换行
  while (!result.empty() &&
         (result.back() == '\n' || result.back() == '\r')) {
    result.pop_back();
  }
  return result;
}

// ================================================================

ServiceOrchestrator::ServiceOrchestrator(rclcpp::Node *node) : node_(node) {
  RCLCPP_INFO(node_->get_logger(),
              "ServiceOrchestrator initialized with %zu managed services",
              kNumManagedServices);
}

ServiceOrchestrator::~ServiceOrchestrator() { StopPeriodicCheck(); }

// ================================================================
//  后台巡检: 定期 systemctl is-active 确认服务存活
// ================================================================

void ServiceOrchestrator::StartPeriodicCheck(int interval_sec) {
  if (check_thread_.joinable()) return;  // 已在运行
  check_stop_.store(false);
  check_thread_ = std::thread([this, interval_sec]() {
    PeriodicCheckLoop(interval_sec);
  });
  RCLCPP_INFO(node_->get_logger(),
              "ServiceOrchestrator: periodic check started (every %ds)",
              interval_sec);
}

void ServiceOrchestrator::StopPeriodicCheck() {
  check_stop_.store(true);
  if (check_thread_.joinable()) {
    check_thread_.join();
  }
}

void ServiceOrchestrator::PeriodicCheckLoop(int interval_sec) {
  while (!check_stop_.load()) {
    // 细粒度睡眠: 每 500ms 检查一次 stop 标志, 避免析构时长时间阻塞
    for (int i = 0; i < interval_sec * 2 && !check_stop_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (check_stop_.load()) break;

    // 获取当前模式
    if (!mode_provider_) continue;
    const auto mode = mode_provider_();

    // IDLE 模式下不需要巡检 (服务可能本来就不运行)
    if (mode == robot::v1::ROBOT_MODE_IDLE ||
        mode == robot::v1::ROBOT_MODE_ESTOP) {
      continue;
    }

    // 获取当前模式需要的服务集
    auto required = GetRequiredServices(mode);
    if (required.empty()) continue;

    // 检查每个 required 服务是否 active
    for (const auto &svc : required) {
      std::string state = QueryServiceField(svc + ".service", "ActiveState");
      if (state != "active") {
        RCLCPP_ERROR(node_->get_logger(),
                     "ServiceOrchestrator: service %s CRASHED "
                     "(state=%s, expected=active for mode %d)",
                     svc.c_str(), state.c_str(), static_cast<int>(mode));

        // 发事件通知
        if (event_buffer_) {
          event_buffer_->AddEvent(
              robot::v1::EVENT_TYPE_SYSTEM_BOOT,
              robot::v1::EVENT_SEVERITY_ERROR,
              "Service crashed: " + svc,
              "Service " + svc + ".service state=" + state +
                  " (expected active for mode " +
                  std::to_string(static_cast<int>(mode)) + ")");
        }

        // 通知上层 (ModeManager 可选择降级)
        if (crash_callback_) {
          crash_callback_(svc, mode);
        }

        // 只报告第一个崩溃的服务, 避免连锁触发
        break;
      }
    }
  }
}

std::set<std::string> ServiceOrchestrator::GetRequiredServices(
    robot::v1::RobotMode mode) const {
  switch (mode) {
  case robot::v1::ROBOT_MODE_IDLE:
    // IDLE: 保持 lidar+slam 用于实时监控 (点云/遥测)
    return {"nav-lidar", "nav-slam"};

  case robot::v1::ROBOT_MODE_MAPPING:
    // 建图: 只需 lidar + SLAM
    return {"nav-lidar", "nav-slam"};

  case robot::v1::ROBOT_MODE_MANUAL:
    // 手动: lidar + SLAM (手柄直接控制)
    return {"nav-lidar", "nav-slam"};

  case robot::v1::ROBOT_MODE_TELEOP:
    // 遥操作: 需要地形分析做避障
    return {"nav-lidar", "nav-slam", "nav-autonomy"};

  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    // 全自主: 完整导航栈
    return {"nav-lidar", "nav-slam", "nav-autonomy", "nav-planning"};

  case robot::v1::ROBOT_MODE_ESTOP:
    // 急停: 不改变服务状态, 仅发停车指令
    return {};

  default:
    return {};
  }
}

void ServiceOrchestrator::OnModeChange(robot::v1::RobotMode /*old_mode*/,
                                       robot::v1::RobotMode new_mode) {
  if (new_mode == robot::v1::ROBOT_MODE_ESTOP) {
    // 急停不动服务
    RCLCPP_WARN(node_->get_logger(),
                "ServiceOrchestrator: ESTOP — services untouched");
    return;
  }

  auto required = GetRequiredServices(new_mode);
  if (required.empty() && new_mode != robot::v1::ROBOT_MODE_IDLE) {
    return;
  }

  // 异步执行, 不阻塞 gRPC 线程
  std::thread([this, required = std::move(required)]() {
    OrchestrateAsync(required);
  }).detach();
}

bool ServiceOrchestrator::StartServiceWithRetry(const std::string &svc,
                                                int max_retries) {
  const std::string unit = svc + ".service";

  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    std::string state = QueryServiceField(unit, "ActiveState");
    if (state == "active") return true;

    // 清除 failed 残留
    if (state == "failed") {
      ManageService(unit, "reset-failed");
      RCLCPP_WARN(node_->get_logger(),
                  "ServiceOrchestrator: reset-failed %s", unit.c_str());
    }

    if (attempt > 0) {
      RCLCPP_WARN(node_->get_logger(),
                  "ServiceOrchestrator: retry %d/%d for %s",
                  attempt, max_retries, unit.c_str());
    }

    RCLCPP_INFO(node_->get_logger(),
                "ServiceOrchestrator: starting %s", unit.c_str());
    ManageService(unit, "start");

    // 等待服务启动 (最多 5 秒)
    for (int w = 0; w < 10; ++w) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      state = QueryServiceField(unit, "ActiveState");
      if (state == "active") {
        RCLCPP_INFO(node_->get_logger(),
                    "ServiceOrchestrator: %s → active", unit.c_str());
        return true;
      }
    }
  }

  // 所有重试失败 — 发事件通知
  RCLCPP_ERROR(node_->get_logger(),
               "ServiceOrchestrator: FAILED to start %s after %d retries",
               unit.c_str(), max_retries);
  if (event_buffer_) {
    event_buffer_->AddEvent(
        robot::v1::EVENT_TYPE_SYSTEM_BOOT,
        robot::v1::EVENT_SEVERITY_ERROR,
        "Service start failed: " + svc,
        "Failed to start " + unit + " after " +
            std::to_string(max_retries + 1) + " attempts");
  }
  return false;
}

void ServiceOrchestrator::OrchestrateAsync(std::set<std::string> required) {
  std::lock_guard<std::mutex> lock(orchestrate_mutex_);

  // 1. 按依赖顺序启动需要的服务
  for (size_t i = 0; i < kNumManagedServices; ++i) {
    const std::string svc = kManagedServices[i];
    if (required.count(svc)) {
      StartServiceWithRetry(svc, 2);
    }
  }

  // 2. 按依赖反序停止不需要的服务
  for (int i = static_cast<int>(kNumManagedServices) - 1; i >= 0; --i) {
    const std::string svc = kManagedServices[i];
    if (!required.count(svc)) {
      std::string state = QueryServiceField(svc + ".service", "ActiveState");
      if (state == "active") {
        RCLCPP_INFO(node_->get_logger(),
                    "ServiceOrchestrator: stopping %s.service", svc.c_str());
        ManageService(svc + ".service", "stop");
      }
    }
  }
}

void ServiceOrchestrator::EnsureBaseServices() {
  std::thread([this]() {
    OrchestrateAsync({"nav-lidar", "nav-slam"});
  }).detach();
}

void ServiceOrchestrator::StopAllNavServices() {
  std::thread([this]() {
    OrchestrateAsync({});  // 空集 = 全部停止
  }).detach();
}

bool ServiceOrchestrator::ManageService(const std::string &service,
                                        const std::string &action) {
  // 使用 sudo: sudoers 规则允许 sunrise 用户无密码管理 nav-* 服务
  std::string cmd =
      "sudo /bin/systemctl " + action + " " + service + " 2>&1";
  int ret = std::system(cmd.c_str());
  return ret == 0;
}

std::string ServiceOrchestrator::QueryServiceField(const std::string &service,
                                                   const std::string &field) {
  return ExecCommand("systemctl show -p " + field + " --value " + service +
                     " 2>/dev/null");
}

std::vector<ServiceOrchestrator::ServiceStatus>
ServiceOrchestrator::GetAllServiceStatuses() const {
  std::vector<ServiceStatus> statuses;
  for (size_t i = 0; i < kNumManagedServices; ++i) {
    ServiceStatus s;
    s.name = kManagedServices[i];
    s.state = QueryServiceField(s.name + ".service", "ActiveState");
    s.sub_state = QueryServiceField(s.name + ".service", "SubState");
    statuses.push_back(std::move(s));
  }
  return statuses;
}

}  // namespace core
}  // namespace remote_monitoring
