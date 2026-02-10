// ota_device.cpp — 设备管理 RPC 实现 (GetDeviceInfo / ManageService)
// 从 ota_service.cpp 拆分

#include "ota_service.hpp"
#include "utils.hpp"

#include <thread>

#ifndef OTA_DAEMON_VERSION
#define OTA_DAEMON_VERSION "1.0.0"
#endif

namespace ota {

// =====================================================================
// RPC: GetDeviceInfo
// =====================================================================
grpc::Status OtaServiceImpl::GetDeviceInfo(
    grpc::ServerContext *, const google::protobuf::Empty *,
    robot::v1::DeviceInfoResponse *response) {

  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_hostname(GetHostname());
  response->set_robot_id(config_.robot_id);
  response->set_hw_id(config_.hw_id);

  for (const auto &ip : GetIPAddresses()) {
    response->add_ip_addresses(ip);
  }

  response->set_disk_total_bytes(GetDiskTotalBytes("/opt/robot"));
  response->set_disk_free_bytes(GetDiskFreeBytes("/opt/robot"));
  response->set_battery_percent(GetBatteryPercent());
  response->set_uptime_seconds(GetUptimeSeconds());
  response->set_os_version(GetOSVersion());
  response->set_ota_daemon_version(OTA_DAEMON_VERSION);

  // Service statuses
  for (const auto &svc : config_.managed_services) {
    auto status = ota::GetServiceStatus(svc);
    auto *s = response->add_services();
    s->set_name(svc);
    s->set_state(status.active_state);
    s->set_sub_state(status.sub_state);
    s->set_uptime_seconds(status.uptime_seconds);
    s->set_restart_count(status.restart_count);
  }

  return grpc::Status::OK;
}

// =====================================================================
// RPC: ManageService
// =====================================================================
grpc::Status OtaServiceImpl::ManageService(
    grpc::ServerContext *, const robot::v1::ManageServiceRequest *request,
    robot::v1::ManageServiceResponse *response) {

  const std::string &svc = request->service_name();

  // Security: only allow managed services
  bool allowed = false;
  for (const auto &s : config_.managed_services) {
    if (s == svc) { allowed = true; break; }
  }
  if (!allowed) {
    response->set_success(false);
    response->set_message("Service not in managed list: " + svc);
    return grpc::Status::OK;
  }

  std::string action;
  switch (request->action()) {
    case robot::v1::SERVICE_ACTION_START: action = "start"; break;
    case robot::v1::SERVICE_ACTION_STOP: action = "stop"; break;
    case robot::v1::SERVICE_ACTION_RESTART: action = "restart"; break;
    case robot::v1::SERVICE_ACTION_STATUS: {
      auto st = ota::GetServiceStatus(svc);
      response->set_success(true);
      response->set_message("Status queried");
      auto *s = response->mutable_status();
      s->set_name(svc);
      s->set_state(st.active_state);
      s->set_sub_state(st.sub_state);
      s->set_uptime_seconds(st.uptime_seconds);
      s->set_restart_count(st.restart_count);
      return grpc::Status::OK;
    }
    default:
      response->set_success(false);
      response->set_message("Unknown action");
      return grpc::Status::OK;
  }

  OtaLogInfo("ManageService: %s %s", action.c_str(), svc.c_str());
  bool ok = ota::ManageService(svc, action);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  auto st = ota::GetServiceStatus(svc);

  response->set_success(ok);
  response->set_message(ok ? action + " succeeded" : action + " failed");
  auto *s = response->mutable_status();
  s->set_name(svc);
  s->set_state(st.active_state);
  s->set_sub_state(st.sub_state);
  s->set_uptime_seconds(st.uptime_seconds);
  s->set_restart_count(st.restart_count);
  return grpc::Status::OK;
}

} // namespace ota
