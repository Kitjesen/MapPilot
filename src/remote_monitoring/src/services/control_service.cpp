#include "remote_monitoring/services/control_service.hpp"
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/idempotency_cache.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/mode_manager.hpp"
#include "remote_monitoring/core/safety_gate.hpp"
#include "remote_monitoring/core/task_manager.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <thread>

namespace remote_monitoring {
namespace services {

namespace {

void FillControlLatency(const google::protobuf::Timestamp &client_ts,
                        google::protobuf::Duration *latency) {
  if (latency == nullptr) {
    return;
  }

  if (client_ts.seconds() == 0 && client_ts.nanos() == 0) {
    latency->set_seconds(0);
    latency->set_nanos(0);
    return;
  }

  const int64_t client_ns = client_ts.seconds() * 1000000000LL + client_ts.nanos();
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  const int64_t latency_ns = std::max<int64_t>(0, now_ns - client_ns);

  latency->set_seconds(latency_ns / 1000000000LL);
  latency->set_nanos(static_cast<int32_t>(latency_ns % 1000000000LL));
}

} // namespace

ControlServiceImpl::ControlServiceImpl(
    std::shared_ptr<core::LeaseManager> lease_mgr,
    std::shared_ptr<core::SafetyGate> safety_gate,
    std::shared_ptr<core::ModeManager> mode_manager,
    std::shared_ptr<core::EventBuffer> event_buffer,
    std::shared_ptr<core::IdempotencyCache> idempotency_cache,
    std::shared_ptr<core::TaskManager> task_manager)
    : lease_mgr_(std::move(lease_mgr)), safety_gate_(std::move(safety_gate)),
      mode_manager_(std::move(mode_manager)),
      event_buffer_(std::move(event_buffer)),
      idempotency_cache_(std::move(idempotency_cache)),
      task_manager_(std::move(task_manager)) {}

robot::v1::RobotMode ControlServiceImpl::GetCurrentMode() const {
  if (mode_manager_) {
    return mode_manager_->GetCurrentMode();
  }
  return robot::v1::ROBOT_MODE_IDLE;
}

std::string ControlServiceImpl::ExtractPeerId(grpc::ServerContext *context) {
  if (context == nullptr) {
    return "unknown";
  }
  // gRPC peer() 返回 "ipv4:x.x.x.x:port" 或 "ipv6:[::]:port"
  std::string peer = context->peer();
  if (peer.empty()) {
    return "unknown";
  }
  return peer;
}

grpc::Status
ControlServiceImpl::AcquireLease(grpc::ServerContext *context,
                                 const robot::v1::AcquireLeaseRequest *request,
                                 robot::v1::AcquireLeaseResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());

  const std::string holder_id = ExtractPeerId(context);
  robot::v1::OperatorLease lease;
  if (lease_mgr_->AcquireLease(holder_id, &lease)) {
    *response->mutable_lease() = lease;
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } else {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_LEASE_CONFLICT);
    response->mutable_base()->set_error_message(
        "Lease already held by another client");
  }

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

grpc::Status
ControlServiceImpl::RenewLease(grpc::ServerContext *,
                               const robot::v1::RenewLeaseRequest *request,
                               robot::v1::RenewLeaseResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());

  robot::v1::OperatorLease lease;
  if (lease_mgr_->RenewLease(request->lease_token(), &lease)) {
    *response->mutable_lease() = lease;
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } else {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_LEASE_EXPIRED);
    response->mutable_base()->set_error_message("Invalid or expired lease");
  }

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

grpc::Status
ControlServiceImpl::ReleaseLease(grpc::ServerContext *,
                                 const robot::v1::ReleaseLeaseRequest *request,
                                 robot::v1::ReleaseLeaseResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());
  lease_mgr_->ReleaseLease(request->lease_token());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

grpc::Status
ControlServiceImpl::SetMode(grpc::ServerContext *,
                            const robot::v1::SetModeRequest *request,
                            robot::v1::SetModeResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());

  // ESTOP 走独立路径
  if (request->mode() == robot::v1::ROBOT_MODE_ESTOP) {
    if (mode_manager_) {
      mode_manager_->EmergencyStop("operator SetMode(ESTOP)");
    }
    response->set_current_mode(robot::v1::ROBOT_MODE_ESTOP);
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } else {
    // 通过 ModeManager 守卫检查
    if (mode_manager_) {
      auto result = mode_manager_->SwitchMode(request->mode());
      if (result.success) {
        response->set_current_mode(request->mode());
        response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
      } else {
        response->set_current_mode(mode_manager_->GetCurrentMode());
        response->mutable_base()->set_error_code(
            robot::v1::ERROR_CODE_MODE_CONFLICT);
        response->mutable_base()->set_error_message(result.reason);
      }
    } else {
      response->mutable_base()->set_error_code(
          robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
      response->mutable_base()->set_error_message("ModeManager not available");
    }
  }

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::EmergencyStop(
    grpc::ServerContext *, const robot::v1::EmergencyStopRequest *request,
    robot::v1::EmergencyStopResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());

  // 委托给 ModeManager (lock-free 独立路径)
  if (mode_manager_) {
    mode_manager_->EmergencyStop(
        request->hard_stop() ? "operator hard_stop" : "operator estop");
  } else {
    // fallback: 直接操作 SafetyGate
    safety_gate_->SetEmergencyStop(true);
  }

  response->set_stopped(true);
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::StreamTeleop(
    grpc::ServerContext *context,
    grpc::ServerReaderWriter<robot::v1::TeleopFeedback,
                             robot::v1::TeleopCommand> *stream) {

  robot::v1::TeleopCommand cmd;
  while (!context->IsCancelled() && stream->Read(&cmd)) {
    // 验证租约
    if (!lease_mgr_->ValidateLease(cmd.lease_token())) {
      robot::v1::TeleopFeedback feedback;
      feedback.mutable_timestamp()->CopyFrom(cmd.timestamp());
      feedback.set_command_sequence(cmd.sequence());
      feedback.add_limit_reasons("invalid_lease");
      feedback.mutable_safety_status()->set_safety_message("Invalid lease");
      FillControlLatency(cmd.timestamp(), feedback.mutable_control_latency());
      stream->Write(feedback);
      return grpc::Status(grpc::PERMISSION_DENIED, "Lease required");
    }

    // 通过 Safety Gate 处理
    const auto limited_vel = safety_gate_->ProcessTeleopCommand(cmd);
    const auto safety_status = safety_gate_->GetSafetyStatus();
    const auto limit_reasons = safety_gate_->GetLimitReasons();

    robot::v1::TeleopFeedback feedback;
    feedback.mutable_timestamp()->CopyFrom(cmd.timestamp());
    feedback.set_command_sequence(cmd.sequence());
    *feedback.mutable_actual_velocity() = limited_vel;
    *feedback.mutable_safety_status() = safety_status;
    for (const auto &reason : limit_reasons) {
      feedback.add_limit_reasons(reason);
    }
    FillControlLatency(cmd.timestamp(), feedback.mutable_control_latency());

    if (!stream->Write(feedback)) {
      break;
    }
  }

  return grpc::Status::OK;
}

grpc::Status
ControlServiceImpl::StartTask(grpc::ServerContext *,
                              const robot::v1::StartTaskRequest *request,
                              robot::v1::StartTaskResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());

  if (!task_manager_) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    response->mutable_base()->set_error_message("Task manager not available");
    return grpc::Status::OK;
  }

  // 解析 params_json: {"waypoints": [{"x":1,"y":2,"z":0,"label":"A"}, ...],
  //                     "loop": false, "arrival_radius": 1.0}
  core::TaskParams params;
  params.type = request->task_type();

  // 简易 JSON 解析 (航点列表)
  // 格式: "x1,y1,z1;x2,y2,z2;..." 或完整 JSON
  // 为简化实现，使用分号分隔的坐标格式
  const std::string &params_json = request->params_json();
  if (!params_json.empty()) {
    std::istringstream stream(params_json);
    std::string token;
    while (std::getline(stream, token, ';')) {
      core::Waypoint wp;
      std::istringstream point_stream(token);
      std::string val;
      if (std::getline(point_stream, val, ',')) wp.x = std::stod(val);
      if (std::getline(point_stream, val, ',')) wp.y = std::stod(val);
      if (std::getline(point_stream, val, ',')) wp.z = std::stod(val);
      if (std::getline(point_stream, val, ',')) wp.label = val;
      params.waypoints.push_back(wp);
    }
  }

  if (params.waypoints.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_INVALID_REQUEST);
    response->mutable_base()->set_error_message(
        "No waypoints provided. Use params_json format: "
        "'x1,y1,z1;x2,y2,z2;...'");
    return grpc::Status::OK;
  }

  // 默认参数
  params.loop = (request->task_type() == robot::v1::TASK_TYPE_INSPECTION);
  params.arrival_radius = 1.0;

  const std::string task_id = task_manager_->StartTask(params);
  if (task_id.empty()) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_RESOURCE_CONFLICT);
    response->mutable_base()->set_error_message(
        "Cannot start task — another task is active");
    return grpc::Status::OK;
  }

  response->set_task_id(task_id);
  auto *task = response->mutable_task();
  task->set_task_id(task_id);
  task->set_type(request->task_type());
  task->set_status(robot::v1::TASK_STATUS_RUNNING);
  task->set_progress_percent(0.0f);
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

grpc::Status
ControlServiceImpl::CancelTask(grpc::ServerContext *,
                               const robot::v1::CancelTaskRequest *request,
                               robot::v1::CancelTaskResponse *response) {

  // Idempotency Check
  std::string cached_resp;
  if (idempotency_cache_->TryGet(request->base().request_id(), &cached_resp)) {
    if (response->ParseFromString(cached_resp)) {
      return grpc::Status::OK;
    }
  }

  response->mutable_base()->set_request_id(request->base().request_id());

  if (!task_manager_) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_SERVICE_UNAVAILABLE);
    return grpc::Status::OK;
  }

  if (!task_manager_->CancelTask(request->task_id())) {
    response->mutable_base()->set_error_code(
        robot::v1::ERROR_CODE_RESOURCE_NOT_FOUND);
    response->mutable_base()->set_error_message(
        "Task not found or not active: " + request->task_id());
    return grpc::Status::OK;
  }

  auto *task = response->mutable_task();
  *task = task_manager_->GetCurrentTask();
  task->set_status(robot::v1::TASK_STATUS_CANCELLED);
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  // Cache Result
  std::string resp_str;
  if (response->SerializeToString(&resp_str)) {
    idempotency_cache_->Set(request->base().request_id(), resp_str);
  }

  return grpc::Status::OK;
}

} // namespace services
} // namespace remote_monitoring
