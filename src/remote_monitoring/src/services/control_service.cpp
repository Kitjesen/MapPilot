#include "remote_monitoring/services/control_service.hpp"
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/idempotency_cache.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/safety_gate.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <thread>

namespace remote_monitoring {
namespace services {

namespace {

std::string ModeToString(robot::v1::RobotMode mode) {
  switch (mode) {
  case robot::v1::ROBOT_MODE_IDLE:
    return "idle";
  case robot::v1::ROBOT_MODE_MANUAL:
    return "manual";
  case robot::v1::ROBOT_MODE_TELEOP:
    return "teleop";
  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    return "autonomous";
  case robot::v1::ROBOT_MODE_MAPPING:
    return "mapping";
  case robot::v1::ROBOT_MODE_ESTOP:
    return "estop";
  default:
    return "unspecified";
  }
}

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
    std::shared_ptr<core::EventBuffer> event_buffer,
    std::shared_ptr<core::IdempotencyCache> idempotency_cache)
    : lease_mgr_(std::move(lease_mgr)), safety_gate_(std::move(safety_gate)),
      event_buffer_(std::move(event_buffer)),
      idempotency_cache_(std::move(idempotency_cache)) {}

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
  current_mode_.store(request->mode());
  response->set_current_mode(request->mode());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  safety_gate_->SetEmergencyStop(request->mode() == robot::v1::ROBOT_MODE_ESTOP);

  if (event_buffer_ != nullptr) {
    std::ostringstream oss;
    oss << "Mode switched to " << ModeToString(request->mode());
    event_buffer_->AddEvent(robot::v1::EVENT_TYPE_MODE_CHANGE,
                            robot::v1::EVENT_SEVERITY_INFO, "Mode Changed",
                            oss.str());
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
  safety_gate_->SetEmergencyStop(true);
  current_mode_.store(robot::v1::ROBOT_MODE_ESTOP);
  response->set_stopped(true);
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);

  if (event_buffer_ != nullptr) {
    event_buffer_->AddEvent(robot::v1::EVENT_TYPE_SAFETY_ESTOP,
                            robot::v1::EVENT_SEVERITY_CRITICAL,
                            "Emergency Stop Activated",
                            request->hard_stop() ? "Hard emergency stop"
                                                 : "Emergency stop");
  }

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
  response->set_task_id("task_001"); // TODO: 实际任务管理
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
