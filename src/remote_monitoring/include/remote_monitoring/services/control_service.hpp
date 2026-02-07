#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "control.grpc.pb.h"
#include "grpcpp/grpcpp.h"

namespace remote_monitoring {
namespace core {
class LeaseManager;
class SafetyGate;
class EventBuffer;
class IdempotencyCache;
class ModeManager;
class TaskManager;
} // namespace core

namespace services {

class ControlServiceImpl final : public robot::v1::ControlService::Service {
public:
  ControlServiceImpl(std::shared_ptr<core::LeaseManager> lease_mgr,
                     std::shared_ptr<core::SafetyGate> safety_gate,
                     std::shared_ptr<core::ModeManager> mode_manager,
                     std::shared_ptr<core::EventBuffer> event_buffer,
                     std::shared_ptr<core::IdempotencyCache> idempotency_cache,
                     std::shared_ptr<core::TaskManager> task_manager = nullptr);

  grpc::Status AcquireLease(grpc::ServerContext *context,
                            const robot::v1::AcquireLeaseRequest *request,
                            robot::v1::AcquireLeaseResponse *response) override;

  grpc::Status RenewLease(grpc::ServerContext *context,
                          const robot::v1::RenewLeaseRequest *request,
                          robot::v1::RenewLeaseResponse *response) override;

  grpc::Status ReleaseLease(grpc::ServerContext *context,
                            const robot::v1::ReleaseLeaseRequest *request,
                            robot::v1::ReleaseLeaseResponse *response) override;

  grpc::Status SetMode(grpc::ServerContext *context,
                       const robot::v1::SetModeRequest *request,
                       robot::v1::SetModeResponse *response) override;

  grpc::Status
  EmergencyStop(grpc::ServerContext *context,
                const robot::v1::EmergencyStopRequest *request,
                robot::v1::EmergencyStopResponse *response) override;

  grpc::Status StreamTeleop(
      grpc::ServerContext *context,
      grpc::ServerReaderWriter<robot::v1::TeleopFeedback,
                               robot::v1::TeleopCommand> *stream) override;

  grpc::Status StartTask(grpc::ServerContext *context,
                         const robot::v1::StartTaskRequest *request,
                         robot::v1::StartTaskResponse *response) override;

  grpc::Status CancelTask(grpc::ServerContext *context,
                          const robot::v1::CancelTaskRequest *request,
                          robot::v1::CancelTaskResponse *response) override;

  // 获取当前模式（委托给 ModeManager）
  robot::v1::RobotMode GetCurrentMode() const;

private:
  // 从 gRPC context 提取客户端标识（peer address）
  static std::string ExtractPeerId(grpc::ServerContext *context);

  std::shared_ptr<core::LeaseManager> lease_mgr_;
  std::shared_ptr<core::SafetyGate> safety_gate_;
  std::shared_ptr<core::ModeManager> mode_manager_;
  std::shared_ptr<core::EventBuffer> event_buffer_;
  std::shared_ptr<core::IdempotencyCache> idempotency_cache_;
  std::shared_ptr<core::TaskManager> task_manager_;
};

} // namespace services
} // namespace remote_monitoring
