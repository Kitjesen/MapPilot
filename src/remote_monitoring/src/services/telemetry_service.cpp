#include "remote_monitoring/services/telemetry_service.hpp"
#include "remote_monitoring/status_aggregator.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <thread>
#include <chrono>

namespace remote_monitoring {
namespace services {

namespace {

bool ShouldEmitEvent(const robot::v1::EventStreamRequest *request,
                     const robot::v1::Event &event) {
  if (!request->filter_types().empty()) {
    bool match = false;
    for (const auto &type : request->filter_types()) {
      if (event.type() == type) {
        match = true;
        break;
      }
    }
    if (!match) {
      return false;
    }
  }

  return event.severity() >= request->min_severity();
}

} // namespace

TelemetryServiceImpl::TelemetryServiceImpl(
  std::shared_ptr<StatusAggregator> aggregator,
  std::shared_ptr<core::EventBuffer> event_buffer)
  : aggregator_(std::move(aggregator)),
    event_buffer_(std::move(event_buffer)) {}

grpc::Status TelemetryServiceImpl::StreamFastState(
  grpc::ServerContext *context,
  const robot::v1::FastStateRequest *request,
  grpc::ServerWriter<robot::v1::FastState> *writer) {
  
  double hz = request->desired_hz() > 0 ? request->desired_hz() : aggregator_->fast_state_hz();
  if (hz > 60.0) hz = 60.0;   // 限制最大频率
  if (hz < 0.5)  hz = 0.5;    // 限制最小频率，防止单客户端长 sleep 阻塞 gRPC 线程
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / hz));
  
  while (!context->IsCancelled()) {
    auto state = aggregator_->GetFastState();
    if (!state || !writer->Write(*state)) {
      break;
    }
    // 可中断 sleep: 每 200ms 检查一次取消，避免客户端断开后长时间阻塞
    auto remaining = period;
    constexpr auto kCheckInterval = std::chrono::milliseconds(200);
    while (remaining > std::chrono::milliseconds(0)) {
      const auto chunk = std::min(remaining, kCheckInterval);
      std::this_thread::sleep_for(chunk);
      if (context->IsCancelled()) break;
      remaining -= chunk;
    }
  }
  
  return grpc::Status::OK;
}

grpc::Status TelemetryServiceImpl::StreamSlowState(
  grpc::ServerContext *context,
  const robot::v1::SlowStateRequest *,
  grpc::ServerWriter<robot::v1::SlowState> *writer) {
  
  double hz = aggregator_->slow_state_hz();
  if (hz < 0.5) hz = 0.5;
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / hz));
  
  while (!context->IsCancelled()) {
    auto state = aggregator_->GetSlowState();
    if (!state || !writer->Write(*state)) {
      break;
    }
    // 可中断 sleep
    auto remaining = period;
    constexpr auto kCheckInterval = std::chrono::milliseconds(500);
    while (remaining > std::chrono::milliseconds(0)) {
      const auto chunk = std::min(remaining, kCheckInterval);
      std::this_thread::sleep_for(chunk);
      if (context->IsCancelled()) break;
      remaining -= chunk;
    }
  }
  
  return grpc::Status::OK;
}

grpc::Status TelemetryServiceImpl::StreamEvents(
  grpc::ServerContext *context,
  const robot::v1::EventStreamRequest *request,
  grpc::ServerWriter<robot::v1::Event> *writer) {
  
  // 先回放历史事件
  const auto history = event_buffer_->GetEventsSince(request->last_event_id());
  std::string cursor = request->last_event_id();
  for (const auto &event : history) {
    cursor = event.event_id();
    if (!ShouldEmitEvent(request, event)) {
      continue;
    }

    if (!writer->Write(event)) {
      return grpc::Status::OK;
    }
  }
  
  // 再进入实时推送：阻塞等待新事件，避免空转 sleep。
  while (!context->IsCancelled()) {
    robot::v1::Event event;
    if (!event_buffer_->WaitForEventAfter(cursor, &event,
                                          std::chrono::milliseconds(1000))) {
      continue;
    }

    cursor = event.event_id();
    if (!ShouldEmitEvent(request, event)) {
      continue;
    }

    if (!writer->Write(event)) {
      return grpc::Status::OK;
    }
  }
  
  return grpc::Status::OK;
}

grpc::Status TelemetryServiceImpl::AckEvent(
  grpc::ServerContext *,
  const robot::v1::AckEventRequest *request,
  robot::v1::AckEventResponse *response) {
  
  event_buffer_->AckEvent(request->event_id());
  
  response->mutable_base()->set_request_id(request->base().request_id());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

}  // namespace services
}  // namespace remote_monitoring
