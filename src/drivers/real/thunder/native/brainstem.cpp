#include "brainstem.hpp"

#include "brainstem.grpc.pb.h"

#include <google/protobuf/empty.pb.h>
#include <grpcpp/grpcpp.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace lingtu::driver {
namespace {

using ApiStatus = brainstem::api::v1::ControlStatus;
using RejectReason = brainstem::api::v1::CommandRejectReason;
using namespace std::chrono_literals;

constexpr auto kMinimumCheckedWalkInterval = 20ms;

bool isZero(const Walk& walk) {
  return walk.x == 0.0 && walk.y == 0.0 && walk.z == 0.0;
}

std::string readPem(const std::string& path, const char* label) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error(
        std::string("cannot read Brainstem ") + label + " PEM: " + path);
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  if (contents.str().empty()) {
    throw std::runtime_error(
        std::string("Brainstem ") + label + " PEM is empty: " + path);
  }
  return contents.str();
}

std::shared_ptr<grpc::Channel> createChannel(
    const std::string& target,
    const BrainstemTlsConfig& tls) {
  if (!tls.enabled()) {
    return grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
  }
  grpc::SslCredentialsOptions options;
  options.pem_root_certs = readPem(tls.ca_file, "CA");
  options.pem_cert_chain = readPem(tls.certificate_file, "client certificate");
  options.pem_private_key = readPem(tls.private_key_file, "client private key");
  const auto credentials = grpc::SslCredentials(options);
  if (!credentials) {
    throw std::runtime_error("failed to create Brainstem mTLS credentials");
  }
  if (tls.server_name.empty()) {
    return grpc::CreateChannel(target, credentials);
  }
  grpc::ChannelArguments arguments;
  arguments.SetSslTargetNameOverride(tls.server_name);
  return grpc::CreateCustomChannel(target, credentials, arguments);
}

std::string grpcError(const grpc::Status& status) {
  return std::to_string(static_cast<int>(status.error_code())) + ":" +
      status.error_message();
}

RpcResult rpcFailure(const grpc::Status& status, const char* rpc_name) {
  ControlState state;
  if (status.error_code() == grpc::StatusCode::UNIMPLEMENTED) {
    state.connected = true;
    state.reason = "protocol_incompatible";
    return {
        false,
        true,
        false,
        state,
        std::string("protocol_incompatible: required Brainstem RPC ") +
            rpc_name + " is not implemented",
    };
  }
  return {false, false, false, state, grpcError(status)};
}

std::string fsmName(const ApiStatus& status) {
  if (!status.has_fsm()) {
    return "unknown";
  }
  using Kind = brainstem::api::v1::CmsStateKind;
  switch (status.fsm().kind()) {
    case Kind::CMS_STATE_KIND_GROUNDED:
      return "grounded";
    case Kind::CMS_STATE_KIND_STANDING:
      return "standing";
    case Kind::CMS_STATE_KIND_WALKING:
      return "walking";
    case Kind::CMS_STATE_KIND_TRANSITIONING:
      return "transitioning";
    case Kind::CMS_STATE_KIND_ZERO:
    default:
      return "zero";
  }
}

std::string ownerName(brainstem::api::v1::ControlOwner owner) {
  using Owner = brainstem::api::v1::ControlOwner;
  switch (owner) {
    case Owner::CONTROL_OWNER_GRPC:
      return "grpc";
    case Owner::CONTROL_OWNER_YUNZHUO:
      return "yunzhuo";
    case Owner::CONTROL_OWNER_NONE:
    default:
      return "none";
  }
}

std::string reasonName(RejectReason reason) {
  std::string name = brainstem::api::v1::CommandRejectReason_Name(reason);
  constexpr const char* prefix = "COMMAND_REJECT_REASON_";
  if (name.rfind(prefix, 0) == 0) {
    name.erase(0, std::char_traits<char>::length(prefix));
  }
  for (char& ch : name) {
    if (ch >= 'A' && ch <= 'Z') {
      ch = static_cast<char>(ch - 'A' + 'a');
    }
  }
  return name.empty() ? "unknown" : name;
}

ControlState decodeStatus(
    const ApiStatus& status,
    RejectReason reason,
    bool connected) {
  ControlState out;
  out.connected = connected;
  out.motors_enabled = status.motor_output_enabled();
  out.critical_fault = status.critical_motor_fault();
  out.lease_valid = status.grpc_lease_active();
  out.lease_remaining_ms = status.lease_remaining_ms();
  out.accepted_sequence = status.last_accepted_sequence();
  out.fsm = fsmName(status);
  out.owner = ownerName(status.owner());
  out.owner_id = status.owner_id();
  out.reason = reasonName(reason);
  out.ready = connected && status.ready_for_walk() && out.lease_valid &&
      out.owner == "grpc" && out.owner_id == "lingtu-driver" &&
      out.motors_enabled && !out.critical_fault &&
      (out.fsm == "standing" || out.fsm == "walking");
  if (!out.ready && out.reason == "none") {
    if (out.critical_fault) {
      out.reason = "motor_fault";
    } else if (!out.motors_enabled) {
      out.reason = "motors_disabled";
    } else if (!out.lease_valid) {
      out.reason = "lease_missing";
    } else if (out.owner != "grpc" || out.owner_id != "lingtu-driver") {
      out.reason = "preempted";
    } else if (out.fsm != "standing" && out.fsm != "walking") {
      out.reason = "fsm_not_ready";
    } else {
      out.reason = "not_ready";
    }
  }
  return out;
}

}  // namespace

struct Brainstem::Impl {
  Impl(
      std::string host,
      unsigned port,
      std::chrono::milliseconds deadline,
      const BrainstemTlsConfig& tls)
      : target(std::move(host) + ":" + std::to_string(port)),
        timeout(deadline),
        channel(createChannel(target, tls)),
        stub(brainstem::api::v1::RobotControl::NewStub(channel)) {}

  std::string target;
  std::chrono::milliseconds timeout;
  std::shared_ptr<grpc::Channel> channel;
  std::unique_ptr<brainstem::api::v1::RobotControl::Stub> stub;
  std::string lease_token;
  std::uint64_t sequence{0};
  bool initial_zero_acknowledged{false};
  bool has_sent_walk{false};
  std::chrono::steady_clock::time_point last_walk_at{};

  void clearLease() {
    lease_token.clear();
    initial_zero_acknowledged = false;
    has_sent_walk = false;
  }
};

Brainstem::Brainstem(
    std::string host,
    unsigned port,
    std::chrono::milliseconds timeout,
    BrainstemTlsConfig tls)
    : impl_(std::make_unique<Impl>(std::move(host), port, timeout, tls)) {}

Brainstem::~Brainstem() {
  release();
}

RpcResult Brainstem::refreshControl() {
  brainstem::api::v1::ControlLease lease;
  grpc::Status status;
  const bool acquiring = impl_->lease_token.empty();
  if (acquiring) {
    brainstem::api::v1::AcquireControlRequest request;
    request.set_client_id("lingtu-driver");
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + impl_->timeout);
    status = impl_->stub->AcquireControl(&context, request, &lease);
  } else {
    brainstem::api::v1::ControlLeaseRequest request;
    request.set_token(impl_->lease_token);
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + impl_->timeout);
    status = impl_->stub->RenewControlLease(&context, request, &lease);
  }

  if (!status.ok()) {
    impl_->clearLease();
    return rpcFailure(
        status,
        acquiring ? "AcquireControl" : "RenewControlLease");
  }

  if (lease.accepted()) {
    if (acquiring) {
      impl_->initial_zero_acknowledged = false;
      impl_->has_sent_walk = false;
    }
    impl_->lease_token = lease.token();
  } else {
    impl_->clearLease();
  }
  ControlState state =
      decodeStatus(lease.status(), lease.reason(), true);
  state.initial_zero_acknowledged = impl_->initial_zero_acknowledged;
  if (lease.accepted() && state.ready &&
      !impl_->initial_zero_acknowledged) {
    RpcResult zero_result = send(Walk{});
    ++zero_result.rpc_calls;
    return zero_result;
  }
  state.ready = state.ready && impl_->initial_zero_acknowledged;
  if (lease.accepted() && !state.ready && state.reason == "none") {
    state.reason = "initial_zero_required";
  }
  return {
      lease.accepted() && state.ready,
      true,
      lease.accepted(),
      state,
      lease.accepted() ? std::string{} : state.reason,
  };
}

RpcResult Brainstem::send(const Walk& walk) {
  if (impl_->lease_token.empty()) {
    ControlState state;
    state.reason = "lease_missing";
    return {false, true, false, state, state.reason, 0};
  }
  if (!impl_->initial_zero_acknowledged && !isZero(walk)) {
    ControlState state;
    state.connected = true;
    state.lease_valid = true;
    state.reason = "initial_zero_required";
    return {false, true, false, state, state.reason, 0};
  }

  if (impl_->has_sent_walk) {
    const auto earliest = impl_->last_walk_at + kMinimumCheckedWalkInterval;
    if (std::chrono::steady_clock::now() < earliest) {
      std::this_thread::sleep_until(earliest);
    }
  }
  impl_->last_walk_at = std::chrono::steady_clock::now();
  impl_->has_sent_walk = true;

  brainstem::api::v1::WalkRequest request;
  request.set_token(impl_->lease_token);
  request.set_sequence(++impl_->sequence);
  auto* direction = request.mutable_direction();
  direction->set_x(walk.x);
  direction->set_y(walk.y);
  direction->set_z(walk.z);
  brainstem::api::v1::CommandAck response;
  grpc::ClientContext context;
  context.set_deadline(std::chrono::system_clock::now() + impl_->timeout);
  const grpc::Status status =
      impl_->stub->WalkChecked(&context, request, &response);
  if (!status.ok()) {
    impl_->clearLease();
    return rpcFailure(status, "WalkChecked");
  }

  ControlState state =
      decodeStatus(response.status(), response.reason(), true);
  const bool ack_matches =
      response.sequence() == request.sequence() &&
      (!response.accepted() ||
       response.status().last_accepted_sequence() == request.sequence());
  if (!ack_matches) {
    impl_->clearLease();
    state.ready = false;
    state.lease_valid = false;
    state.initial_zero_acknowledged = false;
    state.reason = "ack_sequence_mismatch";
    return {false, true, false, state, state.reason};
  }
  if (response.accepted() && !impl_->initial_zero_acknowledged &&
      isZero(walk)) {
    impl_->initial_zero_acknowledged = true;
  }
  state.initial_zero_acknowledged = impl_->initial_zero_acknowledged;
  state.ready = state.ready && impl_->initial_zero_acknowledged;
  if (!response.accepted()) {
    impl_->clearLease();
    state.initial_zero_acknowledged = false;
  }
  return {
      response.accepted() && state.ready,
      true,
      response.accepted(),
      state,
      response.accepted() ? std::string{} : state.reason,
  };
}

void Brainstem::release() noexcept {
  if (!impl_ || impl_->lease_token.empty()) {
    return;
  }
  try {
    brainstem::api::v1::ControlLeaseRequest request;
    request.set_token(impl_->lease_token);
    brainstem::api::v1::ControlStatus response;
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now() + impl_->timeout);
    (void)impl_->stub->ReleaseControl(&context, request, &response);
  } catch (...) {
  }
  impl_->clearLease();
}

const std::string& Brainstem::target() const noexcept {
  return impl_->target;
}

}  // namespace lingtu::driver
