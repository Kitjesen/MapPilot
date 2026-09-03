#include "doso.hpp"

#include <algorithm>
#include <brainstem/client.hpp>
#include <cctype>
#include <stdexcept>
#include <string>
#include <utility>

namespace lingtu::driver {
namespace {

std::string endpoint(std::string host, std::uint16_t port) {
  if (host.find(':') != std::string::npos && host.front() != '[') {
    host = '[' + host + ']';
  }
  return std::move(host) + ':' + std::to_string(port);
}

bool isLoopback(std::string host) {
  std::transform(host.begin(), host.end(), host.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return host == "localhost" || host == "localhost.localdomain" || host == "127.0.0.1" ||
         host == "::1" || host == "[::1]";
}

brainstem::TlsConfig sdkTls(const BrainstemTlsConfig &source) {
  return {
      source.ca_file,
      source.certificate_file,
      source.private_key_file,
      source.server_name,
  };
}

brainstem::Config sdkConfig(const Config &source, const std::string &target) {
  brainstem::Config config;
  config.target = target;
  config.timeout = source.rpc_timeout;
  config.client_id = kBrainstemMotionPrincipal;
  config.allow_insecure = !source.brainstem_tls.enabled() && isLoopback(source.host);
  config.tls = sdkTls(source.brainstem_tls);
  return config;
}

const char *fsmName(brainstem::BodyStateKind kind) noexcept {
  switch (kind) {
    case brainstem::BodyStateKind::Grounded:
      return "grounded";
    case brainstem::BodyStateKind::Standing:
      return "standing";
    case brainstem::BodyStateKind::Walking:
      return "walking";
    case brainstem::BodyStateKind::Transitioning:
      return "transitioning";
    case brainstem::BodyStateKind::Zero:
      return "zero";
    case brainstem::BodyStateKind::Unknown:
      return "unknown";
  }
  return "unknown";
}

ControlState controlState(const brainstem::ControlState &source) {
  ControlState state;
  state.connected = source.connected;
  state.ready = source.motion_ready;
  state.motors_enabled = source.motor_output_enabled;
  state.critical_fault = source.critical_motor_fault;
  state.control_assured = source.lease_valid;
  state.lease_valid = source.lease_valid;
  state.lease_remaining_ms = source.lease_remaining_ms;
  state.accepted_sequence = source.last_accepted_sequence;
  state.initial_zero_acknowledged = source.initial_zero_acknowledged;
  state.fsm = fsmName(source.body.kind);
  state.reason = source.reason;
  return state;
}

Posture posture(brainstem::BodyStateKind kind) noexcept {
  switch (kind) {
    case brainstem::BodyStateKind::Standing:
    case brainstem::BodyStateKind::Walking:
      return Posture::Standing;
    case brainstem::BodyStateKind::Grounded:
      return Posture::Sitting;
    case brainstem::BodyStateKind::Transitioning:
      return Posture::Transitioning;
    case brainstem::BodyStateKind::Zero:
    case brainstem::BodyStateKind::Unknown:
      return Posture::Unknown;
  }
  return Posture::Unknown;
}

Result result(const brainstem::Result &source, unsigned calls = 1) {
  Result mapped;
  mapped.ok = source.ok;
  mapped.transport_ok = source.transport_ok;
  mapped.accepted = source.accepted;
  mapped.state = controlState(source.state);
  mapped.error = source.error;
  mapped.calls = calls;
  mapped.stop_confirmed = source.stop_confirmed;
  return mapped;
}

}  // namespace

struct Doso::Impl {
  explicit Impl(const Config &config)
      : target(endpoint(config.host, config.port)), client(sdkConfig(config, target)) {}

  Result remember(const brainstem::Result &source) {
    last_sdk_state = source.state;
    last_control = controlState(source.state);
    last_body.fresh =
        source.state.connected && source.state.body.kind != brainstem::BodyStateKind::Unknown;
    last_body.posture = last_body.fresh ? posture(source.state.body.kind) : Posture::Unknown;
    last_body.velocity = {};
    last_body.velocity_available = false;
    return result(source);
  }

  std::string target;
  brainstem::Client client;
  brainstem::ControlState last_sdk_state;
  ControlState last_control;
  BodyState last_body;
};

Doso::Doso(const Config &config) : impl_(std::make_unique<Impl>(config)) {}
Doso::~Doso() = default;

Result Doso::refresh() {
  return impl_->remember(impl_->client.refresh());
}

Result Doso::move(const Velocity &velocity) {
  return impl_->remember(impl_->client.move({velocity.vx_mps, velocity.vy_mps, velocity.yaw_rps}));
}

Result Doso::stop() noexcept {
  return impl_->remember(impl_->client.stop());
}

Result Doso::act(BodyAction action) {
  if (action == BodyAction::Recover || action == BodyAction::Damp) {
    ControlState state = impl_->last_control;
    state.ready = false;
    state.reason = std::string("unsupported_body_action:") + bodyActionName(action);
    return {false, true, false, state, state.reason, 0};
  }

  const brainstem::Result sdk_result =
      action == BodyAction::Stand ? impl_->client.standUp() : impl_->client.sitDown();
  return impl_->remember(sdk_result);
}

Capabilities Doso::capabilities() const noexcept {
  return {true, true, false, false};
}

BodyState Doso::state() const {
  return impl_->last_body;
}

HealthState Doso::health() const {
  HealthState state;
  state.fresh = impl_->last_control.connected;
  state.healthy =
      state.fresh && impl_->last_control.motors_enabled && !impl_->last_control.critical_fault;
  state.reason = !state.fresh
                     ? "not_connected"
                     : (impl_->last_control.critical_fault
                            ? "critical_motor_fault"
                            : (impl_->last_control.motors_enabled ? "healthy" : "motors_disabled"));
  return state;
}

AdapterDiagnostics Doso::diagnostics() const {
  return {
      "doso",
      "brainstem_grpc",
      impl_->target,
      impl_->last_sdk_state.control_owner,
      impl_->last_sdk_state.control_owner_id,
  };
}

std::unique_ptr<Body> makeBody(const Config &config) {
  if (config.robot != "doso") {
    throw std::runtime_error("Doso driver binary cannot run robot " + config.robot);
  }
  return std::make_unique<Doso>(config);
}

}  // namespace lingtu::driver
