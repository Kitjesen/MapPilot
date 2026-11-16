#include "doso.hpp"

#include <algorithm>
#include <brainstem/client.hpp>
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
#include <brainstem/external.hpp>
#endif
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
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  state.owner = source.control_owner;
  state.owner_id = source.control_owner_id;
#endif
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

Result unsupportedExternalResult() {
  Result result;
  result.calls = 0;
  result.error = "external_control_unsupported";
  result.state.reason = result.error;
  return result;
}

}  // namespace

struct Doso::Impl {
  explicit Impl(const Config &config)
      : target(endpoint(config.host, config.port)), client(sdkConfig(config, target))
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
        , external(sdkConfig(config, target))
#endif
  {}

  Result remember(const brainstem::Result &source) {
    last_sdk_state = source.state;
    last_control = controlState(source.state);
    last_body.fresh =
        source.state.connected && source.state.body.kind != brainstem::BodyStateKind::Unknown;
    last_body.posture = last_body.fresh ? posture(source.state.body.kind) : Posture::Unknown;
    last_body.velocity = {};
    last_body.velocity_available = false;
    last_body.odometry_position_available = false;
    return result(source);
  }

  std::string target;
  brainstem::Client client;
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  brainstem::ExternalControl external;
  std::string external_session;
  Clock::time_point next_external_query{};
#endif
  brainstem::ControlState last_sdk_state;
  ControlState last_control;
  BodyState last_body;

#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  Result rememberExternal(const brainstem::Response<brainstem::ExternalStatus> &response) {
    auto mapped = result(response.result);
    mapped.state = last_control;
    mapped.state.ready = false;
    mapped.state.control_assured = false;
    mapped.state.lease_valid = false;
    mapped.state.connected = response.result.transport_ok;
    if (response.result.ok) {
      mapped.state.motors_enabled = response.value.motor_output_enabled;
      mapped.state.owner = response.value.selected ? "external" : "none";
      mapped.state.owner_id = response.value.selected ? kDriverMotionPrincipal : "";
      mapped.state.reason = response.value.reason;
    } else {
      mapped.state.reason = response.result.error;
    }
    mapped.stop_confirmed = response.result.ok && response.value.motor_disable_confirmed;
    last_control = mapped.state;
    return mapped;
  }
#endif
};

Doso::Doso(const Config &config) : impl_(std::make_unique<Impl>(config)) {}
Doso::~Doso() = default;

Result Doso::refresh() {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  if (externalSelected()) return impl_->rememberExternal(impl_->external.getStatus());
#endif
  return impl_->remember(impl_->client.refresh());
}

Result Doso::inspectControl() {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  const auto inspected = impl_->remember(impl_->client.getControlStatus().result);
  if (inspected.ok && inspected.transport_ok) impl_->last_body.observed_at = Clock::now();
  return inspected;
#else
  return refresh();
#endif
}

Result Doso::move(const Velocity &velocity) {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  if (externalSelected()) {
    auto state = impl_->last_control;
    state.reason = "external_control_selected";
    return {false, true, false, state, state.reason, 0};
  }
#endif
  return impl_->remember(impl_->client.move({velocity.vx_mps, velocity.vy_mps, velocity.yaw_rps}));
}

Result Doso::stop() noexcept {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  if (externalSelected()) {
    try {
      return impl_->rememberExternal(impl_->external.disable());
    } catch (const std::exception &error) {
      return {false, false, false, impl_->last_control, error.what(), 1};
    }
  }
#endif
  return impl_->remember(impl_->client.stop());
}

Result Doso::act(BodyAction action) {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  if (externalSelected()) {
    auto state = impl_->last_control;
    state.reason = "external_control_selected";
    return {false, true, false, state, state.reason, 0};
  }
#endif
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
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  return {true, true, false, false, true, true};
#else
  return {true, true, false, false, true, false};
#endif
}

Result Doso::motorPower(bool enabled) {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  if (externalSelected()) {
    if (!enabled) return impl_->rememberExternal(impl_->external.disable());
    auto state = impl_->last_control;
    state.reason = "external_enable_required";
    return {false, true, false, state, state.reason, 0};
  }
#endif
  return impl_->remember(enabled ? impl_->client.enableMotors() : impl_->client.disableMotors());
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

ProfileState Doso::readProfile() {
  ProfileState state;
  state.requires_grounded = true;
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  state.requires_disabled = true;
  const auto profile = impl_->client.getProfile();
  state.supported = profile.result.state.reason != "protocol_incompatible";
  if (!profile.result.ok) {
    impl_->last_body.fresh = false;
    state.reason = profile.result.error;
    return state;
  }
  const auto control = impl_->client.getControlStatus();
  if (!control.result.ok) {
    impl_->last_body.fresh = false;
    state.supported = control.result.state.reason != "protocol_incompatible";
    state.reason = control.result.error;
    return state;
  }
  impl_->remember(control.result);
  impl_->last_body.observed_at = Clock::now();
  state.fresh = true;
  state.grounded = control.value.body.kind == brainstem::BodyStateKind::Grounded;
  state.motors_enabled = control.value.motor_output_enabled;
  state.control_free = !control.value.lease_valid && control.value.control_owner == "none";
  state.critical_fault = control.value.critical_motor_fault;
  state.current = profile.value.current;
  state.available = profile.value.available;
  state.reason.clear();
  return state;
#else
  const auto body = impl_->client.getBodyState();
  state.supported = false;
  if (!body.result.ok) {
    impl_->last_body.fresh = false;
    state.reason = body.result.error;
    return state;
  }
  impl_->last_body.fresh = true;
  impl_->last_body.posture = posture(body.value.kind);
  state.fresh = true;
  state.grounded = body.value.kind == brainstem::BodyStateKind::Grounded;
  state.reason = "profile_unsupported";
  return state;
#endif
}

Result Doso::switchProfile(const std::string &name) {
#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
  return impl_->remember(impl_->client.switchProfile(name).result);
#else
  static_cast<void>(name);
  Result result;
  result.calls = 0;
  result.error = "profile_unsupported";
  result.state.reason = result.error;
  return result;
#endif
}

#if defined(LINGTU_BRAINSTEM_EXTERNAL_API)
bool Doso::externalSelected() const { return !impl_->external_session.empty(); }

message::ExternalState Doso::externalState() const {
  if ((!externalSelected() || !impl_->external.status().fresh) &&
      Clock::now() >= impl_->next_external_query) {
    (void)impl_->external.getStatus();
    impl_->next_external_query = Clock::now() + std::chrono::milliseconds(250);
  }
  const auto state = impl_->external.status();
  return {state.session_id, state.selected, state.active, state.motor_output_enabled,
      state.last_sequence, state.reason, state.input_window, state.fresh, state.joint_count,
      state.motor_disable_confirmed};
}

Result Doso::enterExternal(const std::string &session_id) {
  const auto response = impl_->external.enter(session_id);
  if (response.result.ok || !response.result.transport_ok) impl_->external_session = session_id;
  return impl_->rememberExternal(response);
}

Result Doso::enableExternal() { return impl_->rememberExternal(impl_->external.enable()); }

Result Doso::exitExternal() {
  const auto response = impl_->external.exit();
  if (response.result.ok && !response.value.selected && response.value.motor_disable_confirmed) {
    impl_->external_session.clear();
  }
  return impl_->rememberExternal(response);
}

Result Doso::externalHeartbeat() { return impl_->rememberExternal(impl_->external.heartbeat()); }

Result Doso::externalFrame(const message::ExternalFrame &frame) {
  brainstem::ExternalFrame command;
  command.sequence = frame.sequence;
  command.input_window = frame.input_window;
  for (std::size_t i = 0; i < command.joints.size(); ++i) {
    const auto &joint = frame.joints[i];
    command.joints[i] = {joint.q, joint.dq, joint.kp, joint.kd, joint.tau};
  }
  auto mapped = result(impl_->external.send(command));
  mapped.state = impl_->last_control;
  return mapped;
}
#else
bool Doso::externalSelected() const { return false; }
message::ExternalState Doso::externalState() const { return {}; }
Result Doso::enterExternal(const std::string &) { return unsupportedExternalResult(); }
Result Doso::enableExternal() { return unsupportedExternalResult(); }
Result Doso::exitExternal() { return unsupportedExternalResult(); }
Result Doso::externalHeartbeat() { return unsupportedExternalResult(); }
Result Doso::externalFrame(const message::ExternalFrame &) { return unsupportedExternalResult(); }
#endif

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
