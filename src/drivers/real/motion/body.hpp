#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "core.hpp"

namespace lingtu::driver {

inline constexpr char kDriverMotionPrincipal[] = "lingtu-driver@robot";

enum class BodyAction {
  Stand,
  Sit,
  Recover,
  Damp,
};

enum class Posture {
  Unknown,
  Standing,
  Sitting,
  Lying,
  Damping,
  Recovering,
  Transitioning,
};

struct Capabilities {
  bool stand{false};
  bool sit{false};
  bool recover{false};
  bool damp{false};

  bool supports(BodyAction action) const noexcept {
    switch (action) {
      case BodyAction::Stand:
        return stand;
      case BodyAction::Sit:
        return sit;
      case BodyAction::Recover:
        return recover;
      case BodyAction::Damp:
        return damp;
    }
    return false;
  }
};

struct BodyState {
  bool fresh{false};
  Posture posture{Posture::Unknown};
  Velocity velocity;
  bool velocity_available{false};
  std::array<double, 3> odometry_position_m{};
  bool odometry_position_available{false};
  double height_m{0.0};
  bool height_available{false};
};

struct HealthState {
  bool fresh{false};
  bool healthy{false};
  std::uint32_t fault_code{0};
  std::string reason{"unknown"};
};

struct ControlState {
  bool connected{false};
  bool ready{false};
  bool motors_enabled{false};
  bool critical_fault{false};
  bool control_assured{false};
  bool lease_valid{false};
  std::uint32_t lease_remaining_ms{0};
  std::uint64_t accepted_sequence{0};
  bool initial_zero_acknowledged{false};
  std::string fsm{"unknown"};
  std::string reason{"not_connected"};
};

struct AdapterDiagnostics {
  std::string name;
  std::string protocol;
  std::string target;
  std::string control_owner{"none"};
  std::string control_owner_id;
  bool state_code_available{false};
  std::uint32_t state_code{0};
};

struct Result {
  bool ok{false};
  bool transport_ok{false};
  bool accepted{false};
  ControlState state;
  std::string error;
  unsigned calls{1};
  bool stop_confirmed{false};

  bool confirmsStop() const noexcept {
    return ok && transport_ok && accepted && stop_confirmed && !state.ready &&
           !state.control_assured && !state.lease_valid;
  }
};

class Body {
 public:
  virtual ~Body() = default;

  virtual Result refresh() = 0;
  virtual Result move(const Velocity &velocity) = 0;
  // A successful stop has one meaning for every adapter: the vendor endpoint
  // acknowledged its stop/zero command and this Body invalidated local motion
  // readiness. Callers must use Result::confirmsStop(), not transport success.
  virtual Result stop() noexcept = 0;
  virtual Result act(BodyAction action) = 0;
  virtual Capabilities capabilities() const noexcept = 0;
  virtual BodyState state() const = 0;
  virtual HealthState health() const = 0;
  virtual AdapterDiagnostics diagnostics() const = 0;
};

inline const char *bodyActionName(BodyAction action) noexcept {
  switch (action) {
    case BodyAction::Stand:
      return "stand";
    case BodyAction::Sit:
      return "sit";
    case BodyAction::Recover:
      return "recover";
    case BodyAction::Damp:
      return "damp";
  }
  return "unknown";
}

inline const char *postureName(Posture posture) noexcept {
  switch (posture) {
    case Posture::Standing:
      return "standing";
    case Posture::Sitting:
      return "sitting";
    case Posture::Lying:
      return "lying";
    case Posture::Damping:
      return "damping";
    case Posture::Recovering:
      return "recovering";
    case Posture::Transitioning:
      return "transitioning";
    case Posture::Unknown:
      return "unknown";
  }
  return "unknown";
}

struct Config;
std::unique_ptr<Body> makeBody(const Config &config);

}  // namespace lingtu::driver
