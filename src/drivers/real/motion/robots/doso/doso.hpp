#pragma once

#include <memory>

#include "body.hpp"
#include "config.hpp"

namespace lingtu::driver {

inline constexpr const char *kBrainstemMotionPrincipal = kDriverMotionPrincipal;

class Doso final : public Body {
 public:
  explicit Doso(const Config &config);
  ~Doso();

  Doso(const Doso &) = delete;
  Doso &operator=(const Doso &) = delete;

  Result refresh() override;
  Result move(const Velocity &velocity) override;
  Result stop() noexcept override;
  Result act(BodyAction action) override;
  Result motorPower(bool enabled) override;
  Result inspectControl() override;
  Capabilities capabilities() const noexcept override;
  BodyState state() const override;
  HealthState health() const override;
  AdapterDiagnostics diagnostics() const override;
  ProfileState readProfile() override;
  Result switchProfile(const std::string &name) override;
  message::ExternalState externalState() const override;
  bool externalSelected() const override;
  Result enterExternal(const std::string &session_id) override;
  Result enableExternal() override;
  Result exitExternal() override;
  Result externalHeartbeat() override;
  Result externalFrame(const message::ExternalFrame &frame) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::driver
