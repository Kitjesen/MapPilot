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
  Capabilities capabilities() const noexcept override;
  BodyState state() const override;
  HealthState health() const override;
  AdapterDiagnostics diagnostics() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::driver
