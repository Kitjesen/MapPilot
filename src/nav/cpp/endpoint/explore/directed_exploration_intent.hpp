#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "explore_contract.hpp"

namespace lingtu::nav::endpoint {

struct DirectedExplorationIntent {
  lingtu::explore::Pose2D target;
  lingtu::explore::ExploreMapIdentity map;
  std::string session_id;
  std::uint64_t revision{0U};
  double expires_at_s{0.0};
};

struct DirectedExplorationIntentResult {
  bool accepted{false};
  bool changed{false};
  std::string reason;
  std::uint64_t revision{0U};
};

class DirectedExplorationIntentStore final {
 public:
  static constexpr double kMaxTtlSeconds = 3600.0;
  static constexpr double kMaxCoordinateMeters = 1'000'000.0;

  [[nodiscard]] DirectedExplorationIntentResult Set(const std::string &session_id,
                                                    const lingtu::explore::ExploreMapIdentity &map,
                                                    double target_x, double target_y, double ttl_s,
                                                    double now_s);
  [[nodiscard]] DirectedExplorationIntentResult Clear();
  [[nodiscard]] bool Expire(double now_s);
  void Reset();

  [[nodiscard]] std::optional<DirectedExplorationIntent>
  current(const std::string &session_id, const lingtu::explore::ExploreMapIdentity &map,
          double now_s) const;
  [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }

 private:
  [[nodiscard]] std::uint64_t AdvanceRevision() noexcept;
  std::optional<DirectedExplorationIntent> current_;
  std::uint64_t revision_{0U};
};

}  // namespace lingtu::nav::endpoint
