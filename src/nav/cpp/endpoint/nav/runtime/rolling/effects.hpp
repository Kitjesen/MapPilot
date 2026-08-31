#pragma once

#include <functional>
#include <string>

#include "runtime/rolling/lifecycle.hpp"

namespace lingtu::nav::endpoint {

struct RollingSegmentEffectActions {
  std::function<bool()> activate_authority;
  std::function<void(const RollingSegmentInstallPathEffect &)> install_path;
  std::function<void(const RollingSegmentPublishPathEffect &)> publish_path;
  std::function<bool(const RollingSegmentAck &)> publish_ack;
  std::function<bool(const RollingSegmentStatus &)> publish_status;
  std::function<void()> stop_authority;
  std::function<bool(const std::string &)> clear_motion;
};

class RollingSegmentEffectCoordinator {
 public:
  RollingSegmentEffectCoordinator(RollingSegmentLifecycle &lifecycle,
                                  RollingSegmentEffectActions actions);

  [[nodiscard]] bool apply(const RollingSegmentStepResult &step_result);

 private:
  [[nodiscard]] bool applyEffect(const RollingSegmentEffect &effect, bool &abort_batch_on_failure);
  void validateActions() const;

  RollingSegmentLifecycle &lifecycle_;
  RollingSegmentEffectActions actions_;
};

}  // namespace lingtu::nav::endpoint
