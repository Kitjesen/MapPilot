#include "slam.hpp"

#include <memory>
#include <utility>

namespace lingtu::slam {
namespace {

class PointLioBackend final : public ISlamBackend {
 public:
  PointLioBackend() : contract_(makeContractBackend("pointlio")) {}

  Status configure(const SlamConfig& config) override {
    SlamConfig resolved = config;
    resolved.backend = "pointlio";
    const Status status = contract_->configure(resolved);
    if (!status.ok) {
      return status;
    }
    reason_ = "pointlio_algorithm_pending_ros_node_extraction";
    return Status::Ok(reason_);
  }

  Status setMode(SlamMode mode, const std::string& map_path) override {
    const Status status = contract_->setMode(mode, map_path);
    reason_ = "pointlio_algorithm_pending_ros_node_extraction";
    return status.ok ? Status::Ok(reason_) : status;
  }

  Status feedImu(const ImuSample& sample) override { return contract_->feedImu(sample); }
  Status feedLidar(const LidarFrame& frame) override { return contract_->feedLidar(frame); }
  Status feedGnss(const GnssSample& sample) override { return contract_->feedGnss(sample); }
  Status feedVisualOdom(const OdomSample& sample) override { return contract_->feedVisualOdom(sample); }
  Status setInitialPose(const Pose3d& pose) override { return contract_->setInitialPose(pose); }
  Status relocalize(const std::optional<Pose3d>& guess) override { return contract_->relocalize(guess); }

  Status tick() override {
    const Status status = contract_->tick();
    reason_ = "pointlio_algorithm_pending_ros_node_extraction";
    return status.ok ? Status::Ok(reason_) : status;
  }

  Status saveMap(const std::string& pcd_path) override { return contract_->saveMap(pcd_path); }
  Status loadMap(const std::string& pcd_path) override { return contract_->loadMap(pcd_path); }

  SlamOutputs outputs() const override {
    SlamOutputs out = contract_->outputs();
    out.state = SlamState::Degraded;
    out.confidence = 0.0;
    out.localization_quality = 0.0;
    out.reason = reason_;
    return out;
  }

  Status reset() override {
    contract_ = makeContractBackend("pointlio");
    reason_ = "pointlio_algorithm_pending_ros_node_extraction";
    return Status::Ok("reset");
  }

 private:
  std::unique_ptr<ISlamBackend> contract_;
  std::string reason_ = "pointlio_algorithm_pending_ros_node_extraction";
};

}  // namespace

std::unique_ptr<ISlamBackend> makePointLioBackend() {
  return std::make_unique<PointLioBackend>();
}

}  // namespace lingtu::slam
