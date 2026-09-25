#include "manual_relocalization.hpp"

#include <stdexcept>

using namespace lingtu::slam;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

struct Backend : ISlamBackend {
  int ticks = 0;
  int scans = 0;
  int starts = 0;
  bool in_flight = false;
  std::optional<Status> completion;
  Status start_status = Status::Ok();
  std::optional<Pose3d> seed;
  RelocalizationSearch search = RelocalizationSearch::Automatic;
  Status configure(const SlamConfig&) override { return Status::Ok(); }
  Status setMode(SlamMode, const std::string&) override { return Status::Ok(); }
  Status feedImu(const ImuSample&) override { return Status::Ok(); }
  Status feedLidar(const LidarFrame&) override { ++scans; return Status::Ok(); }
  Status feedGnss(const GnssSample&) override { return Status::Ok(); }
  Status feedVisualOdom(const OdomSample&) override { return Status::Ok(); }
  Status setInitialPose(const Pose3d&) override { return Status::Ok(); }
  Status relocalize(const std::optional<Pose3d>&, RelocalizationSearch) override {
    throw std::runtime_error("synchronous registration must not run on the sensor loop");
  }
  Status startRelocalizeAsync(const std::optional<Pose3d>& guess, RelocalizationSearch mode) override {
    ++starts;
    seed = guess;
    search = mode;
    in_flight = start_status.ok;
    return start_status;
  }
  std::optional<Status> pollRelocalizeAsync() override {
    auto result = completion;
    if (result) { completion.reset(); in_flight = false; }
    return result;
  }
  bool relocalizeAsyncInFlight() const override { return in_flight; }
  Status tick() override { ++ticks; return Status::Ok(); }
  Status saveMap(const std::string&) override { return Status::Ok(); }
  Status loadMap(const std::string&) override { return Status::Ok(); }
  SlamOutputs outputs() const override { return {}; }
  Status reset() override { return Status::Ok(); }
};

int main() {
  Backend backend;
  ManualRelocalization manual;
  RelocalizationRequestIdentity identity{"original-request", "seeded_relocalize", "seeded_gicp"};
  Pose3d seed;
  seed.z = 0.35;
  require(manual.start(backend, identity, seed, RelocalizationSearch::Automatic).ok, "start failed");
  identity = {"DDS storage reused", "changed", "changed"};
  require(manual.owns("original-request"), "request identity was not retained");
  require(backend.seed && backend.seed->z == 0.35, "seed height lost");
  require(!manual.start(backend, identity, seed, RelocalizationSearch::Automatic).ok, "overlapping request accepted");
  require(backend.starts == 1, "busy request replaced native work");
  for (int i = 0; i < 50; ++i) {
    backend.feedLidar({});
    backend.tick();
    require(!manual.poll(backend), "reported success before registration finished");
  }
  require(backend.scans == 50 && backend.ticks == 50, "sensor loop stopped");
  backend.completion = Status::Ok("seed_verified");
  auto done = manual.poll(backend);
  require(done && done->status.ok, "missing final success");
  require(done->request.request_id == "original-request" &&
          done->request.action == "seeded_relocalize" && done->request.engine == "seeded_gicp",
          "DDS identity changed before completion");
  require(!manual.pending() && !manual.poll(backend), "completion delivered twice");

  backend.in_flight = true;  // The periodic tracker owns the native worker.
  require(!manual.start(backend, identity, {}, RelocalizationSearch::Global).ok, "periodic work replaced");
  backend.completion = Status::Ok("periodic");
  require(!manual.poll(backend) && backend.completion.has_value(), "consumed periodic completion");
  backend.pollRelocalizeAsync();
  require(manual.start(backend, identity, {}, RelocalizationSearch::Global).ok, "global start failed");
  require(!backend.seed && backend.search == RelocalizationSearch::Global, "global search reused a seed");
  backend.completion = Status::Error("quality_rejected");
  done = manual.poll(backend);
  require(done && !done->status.ok && done->status.message == "quality_rejected", "rejection lost");
  backend.start_status = Status::Error("registered_cloud_unavailable");
  require(!manual.start(backend, identity, {}, RelocalizationSearch::Global).ok && !manual.pending(),
          "failed start left a pending reply");
}
