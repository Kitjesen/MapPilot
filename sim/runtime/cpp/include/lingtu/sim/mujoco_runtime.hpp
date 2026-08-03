#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace lingtu::sim {

struct BodyPose {
  std::int32_t body_id{0};
  std::string name;
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
};

struct SimulationSnapshot {
  std::uint64_t sequence{0};
  std::uint64_t physics_step{0};
  std::uint64_t sim_time_ns{0};
  std::uint32_t reset_generation{0};
  std::vector<BodyPose> bodies;
};

struct MujocoRuntimeConfig {
  std::filesystem::path model_path;
};

// Single-owner MuJoCo runtime. All calls must be made from one owning thread.
// Snapshot storage is allocated while loading the model and reused thereafter.
class MujocoRuntime final {
 public:
  explicit MujocoRuntime(MujocoRuntimeConfig config);
  ~MujocoRuntime();

  MujocoRuntime(MujocoRuntime &&) noexcept;
  MujocoRuntime &operator=(MujocoRuntime &&) noexcept;

  MujocoRuntime(const MujocoRuntime &) = delete;
  MujocoRuntime &operator=(const MujocoRuntime &) = delete;

  const SimulationSnapshot &advance(std::uint32_t steps = 1);
  const SimulationSnapshot &reset();
  void set_paused(bool paused) noexcept;

  [[nodiscard]] bool paused() const noexcept;
  [[nodiscard]] double timestep_seconds() const noexcept;
  [[nodiscard]] const SimulationSnapshot &snapshot() const noexcept;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lingtu::sim
