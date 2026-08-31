#pragma once

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lingtu/maps/save.hpp"

namespace lingtu::maps {
class MapsServiceCore;
}

namespace lingtu::maps::mapd {

struct SlamSnapshotRequest {
  std::string request_id;
  std::string map_id;
  std::string product_session_id;
  std::filesystem::path output_path;
  bool save_patches{true};
};

struct SlamSnapshotAck {
  std::string request_id;
  std::string map_id;
  bool success{false};
  std::string message;
  std::filesystem::path output_path;
  std::string runtime_instance_id;
  std::string product_session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t observation_sequence{0U};
  std::uint64_t captured_at_ns{0U};
  std::string frame_id;
  std::uint64_t point_count{0U};
  std::string state;
  bool healthy{false};
  std::string health_message;
};

class SlamSnapshotExchange {
 public:
  virtual ~SlamSnapshotExchange() = default;
  virtual bool Publish(const SlamSnapshotRequest &request) = 0;
  virtual std::vector<SlamSnapshotAck> TakeAcks() = 0;
};

struct SaveCoordinatorConfig {
  std::string product;
  std::string product_session_id;
  bool save_patches{true};
  SaveMapRequest request_defaults;
};

class SaveCoordinator final {
 public:
  SaveCoordinator(MapsServiceCore &service, SlamSnapshotExchange &exchange,
                  SaveCoordinatorConfig config);

  std::string SaveMapJson(const std::string &request_id, const std::string &map_id);
  std::string RetrySaveMapJson(const std::string &job_id);
  std::string CancelSaveMapJson(const std::string &job_id);
  void Poll();

 private:
  struct Pending {
    std::string job_id;
    std::string map_id;
    std::filesystem::path output_path;
  };

  std::string RequestSnapshot(std::string response, const std::string &public_action);

  MapsServiceCore &service_;
  SlamSnapshotExchange &exchange_;
  SaveCoordinatorConfig config_;
  std::mutex mutex_;
  std::unordered_map<std::string, Pending> pending_;
  std::unordered_map<std::string, std::string> current_capture_;
  std::uint64_t next_capture_attempt_{0U};
};

}  // namespace lingtu::maps::mapd
