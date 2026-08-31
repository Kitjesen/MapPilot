#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::slam {

struct SemanticMapPoint {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint16_t label{0U};
  float confidence{0.0F};
};

struct SemanticMapSnapshot {
  std::uint64_t generation{0U};
  std::string frame_id;
  std::string taxonomy;
  std::uint32_t taxonomy_version{0U};
  std::vector<SemanticMapPoint> points;
};

class SemanticMapClient final {
 public:
  SemanticMapClient() = default;
  ~SemanticMapClient() = default;

  SemanticMapClient(const SemanticMapClient &) = delete;
  SemanticMapClient &operator=(const SemanticMapClient &) = delete;

  bool available() const;
  bool load(const std::string &path, SemanticMapSnapshot *snapshot, std::string *error) const;

};

}  // namespace lingtu::slam
