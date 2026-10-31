#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace lingtu::sim::dds_adapter {

inline constexpr std::size_t kMeasurementHeaderBytes = 76;
inline constexpr std::uint32_t kMaxMeasurementPayloadBytes = 32U * 1024U * 1024U;
inline constexpr std::uint32_t kMaxMeasurementMetadataBytes = 1024U * 1024U;

enum class MeasurementModality : std::uint16_t { Thermal = 1, Acoustic = 2, Semantic = 3 };

struct MeasurementRecord {
  MeasurementModality modality{MeasurementModality::Thermal};
  std::string product_session_id, session_id, instance_id, sensor_id, frame_id;
  std::uint64_t model_generation{0}, reset_generation{0}, sequence{0}, sim_time_ns{0};
  std::uint32_t rows{0}, columns{0}, sample_rate_hz{0};
  // Empty voltage_v means bit0 was absent. Semantic IDs never pass through float.
  std::vector<float> primary, voltage_v;
  std::vector<std::uint32_t> class_ids, instance_ids;
  std::vector<std::uint8_t> valid_mask;
  std::string metadata_json;
};

// Throws runtime_error on malformed input; checks sizes before array allocation.
// Python validates finite JSON semantics; native retains UTF-8 metadata verbatim.
MeasurementRecord decode_measurement_payload(const std::vector<std::uint8_t> &payload);
// Symmetric, validating LTM1 encoder. Checks the complete size before allocation.
std::vector<std::uint8_t> encode_measurement_payload(const MeasurementRecord &record);

class MeasurementStreamValidator final {
 public:
  MeasurementStreamValidator(std::string expected_product_session_id,
                             std::string expected_session_id);
  bool accept(const MeasurementRecord &record, std::string &error);

 private:
  struct State {
    MeasurementModality modality;
    std::string instance_id, frame_id;
    std::uint64_t model_generation, reset_generation, sequence, sim_time_ns;
    std::uint32_t columns, sample_rate_hz;
  };
  std::string product_session_id_, session_id_;
  std::unordered_map<std::string, State> sensors_;
};

}  // namespace lingtu::sim::dds_adapter
