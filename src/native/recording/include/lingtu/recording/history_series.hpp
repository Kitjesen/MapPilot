#pragma once
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lingtu::recording {
struct SeriesQuery {
  std::string topic;
  std::string field;
  double start_s{0};
  std::optional<double> end_s;
  std::size_t max_points{2000};
};
struct SeriesSample {
  double t{};
  std::optional<double> value;
  bool gap_before{};
};
struct SeriesField { std::string path, unit; };
struct HistorySeries {
  std::string topic, field, frame_id;
  std::uint64_t start_time_ns{}, original_count{}, read_count{}, numeric_count{};
  double duration_s{}, start_s{}, end_s{}, gap_threshold_s{};
  std::vector<SeriesField> fields;
  std::vector<SeriesSample> samples;
  std::size_t max_points{};
  bool downsampled{};
  std::string json() const;
};
class SeriesError : public std::runtime_error {
 public:
  SeriesError(std::string code, std::string message) : std::runtime_error(std::move(message)), code(std::move(code)) {}
  std::string code;
};
HistorySeries read_history_series(const std::filesystem::path &file, const SeriesQuery &query);
}  // namespace lingtu::recording
