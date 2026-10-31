#include "lingtu/recording/history_series.hpp"
#include "lingtu/recording/message_preview.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "mcap/reader.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <locale>
#include <map>
#include <sstream>

namespace lingtu::recording {
namespace {
[[noreturn]] void unreadable() {
  throw SeriesError("recording_series_unreadable", "Recorded DDS data or its typed schema could not be read.");
}
bool header_field(const std::string &path) { return path == "stamp_sec" || path == "stamp_nanosec"; }
struct IndexedSample { std::uint64_t index{}, segment{}; SeriesSample sample; };
struct Bucket {
  std::optional<IndexedSample> first, last, minimum, maximum, missing, gap;
  void add(IndexedSample point) {
    if (!first) first = point;
    last = point;
    if (!point.sample.value) { if (!missing) missing = point; }
    else {
      if (!minimum || *point.sample.value < *minimum->sample.value) minimum = point;
      if (!maximum || *point.sample.value > *maximum->sample.value) maximum = point;
    }
    if (point.sample.gap_before && !gap) gap = point;
  }
};
}  // namespace

HistorySeries read_history_series(const std::filesystem::path &file, const SeriesQuery &query) {
  if (!std::isfinite(query.start_s) || query.start_s < 0 ||
      (query.end_s && (!std::isfinite(*query.end_s) || *query.end_s < query.start_s)) ||
      query.max_points < 16 || query.max_points > 10000)
    throw SeriesError("recording_series_invalid_query", "Use a finite nonnegative time window and 16 to 10000 output points.");
  const auto *binding = find_recording_topic(query.topic);
  if (!binding) throw SeriesError("recording_series_unsupported_topic", "This topic has no current LingTu DDS recording contract.");
  mcap::McapReader reader;
  if (!reader.open(file.string()).ok() || !reader.header() || reader.header()->profile != kMcapProfile) unreadable();
  bool failed = false;
  const auto problem = [&](const mcap::Status &) { failed = true; };
  if (!reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan, problem).ok() || failed || !reader.statistics()) unreadable();
  const auto &stats = *reader.statistics();
  HistorySeries result;
  result.topic = std::string(binding->contract->topic);
  result.start_time_ns = stats.messageStartTime;
  result.duration_s = stats.messageCount && stats.messageEndTime >= stats.messageStartTime ?
      static_cast<double>(stats.messageEndTime - stats.messageStartTime) / 1e9 : 0;
  result.start_s = std::min(query.start_s, result.duration_s);
  result.end_s = std::min(query.end_s.value_or(result.duration_s), result.duration_s);
  result.max_points = query.max_points;
  for (const auto &[id, channel] : reader.channels()) {
    if (channel->topic != binding->contract->dds_topic) continue;
    const auto count = stats.channelMessageCounts.find(id);
    if (count != stats.channelMessageCounts.end()) result.original_count += count->second;
    const auto type = channel->metadata.find("lingtu.idl_type");
    const auto schema = reader.schema(channel->schemaId);
    if (channel->messageEncoding != "cdr" || type == channel->metadata.end() ||
        type->second != binding->contract->idl_type || !schema ||
        schema->encoding != "omgidl" || schema->name != binding->contract->idl_type) unreadable();
  }
  if (query.start_s > result.duration_s) return result;
  mcap::ReadMessageOptions options;
  const auto offset_ns = [](double seconds) { return static_cast<std::uint64_t>(std::llround(seconds * 1e9)); };
  options.startTime = result.start_time_ns + offset_ns(result.start_s);
  options.endTime = result.start_time_ns + offset_ns(result.end_s) + 1;
  options.topicFilter = [&](std::string_view topic) { return topic == binding->contract->dds_topic; };
  options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
  std::vector<double> periods;
  std::optional<std::uint64_t> previous;
  std::map<std::string, bool> seen;
  try {
    for (const auto &view : reader.readMessages(problem, options)) {
      ++result.read_count;
      if (previous && view.message.logTime > *previous && periods.size() < 4096)
        periods.push_back(static_cast<double>(view.message.logTime - *previous) / 1e9);
      previous = view.message.logTime;
      auto preview = numeric_preview(message_preview(*binding, view.message.data, view.message.dataSize));
      if (result.frame_id.empty()) result.frame_id = preview.frame_id;
      for (const auto &value : preview.values) {
        if (seen.emplace(value.path, true).second) result.fields.push_back({value.path, numeric_field_unit(value.path)});
        if (result.field.empty() && query.field.empty() && !header_field(value.path)) result.field = value.path;
      }
    }
    if (failed) unreadable();
    if (query.field.empty()) {
      for (const auto *preferred : {"position_rad[0]", "angular_velocity_rad_s.x", "pose.position.x", "twist.linear.x", "progress", "data"})
        if (seen.count(preferred)) { result.field = preferred; break; }
    }
    if (!query.field.empty()) {
      if (!seen.count(query.field) && result.read_count)
        throw SeriesError("recording_series_unsupported_field", "Choose a numeric field from the recorded topic's fields list.");
      result.field = query.field;
    }
    if (result.field.empty() && !result.fields.empty()) result.field = result.fields.front().path;
    if (result.field.empty() || !result.read_count) return result;
    if (!periods.empty()) {
      auto middle = periods.begin() + periods.size() / 2;
      std::nth_element(periods.begin(), middle, periods.end());
      result.gap_threshold_s = *middle * 5;
    }
    // Six representatives per bucket: first/last, min/max, null and gap.
    // Source index rather than timestamp deduplicates, preserving equal-time samples.
    const auto bucket_count = std::max<std::size_t>(1, query.max_points / 6);
    std::vector<Bucket> buckets(bucket_count);
    std::vector<SeriesSample> exact;
    exact.reserve(std::min<std::uint64_t>(result.read_count, query.max_points));
    std::uint64_t index = 0, segment = 0;
    previous.reset();
    for (const auto &view : reader.readMessages(problem, options)) {
      auto preview = numeric_preview(message_preview(*binding, view.message.data, view.message.dataSize));
      std::optional<double> value;
      for (const auto &candidate : preview.values) if (candidate.path == result.field) { value = candidate.value; break; }
      if (value) ++result.numeric_count;
      const auto t = static_cast<double>(view.message.logTime - result.start_time_ns) / 1e9;
      const bool gap = previous && result.gap_threshold_s > 0 &&
          static_cast<double>(view.message.logTime - *previous) / 1e9 > result.gap_threshold_s;
      if (gap || !value) ++segment;
      SeriesSample sample{t, value, gap};
      if (result.read_count <= query.max_points) exact.push_back(sample);
      const auto bucket = std::min<std::size_t>(bucket_count - 1, index * bucket_count / result.read_count);
      buckets[bucket].add({index++, segment, sample});
      if (!value) ++segment;
      previous = view.message.logTime;
    }
    if (failed) unreadable();
    result.downsampled = result.read_count > query.max_points;
    if (!result.downsampled) result.samples = std::move(exact);
    else {
      std::map<std::uint64_t, IndexedSample> retained;
      for (const auto &bucket : buckets) for (const auto *point :
          {&bucket.first, &bucket.last, &bucket.minimum, &bucket.maximum, &bucket.missing, &bucket.gap})
        if (*point) retained.emplace((*point)->index, **point);
      // Segment identity survives discarded samples: multiple gaps/nonfinite
      // intervals in one bucket cannot become a fabricated connecting line.
      std::optional<std::uint64_t> previous_segment;
      for (auto &[unused, point] : retained) {
        if (previous_segment && *previous_segment != point.segment) point.sample.gap_before = true;
        previous_segment = point.segment;
        result.samples.push_back(point.sample);
      }
    }
  } catch (const SeriesError &) { throw; }
  catch (const std::exception &) { unreadable(); }
  return result;
}

std::string HistorySeries::json() const {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"version\":1,\"ok\":true,\"mode\":\"series\",\"topic\":" << json_quote(topic)
      << ",\"field\":" << (field.empty() ? "null" : json_quote(field)) << ",\"fields\":[";
  bool first = true;
  for (const auto &item : fields) {
    if (!first) out << ',';
    first = false;
    out << "{\"path\":" << json_quote(item.path) << ",\"unit\":" << json_quote(item.unit) << '}';
  }
  out << "],\"start_time_ns\":" << json_quote(std::to_string(start_time_ns))
      << ",\"duration_s\":" << json_number(duration_s) << ",\"window\":{\"start_s\":" << json_number(start_s)
      << ",\"end_s\":" << json_number(end_s) << "},\"frame_id\":"
      << (frame_id.empty() ? "null" : json_quote(frame_id)) << ",\"original_count\":" << original_count
      << ",\"read_count\":" << read_count << ",\"numeric_count\":" << numeric_count
      << ",\"returned_count\":" << samples.size() << ",\"gap_threshold_s\":" << json_number(gap_threshold_s)
      << ",\"downsampling\":{\"method\":" << json_quote(downsampled ? "minmax" : "none")
      << ",\"max_points\":" << max_points << "},\"samples\":[";
  first = true;
  for (const auto &sample : samples) {
    if (!first) out << ',';
    first = false;
    out << "{\"t\":" << json_number(sample.t) << ",\"value\":"
        << (sample.value ? json_number(*sample.value) : "null");
    if (sample.gap_before) out << ",\"gap_before\":true";
    out << '}';
  }
  return out.str() + "]}";
}
}  // namespace lingtu::recording
