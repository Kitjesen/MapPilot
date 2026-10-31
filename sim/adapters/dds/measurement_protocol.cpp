#include "measurement_protocol.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lingtu::sim::dds_adapter {
namespace {
std::uint64_t integer(const std::uint8_t *p, unsigned bytes) {
  std::uint64_t value = 0;
  for (unsigned i = 0; i < bytes; ++i) value |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  return value;
}
void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(std::string("LTM1: ") + message);
}
bool whitespace(std::uint32_t c) {
  return (c >= 9 && c <= 13) || (c >= 0x1c && c <= 0x20) || c == 0x85 ||
         c == 0xa0 || c == 0x1680 || (c >= 0x2000 && c <= 0x200a) ||
         c == 0x2028 || c == 0x2029 || c == 0x202f || c == 0x205f || c == 0x3000;
}
// Strict UTF-8, including overlong/surrogate rejection. DDS strings cannot retain NUL.
bool text(const std::string &s, bool identifier) {
  if (s.empty() || (identifier && s.size() > 255)) return false;
  std::uint32_t first = 0, last = 0;
  for (std::size_t i = 0; i < s.size();) {
    const auto lead = static_cast<unsigned char>(s[i++]);
    std::uint32_t cp = lead;
    unsigned tails = 0;
    if (lead >= 0xc2 && lead <= 0xdf) { cp &= 0x1f; tails = 1; }
    else if (lead >= 0xe0 && lead <= 0xef) { cp &= 0x0f; tails = 2; }
    else if (lead >= 0xf0 && lead <= 0xf4) { cp &= 7; tails = 3; }
    else if (lead >= 0x80) return false;
    if (i + tails > s.size()) return false;
    for (unsigned j = 0; j < tails; ++j) {
      const auto next = static_cast<unsigned char>(s[i++]);
      if ((next & 0xc0) != 0x80) return false;
      cp = (cp << 6) | (next & 0x3f);
    }
    if (cp == 0 || (tails == 1 && cp < 0x80) || (tails == 2 && cp < 0x800) ||
        (tails == 3 && cp < 0x10000) || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff))
      return false;
    if (first == 0) first = cp;
    last = cp;
  }
  return !identifier || (!whitespace(first) && !whitespace(last));
}
float real(const std::uint8_t *p) {
  const auto bits = static_cast<std::uint32_t>(integer(p, 4));
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

MeasurementRecord decode_measurement_payload(const std::vector<std::uint8_t> &payload) {
  require(payload.size() >= kMeasurementHeaderBytes && payload.size() <= kMaxMeasurementPayloadBytes,
          "payload outside 76-byte/32 MiB limits");
  const auto *p = payload.data();
  require(std::memcmp(p, "LTM1", 4) == 0, "bad magic");
  require(integer(p + 4, 2) == 1, "unsupported version");
  const auto modality = integer(p + 6, 2);
  require(modality >= 1 && modality <= 3, "unknown modality");
  const auto header_bytes = integer(p + 8, 4), metadata_bytes = integer(p + 12, 4);
  require(metadata_bytes > 0 && metadata_bytes <= kMaxMeasurementMetadataBytes,
          "metadata outside 1-byte/1 MiB limits");
  MeasurementRecord r;
  r.modality = static_cast<MeasurementModality>(modality);
  r.rows = static_cast<std::uint32_t>(integer(p + 16, 4));
  r.columns = static_cast<std::uint32_t>(integer(p + 20, 4));
  r.sample_rate_hz = static_cast<std::uint32_t>(integer(p + 24, 4));
  const auto flags = integer(p + 28, 4);
  const bool acoustic = r.modality == MeasurementModality::Acoustic;
  const bool semantic = r.modality == MeasurementModality::Semantic;
  require(r.rows > 0 && r.columns > 0, "shape must be positive");
  require(acoustic ? r.sample_rate_hz > 0 : r.sample_rate_hz == 0, "invalid sample rate for modality");
  require(acoustic ? flags <= 1 : flags == 0, "unsupported flags");
  const std::uint64_t n = static_cast<std::uint64_t>(r.rows) * r.columns;
  // Bound the product before multiplying by array stride (rows*columns can fill uint64).
  const std::uint64_t stride = semantic || flags == 1 ? 9 : 5;
  require(n <= kMaxMeasurementPayloadBytes / stride, "shape exceeds payload limit");
  r.model_generation = integer(p + 32, 8); r.reset_generation = integer(p + 40, 8);
  r.sequence = integer(p + 48, 8); r.sim_time_ns = integer(p + 56, 8);
  require(r.sequence <= std::numeric_limits<std::uint32_t>::max(), "sequence exceeds LTU1 uint32");
  require(r.sim_time_ns / 1000000000ULL <= static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()),
          "time outside IDL int32 seconds");
  require(p[74] == 0 && p[75] == 0, "reserved bytes must be zero");
  std::array<std::size_t, 5> lengths{};
  std::size_t total_header = kMeasurementHeaderBytes;
  for (std::size_t i = 0; i < lengths.size(); ++i) {
    lengths[i] = static_cast<std::size_t>(integer(p + 64 + i * 2, 2));
    require(lengths[i] > 0 && lengths[i] <= 255, "identifier length outside 1..255");
    total_header += lengths[i];
  }
  require(header_bytes == total_header, "header_bytes disagrees with identifier lengths");
  require(header_bytes + metadata_bytes + n * stride == payload.size(), "payload size/shape mismatch");
  std::string *ids[] = {&r.product_session_id, &r.session_id, &r.instance_id, &r.sensor_id, &r.frame_id};
  std::size_t offset = kMeasurementHeaderBytes;
  for (std::size_t i = 0; i < lengths.size(); ++i) {
    ids[i]->assign(reinterpret_cast<const char *>(p + offset), lengths[i]);
    require(text(*ids[i], true), "identifier must be trimmed nonempty UTF-8 without NUL");
    offset += lengths[i];
  }
  r.metadata_json.assign(reinterpret_cast<const char *>(p + offset), static_cast<std::size_t>(metadata_bytes));
  require(text(r.metadata_json, false), "metadata must be UTF-8 without NUL");
  offset += static_cast<std::size_t>(metadata_bytes);
  const auto count = static_cast<std::size_t>(n);
  const auto *mask = p + offset + count * 4;
  const auto *secondary = mask + count;
  r.valid_mask.assign(mask, mask + count);
  if (semantic) { r.class_ids.resize(count); r.instance_ids.resize(count); }
  else { r.primary.resize(count); if (flags == 1) r.voltage_v.resize(count); }
  for (std::size_t i = 0; i < count; ++i) {
    require(mask[i] <= 1, "mask must contain only 0 or 1");
    if (semantic) {
      const auto cls = static_cast<std::uint32_t>(integer(p + offset + i * 4, 4));
      const auto instance = static_cast<std::uint32_t>(integer(secondary + i * 4, 4));
      require(mask[i] ? cls > 0 && instance > 0 && cls <= 0x7fffffff && instance <= 0x7fffffff
                      : cls == 0 && instance == 0, "invalid semantic labels");
      r.class_ids[i] = cls; r.instance_ids[i] = instance;
    } else {
      const float value = real(p + offset + i * 4);
      require(mask[i] ? std::isfinite(value) && (acoustic || value > 0) : std::isnan(value),
              "valid values must be finite (Kelvin >0), invalid values NaN");
      r.primary[i] = value;
      if (flags == 1) {
        const float volts = real(secondary + i * 4);
        require(mask[i] ? std::isfinite(volts) : std::isnan(volts), "invalid voltage/mask value");
        r.voltage_v[i] = volts;
      }
    }
  }
  return r;
}

std::vector<std::uint8_t> encode_measurement_payload(const MeasurementRecord &r) {
  const bool acoustic = r.modality == MeasurementModality::Acoustic;
  const bool semantic = r.modality == MeasurementModality::Semantic;
  require(acoustic || semantic || r.modality == MeasurementModality::Thermal, "unknown modality");
  require(r.rows > 0 && r.columns > 0, "shape must be positive");
  require(acoustic ? r.sample_rate_hz > 0 : r.sample_rate_hz == 0, "invalid sample rate for modality");
  const bool voltage = !r.voltage_v.empty();
  require(acoustic || !voltage, "voltage only belongs to acoustic");
  const std::uint64_t n = static_cast<std::uint64_t>(r.rows) * r.columns;
  const std::uint64_t stride = semantic || voltage ? 9 : 5;
  require(n <= kMaxMeasurementPayloadBytes / stride, "shape exceeds payload limit");
  require(r.valid_mask.size() == n, "mask size/shape mismatch");
  require(semantic ? r.class_ids.size() == n && r.instance_ids.size() == n && r.primary.empty()
                   : r.primary.size() == n && r.class_ids.empty() && r.instance_ids.empty(),
          "array size/modality mismatch");
  require(!voltage || r.voltage_v.size() == n, "voltage size/shape mismatch");
  const std::string *ids[] = {&r.product_session_id, &r.session_id, &r.instance_id, &r.sensor_id, &r.frame_id};
  std::size_t header = kMeasurementHeaderBytes;
  for (const auto *id : ids) {
    require(text(*id, true), "identifier must be trimmed nonempty UTF-8 without NUL");
    header += id->size();
  }
  require(!r.metadata_json.empty() && r.metadata_json.size() <= kMaxMeasurementMetadataBytes,
          "metadata outside 1-byte/1 MiB limits");
  const auto size = header + r.metadata_json.size() + n * stride;
  require(size <= kMaxMeasurementPayloadBytes, "payload exceeds 32 MiB");
  require(text(r.metadata_json, false), "metadata must be UTF-8 without NUL");
  require(r.sequence <= std::numeric_limits<std::uint32_t>::max(), "sequence exceeds LTU1 uint32");
  require(r.sim_time_ns / 1000000000ULL <= static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()),
          "time outside IDL int32 seconds");
  for (std::size_t i = 0; i < n; ++i) {
    const auto mask = r.valid_mask[i];
    require(mask <= 1, "mask must contain only 0 or 1");
    if (semantic) {
      const auto cls = r.class_ids[i], instance = r.instance_ids[i];
      require(mask ? cls > 0 && instance > 0 && cls <= 0x7fffffff && instance <= 0x7fffffff
                   : cls == 0 && instance == 0, "invalid semantic labels");
    } else {
      const float value = r.primary[i];
      require(mask ? std::isfinite(value) && (acoustic || value > 0) : std::isnan(value),
              "valid values must be finite (Kelvin >0), invalid values NaN");
      if (voltage) require(mask ? std::isfinite(r.voltage_v[i]) : std::isnan(r.voltage_v[i]),
                           "invalid voltage/mask value");
    }
  }
  std::vector<std::uint8_t> out;
  out.reserve(static_cast<std::size_t>(size));
  const auto put = [&out](std::uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) out.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
  };
  const auto floats = [&put](const std::vector<float> &values) {
    for (const float value : values) {
      std::uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      put(bits, 4);
    }
  };
  out.insert(out.end(), {'L', 'T', 'M', '1'});
  put(1, 2); put(static_cast<std::uint16_t>(r.modality), 2);
  put(header, 4); put(r.metadata_json.size(), 4);
  put(r.rows, 4); put(r.columns, 4); put(r.sample_rate_hz, 4); put(voltage ? 1 : 0, 4);
  put(r.model_generation, 8); put(r.reset_generation, 8); put(r.sequence, 8); put(r.sim_time_ns, 8);
  for (const auto *id : ids) put(id->size(), 2);
  put(0, 2);
  for (const auto *id : ids) out.insert(out.end(), id->begin(), id->end());
  out.insert(out.end(), r.metadata_json.begin(), r.metadata_json.end());
  if (semantic) { for (const auto value : r.class_ids) put(value, 4); }
  else floats(r.primary);
  out.insert(out.end(), r.valid_mask.begin(), r.valid_mask.end());
  if (semantic) { for (const auto value : r.instance_ids) put(value, 4); }
  else if (voltage) floats(r.voltage_v);
  return out;
}

MeasurementStreamValidator::MeasurementStreamValidator(std::string product, std::string session)
    : product_session_id_(std::move(product)), session_id_(std::move(session)) {}

bool MeasurementStreamValidator::accept(const MeasurementRecord &r, std::string &error) {
  error.clear();
  const auto reject = [&error](const char *message) { error = message; return false; };
  if (product_session_id_.empty() || r.product_session_id != product_session_id_)
    return reject("measurement product session id mismatch");
  if (session_id_.empty() || r.session_id != session_id_)
    return reject("measurement source session id mismatch");
  const auto found = sensors_.find(r.sensor_id);
  if (found != sensors_.end()) {
    const auto &previous = found->second;
    if (r.instance_id != previous.instance_id || r.frame_id != previous.frame_id ||
        r.modality != previous.modality)
      return reject("measurement sensor instance/frame/modality changed");
    if (r.model_generation < previous.model_generation ||
        (r.model_generation == previous.model_generation && r.reset_generation < previous.reset_generation))
      return reject("measurement generation moved backward");
    const bool same_generation = r.model_generation == previous.model_generation &&
                                 r.reset_generation == previous.reset_generation;
    if (same_generation) {
      if (r.sequence <= previous.sequence || r.sim_time_ns <= previous.sim_time_ns)
        return reject("measurement sequence/time must increase within generation");
      // Source block boundaries are floored to integer nanoseconds by the sample clock.
      if (r.modality == MeasurementModality::Acoustic &&
          r.sim_time_ns - previous.sim_time_ns <
              static_cast<std::uint64_t>(previous.columns) * 1000000000ULL / previous.sample_rate_hz)
        return reject("measurement acoustic blocks overlap their sample duration");
    }
  }
  sensors_.insert_or_assign(r.sensor_id, State{r.modality, r.instance_id, r.frame_id,
      r.model_generation, r.reset_generation, r.sequence, r.sim_time_ns, r.columns, r.sample_rate_hz});
  return true;
}
}  // namespace lingtu::sim::dds_adapter
