#include "drivers/sim/measurement/control_bridge.h"

#include "message/generated/enums.hpp"
#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"
#include "dds/dds.h"
#include "messages.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

namespace {
thread_local char last_error[1024]{};
void error(const char* message) noexcept {
  std::snprintf(last_error, sizeof(last_error), "%s", message);
}
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return value;
}
std::size_t text_size(const char* value, std::size_t capacity, bool nonempty) {
  require(value != nullptr, "sensor control string is null");
  std::size_t size = 0;
  while (size < capacity && value[size] != '\0') ++size;
  require(size < capacity && (!nonempty || size > 0),
          "sensor control string exceeds bounds or required identity is empty");
  return size;
}
template <std::size_t N> void copy_text(char (&out)[N], const char* value, bool nonempty = false) {
  const auto size = text_size(value, N, nonempty);
  std::memcpy(out, value, size + 1);
}
template <typename T> void validate_ids(const T& value) {
  text_size(value.request_id, LT_SENSOR_CONTROL_ID_CAP, true);
  text_size(value.product_session_id, LT_SENSOR_CONTROL_ID_CAP, true);
  text_size(value.session_id, LT_SENSOR_CONTROL_ID_CAP, true);
  text_size(value.sensor_id, LT_SENSOR_CONTROL_ID_CAP, true);
}
void validate(const lt_sensor_control_request& value) {
  validate_ids(value);
  text_size(value.noise_unit, LT_SENSOR_CONTROL_UNIT_CAP, false);
  require(value.operation >= static_cast<int32_t>(lingtu::message::SensorControlKind::kQuery) &&
          value.operation <= static_cast<int32_t>(lingtu::message::SensorControlKind::kSetGimbal),
          "invalid sensor control operation");
  require(value.enabled == 0 || value.enabled == 1, "sensor control enabled must be 0 or 1");
  require(std::isfinite(value.deadline_s) && value.deadline_s >= 0 &&
          std::isfinite(value.rate_hz) && value.rate_hz >= 0 &&
          std::isfinite(value.noise_stddev) && value.noise_stddev >= 0 &&
          std::isfinite(value.pan_deg) && std::isfinite(value.tilt_deg),
          "sensor control numeric fields must be finite; deadline/rate/stddev must be nonnegative");
}
void validate(const lt_sensor_control_result& value) {
  validate_ids(value);
  text_size(value.reason, LT_SENSOR_CONTROL_REASON_CAP, false);
  text_size(value.settings_json, LT_SENSOR_CONTROL_SETTINGS_CAP, false);
  require(value.applied == 0 || value.applied == 1, "sensor control applied must be 0 or 1");
}

struct Participant {
  dds_entity_t value{};
  ~Participant() { if (value > 0) dds_delete(value); }
  Participant() = default;
  Participant(const Participant&) = delete;
  Participant& operator=(const Participant&) = delete;
};
struct Loan {
  dds_entity_t reader;
  void* samples[1]{};
  dds_sample_info_t infos[1]{};
  dds_return_t count;
  explicit Loan(dds_entity_t entity) : reader(entity),
      count(checked(dds_take(reader, samples, infos, 1, 1), "dds_take(sensor control)")) {}
  ~Loan() { if (count > 0) dds_return_loan(reader, samples, count); }
  Loan(const Loan&) = delete;
  Loan& operator=(const Loan&) = delete;
};

lt_sensor_control_request decode(const lingtu_dds_SensorControlRequest& value) {
  lt_sensor_control_request out{};
  copy_text(out.request_id, value.request_id, true);
  copy_text(out.product_session_id, value.product_session_id, true);
  copy_text(out.session_id, value.session_id, true);
  copy_text(out.sensor_id, value.sensor_id, true);
  out.model_generation = value.model_generation;
  out.reset_generation = value.reset_generation;
  out.deadline_s = value.deadline_s;
  out.operation = value.operation;
  out.enabled = value.enabled ? 1 : 0;
  out.rate_hz = value.rate_hz;
  out.noise_stddev = value.noise_stddev;
  out.noise_seed = value.noise_seed;
  copy_text(out.noise_unit, value.noise_unit);
  out.pan_deg = value.pan_deg;
  out.tilt_deg = value.tilt_deg;
  validate(out);
  return out;
}
lt_sensor_control_result decode(const lingtu_dds_SensorControlResult& value) {
  lt_sensor_control_result out{};
  copy_text(out.request_id, value.request_id, true);
  copy_text(out.product_session_id, value.product_session_id, true);
  copy_text(out.session_id, value.session_id, true);
  copy_text(out.sensor_id, value.sensor_id, true);
  out.model_generation = value.model_generation;
  out.reset_generation = value.reset_generation;
  out.applied_sim_time_ns = value.applied_sim_time_ns;
  out.applied = value.applied ? 1 : 0;
  copy_text(out.reason, value.reason);
  copy_text(out.settings_json, value.settings_json);
  validate(out);
  return out;
}
lingtu_dds_SensorControlRequest encode(const lt_sensor_control_request& value) {
  lingtu_dds_SensorControlRequest out{};
  out.request_id = const_cast<char*>(value.request_id);
  out.product_session_id = const_cast<char*>(value.product_session_id);
  out.session_id = const_cast<char*>(value.session_id);
  out.sensor_id = const_cast<char*>(value.sensor_id);
  out.model_generation = value.model_generation;
  out.reset_generation = value.reset_generation;
  out.deadline_s = value.deadline_s;
  out.operation = value.operation;
  out.enabled = value.enabled != 0;
  out.rate_hz = value.rate_hz;
  out.noise_stddev = value.noise_stddev;
  out.noise_seed = value.noise_seed;
  out.noise_unit = const_cast<char*>(value.noise_unit);
  out.pan_deg = value.pan_deg;
  out.tilt_deg = value.tilt_deg;
  return out;
}
lingtu_dds_SensorControlResult encode(const lt_sensor_control_result& value) {
  lingtu_dds_SensorControlResult out{};
  out.request_id = const_cast<char*>(value.request_id);
  out.product_session_id = const_cast<char*>(value.product_session_id);
  out.session_id = const_cast<char*>(value.session_id);
  out.sensor_id = const_cast<char*>(value.sensor_id);
  out.model_generation = value.model_generation;
  out.reset_generation = value.reset_generation;
  out.applied_sim_time_ns = value.applied_sim_time_ns;
  out.applied = value.applied != 0;
  out.reason = const_cast<char*>(value.reason);
  out.settings_json = const_cast<char*>(value.settings_json);
  return out;
}

class ControlBridge final {
 public:
  ControlBridge(int domain, const char* product, const char* source, int owner_mode)
      : product_(product, text_size(product, LT_SENSOR_CONTROL_ID_CAP, true)),
        source_(source, text_size(source, LT_SENSOR_CONTROL_ID_CAP, true)), owner_(owner_mode == 1) {
    require(domain >= 0, "sensor control domain must be nonnegative");
    require(owner_mode == 0 || owner_mode == 1, "sensor control owner_mode must be 0 or 1");
    participant_.value = checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr),
                                 "dds_create_participant(sensor control)");
    request_ = endpoint(&lingtu_dds_SensorControlRequest_desc, lingtu::message::kSensorControlRequest, owner_);
    result_ = endpoint(&lingtu_dds_SensorControlResult_desc, lingtu::message::kSensorControlResult, !owner_);
  }
  template <typename Wire, typename Value> int take(Value& out, bool request) {
    mode(request);
    const auto reader = request ? request_ : result_;
    for (unsigned i = 0; i < 32; ++i) {
      Loan loan(reader);
      if (!loan.count) return 0;
      if (!loan.infos[0].valid_data) continue;
      const auto& value = *static_cast<const Wire*>(loan.samples[0]);
      if (!bound(value)) continue;
      out = decode(value);
      return 1;
    }
    return 0;
  }
  template <typename Value> int write(const Value& value, bool request) {
    mode(!request);
    validate(value);
    require(bound(value), "sensor control message does not match bound product/source");
    const auto wire = encode(value);
    checked(dds_write(request ? request_ : result_, &wire), "dds_write(sensor control)");
    return 0;
  }

 private:
  void mode(bool owner) const { require(owner_ == owner, "sensor control operation is unavailable in this owner_mode"); }
  template <typename Value> bool bound(const Value& value) const {
    return bound(value.product_session_id, value.session_id);
  }
  bool bound(const char* product, const char* source) const {
    return product && source &&
        std::strncmp(product_.c_str(), product, LT_SENSOR_CONTROL_ID_CAP) == 0 &&
        std::strncmp(source_.c_str(), source, LT_SENSOR_CONTROL_ID_CAP) == 0;
  }
  dds_entity_t endpoint(const dds_topic_descriptor_t* descriptor,
                        const lingtu::message::TopicContract& contract, bool reader) {
    const auto topic = checked(dds_create_topic(participant_.value, descriptor,
        contract.dds_topic.data(), nullptr, nullptr), "dds_create_topic(sensor control)");
    auto qos = lingtu::dds::make_qos(contract.qos_profile);
    return reader ? checked(dds_create_reader(participant_.value, topic, qos.get(), nullptr),
                            "dds_create_reader(sensor control)") :
                    checked(dds_create_writer(participant_.value, topic, qos.get(), nullptr),
                            "dds_create_writer(sensor control)");
  }
  std::string product_, source_;
  bool owner_;
  Participant participant_;
  dds_entity_t request_{};
  dds_entity_t result_{};
};

template <typename F> int call(void* handle, const void* value, F operation) noexcept {
  error("");
  try {
    require(handle && value, "invalid sensor control arguments");
    return operation(*static_cast<ControlBridge*>(handle));
  } catch (const std::exception& e) { error(e.what()); }
  catch (...) { error("unknown sensor control transport error"); }
  return -1;
}
}  // namespace

extern "C" void* lt_sensor_control_create(int domain, const char* product, const char* source, int owner_mode) {
  error("");
  try { return new ControlBridge(domain, product, source, owner_mode); }
  catch (const std::exception& e) { error(e.what()); }
  catch (...) { error("unknown sensor control create error"); }
  return nullptr;
}
extern "C" void lt_sensor_control_destroy(void* handle) { delete static_cast<ControlBridge*>(handle); }
extern "C" int lt_sensor_control_take_request(void* handle, lt_sensor_control_request* request) {
  return call(handle, request, [&](ControlBridge& bridge) {
    return bridge.take<lingtu_dds_SensorControlRequest>(*request, true);
  });
}
extern "C" int lt_sensor_control_take_result(void* handle, lt_sensor_control_result* result) {
  return call(handle, result, [&](ControlBridge& bridge) {
    return bridge.take<lingtu_dds_SensorControlResult>(*result, false);
  });
}
extern "C" int lt_sensor_control_write_request(void* handle, const lt_sensor_control_request* request) {
  return call(handle, request, [&](ControlBridge& bridge) { return bridge.write(*request, true); });
}
extern "C" int lt_sensor_control_write_result(void* handle, const lt_sensor_control_result* result) {
  return call(handle, result, [&](ControlBridge& bridge) { return bridge.write(*result, false); });
}
extern "C" const char* lt_sensor_control_last_error(void) { return last_error; }
