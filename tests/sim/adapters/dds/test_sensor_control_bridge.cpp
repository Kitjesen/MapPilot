#include "drivers/sim/measurement/control_bridge.h"
#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"
#include "dds/dds.h"
#include "messages.h"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

namespace {
constexpr int domain = 6;  // Transport fixture only; never the Windows Product domain (17).
unsigned checks = 0;
void require(bool ok, const std::string& reason) {
  ++checks;
  if (!ok) throw std::runtime_error(reason);
}
dds_return_t checked(dds_return_t value) {
  require(value >= 0, std::string("DDS: ") + dds_strretcode(-value));
  return value;
}
template <typename F> void until(F fn, const char* reason) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  do {
    if (fn()) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
  throw std::runtime_error(reason);
}
template <std::size_t N> void text(char (&out)[N], const std::string& value) {
  require(value.size() < N, "test string exceeds capacity");
  std::memset(out, 0, N);
  std::memcpy(out, value.c_str(), value.size() + 1);
}
void failed(int code, const char* reason) {
  require(code == -1 && std::strstr(lt_sensor_control_last_error(), reason),
          std::string("expected error '") + reason + "': " + lt_sensor_control_last_error());
}
using Bridge = std::unique_ptr<void, decltype(&lt_sensor_control_destroy)>;
Bridge bridge(int mode) {
  Bridge out(lt_sensor_control_create(domain, "product", "source", mode), lt_sensor_control_destroy);
  require(out != nullptr, lt_sensor_control_last_error());
  return out;
}
lt_sensor_control_request request() {
  lt_sensor_control_request out{};
  text(out.request_id, "request-123"); text(out.product_session_id, "product");
  text(out.session_id, "source"); text(out.sensor_id, u8"robot/\u96f7\u9706/thermal");
  out.model_generation = 0xfedcba9876543210ULL;
  out.reset_generation = 0x8765432101234567ULL;
  out.deadline_s = 1234.125; out.operation = 4; out.enabled = 1;
  out.rate_hz = 17.75; out.noise_stddev = 0.03125;
  out.noise_seed = 0xf123456789abcdefULL;
  text(out.noise_unit, "K"); out.pan_deg = -123.5; out.tilt_deg = 27.25;
  return out;
}
lt_sensor_control_result result() {
  const auto r = request();
  lt_sensor_control_result out{};
  text(out.request_id, r.request_id); text(out.product_session_id, r.product_session_id);
  text(out.session_id, r.session_id); text(out.sensor_id, r.sensor_id);
  out.model_generation = r.model_generation; out.reset_generation = r.reset_generation;
  out.applied_sim_time_ns = 0xe123456789abcdefULL; out.applied = 1;
  text(out.reason, u8"accepted/\u6e29\u5ea6");
  text(out.settings_json, "opaque settings bytes (not parsed as JSON)");
  return out;
}
lingtu_dds_SensorControlRequest wire(lt_sensor_control_request& r) {
  lingtu_dds_SensorControlRequest out{};
  out.request_id = r.request_id; out.product_session_id = r.product_session_id;
  out.session_id = r.session_id; out.sensor_id = r.sensor_id;
  out.model_generation = r.model_generation; out.reset_generation = r.reset_generation;
  out.deadline_s = r.deadline_s; out.operation = r.operation; out.enabled = r.enabled != 0;
  out.rate_hz = r.rate_hz; out.noise_stddev = r.noise_stddev; out.noise_seed = r.noise_seed;
  out.noise_unit = r.noise_unit; out.pan_deg = r.pan_deg; out.tilt_deg = r.tilt_deg;
  return out;
}
lingtu_dds_SensorControlResult wire(lt_sensor_control_result& r) {
  lingtu_dds_SensorControlResult out{};
  out.request_id = r.request_id; out.product_session_id = r.product_session_id;
  out.session_id = r.session_id; out.sensor_id = r.sensor_id;
  out.model_generation = r.model_generation; out.reset_generation = r.reset_generation;
  out.applied_sim_time_ns = r.applied_sim_time_ns; out.applied = r.applied != 0;
  out.reason = r.reason; out.settings_json = r.settings_json;
  return out;
}
template <typename A, typename B> void same_ids(const A& a, const B& b) {
  require(std::strcmp(a.request_id, b.request_id) == 0, "request_id changed");
  require(std::strcmp(a.product_session_id, b.product_session_id) == 0, "product changed");
  require(std::strcmp(a.session_id, b.session_id) == 0, "source changed");
  require(std::strcmp(a.sensor_id, b.sensor_id) == 0, "sensor changed");
  require(a.model_generation == b.model_generation && a.reset_generation == b.reset_generation,
          "generation changed");
}
template <typename T> void same_request(const T& a, const lt_sensor_control_request& b) {
  same_ids(a, b);
  require(a.deadline_s == b.deadline_s && a.operation == b.operation &&
          static_cast<int32_t>(a.enabled) == b.enabled &&
          a.rate_hz == b.rate_hz && a.noise_stddev == b.noise_stddev && a.noise_seed == b.noise_seed &&
          a.pan_deg == b.pan_deg && a.tilt_deg == b.tilt_deg &&
          std::strcmp(a.noise_unit, b.noise_unit) == 0, "request scalar/unit changed");
}
template <typename T> void same_result(const T& a, const lt_sensor_control_result& b) {
  same_ids(a, b);
  require(a.applied_sim_time_ns == b.applied_sim_time_ns && static_cast<int32_t>(a.applied) == b.applied &&
          std::strcmp(a.reason, b.reason) == 0 && std::strcmp(a.settings_json, b.settings_json) == 0,
          "result scalar/text changed");
}
struct Peer {
  dds_entity_t participant{};
  std::array<dds_entity_t, 2> writers{}, readers{};
  Peer() {
    participant = checked(dds_create_participant(domain, nullptr, nullptr));
    try {
      const lingtu::message::TopicContract contracts[] = {
          lingtu::message::kSensorControlRequest, lingtu::message::kSensorControlResult};
      const dds_topic_descriptor_t* descriptors[] = {
          &lingtu_dds_SensorControlRequest_desc, &lingtu_dds_SensorControlResult_desc};
      for (std::size_t i = 0; i < writers.size(); ++i) {
        const auto topic = checked(dds_create_topic(participant, descriptors[i],
            contracts[i].dds_topic.data(), nullptr, nullptr));
        auto qos = lingtu::dds::make_qos(contracts[i].qos_profile);
        writers[i] = checked(dds_create_writer(participant, topic, qos.get(), nullptr));
        readers[i] = checked(dds_create_reader(participant, topic, qos.get(), nullptr));
      }
    } catch (...) { dds_delete(participant); throw; }
  }
  ~Peer() { dds_delete(participant); }
  Peer(const Peer&) = delete;
  Peer& operator=(const Peer&) = delete;
  void matched(std::uint32_t count) {
    until([&] {
      for (std::size_t i = 0; i < writers.size(); ++i) {
        dds_publication_matched_status_t pub{};
        dds_subscription_matched_status_t sub{};
        checked(dds_get_publication_matched_status(writers[i], &pub));
        checked(dds_get_subscription_matched_status(readers[i], &sub));
        if (pub.current_count != count || sub.current_count != count) return false;
      }
      return true;
    }, "DDS discovery/close timed out");
  }
  template <typename T> void write(std::size_t index, const T& value) {
    checked(dds_write(writers[index], &value));
    checked(dds_wait_for_acks(writers[index], DDS_SECS(2)));
  }
  template <typename Wire, typename F> void observe(std::size_t index, F verify) {
    until([&] {
      void* samples[1]{}; dds_sample_info_t infos[1]{};
      const auto n = checked(dds_take(readers[index], samples, infos, 1, 1));
      if (!n) return false;
      try {
        if (infos[0].valid_data) verify(*static_cast<const Wire*>(samples[0]));
      } catch (...) { dds_return_loan(readers[index], samples, n); throw; }
      checked(dds_return_loan(readers[index], samples, n));
      return infos[0].valid_data;
    }, "native DDS observation timed out");
  }
};
template <typename T, typename Take> T delivered(void* handle, Take take) {
  T out{};
  until([&] {
    const auto code = take(handle, &out);
    require(code >= 0, lt_sensor_control_last_error());
    return code == 1;
  }, "bridge delivery timed out");
  return out;
}
template <typename T, typename Take> void none(void* handle, Take take) {
  T out{}; text(out.request_id, "unchanged");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(40);
  do {
    require(take(handle, &out) == 0 && !*lt_sensor_control_last_error(), "foreign data delivered");
    require(std::strcmp(out.request_id, "unchanged") == 0, "empty take changed output");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
}
template <typename T, typename Take> void bad_sample(void* handle, Take take, const char* reason) {
  T out{}; text(out.request_id, "unchanged");
  until([&] {
    const auto code = take(handle, &out);
    require(code != 1, "invalid incoming sample delivered");
    if (code != -1) return false;
    failed(code, reason); return true;
  }, "missing invalid incoming sample error");
  require(std::strcmp(out.request_id, "unchanged") == 0, "error take changed output");
}

void layout_and_arguments() {
  static_assert(std::is_standard_layout_v<lt_sensor_control_request>);
  static_assert(std::is_standard_layout_v<lt_sensor_control_result>);
  static_assert(sizeof(lt_sensor_control_request) == 1112);
  static_assert(sizeof(lt_sensor_control_result) == 9504);
  static_assert(offsetof(lt_sensor_control_request, deadline_s) == 1040);
  static_assert(offsetof(lt_sensor_control_request, operation) == 1048);
  static_assert(offsetof(lt_sensor_control_request, enabled) == 1052);
  static_assert(offsetof(lt_sensor_control_request, rate_hz) == 1056);
  static_assert(offsetof(lt_sensor_control_request, noise_unit) == 1080);
  static_assert(offsetof(lt_sensor_control_request, pan_deg) == 1096);
  static_assert(offsetof(lt_sensor_control_result, applied) == 1048);
  static_assert(offsetof(lt_sensor_control_result, reason) == 1052);
  static_assert(offsetof(lt_sensor_control_result, settings_json) == 1308);
  const auto bad_create = [](int d, const char* p, const char* s, int m, const char* reason) {
    Bridge out(lt_sensor_control_create(d, p, s, m), lt_sensor_control_destroy);
    require(!out && std::strstr(lt_sensor_control_last_error(), reason), "bad create accepted/wrong error");
  };
  bad_create(-1, "p", "s", 1, "domain"); bad_create(domain, "p", "s", 2, "owner_mode");
  bad_create(domain, nullptr, "s", 1, "null"); bad_create(domain, "p", nullptr, 1, "null");
  bad_create(domain, "", "s", 1, "empty"); bad_create(domain, "p", "", 1, "empty");
  const std::string long_id(256, 'x');
  bad_create(domain, long_id.c_str(), "s", 0, "bounds");
  bad_create(domain, "p", long_id.c_str(), 0, "bounds");
  auto r = request(); auto s = result();
  failed(lt_sensor_control_write_request(nullptr, &r), "arguments");
  failed(lt_sensor_control_take_request(nullptr, &r), "arguments");
  failed(lt_sensor_control_write_result(nullptr, &s), "arguments");
  failed(lt_sensor_control_take_result(nullptr, &s), "arguments");
  const std::string original = lt_sensor_control_last_error();
  lt_sensor_control_destroy(nullptr);
  require(original == lt_sensor_control_last_error(), "destroy erased original error");
  std::printf("PASS layout/arguments: request=%zu result=%zu align=%zu/%zu\n",
              sizeof(r), sizeof(s), alignof(decltype(r)), alignof(decltype(s)));
}

void roundtrip(void* owner, void* client, Peer& peer) {
  for (int op = 1; op <= 7; ++op) {
    auto r = request(); r.operation = op; r.enabled = op % 2;
    require(lt_sensor_control_write_request(client, &r) == 0, lt_sensor_control_last_error());
    same_request(delivered<lt_sensor_control_request>(owner, lt_sensor_control_take_request), r);
    peer.observe<lingtu_dds_SensorControlRequest>(0, [&](const auto& v) { same_request(v, r); });
    auto s = result(); s.applied = op % 2;
    require(lt_sensor_control_write_result(owner, &s) == 0, lt_sensor_control_last_error());
    same_result(delivered<lt_sensor_control_result>(client, lt_sensor_control_take_result), s);
    peer.observe<lingtu_dds_SensorControlResult>(1, [&](const auto& v) { same_result(v, s); });
  }
  // Identical request IDs and old generations/deadlines remain caller policy.
  auto r = request(); r.deadline_s = 0; r.model_generation = 0; r.reset_generation = 0;
  for (int i = 0; i < 2; ++i) {
    require(lt_sensor_control_write_request(client, &r) == 0, lt_sensor_control_last_error());
    same_request(delivered<lt_sensor_control_request>(owner, lt_sensor_control_take_request), r);
    peer.observe<lingtu_dds_SensorControlRequest>(0, [&](const auto& v) { same_request(v, r); });
  }
  none<lt_sensor_control_request>(owner, lt_sensor_control_take_request);
  none<lt_sensor_control_result>(client, lt_sensor_control_take_result);
  std::puts("PASS all 7 operations/exact fields/caller policy");
}
void modes_and_binding(void* owner, void* client) {
  auto r = request(); auto s = result();
  failed(lt_sensor_control_take_request(client, &r), "owner_mode");
  failed(lt_sensor_control_write_request(owner, &r), "owner_mode");
  failed(lt_sensor_control_take_result(owner, &s), "owner_mode");
  failed(lt_sensor_control_write_result(client, &s), "owner_mode");
  failed(lt_sensor_control_take_request(owner, nullptr), "arguments");
  failed(lt_sensor_control_write_request(client, nullptr), "arguments");
  failed(lt_sensor_control_take_result(client, nullptr), "arguments");
  failed(lt_sensor_control_write_result(owner, nullptr), "arguments");
  for (bool product : {true, false}) {
    r = request(); s = result();
    if (product) { text(r.product_session_id, "foreign"); text(s.product_session_id, "foreign"); }
    else { text(r.session_id, "foreign"); text(s.session_id, "foreign"); }
    failed(lt_sensor_control_write_request(client, &r), "bound");
    failed(lt_sensor_control_write_result(owner, &s), "bound");
  }
  none<lt_sensor_control_request>(owner, lt_sensor_control_take_request);
  none<lt_sensor_control_result>(client, lt_sensor_control_take_result);
  std::puts("PASS mode/null arguments/outgoing identity");
}
void foreign_samples(void* owner, void* client, Peer& peer) {
  for (bool product : {true, false}) {
    auto r = request(); auto s = result();
    if (product) { text(r.product_session_id, "foreign"); text(s.product_session_id, "foreign"); }
    else { text(r.session_id, "foreign"); text(s.session_id, "foreign"); }
    peer.write(0, wire(r)); peer.write(1, wire(s));
    none<lt_sensor_control_request>(owner, lt_sensor_control_take_request);
    none<lt_sensor_control_result>(client, lt_sensor_control_take_result);
  }
  auto r = request(); auto s = result();
  peer.write(0, wire(r)); peer.write(1, wire(s));
  same_request(delivered<lt_sensor_control_request>(owner, lt_sensor_control_take_request), r);
  same_result(delivered<lt_sensor_control_result>(client, lt_sensor_control_take_result), s);
  std::puts("PASS incoming identities do not auto-bind");
}
void bounded_take(void* owner, void* client, Peer& peer) {
  auto s = result(); text(s.session_id, "foreign");
  // CommandAck retains 64 samples: a matching 34th sample must not be reached
  // by the first poll. Acks ensure all samples arrived before the poll.
  for (int i = 0; i < 33; ++i) peer.write(1, wire(s));
  s = result(); peer.write(1, wire(s));
  lt_sensor_control_result out{}; text(out.request_id, "unchanged");
  require(lt_sensor_control_take_result(client, &out) == 0, "take exceeded 32-sample budget");
  require(std::strcmp(out.request_id, "unchanged") == 0, "bounded empty take changed output");
  same_result(delivered<lt_sensor_control_result>(client, lt_sensor_control_take_result), s);
  auto r = request(); auto rw = wire(r);
  checked(dds_dispose(peer.writers[0], &rw));
  checked(dds_wait_for_acks(peer.writers[0], DDS_SECS(2)));
  none<lt_sensor_control_request>(owner, lt_sensor_control_take_request);
  peer.write(0, rw);
  same_request(delivered<lt_sensor_control_request>(owner, lt_sensor_control_take_request), r);
  std::puts("PASS 32-sample budget/disposed sample recovery");
}
void scalar_bounds(void* owner, void* client, Peer& peer) {
  using R = lt_sensor_control_request;
  double R::* fields[] = {&R::deadline_s, &R::rate_hz, &R::noise_stddev, &R::pan_deg, &R::tilt_deg};
  for (auto field : fields) {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      auto r = request(); r.*field = invalid;
      failed(lt_sensor_control_write_request(client, &r), "finite");
      peer.write(0, wire(r));
      bad_sample<R>(owner, lt_sensor_control_take_request, "finite");
    }
  }
  for (auto field : {&R::deadline_s, &R::rate_hz, &R::noise_stddev}) {
    auto r = request(); r.*field = -1;
    failed(lt_sensor_control_write_request(client, &r), "nonnegative");
  }
  for (int invalid : {-1, 2}) {
    auto r = request(); r.enabled = invalid;
    failed(lt_sensor_control_write_request(client, &r), "enabled");
    auto s = result(); s.applied = invalid;
    failed(lt_sensor_control_write_result(owner, &s), "applied");
  }
  for (int invalid : {0, 8, -1}) {
    auto r = request(); r.operation = invalid;
    failed(lt_sensor_control_write_request(client, &r), "operation");
    peer.write(0, wire(r)); bad_sample<R>(owner, lt_sensor_control_take_request, "operation");
  }
  auto r = request(); peer.write(0, wire(r));
  same_request(delivered<R>(owner, lt_sensor_control_take_request), r);
  std::puts("PASS finite/scalar bounds and loan recovery");
}
void string_bounds(void* owner, void* client, Peer& peer) {
  auto r = request(); auto s = result();
  char* r_ids[] = {r.request_id, r.product_session_id, r.session_id, r.sensor_id};
  for (auto id : r_ids) {
    const std::string original = id;
    id[0] = '\0'; failed(lt_sensor_control_write_request(client, &r), "empty");
    std::memset(id, 'x', LT_SENSOR_CONTROL_ID_CAP);
    failed(lt_sensor_control_write_request(client, &r), "bounds");
    std::memset(id, 0, LT_SENSOR_CONTROL_ID_CAP); std::memcpy(id, original.c_str(), original.size() + 1);
  }
  char* s_ids[] = {s.request_id, s.product_session_id, s.session_id, s.sensor_id};
  for (auto id : s_ids) {
    const std::string original = id;
    id[0] = '\0'; failed(lt_sensor_control_write_result(owner, &s), "empty");
    std::memset(id, 'x', LT_SENSOR_CONTROL_ID_CAP);
    failed(lt_sensor_control_write_result(owner, &s), "bounds");
    std::memset(id, 0, LT_SENSOR_CONTROL_ID_CAP); std::memcpy(id, original.c_str(), original.size() + 1);
  }
  std::memset(r.noise_unit, 'u', sizeof(r.noise_unit));
  failed(lt_sensor_control_write_request(client, &r), "bounds"); r = request();
  std::memset(s.reason, 'r', sizeof(s.reason));
  failed(lt_sensor_control_write_result(owner, &s), "bounds"); s = result();
  std::memset(s.settings_json, 's', sizeof(s.settings_json));
  failed(lt_sensor_control_write_result(owner, &s), "bounds"); s = result();
  const std::string over_id(256, 'i'), over_unit(16, 'u'), over_reason(256, 'r'), over_settings(8192, 's');
  auto rw = wire(r); rw.sensor_id = const_cast<char*>(over_id.c_str()); peer.write(0, rw);
  bad_sample<lt_sensor_control_request>(owner, lt_sensor_control_take_request, "bounds");
  rw = wire(r); rw.noise_unit = const_cast<char*>(over_unit.c_str()); peer.write(0, rw);
  bad_sample<lt_sensor_control_request>(owner, lt_sensor_control_take_request, "bounds");
  auto sw = wire(s); sw.reason = const_cast<char*>(over_reason.c_str()); peer.write(1, sw);
  bad_sample<lt_sensor_control_result>(client, lt_sensor_control_take_result, "bounds");
  sw = wire(s); sw.settings_json = const_cast<char*>(over_settings.c_str()); peer.write(1, sw);
  bad_sample<lt_sensor_control_result>(client, lt_sensor_control_take_result, "bounds");
  text(r.request_id, std::string(255, 'q')); text(r.sensor_id, std::string(255, 'i'));
  text(r.noise_unit, std::string(15, 'u'));
  require(lt_sensor_control_write_request(client, &r) == 0, lt_sensor_control_last_error());
  same_request(delivered<lt_sensor_control_request>(owner, lt_sensor_control_take_request), r);
  text(s.request_id, std::string(255, 'q')); text(s.sensor_id, std::string(255, 'i'));
  text(s.reason, std::string(255, 'r')); text(s.settings_json, std::string(8191, 's'));
  require(lt_sensor_control_write_result(owner, &s) == 0, lt_sensor_control_last_error());
  same_result(delivered<lt_sensor_control_result>(client, lt_sensor_control_take_result), s);
  s.reason[0] = '\0'; s.settings_json[0] = '\0'; r.noise_unit[0] = '\0';
  peer.write(0, wire(r)); peer.write(1, wire(s));
  same_request(delivered<lt_sensor_control_request>(owner, lt_sensor_control_take_request), r);
  same_result(delivered<lt_sensor_control_result>(client, lt_sensor_control_take_result), s);
  const std::string max_product(255, 'p'), max_source(255, 's');
  Bridge max_owner(lt_sensor_control_create(domain, max_product.c_str(), max_source.c_str(), 1),
                   lt_sensor_control_destroy);
  require(max_owner != nullptr, lt_sensor_control_last_error());
  Bridge max_client(lt_sensor_control_create(domain, max_product.c_str(), max_source.c_str(), 0),
                    lt_sensor_control_destroy);
  require(max_client != nullptr, lt_sensor_control_last_error());
  peer.matched(3);
  text(r.product_session_id, max_product); text(r.session_id, max_source);
  text(s.product_session_id, max_product); text(s.session_id, max_source);
  require(lt_sensor_control_write_request(max_client.get(), &r) == 0, lt_sensor_control_last_error());
  same_request(delivered<lt_sensor_control_request>(max_owner.get(), lt_sensor_control_take_request), r);
  require(lt_sensor_control_write_result(max_owner.get(), &s) == 0, lt_sensor_control_last_error());
  same_result(delivered<lt_sensor_control_result>(max_client.get(), lt_sensor_control_take_result), s);
  max_owner.reset(); max_client.reset(); peer.matched(2);
  std::puts("PASS terminated/max/oversize/empty optional strings");
}
}  // namespace

int main() {
  try {
    layout_and_arguments();
    auto owner = bridge(1); auto client = bridge(0); Peer peer; peer.matched(2);
    roundtrip(owner.get(), client.get(), peer);
    modes_and_binding(owner.get(), client.get());
    foreign_samples(owner.get(), client.get(), peer);
    bounded_take(owner.get(), client.get(), peer);
    scalar_bounds(owner.get(), client.get(), peer);
    string_bounds(owner.get(), client.get(), peer);
    failed(lt_sensor_control_take_request(owner.get(), nullptr), "arguments");
    const std::string original = lt_sensor_control_last_error();
    owner.reset(); client.reset();
    require(original == lt_sensor_control_last_error(), "close erased original error");
    peer.matched(1);
    auto reopened_owner = bridge(1); auto reopened_client = bridge(0); peer.matched(2);
    reopened_owner.reset(); reopened_client.reset(); peer.matched(1);
    std::puts("PASS DDS close/reopen cleanup");
    std::printf("8 cases passed, %u checks (including DDS polling)\n", checks);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL: %s (bridge: %s)\n", e.what(), lt_sensor_control_last_error());
    return 1;
  }
}
