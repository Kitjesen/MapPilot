#include "brainstem_api/cms.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <google/protobuf/empty.pb.h>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "core.hpp"
#include "dds.hpp"
#include "dds/dds.h"
#include "doso.hpp"
#include "posture.hpp"
#include "messages.h"
#include "transport/dds/qos.hpp"
#include "message/generated/topics.hpp"
#include "message/protocol/navigation.hpp"
#include "status.hpp"

namespace {

using namespace std::chrono_literals;

void check(bool value, const char *message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

void close(double actual, double expected, const char *message) {
  if (std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error(message);
  }
}

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

std::uint64_t boottimeNanoseconds() {
  timespec value{};
  check(clock_gettime(CLOCK_BOOTTIME, &value) == 0, "CLOCK_BOOTTIME must be available");
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

class WalkService final : public brainstem::api::v1::RobotControl::Service {
 public:
  grpc::Status GetControlStatus(grpc::ServerContext *, const google::protobuf::Empty *,
                                brainstem::api::v1::ControlStatus *response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    fillReady(*response);
    response->set_owner(query_owner_);
    response->set_owner_id(query_owner_ == brainstem::api::v1::CONTROL_OWNER_YUNZHUO
                               ? "physical-remote" : "");
    response->set_grpc_lease_active(false);
    response->set_ready_for_walk(false);
    return grpc::Status::OK;
  }
  grpc::Status GetProfile(grpc::ServerContext *, const google::protobuf::Empty *,
                          brainstem::api::v1::ProfileInfo *response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    response->set_current(profile_);
    response->add_available("ordinary");
    response->add_available("stair-test");
    return grpc::Status::OK;
  }

  grpc::Status GetCmsState(grpc::ServerContext *, const google::protobuf::Empty *,
                           brainstem::api::v1::CmsState *response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    response->set_kind(fsm_kind_);
    return grpc::Status::OK;
  }

  grpc::Status SwitchProfile(grpc::ServerContext *, const brainstem::api::v1::ProfileRequest *request,
                             brainstem::api::v1::ProfileInfo *response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fsm_kind_ != brainstem::api::v1::CMS_STATE_KIND_GROUNDED || motors_enabled_ ||
        query_owner_ != brainstem::api::v1::CONTROL_OWNER_NONE) {
      return {grpc::StatusCode::FAILED_PRECONDITION, "must be Grounded, disabled and unowned"};
    }
    ++switch_calls_;
    profile_ = request->name();
    response->set_current(profile_);
    return grpc::Status::OK;
  }
  grpc::Status Enable(grpc::ServerContext *, const google::protobuf::Empty *,
                       google::protobuf::Empty *) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++enable_calls_;
    motors_enabled_ = true;
    return grpc::Status::OK;
  }
  grpc::Status Disable(grpc::ServerContext *, const google::protobuf::Empty *,
                        google::protobuf::Empty *) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++disable_calls_;
    motors_enabled_ = false;
    query_owner_ = brainstem::api::v1::CONTROL_OWNER_NONE;
    return grpc::Status::OK;
  }
  grpc::Status AcquireControl(grpc::ServerContext *,
                              const brainstem::api::v1::AcquireControlRequest *request,
                              brainstem::api::v1::ControlLease *response) override {
    check(request->client_id() == lingtu::driver::kBrainstemMotionPrincipal,
          "driver client id must be explicit");
    response->set_accepted(true);
    response->set_token("test-token");
    response->set_lease_duration_ms(300);
    fillReady(*response->mutable_status());
    response->mutable_status()->set_ready_for_walk(false);
    return grpc::Status::OK;
  }

  grpc::Status RenewControlLease(grpc::ServerContext *,
                                 const brainstem::api::v1::ControlLeaseRequest *request,
                                 brainstem::api::v1::ControlLease *response) override {
    response->set_accepted(request->token() == "test-token");
    response->set_token("test-token");
    response->set_lease_duration_ms(300);
    fillReady(*response->mutable_status());
    return grpc::Status::OK;
  }

  grpc::Status WalkChecked(grpc::ServerContext *, const brainstem::api::v1::WalkRequest *request,
                           brainstem::api::v1::CommandAck *response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool accepted = request->token() == "test-token" && !reject_;
    response->set_accepted(accepted);
    response->set_sequence(mismatch_ack_ ? request->sequence() + 1 : request->sequence());
    response->set_reason(accepted ? brainstem::api::v1::COMMAND_REJECT_REASON_NONE
                                  : brainstem::api::v1::COMMAND_REJECT_REASON_PREEMPTED);
    fillReady(*response->mutable_status());
    response->mutable_status()->set_last_accepted_sequence(mismatch_ack_ ? request->sequence() + 1
                                                                         : request->sequence());
    mismatch_ack_ = false;
    response->mutable_status()->set_ready_for_walk(accepted);
    if (!accepted) {
      response->mutable_status()->set_grpc_lease_active(false);
      response->mutable_status()->set_owner(brainstem::api::v1::CONTROL_OWNER_YUNZHUO);
      response->mutable_status()->set_owner_id("");
      return grpc::Status::OK;
    }
    last_ = lingtu::driver::Velocity{request->direction().x(), request->direction().y(),
                                     request->direction().z()};
    const auto accepted_at = std::chrono::steady_clock::now();
    if (calls_ == 0) {
      first_ = last_;
      first_at_ = accepted_at;
    }
    last_at_ = accepted_at;
    ++calls_;
    return grpc::Status::OK;
  }

  grpc::Status ReleaseControl(grpc::ServerContext *,
                              const brainstem::api::v1::ControlLeaseRequest *,
                              brainstem::api::v1::ControlStatus *response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++release_calls_;
    if (fail_release_) {
      fail_release_ = false;
      return grpc::Status(grpc::StatusCode::UNAVAILABLE, "release rejected");
    }
    fillReady(*response);
    response->set_ready_for_walk(false);
    response->set_grpc_lease_active(false);
    response->set_owner(brainstem::api::v1::CONTROL_OWNER_NONE);
    response->set_owner_id("");
    return grpc::Status::OK;
  }

  grpc::Status StandUp(grpc::ServerContext *, const google::protobuf::Empty *,
                       google::protobuf::Empty *) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stand_calls_;
    return grpc::Status::OK;
  }

  grpc::Status SitDown(grpc::ServerContext *, const google::protobuf::Empty *,
                       google::protobuf::Empty *) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++sit_calls_;
    return grpc::Status::OK;
  }

  void rejectNextCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    reject_ = true;
  }

  void mismatchNextAck() {
    std::lock_guard<std::mutex> lock(mutex_);
    mismatch_ack_ = true;
  }

  void failNextRelease() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_release_ = true;
  }

  lingtu::driver::Velocity last() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_;
  }

  lingtu::driver::Velocity first() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return first_;
  }

  std::chrono::milliseconds acceptedCommandSpacing() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::chrono::duration_cast<std::chrono::milliseconds>(last_at_ - first_at_);
  }

  unsigned calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

  unsigned releaseCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return release_calls_;
  }

  unsigned standCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stand_calls_;
  }

  unsigned sitCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sit_calls_;
  }
  unsigned enableCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enable_calls_;
  }
  unsigned disableCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return disable_calls_;
  }
  unsigned switchCalls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return switch_calls_;
  }

  void setFsm(brainstem::api::v1::CmsStateKind kind) {
    std::lock_guard<std::mutex> lock(mutex_);
    fsm_kind_ = kind;
  }

  void setQueryOwner(brainstem::api::v1::ControlOwner owner) {
    std::lock_guard<std::mutex> lock(mutex_);
    query_owner_ = owner;
  }

 private:
  void fillReady(brainstem::api::v1::ControlStatus &status) {
    status.mutable_fsm()->set_kind(fsm_kind_);
    status.set_motor_output_enabled(motors_enabled_);
    status.set_owner(brainstem::api::v1::CONTROL_OWNER_GRPC);
    status.set_grpc_lease_active(true);
    status.set_lease_remaining_ms(300);
    status.set_ready_for_walk(true);
    status.set_owner_id(lingtu::driver::kBrainstemMotionPrincipal);
  }

  mutable std::mutex mutex_;
  lingtu::driver::Velocity first_;
  lingtu::driver::Velocity last_;
  std::chrono::steady_clock::time_point first_at_{};
  std::chrono::steady_clock::time_point last_at_{};
  unsigned calls_{0};
  unsigned release_calls_{0};
  unsigned stand_calls_{0};
  unsigned sit_calls_{0};
  unsigned enable_calls_{0}, disable_calls_{0}, switch_calls_{0};
  bool motors_enabled_{true};
  brainstem::api::v1::CmsStateKind fsm_kind_{brainstem::api::v1::CMS_STATE_KIND_STANDING};
  brainstem::api::v1::ControlOwner query_owner_{brainstem::api::v1::CONTROL_OWNER_YUNZHUO};
  bool reject_{false};
  bool mismatch_ack_{false};
  bool fail_release_{false};
  std::string profile_{"ordinary"};
};

class LegacyWalkOnlyService final : public brainstem::api::v1::RobotControl::Service {
 public:
  grpc::Status Walk(grpc::ServerContext *, const brainstem::api::v1::Vector3 *,
                    google::protobuf::Empty *) override {
    return grpc::Status::OK;
  }
};

class FinalVelocityWriter {
 public:
  explicit FinalVelocityWriter(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(test)");
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, &lingtu_dds_FinalVelocityCommand_desc,
                                 lingtu::message::kNavCmdVel.dds_topic.data(), nullptr, nullptr),
                "dds_create_topic(test_cmd_vel)");
    auto qos =
        lingtu::dds::make_qos(lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
    writer_ = checked(dds_create_writer(participant_, topic, qos.get(), nullptr),
                      "dds_create_writer(test_cmd_vel)");
  }

  ~FinalVelocityWriter() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  void write(const std::string &host_boot_id, const std::string &producer_boot_id,
             std::uint64_t output_seq, std::uint64_t source_boottime_ns, double vx, double vy,
             double wz) {
    lingtu_dds_FinalVelocityCommand msg{};
    msg.host_boot_id = const_cast<char *>(host_boot_id.c_str());
    msg.producer_boot_id = const_cast<char *>(producer_boot_id.c_str());
    msg.output_seq = output_seq;
    msg.source_boottime_ns = source_boottime_ns;
    msg.source_wall_ns = 123456789ULL;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.angular.z = wz;
    checked(dds_write(writer_, &msg), "dds_write(test_cmd_vel)");
  }

 private:
  dds_entity_t participant_{0};
  dds_entity_t writer_{0};
};

void testProfileDdsFreshnessAndReadback() {
  constexpr int domain = 95;
  const auto participant = checked(dds_create_participant(domain, nullptr, nullptr),
                                   "dds_create_participant(profile_test)");
  const auto request_topic = checked(dds_create_topic(participant,
      &lingtu_dds_MotionProfileRequest_desc, lingtu::message::kMotionProfileRequest.dds_topic.data(),
      nullptr, nullptr), "dds_create_topic(profile_request_test)");
  auto request_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
      lingtu::message::kMotionProfileRequest.dds_topic));
  const auto writer = checked(dds_create_writer(participant, request_topic, request_qos.get(), nullptr),
                              "dds_create_writer(profile_request_test)");
  const auto state_topic = checked(dds_create_topic(participant,
      &lingtu_dds_MotionProfileState_desc, lingtu::message::kMotionProfileState.dds_topic.data(),
      nullptr, nullptr), "dds_create_topic(profile_state_test)");
  auto state_qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
      lingtu::message::kMotionProfileState.dds_topic));
  const auto reader = checked(dds_create_reader(participant, state_topic, state_qos.get(), nullptr),
                              "dds_create_reader(profile_state_test)");
  lingtu::driver::DdsReader driver(domain);
  const auto wall = [] { return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count(); };
  const auto publish = [&](double stamp) {
    lingtu_dds_MotionProfileRequest request{};
    request.header.stamp.sec = static_cast<std::int32_t>(stamp);
    request.header.stamp.nanosec = static_cast<std::uint32_t>((stamp - request.header.stamp.sec) * 1e9);
    request.task_id = const_cast<char *>("existing-task");
    request.requirement = const_cast<char *>("stairs_up");
    checked(dds_write(writer, &request), "dds_write(profile_request_test)");
  };
  for (int i = 0; i < 10; ++i) {
    publish(wall() - 5.0);
    std::this_thread::sleep_for(10ms);
    check(!driver.takeProfileRequest(wall()), "stale profile requests may not alter the body");
  }
  std::optional<lingtu::driver::ProfileRequest> request;
  for (int i = 0; i < 100 && !request; ++i) {
    publish(wall());
    std::this_thread::sleep_for(10ms);
    request = driver.takeProfileRequest(wall());
  }
  check(request && request->task_id == "existing-task" && request->requirement == "stairs_up",
        "fresh request must reuse existing task identity");
  bool observed = false;
  for (int i = 0; i < 100 && !observed; ++i) {
    driver.writeProfileState({"existing-task", "stairs_up", "ready", "stair-test", "stair-test", ""}, wall());
    std::this_thread::sleep_for(10ms);
    void *sample = nullptr;
    dds_sample_info_t info{};
    const auto count = dds_take(reader, &sample, &info, 1, 1);
    if (count > 0) {
      if (info.valid_data) {
        const auto &state = *static_cast<lingtu_dds_MotionProfileState *>(sample);
        observed = std::string(state.task_id) == "existing-task" &&
                   std::string(state.active_profile) == "stair-test" && std::string(state.phase) == "ready";
      }
      dds_return_loan(reader, &sample, count);
    }
  }
  check(observed, "profile DDS state must preserve actual vs requested model names");
  dds_delete(participant);
}

std::string readTextFile(const std::filesystem::path &path);

void testFinalVelocityQosExpiresQueuedMotion() {
  check(lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic) ==
            lingtu::dds::QosProfile::FinalVelocityCommand,
        "final velocity must use its dedicated QoS profile");
  auto qos =
      lingtu::dds::make_qos(lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
  dds_duration_t lifespan = 0;
  check(dds_qget_lifespan(qos.get(), &lifespan), "final velocity QoS must define a lifespan");
  check(lifespan == DDS_MSECS(200), "final velocity samples must expire after 200ms");
}

void testFreshnessRejectionsAreVisibleInDriverStatus() {
  lingtu::driver::RuntimeStats stats;
  stats.capabilities = {true, true, true, true};
  stats.body.fresh = true;
  stats.body.posture = lingtu::driver::Posture::Standing;
  stats.body.velocity = {0.25, -0.1, 0.4};
  stats.body.velocity_available = true;
  stats.body.odometry_position_m = {1.25, -0.5, 0.1};
  stats.body.odometry_position_available = true;
  stats.body.height_m = 0.31;
  stats.body.height_available = true;
  stats.health.fresh = true;
  stats.health.healthy = true;
  stats.health.reason = "healthy";
  stats.control.control_assured = true;
  stats.last_output_kind = "motion_command";
  lingtu::driver::recordVelocityTracking(stats, {0.3, 0.0, 0.0}, stats.body);
  lingtu::driver::recordFreshnessDecision(stats,
                                          {true, lingtu::driver::CommandFreshnessReason::Accepted});
  stats.zero_output = true;
  lingtu::driver::recordFreshnessDecision(stats,
                                          {false, lingtu::driver::CommandFreshnessReason::Expired});


  lingtu::driver::Config config;
  config.status_file = "/tmp/lingtu_driver_freshness_status_" +
                       std::to_string(static_cast<unsigned long long>(boottimeNanoseconds())) +
                       ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  lingtu::driver::AdapterDiagnostics adapter{"doso", "brainstem_grpc", "brainstem.test:13145",
                                             "grpc", lingtu::driver::kDriverMotionPrincipal};
  adapter.state_code_available = true;
  adapter.state_code = 100;
  lingtu::driver::writeStatus(config, core, stats, adapter, 123.0);

  const std::string status = readTextFile(config.status_file);
  check(status.find("\"freshness_accepted\": 1") != std::string::npos,
        "status must count accepted freshness envelopes");
  check(status.find("\"freshness_rejected\": 1") != std::string::npos,
        "status must count rejected freshness envelopes");
  check(status.find("\"freshness_expired\": 1") != std::string::npos,
        "status must expose expired command rejections");
  check(status.find("\"last_freshness_reason\": \"expired\"") != std::string::npos,
        "status must expose the latest freshness decision");
  check(status.find("\"capabilities\": {\"stand\": true, \"sit\": true, "
                    "\"recover\": true, \"damp\": true}") != std::string::npos,
        "status must expose body-action capabilities");
  check(status.find("\"posture\": \"standing\"") != std::string::npos,
        "status must expose neutral body posture");
  check(status.find("\"vx_mps\": 0.25") != std::string::npos,
        "status must expose measured velocity in physical units");
  check(status.find("\"odometry_position_available\": true") != std::string::npos,
        "status must expose whether body odometry position is available");
  check(status.find("\"x_m\": 1.25") != std::string::npos,
        "status must expose Go2-compatible odometry position in metres");
  check(status.find("\"last_output_kind\": \"motion_command\"") != std::string::npos,
        "status must distinguish motion output from watchdog or shutdown zeroes");
  check(status.find("\"tracking_samples\": 1") != std::string::npos,
        "status must count command/observation velocity comparisons");
  check(status.find("\"mean_commanded_linear_mps\": 0.3") != std::string::npos,
        "status must retain commanded physical speed evidence");
  check(status.find("\"mean_observed_linear_mps\": 0.269258") != std::string::npos,
        "status must retain observed physical speed evidence");
  check(status.find("\"healthy\": true") != std::string::npos,
        "status must expose neutral health state");
  check(status.find("\"schema_version\": \"lingtu.driver.status.v2\"") != std::string::npos,
        "driver status must use the velocity/adapter schema");
  check(status.find("\"adapter\": {\"protocol\": \"brainstem_grpc\"") != std::string::npos,
        "transport details must live under adapter diagnostics");
  check(status.find("\"state_code\": 100") != std::string::npos,
        "vendor state code must remain adapter diagnostics");
  check(status.find("\"control_assured\": true") != std::string::npos,
        "public control status must expose adapter-neutral control assurance");
  check(status.find("\"health\": {\"fresh\": true, \"healthy\": true") != std::string::npos,
        "adapter state code must not rewrite neutral public health");
  check(status.find("\"control\": {\"protocol\"") == std::string::npos,
        "public control state must not expose adapter protocol");
  check(status.find("\"brainstem\":") == std::string::npos,
        "driver status must not duplicate vendor state");
  std::error_code remove_error;
  std::filesystem::remove(config.status_file, remove_error);
}

lingtu::driver::Config dosoConfig(int port, std::chrono::milliseconds timeout = 500ms,
                                  lingtu::driver::BrainstemTlsConfig tls = {}) {
  lingtu::driver::Config config;
  config.robot = "doso";
  config.host = "127.0.0.1";
  config.port = static_cast<std::uint16_t>(port);
  config.rpc_timeout = timeout;
  config.brainstem_tls = std::move(tls);
  return config;
}

void testDosoStandAndSitUseBrainstem() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mock Brainstem server must start");

  lingtu::driver::Doso client(dosoConfig(port));
  const auto capabilities = client.capabilities();
  check(capabilities.supports(lingtu::driver::BodyAction::Stand),
        "Doso must expose Brainstem StandUp");
  check(capabilities.supports(lingtu::driver::BodyAction::Sit),
        "Doso must expose Brainstem SitDown");
  check(!capabilities.supports(lingtu::driver::BodyAction::Damp),
        "Doso must not advertise a missing damping action");

  check(client.refresh().ok, "test lease must be acquired before SitDown");
  check(client.state().posture == lingtu::driver::Posture::Standing,
        "Brainstem Standing must map to the common BodyState");
  const auto sit = client.act(lingtu::driver::BodyAction::Sit);
  check(sit.ok && sit.accepted, "Doso SitDown must be accepted");
  check(service.sitCalls() == 1, "Doso SitDown must call Brainstem once");
  check(service.releaseCalls() == 1, "SitDown must stop and release locomotion first");
  check(client.state().posture == lingtu::driver::Posture::Transitioning,
        "accepted posture action must expose a transitioning BodyState");

  const auto stand = client.act(lingtu::driver::BodyAction::Stand);
  check(stand.ok && stand.accepted, "Doso StandUp must be accepted");
  check(service.standCalls() == 1, "Doso StandUp must call Brainstem once");

  const auto result = client.act(lingtu::driver::BodyAction::Damp);
  check(!result.ok, "unsupported body action must not succeed");
  check(!result.accepted, "unsupported body action must not be accepted");
  check(result.calls == 0, "unsupported body action must not issue vendor I/O");
  check(result.state.reason == "unsupported_body_action:damp",
        "unsupported action reason must be explicit");

  service.setFsm(brainstem::api::v1::CMS_STATE_KIND_GROUNDED);
  check(!client.refresh().ok, "grounded Doso must not be locomotion-ready");
  check(client.state().posture == lingtu::driver::Posture::Sitting,
        "Brainstem Grounded must map to the common sitting posture");
  check(client.stop().confirmsStop(), "grounded test lease must stop cleanly");

  server->Shutdown();
  server->Wait();
}

void testPostureDdsToBrainstem() {
  WalkService service;
  service.setFsm(brainstem::api::v1::CMS_STATE_KIND_GROUNDED);
  service.setQueryOwner(brainstem::api::v1::CONTROL_OWNER_NONE);
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  auto server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "posture mock server must start");
  constexpr int domain = 96;
  const auto participant = checked(dds_create_participant(domain, nullptr, nullptr), "posture participant");
  const auto request_topic = checked(dds_create_topic(participant,
      &lingtu_dds_MotionPostureRequest_desc, lingtu::message::kMotionPostureRequest.dds_topic.data(),
      nullptr, nullptr), "posture request topic");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(
      lingtu::message::kMotionPostureRequest.dds_topic));
  const auto writer = checked(dds_create_writer(participant, request_topic, qos.get(), nullptr),
                              "posture request writer");
  const auto ack_topic = checked(dds_create_topic(participant, &lingtu_dds_MotionPostureAck_desc,
      lingtu::message::kMotionPostureAck.dds_topic.data(), nullptr, nullptr), "posture ACK topic");
  const auto ack_reader = checked(dds_create_reader(participant, ack_topic, qos.get(), nullptr),
                                  "posture ACK reader");
  lingtu::driver::DdsReader driver(domain);
  lingtu::driver::Doso body(dosoConfig(port));
  lingtu::driver::PostureControl posture;
  lingtu_dds_MotionPostureRequest wire{};
  wire.host_boot_id = const_cast<char *>(driver.hostBootId().c_str());
  wire.producer_boot_id = const_cast<char *>("nav-boot-posture");
  wire.client_id = const_cast<char *>("sdk-posture");
  wire.request_id = const_cast<char *>("stand-from-grounded");
  wire.action = static_cast<int>(lingtu::message::NavigationCommandKind::Stand);
  wire.profile_name = const_cast<char *>("");
  for (int attempt = 0; attempt < 10; ++attempt) {
    wire.source_boottime_ns = boottimeNanoseconds() - 2000000000ULL;
    checked(dds_write(writer, &wire), "expired posture write");
    std::this_thread::sleep_for(10ms);
    check(!driver.takePostureRequest(), "expired posture request must not reach body action");
  }
  wire.host_boot_id = const_cast<char *>("previous-host-boot");
  wire.source_boottime_ns = boottimeNanoseconds();
  checked(dds_write(writer, &wire), "wrong boot posture write");
  std::this_thread::sleep_for(20ms);
  check(!driver.takePostureRequest(), "posture request from another host boot must be rejected");
  wire.host_boot_id = const_cast<char *>(driver.hostBootId().c_str());
  std::optional<lingtu::message::PostureRequest> request;
  for (int attempt = 0; attempt < 100 && !request; ++attempt) {
    wire.source_boottime_ns = boottimeNanoseconds();
    checked(dds_write(writer, &wire), "posture request write");
    std::this_thread::sleep_for(10ms);
    request = driver.takePostureRequest();
  }
  check(request && request->producer_boot_id == "nav-boot-posture" &&
        request->client_id == "sdk-posture" && request->request_id == "stand-from-grounded",
        "posture DDS must preserve all request identities");
  const auto reply = posture.request(*request, body, "", false, false);
  check(reply.ack.accepted && service.standCalls() == 1 && service.calls() == 0,
        "Grounded DDS stand must reach StandUp without walking commands");
  bool observed = false;
  for (int attempt = 0; attempt < 100 && !observed; ++attempt) {
    check(driver.writePostureAck(reply.ack, 123.0), "posture ACK write");
    std::this_thread::sleep_for(10ms);
    void *sample = nullptr;
    dds_sample_info_t info{};
    const auto count = dds_take(ack_reader, &sample, &info, 1, 1);
    check(count >= 0, "posture ACK take");
    if (count == 0) continue;
    const auto &ack = *static_cast<lingtu_dds_MotionPostureAck *>(sample);
    observed = info.valid_data && ack.accepted && ack.final_result && ack.phase == 0 &&
        ack.action == wire.action && std::string(ack.request_id) == wire.request_id &&
        std::string(ack.client_id) == wire.client_id &&
        std::string(ack.producer_boot_id) == wire.producer_boot_id;
    checked(dds_return_loan(ack_reader, &sample, 1), "posture ACK loan");
  }
  check(observed, "driver ACK must carry the original posture identities");
  check(posture.request(*request, body, "", false, false).ack.accepted && service.standCalls() == 1,
        "replayed DDS request must reuse final ACK without another StandUp");
  auto invoke = [&](lingtu::message::NavigationCommandKind action, const char *id, const char *profile) {
    wire.action = static_cast<int>(action);
    wire.request_id = const_cast<char *>(id);
    wire.profile_name = const_cast<char *>(profile);
    std::optional<lingtu::message::PostureRequest> received;
    for (int attempt = 0; attempt < 100 && !received; ++attempt) {
      wire.source_boottime_ns = boottimeNanoseconds();
      checked(dds_write(writer, &wire), "motor/profile request write");
      std::this_thread::sleep_for(10ms);
      const auto candidate = driver.takePostureRequest();
      if (candidate && candidate->request_id == id) received = candidate;
    }
    check(received && received->profile_name == profile, "DDS preserves explicit profile target");
    const auto result = posture.request(*received, body, "", false, false);
    check(result.ack.accepted, "DDS motor/profile command is accepted by actual adapter");
    bool matched = false;
    for (int attempt = 0; attempt < 100 && !matched; ++attempt) {
      check(driver.writePostureAck(result.ack, 124.0), "motor/profile ACK write");
      std::this_thread::sleep_for(10ms);
      void *sample = nullptr;
      dds_sample_info_t info{};
      const auto count = dds_take(ack_reader, &sample, &info, 1, 1);
      check(count >= 0, "motor/profile ACK take");
      if (!count) continue;
      const auto &ack = *static_cast<lingtu_dds_MotionPostureAck *>(sample);
      matched = info.valid_data && ack.accepted && ack.action == wire.action &&
          std::string(ack.request_id) == id && std::string(ack.profile_name) == profile;
      checked(dds_return_loan(ack_reader, &sample, 1), "motor/profile ACK loan");
    }
    check(matched, "DDS ACK preserves motor action and target profile");
    check(posture.request(*received, body, "", false, false).ack.accepted,
          "replayed motor/profile request uses cached ACK");
  };
  invoke(lingtu::message::NavigationCommandKind::DisableMotors, "disable-from-sdk", "");
  invoke(lingtu::message::NavigationCommandKind::SwitchProfile, "profile-from-sdk", "stair-test");
  invoke(lingtu::message::NavigationCommandKind::EnableMotors, "enable-from-sdk", "");
  check(service.disableCalls() == 1 && service.switchCalls() == 1 && service.enableCalls() == 1 &&
      service.standCalls() == 1 && service.sitCalls() == 0 && service.calls() == 0,
      "public motor/profile commands execute once without automatic posture or walking");
  dds_delete(participant);
  server->Shutdown();
  server->Wait();
}

void testReadOnlyControlPreservesOwner() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  auto server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "inspection mock server must start");
  lingtu::driver::Doso client(dosoConfig(port));
  const auto observed = client.inspectControl();
  check(observed.ok && observed.transport_ok && !observed.accepted,
        "control query must remain read-only");
  check(observed.state.owner == "yunzhuo" && observed.state.owner_id == "physical-remote",
        "control query must preserve actual source owner");
  check(service.calls() == 0 && service.releaseCalls() == 0 &&
        service.standCalls() == 0 && service.sitCalls() == 0,
        "inspection must not send walking or posture commands");
  server->Shutdown();
  server->Wait();
}

void testDosoProfileUsesIndependentActualReadback() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  auto server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "profile mock Brainstem server must start");
  lingtu::driver::Doso client(dosoConfig(port));
  const auto initial = client.readProfile();
  check(initial.supported && initial.fresh && initial.requires_grounded && initial.requires_disabled &&
        initial.motors_enabled == true && !initial.control_free && !initial.grounded &&
        initial.current == "ordinary" && initial.available.size() == 2,
        "DOSO profile state must report actual controller and its Grounded prerequisite");
  check(!client.switchProfile("stair-test").ok, "standing profile switch must preserve server rejection");
  service.setFsm(brainstem::api::v1::CMS_STATE_KIND_GROUNDED);
  check(client.readProfile().grounded, "Grounded must come from actual body query");
  check(!client.switchProfile("stair-test").ok, "Grounded still rejects profile switching with enabled gate");
  service.setQueryOwner(brainstem::api::v1::CONTROL_OWNER_NONE);
  const auto disabled = client.motorPower(false);
  check(disabled.ok && disabled.accepted && !disabled.state.ready, "adapter exposes real Disable RPC");
  const auto grounded_disabled = client.readProfile();
  check(grounded_disabled.motors_enabled == false && grounded_disabled.control_free,
        "profile state independently reads disabled gate and free control");
  const auto accepted = client.switchProfile("stair-test");
  check(accepted.ok && accepted.accepted && !accepted.state.ready,
        "switch ACK must not claim motion readiness");
  check(client.readProfile().current == "stair-test", "active profile must come from GetProfile");
  check(client.motorPower(true).ok && client.readProfile().motors_enabled == true,
        "adapter exposes Enable RPC with independent gate readback");
  check(service.disableCalls() == 1 && service.enableCalls() == 1 && service.switchCalls() == 1,
        "adapter forwards one hardware call for each explicit operation");
  server->Shutdown();
  server->Wait();
}

void testZeroOutputStateInStatus() {
  lingtu::driver::RuntimeStats stats;
  stats.control.reason = "preempted";
  stats.zero_output = true;

  lingtu::driver::Config config;
  config.status_file = "/tmp/lingtu_driver_rejection_status_" +
                       std::to_string(static_cast<unsigned long long>(boottimeNanoseconds())) +
                       ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  const lingtu::driver::AdapterDiagnostics adapter{"doso", "brainstem_grpc", "brainstem.test:13145",
                                                   "grpc", lingtu::driver::kDriverMotionPrincipal};
  lingtu::driver::writeStatus(config, core, stats, adapter, 123.0);

  std::string status = readTextFile(config.status_file);
  check(status.find("\"zero_output\": true") != std::string::npos, "zero output serialized");
  check(status.find("\"decision\": \"preempted\"") != std::string::npos,
        "current command rejection must retain Brainstem reason");

  stats.zero_output = false;
  lingtu::driver::writeStatus(config, core, stats, adapter, 124.0);
  status = readTextFile(config.status_file);
  check(status.find("\"zero_output\": false") != std::string::npos, "unknown output serialized");

  std::error_code remove_error;
  std::filesystem::remove(config.status_file, remove_error);
}

void testDdsToBrainstemPath() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mock Brainstem server must start");

  constexpr int kDomain = 91;
  lingtu::driver::DdsReader reader(kDomain);
  FinalVelocityWriter writer(kDomain);
  lingtu::driver::Doso client(dosoConfig(port));
  lingtu::driver::Core core({0.5, 2.0, 200ms}, reader.hostBootId());

  std::optional<lingtu::driver::TwistSample> sample;
  for (int attempt = 0; attempt < 100 && !sample; ++attempt) {
    writer.write(reader.hostBootId(), "endpoint-process-a", 1, boottimeNanoseconds(), 0.25, -0.5,
                 1.0);
    std::this_thread::sleep_for(10ms);
    sample = reader.takeLatest().latest;
  }
  check(sample.has_value(), "typed DDS reader must receive cmd_vel");
  check(sample->producer_boot_id == "endpoint-process-a", "DDS producer boot id must be preserved");

  const auto accepted =
      core.accept(sample->freshnessInput(), "body", sample->vx, sample->vy, sample->wz,
                  sample->received_at);
  check(accepted.freshness.accepted, "driver core must admit fresh DDS metadata");
  check(accepted.action.has_value(), "driver core must accept DDS command");
  const auto lease = client.refresh();
  check(lease.ok, "Brainstem control lease must be acquired");
  check(lease.state.ready, "Brainstem state must prove motion readiness");
  check(lease.state.initial_zero_acknowledged,
        "driver must not become ready before Brainstem acknowledges initial zero");
  check(service.calls() == 1, "lease acquisition must send exactly one checked zero");
  const auto initial = service.first();
  close(initial.vx_mps, 0.0, "initial checked zero x");
  close(initial.vy_mps, 0.0, "initial checked zero y");
  close(initial.yaw_rps, 0.0, "initial checked zero z");
  const auto result = client.move(accepted.action->velocity);
  check(result.ok, "Brainstem Walk RPC must succeed");
  check(result.accepted, "Brainstem must explicitly accept the command");
  check(service.calls() == 2, "mock Brainstem must receive zero before motion");
  check(service.acceptedCommandSpacing() >= 18ms,
        "checked commands must respect Brainstem's 18ms rate limit");
  const auto received = service.last();
  close(received.vx_mps, 0.25, "forward velocity must remain m/s");
  close(received.vy_mps, -0.5, "lateral velocity must remain m/s");
  close(received.yaw_rps, 1.0, "yaw velocity must remain rad/s");

  std::optional<lingtu::driver::TwistSample> duplicate_sample;
  for (int attempt = 0; attempt < 100 && !duplicate_sample; ++attempt) {
    writer.write(reader.hostBootId(), "endpoint-process-a", 1, boottimeNanoseconds(), 0.4, 0.0,
                 0.0);
    std::this_thread::sleep_for(10ms);
    duplicate_sample = reader.takeLatest().latest;
  }
  check(duplicate_sample.has_value(), "duplicate DDS sample must be observable");
  const auto duplicate =
      core.accept(duplicate_sample->freshnessInput(), "body", duplicate_sample->vx,
                  duplicate_sample->vy, duplicate_sample->wz, duplicate_sample->received_at);
  check(duplicate.freshness.accepted, "latest input must not be rejected by a repeated sequence");
  check(duplicate.action.has_value(), "latest input must produce the current velocity action");
  check(client.move(duplicate.action->velocity).accepted, "latest command did not reach Brainstem");
  check(service.calls() == 3, "latest command must be forwarded once");

  service.mismatchNextAck();
  const auto mismatched = client.move({});
  check(!mismatched.ok, "mismatched ACK sequence must fail closed");
  check(mismatched.transport_ok, "ACK mismatch must remain distinguishable from transport loss");
  check(!mismatched.accepted, "mismatched ACK must not be accepted");
  check(mismatched.state.reason == "ack_sequence_mismatch", "ACK mismatch reason must be explicit");

  const auto reacquired = client.refresh();
  check(reacquired.ok, "driver must reacquire control after ACK mismatch");

  service.rejectNextCommand();
  const auto rejected = client.move({});
  check(!rejected.ok, "preempted command must fail closed");
  check(rejected.transport_ok, "preemption must remain distinguishable from transport loss");
  check(!rejected.accepted, "preempted command must carry an explicit rejection");
  check(rejected.state.reason == "preempted", "rejection reason must be preserved");

  server->Shutdown();
  server->Wait();
}

void testStopAndReleaseRequiresAnAcknowledgedZeroAndRelease() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mock Brainstem server must start");

  lingtu::driver::Doso client(dosoConfig(port));
  check(client.refresh().ok, "test lease must be acquired");
  check(client.move({0.4, 0.0, 0.0}).ok, "test motion must be accepted");

  const auto result = client.stop();
  check(result.ok, "stop and release must be acknowledged");
  check(result.transport_ok, "stop and release transport must succeed");
  check(result.accepted, "stop and release must be explicitly accepted");
  check(result.confirmsStop(), "Body::stop must carry the common confirmation semantics");
  check(result.state.reason == "stop_confirmed", "stop confirmation reason must be explicit");
  check(service.releaseCalls() == 1, "release RPC must be observed");
  const auto stopped = service.last();
  close(stopped.vx_mps, 0.0, "release stop x");
  close(stopped.vy_mps, 0.0, "release stop y");
  close(stopped.yaw_rps, 0.0, "release stop z");

  server->Shutdown();
  server->Wait();
}

void testStopAndReleaseSurfacesReleaseTransportFailure() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mock Brainstem server must start");

  lingtu::driver::Doso client(dosoConfig(port));
  check(client.refresh().ok, "test lease must be acquired");
  check(client.move({0.4, 0.0, 0.0}).ok, "test motion must be accepted");
  service.failNextRelease();

  const auto result = client.stop();
  check(!result.ok, "failed release must not be reported as released");
  check(!result.transport_ok, "release transport failure must remain visible");
  check(!result.accepted, "failed release must not be accepted");
  check(result.state.reason == "release_transport_error",
        "release failure reason must be explicit");
  check(result.error.find("ReleaseControl") != std::string::npos,
        "release failure must identify the RPC");
  const auto stopped = service.last();
  close(stopped.vx_mps, 0.0, "failed release still sends stop x");
  close(stopped.vy_mps, 0.0, "failed release still sends stop y");
  close(stopped.yaw_rps, 0.0, "failed release still sends stop z");

  server->Shutdown();
  server->Wait();
}

void testLegacyWalkOnlyServerFailsClosedWithExplicitProtocolReason() {
  LegacyWalkOnlyService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "legacy mock server must start");

  lingtu::driver::Doso client(dosoConfig(port));
  const auto result = client.refresh();
  check(!result.ok, "Walk-only Brainstem must not become ready");
  check(result.transport_ok, "reachable incompatible server is not transport loss");
  check(!result.accepted, "Walk-only Brainstem must not acquire a lease");
  check(result.state.reason == "protocol_incompatible",
        "missing lease RPC must report protocol incompatibility");
  check(result.error.find("AcquireControl") != std::string::npos,
        "protocol error must name the missing RPC");

  server->Shutdown();
  server->Wait();
}

std::string readTextFile(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void testMutualTlsBrainstemPath() {
  char directory_template[] = "/tmp/lingtu_driver_tls_XXXXXX";
  const char *created = mkdtemp(directory_template);
  check(created != nullptr, "TLS fixture directory must be created");
  const std::filesystem::path directory(created);
  const auto config = directory / "openssl.cnf";
  const auto certificate = directory / "peer.crt";
  const auto private_key = directory / "peer.key";
  {
    std::ofstream out(config);
    out << "[req]\ndistinguished_name=dn\n[dn]\n";
  }
  const std::string command =
      "openssl req -x509 -newkey rsa:2048 -nodes -days 1 "
      "-config " +
      config.string() +
      " -subj /CN=localhost "
      "-addext subjectAltName=DNS:localhost,IP:127.0.0.1 "
      "-keyout " +
      private_key.string() + " -out " + certificate.string() + " >/dev/null 2>&1";
  check(std::system(command.c_str()) == 0, "openssl TLS fixture generation");

  WalkService service;
  grpc::SslServerCredentialsOptions options(
      GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY);
  options.pem_root_certs = readTextFile(certificate);
  options.pem_key_cert_pairs.push_back({readTextFile(private_key), readTextFile(certificate)});
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort("127.0.0.1:0", grpc::SslServerCredentials(options), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mTLS mock Brainstem server must start");

  lingtu::driver::BrainstemTlsConfig tls;
  tls.ca_file = certificate.string();
  tls.certificate_file = certificate.string();
  tls.private_key_file = private_key.string();
  lingtu::driver::Doso client(dosoConfig(port, 1000ms, tls));
  const auto result = client.refresh();
  check(result.ok, "mTLS Brainstem lease and initial zero must succeed");
  check(result.state.initial_zero_acknowledged, "mTLS path must preserve initial zero ACK gate");
  check(service.calls() == 1, "mTLS acquisition must deliver checked zero");

  check(client.stop().ok, "mTLS stop and release must succeed");
  server->Shutdown();
  server->Wait();
  std::error_code remove_error;
  std::filesystem::remove_all(directory, remove_error);
}

}  // namespace

int main() {
  try {
    testFinalVelocityQosExpiresQueuedMotion();
    testProfileDdsFreshnessAndReadback();
    testFreshnessRejectionsAreVisibleInDriverStatus();
    testDosoStandAndSitUseBrainstem();
    testReadOnlyControlPreservesOwner();
    testPostureDdsToBrainstem();
    testDosoProfileUsesIndependentActualReadback();
    testZeroOutputStateInStatus();
    testDdsToBrainstemPath();
    testStopAndReleaseRequiresAnAcknowledgedZeroAndRelease();
    testStopAndReleaseSurfacesReleaseTransportFailure();
    testLegacyWalkOnlyServerFailsClosedWithExplicitProtocolReason();
    testMutualTlsBrainstemPath();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_driver_io: FAIL: %s\n", exc.what());
    return 1;
  }
}
