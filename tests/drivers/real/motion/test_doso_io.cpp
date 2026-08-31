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
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "message/cpp/navigation_command.hpp"
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

  void setFsm(brainstem::api::v1::CmsStateKind kind) {
    std::lock_guard<std::mutex> lock(mutex_);
    fsm_kind_ = kind;
  }

 private:
  void fillReady(brainstem::api::v1::ControlStatus &status) {
    status.mutable_fsm()->set_kind(fsm_kind_);
    status.set_motor_output_enabled(true);
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
  brainstem::api::v1::CmsStateKind fsm_kind_{brainstem::api::v1::CMS_STATE_KIND_STANDING};
  bool reject_{false};
  bool mismatch_ack_{false};
  bool fail_release_{false};
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

void testDriverSafetyStopUsesGlobalStopKind() {
  constexpr int kDomain = 94;
  const dds_entity_t participant =
      checked(dds_create_participant(static_cast<dds_domainid_t>(kDomain), nullptr, nullptr),
              "dds_create_participant(test_safety_stop)");
  const dds_entity_t topic = checked(
      dds_create_topic(participant, &lingtu_dds_NavigationCommandRequest_desc,
                       lingtu::message::kNavCommandRequest.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_safety_stop)");
  auto qos = lingtu::dds::make_qos(
      lingtu::dds::qos_for_topic(lingtu::message::kNavCommandRequest.dds_topic));
  const dds_entity_t reader = checked(dds_create_reader(participant, topic, qos.get(), nullptr),
                                      "dds_create_reader(test_safety_stop)");
  lingtu::driver::DdsReader driver(kDomain);

  bool observed = false;
  for (std::uint64_t attempt = 1U; attempt <= 100U && !observed; ++attempt) {
    check(driver.writeNavigationStop("brainstem_control_lost:transport_error", attempt, 123.0),
          "driver safety stop write failed");
    std::this_thread::sleep_for(10ms);
    void *raw_sample = nullptr;
    dds_sample_info_t info{};
    const dds_return_t count = dds_take(reader, &raw_sample, &info, 1, 1);
    check(count >= 0, "driver safety stop take failed");
    if (count == 0) {
      continue;
    }
    auto *message = static_cast<lingtu_dds_NavigationCommandRequest *>(raw_sample);
    if (info.valid_data) {
      check(message->kind ==
                static_cast<std::int32_t>(lingtu::message::NavigationCommandKind::Stop),
            "driver fail-safe must publish canonical global Stop");
      check(message->task_id == nullptr || std::string(message->task_id).empty(),
            "global Stop must not impersonate a task cancellation");
      check(message->client_id != nullptr &&
                std::string(message->client_id) == lingtu::driver::kBrainstemMotionPrincipal,
            "driver safety stop must preserve source identity");
      check(message->reason != nullptr &&
                std::string(message->reason) == "brainstem_control_lost:transport_error",
            "driver safety stop must preserve its fail-closed reason");
      observed = true;
    }
    checked(dds_return_loan(reader, &raw_sample, 1), "dds_return_loan(test_safety_stop)");
  }
  check(observed, "driver safety stop was not observed on DDS");
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
  stats.output_ack.record("producer-before-rejection", 42, true,
                          lingtu::driver::OutputAckState::Clock::now());
  lingtu::driver::recordFreshnessDecision(
      stats, {false, lingtu::driver::CommandFreshnessReason::DuplicateSequence});
  lingtu::driver::recordFreshnessDecision(stats,
                                          {false, lingtu::driver::CommandFreshnessReason::Expired});

  const auto output_ack = stats.output_ack.current(lingtu::driver::OutputAckState::Clock::now());
  check(!output_ack.accepted(), "freshness rejection must invalidate ACK");
  check(output_ack.producerBootId().empty(),
        "freshness rejection must clear current producer identity");
  check(output_ack.outputSequence() == 0, "freshness rejection must clear current output sequence");

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
  check(status.find("\"output_ack\": {\"producer_boot_id\": \"\", "
                    "\"output_sequence\": 0, \"accepted\": false}") != std::string::npos,
        "status must not expose stale output ACK identity after rejection");
  check(status.find("\"freshness_accepted\": 1") != std::string::npos,
        "status must count accepted freshness envelopes");
  check(status.find("\"freshness_rejected\": 2") != std::string::npos,
        "status must count rejected freshness envelopes");
  check(status.find("\"freshness_duplicate_sequence\": 1") != std::string::npos,
        "status must expose duplicate sequence rejections");
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

void testCurrentCommandRejectionIsTemporarilyCorrelatedInStatus() {
  lingtu::driver::RuntimeStats stats;
  stats.control.reason = "preempted";
  const auto now = lingtu::driver::Clock::now();
  stats.output_ack.record("producer-rejected", 73, false, now);

  lingtu::driver::Config config;
  config.status_file = "/tmp/lingtu_driver_rejection_status_" +
                       std::to_string(static_cast<unsigned long long>(boottimeNanoseconds())) +
                       ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  const lingtu::driver::AdapterDiagnostics adapter{"doso", "brainstem_grpc", "brainstem.test:13145",
                                                   "grpc", lingtu::driver::kDriverMotionPrincipal};
  lingtu::driver::writeStatus(config, core, stats, adapter, 123.0);

  std::string status = readTextFile(config.status_file);
  check(status.find("\"output_ack\": {\"producer_boot_id\": \"producer-rejected\", "
                    "\"output_sequence\": 73, \"accepted\": false}") != std::string::npos,
        "current command rejection must retain exact sequence identity");
  check(status.find("\"decision\": \"preempted\"") != std::string::npos,
        "current command rejection must retain Brainstem reason");

  stats.output_ack.expire(now + lingtu::driver::OutputAckState::kRejectedEvidenceLifetime);
  lingtu::driver::writeStatus(config, core, stats, adapter, 124.0);
  status = readTextFile(config.status_file);
  check(status.find("\"output_ack\": {\"producer_boot_id\": \"\", "
                    "\"output_sequence\": 0, \"accepted\": false}") != std::string::npos,
        "expired command rejection must not expose stale sequence identity");

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
  check(sample->frame == "body", "DDS frame must remain body");
  check(sample->output_seq == 1, "DDS output sequence must be preserved");
  check(sample->producer_boot_id == "endpoint-process-a", "DDS producer boot id must be preserved");

  const auto accepted =
      core.accept(sample->freshnessInput(), sample->frame, sample->vx, sample->vy, sample->wz);
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
      core.accept(duplicate_sample->freshnessInput(), duplicate_sample->frame, duplicate_sample->vx,
                  duplicate_sample->vy, duplicate_sample->wz);
  check(!duplicate.freshness.accepted, "duplicate output sequence must be rejected");
  check(!duplicate.action.has_value(), "duplicate command must not produce an RPC action");
  check(service.calls() == 2, "duplicate command must not reach Brainstem");

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
    testDriverSafetyStopUsesGlobalStopKind();
    testFreshnessRejectionsAreVisibleInDriverStatus();
    testDosoStandAndSitUseBrainstem();
    testCurrentCommandRejectionIsTemporarilyCorrelatedInStatus();
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
