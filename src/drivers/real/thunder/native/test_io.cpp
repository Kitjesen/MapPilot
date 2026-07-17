#include "brainstem.hpp"
#include "core.hpp"
#include "dds.hpp"
#include "status.hpp"

#include "brainstem.grpc.pb.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <google/protobuf/empty.pb.h>
#include <grpcpp/grpcpp.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

void check(bool value, const char* message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

void close(double actual, double expected, const char* message) {
  if (std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error(message);
  }
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

std::uint64_t boottimeNanoseconds() {
  timespec value{};
  check(
      clock_gettime(CLOCK_BOOTTIME, &value) == 0,
      "CLOCK_BOOTTIME must be available");
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
      static_cast<std::uint64_t>(value.tv_nsec);
}

class WalkService final : public brainstem::api::v1::RobotControl::Service {
 public:
  grpc::Status AcquireControl(
      grpc::ServerContext*,
      const brainstem::api::v1::AcquireControlRequest* request,
      brainstem::api::v1::ControlLease* response) override {
    check(request->client_id() == "lingtu-driver", "driver client id must be explicit");
    response->set_accepted(true);
    response->set_token("test-token");
    response->set_lease_duration_ms(300);
    fillReady(*response->mutable_status());
    return grpc::Status::OK;
  }

  grpc::Status RenewControlLease(
      grpc::ServerContext*,
      const brainstem::api::v1::ControlLeaseRequest* request,
      brainstem::api::v1::ControlLease* response) override {
    response->set_accepted(request->token() == "test-token");
    response->set_token("test-token");
    response->set_lease_duration_ms(300);
    fillReady(*response->mutable_status());
    return grpc::Status::OK;
  }

  grpc::Status WalkChecked(
      grpc::ServerContext*,
      const brainstem::api::v1::WalkRequest* request,
      brainstem::api::v1::CommandAck* response) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool accepted = request->token() == "test-token" && !reject_;
    response->set_accepted(accepted);
    response->set_sequence(
        mismatch_ack_ ? request->sequence() + 1 : request->sequence());
    response->set_reason(
        accepted
            ? brainstem::api::v1::COMMAND_REJECT_REASON_NONE
            : brainstem::api::v1::COMMAND_REJECT_REASON_PREEMPTED);
    fillReady(*response->mutable_status());
    response->mutable_status()->set_last_accepted_sequence(
        mismatch_ack_ ? request->sequence() + 1 : request->sequence());
    mismatch_ack_ = false;
    response->mutable_status()->set_ready_for_walk(accepted);
    if (!accepted) {
      response->mutable_status()->set_grpc_lease_active(false);
      response->mutable_status()->set_owner(
          brainstem::api::v1::CONTROL_OWNER_YUNZHUO);
      response->mutable_status()->set_owner_id("");
      return grpc::Status::OK;
    }
    last_ = lingtu::driver::Walk{
        request->direction().x(),
        request->direction().y(),
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

  grpc::Status ReleaseControl(
      grpc::ServerContext*,
      const brainstem::api::v1::ControlLeaseRequest*,
      brainstem::api::v1::ControlStatus* response) override {
    fillReady(*response);
    response->set_ready_for_walk(false);
    response->set_grpc_lease_active(false);
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

  lingtu::driver::Walk last() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_;
  }

  lingtu::driver::Walk first() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return first_;
  }

  std::chrono::milliseconds acceptedCommandSpacing() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        last_at_ - first_at_);
  }

  unsigned calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

 private:
  static void fillReady(brainstem::api::v1::ControlStatus& status) {
    status.mutable_fsm()->set_kind(
        brainstem::api::v1::CMS_STATE_KIND_STANDING);
    status.set_motor_output_enabled(true);
    status.set_owner(brainstem::api::v1::CONTROL_OWNER_GRPC);
    status.set_grpc_lease_active(true);
    status.set_lease_remaining_ms(300);
    status.set_ready_for_walk(true);
    status.set_owner_id("lingtu-driver");
  }

  mutable std::mutex mutex_;
  lingtu::driver::Walk first_;
  lingtu::driver::Walk last_;
  std::chrono::steady_clock::time_point first_at_{};
  std::chrono::steady_clock::time_point last_at_{};
  unsigned calls_{0};
  bool reject_{false};
  bool mismatch_ack_{false};
};

class LegacyWalkOnlyService final
    : public brainstem::api::v1::RobotControl::Service {
 public:
  grpc::Status Walk(
      grpc::ServerContext*,
      const brainstem::api::v1::Vector3*,
      google::protobuf::Empty*) override {
    return grpc::Status::OK;
  }
};

class FinalVelocityWriter {
 public:
  explicit FinalVelocityWriter(int domain_id) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(test)");
    const dds_entity_t topic = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_FinalVelocityCommand_desc,
            lingtu::message::kNavCmdVel.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(test_cmd_vel)");
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
    writer_ = checked(
        dds_create_writer(participant_, topic, qos.get(), nullptr),
        "dds_create_writer(test_cmd_vel)");
  }

  ~FinalVelocityWriter() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  void write(
      const std::string& host_boot_id,
      const std::string& producer_boot_id,
      std::uint64_t output_seq,
      std::uint64_t source_boottime_ns,
      double vx,
      double vy,
      double wz) {
    lingtu_dds_FinalVelocityCommand msg{};
    msg.host_boot_id = const_cast<char*>(host_boot_id.c_str());
    msg.producer_boot_id = const_cast<char*>(producer_boot_id.c_str());
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

std::string readTextFile(const std::filesystem::path& path);

void testFinalVelocityQosExpiresQueuedMotion() {
  check(
      lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic) ==
          lingtu::dds::QosProfile::FinalVelocityCommand,
      "final velocity must use its dedicated QoS profile");
  auto qos = lingtu::dds::make_qos(
      lingtu::dds::qos_for_topic(lingtu::message::kNavCmdVel.dds_topic));
  dds_duration_t lifespan = 0;
  check(
      dds_qget_lifespan(qos.get(), &lifespan),
      "final velocity QoS must define a lifespan");
  check(
      lifespan == DDS_MSECS(200),
      "final velocity samples must expire after 200ms");
}

void testFreshnessRejectionsAreVisibleInDriverStatus() {
  lingtu::driver::RuntimeStats stats;
  lingtu::driver::recordFreshnessDecision(
      stats,
      {true, lingtu::driver::CommandFreshnessReason::Accepted});
  lingtu::driver::recordFreshnessDecision(
      stats,
      {false, lingtu::driver::CommandFreshnessReason::DuplicateSequence});
  lingtu::driver::recordFreshnessDecision(
      stats,
      {false, lingtu::driver::CommandFreshnessReason::Expired});

  lingtu::driver::Config config;
  config.status_file =
      "/tmp/lingtu_driver_freshness_status_" +
      std::to_string(static_cast<unsigned long long>(boottimeNanoseconds())) +
      ".json";
  lingtu::driver::Core core(config.limits, "host-boot");
  lingtu::driver::writeStatus(
      config, core, stats, "brainstem.test:13145", 123.0);

  const std::string status = readTextFile(config.status_file);
  check(
      status.find("\"freshness_accepted\": 1") != std::string::npos,
      "status must count accepted freshness envelopes");
  check(
      status.find("\"freshness_rejected\": 2") != std::string::npos,
      "status must count rejected freshness envelopes");
  check(
      status.find("\"freshness_duplicate_sequence\": 1") !=
          std::string::npos,
      "status must expose duplicate sequence rejections");
  check(
      status.find("\"freshness_expired\": 1") != std::string::npos,
      "status must expose expired command rejections");
  check(
      status.find("\"last_freshness_reason\": \"expired\"") !=
          std::string::npos,
      "status must expose the latest freshness decision");
  std::error_code remove_error;
  std::filesystem::remove(config.status_file, remove_error);
}

void testDdsToBrainstemPath() {
  WalkService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort(
      "127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mock Brainstem server must start");

  constexpr int kDomain = 91;
  lingtu::driver::DdsReader reader(kDomain);
  FinalVelocityWriter writer(kDomain);
  lingtu::driver::Brainstem client("127.0.0.1", static_cast<unsigned>(port), 500ms);
  lingtu::driver::Core core({0.5, 2.0, 200ms}, reader.hostBootId());

  std::optional<lingtu::driver::TwistSample> sample;
  for (int attempt = 0; attempt < 100 && !sample; ++attempt) {
    writer.write(
        reader.hostBootId(),
        "endpoint-process-a",
        1,
        boottimeNanoseconds(),
        0.25,
        -0.5,
        1.0);
    std::this_thread::sleep_for(10ms);
    sample = reader.takeLatest().latest;
  }
  check(sample.has_value(), "typed DDS reader must receive cmd_vel");
  check(sample->frame == "body", "DDS frame must remain body");
  check(sample->output_seq == 1, "DDS output sequence must be preserved");
  check(
      sample->producer_boot_id == "endpoint-process-a",
      "DDS producer boot id must be preserved");

  const auto accepted = core.accept(
      sample->freshnessInput(),
      sample->frame,
      sample->vx,
      sample->vy,
      sample->wz);
  check(accepted.freshness.accepted, "driver core must admit fresh DDS metadata");
  check(accepted.action.has_value(), "driver core must accept DDS command");
  const auto lease = client.refreshControl();
  check(lease.ok, "Brainstem control lease must be acquired");
  check(lease.state.ready, "Brainstem state must prove motion readiness");
  check(
      lease.state.initial_zero_acknowledged,
      "driver must not become ready before Brainstem acknowledges initial zero");
  check(service.calls() == 1, "lease acquisition must send exactly one checked zero");
  const auto initial = service.first();
  close(initial.x, 0.0, "initial checked zero x");
  close(initial.y, 0.0, "initial checked zero y");
  close(initial.z, 0.0, "initial checked zero z");
  const auto result = client.send(accepted.action->walk);
  check(result.ok, "Brainstem Walk RPC must succeed");
  check(result.accepted, "Brainstem must explicitly accept the command");
  check(service.calls() == 2, "mock Brainstem must receive zero before motion");
  check(
      service.acceptedCommandSpacing() >= 18ms,
      "checked commands must respect Brainstem's 18ms rate limit");
  const auto received = service.last();
  close(received.x, 0.5, "Walk x normalization");
  close(received.y, -1.0, "Walk y normalization");
  close(received.z, 0.5, "Walk z normalization");

  std::optional<lingtu::driver::TwistSample> duplicate_sample;
  for (int attempt = 0; attempt < 100 && !duplicate_sample; ++attempt) {
    writer.write(
        reader.hostBootId(),
        "endpoint-process-a",
        1,
        boottimeNanoseconds(),
        0.4,
        0.0,
        0.0);
    std::this_thread::sleep_for(10ms);
    duplicate_sample = reader.takeLatest().latest;
  }
  check(duplicate_sample.has_value(), "duplicate DDS sample must be observable");
  const auto duplicate = core.accept(
      duplicate_sample->freshnessInput(),
      duplicate_sample->frame,
      duplicate_sample->vx,
      duplicate_sample->vy,
      duplicate_sample->wz);
  check(!duplicate.freshness.accepted, "duplicate output sequence must be rejected");
  check(!duplicate.action.has_value(), "duplicate command must not produce an RPC action");
  check(service.calls() == 2, "duplicate command must not reach Brainstem");

  service.mismatchNextAck();
  const auto mismatched = client.send({});
  check(!mismatched.ok, "mismatched ACK sequence must fail closed");
  check(
      mismatched.transport_ok,
      "ACK mismatch must remain distinguishable from transport loss");
  check(!mismatched.accepted, "mismatched ACK must not be accepted");
  check(
      mismatched.state.reason == "ack_sequence_mismatch",
      "ACK mismatch reason must be explicit");

  const auto reacquired = client.refreshControl();
  check(reacquired.ok, "driver must reacquire control after ACK mismatch");

  service.rejectNextCommand();
  const auto rejected = client.send({});
  check(!rejected.ok, "preempted command must fail closed");
  check(rejected.transport_ok, "preemption must remain distinguishable from transport loss");
  check(!rejected.accepted, "preempted command must carry an explicit rejection");
  check(rejected.state.reason == "preempted", "rejection reason must be preserved");

  server->Shutdown();
  server->Wait();
}

void testLegacyWalkOnlyServerFailsClosedWithExplicitProtocolReason() {
  LegacyWalkOnlyService service;
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort(
      "127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "legacy mock server must start");

  lingtu::driver::Brainstem client(
      "127.0.0.1", static_cast<unsigned>(port), 500ms);
  const auto result = client.refreshControl();
  check(!result.ok, "Walk-only Brainstem must not become ready");
  check(result.transport_ok, "reachable incompatible server is not transport loss");
  check(!result.accepted, "Walk-only Brainstem must not acquire a lease");
  check(
      result.state.reason == "protocol_incompatible",
      "missing lease RPC must report protocol incompatibility");
  check(
      result.error.find("AcquireControl") != std::string::npos,
      "protocol error must name the missing RPC");

  server->Shutdown();
  server->Wait();
}

std::string readTextFile(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(
      std::istreambuf_iterator<char>(input),
      std::istreambuf_iterator<char>());
}

void testMutualTlsBrainstemPath() {
  char directory_template[] = "/tmp/lingtu_driver_tls_XXXXXX";
  const char* created = mkdtemp(directory_template);
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
      "-config " + config.string() +
      " -subj /CN=localhost "
      "-addext subjectAltName=DNS:localhost,IP:127.0.0.1 "
      "-keyout " + private_key.string() + " -out " + certificate.string() +
      " >/dev/null 2>&1";
  check(std::system(command.c_str()) == 0, "openssl TLS fixture generation");

  WalkService service;
  grpc::SslServerCredentialsOptions options(
      GRPC_SSL_REQUEST_AND_REQUIRE_CLIENT_CERTIFICATE_AND_VERIFY);
  options.pem_root_certs = readTextFile(certificate);
  options.pem_key_cert_pairs.push_back(
      {readTextFile(private_key), readTextFile(certificate)});
  grpc::ServerBuilder builder;
  int port = 0;
  builder.AddListeningPort(
      "127.0.0.1:0", grpc::SslServerCredentials(options), &port);
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  check(server != nullptr && port > 0, "mTLS mock Brainstem server must start");

  lingtu::driver::BrainstemTlsConfig tls;
  tls.ca_file = certificate.string();
  tls.certificate_file = certificate.string();
  tls.private_key_file = private_key.string();
  lingtu::driver::Brainstem client(
      "127.0.0.1", static_cast<unsigned>(port), 1000ms, tls);
  const auto result = client.refreshControl();
  check(result.ok, "mTLS Brainstem lease and initial zero must succeed");
  check(
      result.state.initial_zero_acknowledged,
      "mTLS path must preserve initial zero ACK gate");
  check(service.calls() == 1, "mTLS acquisition must deliver checked zero");

  client.release();
  server->Shutdown();
  server->Wait();
  std::error_code remove_error;
  std::filesystem::remove_all(directory, remove_error);
}

}  // namespace

int main() {
  try {
    testFinalVelocityQosExpiresQueuedMotion();
    testFreshnessRejectionsAreVisibleInDriverStatus();
    testDdsToBrainstemPath();
    testLegacyWalkOnlyServerFailsClosedWithExplicitProtocolReason();
    testMutualTlsBrainstemPath();
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_driver_io: FAIL: %s\n", exc.what());
    return 1;
  }
}
