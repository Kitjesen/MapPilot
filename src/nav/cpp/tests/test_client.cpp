#include "client.hpp"
#include "client_c.h"
#include "clock_sync.hpp"

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "message/cpp/exploration_command.hpp"
#include "message/cpp/inspection_command.hpp"
#include "message/cpp/navigation_command.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

#include <unistd.h>

namespace {

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double stampSeconds(const lingtu_dds_Time& stamp) {
  return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

dds_entity_t createReader(
    dds_entity_t participant,
    dds_entity_t subscriber,
    const lingtu::message::TopicContract& contract,
    const dds_topic_descriptor_t* descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(
          participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_nav_client_reader)");
  auto qos = lingtu::dds::make_qos(
      lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(
      dds_create_reader(subscriber, topic, qos.get(), nullptr),
      "dds_create_reader(test_nav_client)");
}

dds_entity_t createWriter(
    dds_entity_t participant,
    dds_entity_t publisher,
    const lingtu::message::TopicContract& contract,
    const dds_topic_descriptor_t* descriptor) {
  const dds_entity_t topic = checked(
      dds_create_topic(
          participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic(test_nav_client_writer)");
  auto qos = lingtu::dds::make_qos(
      lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(
      dds_create_writer(publisher, topic, qos.get(), nullptr),
      "dds_create_writer(test_nav_client)");
}

template <typename Message>
Message* takeOne(dds_entity_t reader, int timeout_ms = 1000) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    void* raw_sample = nullptr;
    dds_sample_info_t info{};
    const dds_return_t count = dds_take(reader, &raw_sample, &info, 1, 1);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(test_nav_client): ") + dds_strretcode(-count));
    }
    if (count == 1 && info.valid_data) {
      return static_cast<Message*>(raw_sample);
    }
    if (count == 1) {
      checked(
          dds_return_loan(reader, &raw_sample, 1),
          "dds_return_loan(test_nav_client_invalid)");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw std::runtime_error("timed out waiting for navigation command request");
}

template <typename Message>
void returnLoan(dds_entity_t reader, Message* sample) {
  void* raw_sample = sample;
  checked(
      dds_return_loan(reader, &raw_sample, 1),
      "dds_return_loan(test_nav_client)");
}

void writeAck(
    dds_entity_t writer,
    const std::string& request_id,
    lingtu::message::NavigationCommandKind kind,
    bool accepted,
    const std::string& reason,
    double endpoint_stamp_s = nowSeconds()) {
  lingtu_dds_NavigationCommandAck ack{};
  ack.header.stamp.sec = static_cast<std::int32_t>(endpoint_stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (endpoint_stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("map");
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.kind = static_cast<std::int32_t>(kind);
  ack.accepted = accepted;
  ack.reason = const_cast<char*>(reason.c_str());
  checked(dds_write(writer, &ack), "dds_write(test_nav_client_ack)");
}

void writeExplorationAck(
    dds_entity_t writer,
    const std::string& request_id,
    lingtu::message::ExplorationCommandKind kind,
    bool accepted,
    const std::string& reason,
    const std::string& session_id) {
  lingtu_dds_ExplorationCommandAck ack{};
  const double stamp_s = nowSeconds();
  ack.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("map");
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.kind = static_cast<std::int32_t>(kind);
  ack.accepted = accepted;
  ack.reason = const_cast<char*>(reason.c_str());
  ack.session_id = const_cast<char*>(session_id.c_str());
  checked(dds_write(writer, &ack), "dds_write(test_exploration_client_ack)");
}

void writeInspectionAck(
    dds_entity_t writer,
    const std::string& request_id,
    lingtu::message::InspectionCommandKind kind,
    bool accepted,
    const std::string& reason) {
  lingtu_dds_InspectionCommandAck ack{};
  const double stamp_s = nowSeconds();
  ack.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  ack.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(ack.header.stamp.sec)) * 1e9);
  ack.header.frame_id = const_cast<char*>("map");
  ack.request_id = const_cast<char*>(request_id.c_str());
  ack.kind = static_cast<std::int32_t>(kind);
  ack.accepted = accepted;
  ack.reason = const_cast<char*>(reason.c_str());
  ack.run_id = const_cast<char*>("inspection-test-run");
  checked(dds_write(writer, &ack), "dds_write(test_inspection_client_ack)");
}

template <typename Send, typename Verify>
void sendAndReply(
    dds_entity_t request_reader,
    dds_entity_t ack_writer,
    Send&& send,
    Verify&& verify,
    bool accepted = true,
    const std::string& reason = "accepted") {
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      send();
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  auto* request = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  verify(*request);
  const std::string request_id = request->request_id;
  const auto kind = static_cast<lingtu::message::NavigationCommandKind>(request->kind);
  writeAck(ack_writer, request_id, kind, accepted, reason);
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
}

template <typename Send, typename Verify>
void sendExplorationAndReply(
    dds_entity_t request_reader,
    dds_entity_t ack_writer,
    Send&& send,
    Verify&& verify,
    bool accepted = true,
    const std::string& reason = "accepted") {
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      send();
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  auto* request = takeOne<lingtu_dds_ExplorationCommandRequest>(request_reader);
  verify(*request);
  const std::string request_id = request->request_id;
  const std::string session_id = request->session_id;
  const auto kind =
      static_cast<lingtu::message::ExplorationCommandKind>(request->kind);
  writeExplorationAck(
      ack_writer, request_id, kind, accepted, reason, session_id);
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
}

template <typename Send, typename Verify>
void sendInspectionAndReply(
    dds_entity_t request_reader,
    dds_entity_t ack_writer,
    Send&& send,
    Verify&& verify,
    bool accepted = true,
    const std::string& reason = "accepted") {
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      send();
    } catch (...) {
      sender_error = std::current_exception();
    }
  });
  auto* request = takeOne<lingtu_dds_InspectionCommandRequest>(request_reader);
  verify(*request);
  const std::string request_id = request->request_id;
  const auto kind =
      static_cast<lingtu::message::InspectionCommandKind>(request->kind);
  writeInspectionAck(ack_writer, request_id, kind, accepted, reason);
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
}

void testExplorationCommands() {
  using ExplorationKind = lingtu::message::ExplorationCommandKind;
  const int domain_id = 205 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_exploration_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_exploration_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_exploration_client)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavExplorationCommand,
      &lingtu_dds_ExplorationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavExplorationAck,
      &lingtu_dds_ExplorationCommandAck_desc);

  lingtu::nav::commands::Client session(domain_id);
  auto& exploration = session.exploration();
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() {
        exploration.start(
            "session-a", "operator_start", 1000, "explore-start");
      },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kStart),
            "exploration start kind mismatch");
        check(std::string(request.header.frame_id) == "map",
              "exploration command frame mismatch");
        check(std::string(request.session_id) == "session-a",
              "exploration session id mismatch");
        check(std::string(request.reason) == "operator_start",
              "exploration start reason mismatch");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() { exploration.pause("operator_pause", 1000, "explore-pause"); },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kPause),
            "exploration pause kind mismatch");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() { exploration.resume("operator_resume", 1000, "explore-resume"); },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kResume),
            "exploration resume kind mismatch");
      });
  sendExplorationAndReply(
      request_reader,
      ack_writer,
      [&]() { exploration.stop("operator_stop", 1000, "explore-stop"); },
      [&](const lingtu_dds_ExplorationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(ExplorationKind::kStop),
            "exploration stop kind mismatch");
        check(std::string(request.reason) == "operator_stop",
              "exploration stop reason mismatch");
      });

  bool rejected = false;
  try {
    sendExplorationAndReply(
        request_reader,
        ack_writer,
        [&]() {
          exploration.start(
              "session-b", "operator_start", 1000, "explore-reject");
        },
        [&](const lingtu_dds_ExplorationCommandRequest&) {},
        false,
        "exploration_inputs_not_ready");
  } catch (const std::runtime_error& exc) {
    rejected = std::string(exc.what()).find("exploration_inputs_not_ready") !=
        std::string::npos;
  }
  check(rejected, "exploration command rejection was not surfaced");
  dds_delete(participant);
}

void testInspectionCommands() {
  using InspectionKind = lingtu::message::InspectionCommandKind;
  // CycloneDDS' default UDP port mapping overflows above domain 232.
  const int domain_id = 220 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_inspection_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_inspection_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_inspection_client)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavInspectionCommand,
      &lingtu_dds_InspectionCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavInspectionAck,
      &lingtu_dds_InspectionCommandAck_desc);

  lingtu::nav::commands::Client session(domain_id);
  auto& inspection = session.inspection();
  sendInspectionAndReply(
      request_reader,
      ack_writer,
      [&]() { inspection.start("route-a", 7, 1000, "inspection-start"); },
      [&](const lingtu_dds_InspectionCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kStart),
              "inspection start kind mismatch");
        check(std::string(request.header.frame_id) == "map",
              "inspection frame mismatch");
        check(std::string(request.route_id) == "route-a",
              "inspection route id mismatch");
        check(request.route_revision == 7U,
              "inspection route revision mismatch");
      });
  sendInspectionAndReply(
      request_reader,
      ack_writer,
      [&]() { inspection.pause("operator_hold", 1000, "inspection-pause"); },
      [&](const lingtu_dds_InspectionCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kPause),
              "inspection pause kind mismatch");
        check(std::string(request.reason) == "operator_hold",
              "inspection pause reason mismatch");
      });
  sendInspectionAndReply(
      request_reader,
      ack_writer,
      [&]() { inspection.resume("operator_resume", 1000, "inspection-resume"); },
      [&](const lingtu_dds_InspectionCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kResume),
              "inspection resume kind mismatch");
      });
  sendInspectionAndReply(
      request_reader,
      ack_writer,
      [&]() { inspection.cancel("operator_cancel", 1000, "inspection-cancel"); },
      [&](const lingtu_dds_InspectionCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(InspectionKind::kCancel),
              "inspection cancel kind mismatch");
      });

  bool rejected = false;
  try {
    sendInspectionAndReply(
        request_reader,
        ack_writer,
        [&]() { inspection.start("route-b", 1, 1000, "inspection-reject"); },
        [&](const lingtu_dds_InspectionCommandRequest&) {},
        false,
        "inspection_route_not_found");
  } catch (const std::runtime_error& exc) {
    rejected =
        std::string(exc.what()).find("inspection_route_not_found") !=
        std::string::npos;
  }
  check(rejected, "inspection rejection ACK was not surfaced to caller");
  dds_delete(participant);
}

void testEstopBypassesGoalAckWait() {
  using NavigationKind = lingtu::message::NavigationCommandKind;
  const int domain_id = 215 + static_cast<int>(getpid() % 5);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_estop_priority)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_estop_priority)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_estop_priority)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu_nav_client_handle client = lingtu_nav_client_create(domain_id);
  check(client != nullptr, "C ABI client creation failed in priority test");
  int goal_result = 0;
  int estop_result = 0;
  std::string goal_error;
  std::string estop_error;
  std::exception_ptr observation_error;
  std::thread goal_sender([&]() {
    goal_result = lingtu_nav_client_send_goal_with_id(
        client, "goal-without-ack", 1.0, 2.0, 0.0, 0.0, 500);
    if (goal_result != 0) {
      goal_error = lingtu_nav_client_last_error(client);
    }
  });

  auto* goal_request =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      std::string(goal_request->request_id) == "goal-without-ack",
      "priority test did not observe the blocked goal");
  returnLoan(request_reader, goal_request);

  const auto estop_started = std::chrono::steady_clock::now();
  std::chrono::steady_clock::duration estop_wire_delay{};
  std::thread estop_sender([&]() {
    estop_result = lingtu_nav_client_estop_with_id(
        client, "priority-estop", "operator_estop", 500);
    if (estop_result != 0) {
      estop_error = lingtu_nav_client_last_error(client);
    }
  });

  try {
    auto* estop_request =
        takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 250);
    estop_wire_delay = std::chrono::steady_clock::now() - estop_started;
    check(
        std::string(estop_request->request_id) == "priority-estop",
        "priority test observed the wrong request id");
    check(
        estop_request->kind == static_cast<std::int32_t>(NavigationKind::Estop),
        "priority request must be estop");
    writeAck(
        ack_writer,
        "priority-estop",
        NavigationKind::Estop,
        true,
        "estop_latched");
    returnLoan(request_reader, estop_request);
  } catch (...) {
    observation_error = std::current_exception();
  }

  estop_sender.join();
  goal_sender.join();
  lingtu_nav_client_destroy(client);
  dds_delete(participant);
  if (observation_error) {
    std::rethrow_exception(observation_error);
  }
  check(
      estop_result == 0,
      estop_error.empty() ? "estop failed in priority test" : estop_error.c_str());
  check(goal_result != 0, "goal without ACK must time out");
  check(
      goal_error.find("timed out waiting for navigation command ACK") !=
          std::string::npos,
      "goal timeout did not preserve the calling thread's C ABI error");
  check(
      estop_wire_delay < std::chrono::milliseconds(250),
      "estop was blocked behind the goal ACK wait");
}

void runTest() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  const int domain_id = 170 + static_cast<int>(getpid() % 20);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().sendGoal(1.25, -2.5, 0.4, 0.6, 1000, "goal-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(std::string(request.request_id) == "goal-001", "goal request id mismatch");
        check(request.kind == static_cast<std::int32_t>(CommandKind::Goal), "goal kind mismatch");
        check(std::abs(request.goal.position.x - 1.25) < 1e-9, "goal x mismatch");
        check(std::abs(request.goal.position.y + 2.5) < 1e-9, "goal y mismatch");
        check(std::string(request.header.frame_id) == "map", "goal frame mismatch");
        check(std::abs(request.goal.orientation.z - std::sin(0.3)) < 1e-9, "goal yaw mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().cancel("operator_cancel", 1000, "cancel-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::Cancel), "cancel kind mismatch");
        check(std::string(request.reason) == "operator_cancel", "cancel reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().sendTeleop(0.2, -0.1, 0.5, 1000, "teleop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::Teleop), "teleop kind mismatch");
        check(std::abs(request.velocity.linear.x - 0.2) < 1e-9, "teleop vx mismatch");
        check(std::abs(request.velocity.linear.y + 0.1) < 1e-9, "teleop vy mismatch");
        check(std::abs(request.velocity.angular.z - 0.5) < 1e-9, "teleop wz mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().stop("operator_stop", 1000, "stop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::Stop), "stop kind mismatch");
        check(std::string(request.reason) == "operator_stop", "stop reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().estop("operator_estop", 1000, "estop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(request.kind == static_cast<std::int32_t>(CommandKind::Estop), "estop kind mismatch");
        check(std::string(request.reason) == "operator_estop", "estop reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().clearEstop("operator_reset", 1000, "clear-estop-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(CommandKind::ClearEstop),
            "clear estop kind mismatch");
        check(std::string(request.reason) == "operator_reset", "clear estop reason mismatch");
      });

  sendAndReply(
      request_reader,
      ack_writer,
      [&]() { client.navigation().resumeAutonomy("operator_resume", 1000, "resume-001"); },
      [&](const lingtu_dds_NavigationCommandRequest& request) {
        check(
            request.kind == static_cast<std::int32_t>(CommandKind::ResumeAutonomy),
            "resume autonomy kind mismatch");
        check(
            std::string(request.reason) == "operator_resume",
            "resume autonomy reason mismatch");
      });

  bool rejected = false;
  try {
    sendAndReply(
        request_reader,
        ack_writer,
        [&]() {
          client.navigation().sendGoal(
              9.0, 9.0, 0.0, 0.0, 1000, "goal-reject");
        },
        [&](const lingtu_dds_NavigationCommandRequest&) {},
        false,
        "active_octomap_not_configured");
  } catch (const std::runtime_error& exc) {
    rejected = std::string(exc.what()).find("active_octomap_not_configured") !=
        std::string::npos;
  }
  check(rejected, "rejected command ACK was not surfaced to caller");

  std::string stale_rejection;
  std::exception_ptr stale_sender_error;
  std::thread stale_sender([&]() {
    try {
      client.navigation().sendTeleop(
          0.1, 0.0, 0.0, 1000, "teleop-reject-with-clock-diagnostics");
    } catch (...) {
      stale_sender_error = std::current_exception();
    }
  });
  auto* first_stale =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      std::string(first_stale->request_id) ==
          "teleop-reject-with-clock-diagnostics",
      "first stale teleop request id mismatch");
  writeAck(
      ack_writer,
      first_stale->request_id,
      CommandKind::Teleop,
      false,
      "teleop_source_stamp_stale");
  returnLoan(request_reader, first_stale);

  auto* recovery_sync =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      recovery_sync->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
          std::string(recovery_sync->reason) == "client_clock_sync",
      "clock rejection recovery must establish time through a safe stop");
  writeAck(
      ack_writer,
      recovery_sync->request_id,
      CommandKind::Stop,
      true,
      "stopped");
  returnLoan(request_reader, recovery_sync);

  auto* retry_stale =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader);
  check(
      std::string(retry_stale->request_id) ==
          "teleop-reject-with-clock-diagnostics-clock-retry-1",
      "clock recovery retry must use a distinct traceable request id");
  writeAck(
      ack_writer,
      retry_stale->request_id,
      CommandKind::Teleop,
      false,
      "teleop_source_stamp_stale");
  returnLoan(request_reader, retry_stale);
  stale_sender.join();
  if (stale_sender_error) {
    try {
      std::rethrow_exception(stale_sender_error);
    } catch (const std::runtime_error& exc) {
      stale_rejection = exc.what();
    }
  }
  check(
      stale_rejection.find("teleop_source_stamp_stale") != std::string::npos,
      "stale teleop rejection was not surfaced to caller");
  for (const char* field : {
           "sync_rtt_ms=",
           "endpoint_stamp_s=",
           "local_wall_receive_s=",
           "clock_offset_s=",
           "send_source_stamp_s=",
       }) {
    check(
        stale_rejection.find(field) != std::string::npos,
        "clock rejection exception is missing audit diagnostics");
  }

  dds_delete(participant);
}

void testSourceStampIsRefreshedAfterDiscovery() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  const int domain_id = 195 + static_cast<int>(getpid() % 20);
  lingtu::nav::commands::Client client(domain_id);
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      client.navigation().sendTeleop(
          0.1, 0.0, 0.0, 2000, "teleop-delayed-discovery");
    } catch (...) {
      sender_error = std::current_exception();
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_delayed)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_delayed)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_delayed)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  auto* sync = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  check(
      sync->kind == static_cast<std::int32_t>(CommandKind::Stop),
      "delayed first teleop must establish the endpoint clock");
  writeAck(
      ack_writer,
      sync->request_id,
      CommandKind::Stop,
      true,
      "stopped");
  returnLoan(request_reader, sync);
  auto* request = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  check(
      request->kind == static_cast<std::int32_t>(CommandKind::Teleop),
      "clock synchronization must be followed by teleop");
  const double publication_age_s = nowSeconds() - stampSeconds(request->header.stamp);
  check(
      publication_age_s >= 0.0 && publication_age_s < 0.25,
      "source stamp must be refreshed after delayed DDS discovery");
  writeAck(
      ack_writer,
      request->request_id,
      CommandKind::Teleop,
      true,
      "accepted");
  returnLoan(request_reader, request);
  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  dds_delete(participant);
}

void testFirstTeleopUsesEndpointClockAnchor() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  constexpr double kEndpointClockOffsetS = -1.0;
  const int domain_id = 150 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_clock_anchor)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_clock_anchor)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_clock_anchor)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      client.navigation().sendTeleop(
          0.1, 0.0, 0.0, 2000, "teleop-clock-anchor");
    } catch (...) {
      sender_error = std::current_exception();
    }
  });

  auto* first = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  const bool first_was_clock_sync =
      first->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
      std::string(first->reason) == "client_clock_sync";
  lingtu_dds_NavigationCommandRequest* teleop = first;
  if (first_was_clock_sync) {
    const double sync_endpoint_s = nowSeconds() + kEndpointClockOffsetS;
    writeAck(
        ack_writer,
        first->request_id,
        CommandKind::Stop,
        true,
        "stopped",
        sync_endpoint_s);
    returnLoan(request_reader, first);
    teleop = takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  }
  const bool second_was_teleop =
      teleop->kind == static_cast<std::int32_t>(CommandKind::Teleop);
  const double endpoint_receive_s = nowSeconds() + kEndpointClockOffsetS;
  const double source_age_s =
      endpoint_receive_s - stampSeconds(teleop->header.stamp);
  const bool source_age_valid = source_age_s >= -0.05 && source_age_s < 0.25;
  writeAck(
      ack_writer,
      teleop->request_id,
      CommandKind::Teleop,
      true,
      "accepted",
      endpoint_receive_s);
  returnLoan(request_reader, teleop);

  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  check(
      first_was_clock_sync,
      "a fresh motion client must establish endpoint time with a safe stop");
  check(
      second_was_teleop,
      "clock synchronization must be followed by the requested teleop command");
  check(
      source_age_valid,
      "first teleop source stamp must use the endpoint clock domain");
  dds_delete(participant);
}

void testDelayedClockAckIsResampledBeforeTeleop() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  constexpr double kEndpointClockOffsetS = -1.0;
  const int domain_id = 140 + static_cast<int>(getpid() % 10);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_delayed_clock_ack)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_delayed_clock_ack)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_delayed_clock_ack)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client client(domain_id);
  std::exception_ptr sender_error;
  std::thread sender([&]() {
    try {
      client.navigation().sendTeleop(
          0.1, 0.0, 0.0, 2500, "teleop-delayed-clock-ack");
    } catch (...) {
      sender_error = std::current_exception();
    }
  });

  auto* first_sync =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  check(
      first_sync->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
          std::string(first_sync->reason) == "client_clock_sync",
      "first delayed-clock request must be a safe clock synchronization stop");
  const double delayed_ack_stamp_s = nowSeconds() + kEndpointClockOffsetS;
  const std::string first_sync_id = first_sync->request_id;
  returnLoan(request_reader, first_sync);
  std::this_thread::sleep_for(std::chrono::milliseconds(350));
  writeAck(
      ack_writer,
      first_sync_id,
      CommandKind::Stop,
      true,
      "stopped",
      delayed_ack_stamp_s);

  auto* second =
      takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  const bool delayed_sample_was_resampled =
      second->kind == static_cast<std::int32_t>(CommandKind::Stop) &&
      std::string(second->reason) == "client_clock_sync";
  lingtu_dds_NavigationCommandRequest* teleop = second;
  if (delayed_sample_was_resampled) {
    writeAck(
        ack_writer,
        second->request_id,
        CommandKind::Stop,
        true,
        "stopped",
        nowSeconds() + kEndpointClockOffsetS);
    returnLoan(request_reader, second);
    teleop =
        takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
  }
  const bool teleop_followed_sync =
      teleop->kind == static_cast<std::int32_t>(CommandKind::Teleop);
  const double endpoint_receive_s = nowSeconds() + kEndpointClockOffsetS;
  const double source_age_s =
      endpoint_receive_s - stampSeconds(teleop->header.stamp);
  const bool source_age_valid = source_age_s >= -0.05 && source_age_s < 0.25;
  writeAck(
      ack_writer,
      teleop->request_id,
      CommandKind::Teleop,
      true,
      "accepted",
      endpoint_receive_s);
  returnLoan(request_reader, teleop);

  sender.join();
  if (sender_error) {
    std::rethrow_exception(sender_error);
  }
  check(
      delayed_sample_was_resampled,
      "a delayed clock ACK must be resampled instead of aging the first teleop");
  check(teleop_followed_sync, "clock resampling must preserve the teleop request");
  check(
      source_age_valid,
      "teleop after clock resampling must satisfy the endpoint freshness window");
  dds_delete(participant);
}

void testConcurrentClientsKeepClockAcksIsolated() {
  using CommandKind = lingtu::message::NavigationCommandKind;
  constexpr double kEndpointClockOffsetS = -0.75;
  const int domain_id = 120 + static_cast<int>(getpid() % 20);
  const dds_entity_t participant = checked(
      dds_create_participant(
          static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
      "dds_create_participant(test_nav_client_concurrent)");
  const dds_entity_t subscriber = checked(
      dds_create_subscriber(participant, nullptr, nullptr),
      "dds_create_subscriber(test_nav_client_concurrent)");
  const dds_entity_t publisher = checked(
      dds_create_publisher(participant, nullptr, nullptr),
      "dds_create_publisher(test_nav_client_concurrent)");
  const dds_entity_t request_reader = createReader(
      participant,
      subscriber,
      lingtu::message::kNavCommandRequest,
      &lingtu_dds_NavigationCommandRequest_desc);
  const dds_entity_t ack_writer = createWriter(
      participant,
      publisher,
      lingtu::message::kNavCommandAck,
      &lingtu_dds_NavigationCommandAck_desc);

  lingtu::nav::commands::Client first_client(domain_id);
  lingtu::nav::commands::Client second_client(domain_id);
  std::exception_ptr first_error;
  std::exception_ptr second_error;
  std::thread first_sender([&]() {
    try {
      first_client.navigation().sendTeleop(
          0.11, 0.0, 0.0, 2000, "teleop-client-a");
    } catch (...) {
      first_error = std::current_exception();
    }
  });
  std::thread second_sender([&]() {
    try {
      second_client.navigation().sendTeleop(
          0.22, 0.0, 0.0, 2000, "teleop-client-b");
    } catch (...) {
      second_error = std::current_exception();
    }
  });

  std::set<std::string> sync_ids;
  bool first_teleop_seen = false;
  bool second_teleop_seen = false;
  bool source_ages_valid = true;
  for (int i = 0; i < 4; ++i) {
    auto* request =
        takeOne<lingtu_dds_NavigationCommandRequest>(request_reader, 2000);
    const auto kind = static_cast<CommandKind>(request->kind);
    const std::string request_id = request->request_id;
    if (kind == CommandKind::Stop &&
        std::string(request->reason) == "client_clock_sync") {
      sync_ids.insert(request_id);
      writeAck(
          ack_writer,
          request_id,
          CommandKind::Stop,
          true,
          "stopped",
          nowSeconds() + kEndpointClockOffsetS);
    } else if (kind == CommandKind::Teleop) {
      const double endpoint_receive_s = nowSeconds() + kEndpointClockOffsetS;
      const double source_age_s =
          endpoint_receive_s - stampSeconds(request->header.stamp);
      source_ages_valid = source_ages_valid &&
          source_age_s >= -0.05 && source_age_s < 0.25;
      if (request_id == "teleop-client-a") {
        first_teleop_seen = std::abs(request->velocity.linear.x - 0.11) < 1e-9;
      } else if (request_id == "teleop-client-b") {
        second_teleop_seen = std::abs(request->velocity.linear.x - 0.22) < 1e-9;
      }
      writeAck(
          ack_writer,
          request_id,
          CommandKind::Teleop,
          true,
          "accepted",
          endpoint_receive_s);
    }
    returnLoan(request_reader, request);
  }

  first_sender.join();
  second_sender.join();
  if (first_error) {
    std::rethrow_exception(first_error);
  }
  if (second_error) {
    std::rethrow_exception(second_error);
  }
  check(sync_ids.size() == 2, "each concurrent client must use its own clock ACK");
  check(first_teleop_seen, "first concurrent client teleop was not delivered");
  check(second_teleop_seen, "second concurrent client teleop was not delivered");
  check(
      source_ages_valid,
      "concurrent client teleop timestamps must remain in endpoint time");
  dds_delete(participant);
}

void testClockOffsetTracksRealtimeRollback() {
  constexpr double kEndpointAckStampS = 1000.0;
  constexpr double kLocalReceiveWallS = 1001.0;
  constexpr double kSteadyElapsedS = 0.20;
  constexpr double kLocalWallAfterRollbackS = 1000.60;
  constexpr double kFutureToleranceS = 0.05;

  const double offset = lingtu::nav::commands::endpointClockOffset(
      kEndpointAckStampS,
      kLocalReceiveWallS);
  const double endpoint_now_s = kLocalWallAfterRollbackS + offset;
  const double old_steady_projection_s = kEndpointAckStampS + kSteadyElapsedS;
  const double offset_projection_s = lingtu::nav::commands::endpointSourceTime(
      kLocalWallAfterRollbackS,
      offset);

  check(
      old_steady_projection_s - endpoint_now_s > kFutureToleranceS,
      "regression setup must make the steady anchor future-dated after rollback");
  check(
      std::abs(offset_projection_s - endpoint_now_s) < 1e-12,
      "clock offset projection must follow endpoint CLOCK_REALTIME rollback");
}

}  // namespace

int main() {
  try {
    check(
        lingtu_nav_client_abi_version() == LINGTU_NAV_CLIENT_ABI_VERSION,
        "navigation client ABI version mismatch");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_NAVIGATION) != 0U,
        "navigation client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_INSPECTION) != 0U,
        "inspection client capability missing");
    check(
        (lingtu_nav_client_capabilities() &
         LINGTU_NAV_CLIENT_CAP_EXPLORATION) != 0U,
        "exploration client capability missing");
    runTest();
    testExplorationCommands();
    testInspectionCommands();
    testEstopBypassesGoalAckWait();
    testSourceStampIsRefreshedAfterDiscovery();
    testFirstTeleopUsesEndpointClockAnchor();
    testDelayedClockAckIsResampledBeforeTeleop();
    testConcurrentClientsKeepClockAcksIsolated();
    testClockOffsetTracksRealtimeRollback();
    std::puts("test_nav_client: PASS");
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_nav_client: FAIL: %s\n", exc.what());
    return 1;
  }
}
