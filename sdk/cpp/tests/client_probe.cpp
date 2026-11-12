#include <lingtu/sdk.hpp>

#include <iostream>
#include <stdexcept>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

int main(int argc, char** argv) {
  if (argc < 3) return 1;
#ifdef _WIN32
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  try {
    const std::string action = argv[2];
    lingtu::sdk::Client client(argv[1], "test-key", std::chrono::seconds(2),
                                action == "media_http" ? "viewer-secret" : "");
    if (action == "stop_product") {
      std::cout << client.system().stop_product("session-1", R"({"request_id":"stop-1"})");
    } else if (action == "resume_motion") {
      std::cout << client.system().resume_motion(R"({"request_id":"resume-1","client_id":"operator"})");
    } else if (action == "goal_status") {
      std::cout << client.navigation().goal_status("goal/a?");
    } else if (action == "stop_recording_session") {
      std::cout << client.observations().stop_recording("recording-1");
    } else if (action == "product") {
      client.system().product_status();
      std::cout << client.system().switch_product("nav", "session-1", R"({"request_id":"switch-1","map_name":"hall","initial_pose":{"x":1,"y":2,"z":0,"yaw":0}})") << '\n';
      client.system().product_operation("switch/a?");
    } else if (action == "recordings") {
      if (argc < 4) return 1;
      const auto target = std::filesystem::u8path(argv[3]);
      auto observations = client.observations();
      observations.recordings(7);
      observations.recording("record-1");
      observations.recording_catalog();
      observations.recording_streams();
      observations.recording_summary("record-1");
      observations.recording_series("record-1", "/nav/state", {{"field", "pose.x"}});
      observations.recording_scene_info("record-1");
      observations.recording_scene("record-1", {{"path_topic", "/path/a"}, {"path_topic", "/path/b"}});
      observations.recording_camera_frame("record-1", 1.5, 2);
      observations.download_recording_file("record-1", "camera/front +.mkv", target);
      observations.download_recording_archive("record-1", target);
      observations.validate_recording("record-1");
      std::cout << observations.delete_recording("record-1");
    } else if (action == "inspection") {
      if (argc < 4) return 1;
      const auto target = std::filesystem::u8path(argv[3]);
      auto tasks = client.tasks();
      tasks.inspection_route("route-1", "hall +");
      tasks.save_inspection_route(R"({"id":"route-1","map_id":"hall","revision":2,"points":[{"id":"p1","x":1,"y":2}]})");
      tasks.delete_inspection_route("route-1", "hall");
      tasks.inspection_tasks({{"include_terminal", "true"}, {"limit", "7"}});
      tasks.list_inspection_evidence(5);
      tasks.inspection_evidence("evidence-1");
      std::cout << tasks.review_inspection_evidence("evidence-1", "anomaly", "glass\nwall");
      tasks.download_inspection_artifact("evidence-1", "rgb", target);
      tasks.download_inspection_measurement("evidence-1", "sensor +", target);
    } else if (action == "goal") {
      std::cout << client.navigation().go(1, 2, 3, R"({"request_id":"same-request","task_id":"same-task","yaw":0.4})");
    } else if (action == "pause") {
      std::cout << client.navigation().pause("task/a?", R"({"request_id":"pause-1"})");
    } else if (action == "save") {
      std::cout << client.map().save(R"({"name":"hall","auto_process":false,"request_id":"save_1"})");
    } else if (action == "reoptimize") {
      std::cout << client.map().reoptimize_operation("save_1", R"({"request_id":"reopt_1"})");
    } else if (action == "relocalize") {
      std::cout << client.navigation().relocalize("hall", R"({"x":1,"y":2,"yaw":0.2})", R"({"request_id":"loc_1"})");
    } else if (action == "validate") {
      std::cout << client.map().validate_plan("hall", R"({"x":1,"y":2,"z":0})");
    } else if (action == "no_go") {
      std::cout << client.map().set_no_go("hall", R"({"id":"door","min":[1,2,0],"max":[2,3,2]})", 12);
    } else if (action == "delete_no_go") {
      std::cout << client.map().delete_no_go("hall", "door/a", 12);
    } else if (action == "surface") {
      std::cout << client.map().surface("hall", {{"headroom_m", "0.5"}, {"radius_m", "0.3"}});
    } else if (action == "rename") {
      std::cout << client.map().rename("hall", "hall-copy");
    } else if (action == "reset") {
      std::cout << client.map().reset_cloud();
    } else if (action == "profile") {
      std::cout << client.system().switch_profile("walking", R"({"request_id":"profile-1","client_id":"sdk"})");
    } else if (action == "follow") {
      std::cout << client.tasks().get_follow_task("task/a?", "session +&?");
    } else if (action == "instruction") {
      std::cout << client.tasks().send_instruction("line\n\"quoted\"", R"({"request_id":"instruction-1"})");
    } else if (action == "embedding") {
      std::cout << client.observations().search_memory_embedding("[0.2,0.4]", R"({"top_k":3,"label":"door"})");
    } else if (action == "snapshot") {
      std::cout << client.observations().camera_snapshot();
    } else if (action == "media_http") {
      std::cout << client.observations().camera_snapshot();
    } else if (action == "cloud" || action == "camera" || action == "feed_close") {
      auto feed = action == "camera" ? client.watch_camera("viewer-secret") : client.watch_cloud();
      if (auto frame = feed.receive()) std::cout << frame->data;
      if (action == "feed_close" && feed.receive()) return 4;
      feed.close();
    } else if (action == "teleop" || action == "teleop_twice") {
      auto session = client.teleop("cpp-client");
      std::cout << session.move(0.25, -0.1, 0.3, false, "move-1") << '\n';
      if (action == "teleop_twice") std::cout << session.move(0.4, 0.1, 0.2, true, "move-2") << '\n';
      std::cout << session.hold("hold-1");
      session.close();
    } else if (action == "download") {
      if (argc < 4) return 1;
      client.map().download_source("save_1", std::filesystem::u8path(argv[3]));
    } else if (action == "events" || action == "callback_error") {
      client.events([&](const lingtu::sdk::Event& event) {
        if (action == "callback_error") throw std::runtime_error("callback failed");
        std::cout << event.event << '\n' << event.id << '\n' << event.data;
        return false;
      }, {{"topic", "/nav/state"}}, "6");
    } else if (action == "position") {
      std::cout << client.navigation().position();
    } else if (action == "arrived") {
      std::cout << client.navigation().wait_until_arrived("same-task", std::chrono::seconds(1), std::chrono::milliseconds(1));
    } else if (action == "source_wait") {
      std::cout << client.map().save_and_wait(R"({"name":"hall","auto_process":false,"request_id":"save_1"})",
                                           std::chrono::seconds(1), std::chrono::milliseconds(1), true);
    } else if (action == "invalid") {
      std::cout << client.map().save("[]");
    } else if (action == "move") {
      lingtu::sdk::Client moved(std::move(client));
      std::cout << moved.system().health();
    } else {
      std::cout << client.system().health();
    }
    return 0;
  } catch (const lingtu::sdk::Error& error) {
    std::cout << error.status() << '\n' << error.body();
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what();
    return 3;
  }
}
