#include <iostream>
#include <stdexcept>
#include <thread>

#include "body.hpp"
#include "config.hpp"
#include "fake_sdk.hpp"

namespace {
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
auto makeBody() {
  fake_unitree::reset();
  lingtu::driver::Config config;
  config.robot = "go2";
  config.network_interface = "eth0";
  return lingtu::driver::makeBody(config);
}
void expectCalls(std::initializer_list<std::string> calls) {
  require(fake_unitree::calls == std::vector<std::string>(calls), "unexpected SDK call order");
}
}  // namespace

int main() {
  try {
    {
      auto body = makeBody();
      require(!body->move({0.5, 0, 0}).accepted, "unconfigured movement accepted");
      expectCalls({});
      require(body->refresh().state.ready, "acquisition failed");
      expectCalls({"stop", "get", "disable", "get"});
      fake_unitree::calls.clear();
      for (int i = 0; i < 100; ++i) require(body->refresh().ok, "refresh failed");
      require(body->move({0.5, 0, 0}).accepted, "configured movement rejected");
      require(body->move({}).accepted, "zero rejected");
      expectCalls({"move", "stop"});
      require(body->stop().confirmsStop(), "stop/release failed");
      fake_unitree::enabled = true;
      fake_unitree::calls.clear();
      require(!body->move({0, 0, 0.25}).accepted, "movement accepted after release");
      require(body->refresh().ok, "reacquisition failed");
      expectCalls({"stop", "get", "disable", "get"});
    }
    {
      auto body = makeBody();
      fake_unitree::enabled = false;
      require(body->refresh().ok, "already disabled rejected");
      expectCalls({"stop", "get"});
    }
    for (int failure = 0; failure < 5; ++failure) {
      auto body = makeBody();
      if (failure == 0) fake_unitree::stop_code = 42;
      if (failure == 1) fake_unitree::failed_query = 1;
      if (failure == 2) fake_unitree::set_code = 42;
      if (failure == 3) fake_unitree::failed_query = 2;
      if (failure == 4) fake_unitree::apply_setting = false;
      const auto result = body->refresh();
      require(!result.ok && !result.state.ready && !result.error.empty(), "failure not reported");
      const auto previous_calls = fake_unitree::calls;
      require(!body->move({0.5, 0, 0}).accepted, "motion allowed after failed acquisition");
      require(fake_unitree::calls == previous_calls, "failed acquisition forwarded motion");
      if (failure == 0) expectCalls({"stop"});
      fake_unitree::stop_code = fake_unitree::set_code = fake_unitree::failed_query = 0;
      fake_unitree::apply_setting = true;
      require(body->stop().confirmsStop(), "failure stop unconfirmed");
      require(body->refresh().ok, "retry did not recover");
    }
    {
      auto body = makeBody();
      require(body->refresh().ok, "initial acquisition failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(550));
      require(!body->refresh().ok, "stale state accepted");
      // The driver main loop releases after a failed refresh before retrying.
      require(body->stop().confirmsStop(), "disconnect stop failed");
      fake_unitree::enabled = true;
      const unitree_go::msg::dds_::SportModeState_ state;
      fake_unitree::state_callback(&state);
      fake_unitree::calls.clear();
      require(body->refresh().ok, "reconnect failed");
      expectCalls({"stop", "get", "disable", "get"});
    }
    std::cout << "Go2 acquisition, failures, hot path and reconnect passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
