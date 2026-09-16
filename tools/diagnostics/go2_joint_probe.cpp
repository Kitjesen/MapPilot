// Read-only SDK diagnostic: subscribe for five seconds, never publish commands.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: go2_joint_probe <network-interface>\n";
    return 1;
  }
  using State = unitree_go::msg::dds_::LowState_;
  std::mutex mutex;
  std::uint64_t received = 0;
  std::uint64_t distinct_ticks = 0;
  std::uint32_t first_tick = 0;
  std::uint32_t last_tick = 0;
  double stamp_s = 0.0;
  std::array<double, 12> q{}, dq{}, tau{};
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  {
    unitree::robot::ChannelSubscriber<State> subscriber("rt/lowstate");
    subscriber.InitChannel([&](const void *message) {
      if (message == nullptr) return;
      const auto &state = *static_cast<const State *>(message);
      std::lock_guard<std::mutex> lock(mutex);
      if (received == 0) first_tick = state.tick();
      if (received == 0 || state.tick() != last_tick) ++distinct_ticks;
      ++received;
      last_tick = state.tick();
      stamp_s = std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      for (std::size_t i = 0; i < q.size(); ++i) {
        q[i] = state.motor_state()[i].q();
        dq[i] = state.motor_state()[i].dq();
        tau[i] = state.motor_state()[i].tau_est();
      }
    }, 1);
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  unitree::robot::ChannelFactory::Instance()->Release();
  std::lock_guard<std::mutex> lock(mutex);
  auto values = [](const auto &array) {
    std::cout << '[';
    for (std::size_t i = 0; i < array.size(); ++i) {
      if (i) std::cout << ',';
      if (std::isfinite(array[i])) std::cout << array[i];
      else std::cout << "null";
    }
    std::cout << ']';
  };
  std::cout << std::setprecision(15)
            << "{\"topic\":\"rt/lowstate\",\"duration_s\":5,\"received\":" << received
            << ",\"distinct_ticks\":" << distinct_ticks << ",\"first_tick\":" << first_tick
            << ",\"last_tick\":" << last_tick << ",\"sample_receive_stamp_s\":" << stamp_s
            << ",\"order\":\"FR/FL/RR/RL:hip/thigh/calf\",\"position\":";
  values(q);
  std::cout << ",\"velocity\":";
  values(dq);
  std::cout << ",\"effort\":";
  values(tau);
  std::cout << "}\n";
  return received > 0 ? 0 : 2;
}
