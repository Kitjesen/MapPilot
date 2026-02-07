/**
 * robot_state_publisher.cpp
 *
 * ROS2 数据采集节点示例：将 Unitree SDK 的机器人数据转换为
 * interface::msg::RobotState 消息并发布到 /robot_state topic。
 *
 * 使用方法:
 *   1. 链接你的 Unitree SDK (unitree_legged_sdk)
 *   2. 在 CMakeLists.txt 中编译此节点
 *   3. ros2 run interface robot_state_publisher
 *
 * 当前为模板/示例代码，需要根据你的硬件 SDK 填写实际数据读取逻辑。
 * 标有 "TODO: 替换为真实 SDK 调用" 的地方需要你来实现。
 */

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "interface/msg/robot_state.hpp"

// TODO: 取消注释并链接你的 Unitree SDK
// #include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std::chrono_literals;

class RobotStatePublisher : public rclcpp::Node {
public:
  RobotStatePublisher() : Node("robot_state_publisher") {
    // 参数
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<std::string>("topic", "/robot_state");

    const double rate = this->get_parameter("publish_rate_hz").as_double();
    const auto topic = this->get_parameter("topic").as_string();

    publisher_ = this->create_publisher<interface::msg::RobotState>(topic, 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(rate, 1.0)),
        std::bind(&RobotStatePublisher::publish_state, this));

    RCLCPP_INFO(this->get_logger(),
                "RobotStatePublisher started: topic='%s', rate=%.1f Hz",
                topic.c_str(), rate);

    // TODO: 初始化 Unitree SDK
    // udp_ = std::make_unique<UNITREE_LEGGED_SDK::UDP>(
    //     UNITREE_LEGGED_SDK::LOWLEVEL);
    // udp_->InitCmdData(cmd_);
  }

private:
  void publish_state() {
    interface::msg::RobotState msg;

    // Header
    msg.header.stamp = this->now();
    msg.header.frame_id = "body";

    // ====================================================================
    // TODO: 替换为真实 SDK 调用
    // ====================================================================
    //
    // 示例 (Unitree Go1 Low-level SDK):
    //
    //   udp_->Recv();
    //   udp_->GetRecv(state_);
    //
    //   // 关节位置 (12 DOF)
    //   for (int i = 0; i < 12; ++i) {
    //     msg.joint_positions[i] = state_.motorState[i].q;
    //     msg.joint_velocities[i] = state_.motorState[i].dq;
    //     msg.joint_efforts[i] = state_.motorState[i].tauEst;
    //   }
    //
    //   // 电池
    //   msg.battery.percentage = state_.bms.SOC;
    //   double total_mv = 0;
    //   for (int i = 0; i < 10; ++i) total_mv += state_.bms.cell_vol[i];
    //   msg.battery.voltage = total_mv / 1000.0f;
    //   msg.battery.current = state_.bms.current;
    //   msg.battery.temperature[0] = state_.bms.BQ_NTC[0];
    //   msg.battery.temperature[1] = state_.bms.BQ_NTC[1];
    //   msg.battery.cycle_count = state_.bms.cycle;
    //   msg.battery.status = 0; // 0=normal
    //
    //   // 足端力
    //   for (int i = 0; i < 4; ++i) {
    //     msg.foot_forces[i] = state_.footForce[i];
    //   }
    //
    //   // IMU
    //   msg.imu_quaternion[0] = state_.imu.quaternion[0]; // w
    //   msg.imu_quaternion[1] = state_.imu.quaternion[1]; // x
    //   msg.imu_quaternion[2] = state_.imu.quaternion[2]; // y
    //   msg.imu_quaternion[3] = state_.imu.quaternion[3]; // z
    //   msg.imu_gyroscope[0] = state_.imu.gyroscope[0];
    //   msg.imu_gyroscope[1] = state_.imu.gyroscope[1];
    //   msg.imu_gyroscope[2] = state_.imu.gyroscope[2];
    //   msg.imu_accelerometer[0] = state_.imu.accelerometer[0];
    //   msg.imu_accelerometer[1] = state_.imu.accelerometer[1];
    //   msg.imu_accelerometer[2] = state_.imu.accelerometer[2];
    //   msg.imu_temperature = state_.imu.temperature;
    //
    // ====================================================================

    // ---- 模拟数据（开发测试用，正式使用时删除） ----
    const double t = this->now().seconds();
    for (size_t i = 0; i < 12; ++i) {
      msg.joint_positions[i] = 0.1f * std::sin(t + i * 0.5);
      msg.joint_velocities[i] = 0.05f * std::cos(t + i * 0.5);
      msg.joint_efforts[i] = 0.0f;
    }
    msg.battery.percentage = 75;
    msg.battery.voltage = 25.2f;
    msg.battery.current = -1500.0f;
    msg.battery.temperature[0] = 35;
    msg.battery.temperature[1] = 36;
    msg.battery.status = 0;
    msg.battery.cycle_count = 120;
    for (size_t i = 0; i < 4; ++i) {
      msg.foot_forces[i] = 50.0f + 10.0f * std::sin(t + i);
    }
    msg.imu_quaternion[0] = 1.0f; // w
    msg.imu_quaternion[1] = 0.0f; // x
    msg.imu_quaternion[2] = 0.0f; // y
    msg.imu_quaternion[3] = 0.0f; // z
    msg.imu_gyroscope = {0.0f, 0.0f, 0.0f};
    msg.imu_accelerometer = {0.0f, 0.0f, 9.81f};
    msg.imu_temperature = 40;
    // ---- 模拟数据结束 ----

    publisher_->publish(msg);
  }

  rclcpp::Publisher<interface::msg::RobotState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TODO: Unitree SDK 成员
  // std::unique_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  // UNITREE_LEGGED_SDK::LowState state_{};
  // UNITREE_LEGGED_SDK::LowCmd cmd_{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
