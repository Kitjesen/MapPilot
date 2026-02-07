# RobotState.msg - 机器人综合状态
std_msgs/Header header

# 关节角度 (12 DOF: FR_hip/thigh/calf, FL_hip/thigh/calf, RR_hip/thigh/calf, RL_hip/thigh/calf)
float32[12] joint_positions    # 关节位置 (rad)
float32[12] joint_velocities   # 关节速度 (rad/s)
float32[12] joint_efforts      # 关节力矩 (N*m)

# 电池
BatteryState battery

# 足端力
float32[4] foot_forces         # FR, FL, RR, RL 足端力

# IMU (可选，odom 里也有)
float32[4] imu_quaternion      # w, x, y, z
float32[3] imu_gyroscope       # rad/s
float32[3] imu_accelerometer   # m/s^2
int8 imu_temperature           # degrees C
