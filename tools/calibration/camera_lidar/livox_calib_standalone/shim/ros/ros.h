// ROS shim — no-op for standalone build
#ifndef ROS_SHIM_HPP
#define ROS_SHIM_HPP
#include <iostream>
#include <string>
#include "sensor_msgs/PointCloud2.h"

#define ROS_INFO_STREAM(x) std::cout << "[INFO] " << x << std::endl
#define ROS_ERROR_STREAM(x) std::cerr << "[ERROR] " << x << std::endl
#define ROS_INFO(x, ...) printf("[INFO] " x "\n", ##__VA_ARGS__)
#define ROS_ERROR(x, ...) printf("[ERROR] " x "\n", ##__VA_ARGS__)
#define ROS_WARN(x, ...) printf("[WARN] " x "\n", ##__VA_ARGS__)

namespace ros {
  inline void init(int, char**, const std::string&) {}
  inline bool ok() { return false; }
  class Rate { public: Rate(double) {} void sleep() {} };
  class Publisher {
  public:
    template<typename M> void publish(const M&) {}
  };
  class NodeHandle {
  public:
    template<typename T> bool param(const std::string&, T&, const T& d) { return false; }
    template<typename M> Publisher advertise(const std::string&, int) { return Publisher(); }
  };
}
#endif
