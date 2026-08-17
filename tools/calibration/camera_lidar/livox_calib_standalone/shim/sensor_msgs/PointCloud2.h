// sensor_msgs shim
#ifndef SENSOR_MSGS_HPP
#define SENSOR_MSGS_HPP
#include <string>
#include <vector>
namespace sensor_msgs {
  struct Header { std::string frame_id; uint32_t seq = 0; double stamp = 0; };
  struct PointCloud2 {
    Header header;
    uint32_t height = 0, width = 0;
    std::vector<uint8_t> data;
  };
  struct Image {
    Header header;
    uint32_t height = 0, width = 0;
    std::string encoding;
    std::vector<uint8_t> data;
  };
  namespace image_encodings {
    const std::string BGR8 = "bgr8";
  }
}
#endif
