// cv_bridge shim — stub implementations
#ifndef CV_BRIDGE_HPP
#define CV_BRIDGE_HPP
#include <opencv2/core.hpp>
#include "sensor_msgs/PointCloud2.h"
#include <memory>

namespace cv_bridge {

struct CvImage {
  cv::Mat image;
  std::string encoding;
  sensor_msgs::Header header;
  CvImage() {}
  CvImage(const sensor_msgs::Header& h, const std::string& enc, const cv::Mat& img)
    : header(h), encoding(enc), image(img) {}
  typedef std::shared_ptr<CvImage> Ptr;
  sensor_msgs::Image::Ptr toImageMsg() {
    auto msg = std::make_shared<sensor_msgs::Image>();
    return msg;
  }
};

typedef std::shared_ptr<CvImage> CvImagePtr;

inline CvImagePtr toCvCopy(const sensor_msgs::Image&, const std::string& encoding) {
  return std::make_shared<CvImage>();
}

} // namespace cv_bridge
#endif
