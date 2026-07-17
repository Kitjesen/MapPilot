#pragma once

#include "native/module.hpp"

#include <array>
#include <string>

namespace lingtu::drivers::gnss {

class DdsModule {
 public:
  explicit DdsModule(const Config& config);
  ~DdsModule();

  DdsModule(const DdsModule&) = delete;
  DdsModule& operator=(const DdsModule&) = delete;

  void publish_status(
      const lingtu::gnss::FixSample& fix,
      const std::string& device,
      const std::string& error,
      double stamp_s,
      Status& status);
  std::array<double, 9> publish_fix(
      const lingtu::gnss::FixSample& fix,
      double stamp_s,
      Status& status);
  void publish_odom(
      const lingtu::gnss::FixSample& fix,
      const std::array<double, 9>& covariance,
      double stamp_s,
      Status& status);

 private:
  class Impl;
  Impl* impl_{nullptr};
};

}  // namespace lingtu::drivers::gnss
