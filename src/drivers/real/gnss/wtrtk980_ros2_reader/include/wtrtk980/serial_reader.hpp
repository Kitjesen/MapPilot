#pragma once

#include <optional>
#include <string>

#include "wtrtk980/gps_fix.hpp"

namespace wtrtk980 {

class SerialReader {
 public:
  SerialReader(std::string device = "/dev/wtrtk980", int baud = 115200, int timeout_ms = 1000);
  ~SerialReader();

  SerialReader(const SerialReader&) = delete;
  SerialReader& operator=(const SerialReader&) = delete;

  void open();
  void close();
  bool isOpen() const;
  std::optional<std::string> readRawLine();
  std::optional<GpsFix> readUpdate(int timeout_ms = -1);

  const std::string& device() const { return device_; }
  GpsFix& state() { return state_; }
  const GpsFix& state() const { return state_; }

 private:
  std::string device_;
  int baud_;
  int timeout_ms_;
  int fd_ = -1;
  GpsFix state_;

  void configurePort();
};

}  // namespace wtrtk980
