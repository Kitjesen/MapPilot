#pragma once

#include <optional>
#include <string>
#include <vector>

namespace lingtu::drivers::wtrtk980 {

std::vector<std::string> list_serial_devices();
std::string find_wtrtk980_device(
    const std::string& preferred = "/dev/wtrtk980",
    int baud = 115200,
    int timeout_ms = 500);

class SerialReader {
 public:
  SerialReader(std::string device, int baud, int timeout_ms);
  ~SerialReader();

  SerialReader(const SerialReader&) = delete;
  SerialReader& operator=(const SerialReader&) = delete;

  void open();
  void close();
  bool is_open() const { return fd_ >= 0; }
  std::optional<std::string> read_line();
  const std::string& device() const { return device_; }

 private:
  std::string device_;
  int baud_;
  int timeout_ms_;
  int fd_{-1};

  void configure_port();
};

}  // namespace lingtu::drivers::wtrtk980
