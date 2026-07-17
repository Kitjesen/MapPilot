#include "wtrtk980/serial.hpp"

#ifdef _WIN32
#include <stdexcept>
#else
#include <dirent.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <utility>
#endif

namespace lingtu::drivers::wtrtk980 {

#ifdef _WIN32

std::vector<std::string> list_serial_devices() {
  return {};
}

std::string find_wtrtk980_device(const std::string&, int, int) {
  throw std::runtime_error("WTRTK-980 serial reader is only supported on POSIX targets");
}

SerialReader::SerialReader(std::string device, int baud, int timeout_ms)
    : device_(std::move(device)), baud_(baud), timeout_ms_(timeout_ms) {}

SerialReader::~SerialReader() = default;

void SerialReader::open() {
  throw std::runtime_error("WTRTK-980 serial reader is only supported on POSIX targets");
}

void SerialReader::close() {}

std::optional<std::string> SerialReader::read_line() {
  return std::nullopt;
}

void SerialReader::configure_port() {}

#else
namespace {

bool starts_with(const std::string& text, const std::string& prefix) {
  return text.rfind(prefix, 0) == 0;
}

speed_t baud_to_speed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      throw std::runtime_error("unsupported GNSS baud rate: " + std::to_string(baud));
  }
}

bool path_exists(const std::string& path) {
  return access(path.c_str(), F_OK) == 0;
}

}  // namespace

std::vector<std::string> list_serial_devices() {
  std::vector<std::string> devices;
  std::set<std::string> seen;
  DIR* dir = opendir("/dev");
  if (!dir) {
    return devices;
  }
  while (dirent* ent = readdir(dir)) {
    const std::string name(ent->d_name);
    if (!starts_with(name, "ttyUSB") && !starts_with(name, "ttyACM")) {
      continue;
    }
    const std::string path = "/dev/" + name;
    std::string real = path;
    try {
      real = std::filesystem::weakly_canonical(path).string();
    } catch (...) {
    }
    if (seen.insert(real).second) {
      devices.push_back(path);
    }
  }
  closedir(dir);
  std::sort(devices.begin(), devices.end());
  return devices;
}

std::string find_wtrtk980_device(const std::string& preferred, int baud, int timeout_ms) {
  std::vector<std::string> candidates;
  if (!preferred.empty() && preferred != "auto" && path_exists(preferred)) {
    candidates.push_back(preferred);
  }
  if (path_exists("/dev/wtrtk980")) {
    candidates.push_back("/dev/wtrtk980");
  }
  const auto serial = list_serial_devices();
  candidates.insert(candidates.end(), serial.begin(), serial.end());

  std::set<std::string> seen;
  for (const auto& candidate : candidates) {
    std::string real = candidate;
    try {
      real = std::filesystem::weakly_canonical(candidate).string();
    } catch (...) {
    }
    if (!seen.insert(real).second) {
      continue;
    }
    try {
      SerialReader reader(candidate, baud, timeout_ms);
      reader.open();
      for (int i = 0; i < 4; ++i) {
        const auto line = reader.read_line();
        if (!line) {
          continue;
        }
        if (line->find("$GNGGA") == 0 || line->find("$GPGGA") == 0 ||
            line->find("$GNRMC") == 0 || line->find("$GPRMC") == 0 ||
            line->find("#BASEINFOA") == 0) {
          return candidate;
        }
      }
    } catch (...) {
    }
  }

  return {};
}

SerialReader::SerialReader(std::string device, int baud, int timeout_ms)
    : device_(std::move(device)), baud_(baud), timeout_ms_(timeout_ms) {}

SerialReader::~SerialReader() {
  close();
}

void SerialReader::open() {
  if (is_open()) {
    return;
  }
  if (device_.empty() || device_ == "auto") {
    device_ = find_wtrtk980_device("/dev/wtrtk980", baud_, timeout_ms_);
    if (device_.empty()) {
      throw std::runtime_error("WTRTK-980 GNSS device not found");
    }
  }
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("failed to open " + device_ + ": " + std::strerror(errno));
  }
  configure_port();
}

void SerialReader::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialReader::configure_port() {
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    throw std::runtime_error("tcgetattr failed for " + device_);
  }
  cfmakeraw(&tty);
  const speed_t speed = baud_to_speed(baud_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= static_cast<unsigned int>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<unsigned int>(~PARENB);
  tty.c_cflag &= static_cast<unsigned int>(~CSTOPB);
  tty.c_cflag &= static_cast<unsigned int>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<unsigned int>(~CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("tcsetattr failed for " + device_);
  }
  tcflush(fd_, TCIOFLUSH);
}

std::optional<std::string> SerialReader::read_line() {
  open();
  std::string line;
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    const int remaining = timeout_ms_ - static_cast<int>(elapsed);
    if (remaining <= 0) {
      return std::nullopt;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    timeval tv{};
    tv.tv_sec = remaining / 1000;
    tv.tv_usec = (remaining % 1000) * 1000;
    const int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      return std::nullopt;
    }

    char c = 0;
    const ssize_t n = ::read(fd_, &c, 1);
    if (n <= 0) {
      continue;
    }
    if (c == '\n') {
      while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
      }
      return line.empty() ? std::nullopt : std::optional<std::string>(line);
    }
    line.push_back(c);
  }
}

#endif

}  // namespace lingtu::drivers::wtrtk980
