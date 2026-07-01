#include "wtrtk980/serial_reader.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <stdexcept>

#include "wtrtk980/nmea_parser.hpp"
#include "wtrtk980/serial_detector.hpp"

namespace wtrtk980 {
namespace {

speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: throw std::runtime_error("unsupported baud rate: " + std::to_string(baud));
  }
}

}  // namespace

SerialReader::SerialReader(std::string device, int baud, int timeout_ms)
    : device_(std::move(device)), baud_(baud), timeout_ms_(timeout_ms) {}

SerialReader::~SerialReader() { close(); }

void SerialReader::open() {
  if (isOpen()) return;
  if (device_ == "auto") {
    auto result = findWtrtk980Device();
    if (result.device.empty()) throw std::runtime_error("WTRTK-980 not found");
    device_ = result.device;
  }
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) throw std::runtime_error("failed to open " + device_ + ": " + std::strerror(errno));
  configurePort();
}

void SerialReader::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialReader::isOpen() const { return fd_ >= 0; }

void SerialReader::configurePort() {
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) throw std::runtime_error("tcgetattr failed");
  cfmakeraw(&tty);
  speed_t speed = baudToSpeed(baud_);
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
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) throw std::runtime_error("tcsetattr failed");
  tcflush(fd_, TCIOFLUSH);
}

std::optional<std::string> SerialReader::readRawLine() {
  open();
  std::string line;
  auto start = std::chrono::steady_clock::now();
  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    int remaining = timeout_ms_ - static_cast<int>(elapsed);
    if (remaining <= 0) return std::nullopt;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    timeval tv{};
    tv.tv_sec = remaining / 1000;
    tv.tv_usec = (remaining % 1000) * 1000;
    int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) return std::nullopt;

    char c = 0;
    ssize_t n = ::read(fd_, &c, 1);
    if (n <= 0) continue;
    if (c == '\n') {
      while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
      return line.empty() ? std::nullopt : std::optional<std::string>(line);
    }
    line.push_back(c);
  }
}

std::optional<GpsFix> SerialReader::readUpdate(int timeout_ms) {
  int old_timeout = timeout_ms_;
  if (timeout_ms >= 0) timeout_ms_ = timeout_ms;
  auto line = readRawLine();
  if (timeout_ms >= 0) timeout_ms_ = old_timeout;
  if (!line) return std::nullopt;
  if (parseNmeaLine(*line, state_)) return state_;
  return std::nullopt;
}

}  // namespace wtrtk980
