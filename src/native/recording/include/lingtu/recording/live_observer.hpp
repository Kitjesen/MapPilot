#pragma once

#include <memory>
#include <string>
#include <vector>

namespace lingtu::recording {

// One read-only DDS participant. Counts describe samples delivered to this
// observer, not the sensor's promised rate or network transport overhead.
class LiveObserver {
 public:
  explicit LiveObserver(int domain, const std::vector<std::string> &topics = {});
  ~LiveObserver();
  LiveObserver(const LiveObserver &) = delete;
  LiveObserver &operator=(const LiveObserver &) = delete;
  void poll();
  std::string snapshot();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::recording
