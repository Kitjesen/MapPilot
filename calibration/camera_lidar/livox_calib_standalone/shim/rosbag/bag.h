// rosbag shim — no-op stubs for standalone build
#ifndef ROSBAG_BAG_HPP
#define ROSBAG_BAG_HPP
#include <string>
#include <vector>
#include <stdexcept>
#include <memory>

namespace rosbag {

  namespace bagmode { static const int Read = 1; static const int Write = 2; }

  class BagException : public std::runtime_error {
  public:
    BagException(const std::string& msg) : std::runtime_error(msg) {}
  };

  class MessageInstance {
  public:
    template<typename T>
    std::shared_ptr<T> instantiate() const { return std::make_shared<T>(); }
  };

  class TopicQuery {
  public:
    TopicQuery(const std::string&) {}
    TopicQuery(const std::vector<std::string>&) {}
  };

  class Bag {
  public:
    void open(const std::string&, int) {}
    void close() {}
    bool isOpen() { return false; }
  };

  class View {
  public:
    View(Bag&, const TopicQuery&) {}
    View(Bag&) {}
    struct iterator {
      bool operator!=(const iterator&) { return false; }
      iterator& operator++() { return *this; }
      MessageInstance operator*() { return MessageInstance(); }
    };
    iterator begin() { return iterator(); }
    iterator end() { return iterator(); }
  };

} // namespace rosbag
#endif
