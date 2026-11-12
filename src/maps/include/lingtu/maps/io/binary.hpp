#pragma once

#include <cstdint>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace lingtu::maps {

// Numeric payloads use the endianness declared by the owning artifact header.
class BinaryWriter {
 public:
  explicit BinaryWriter(std::ostream& stream) : stream_(stream) {}
  template<class T> void value(const T& value) {
    static_assert(std::is_arithmetic_v<T>);
    bytes(&value,sizeof(value));
  }
  template<class T> void vector(const std::vector<T>& values) {
    static_assert(std::is_arithmetic_v<T>);
    value<std::uint64_t>(values.size());
    if(!values.empty())bytes(values.data(),values.size()*sizeof(T));
  }
  void text(const std::string& text) {
    value<std::uint64_t>(text.size());
    bytes(text.data(),text.size());
  }
 private:
  void bytes(const void* data,std::size_t count) {
    stream_.write(static_cast<const char*>(data),static_cast<std::streamsize>(count));
    if(!stream_)throw std::runtime_error("cannot write prepared map");
  }
  std::ostream& stream_;
};

class BinaryReader {
 public:
  explicit BinaryReader(std::istream& stream) : stream_(stream) {
    const auto begin=stream_.tellg();
    stream_.seekg(0,std::ios::end);const auto end=stream_.tellg();
    if(begin<0 || end<begin)throw std::runtime_error("cannot read prepared map");
    remaining_=static_cast<std::uint64_t>(end-begin);stream_.seekg(begin);
  }
  template<class T> T value() {
    static_assert(std::is_arithmetic_v<T>);
    T result{};bytes(&result,sizeof(result));return result;
  }
  std::size_t count(std::size_t minimum_bytes) {
    const auto count=value<std::uint64_t>();
    if(count>remaining_/minimum_bytes)throw std::runtime_error("truncated prepared map");
    return static_cast<std::size_t>(count);
  }
  template<class T> std::vector<T> vector() {
    static_assert(std::is_arithmetic_v<T>);
    std::vector<T> result(count(sizeof(T)));
    if(!result.empty())bytes(result.data(),result.size()*sizeof(T));
    return result;
  }
  std::string text() {
    std::string result(count(1),'\0');
    if(!result.empty())bytes(result.data(),result.size());
    return result;
  }
  bool complete() const { return remaining_==0; }
 private:
  void bytes(void* data,std::size_t count) {
    if(count>remaining_)throw std::runtime_error("truncated prepared map");
    stream_.read(static_cast<char*>(data),static_cast<std::streamsize>(count));
    if(!stream_)throw std::runtime_error("cannot read prepared map");
    remaining_-=count;
  }
  std::istream& stream_;
  std::uint64_t remaining_{};
};

} // namespace lingtu::maps
