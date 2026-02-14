#include "remote_monitoring/core/event_buffer.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>

namespace remote_monitoring {
namespace core {

EventBuffer::EventBuffer(size_t max_size) : max_size_(max_size) {}

EventBuffer::~EventBuffer() {
  std::lock_guard<std::mutex> lock(persist_mutex_);
  if (log_stream_.is_open()) {
    log_stream_.flush();
    log_stream_.close();
  }
}

void EventBuffer::EnablePersistence(const std::string &log_path,
                                    size_t max_bytes) {
  std::lock_guard<std::mutex> lock(persist_mutex_);
  log_path_ = log_path;
  log_max_bytes_ = max_bytes;

  // 确保目录存在
  try {
    auto parent = std::filesystem::path(log_path).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
  } catch (...) {
    // best-effort
  }

  log_stream_.open(log_path, std::ios::app);
  if (log_stream_.is_open()) {
    // 获取当前文件大小
    log_stream_.seekp(0, std::ios::end);
    log_current_bytes_ = static_cast<size_t>(log_stream_.tellp());
  }
}

void EventBuffer::AddEvent(robot::v1::EventType type,
                           robot::v1::EventSeverity severity,
                           const std::string &title,
                           const std::string &description) {
  std::string persist_line;  // 在锁外执行 I/O

  {
    std::lock_guard<std::mutex> lock(mutex_);

    robot::v1::Event event;
    event.set_event_id(GenerateEventId());
    event.set_type(type);
    event.set_severity(severity);
    event.set_title(title);
    event.set_description(description);

    const auto now = std::chrono::system_clock::now();
    const auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()).count();
    event.mutable_timestamp()->set_seconds(now_sec);

    buffer_.push_back(event);

    // 保持缓冲区大小
    while (buffer_.size() > max_size_) {
      buffer_.pop_front();
    }

    // 在锁内序列化为字符串，锁外写磁盘
    persist_line = FormatEventLine(event);

    cv_.notify_all();
  }
  // mutex_ 已释放 — 磁盘 I/O 不阻塞 ROS executor 线程上的其他回调
  PersistLine(persist_line);
}

std::vector<robot::v1::Event> EventBuffer::GetEventsSince(const std::string &last_event_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<robot::v1::Event> result;
  bool found_last = last_event_id.empty();
  
  for (const auto &event : buffer_) {
    if (found_last) {
      result.push_back(event);
    } else if (event.event_id() == last_event_id) {
      found_last = true;
    }
  }
  
  return result;
}

std::vector<robot::v1::Event> EventBuffer::GetLatestEvents(size_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  std::vector<robot::v1::Event> result;
  const size_t start = buffer_.size() > count ? buffer_.size() - count : 0;
  
  for (size_t i = start; i < buffer_.size(); ++i) {
    result.push_back(buffer_[i]);
  }
  
  return result;
}

void EventBuffer::AckEvent(const std::string &event_id) {
  (void)event_id;
  // 可选：记录已确认的事件（用于审计）
}

bool EventBuffer::WaitForEventAfter(const std::string &last_event_id,
                                    robot::v1::Event *event,
                                    std::chrono::milliseconds timeout) {
  if (event == nullptr) {
    return false;
  }

  size_t cached_idx = 0;  // 缓存 predicate 结果，避免二次 O(N) 扫描

  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, timeout, [&]() {
        cached_idx = NextIndexLocked(last_event_id);
        return cached_idx != buffer_.size();
      })) {
    return false;
  }

  if (cached_idx >= buffer_.size()) {
    return false;  // 理论上不会到达，但防御性检查
  }

  *event = buffer_[cached_idx];
  return true;
}

std::string EventBuffer::GenerateEventId() {
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  char buf[20];  // 12 hex + 6 hex + null = 19
  std::snprintf(buf, sizeof(buf), "%012lx%06lx",
                static_cast<unsigned long>(now_ms),
                static_cast<unsigned long>(next_sequence_++));
  return std::string(buf);
}

size_t EventBuffer::NextIndexLocked(const std::string &last_event_id) const {
  if (buffer_.empty()) {
    return buffer_.size();
  }

  if (last_event_id.empty()) {
    return 0;
  }

  for (size_t i = 0; i < buffer_.size(); ++i) {
    if (buffer_[i].event_id() == last_event_id) {
      const size_t next_idx = i + 1;
      return next_idx < buffer_.size() ? next_idx : buffer_.size();
    }
  }

  // last_event_id 已经不在 ring buffer 中时，从最早可用事件继续。
  return 0;
}

// ================================================================
//  持久化: 每个事件追加一行 JSONL 到磁盘
// ================================================================

// 纯函数: 将事件序列化为 JSONL 字符串 (无 I/O, 可在锁内调用)
// 使用直接 string 拼接替代 ostringstream，避免 locale facet 分配。
std::string EventBuffer::FormatEventLine(const robot::v1::Event &event) {
  if (log_path_.empty()) return {};

  static const char *sev_names[] = {
      "UNSPECIFIED", "INFO", "WARNING", "ERROR", "CRITICAL"};
  int sev_idx = static_cast<int>(event.severity());
  if (sev_idx < 0 || sev_idx > 4) sev_idx = 0;

  // 内联 JSON 转义，直接追加到 line，无中间 string 分配
  auto append_escaped = [](std::string &out, const std::string &s) {
    for (char c : s) {
      switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        default:   out += c;      break;
      }
    }
  };

  std::string line;
  line.reserve(128 + event.title().size() + event.description().size());

  line += "{\"id\":\"";
  line += event.event_id();
  line += "\",\"ts\":";
  // std::to_chars 需要 C++17 — 用 snprintf 到栈缓冲
  char num_buf[24];
  std::snprintf(num_buf, sizeof(num_buf), "%ld",
                static_cast<long>(event.timestamp().seconds()));
  line += num_buf;
  line += ",\"type\":";
  std::snprintf(num_buf, sizeof(num_buf), "%d",
                static_cast<int>(event.type()));
  line += num_buf;
  line += ",\"severity\":\"";
  line += sev_names[sev_idx];
  line += "\",\"title\":\"";
  append_escaped(line, event.title());
  line += "\",\"desc\":\"";
  append_escaped(line, event.description());
  line += "\"}\n";

  return line;
}

// 磁盘 I/O: 在 mutex_ 外调用, 不阻塞 ROS executor 的回调链
void EventBuffer::PersistLine(const std::string &line) {
  if (line.empty()) return;

  // persist_mutex_ 保护 log_stream_ (独立于数据 mutex_)
  std::lock_guard<std::mutex> lock(persist_mutex_);
  if (!log_stream_.is_open()) return;

  try {
    // 日志轮转
    if (log_max_bytes_ > 0 && log_current_bytes_ > log_max_bytes_) {
      RotateLogLocked();
    }

    log_stream_ << line;
    // 不再每次 flush — 依赖 OS page cache, 进程退出时 ~EventBuffer 负责 flush。
    // 对于嵌入式系统, 丢失最后几条日志 (断电) 可接受, 换取零阻塞写入。
    log_current_bytes_ += line.size();
  } catch (...) {
    // best-effort
  }
}

void EventBuffer::RotateLogLocked() {
  try {
    log_stream_.close();

    // 旧文件重命名: events.jsonl → events.jsonl.1
    // 如果 .1 已存在则覆盖 (只保留一个历史文件)
    const std::string rotated = log_path_ + ".1";
    std::filesystem::rename(log_path_, rotated);

    // 重新打开
    log_stream_.open(log_path_, std::ios::app);
    log_current_bytes_ = 0;
  } catch (...) {
    // 轮转失败时, 尝试重新以 append 模式打开原文件
    log_stream_.open(log_path_, std::ios::app);
  }
}

}  // namespace core
}  // namespace remote_monitoring
