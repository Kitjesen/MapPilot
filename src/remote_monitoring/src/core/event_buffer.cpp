#include "remote_monitoring/core/event_buffer.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace remote_monitoring {
namespace core {

EventBuffer::EventBuffer(size_t max_size) : max_size_(max_size) {}

EventBuffer::~EventBuffer() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (log_stream_.is_open()) {
    log_stream_.flush();
    log_stream_.close();
  }
}

void EventBuffer::EnablePersistence(const std::string &log_path,
                                    size_t max_bytes) {
  std::lock_guard<std::mutex> lock(mutex_);
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
  std::lock_guard<std::mutex> lock(mutex_);
  
  robot::v1::Event event;
  event.set_event_id(GenerateEventId());
  event.set_type(type);
  event.set_severity(severity);
  event.set_title(title);
  event.set_description(description);
  
  const auto now = std::chrono::system_clock::now();
  const auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  event.mutable_timestamp()->set_seconds(now_sec);
  
  buffer_.push_back(event);
  
  // 保持缓冲区大小
  while (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }

  // 持久化到磁盘 (best-effort)
  PersistEventLocked(event);

  cv_.notify_all();
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

  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, timeout, [&]() {
        return NextIndexLocked(last_event_id) != buffer_.size();
      })) {
    return false;
  }

  const size_t idx = NextIndexLocked(last_event_id);
  if (idx >= buffer_.size()) {
    return false;
  }

  *event = buffer_[idx];
  return true;
}

std::string EventBuffer::GenerateEventId() {
  std::ostringstream oss;
  const auto now = std::chrono::system_clock::now();
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
  oss << std::hex << std::setfill('0') << std::setw(12) << now_ms
      << std::setw(6) << next_sequence_++;
  return oss.str();
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

void EventBuffer::PersistEventLocked(const robot::v1::Event &event) {
  if (log_path_.empty() || !log_stream_.is_open()) return;

  try {
    // 日志轮转检查
    if (log_max_bytes_ > 0 && log_current_bytes_ > log_max_bytes_) {
      RotateLogLocked();
    }

    // severity → 字符串
    static const char *sev_names[] = {
        "UNSPECIFIED", "INFO", "WARNING", "ERROR", "CRITICAL"};
    int sev_idx = static_cast<int>(event.severity());
    if (sev_idx < 0 || sev_idx > 4) sev_idx = 0;

    // 简单 JSON 序列化 (避免引入 JSON 库, 转义双引号和反斜杠)
    auto escape_json = [](const std::string &s) -> std::string {
      std::string out;
      out.reserve(s.size() + 8);
      for (char c : s) {
        if (c == '"')
          out += "\\\"";
        else if (c == '\\')
          out += "\\\\";
        else if (c == '\n')
          out += "\\n";
        else if (c == '\r')
          out += "\\r";
        else
          out += c;
      }
      return out;
    };

    std::ostringstream line;
    line << "{\"id\":\"" << event.event_id() << "\""
         << ",\"ts\":" << event.timestamp().seconds()
         << ",\"type\":" << static_cast<int>(event.type())
         << ",\"severity\":\"" << sev_names[sev_idx] << "\""
         << ",\"title\":\"" << escape_json(event.title()) << "\""
         << ",\"desc\":\"" << escape_json(event.description()) << "\""
         << "}\n";

    const std::string s = line.str();
    log_stream_ << s;
    log_stream_.flush();
    log_current_bytes_ += s.size();
  } catch (...) {
    // best-effort: 持久化失败不影响内存事件
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
