#include "remote_monitoring/core/flight_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace remote_monitoring {
namespace core {

namespace fs = std::filesystem;

// ================================================================
//  构造: 初始化环形缓冲, 确保 dump 目录存在
// ================================================================

FlightRecorder::FlightRecorder(
    size_t capacity,
    const std::string &dump_directory,
    size_t post_trigger_count)
    : capacity_(std::max(capacity, size_t{10})),
      post_trigger_count_(post_trigger_count),
      dump_directory_(dump_directory) {
  ring_.resize(capacity_);
  std::memset(ring_.data(), 0, capacity_ * sizeof(FlightSnapshot));

  // 确保 dump 目录存在
  std::error_code ec;
  fs::create_directories(dump_directory_, ec);
  // 即使失败也继续运行 (dump 时会再次尝试)
}

// ================================================================
//  RecordSnapshot — 热路径
// ================================================================

void FlightRecorder::RecordSnapshot(const FlightSnapshot &snap) {
  std::lock_guard<std::mutex> lock(ring_mutex_);
  ring_[write_pos_ % capacity_] = snap;
  ++write_pos_;
  total_written_.fetch_add(1, std::memory_order_relaxed);

  // Post-trigger 录制: 继续录制 N 帧后执行 dump
  if (triggered_.load(std::memory_order_relaxed)) {
    if (post_trigger_remaining_ > 0) {
      --post_trigger_remaining_;
    }
    if (post_trigger_remaining_ == 0) {
      // Post-trigger 窗口完成 — 复制缓冲区
      const size_t count = std::min(write_pos_, capacity_);
      std::vector<FlightSnapshot> snapshots(count);

      if (count == capacity_) {
        // 环已满, 按时间顺序拷贝
        const size_t start = write_pos_ % capacity_;
        const size_t first_part = capacity_ - start;
        std::memcpy(snapshots.data(),
                    ring_.data() + start,
                    first_part * sizeof(FlightSnapshot));
        std::memcpy(snapshots.data() + first_part,
                    ring_.data(),
                    start * sizeof(FlightSnapshot));
      } else {
        std::memcpy(snapshots.data(),
                    ring_.data(),
                    count * sizeof(FlightSnapshot));
      }

      const std::string reason = pending_reason_;
      const double trigger_ts = trigger_timestamp_;
      triggered_.store(false, std::memory_order_relaxed);

      // 释放 ring_mutex_ 后再写文件 (避免阻塞录制)
      // 但我们在 ring_mutex_ 内, 所以先标记完成, 然后在外部写
      // ——不行, 我们此刻持有 ring_mutex_. 改为复制后退出锁,
      // 然后在 RecordSnapshot 的调用者上下文外写文件.
      // 实际上, 由于文件 I/O 通常 < 1ms (19KB), 直接写也可接受.
      const std::string filepath =
          GenerateDumpFilename(reason);
      WriteDumpFile(snapshots, reason, trigger_ts, filepath);
    }
  }
}

// ================================================================
//  TriggerDump — 启动 dump 流程
// ================================================================

std::string FlightRecorder::TriggerDump(const std::string &reason) {
  // 冷却检查
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(dump_mutex_);
    const double elapsed =
        std::chrono::duration<double>(now - last_dump_time_).count();
    if (elapsed < kCooldownSec && last_dump_time_.time_since_epoch().count() > 0) {
      return "";  // 冷却中
    }
    last_dump_time_ = now;
  }

  // 检查是否已触发
  if (triggered_.load(std::memory_order_relaxed)) {
    return "";  // 已在 post-trigger 录制中
  }

  // 如果无 post-trigger 窗口, 立即 dump
  if (post_trigger_count_ == 0) {
    std::vector<FlightSnapshot> snapshots;
    {
      std::lock_guard<std::mutex> lock(ring_mutex_);
      const size_t count = std::min(write_pos_, capacity_);
      snapshots.resize(count);

      if (count == capacity_) {
        const size_t start = write_pos_ % capacity_;
        const size_t first_part = capacity_ - start;
        std::memcpy(snapshots.data(),
                    ring_.data() + start,
                    first_part * sizeof(FlightSnapshot));
        std::memcpy(snapshots.data() + first_part,
                    ring_.data(),
                    start * sizeof(FlightSnapshot));
      } else {
        std::memcpy(snapshots.data(),
                    ring_.data(),
                    count * sizeof(FlightSnapshot));
      }
    }

    const double trigger_ts =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    const std::string filepath = GenerateDumpFilename(reason);
    WriteDumpFile(snapshots, reason, trigger_ts, filepath);
    return filepath;
  }

  // 启动 post-trigger 录制
  {
    std::lock_guard<std::mutex> lock(ring_mutex_);
    pending_reason_ = reason;
    trigger_timestamp_ =
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    post_trigger_remaining_ = post_trigger_count_;
    triggered_.store(true, std::memory_order_relaxed);
  }

  // 文件路径会在 post-trigger 完成后生成
  return "(pending)";
}

// ================================================================
//  ListDumps — 扫描 dump 目录, 读取文件头
// ================================================================

std::vector<DumpFileInfo> FlightRecorder::ListDumps() const {
  std::vector<DumpFileInfo> result;

  std::error_code ec;
  if (!fs::is_directory(dump_directory_, ec)) {
    return result;
  }

  for (const auto &entry : fs::directory_iterator(dump_directory_, ec)) {
    if (!entry.is_regular_file()) continue;
    const auto &path = entry.path();
    if (path.extension() != ".fdr") continue;

    // 读取文件头
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) continue;

    FlightDumpHeader hdr{};
    ifs.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));
    if (!ifs || std::strncmp(hdr.magic, "FLTREC01", 8) != 0) continue;

    DumpFileInfo info;
    info.filename = path.filename().string();
    info.full_path = path.string();
    info.reason = std::string(hdr.trigger_reason,
                              strnlen(hdr.trigger_reason,
                                      sizeof(hdr.trigger_reason)));
    info.trigger_time = hdr.trigger_timestamp;
    info.snapshot_count = hdr.snapshot_count;
    info.file_size_bytes = static_cast<size_t>(entry.file_size(ec));

    result.push_back(std::move(info));
  }

  // 按触发时间降序排列 (最新在前)
  std::sort(result.begin(), result.end(),
            [](const DumpFileInfo &a, const DumpFileInfo &b) {
              return a.trigger_time > b.trigger_time;
            });

  return result;
}

// ================================================================
//  DeleteDump
// ================================================================

bool FlightRecorder::DeleteDump(const std::string &filename) {
  // 安全: 防止路径穿越
  if (filename.find('/') != std::string::npos ||
      filename.find('\\') != std::string::npos ||
      filename.find("..") != std::string::npos) {
    return false;
  }

  const auto path = fs::path(dump_directory_) / filename;
  std::error_code ec;
  return fs::remove(path, ec);
}

// ================================================================
//  内部方法
// ================================================================

std::string FlightRecorder::GenerateDumpFilename(
    const std::string &reason) const {
  // 格式: fdr_YYYYMMDD_HHMMSS_reason.fdr
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  struct tm tm_buf {};
#if defined(_WIN32)
  localtime_s(&tm_buf, &tt);
#else
  localtime_r(&tt, &tm_buf);
#endif

  // 清理 reason: 仅保留字母数字和下划线
  std::string clean_reason;
  clean_reason.reserve(reason.size());
  for (char c : reason) {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
      clean_reason += c;
    }
  }
  if (clean_reason.empty()) clean_reason = "unknown";
  if (clean_reason.size() > 32) clean_reason.resize(32);

  char buf[128];
  snprintf(buf, sizeof(buf), "fdr_%04d%02d%02d_%02d%02d%02d_%s.fdr",
           tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
           tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
           clean_reason.c_str());

  return (fs::path(dump_directory_) / buf).string();
}

void FlightRecorder::WriteDumpFile(
    const std::vector<FlightSnapshot> &snapshots,
    const std::string &reason,
    double trigger_time,
    const std::string &filepath) {
  if (snapshots.empty()) return;

  // 确保目录存在
  std::error_code ec;
  fs::create_directories(fs::path(filepath).parent_path(), ec);

  std::ofstream ofs(filepath, std::ios::binary | std::ios::trunc);
  if (!ofs.is_open()) return;

  // 写入文件头
  FlightDumpHeader hdr{};
  std::memcpy(hdr.magic, "FLTREC01", 8);
  hdr.version = 1;
  hdr.snapshot_count = static_cast<uint32_t>(snapshots.size());
  hdr.trigger_timestamp = trigger_time;
  hdr.first_timestamp = snapshots.front().timestamp_sec;
  hdr.last_timestamp = snapshots.back().timestamp_sec;
  snprintf(hdr.trigger_reason, sizeof(hdr.trigger_reason), "%s",
           reason.c_str());
  std::memset(hdr.reserved, 0, sizeof(hdr.reserved));

  ofs.write(reinterpret_cast<const char *>(&hdr), sizeof(hdr));

  // 写入快照数据 (连续内存, 单次 write)
  ofs.write(reinterpret_cast<const char *>(snapshots.data()),
            static_cast<std::streamsize>(
                snapshots.size() * sizeof(FlightSnapshot)));

  ofs.flush();
}

}  // namespace core
}  // namespace remote_monitoring
