#pragma once
/**
 * FlightRecorder — 飞行数据记录器 / 黑盒（全新子系统）
 *
 * 从零创建，灵感来自航空 FDR (Flight Data Recorder)。
 *
 * 设计理念:
 *   - 环形缓冲区持续记录最近 N 秒的机器人状态快照 (10Hz × 30s = 300 帧)
 *   - 事故发生时（急停/定位丢失/任务失败）冻结缓冲区，
 *     继续录制 post-trigger 窗口，然后 dump 到二进制文件
 *   - dump 文件可通过现有 DataService 文件下载 RPC 回传分析
 *   - POD 快照设计: 64 bytes/帧, 缓冲区 < 20KB, 零开销
 *
 * 快照内容 (64 bytes per frame):
 *   时间戳, 位姿(x,y,z,rpy), 速度(vx,vy,wz),
 *   ICP score, 定位健康分, 电量, CPU温度,
 *   模式, 健康等级, TF状态, 急停状态
 *
 * 文件格式: [FlightDumpHeader 256B] + [FlightSnapshot × N]
 *
 * 线程安全:
 *   - RecordSnapshot: 由单一定时器线程调用, ring_mutex_ 保护
 *   - TriggerDump: 可从任意线程调用, ring_mutex_ + dump_mutex_ 保护
 *   - 文件 I/O 在 ring_mutex_ 外完成, 不阻塞录制
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace remote_monitoring {
namespace core {

// ==================== 快照结构 (POD, 64 bytes) ====================

#pragma pack(push, 1)
struct FlightSnapshot {
  double timestamp_sec;      // 8B  Unix epoch seconds
  float  x, y, z;            // 12B odom frame position
  float  roll, pitch, yaw;   // 12B degrees
  float  vx, vy, wz;         // 12B body frame velocity (m/s, rad/s)
  float  icp_fitness;         // 4B  raw ICP score (lower=better)
  float  loc_health_score;    // 4B  0-100 from LocalizationScorer
  float  battery_percent;     // 4B  0-100
  float  cpu_temp;            // 4B  degrees C
  uint8_t mode;               // 1B  RobotMode enum value
  uint8_t health_level;       // 1B  HealthLevel enum value
  uint8_t tf_ok;              // 1B  bool
  uint8_t estop;              // 1B  bool
};
#pragma pack(pop)

static_assert(sizeof(FlightSnapshot) == 64,
              "FlightSnapshot must be exactly 64 bytes");

// ==================== Dump 文件头 (256 bytes) ====================

#pragma pack(push, 1)
struct FlightDumpHeader {
  char     magic[8];            // "FLTREC01"
  uint32_t version;             // 1
  uint32_t snapshot_count;      // 快照数
  double   trigger_timestamp;   // Unix epoch
  double   first_timestamp;     // 首帧时间
  double   last_timestamp;      // 末帧时间
  char     trigger_reason[128]; // UTF-8 触发原因  (40+128=168)
  char     reserved[88];        // 保留 (168+88=256)
};
#pragma pack(pop)

static_assert(sizeof(FlightDumpHeader) == 256,
              "FlightDumpHeader must be exactly 256 bytes");

// ==================== Dump 文件信息 ====================

struct DumpFileInfo {
  std::string filename;      // 文件名 (不含路径)
  std::string full_path;     // 完整路径
  std::string reason;        // 触发原因
  double      trigger_time;  // Unix epoch
  uint32_t    snapshot_count;
  size_t      file_size_bytes;
};

// ==================== FlightRecorder ====================

class FlightRecorder {
public:
  /// @param capacity 环形缓冲容量 (快照数, 默认 300 = 30s @ 10Hz)
  /// @param dump_directory dump 文件存储目录
  /// @param post_trigger_count 触发后继续录制的帧数 (默认 50 = 5s @ 10Hz)
  explicit FlightRecorder(
      size_t capacity = 300,
      const std::string &dump_directory = "/opt/robot/flight_records",
      size_t post_trigger_count = 50);

  /// 记录一帧快照
  /// 热路径 (10Hz), 仅 memcpy + 计数器自增, 无文件 I/O
  void RecordSnapshot(const FlightSnapshot &snap);

  /// 触发 dump
  /// @param reason 触发原因 (如 "estop", "loc_lost", "task_failed")
  /// @return dump 文件完整路径; 空字符串 = 触发失败 (冷却中或已触发)
  std::string TriggerDump(const std::string &reason);

  /// 列出已有 dump 文件 (读取文件头)
  std::vector<DumpFileInfo> ListDumps() const;

  /// 删除指定 dump 文件
  bool DeleteDump(const std::string &filename);

  /// 是否处于 post-trigger 录制中
  bool IsTriggered() const {
    return triggered_.load(std::memory_order_relaxed);
  }

  /// 统计: 累计录制帧数
  size_t TotalFramesRecorded() const {
    return total_written_.load(std::memory_order_relaxed);
  }

private:
  std::string GenerateDumpFilename(const std::string &reason) const;
  void WriteDumpFile(const std::vector<FlightSnapshot> &snapshots,
                     const std::string &reason,
                     double trigger_time,
                     const std::string &filepath);

  // 环形缓冲
  std::mutex ring_mutex_;
  std::vector<FlightSnapshot> ring_;
  size_t capacity_;
  size_t write_pos_{0};
  std::atomic<size_t> total_written_{0};

  // 触发状态
  std::atomic<bool> triggered_{false};
  size_t post_trigger_count_;
  size_t post_trigger_remaining_{0};
  double trigger_timestamp_{0.0};
  std::string pending_reason_;

  // dump 冷却 (防止高频触发)
  std::mutex dump_mutex_;
  std::chrono::steady_clock::time_point last_dump_time_;
  static constexpr double kCooldownSec = 10.0;

  // 存储目录
  std::string dump_directory_;
};

}  // namespace core
}  // namespace remote_monitoring
