#pragma once

#include <chrono>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace remote_monitoring {
namespace core {

/**
 * @brief LRU IdempotencyCache — 保证重复 RPC 请求返回相同结果。
 *
 * 数据结构: unordered_map (O(1) 查找) + list (插入序, O(1) 淘汰)。
 *
 * - Set(): O(1) amortized (push_back + map insert; 超限时 pop_front)
 * - TryGet(): O(1) (map find + expiry check)
 * - Cleanup(): O(K) 其中 K = 过期条目数 (从 oldest 端扫描, 遇到未过期即停)
 * - EvictOldest(): O(1) (pop list front + map erase)
 *
 * Thread-safe.
 */
class IdempotencyCache {
public:
  struct CacheEntry {
    std::string request_id;
    std::string response_data;
    std::chrono::steady_clock::time_point timestamp;
    bool is_error{false};
  };

  /**
   * @param ttl_seconds Duration to keep cached responses. Default 10 minutes.
   * @param max_entries Hard cap on cache size. Oldest entries evicted on overflow.
   */
  explicit IdempotencyCache(int ttl_seconds = 600, size_t max_entries = 10000)
      : ttl_(std::chrono::seconds(ttl_seconds)),
        max_entries_(max_entries) {}

  bool TryGet(const std::string &request_id, std::string *out_response) {
    if (request_id.empty()) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = index_.find(request_id);
    if (it == index_.end()) return false;

    auto &entry = *(it->second);
    if (IsExpired(entry)) {
      // 过期: 从 list 和 map 中删除
      order_.erase(it->second);
      index_.erase(it);
      return false;
    }
    if (out_response) {
      *out_response = entry.response_data;
    }
    return true;
  }

  void Set(const std::string &request_id, std::string response_data) {
    if (request_id.empty()) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // 如果已存在, 更新并移到尾部
    auto it = index_.find(request_id);
    if (it != index_.end()) {
      it->second->response_data = std::move(response_data);
      it->second->timestamp = std::chrono::steady_clock::now();
      // 移到 list 尾部 (最新)
      order_.splice(order_.end(), order_, it->second);
      return;
    }

    // 超过容量上限: O(1) 淘汰最旧
    while (index_.size() >= max_entries_ && !order_.empty()) {
      EvictFrontLocked();
    }

    // 插入新条目到 list 尾部
    CacheEntry entry;
    entry.request_id = request_id;
    entry.response_data = std::move(response_data);
    entry.timestamp = std::chrono::steady_clock::now();
    entry.is_error = false;

    order_.push_back(std::move(entry));
    auto list_it = std::prev(order_.end());
    index_[request_id] = list_it;
  }

  /// 清理过期条目 — 从最旧端扫描, 遇到未过期即停 O(K)
  void Cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!order_.empty()) {
      auto &oldest = order_.front();
      if (!IsExpired(oldest)) break;  // list 按时间排序, 后面都更新
      index_.erase(oldest.request_id);
      order_.pop_front();
    }
  }

private:
  bool IsExpired(const CacheEntry &entry) const {
    return (std::chrono::steady_clock::now() - entry.timestamp) > ttl_;
  }

  /// O(1) 淘汰最旧条目 (已持有 mutex_)
  void EvictFrontLocked() {
    auto &oldest = order_.front();
    index_.erase(oldest.request_id);
    order_.pop_front();
  }

  std::mutex mutex_;
  std::list<CacheEntry> order_;                                        // 按插入/更新时间排序
  std::unordered_map<std::string, std::list<CacheEntry>::iterator> index_;  // O(1) 查找
  std::chrono::seconds ttl_;
  size_t max_entries_;
};

} // namespace core
} // namespace remote_monitoring
