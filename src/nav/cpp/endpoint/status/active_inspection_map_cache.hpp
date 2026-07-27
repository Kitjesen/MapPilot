#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace lingtu::nav::endpoint {

struct ActiveInspectionMapIdentity {
  std::string map_id;
  std::int64_t version{0};
};

struct ActiveInspectionMapCacheConfig {
  double poll_interval_s{1.0};
  double stale_after_s{3.0};
};

struct ActiveInspectionMapSnapshot {
  std::optional<ActiveInspectionMapIdentity> identity;
  std::uint64_t sequence{0};
  double refreshed_at_s{0.0};
  bool fresh{false};
  std::string reason{"not_refreshed"};
};

struct ActiveInspectionMapCacheDiagnostics {
  std::uint64_t attempts{0};
  std::uint64_t successes{0};
  std::uint64_t failures{0};
  std::uint64_t identity_changes{0};
  bool reading{false};
  bool running{false};
  std::string last_error;
};

class ActiveInspectionMapCache {
 public:
  using Reader = std::function<std::optional<ActiveInspectionMapIdentity>()>;

  explicit ActiveInspectionMapCache(Reader reader, ActiveInspectionMapCacheConfig config = {});
  ~ActiveInspectionMapCache();

  ActiveInspectionMapCache(const ActiveInspectionMapCache &) = delete;
  ActiveInspectionMapCache &operator=(const ActiveInspectionMapCache &) = delete;

  // Schedules an immediate refresh without blocking the caller.
  void requestRefresh();

  // Waits for refreshes requested before this call and any in-flight read.
  void flush();

  // Interrupts the polling wait and joins the worker. Safe to call repeatedly.
  void stop();

  ActiveInspectionMapSnapshot snapshot(double now_steady_s) const;
  ActiveInspectionMapCacheDiagnostics diagnostics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::nav::endpoint
