#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class LeaseManager {
public:
  LeaseManager();
  
  // 获取租约
  bool AcquireLease(const std::string &holder_id, robot::v1::OperatorLease *lease);
  
  // 续约
  bool RenewLease(const std::string &lease_token, robot::v1::OperatorLease *lease);
  
  // 释放租约
  bool ReleaseLease(const std::string &lease_token);
  
  // 验证租约
  bool ValidateLease(const std::string &lease_token);

  // 检查是否有任何活跃租约（未过期）
  bool HasActiveLease();
  
  // 检查超时（定期调用）
  void CheckTimeout();

private:
  std::string GenerateToken();
  
  std::mutex mutex_;
  std::string current_token_;
  std::string current_holder_;
  std::chrono::system_clock::time_point expires_at_;
  std::chrono::milliseconds ttl_ms_{30000};  // 30s 默认
};

}  // namespace core
}  // namespace remote_monitoring
