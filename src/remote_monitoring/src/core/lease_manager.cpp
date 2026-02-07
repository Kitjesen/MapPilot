#include "remote_monitoring/core/lease_manager.hpp"

#include <random>
#include <sstream>
#include <iomanip>

namespace remote_monitoring {
namespace core {

LeaseManager::LeaseManager() {}

bool LeaseManager::AcquireLease(const std::string &holder_id, robot::v1::OperatorLease *lease) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  const auto now = std::chrono::system_clock::now();
  
  // 检查是否已有持有者
  if (!current_token_.empty() && expires_at_ > now) {
    return false;  // 已被占用
  }
  
  // 生成新租约
  current_token_ = GenerateToken();
  current_holder_ = holder_id;
  expires_at_ = now + ttl_ms_;
  
  lease->set_lease_token(current_token_);
  lease->set_holder_id(holder_id);
  lease->mutable_issued_at()->set_seconds(
    std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
  lease->mutable_expires_at()->set_seconds(
    std::chrono::duration_cast<std::chrono::seconds>(expires_at_.time_since_epoch()).count());
  lease->mutable_ttl()->set_seconds(ttl_ms_.count() / 1000);
  
  return true;
}

bool LeaseManager::RenewLease(const std::string &lease_token, robot::v1::OperatorLease *lease) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (lease_token != current_token_) {
    return false;
  }
  
  const auto now = std::chrono::system_clock::now();
  expires_at_ = now + ttl_ms_;
  
  lease->set_lease_token(current_token_);
  lease->set_holder_id(current_holder_);
  lease->mutable_expires_at()->set_seconds(
    std::chrono::duration_cast<std::chrono::seconds>(expires_at_.time_since_epoch()).count());
  lease->mutable_ttl()->set_seconds(ttl_ms_.count() / 1000);
  
  return true;
}

bool LeaseManager::ReleaseLease(const std::string &lease_token) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (lease_token == current_token_) {
    current_token_.clear();
    current_holder_.clear();
    return true;
  }
  
  return false;
}

bool LeaseManager::ValidateLease(const std::string &lease_token) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (lease_token != current_token_) {
    return false;
  }
  
  const auto now = std::chrono::system_clock::now();
  return expires_at_ > now;
}

bool LeaseManager::HasActiveLease() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_token_.empty()) {
    return false;
  }
  return expires_at_ > std::chrono::system_clock::now();
}

void LeaseManager::CheckTimeout() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  const auto now = std::chrono::system_clock::now();
  if (!current_token_.empty() && expires_at_ <= now) {
    current_token_.clear();
    current_holder_.clear();
  }
}

std::string LeaseManager::GenerateToken() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dis;
  
  std::ostringstream oss;
  oss << "lease_" << std::hex << std::setfill('0') << std::setw(16) << dis(gen);
  return oss.str();
}

}  // namespace core
}  // namespace remote_monitoring
