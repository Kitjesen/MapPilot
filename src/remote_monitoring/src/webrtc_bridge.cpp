#ifdef WEBRTC_ENABLED

#include "remote_monitoring/webrtc_bridge.hpp"

#include <chrono>
#include <sstream>

namespace remote_monitoring {

// ==================== WebRTCPeer Implementation ====================

WebRTCPeer::WebRTCPeer(const std::string &session_id)
    : session_id_(session_id) {
  RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"), 
              "Creating WebRTC peer for session: %s", session_id.c_str());
}

WebRTCPeer::~WebRTCPeer() {
  Close();
}

bool WebRTCPeer::Initialize(const rtc::Configuration &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  try {
    peer_connection_ = std::make_shared<rtc::PeerConnection>(config);
    
    // 设置回调
    peer_connection_->onLocalDescription([this](rtc::Description description) {
      std::string sdp = std::string(description);
      std::string type = description.typeString();
      
      RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"),
                  "Local description generated (type: %s)", type.c_str());
      
      if (on_local_description_) {
        on_local_description_(sdp, type);
      }
    });
    
    peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
      std::string cand = std::string(candidate);
      std::string mid = candidate.mid();
      
      RCLCPP_DEBUG(rclcpp::get_logger("WebRTCPeer"),
                   "Local ICE candidate: %s", cand.c_str());
      
      if (on_local_candidate_) {
        on_local_candidate_(cand, mid);
      }
    });
    
    peer_connection_->onStateChange([this](rtc::PeerConnection::State state) {
      RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"),
                  "Connection state changed: %d", static_cast<int>(state));
      
      connected_ = (state == rtc::PeerConnection::State::Connected);
      
      if (on_state_change_) {
        on_state_change_(state);
      }
    });
    
    peer_connection_->onGatheringStateChange([this](rtc::PeerConnection::GatheringState state) {
      RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"),
                  "ICE gathering state changed: %d", static_cast<int>(state));
      
      if (on_gathering_state_change_) {
        on_gathering_state_change_(state);
      }
    });
    
    // 添加视频轨道
    rtc::Description::Video video_desc("video", rtc::Description::Direction::SendOnly);
    video_desc.addH264Codec(96);  // H.264 payload type 96
    video_desc.addSSRC(1, "video-stream");
    
    video_track_ = peer_connection_->addTrack(video_desc);
    
    RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"),
                "WebRTC peer initialized successfully");
    return true;
    
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Failed to initialize peer connection: %s", e.what());
    return false;
  }
}

void WebRTCPeer::SetRemoteDescription(const std::string &sdp, const std::string &type) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!peer_connection_) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Peer connection not initialized");
    return;
  }
  
  try {
    rtc::Description::Type desc_type = (type == "offer") 
        ? rtc::Description::Type::Offer 
        : rtc::Description::Type::Answer;
    
    rtc::Description description(sdp, desc_type);
    peer_connection_->setRemoteDescription(description);
    
    RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"),
                "Remote description set (type: %s)", type.c_str());
                
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Failed to set remote description: %s", e.what());
  }
}

void WebRTCPeer::CreateAnswer() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!peer_connection_) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Peer connection not initialized");
    return;
  }
  
  try {
    peer_connection_->setLocalDescription(rtc::Description::Type::Answer);
    RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"), "Creating answer...");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Failed to create answer: %s", e.what());
  }
}

void WebRTCPeer::AddRemoteCandidate(const std::string &candidate, const std::string &mid) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!peer_connection_) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Peer connection not initialized");
    return;
  }
  
  try {
    rtc::Candidate cand(candidate, mid);
    peer_connection_->addRemoteCandidate(cand);
    
    RCLCPP_DEBUG(rclcpp::get_logger("WebRTCPeer"),
                 "Added remote ICE candidate");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Failed to add remote candidate: %s", e.what());
  }
}

void WebRTCPeer::SendVideoFrame(const uint8_t *data, size_t size, uint64_t /*timestamp_us*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!video_track_ || !connected_) {
    return;
  }
  
  try {
    // 注意：这里假设 data 已经是 H.264 编码的 NAL 单元
    // 如果是原始帧，需要先编码
    auto byte_ptr = reinterpret_cast<const std::byte *>(data);
    video_track_->send(byte_ptr, size);
    
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("WebRTCPeer"),
                 "Failed to send video frame: %s", e.what());
  }
}

bool WebRTCPeer::IsConnected() const {
  return connected_.load();
}

void WebRTCPeer::SetOnLocalDescription(OnLocalDescriptionCallback cb) {
  on_local_description_ = std::move(cb);
}

void WebRTCPeer::SetOnLocalCandidate(OnLocalCandidateCallback cb) {
  on_local_candidate_ = std::move(cb);
}

void WebRTCPeer::SetOnStateChange(OnStateChangeCallback cb) {
  on_state_change_ = std::move(cb);
}

void WebRTCPeer::SetOnGatheringStateChange(OnGatheringStateChangeCallback cb) {
  on_gathering_state_change_ = std::move(cb);
}

void WebRTCPeer::Close() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  connected_ = false;
  
  if (video_track_) {
    video_track_.reset();
  }
  
  if (peer_connection_) {
    peer_connection_->close();
    peer_connection_.reset();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("WebRTCPeer"),
              "WebRTC peer closed: %s", session_id_.c_str());
}

// ==================== WebRTCBridge Implementation ====================

WebRTCBridge::WebRTCBridge(rclcpp::Node *node)
    : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "WebRTCBridge created");
}

WebRTCBridge::~WebRTCBridge() {
  UnsubscribeCameraTopic();
  
  // 关闭所有 peer 连接
  std::lock_guard<std::mutex> lock(peers_mutex_);
  for (auto &[id, peer] : peers_) {
    peer->Close();
  }
  peers_.clear();
  
  RCLCPP_INFO(node_->get_logger(), "WebRTCBridge destroyed");
}

bool WebRTCBridge::Initialize() {
  if (initialized_) {
    return true;
  }
  
  // 从参数获取 STUN/TURN 服务器配置
  std::string stun_server = node_->declare_parameter<std::string>(
      "webrtc.stun_server", "stun:stun.l.google.com:19302");
  
  std::string turn_server = node_->declare_parameter<std::string>(
      "webrtc.turn_server", "");
  std::string turn_username = node_->declare_parameter<std::string>(
      "webrtc.turn_username", "");
  std::string turn_password = node_->declare_parameter<std::string>(
      "webrtc.turn_password", "");
  
  // 配置 ICE 服务器
  rtc_config_.iceServers.push_back(rtc::IceServer(stun_server));
  
  if (!turn_server.empty() && !turn_username.empty()) {
    // TURN server: hostname, service(port), username, password, relay type
    // Parse turn_server which may be "turn:host:port" or just "host:port"
    std::string turn_host = turn_server;
    std::string turn_port = "3478";
    // Strip "turn:" prefix if present
    if (turn_host.find("turn:") == 0) {
      turn_host = turn_host.substr(5);
    }
    // Extract port if present
    auto colon_pos = turn_host.rfind(':');
    if (colon_pos != std::string::npos) {
      turn_port = turn_host.substr(colon_pos + 1);
      turn_host = turn_host.substr(0, colon_pos);
    }
    rtc_config_.iceServers.push_back(
        rtc::IceServer(turn_host, turn_port, turn_username, turn_password,
                       rtc::IceServer::RelayType::TurnUdp));
  }
  
  RCLCPP_INFO(node_->get_logger(), 
              "WebRTCBridge initialized with STUN server: %s", stun_server.c_str());
  
  initialized_ = true;
  return true;
}

std::shared_ptr<WebRTCPeer> WebRTCBridge::CreatePeer(const std::string &session_id) {
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "WebRTCBridge not initialized, cannot create peer");
    return nullptr;
  }
  
  std::lock_guard<std::mutex> lock(peers_mutex_);
  
  // 检查是否已存在
  auto it = peers_.find(session_id);
  if (it != peers_.end()) {
    RCLCPP_WARN(node_->get_logger(),
                "Peer already exists for session: %s", session_id.c_str());
    return it->second;
  }
  
  // 创建新的 peer
  auto peer = std::make_shared<WebRTCPeer>(session_id);
  
  if (!peer->Initialize(rtc_config_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to initialize peer for session: %s", session_id.c_str());
    return nullptr;
  }
  
  // 设置回调以转发到 bridge 级别
  peer->SetOnLocalDescription([this, session_id](const std::string &sdp, const std::string &type) {
    if (on_local_description_) {
      on_local_description_(session_id, sdp, type);
    }
  });
  
  peer->SetOnLocalCandidate([this, session_id](const std::string &candidate, const std::string &mid) {
    if (on_local_candidate_) {
      on_local_candidate_(session_id, candidate, mid);
    }
  });
  
  peer->SetOnStateChange([this, session_id](rtc::PeerConnection::State state) {
    std::string state_str;
    switch (state) {
      case rtc::PeerConnection::State::New: state_str = "new"; break;
      case rtc::PeerConnection::State::Connecting: state_str = "connecting"; break;
      case rtc::PeerConnection::State::Connected: state_str = "connected"; break;
      case rtc::PeerConnection::State::Disconnected: state_str = "disconnected"; break;
      case rtc::PeerConnection::State::Failed: state_str = "failed"; break;
      case rtc::PeerConnection::State::Closed: state_str = "closed"; break;
      default: state_str = "unknown"; break;
    }
    
    if (on_connection_state_) {
      on_connection_state_(session_id, state_str);
    }
  });
  
  peer->SetOnGatheringStateChange([this, session_id](rtc::PeerConnection::GatheringState state) {
    if (state == rtc::PeerConnection::GatheringState::Complete) {
      if (on_ice_gathering_done_) {
        on_ice_gathering_done_(session_id);
      }
    }
  });
  
  peers_[session_id] = peer;
  
  RCLCPP_INFO(node_->get_logger(), 
              "Created WebRTC peer for session: %s (total peers: %zu)",
              session_id.c_str(), peers_.size());
  
  return peer;
}

std::shared_ptr<WebRTCPeer> WebRTCBridge::GetPeer(const std::string &session_id) {
  std::lock_guard<std::mutex> lock(peers_mutex_);
  auto it = peers_.find(session_id);
  return (it != peers_.end()) ? it->second : nullptr;
}

void WebRTCBridge::RemovePeer(const std::string &session_id) {
  std::lock_guard<std::mutex> lock(peers_mutex_);
  
  auto it = peers_.find(session_id);
  if (it != peers_.end()) {
    it->second->Close();
    peers_.erase(it);
    
    RCLCPP_INFO(node_->get_logger(),
                "Removed WebRTC peer: %s (remaining: %zu)",
                session_id.c_str(), peers_.size());
  }
}

void WebRTCBridge::HandleOffer(const std::string &session_id, const std::string &sdp) {
  auto peer = GetPeer(session_id);
  
  if (!peer) {
    peer = CreatePeer(session_id);
    if (!peer) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to create peer for offer: %s", session_id.c_str());
      return;
    }
  }
  
  peer->SetRemoteDescription(sdp, "offer");
  peer->CreateAnswer();
}

void WebRTCBridge::HandleIceCandidate(const std::string &session_id,
                                       const std::string &candidate,
                                       const std::string &mid) {
  auto peer = GetPeer(session_id);
  
  if (!peer) {
    RCLCPP_WARN(node_->get_logger(),
                "No peer found for ICE candidate: %s", session_id.c_str());
    return;
  }
  
  peer->AddRemoteCandidate(candidate, mid);
}

void WebRTCBridge::SetOnLocalDescription(OnLocalDescriptionCallback cb) {
  on_local_description_ = std::move(cb);
}

void WebRTCBridge::SetOnLocalCandidate(OnLocalCandidateCallback cb) {
  on_local_candidate_ = std::move(cb);
}

void WebRTCBridge::SetOnConnectionState(OnConnectionStateCallback cb) {
  on_connection_state_ = std::move(cb);
}

void WebRTCBridge::SetOnIceGatheringDone(OnIceGatheringDoneCallback cb) {
  on_ice_gathering_done_ = std::move(cb);
}

void WebRTCBridge::SubscribeCameraTopic(const std::string &topic) {
  if (current_camera_topic_ == topic && compressed_image_sub_) {
    return;  // 已订阅
  }
  
  UnsubscribeCameraTopic();
  
  current_camera_topic_ = topic;
  
  // 尝试订阅压缩图像 topic
  std::string compressed_topic = topic;
  if (topic.find("/compressed") == std::string::npos) {
    compressed_topic = topic + "/compressed";
  }
  
  try {
    compressed_image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
        compressed_topic, 10,
        std::bind(&WebRTCBridge::OnCompressedImageReceived, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(),
                "Subscribed to compressed image topic: %s", compressed_topic.c_str());
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(),
                "Failed to subscribe to compressed topic, trying raw: %s", e.what());
    
    // 回退到原始图像 topic
    raw_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        topic, 10,
        std::bind(&WebRTCBridge::OnRawImageReceived, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(),
                "Subscribed to raw image topic: %s", topic.c_str());
  }
}

void WebRTCBridge::UnsubscribeCameraTopic() {
  compressed_image_sub_.reset();
  raw_image_sub_.reset();
  current_camera_topic_.clear();
}

size_t WebRTCBridge::GetActivePeerCount() const {
  std::lock_guard<std::mutex> lock(peers_mutex_);
  
  size_t count = 0;
  for (const auto &[id, peer] : peers_) {
    if (peer->IsConnected()) {
      ++count;
    }
  }
  return count;
}

void WebRTCBridge::OnCompressedImageReceived(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  if (!msg || msg->data.empty()) {
    return;
  }
  
  // 将时间戳转换为微秒
  uint64_t timestamp_us = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL +
                          static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000ULL;
  
  // 检查格式（JPEG 可直接传输，后续需要 H.264 编码）
  // 注：实际部署时应使用 H.264 编码器（如 OpenH264 或硬件编码）
  // 这里暂时直接传输压缩数据用于测试
  
  BroadcastFrame(msg->data.data(), msg->data.size(), timestamp_us);
}

void WebRTCBridge::OnRawImageReceived(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (!msg || msg->data.empty()) {
    return;
  }
  
  uint64_t timestamp_us = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000ULL +
                          static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000ULL;
  
  // TODO: 需要先编码为 H.264
  // 当前实现仅为占位，实际需要调用编码器
  auto encoded = EncodeFrame(msg->data.data(), msg->data.size(), 
                             msg->width, msg->height);
  
  if (!encoded.empty()) {
    BroadcastFrame(encoded.data(), encoded.size(), timestamp_us);
  }
}

void WebRTCBridge::BroadcastFrame(const uint8_t *data, size_t size, uint64_t timestamp_us) {
  std::lock_guard<std::mutex> lock(peers_mutex_);
  
  for (auto &[id, peer] : peers_) {
    if (peer->IsConnected()) {
      peer->SendVideoFrame(data, size, timestamp_us);
    }
  }
}

std::vector<uint8_t> WebRTCBridge::EncodeFrame(const uint8_t * /*data*/, size_t /*size*/, 
                                                int /*width*/, int /*height*/) {
  // TODO: 集成 OpenH264 编码器
  // 当前返回空，表示跳过原始帧
  // 实际部署需要：
  // 1. 初始化 OpenH264 编码器 (ISVCEncoder)
  // 2. 将 RGB/BGR/YUV 帧编码为 H.264 NAL 单元
  // 3. 返回编码后的数据
  
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "Raw image encoding not implemented yet, skipping frame");
  
  return {};
}

}  // namespace remote_monitoring

#endif  // WEBRTC_ENABLED
