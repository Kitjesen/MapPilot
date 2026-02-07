#pragma once

#ifdef WEBRTC_ENABLED

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <rtc/rtc.hpp>  // libdatachannel

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace remote_monitoring {

// Forward declaration
class WebRTCBridge;

/**
 * @brief WebRTC PeerConnection 封装类
 * 
 * 管理单个 WebRTC 连接，包括：
 * - SDP offer/answer 交换
 * - ICE candidate 处理
 * - 视频轨道发送
 */
class WebRTCPeer {
public:
  using OnLocalDescriptionCallback = std::function<void(const std::string &sdp, const std::string &type)>;
  using OnLocalCandidateCallback = std::function<void(const std::string &candidate, const std::string &mid)>;
  using OnStateChangeCallback = std::function<void(rtc::PeerConnection::State state)>;
  using OnGatheringStateChangeCallback = std::function<void(rtc::PeerConnection::GatheringState state)>;

  explicit WebRTCPeer(const std::string &session_id);
  ~WebRTCPeer();

  // 初始化 PeerConnection
  bool Initialize(const rtc::Configuration &config);

  // SDP 处理
  void SetRemoteDescription(const std::string &sdp, const std::string &type);
  void CreateAnswer();
  
  // ICE 候选处理
  void AddRemoteCandidate(const std::string &candidate, const std::string &mid);

  // 视频数据发送
  void SendVideoFrame(const uint8_t *data, size_t size, uint64_t timestamp_us);

  // 状态查询
  bool IsConnected() const;
  const std::string &GetSessionId() const { return session_id_; }

  // 回调设置
  void SetOnLocalDescription(OnLocalDescriptionCallback cb);
  void SetOnLocalCandidate(OnLocalCandidateCallback cb);
  void SetOnStateChange(OnStateChangeCallback cb);
  void SetOnGatheringStateChange(OnGatheringStateChangeCallback cb);

  // 关闭连接
  void Close();

private:
  std::string session_id_;
  std::shared_ptr<rtc::PeerConnection> peer_connection_;
  std::shared_ptr<rtc::Track> video_track_;
  
  OnLocalDescriptionCallback on_local_description_;
  OnLocalCandidateCallback on_local_candidate_;
  OnStateChangeCallback on_state_change_;
  OnGatheringStateChangeCallback on_gathering_state_change_;
  
  std::atomic<bool> connected_{false};
  mutable std::mutex mutex_;
};

/**
 * @brief WebRTC 桥接器
 * 
 * 负责：
 * - 管理多个 PeerConnection
 * - 订阅 ROS2 camera topic
 * - 将图像数据编码并发送到所有活动连接
 */
class WebRTCBridge {
public:
  using OnLocalDescriptionCallback = std::function<void(const std::string &session_id, const std::string &sdp, const std::string &type)>;
  using OnLocalCandidateCallback = std::function<void(const std::string &session_id, const std::string &candidate, const std::string &mid)>;
  using OnConnectionStateCallback = std::function<void(const std::string &session_id, const std::string &state)>;
  using OnIceGatheringDoneCallback = std::function<void(const std::string &session_id)>;

  explicit WebRTCBridge(rclcpp::Node *node);
  ~WebRTCBridge();

  // 禁止拷贝
  WebRTCBridge(const WebRTCBridge &) = delete;
  WebRTCBridge &operator=(const WebRTCBridge &) = delete;

  // 初始化（设置 STUN/TURN 服务器等）
  bool Initialize();

  // 创建新的 peer 连接
  std::shared_ptr<WebRTCPeer> CreatePeer(const std::string &session_id);

  // 获取已存在的 peer
  std::shared_ptr<WebRTCPeer> GetPeer(const std::string &session_id);

  // 移除 peer 连接
  void RemovePeer(const std::string &session_id);

  // 处理来自客户端的 offer
  void HandleOffer(const std::string &session_id, const std::string &sdp);

  // 处理来自客户端的 ICE candidate
  void HandleIceCandidate(const std::string &session_id, 
                          const std::string &candidate, 
                          const std::string &mid);

  // 回调设置
  void SetOnLocalDescription(OnLocalDescriptionCallback cb);
  void SetOnLocalCandidate(OnLocalCandidateCallback cb);
  void SetOnConnectionState(OnConnectionStateCallback cb);
  void SetOnIceGatheringDone(OnIceGatheringDoneCallback cb);

  // 订阅摄像头 topic
  void SubscribeCameraTopic(const std::string &topic);
  void UnsubscribeCameraTopic();

  // 获取活动连接数
  size_t GetActivePeerCount() const;

private:
  // ROS2 图像回调
  void OnCompressedImageReceived(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void OnRawImageReceived(const sensor_msgs::msg::Image::SharedPtr msg);

  // 向所有活动 peer 广播帧
  void BroadcastFrame(const uint8_t *data, size_t size, uint64_t timestamp_us);

  // H.264 编码 (raw RGB/BGR -> H.264 NAL)
  // 需要 libx264 支持，否则返回空
  std::vector<uint8_t> EncodeFrame(const uint8_t *data, size_t size, int width, int height);

  // x264 编码器初始化/清理
  bool InitEncoder(int width, int height);
  void DestroyEncoder();

  rclcpp::Node *node_;
  rtc::Configuration rtc_config_;
  
  mutable std::mutex peers_mutex_;
  std::unordered_map<std::string, std::shared_ptr<WebRTCPeer>> peers_;

  // ROS2 订阅
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  std::string current_camera_topic_;
  
  // 回调
  OnLocalDescriptionCallback on_local_description_;
  OnLocalCandidateCallback on_local_candidate_;
  OnConnectionStateCallback on_connection_state_;
  OnIceGatheringDoneCallback on_ice_gathering_done_;

  std::atomic<bool> initialized_{false};

  // x264 编码器状态
  struct x264_t *encoder_{nullptr};
  int encoder_width_{0};
  int encoder_height_{0};
  int64_t frame_count_{0};
};

}  // namespace remote_monitoring

#endif  // WEBRTC_ENABLED
