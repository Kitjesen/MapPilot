#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "data.grpc.pb.h"
#include "grpcpp/grpcpp.h"
#include "rclcpp/rclcpp.hpp"

#ifdef WEBRTC_ENABLED
#include "remote_monitoring/webrtc_bridge.hpp"
#endif

namespace remote_monitoring {
namespace services {

class DataServiceImpl final : public robot::v1::DataService::Service {
public:
  explicit DataServiceImpl(rclcpp::Node *node);

  grpc::Status
  ListResources(grpc::ServerContext *context,
                const google::protobuf::Empty *request,
                robot::v1::ListResourcesResponse *response) override;

  grpc::Status
  Subscribe(grpc::ServerContext *context,
            const robot::v1::SubscribeRequest *request,
            grpc::ServerWriter<robot::v1::DataChunk> *writer) override;

  grpc::Status Unsubscribe(grpc::ServerContext *context,
                           const robot::v1::UnsubscribeRequest *request,
                           robot::v1::UnsubscribeResponse *response) override;

  grpc::Status
  DownloadFile(grpc::ServerContext *context,
               const robot::v1::DownloadFileRequest *request,
               grpc::ServerWriter<robot::v1::FileChunk> *writer) override;

  grpc::Status StartCamera(grpc::ServerContext *context,
                           const robot::v1::StartCameraRequest *request,
                           robot::v1::StartCameraResponse *response) override;

  grpc::Status StopCamera(grpc::ServerContext *context,
                          const robot::v1::StopCameraRequest *request,
                          robot::v1::StopCameraResponse *response) override;

  // WebRTC 双向信令流
  grpc::Status WebRTCSignaling(
      grpc::ServerContext *context,
      grpc::ServerReaderWriter<robot::v1::WebRTCSignal,
                               robot::v1::WebRTCSignal> *stream) override;

private:
  bool IsCompressionSupported(robot::v1::CompressionType compression) const;
  double ResolveFrequency(const robot::v1::SubscribeRequest *request) const;
  std::string ResolveTopic(const robot::v1::SubscribeRequest *request) const;

  bool RegisterSubscription(
      const std::string &subscription_id,
      std::shared_ptr<std::atomic_bool> *cancel_flag);
  void RemoveSubscription(const std::string &subscription_id);

  struct WebrtcSessionInfo {
    std::string camera_id;
    std::string topic;
    std::string offer_path;
    std::string ice_path;
  };

  // WebRTC 双向信令会话状态
  struct WebRTCSession {
    std::string session_id;
    robot::v1::WebRTCSessionConfig config;
    std::queue<robot::v1::WebRTCSignal> outgoing_signals;
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic_bool active{true};
    std::atomic_bool peer_connected{false};
    std::string local_sdp;
    std::string remote_sdp;
    std::vector<std::string> local_ice_candidates;
    std::vector<std::string> remote_ice_candidates;
  };

  rclcpp::Node *node_;
  std::string camera_topic_;
  std::string camera_fallback_topic_;
  std::string map_topic_;
  std::string pointcloud_topic_;
  std::string terrain_topic_;
  std::string file_root_;
  bool webrtc_enabled_{false};
  int webrtc_offer_timeout_ms_{3000};
  std::string webrtc_start_command_;
  std::string webrtc_stop_command_;
  std::string webrtc_offer_path_;
  std::string webrtc_ice_path_;
  mutable std::mutex subscriptions_mutex_;
  std::unordered_map<std::string, std::shared_ptr<std::atomic_bool>>
      active_subscriptions_;
  mutable std::mutex camera_sessions_mutex_;
  std::unordered_set<std::string> active_camera_sessions_;
  mutable std::mutex webrtc_mutex_;
  std::unordered_map<std::string, WebrtcSessionInfo> webrtc_sessions_;
  
  // WebRTC 双向信令会话管理
  mutable std::mutex webrtc_signaling_mutex_;
  std::unordered_map<std::string, std::shared_ptr<WebRTCSession>> webrtc_signaling_sessions_;
  
  // WebRTC 信令处理辅助方法
  std::shared_ptr<WebRTCSession> GetOrCreateWebRTCSession(const std::string &session_id);
  void RemoveWebRTCSession(const std::string &session_id);
  void HandleWebRTCOffer(std::shared_ptr<WebRTCSession> session, const robot::v1::WebRTCSignal &signal);
  void HandleWebRTCAnswer(std::shared_ptr<WebRTCSession> session, const robot::v1::WebRTCSignal &signal);
  void HandleWebRTCIceCandidate(std::shared_ptr<WebRTCSession> session, const robot::v1::WebRTCSignal &signal);

#ifdef WEBRTC_ENABLED
  // WebRTC 媒体桥接器
  std::unique_ptr<WebRTCBridge> webrtc_bridge_;
  void InitializeWebRTCBridge();
  void SendWebRTCSignalToSession(const std::string &session_id, robot::v1::WebRTCSignal signal);
#endif
};

} // namespace services
} // namespace remote_monitoring
