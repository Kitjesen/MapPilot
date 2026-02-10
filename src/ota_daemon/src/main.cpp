// ota_daemon — 独立 OTA 守护进程
// 不依赖 ROS2, 纯 gRPC + 文件操作 + systemd 管理
// 端口: 50052 (默认)

#include "ota_service.hpp"
#include "utils.hpp"

#include <csignal>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/server_credentials.h>

static std::unique_ptr<grpc::Server> g_server;

static void SignalHandler(int sig) {
  ota::OtaLogInfo("Received signal %d, shutting down...", sig);
  if (g_server) {
    g_server->Shutdown();
  }
}

static void PrintBanner(const ota::OtaDaemonConfig &config) {
  ota::OtaLogInfo("========================================");
  ota::OtaLogInfo("  OTA Daemon v%s", OTA_DAEMON_VERSION);
  ota::OtaLogInfo("  Port:     %d", config.grpc_port);
  ota::OtaLogInfo("  Robot:    %s (%s)", config.robot_id.c_str(), config.hw_id.c_str());
  ota::OtaLogInfo("  Hostname: %s", ota::GetHostname().c_str());
  auto ips = ota::GetIPAddresses();
  for (const auto &ip : ips) {
    ota::OtaLogInfo("  IP:       %s", ip.c_str());
  }
  ota::OtaLogInfo("  TLS:      %s", config.tls_cert_path.empty() ? "off" : "on");
  ota::OtaLogInfo("  Manifest: %s", config.ota_manifest_path.c_str());
  ota::OtaLogInfo("========================================");
}

int main(int argc, char *argv[]) {
  // 解析命令行
  std::string config_path = "/opt/robot/ota/ota_daemon.yaml";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      fprintf(stdout,
              "Usage: ota_daemon [--config <path>]\n"
              "\n"
              "Options:\n"
              "  --config, -c  Path to YAML config file\n"
              "                Default: /opt/robot/ota/ota_daemon.yaml\n"
              "  --help, -h    Show this help\n");
      return 0;
    }
  }

  // 加载配置
  ota::OtaDaemonConfig config;
  if (ota::FileExists(config_path)) {
    config = ota::LoadConfig(config_path);
    ota::OtaLogInfo("Loaded config from %s", config_path.c_str());
  } else {
    ota::OtaLogWarn("Config not found at %s, using defaults", config_path.c_str());
  }

  // 设置日志级别
  if (config.log_level == "DEBUG") ota::SetLogLevel(ota::LogLevel::kDebug);
  else if (config.log_level == "WARN") ota::SetLogLevel(ota::LogLevel::kWarn);
  else if (config.log_level == "ERROR") ota::SetLogLevel(ota::LogLevel::kError);
  else ota::SetLogLevel(ota::LogLevel::kInfo);

  PrintBanner(config);

  // 信号处理
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  // 创建 OTA 服务
  ota::OtaServiceImpl ota_service(config);

  // 构建 gRPC Server
  std::string listen_addr = config.bind_address + ":" + std::to_string(config.grpc_port);

  grpc::ServerBuilder builder;

  // TLS 配置
  std::shared_ptr<grpc::ServerCredentials> creds;
  if (!config.tls_cert_path.empty() && !config.tls_key_path.empty()) {
    std::string cert = ota::ReadFileToString(config.tls_cert_path);
    std::string key = ota::ReadFileToString(config.tls_key_path);
    if (!cert.empty() && !key.empty()) {
      grpc::SslServerCredentialsOptions ssl_opts;
      ssl_opts.pem_key_cert_pairs.push_back({key, cert});
      creds = grpc::SslServerCredentials(ssl_opts);
      ota::OtaLogInfo("TLS enabled");
    } else {
      ota::OtaLogWarn("TLS cert/key empty, falling back to insecure");
      creds = grpc::InsecureServerCredentials();
    }
  } else {
    creds = grpc::InsecureServerCredentials();
  }

  builder.AddListeningPort(listen_addr, creds);
  builder.RegisterService(&ota_service);

  // gRPC 参数
  builder.SetMaxReceiveMessageSize(256 * 1024 * 1024);  // 256MB (大模型文件)
  builder.SetMaxSendMessageSize(64 * 1024 * 1024);

  g_server = builder.BuildAndStart();
  if (!g_server) {
    ota::OtaLogError("Failed to start gRPC server on %s", listen_addr.c_str());
    return 1;
  }

  ota::OtaLogInfo("OTA Daemon listening on %s", listen_addr.c_str());
  g_server->Wait();

  ota::OtaLogInfo("OTA Daemon shutdown complete");
  return 0;
}
