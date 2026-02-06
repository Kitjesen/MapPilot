# Flutter Web 客户端完善计划：gRPC-Web 支持

## 目标

让 Flutter Web 应用能够直接连接到 gRPC 服务，无需 WebSocket 桥接。

---

## 方案：gRPC-Web + Envoy 代理

### 架构图

```
Flutter Web (浏览器)
    ↓ HTTP/1.1 + gRPC-Web 协议
Envoy 代理 (0.0.0.0:8082)
    ↓ HTTP/2 + 原生 gRPC
gRPC Gateway (192.168.66.190:50051)
    ↓
ROS2 节点
```

---

## 实施步骤

### 阶段 1：安装和配置 Envoy 代理 ⭐

#### 1.1 安装 Envoy

```bash
# 方式 1：使用 Docker（推荐）
docker pull envoyproxy/envoy:v1.28-latest

# 方式 2：直接安装
sudo apt-get install envoy
```

#### 1.2 创建 Envoy 配置文件

创建 `client/flutter_monitor/envoy.yaml`：

```yaml
admin:
  address:
    socket_address:
      address: 0.0.0.0
      port_value: 9901

static_resources:
  listeners:
  - name: listener_0
    address:
      socket_address:
        address: 0.0.0.0
        port_value: 8082
    filter_chains:
    - filters:
      - name: envoy.filters.network.http_connection_manager
        typed_config:
          "@type": type.googleapis.com/envoy.extensions.filters.network.http_connection_manager.v3.HttpConnectionManager
          stat_prefix: grpc_web
          codec_type: AUTO
          route_config:
            name: local_route
            virtual_hosts:
            - name: backend
              domains: ["*"]
              routes:
              - match:
                  prefix: "/"
                  grpc: {}
                route:
                  cluster: grpc_backend
                  timeout: 60s
              cors:
                allow_origin_string_match:
                - prefix: "*"
                allow_methods: GET, PUT, DELETE, POST, OPTIONS
                allow_headers: keep-alive,user-agent,cache-control,content-type,content-transfer-encoding,custom-header-1,x-accept-content-transfer-encoding,x-accept-response-streaming,x-user-agent,x-grpc-web,grpc-timeout
                max_age: "1728000"
                expose_headers: custom-header-1,grpc-status,grpc-message
          http_filters:
          - name: envoy.filters.http.grpc_web
            typed_config:
              "@type": type.googleapis.com/envoy.extensions.filters.http.grpc_web.v3.GrpcWeb
          - name: envoy.filters.http.cors
            typed_config:
              "@type": type.googleapis.com/envoy.extensions.filters.http.cors.v3.Cors
          - name: envoy.filters.http.router
            typed_config:
              "@type": type.googleapis.com/envoy.extensions.filters.http.router.v3.Router

  clusters:
  - name: grpc_backend
    type: LOGICAL_DNS
    lb_policy: ROUND_ROBIN
    dns_lookup_family: V4_ONLY
    typed_extension_protocol_options:
      envoy.extensions.upstreams.http.v3.HttpProtocolOptions:
        "@type": type.googleapis.com/envoy.extensions.upstreams.http.v3.HttpProtocolOptions
        explicit_http_config:
          http2_protocol_options: {}
    load_assignment:
      cluster_name: grpc_backend
      endpoints:
      - lb_endpoints:
        - endpoint:
            address:
              socket_address:
                address: 192.168.66.190
                port_value: 50051
```

#### 1.3 启动 Envoy

```bash
# 使用 Docker
docker run -d \
  -p 8082:8082 \
  -p 9901:9901 \
  -v $(pwd)/client/flutter_monitor/envoy.yaml:/etc/envoy/envoy.yaml:ro \
  --name envoy-grpc-web \
  envoyproxy/envoy:v1.28-latest

# 或者直接运行
envoy -c client/flutter_monitor/envoy.yaml
```

#### 1.4 验证 Envoy

```bash
# 检查 Envoy 是否运行
curl http://localhost:9901/stats

# 测试 gRPC-Web 连接
grpcurl -plaintext -d '{}' \
  localhost:8082 \
  remote_monitoring.SystemService/GetRobotInfo
```

---

### 阶段 2：修改 Flutter Web 客户端使用 gRPC-Web

#### 2.1 添加 gRPC-Web 依赖

修改 `pubspec.yaml`：

```yaml
dependencies:
  flutter:
    sdk: flutter
  
  # gRPC 依赖
  grpc: ^3.2.4          # 用于 Linux/Mobile
  protobuf: ^3.1.0
  
  # 条件导入（Web 使用不同的 channel）
  # Web 会自动使用 XHR/Fetch transport
```

#### 2.2 创建平台特定的客户端

创建 `lib/services/robot_client_web.dart`：

```dart
import 'package:grpc/grpc_web.dart';
import '../generated/telemetry.pbgrpc.dart';
import '../generated/system.pbgrpc.dart';

class RobotClientWeb {
  final String host;
  final int port;
  
  late GrpcWebClientChannel _channel;
  late TelemetryServiceClient _telemetryClient;
  late SystemServiceClient _systemClient;
  
  RobotClientWeb({required this.host, this.port = 8082});
  
  Future<bool> connect() async {
    try {
      // gRPC-Web 使用 HTTP/1.1，连接到 Envoy 代理
      _channel = GrpcWebClientChannel.xhr(
        Uri.parse('http://$host:$port'),
      );
      
      _telemetryClient = TelemetryServiceClient(_channel);
      _systemClient = SystemServiceClient(_channel);
      
      // 测试连接
      final info = await _systemClient.getRobotInfo(
        Empty(),
        options: CallOptions(timeout: Duration(seconds: 3)),
      );
      
      print('Connected to robot via gRPC-Web: ${info.robotId}');
      return true;
    } catch (e) {
      print('Connection failed: $e');
      return false;
    }
  }
  
  // ... 其他方法与原 robot_client.dart 相同
}
```

#### 2.3 条件编译（Web vs Native）

修改 `lib/services/robot_client.dart`：

```dart
// 根据平台导出不同的实现
export 'robot_client_stub.dart'
    if (dart.library.io) 'robot_client_native.dart'
    if (dart.library.html) 'robot_client_web.dart';
```

---

### 阶段 3：更新 Flutter Web 配置

#### 3.1 修改连接地址

在 Web 版本中，默认连接到 Envoy 代理：
- 原生版本：`192.168.66.190:50051` (直连 gRPC)
- Web 版本：`192.168.66.190:8082` (通过 Envoy)

#### 3.2 更新 main.dart

```dart
import 'package:flutter/foundation.dart' show kIsWeb;

class _ConnectionScreenState extends State<ConnectionScreen> {
  final _hostController = TextEditingController(text: '192.168.66.190');
  final _portController = TextEditingController(
    text: kIsWeb ? '8082' : '50051'  // Web 使用 Envoy 端口
  );
  
  // ...
}
```

---

### 阶段 4：测试和验证

#### 4.1 启动完整系统

```bash
# 终端 1: 启动 gRPC Gateway
ros2 launch remote_monitoring remote_monitoring.launch.py

# 终端 2: 启动 Envoy 代理
docker run -d -p 8082:8082 -p 9901:9901 \
  -v $(pwd)/client/flutter_monitor/envoy.yaml:/etc/envoy/envoy.yaml:ro \
  envoyproxy/envoy:v1.28-latest

# 终端 3: 启动 Flutter Web
cd client/flutter_monitor
flutter run -d web-server --web-port 8080 --web-hostname 0.0.0.0
```

#### 4.2 访问和测试

1. 浏览器打开：`http://192.168.66.190:8080`
2. 输入机器人 IP：`192.168.66.190`
3. 端口自动设置为 `8082`（Envoy 代理）
4. 点击连接，验证数据流

#### 4.3 验证清单

- [ ] Envoy 代理正常运行（`curl localhost:9901/stats`）
- [ ] Web 应用可以获取机器人信息
- [ ] FastState 流正常（10 Hz）
- [ ] SlowState 流正常（1 Hz）
- [ ] 浏览器控制台无错误

---

## 优势对比

| 方案 | 优点 | 缺点 |
|------|------|------|
| **原生 gRPC** | 性能最好，类型安全 | 浏览器不支持 |
| **gRPC-Web** ✅ | 标准方案，浏览器支持 | 需要代理层 |
| WebSocket 桥接 | 实现简单 | 非标准，维护成本高 |

---

## 部署方案

### 开发环境

```bash
# 使用 Docker Compose 统一管理
docker-compose up
```

创建 `docker-compose.yml`：

```yaml
version: '3'
services:
  envoy:
    image: envoyproxy/envoy:v1.28-latest
    ports:
      - "8082:8082"
      - "9901:9901"
    volumes:
      - ./client/flutter_monitor/envoy.yaml:/etc/envoy/envoy.yaml:ro
    restart: always
```

### 生产环境

建议将 Envoy 集成到 Kubernetes 或使用 systemd 管理：

```bash
# 创建 systemd 服务
sudo nano /etc/systemd/system/envoy-grpc-web.service
```

---

## 后续扩展

### 1. HTTPS 支持

为生产环境添加 TLS：

```yaml
# envoy.yaml 添加 TLS 配置
transport_socket:
  name: envoy.transport_sockets.tls
  typed_config:
    "@type": type.googleapis.com/envoy.extensions.transport_sockets.tls.v3.DownstreamTlsContext
    common_tls_context:
      tls_certificates:
      - certificate_chain:
          filename: "/etc/envoy/certs/cert.pem"
        private_key:
          filename: "/etc/envoy/certs/key.pem"
```

### 2. 负载均衡

支持多个 gRPC Gateway 实例：

```yaml
clusters:
- name: grpc_backend
  type: STRICT_DNS
  load_assignment:
    endpoints:
    - lb_endpoints:
      - endpoint:
          address: { address: robot1.local, port_value: 50051 }
      - endpoint:
          address: { address: robot2.local, port_value: 50051 }
```

### 3. 监控和日志

配置 Envoy 指标和访问日志：

```yaml
access_log:
- name: envoy.access_loggers.file
  typed_config:
    "@type": type.googleapis.com/envoy.extensions.access_loggers.file.v3.FileAccessLog
    path: "/var/log/envoy/access.log"
```

---

## 参考资料

- [gRPC-Web 官方文档](https://github.com/grpc/grpc-web)
- [Envoy gRPC-Web 过滤器](https://www.envoyproxy.io/docs/envoy/latest/configuration/http/http_filters/grpc_web_filter)
- [Flutter gRPC-Web 支持](https://pub.dev/packages/grpc)

---

## 总结

使用 gRPC-Web + Envoy 是浏览器访问 gRPC 服务的**标准和推荐方案**：

✅ **优点**:
- 符合 gRPC 生态标准
- 支持双向流（streaming）
- 性能接近原生 gRPC
- 维护成本低

⚠️ **注意**:
- 需要部署 Envoy 代理（但可以 Docker 化）
- 增加一层网络跳转（延迟 ~1-2ms）

📅 **实施时间**: 约 2-4 小时（包括测试）
