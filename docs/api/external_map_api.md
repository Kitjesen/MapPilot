# 外部地图 HTTP 接口

外部合作方只连接 Gateway，不需要实现 DDS、Unix Domain Socket（UDS），也不需要读取 Sunrise 上的地图目录。

```text
浏览器 / App / SDK / 上位机
          │ HTTP :5050
          ▼
       Gateway
          │ Sunrise 本机 UDS（内部实现）
          ▼
         mapd
```

UDS 全称 **Unix Domain Socket**，是同一台 Linux 设备上进程间通信使用的套接字。它不使用对外 IP，其他设备不能直接连接。PCD 下载时，Gateway 通过内部地图客户端取得 mapd 已校验的只读 artifact，再把字节流转成 HTTP 响应。

## 1. 谁持有访问密钥

每台机器人有两把彼此独立的设备密钥：

| 配置项 | 用途 | 应交给谁 |
| --- | --- | --- |
| `LINGTU_API_KEY` | Gateway 管理员权限，包括运行模式和运动相关接口 | 设备运维程序；不要交给普通客户业务程序 |
| `LINGTU_MAP_API_KEY` | 只允许地图列表、保存操作、操作状态和 PCD 下载 | 需要接入地图功能的客户后端或上位机 |

它们不是向第三方申请的云端 key，也不是用户账号。交付机器人时，在该台 Sunrise 上生成；密钥归客户组织管理和轮换。LingTu 的安装人员可以代为执行初始化脚本，但不应在厂商服务器、工单或代码仓库中留存密钥副本。

客户程序通过以下请求头发送分配给它的地图密钥：

```http
X-API-Key: <secret>
```

它只负责鉴权，不负责链路加密。机器人接口应位于可信局域网或 VPN 中；跨越不可信网络时，应由部署层提供 TLS，不要把端口 `5050` 直接暴露到公网。

### 在 Sunrise 上生成

```bash
sudo bash /opt/lingtu/current/scripts/deploy/thunder/configure_gateway_api_key.sh
sudo systemctl restart lt-host.service
```

脚本使用 Python `secrets` 分别生成两组 48 字节随机值，并按最小权限分别写入：

```text
/etc/lingtu/gateway.env     root:root    0600  LINGTU_API_KEY only
/etc/lingtu/map-client.env  root:sunrise 0640  LINGTU_MAP_API_KEY only
```

`gateway.env` 只保存管理员密钥，客户程序不得读取它。`map-client.env`
只保存地图权限密钥；外部客户程序通过安全交付得到地图密钥并存入自己的
secret manager。配置脚本不会打印密钥。

只轮换交给客户程序的地图 key（不会影响设备运维 key）：

```bash
sudo bash /opt/lingtu/current/scripts/deploy/thunder/configure_gateway_api_key.sh --rotate-map
sudo systemctl restart lt-host.service
```

`--rotate-map` 只更新 `/etc/lingtu/map-client.env`，不会改变管理员 `gateway.env`。轮换后应通过客户认可的安全渠道交付新地图密钥；客户端和地图 CLI 仍不得读取管理员文件。

需要同时轮换管理员 key 和地图 key 时：

```bash
sudo bash /opt/lingtu/current/scripts/deploy/thunder/configure_gateway_api_key.sh --rotate
sudo systemctl restart lt-host.service
```

重启 Host 后，被轮换的旧 key 立即失效。地图 key 应通过客户认可的安全渠道交付，并存入客户后端的 secret manager 或不进 Git 的私密环境文件。不要把长期有效的 key 写进网页 JavaScript、手机 App 安装包、源码、URL 查询参数、浏览器历史、截图或日志。浏览器接入时，推荐由客户自己的后端代为调用机器人；需要多用户登录、审计或公网访问时，应在 Gateway 前增加客户的身份网关，而不是给每位用户复制设备管理员 key。

## 2. 客户端连接

```bash
export LINGTU_GATEWAY='http://192.168.66.13:5050'
# LINGTU_MAP_API_KEY 由客户后端或上位机的受保护环境注入。

curl -fsS \
  "${LINGTU_GATEWAY}/api/v1/auth/check"
```

`GET /api/v1/auth/check` 用于查询 Gateway 是否要求认证。正式请求统一使用 `X-API-Key`；不要使用 `?api_key=...`，查询参数容易进入访问日志、历史记录和 referrer。

运行中的 `/openapi.json`、`/docs` 和 `/redoc` 是字段 schema 的权威来源。

## 3. 当前支持的地图接口

### 查询地图列表

```http
GET /api/v1/slam/maps
```

```bash
curl -fsS \
  -H "X-API-Key: ${LINGTU_MAP_API_KEY}" \
  "${LINGTU_GATEWAY}/api/v1/slam/maps"
```

响应包括 `maps`、`count` 和 `active`。调用方应检查每张地图的 `activation_ready` 和 `state`，不能只根据名称判断地图产物能否激活。`activation_ready` 只表示 `map.pcd`、规划 artifact 和 `metadata.json` 已具备，不表示定位、传感器、控制链或运行时导航已经就绪。响应不会暴露 Sunrise 的地图目录、artifact 文件路径或隐藏维护目录。

### 提交保存地图

```http
POST /api/v1/map/save
Content-Type: application/json

{
  "name": "warehouse",
  "request_id": "01K1M9S4FX27T8XMY6QJNBAV3W"
}
```

- `name` 可省略，由服务端生成。
- `request_id` 由调用方稳定生成。同一次业务请求发生网络重试时必须复用同一个 ID；新接入程序应使用 26 字符大写 ULID，LingTu SDK 会自动生成。
- HTTP 不暴露 loop、PGO 或 HBA 参数。Fast-LIO2 捕获只冻结地图、位姿、body-local
  patches 与 manifest；SaveMap 在完整 patch bundle 上自动调用 native
  `lt_pgo --auto-constraints`。只有完整相邻链和至少一个可信 loop 同时存在时才优化；
  否则保留 raw source，并在 `map_optimization.json` 中记录具体 skip code。
  `pose_graph.constraints` 是优化器私有临时输入，不是 API 输出。旧 ROS2 PGO/HBA
  不在运行时链路中；现场回环质量与 S100P 性能尚未验收。

```bash
curl -fsS -X POST \
  -H "X-API-Key: ${LINGTU_MAP_API_KEY}" \
  -H 'Content-Type: application/json' \
  --data '{
    "name": "warehouse",
    "request_id": "01K1M9S4FX27T8XMY6QJNBAV3W"
  }' \
  "${LINGTU_GATEWAY}/api/v1/map/save"
```

保存地图可能持续几十秒或更久，因此 HTTP `202` 只表示任务已受理、仍在运行，不表示保存成功。此时响应的关键字段为：

```json
{
  "ok": true,
  "success": null,
  "accepted": true,
  "status": "running",
  "reason_code": "map_save_in_progress",
  "operation_id": "01K1MA6Q9J7R5C8D2N4P0V1X3Y",
  "operation": {"state": "RUNNING"}
}
```

保存只应在操作员已经启动 mapping Product、SLAM 支持保存且机器人状态允许时调用。外部调用方不负责启动或切换 Product。

`request_id` 是客户端提供的幂等键，网络超时后可以用同一个值安全重放提交；`operation_id` 是服务端返回的保存操作查询号，不是地图名称、地图 ID 或地图版本。两者职责不同，客户端不得假设它们相等，并应以响应中的 `operation_id` 查询操作。迁移期间，两者的字符串值在某些版本中可能恰好相同；这只是实现细节，不构成接口保证。完整语义和迁移边界见 [LingTu ID Registry](../architecture/ID_REGISTRY.md)。普通 SDK 用户不需要处理这些细节，直接使用下文的 `save_map_and_wait()`。

### 查询、取消或重试保存操作

```http
GET  /api/v1/maps/operations/{operation_id}
POST /api/v1/maps/operations/{operation_id}/cancel
POST /api/v1/maps/operations/{operation_id}/retry
```

```bash
OPERATION_ID='01K1MA6Q9J7R5C8D2N4P0V1X3Y'
curl -fsS \
  -H "X-API-Key: ${LINGTU_MAP_API_KEY}" \
  "${LINGTU_GATEWAY}/api/v1/maps/operations/${OPERATION_ID}"
```

建议最多每秒轮询一次。只有响应中的 `operation.state` 进入 `SUCCEEDED`、`FAILED` 或 `CANCELLED` 才是终态。HTTP 请求超时不等于地图保存失败，也不等于操作未受理；应使用同一个 `operation_id` 查询。

### 下载 PCD

```http
GET /api/v1/maps/{name}/pcd
```

```bash
curl -fL \
  -H "X-API-Key: ${LINGTU_MAP_API_KEY}" \
  --output warehouse.pcd \
  "${LINGTU_GATEWAY}/api/v1/maps/warehouse/pcd"
```

这是二进制流，不要按 JSON 解析。成功响应包含 `Content-Length`。PCD 下载响应不包含、客户端也不得依赖 Sunrise 地图文件路径。

## 4. 现场地图切换

Gateway 不提供独立的地图激活或恢复接口。切图同时涉及旧 Product 退出、地图身份、定位、规划进程和 readiness，统一由 ProductControl 完成：

```bash
python -m lingtu.control switch nav --robot unitree/go2 --env real --map warehouse --relocalize
```

合作方不得通过 Gateway 直接修改 active map。需要切图时，由获授权的操作者执行 ProductControl 命令，再重新查询 readiness。

## 5. 错误处理

| HTTP 状态 | 含义 | 调用方处理 |
| --- | --- | --- |
| `401/403` | key 缺失或错误 | 停止请求并修复凭据 |
| `404` | 地图、任务或 artifact 不存在 | 检查标识，不要盲目重试 |
| `409` | 当前运行状态不允许操作 | 查询任务/运行状态后再决定 |
| `422` | 请求字段不符合 schema | 按运行中 OpenAPI 修正请求 |
| `502/503` | mapd 或内部地图入口不可用 | 保留错误，按退避策略重试只读请求 |

不要把 HTTP `2xx` 当成物理机器人已经完成动作的证据。

## 6. 交付给合作方的最小信息

- Gateway 地址，例如可信网络内的 `http://<robot>:5050`；
- 通过安全渠道交付的 `LINGTU_MAP_API_KEY`（不是管理员 key）；
- 本页列出的受支持接口；
- 运行中的 `/openapi.json`；
- 超时、幂等 `request_id` 和错误码处理规则。

合作方不需要 UDS 地址、DDS topic、Sunrise 地图目录或 mapd 文件权限。


## 7. Python SDK 示例

SDK 会隐藏现有 HTTP 路径命名差异，并自动携带 `X-API-Key`：

```python
import os
from pathlib import Path

from lingtu.sdk import LingTuClient

robot = LingTuClient(
    "192.168.66.13",
    api_key=os.environ["LINGTU_MAP_API_KEY"],
)

result = robot.save_map_and_wait(
    "warehouse",
    timeout=300,
)

robot.download_map_pcd("warehouse", Path("warehouse.pcd"))
```

`save_map_and_wait()` 会自动生成稳定请求号、提交保存并等待终态。保存失败、取消或超时会抛出异常；成功返回服务端的最终状态。异步客户端提供同名 `async` 方法。

只有需要后台提交、展示进度或允许人工取消的高级客户端，才需要使用 `save_map()` 返回的 `operation_id`，再调用 `get_map_operation()`、`cancel_map_operation()` 或 `retry_map_operation()`。
