# WebRTC 相机流 — 部署与实机测量

相对 JPEG-over-WebSocket 的 ~300 ms glass-to-glass，WebRTC + H.264 把相机
端到端延迟压到 80–150 ms（LAN，软编）。本文档记录如何在 S100P 上部署、
如何量化延迟、以及 Phase B（硬编）的路线图。

**三档方案现状**（`CameraFeed` 自动选最快可用的）：

| 优先级 | 名字 | 实现 | 延迟 (LAN) | CPU | 状态 |
|--------|------|------|----------|-----|------|
| 1 | 图传（go2rtc sidecar） | Go 二进制 + ffmpeg | **30-60ms** | <5% | 需 `install_go2rtc.sh` |
| 2 | aiortc（Python） | 当前默认 | 80-150ms | ~30% | 开箱 |
| 3 | JPEG-over-WS（fallback） | cv2.imencode | 250-400ms | 高 | 永远兜底 |

## 1. 最小部署

```bash
# S100P 上
pip install 'aiortc>=1.14' 'av>=13'

# 首次跑：
python lingtu.py nav
```

启动日志应当看到：

```
WebRTCStreamModule: ... enabled
GatewayModule: WebRTC signalling mounted at POST /api/v1/webrtc/offer
```

浏览器访问 `http://<robot-ip>:5050/`，`CameraFeed` 右下角 hint 会显示
`H.264 · 30fps · 1.20 Mbit/s · 8ms enc`。如果显示 `MJPEG`，说明 aiortc
未加载或 `/api/v1/webrtc/offer` 返回 503，按 §4 排查。

## 2. 可调参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `LINGTU_WEBRTC_BITRATE` | `2500000` (2.5 Mbit/s) | 视频流 RTP 上限。弱网或 Wi-Fi 2.4G 建议 800000–1500000。 |

未来的参数（Phase B 加）：`LINGTU_WEBRTC_H264_TOPIC`（指向
`hobot_codec` 的共享内存 H.264 话题，开启硬编直通）。

## 3. 实机量化延迟

### 3.1 Chrome 内置 WebRTC 诊断（首选）

打开 `chrome://webrtc-internals/`，找到活跃的 `RTCPeerConnection`，看：

- `currentRoundTripTime` — 网络 RTT，LAN 应 <10 ms
- `jitterBufferDelay / jitterBufferEmittedCount` — 浏览器侧抖动缓冲
- `framesDecoded`, `framesDropped` — 解码健康度
- `freezeCount` — 冻结次数（非零说明网络/编码打嗝）

端到端延迟 ≈ `RTT/2 + encode_avg_ms + jitterBufferDelay + decode (~5ms)`。
典型 LAN 组成：`5 + 10 + 30 + 5 = ~50 ms`（一路顺的话）。

### 3.2 肉眼 + 秒表法（最直观）

在屏幕上开一个高精度毫秒时钟（`https://time.is/ms`），把手机相机对准
时钟，机器人摄像头对准手机屏幕；让浏览器回显画面与原时钟并排，相机
快速拍下两者 → 差值就是 glass-to-glass 延迟。重复 5–10 次取均值。

### 3.3 编码/网络分解

```bash
# 在机器人 S100P 上查一下当前聚合指标
curl -s http://localhost:5050/api/v1/webrtc/stats | jq
```

关键字段：

- `encode_avg_ms` — 编码平均耗时/帧。软编 libx264 在 A78AE 上：
  - 720p30 ≈ 5–10 ms（正常）
  - 持续 >15 ms 说明 CPU 打满，降码率或切 Phase B
- `bitrate_bps` — 实际上行码率，应当略低于 `max_bitrate`
- `fps` — 实际编码帧率，应接近 30（如果 source 是 30 Hz）
- `peers[].packets_lost` — 累积丢包，LAN 应当近 0

### 3.4 源头 → 消费者时间戳（精确）

如果需要严谨数字，在 `CameraBridgeModule` 接 image callback 时把毫秒
时间戳 LSB 编入画面（例如画一个二进制条码），浏览器 canvas 逐帧读
时间戳并与 `performance.now()` 做差。此法可绕过所有人眼估计。

## 4. 出问题怎么办

| 现象 | 原因 | 解决 |
|------|------|------|
| 右下角显示 `MJPEG` 而非 `H.264` | `/offer` 返回 503 | 检查 aiortc 装了没：`python -c 'import aiortc; print(aiortc.__version__)'` |
| 视频区一直"等待画面帧…" | `color_image` 端口未接线 / CameraBridge 未启动 | `curl /api/v1/health`，看 `WebRTCStreamModule.has_frame` |
| iOS Safari 黑屏但 Chrome 正常 | SDP 里 VP8 排在 H.264 前面 | 已做 `_force_h264_first`，若仍复现请抓 `chrome://webrtc-internals` 的 SDP 对比 |
| 偶发冻结 / 长尾卡顿 | 网络抖动 / 编码打嗝 | 降 `LINGTU_WEBRTC_BITRATE`；或看 `encode_avg_ms` 是否飙高 |
| `ICE failed` 瞬间断线 | 浏览器与机器人走了非同网段 IP | 确保 host candidate 双方可达；LAN 场景禁用 VPN |

## 5. 图传模式（Go2RTC sidecar，推荐生产路径）

aiortc 软编 + Python SRTP/RTP 在 A78AE 上剩下约 15-20 ms 纯 Python overhead
压不下去了。如果需要 <80 ms 稳定体验，起一个 go2rtc Go 进程做相机热路径。

### 5.1 架构

```
Orbbec Gemini 335L
  ├─ RGB UVC (/dev/video0)  ─→ go2rtc  ─(WebRTC/UDP)→  浏览器 <video>
  │                            │
  │                            └─(WHEP POST /sdp)──▶ Gateway :5050
  │                                                   反代 /api/v1/webrtc/whep
  └─ Depth/IR (OBK SDK) ──▶ ROS2 ──▶ CameraBridge ──▶ SLAM/感知 (不变)
```

媒体走 UDP 直连 go2rtc，不经过 Python；Python 只代理一次 SDP 握手。

### 5.2 部署（在机器人 S100P 上）

```bash
# 1. 安装
sudo bash scripts/webrtc/install_go2rtc.sh
# 2. 如果相机设备号不是 /dev/video0:
v4l2-ctl --list-devices                          # 找到 RGB 节点
sudo sed -i 's|/dev/video0|/dev/videoN|' /etc/go2rtc/go2rtc.yaml
sudo systemctl restart go2rtc
# 3. 验证
curl -s http://localhost:1984/api/streams | jq
```

浏览器刷新后，`CameraFeed` 右下角 hint 会显示 `图传 · Go2RTC · H.264`。
HUD 仅 aiortc 路径有，WHEP 路径用 `chrome://webrtc-internals` 看。

### 5.3 硬件编码开启确认

`config/go2rtc.yaml` 默认在 ffmpeg pipeline 加了 `#hardware`，go2rtc 会
自动探测 v4l2_m2m / vaapi / rkmpp。开启成功的话：

```bash
journalctl -u go2rtc -n 50 | grep -i 'codec\|encode\|hw'
# 看到 "v4l2m2m" 或 "hobot" 字样 → HW 编码命中
# 看到 "libx264" → 回落软编 (仍然比 aiortc 快,因为 Go 侧 SRTP 零开销)
```

S100P Nash BPU 如果 BSP 没暴露 v4l2 codec 节点（高概率），当前只能走
软编 x264；下一阶段用 `hobot_codec` ROS2 节点预编码 + go2rtc RTSP 入站
拼起来（见 §6）。

### 5.4 单端口约束

go2rtc 默认监听 `:1984`，但 `config/go2rtc.yaml` 改成 `127.0.0.1:1984`
只接受本机访问。浏览器通过 Gateway :5050 的 `/api/v1/webrtc/whep` 反代
拿到 SDP answer，然后真正的媒体包走 WebRTC UDP 直连（ICE host
candidate = 机器人 LAN IP）。用户只开 :5050 一个端口即可。

WebRTC UDP 端口范围由 go2rtc `webrtc.listen` 控制，默认自动。如果防火墙
锁死 UDP，config 里加 `webrtc: { listen: ':8555/tcp' }` 走 TCP 候选。

### 5.5 关掉 go2rtc 回到 aiortc

`sudo systemctl stop go2rtc` → 前端 `/api/v1/webrtc/go2rtc/status` 探测
失败 → `CameraFeed` 自动退到 aiortc 路径。不需要前端改配置。

## 6. Phase B 路线（S100P Nash BPU 硬编）

当前软编在 A78AE 上占 ~30 % 单核，持续跑有热节流风险。Phase B 方案：

1. 在 S100P 上起 `hobot_codec` systemd 服务，输入 `/hbmem_img` NV12，
   输出 `/image_h264`（共享内存话题）。Horizon 官方仓：
   <https://github.com/D-Robotics/hobot_codec>
2. 给 `WebRTCStreamModule` 加"预编码直通"模式：订阅 `/image_h264`，
   把 NAL 单元直接喂给 aiortc 的 `H264PayloadContext`，跳过 libx264。
   改动范围：~80 行，协议/前端 0 修改。
3. 预期收益：`encode_avg_ms` 从 5–10 ms 降到 <1 ms，CPU 占用 30% → <5%，
   持续温度曲线明显改善。

如果 Phase B 后还不够快（需要 <50 ms），再考虑换 pion/str0m 原生 WebRTC
sidecar；纯 Python aiortc 在 asyncio + SRTP 层还有 ~10–15 ms overhead
是物理上限。

## 6. 代码结构索引

| 文件 | 作用 |
|------|------|
| `webrtc_stream_module.py` | 模块主体：In[Image] → H.264 → aiortc MediaStreamTrack |
| `webrtc_stream_module.py::_force_h264_first` | SDP 重排（iOS Safari 兼容） |
| `webrtc_stream_module.py::collect_stats` | getStats 聚合 + 码率增量计算 |
| `src/gateway/gateway_module.py::post_webrtc_offer` | SDP offer/answer 信令 |
| `src/gateway/gateway_module.py::get_webrtc_stats` | 实机监控接口 |
| `web/src/hooks/useWebRTC.ts` | 浏览器侧 PeerConnection + 1Hz stats 轮询 |
| `web/src/components/CameraFeed.tsx` | `<video>` + HUD + JPEG fallback |
