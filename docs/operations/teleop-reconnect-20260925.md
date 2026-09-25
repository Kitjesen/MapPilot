# Go2 网线重连控制位修复（2026-09-25）

## 原因与修复

旧 `/ws/teleop` 无限等待 receive。TCP 半开时，原生运动租约过期并不释放
Gateway 的网页连接占用位。前端收到 connection-level control_in_use 后又停止
重试；input ACK 超时也仅等待浏览器关闭握手。界面还会保留旧的拒绝提示。

修复：服务端 receive 3 秒超时，进入原有 disconnect 的零速/释放流程，随后
释放网页控制位；空闲网页每秒使用 input_request/ACK 检查存活（不提交速度或
续租原生运动权限）。前端持续重试被占用连接，ACK 超时后不等待旧关闭握手，
迟到消息按 socket 身份忽略；连接重新打开时清理历史拒绝提示。新鲜活动的
其他控制端仍会被保护，不允许抢占；重连不重放方向键或旧速度。

## 验证边界

- 本地 Gateway 15 项测试通过，包含无 disconnect 事件的静默连接超时释放。
- 前端相关 45 项测试通过，包含占用重试、空闲链路黑洞、关闭握手卡住、
  旧消息隔离和禁止重放速度。提示清理后相关 30 项再次通过。
- TypeScript/Vite 构建通过；保留已有大 chunk/插件耗时警告。
- NX `.73` 无运动 WebSocket 验证：静默连接 3.027 秒关闭，新连接收到 input_ack。
  探针没有发送 velocity/hold。驱动 ready/connected，final_cmd_vel 全零，
  SLAM mapping/TRACKING。
- `.74` 在 `.73` 基础上清理重连后的历史拒绝提示。最终部署状态见
  go2-offline-mapping.md。现场拔插网线及监督运动仍需操作者验证。

浏览器必须刷新以加载修复后的 JS。后台标签页计时器被浏览器暂停时，连接会
按失联释放；回到前台后重新连接并重新按键。服务端 3 秒是网页占用回收时间，
不是运动急停时间；原有原生运动 freshness/lease 限制保持不变。
