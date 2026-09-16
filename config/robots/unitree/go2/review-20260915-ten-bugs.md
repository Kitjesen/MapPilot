# 2026-09-15：新增 10 项代码缺陷审查

本轮按“再挖掘 10 个错误”审查当前工作区，没有修改运行代码、安装版本或发送运动指令。
下面是本地代码证据及受控复现，不是 10 个实机故障都已现场发生的结论。
此前修好的体素地面保留、完整姿态、快照时间、首帧后断流问题没有重复计数。

## 1. [P2] 保存/更新当前位置丢失实际高度

- 位置：[SceneView.tsx](../../../../web/src/components/SceneView.tsx:1025)，更新分支在 1098 行。
- 触发：机器人当前位置的地图 Z 不等于零，点击保存；或移动到不同高度后点击更新当前位置。
- 原因：保存固定写 `z: 0`；更新写 `z: loc.z ?? 0`，两者都不读取 `robotZ`。
- 复现：执行实际回调，机器人 Z=0.8 m，保存结果 Z=0；旧位置 Z=-0.1 m，更新后仍为 -0.1。
- 影响：记录的位置本身错误；后续常用位置导航把这个 Z 传给候选目标，可能选择错误高度层或预检失败。
  不据此断言规划器一定执行到错误高度。
- 修正：保存与更新均使用同一有效 map 位姿的 XYZ，补充非零高度的回调回归。

## 2. [P2] SLAM 重置后网页仍沿用旧轨迹身份

- 位置：[sceneTelemetry.ts](../../../../web/src/services/sceneTelemetry.ts:8)，
  [SceneView.tsx](../../../../web/src/components/SceneView.tsx:764)。
- 触发：同一 Product、同一 SLAM 进程内重置建图核心，`source_epoch` 改变。
- 原因：`scenePoseEpoch` 只包含 session、runtime、map_frame_jump_sequence，遗漏 `source_epoch`。
  原生 [fastlio.cpp](../../../../src/localization/slam/cpp/fastlio.cpp:1767) 的 resetCore 会增加 source_epoch。
- 复现：source_epoch 从 10 改成 11，网页 poseEpoch 和 sessionStorage 轨迹键均不变。
- 影响：旧坐标时代的已走轨迹与新位置继续串联；旧建图查询点也可能继续被视为当前查询。
  这不是点云转换矩阵整体错误的证据。
- 修正：在现有坐标身份中纳入 source_epoch，让轨迹、查询点及姿态插值共同失效。

## 3. [P2] 点云体素取整向零截断，跨原点的点误合并

- 位置：[cloud_viewer.py](../../../../src/gateway/services/cloud_viewer.py:1294)，
  [cloud_scene_cache.py](../../../../src/gateway/services/cloud_scene_cache.py:80)。
- 触发：点坐标位于体素原点两侧，包含负数；当前 registered scan 降采样也使用此函数。
- 原因：浮点数直接 `astype(int)`，没有先 floor；原点附近的一个格实际覆盖接近两个格宽。
- 复现：0.15 m 体素下，X=-0.14 与 X=+0.14 两点相距 0.28 m，却只保留一个；floor 后是两个格。
- 影响：原点附近抽样不对称、细节丢失。属于 Gateway 显示降采样问题，未证明 native 碰撞占据格同样错误。
- 修正：统一这些已有体素函数的 floor 语义，并覆盖正负坐标。

## 4. [P2] 原生角速度在 Host 状态转接中丢失

- 位置：[status.py](../../../../src/localization/adapters/status.py:889)，
  [cyclone_runtime.cpp](../../../../src/localization/slam/cpp/cyclone_runtime.cpp:752)。
- 触发：网页经 native SLAM 状态快照获取里程计，机器人发生旋转。
- 原因：快照没有转发已有 `odometry_twist_body`；Python 只生成线速度，Twist 的 angular 默认为零。
  原生 [fastlio.cpp](../../../../src/localization/slam/cpp/fastlio.cpp:1928) 已计算同步、去偏置后的机身角速度。
- 复现：真实状态适配函数在角速度缺失时返回数值 0；Gateway 随后把该值发布为 `wz`。
- 影响：实测旋转遥测失真，无法据此判断 Q/E 的实际响应。不是“控制指令因此不能旋转”。
- 修正：转发原生完整机身 Twist；缺失时明确表示不可用，不补成有效零。
  同时避免再次从 IMU 原点的 fastlio_velocity 重建另一份机身速度。

## 5. [P2] WebSocket 重试会清空正常的 HTTP 备用地图

- 位置：[useBinaryCloud.ts](../../../../web/src/hooks/useBinaryCloud.ts:422)。
- 触发：HTTP 点云可读，但 WS 暂时不可用；这是 hook 明确支持的代理/部署场景。
- 原因：每次 connect/onopen 在收到替换帧之前执行 resetCloudState，清空点数组。
- 复现：HTTP 显示 1 点后推进重试定时器，尚未收到任何新帧，点数变成 0。
- 影响：重试期间地图闪空，已有可用参考图被移除。
- 修正：连接状态与几何缓存分开更新；同坐标身份下保留备用图，收到有效新帧后替换。

## 6. [P2] HTTP 备用请求挂起后没有有界恢复

- 位置：[useBinaryCloud.ts](../../../../web/src/hooks/useBinaryCloud.ts:241)。
- 触发：WS 断流进入 HTTP 备用，随后网络半断开，fetch 长时间不完成。
- 原因：无请求超时/AbortSignal；下一次轮询只在当前请求 finally 后安排。
- 复现：模拟挂起 fetch 并推进 60 秒，只发起了 1 次请求，没有再尝试。
- 影响：备用通道停留在旧图/等待状态，恢复依赖底层请求何时失败或 WS 何时重新收到帧。
- 修正：为该请求加有界超时，退出备用/卸载时取消；失败后继续现有轮询。

## 7. [P2] 旧 HTTP 请求结束后会创建第二条轮询链

- 位置：[useBinaryCloud.ts](../../../../web/src/hooks/useBinaryCloud.ts:286)。
- 触发：第一次 HTTP 请求未结束，WS 恢复，随后再次断流；第一轮旧请求这时完成。
- 原因：响应写入检查 generation，但 finally 只看当前 httpFallbackActive，不检查自己属于哪一轮。
- 复现：旧、新两请求完成后，同一截止时间产生两次新的 fetch；请求总数由 2 变成 4。
- 影响：网络抖动后重复拉取大点云，增加带宽、解码和主线程工作；并发响应又没有顺序游标，存在回退旧图的风险。
- 修正：finally 与写入使用同一 generation 判定，保证仅当前轮拥有一个轮询定时器。

## 8. [P2] 无数据期间关闭页面，点云/相机订阅不能及时退出

- 位置：[realtime.py](../../../../src/gateway/routes/realtime.py:334)，scan 在 363 行，相机循环在 61 行。
- 触发：点云停发时关闭页面；或者相机停帧后关闭相机面板。
- 原因：点云循环只等 q.get，相机循环只看缓存序号，没有消费 websocket.disconnect；没有新发送也就不能通过发送失败退出。
- 复现：使用真实 Starlette WebSocket，断开消息已排队，endpoint 仍等待点云队列，订阅仍为 1；取消任务后 finally 才清理。
- 影响：断流期间反复开关页面留下任务和订阅，客户端数失真；相机订阅还影响客户端连接计数。
- 修正：发送与断开接收共同控制端点生命周期，在任一结束后清理现有订阅。

## 9. [P2] 相机新订阅把旧 JPEG 缓存当成新画面

- 位置：[realtime.py](../../../../src/gateway/routes/realtime.py:61)，
  [gateway_module.py](../../../../src/gateway/gateway_module.py:532)，
  [useCamera.ts](../../../../web/src/hooks/useCamera.ts:66)。
- 触发：相机停止产生新帧，但 Host 保留最后 JPEG；此时重新打开面板。
- 原因：缓存只有 bytes/seq，没有源帧时间；新订阅 last_seq=None 会立即发送旧缓存，网页按接收时刻认定新鲜。
- 复现：无任何新相机帧，打开真实端点仍立即收到旧 JPEG；消息不携带源时间。
- 影响：旧图暂时显示成实时画面，直到前端约 3 秒无帧后清除；每次重新打开又重复。
- 修正：缓存关联源帧时间/新鲜度，在已有传输契约中传递或拒发过期帧，不能以重连接收时刻刷新源龄。

## 10. [P2] WebRTC 连接正常被当作视频持续正常

- 位置：[CameraFeed.tsx](../../../../web/src/components/CameraFeed.tsx:35)，
  [useWHEP.ts](../../../../web/src/hooks/useWHEP.ts:90)。
- 触发：启用可选 go2rtc/WebRTC 后，相机停止视频帧，但 ICE/PeerConnection 仍 connected。
- 原因：hasVideo 与相机健康只取 whep.connected，并据此关闭 JPEG；没有以视频解码帧进展判定实时性。
- 复现范围：执行当前组件的源选择与健康表达式，connected=true 时无需任何帧新鲜度证据，就关闭 JPEG 并返回 live。
  本轮未在实机注入 WebRTC 视频断流，也不据此声称当前 JPEG 部署已触发该分支。
- 影响：冻结视频仍标“实时画面”，可用的 JPEG 备用也不会接管。
- 修正：以实际视频帧进展决定实时状态与备用切换，连接状态只表示传输会话状态。

## 证据与范围

- [cloud-probes.test.ts](../../../../build/go2-audit-20260915-ten/cloud-probes.test.ts)：3 个真实 hook 受控复现，全部复现。
- [ui-probes.test.ts](../../../../build/go2-audit-20260915-ten/ui-probes.test.ts)：3 个实际函数/回调/表达式复现，全部复现。
- [python_probes.py](../../../../build/go2-audit-20260915-ten/python_probes.py)：量化、里程计、ASGI 订阅、旧 JPEG 4 组探针，全部复现。
- 探针通过表示“现有缺陷可重现”，不表示已经修复；没有把它们加入正常通过条件的生产回归套件。
- 优先处理 1、2、4 的目标/坐标/遥测正确性，再处理 5–9 的断流与恢复，3 和 10 随对应路径一起修复。
- 没有证据表明这 10 项足以解释全部实机运动卡顿、历史点重影或人员碰撞。动态避障与持续运动仍需独立验收。
- 未计入：已经记录过的 accumulated_column_carving；没有当前原生调用证据的旧增量缓存计数增长；未复现的外参错误猜测。
