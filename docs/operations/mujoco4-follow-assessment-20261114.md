# MuJoCo 4 局部跟踪与人物跟随评估

日期：2026-11-14

## 结论

MuJoCo 4 的 ThunderV4 仿真包含前置 RGB-D 相机。相机挂载、640×480 配置、30 Hz 传感器声明、RGB-D 观测投影和 `VisualServoModule` 的组件级跟踪均已通过测试。

完整的“移动人物 → 视觉观测 → 跟随目标更新 → 原生导航执行”本轮尚未通过。阻塞发生在原生运动验收的外部 arm 和 resume 阶段，跟踪 worker 没有获得开始信号，因此没有生成有效跟随目标。

## 已验证

| 范围 | 结果 | 证据 |
| --- | --- | --- |
| PersonTracker、重识别、丢失和速度预测单测 | 174 项通过 | `pytest`：PersonTracker、VisualServo、RGB-D fixture、动态场景 |
| MuJoCo 4 相机挂载和视觉 rig | 16 项通过 | `front_camera`、`camera1_link`、前向视场、RGB-D camera module |
| C++ 局部路径／样条跟踪 | 67 项通过 | `test_path_follower_core.exe` |
| MuJoCo 4 运行时版本 | 已确认 | MuJoCo 4.0.1，Python 3.11 |
| 原生传感器发布和 driver bridge | 重编后能持续发布 | `lingtu_mujoco_sensor_publisher`、`lingtu_mujoco_driver_bridge` |
| 终止停机清理 | 第二次 smoke 已通过 | exact zero、STOPPED ACK、authority cleared、进程清理 |
| RGB-D 跟踪组件录屏回放 | 通过（合成夹具） | 50 个可见观测、11 个跟随目标、11 个目标回执、1 次取消 |

## 机器狗是否有视觉

有两层视觉路径。

仿真侧的 `thunderv4_front_vision@1.0.0` 提供 `front_rgb` 和 `front_depth`，配置为 640×480、30 Hz。`front_camera` 安装在 `camera1_link`，前向中心射线不会撞到机器人自身。

`tracking` Product 会装配 `camera`、`PerceptionModule` 和 `VisualServoModule`。当前真实 Thunder 上的 Orbbec Gemini 335 已经提供 RGB-D 共享内存和视频服务，但真实人物识别、身份重识别和实机跟随仍没有现场验收证据。

## 本轮完整 smoke 的阻塞

最新一次使用 MuJoCo 4.0.1、重编后的原生适配器和 `vision` extra 运行了 RGB-D follow smoke。结果仍未进入人物跟随阶段。

现场证据位于：

`build/assessment-mujoco4-follow-smoke-20261114e/native/motion/`

主要状态如下：

- `motion_arm_status.json` 为 `timed_out`，`last_error=external_arm_timeout`。
- `driver_ready` 没有成为可用状态。
- `resume` 没有成功放行原生控制循环。
- `rgbd-follow-report.json` 的 `samples=0`、`goals=0`。
- 终止停机已经成功，说明退出清理路径可以工作。

第一次 smoke 还暴露了验收环境缺少 OpenCV。`rgbd_follow_worker.py` 使用 `cv2` 做视频记录和图像转换，运行时需要 `sim-recording` 或显式安装 `vision` extra。缺少这个依赖会让 worker 在进入观测前退出。

## 当前覆盖边界

已经覆盖的内容：

- RGB-D 相机读取和深度投影
- 场景人物的模拟身份和三维位置
- PersonTracker 的目标锁定、位置平滑、速度预测、目标丢失和重识别路径
- VisualServo 的人物目标和地图坐标目标生成
- 局部 path／spline follower 的速度、转向、限加速度、停止和重规划行为

尚未覆盖的内容：

- 生产人物检测器在 MuJoCo 录屏中的实际检出率
- 两人交叉时的身份保持
- 遮挡后重新识别
- 深度失效或深度跳变
- 完整 tracking Product 启停后再执行 follow
- 原生导航控制循环接收跟随目标后的真实运动
- 真实机器人相机标定和实机跟随

## 可运行录屏示例

为了先完整展示视觉跟踪链路，本轮使用同一 MuJoCo 4 场景、前置相机和 RGB-D 观测源，给人物 mocap 轨迹注入了连续位移，并使用受控的仿真控制回执完成目标提交和取消。这个回放不启动真实机器人，也不绕过真实产品的运动安全门禁。

输出目录：

`build/mujoco4-follow-demo-20261114/`

包含：

- `rgbd-follow-demo.mp4`
- `rgbd-follow-demo.gif`
- `rgbd-first.png`
- `rgbd-latest.png`
- `rgbd-follow-report.json`
- `rgbd-follow.jsonl`

回放报告记录 `visible_fraction=1.0`、`goals=11`、`accepted_goals=11`、`cancels=1`。这份旧回放使用颜色 fixture，只能作为 RGB-D 投影和任务回执夹具，不能作为学习式人物识别证据。

回放过程中发现并修正了 `rgbd_follow_worker.py` 对 `VisualGoalRequest` 的错误解包。worker 原先把消息对象当成平面坐标读取，第一帧目标会触发 `goal.x` 异常，修复后改为读取 `goal.goal.pose` 和消息自身的 `task_id`。

随后生成了带机器人位姿移动的长程回放：

`build/mujoco4-follow-long-demo-20261114/`

这段回放中机器人从约 `y=6.45 m` 移动到 `y=18.95 m`，人物从约 `y=8.78 m` 移动到 `y=21.28 m`。共记录 96 个可见观测、43 个跟随目标和 1 次结束取消。收尾时又修正了 `VisualGoalCancel` 被当作字符串传给控制命令的问题。

## 下一步

1. 给 MuJoCo tracking 验收增加一个明确的仿真 arm 入口，避免人物跟随组件测试依赖人工外部 arm。
2. 把 follow worker 的启动条件和 native resume 结果写入同一份结构化报告，区分“未开始”与“跟踪失败”。
3. 先在无原生运动的组件场景中验收移动人物、目标更新、目标丢失和取消。
4. 再运行完整 tracking Product，验收 goal admission、native task 状态和停止清理。
5. 先使用 `--detector yoloe` 重新跑 MuJoCo，记录真实检测帧率、检出率、深度有效率、目标跳变和跟随距离，再迁移到真实 Orbbec RGB-D 输入。

本轮没有修改跟踪算法参数，也没有把仿真通过结果当作真实机器狗跟随能力。
