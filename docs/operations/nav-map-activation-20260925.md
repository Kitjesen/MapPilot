# 新保存地图导航切换顺序修复

903room_v4 保存成功（132 关键帧，129 秒）。`.74` 首次和带状态采样的第二次
nav 切换均定位通过，却在 mapd 就绪检查时回退。

实机证据：mapd 接收了 207 个观测，处理数为零，错误为
`observation_saved_map_identity_mismatch`。原活动地图为 903room；SLAM 已加载
903room_v4。ProductControl 仅在 on_process_ready(maps) 回调中 stage_map；
mapd 必须先切换地图，才能接受新图观测并满足 ready，形成启动依赖循环。

修复：SystemdRunner apply/transition 提供 on_process_started 回调，在进程启动
后、数据就绪等待前调用。ProductControl 在此阶段通过原生 DDS stage_map，
继续核验精确地图身份；SLAM 定位仍在原有 on_process_ready 回调中完成。
地图提交及失败恢复仍由原 ProductControl 事务负责，未绕过一致性检查或
修改地图文件，未扩大就绪超时。

本地验证：生命周期两组共 84 项，首次 80 通过、4 项旧预期仍含已删除二维
地形进程；更新这些预期后四项全部通过。新增四项覆盖冷启动/切换时先激活后
就绪，以及激活失败不能启动驱动；ruff 检查通过。`.75` 打包并通过安装预检。
现场结果见 go2-offline-mapping.md 最新记录。
