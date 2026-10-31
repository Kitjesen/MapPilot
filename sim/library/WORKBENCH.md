# Agent 3D Workbench 接入记录

日期：2026-10-18。它是资产制作工具，不是 LingTu 物理运行时的依赖。

## 已安装

- 源码：`D:/inovxio/tools/agent-3d-workbench`
- 上游：[Hockwang/agent-3d-workbench](https://github.com/Hockwang/agent-3d-workbench)
- 固定提交：`9b39d35ec0667e5e27ebe9c0809ec94dfee002d7`，MIT许可。
- 环境：独立 `.venv`，Python 3.12.13；`uv sync --locked`；使用现有Node 24.11.1、Blender 5.2。
- Codex MCP名称：`agent-3d-workbench`，全局启用。新会话/重连后加载工具；本轮通过官方MCP SDK调用并验证。
- 本地状态：`D:/inovxio/tools/agent-3d-workbench-state`，服务配置为空，不连接付费生成API。
- 当前项目工作台数据由上游按聊天目录绑定在
  `C:/Users/99563/Documents/Codex/2026-10-12/new-chat/.3dstudio/job`。
- 完成交付复制进 LingTu 的 `sim/library/industrial`，不以临时任务目录作为唯一资产来源。

## 实际完成

1. `studio_capabilities`：运动、Blender、Python、CadQuery可用。
2. `studio_edit`：导入双开检修门25个部件。
3. `studio_motion`：左右门及附件16部件绑定铰链；6秒、30fps开合动作；导出GLB与可编辑ZIP。
4. 对导出动画逐部件核对90°开门支点、固定框不动及闭合回程，25项位置检查通过。
5. `studio_task`：三个CC0资产通过Blender任务导入、打包贴图、导出FBX/GLB/Blender及预览。

任务记录：`build/workbench-20261018/*-task.json`。
动画数值检查：`build/workbench-20261018/door-motion-verification.json`。
源几何不变检查：`build/workbench-20261018/door-edit-report.json`。

## Windows 本地补丁

上游完整桌面流程原先未声明通过Windows验证，本机保留以下最小补丁，未推送上游：

- `studio/__init__.py`：用Win32进程句柄等待状态判活；禁止Windows下`os.kill(pid,0)`终止服务。
- `studio/core/editor.py`：已实际出现的状态文件WinError 5/32/33增加最多1秒的替换重试；持续错误仍失败。
- `studio/core/task_worker.py`：Windows异常清理不引用不存在的`signal.SIGKILL`。
- `tests/test_windows_liveness_local.py`：3项回归通过，覆盖活进程不被探针结束、短暂/持续替换失败、强制/正常清理。

真实失败记录保留：首个工业桶任务因本地任务脚本误用`bpy.ops.wm.pack_all`失败，
已改为`bpy.ops.file.pack_all`；首次推车任务遭遇状态替换与清理异常，后续修复后的新任务完成。
没有把失败或中断任务改成成功。

## 本项目可复用入口

- `sim/tools/assets/polyhaven_intake.py`：下载选定glTF及其完整资源依赖，记录来源和CC0许可。
- `sim/tools/assets/workbench_prepare_asset.py`：作为Workbench Blender任务脚本提交。
- `sim/tools/assets/library_preview/server.cjs`：现有只读素材库，含FBX和动作工程下载。
- 本机MCP调用辅助：`D:/inovxio/tools/agent-3d-workbench-state/call.py`。
- 本机启动脚本：`D:/inovxio/tools/agent-3d-workbench-state/Open-Workbench.ps1`，输出工作台URL；
  素材库服务启动：`sim/tools/assets/library_preview/start.ps1`。后台服务退出时可用这些脚本重新启动。

后续：先拆推车车轮/柜门、管道阀门手轮，保持UV和贴图；再绑定MuJoCo body。
运动预览不是实时受力求解。不得让UE动画/Chaos覆盖MuJoCo权威位姿。
