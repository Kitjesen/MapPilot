# CLI — 命令行入口与交互界面

## 概述

`cli/` 是 LingTu 的终端入口层，负责命令行参数解析、profile 选择、交互式 REPL、运行状态展示、健康检查和守护进程控制。

边界约定：`cli/` 可以调用 `runtime/` 的运行时配置、Blueprint 和状态工具；`runtime/` 不应反向依赖 `cli/`。Field Product 只由 `config/runtime_graph/products/` 声明；CLI 直接读取 `src/runtime/profiles/catalog/` 中分类明确的本地 Profile 与 Host 默认值。

## 当前目录结构

| 分组 | 文件 | 职责 |
| --- | --- | --- |
| 入口与编排 | `main.py` | CLI 主入口：参数解析、profile 选择、系统组装与启动 |
| 入口与编排 | `repl.py` | 交互式 REPL：导航、地图、语义、agent、teleop、monitor 命令 |
| Profile / 路径 / 引导 | `runtime.profiles.catalog` | 本地 Profile 与 Host 默认值 |
| Profile / 路径 / 引导 | `bootstrap.py` | 引导初始化：设置项目路径、把 `src/` 加入 import path |
| Profile / 路径 / 引导 | `paths.py` | 项目根目录与关键路径访问器 |
| 运行时控制 | `run_state.py` | session 生命周期、PID/状态文件、组件启停协调 |
| 运行时控制 | `runtime_extra.py` | 预检、端口清理、health、daemon 辅助命令 |
| 运行时控制 | `runtime_audit.py` | 运行时契约审计、profile graph 与端点一致性检查 |
| 运行时显示 | `runtime_display.py` | session 状态面板、运行时 payload 格式化 |
| 输出与终端 UI | `ui.py` | Banner、profile 选择器、提示框等终端 UI |
| 输出与终端 UI | `term.py` | 颜色、TTY 检测、终端格式化工具 |
| 输出与终端 UI | `logging_util.py` | 统一日志格式、JSONL 日志、stderr 过滤 |
| 包声明 | `__init__.py` | `cli` package marker |

## 使用

```bash
python lingtu.py              # 交互式 profile 选择
python lingtu.py --list       # 列出常用 profile
python lingtu.py --list --all # 列出 Field Product、仿真、开发与兼容入口
python lingtu.py stub         # 框架测试模式
python lingtu.py dev          # 语义 pipeline 开发模式
python lingtu.py sim          # MuJoCo 仿真
# 实机产品切换必须经过 ProductControl；lingtu.py 只启动 Host/本地开发图。
bash scripts/lingtu mode switch nav --map <map-name>
bash scripts/lingtu mode switch inspection --map <map-name>
python lingtu.py runtime-audit nav --env real  # 启动前契约审计
python lingtu.py status       # 查看运行状态
python lingtu.py health       # 健康检查
python lingtu.py stop         # 停止运行中的 session
```

Profile 和 Field Product 数量会随产品、仿真和兼容入口演进，不在文档中写死；以
`python lingtu.py --list --all` 和 `src/runtime/profiles/catalog/` 为准。

## 整理约定

- `cli/` 只保留源码和说明文档；`__pycache__/`、`.pytest_cache/`、临时输出等生成物不要放入目录。
- 新增 CLI 能力时，优先放入职责最接近的现有文件；只有当文件职责明显过载时再拆分模块。
- 不要把 profile 真实定义重新搬回 `cli/`，避免 `runtime -> cli` 的反向依赖。
- 不要在 `cli/main.py` 增加现场 systemd 编排。现场模式切换统一调用
  `lingtu.control.ProductControl`；`lingtu.py` 只负责受管 Host 与本地开发执行。
- 不要在 CLI 层引入 ROS 2 运行时耦合。Field Product 使用原生 typed DDS
  与显式 Endpoint；ROS2 只允许出现在命名清楚的离线工具或兼容适配器中。
- 用户可见文案可以保留中文；新代码注释遵循仓库约定使用英文。
