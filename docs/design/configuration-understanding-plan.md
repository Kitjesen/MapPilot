# 配置与架构理解成本收敛计划

## 当前配置入口

| 内容 | 位置 |
| --- | --- |
| 产品能力、进程、话题和所选模型后端 | `config/system/products/*.yaml` |
| real/sim 的实现与 Host 覆盖 | `config/system/envs/*.yaml` |
| 共享组件参数和原生默认值 | `config/system/settings/*.yaml` |
| 共享模型名、地址、密钥变量名与超时 | `config/system/models.yaml` |
| 机器人几何、设备和标定 | `config/robots/<vendor>/<model>/robot.yaml` |

Product 的 `includes` 引用组件参数。相机预览需要的进程、话题和能力由 Product 或变体声明，
`camera.yaml` 管采集和视频参数。变体可用 `*_add` 增补进程、可选进程、话题和能力，
普通字段继续整字段覆盖。各组件参数的归属和派生关系见 [系统配置](../../config/system/README.md)。

## Host 模型配置

`nav/semantic` 和 `tracking` 都通过 `host.llm: kimi` 使用 `kimi-k3`、Moonshot 地址、
`MOONSHOT_API_KEY` 和 30 秒超时。跟踪使用 LLMModule 的视觉客户端做人物选择。
`inspection` 选择 `qwen`，三条语义客户端路径统一收到 `qwen-turbo`、SDK 默认地址、
`DASHSCOPE_API_KEY` 和 10 秒超时。Qwen 调用仍要求环境中有 DashScope SDK，
现有 LLM 依赖组未安装该 SDK。

```text
Product + Env + 调用方覆盖 → 确定模型后端 → models.yaml 填充未显式设置的字段
  → RunPlan.host_config → Host 装配 → LLMModule / AgentPlanner / GoalResolver
```

显式字段的优先级由低到高是 Product、Env、调用方覆盖。GoalResolver 通过 SemanticPlanner
接收同一套解析值。Host 恢复已保存的 RunPlan 时直接消费快照。快照只保存密钥变量名，
密钥值从 Host 进程环境读取。CLI 的 `--set` 用于原生参数。

## 验收

- 核对 Go2 和 Thunder 的 `nav/semantic`、`tracking` 与 `inspection` 解析结果，包括
  模型五项字段及既有进程、话题、标定和运动所有权。
- 用本地替身验证三条客户端路径消费相同配置、跟踪装配视觉客户端、普通导航保持原有模块图。
- 用安装包与 RunPlan 恢复验证配置文件随发布交付，并确认 Host 直接消费快照。
  云服务连通性和实机行为分别验证。
