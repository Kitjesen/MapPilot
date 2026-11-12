# 巡检日历与时间计划

## 参考与证据

用户参考：[Inspora — Date Range Picker](https://www.inspora.design/posts/1-30)，作者 Jeet。

REA 被动 CDP 记录：`ev_ee1c67fca5c261051ef731b4d0328ef2083275eaa2527d045fed2fd5b4131065`；目标发现记录：`ev_07cb4a877e6a14ec53cd9bda2802f426dec0bd309dadc8e71b25906632bdaf3e`。导出证据位于 `artifacts/ui/inspection-calendar-20261110/rea-reference.json`。

已观察到的内容：

- 页面内容是约 38.97 秒的循环演示视频，页面自身没有日历日期按钮；原作品链接指向作者的 X 帖子。
- 画面采用浅灰外壳、白色内卡、大圆角和轻阴影。顶部并列显示起止日期，月历以淡蓝连续胶囊表示日期区间，以圆形强调两个端点。
- 底部有快捷日期范围和清除入口。具体原始代码、键盘行为和日期算法未从参考页取得。

首轮 REA launch 抓取保留了截图和 DOM，但清理确认超时；检查没有遗留的 headless Chrome 后，改为自有浏览器配合 REA 被动观察并正常导出。自有浏览器已关闭。实现复用项目自己的 React、Lucide 和 CSS，视觉参考与本项目新增业务行为分开记录。

## 放在哪里

在现有“巡检”工作区增加“执行／计划”切换。执行页保留路线、实时任务和报告；计划页提供月历、当天计划、发生记录以及创建／编辑表单。点击发生记录中的真实 `task_id`，回到执行页读取原任务状态。

参考中的“入住／退房”转换为“开始／结束日期”。计划包含已保存的路线与版本、北京时间、单次／每天／每周及周几。日期范围为闭区间。原有 `loop_interval_s` 继续表示同一任务内的轮次间隔，不作为日历调度。

## 数据与接口

计划绑定 `route_id/map_id/revision`，不复制路线几何，不在到点时自动换地图或采用新版本。

```text
schedule
  id, name, route_id, map_id, revision
  start_date, end_date, local_time
  frequency = once | daily | weekly
  weekdays = 0(Mon)..6(Sun)
  timezone = Asia/Shanghai
  enabled, next_run_at, created_at, updated_at

occurrence
  id, schedule_id, scheduled_for, request_id
  state = dispatching | submitted | rejected | missed | unknown
  reason, task_id
```

| 接口 | 用途 |
| --- | --- |
| `GET /api/v1/inspection/schedules?start=YYYY-MM-DD&end=YYYY-MM-DD` | 范围内计划、发生记录与定时服务状态 |
| `POST /api/v1/inspection/schedules` | 创建计划 |
| `PUT /api/v1/inspection/schedules/{id}` | 修改日期、路线引用、重复规则或启用状态 |
| `DELETE /api/v1/inspection/schedules/{id}` | 删除未来计划，保留发生记录 |

首版时区固定为北京时间，API 不接受其它时区。新计划默认停用，用户可以明确保存并启用。

## 谁负责执行

浏览器只管理计划与展示日期。Gateway Module 生命周期管理定时线程；真实 Gateway 的 Uvicorn 配置关闭 ASGI lifespan，因此不依赖 HTTP startup 事件。

到点调度先读取现有 `build_navigation_status(gw)` 的权威状态。状态缺失、过期、人工占用、已有任务或其它未就绪原因都会阻止启动，记录具体原因。通过后调用现有 `start_inspection_task` 完整入口，保留 Product session、活动地图、路线版本、录制、请求去重与原生 ACK 语义。

`submitted` 只代表请求受理，执行中／已完成／失败仍由既有巡检任务接口提供。调度不发布速度，不绕过原生导航仲裁，也不把模型或网页状态当作机器人状态。

每次发生以计划和计划时间去重。启动前持久记录提交身份；重启后未确认结果的提交不会自动重放。错过时间不会补排旧运动。停用或删除计划影响之后的发生，已提交的任务继续使用原任务取消入口。

## 验收边界

本机 4326 工作区没有机器人 Product／原生巡检执行器；只构建 Gateway HTTP app 的工作区不会启动 Module 定时线程，页面必须显示“定时服务未运行”。本机契约测试用可控时间和假执行器验证到点、去重、错过、重启与门禁。目标机的自动巡检与现场动作需单独验收。

推进清单：

- [x] REA 核对参考内容并保存证据。
- [x] 核对现有路线、任务和日志执行边界。
- [x] 接入计划存储、API、生命周期与日期调度。
- [x] 接入巡检页日历与计划编辑。
- [x] 完成本机契约、生产构建和浏览器验收。
- [ ] 目标机定时服务与监督下的到点执行验收。

本机验证记录（2026-11-10）：

- 新计划后端 15 项通过；原巡检路由 64 项在相关合跑中通过。包含真实原启动事务的录制／任务提交调用链，执行器和时钟使用受控测试输入。
- Web 日期／接口／日历 7 项、原巡检工作台 20 项通过；覆盖跨月、闰年、周规则、保存后的路线版本、后台刷新保留操作错误和拒绝原因中文显示。
- bootstrap 原有 31 项通过；新增链接遗漏响应 schema 的一项在补齐字段后单独通过。定向 Ruff、ESLint 和生产 TypeScript/Vite 构建通过。
- 实际 4326 API 返回北京时间、空计划和 `scheduler_running=false`；正式页面可以进入“巡检 → 计划”，如实显示未运行和需要保存路线。未向机器人发送指令。
- 生产构建的浏览器验收 8 项通过，包含日期范围、每周规则、编辑启停、保存错误保留、删除后保留记录、任务跳转、刷新恢复、清除日期，以及 390 px 触屏实际填写和保存。页面异常、未知写请求和横向溢出均为 0；测试浏览器已关闭。接口数据明确标为本地 fixture，没有调用真实模型或机器人。
- 参考证据、截图、脚本和后续浏览器结果位于 `artifacts/ui/inspection-calendar-20261110/`。
