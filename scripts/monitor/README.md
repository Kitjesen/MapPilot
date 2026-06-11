# monitor — 远程监控机器人

通过飞书 / Telegram 远程监控灵途导航状态。

## 文件

| 文件 | 说明 |
|------|------|
| `feishu_monitor_bot.py` | 飞书监控主程序（ROS2 节点，监听 `/nav/semantic/status`） |
| `telegram_monitor_bot.py` | Telegram 监控主程序 |
| `feishu_config_template.py` | 飞书配置模板 |
| `requirements_feishu.txt` | 飞书 Python 依赖 |
| `requirements_telegram.txt` | Telegram Python 依赖 |

## 飞书机器人 (thunder)

### 快速开始

```bash
# 安装依赖
pip3 install -r monitor/requirements_feishu.txt

# 配置：编辑 monitor/feishu_monitor_bot.py
# 填入 APP_ID / APP_SECRET / RECEIVE_ID

# 运行（需先 source ROS2 环境）
source /opt/ros/humble/setup.bash
python3 monitor/feishu_monitor_bot.py
```

### 飞书应用配置

1. 访问 https://open.feishu.cn/ → 创建企业自建应用
2. 获取 **App ID** 和 **App Secret**
3. 添加权限: `im:message`、`im:message:send_as_bot`
4. 发布应用
5. 获取接收者 **open_id**（`ou_` 开头）或 **chat_id**

### 消息格式

```
🤖 灵途状态更新 (thunder)
📍 当前状态: NAVIGATING
🎯 目标: kitchen
📏 距离: 2.35m
⏱️ 时间: 15.2s
✅ 成功率: 87.5%
```

### 常见错误

| 错误 | 解决 |
|------|------|
| `app_access_token invalid` | 检查 App ID / Secret |
| `no permission` | 添加 `im:message` 权限并发布 |
| `invalid receive_id` | 确认 open_id 格式 (`ou_` 开头) |
| 收不到消息 | 搜索应用并添加为好友 |

## Telegram 机器人

```bash
pip3 install -r monitor/requirements_telegram.txt
python3 monitor/telegram_monitor_bot.py
```
