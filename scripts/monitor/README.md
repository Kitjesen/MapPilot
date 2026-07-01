# monitor - remote status bots

Feishu/Lark and Telegram monitor bots poll LingTu Gateway REST endpoints by
default. They do not require ROS2, `rclpy`, or topic subscriptions.

## Files

| File | Purpose |
| --- | --- |
| `gateway_status.py` | Standard-library Gateway status collector and formatter |
| `feishu_monitor_bot.py` | Feishu/Lark monitor bot |
| `telegram_monitor_bot.py` | Telegram monitor bot |
| `feishu_config_template.py` | Environment variable template |
| `requirements_feishu.txt` | Feishu dependencies |
| `requirements_telegram.txt` | Telegram dependencies |

## Feishu

```bash
pip3 install -r scripts/monitor/requirements_feishu.txt

export FEISHU_APP_ID="cli_xxx"
export FEISHU_APP_SECRET="..."
export FEISHU_RECEIVE_ID="ou_or_chat_id"
export FEISHU_RECEIVE_ID_TYPE="open_id"  # or chat_id
export LINGTU_GATEWAY_URL="http://localhost:5050"

python3 scripts/monitor/feishu_monitor_bot.py
```

## Telegram

```bash
pip3 install -r scripts/monitor/requirements_telegram.txt

export TELEGRAM_BOT_TOKEN="..."
export TELEGRAM_CHAT_ID="..."
export LINGTU_GATEWAY_URL="http://localhost:5050"

python3 scripts/monitor/telegram_monitor_bot.py
```

Both bots also honor `LINGTU_MONITOR_POLL_SEC` for the polling interval.
