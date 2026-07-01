#!/usr/bin/env python3
"""Feishu/Lark monitor environment template.

Do not commit real credentials. Export these variables in the service
environment or an ignored local env file before starting the monitor bot.
"""

# Required:
#   export FEISHU_APP_ID="cli_xxxxxxxxxxxxxxxx"
#   export FEISHU_APP_SECRET="xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#   export FEISHU_RECEIVE_ID="ou_xxxxxxxxxxxxxxxxxxxxxxxx"
#
# Optional:
#   export FEISHU_RECEIVE_ID_TYPE="open_id"  # or "chat_id"
#   export LINGTU_GATEWAY_URL="http://localhost:5050"
#   export LINGTU_MONITOR_POLL_SEC="30"
#
# Run:
#   python3 scripts/monitor/feishu_monitor_bot.py
