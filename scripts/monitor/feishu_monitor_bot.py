#!/usr/bin/env python3
"""Gateway-backed Feishu/Lark monitor bot for LingTu robots."""

from __future__ import annotations

import asyncio
import json
import logging
import os
from typing import Any

import lark_oapi as lark
from lark_oapi.api.im.v1 import CreateMessageRequest, CreateMessageRequestBody

from gateway_status import collect_gateway_status, format_status


logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("lingtu-feishu-monitor")


class FeishuMonitorBot:
    """Feishu bot that polls LingTu Gateway instead of subscribing to ROS2."""

    def __init__(
        self,
        app_id: str,
        app_secret: str,
        receive_id: str,
        *,
        gateway_url: str,
        poll_interval_sec: float = 30.0,
    ) -> None:
        self.app_id = app_id
        self.app_secret = app_secret
        self.receive_id = receive_id
        self.gateway_url = gateway_url.rstrip("/")
        self.poll_interval_sec = poll_interval_sec
        self._last_summary = ""
        self.client = (
            lark.Client.builder()
            .app_id(self.app_id)
            .app_secret(self.app_secret)
            .build()
        )

    def _collect_status(self) -> dict[str, Any]:
        return collect_gateway_status(self.gateway_url)

    def _format_status(self, status: dict[str, Any]) -> str:
        return format_status(status, title="LingTu Feishu Status")

    def send_message(self, text: str) -> None:
        request = (
            CreateMessageRequest.builder()
            .receive_id_type(os.environ.get("FEISHU_RECEIVE_ID_TYPE", "open_id"))
            .request_body(
                CreateMessageRequestBody.builder()
                .receive_id(self.receive_id)
                .msg_type("text")
                .content(json.dumps({"text": text}, ensure_ascii=False))
                .build()
            )
            .build()
        )
        response = self.client.im.v1.message.create(request)
        if not response.success():
            logger.error("Feishu send failed: code=%s msg=%s", response.code, response.msg)

    def send_card_message(self, title: str, content: str) -> None:
        card = {
            "config": {"wide_screen_mode": True},
            "header": {"title": {"tag": "plain_text", "content": title}, "template": "blue"},
            "elements": [{"tag": "div", "text": {"tag": "lark_md", "content": content}}],
        }
        request = (
            CreateMessageRequest.builder()
            .receive_id_type(os.environ.get("FEISHU_RECEIVE_ID_TYPE", "open_id"))
            .request_body(
                CreateMessageRequestBody.builder()
                .receive_id(self.receive_id)
                .msg_type("interactive")
                .content(json.dumps(card, ensure_ascii=False))
                .build()
            )
            .build()
        )
        response = self.client.im.v1.message.create(request)
        if not response.success():
            logger.error("Feishu card send failed: code=%s msg=%s", response.code, response.msg)

    async def poll_gateway(self) -> None:
        while True:
            status = self._collect_status()
            summary = "|".join(
                str(status.get(key))
                for key in ("state", "health_status", "slam_status", "localization_state", "blockers")
            )
            if summary != self._last_summary:
                self._last_summary = summary
                self.send_message(self._format_status(status))
            await asyncio.sleep(self.poll_interval_sec)

    async def run(self) -> None:
        self.send_card_message(
            "LingTu monitor started",
            f"Gateway: `{self.gateway_url}`\n\nPolling robot status without ROS2 topic subscriptions.",
        )
        await self.poll_gateway()


def _required_env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise SystemExit(f"Missing required environment variable: {name}")
    return value


def main() -> None:
    bot = FeishuMonitorBot(
        _required_env("FEISHU_APP_ID"),
        _required_env("FEISHU_APP_SECRET"),
        _required_env("FEISHU_RECEIVE_ID"),
        gateway_url=os.environ.get("LINGTU_GATEWAY_URL", "http://localhost:5050"),
        poll_interval_sec=float(os.environ.get("LINGTU_MONITOR_POLL_SEC", "30")),
    )
    try:
        asyncio.run(bot.run())
    except KeyboardInterrupt:
        bot.send_message("LingTu monitor stopped")


if __name__ == "__main__":
    main()
