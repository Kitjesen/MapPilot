#!/usr/bin/env python3
"""Gateway-backed Telegram monitor bot for LingTu robots."""

from __future__ import annotations

import asyncio
import os
from typing import Any

from telegram import Update
from telegram.ext import Application, CommandHandler, ContextTypes

from gateway_status import collect_gateway_status, format_status


class TelegramMonitorBot:
    """Telegram bot that polls LingTu Gateway instead of subscribing to ROS2."""

    def __init__(
        self,
        bot_token: str,
        chat_id: str,
        *,
        gateway_url: str,
        poll_interval_sec: float = 30.0,
    ) -> None:
        self.bot_token = bot_token
        self.chat_id = chat_id
        self.gateway_url = gateway_url.rstrip("/")
        self.poll_interval_sec = poll_interval_sec
        self.app: Application[Any] | None = None
        self._last_summary = ""

    def _collect_status(self) -> dict[str, Any]:
        return collect_gateway_status(self.gateway_url)

    def _format_status(self, status: dict[str, Any]) -> str:
        return format_status(status, title="LingTu Telegram Status")

    async def send_message(self, text: str) -> None:
        if self.app is None:
            return
        await self.app.bot.send_message(chat_id=self.chat_id, text=text)

    async def cmd_status(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
        if update.message is None:
            return
        await update.message.reply_text(self._format_status(self._collect_status()))

    async def cmd_start(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
        if update.message is None:
            return
        await update.message.reply_text(
            "LingTu monitor bot started\n\n"
            "Available commands:\n"
            "/status - query current Gateway status\n"
            "/help   - show help"
        )

    async def poll_gateway(self) -> None:
        while True:
            status = self._collect_status()
            summary = "|".join(
                str(status.get(key))
                for key in ("state", "health_status", "slam_status", "localization_state", "blockers")
            )
            if summary != self._last_summary:
                self._last_summary = summary
                await self.send_message(self._format_status(status))
            await asyncio.sleep(self.poll_interval_sec)

    async def run(self) -> None:
        self.app = Application.builder().token(self.bot_token).build()
        self.app.add_handler(CommandHandler("start", self.cmd_start))
        self.app.add_handler(CommandHandler("help", self.cmd_start))
        self.app.add_handler(CommandHandler("status", self.cmd_status))

        await self.app.initialize()
        await self.app.start()
        await self.app.updater.start_polling()
        poll_task = asyncio.create_task(self.poll_gateway())
        try:
            await asyncio.Event().wait()
        finally:
            poll_task.cancel()
            await self.app.updater.stop()
            await self.app.stop()
            await self.app.shutdown()


def _required_env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        raise SystemExit(f"Missing required environment variable: {name}")
    return value


def main() -> None:
    bot = TelegramMonitorBot(
        _required_env("TELEGRAM_BOT_TOKEN"),
        _required_env("TELEGRAM_CHAT_ID"),
        gateway_url=os.environ.get("LINGTU_GATEWAY_URL", "http://localhost:5050"),
        poll_interval_sec=float(os.environ.get("LINGTU_MONITOR_POLL_SEC", "30")),
    )
    asyncio.run(bot.run())


if __name__ == "__main__":
    main()
