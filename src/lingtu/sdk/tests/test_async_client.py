"""Async SDK failure-mode tests."""

from __future__ import annotations

import unittest

import pytest

aiohttp = pytest.importorskip("aiohttp")

from lingtu.sdk import AsyncLingTuClient


class _FailingSession:
    def get(self, *args, **kwargs):
        raise aiohttp.ClientConnectionError("offline")

    def post(self, *args, **kwargs):
        raise aiohttp.ClientConnectionError("offline")


class _PlainTextResponse:
    status = 500

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        return None

    async def json(self):
        raise ValueError("not json")

    async def text(self):
        return "server error"


class _PlainTextSession:
    def get(self, *args, **kwargs):
        return _PlainTextResponse()

    def post(self, *args, **kwargs):
        return _PlainTextResponse()


class TestAsyncLingTuClient(unittest.IsolatedAsyncioTestCase):
    async def test_async_health_unreachable_returns_error_payload(self):
        robot = AsyncLingTuClient()
        robot._session = _FailingSession()

        raw = await robot._get("/api/v1/health")

        self.assertIs(raw["ok"], False)
        self.assertEqual(raw["error"], "robot not reachable")
        self.assertEqual(raw["path"], "/api/v1/health")

    async def test_async_command_unreachable_returns_failed_result(self):
        robot = AsyncLingTuClient()
        robot._session = _FailingSession()

        result = await robot.go(1.0, 2.0)

        self.assertIs(result.ok, False)
        self.assertEqual(result.message, "robot not reachable")
        self.assertEqual(result.raw["path"], "/api/v1/goal")

    async def test_async_non_json_response_is_server_error_not_unreachable(self):
        robot = AsyncLingTuClient()
        robot._session = _PlainTextSession()

        raw = await robot._get("/api/v1/health")

        self.assertIs(raw["ok"], False)
        self.assertEqual(raw["error"], "HTTP 500")
        self.assertEqual(raw["detail"], "server error")
        self.assertEqual(raw["path"], "/api/v1/health")
