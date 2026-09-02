"""Tests for the explicit asynchronous SDK adapter."""

from __future__ import annotations

import unittest
from pathlib import Path
from unittest.mock import Mock

from lingtu.sdk import AsyncLingTuClient, CommandResult, Pose2D
from lingtu.sdk.client import LingTuClient


class TestAsyncLingTuClient(unittest.IsolatedAsyncioTestCase):
    """Verify delegation without repeating synchronous client behavior tests."""

    def setUp(self) -> None:
        self.robot = AsyncLingTuClient()
        self.client = Mock(spec=LingTuClient)
        self.robot._client = self.client

    async def test_go_delegates_all_arguments(self) -> None:
        expected = CommandResult(ok=True, request_id="request-1")
        self.client.go.return_value = expected

        result = await self.robot.go(1.0, 2.0, 0.5, request_id="request-1")

        self.assertIs(result, expected)
        self.client.go.assert_called_once_with(
            1.0,
            2.0,
            0.5,
            request_id="request-1",
        )

    async def test_person_following_delegates_to_sync_client(self) -> None:
        expected = CommandResult(ok=True)
        self.client.follow_person.return_value = expected
        self.client.stop_following.return_value = expected

        self.assertIs(await self.robot.follow_person("person_01"), expected)
        self.assertIs(await self.robot.stop_following(), expected)

        self.client.follow_person.assert_called_once_with("person_01")
        self.client.stop_following.assert_called_once_with()

    async def test_recording_start_preserves_capture_options(self) -> None:
        expected = CommandResult(ok=True)
        self.client.recording_start.return_value = expected

        result = await self.robot.recording_start(
            "inspection",
            duration=120,
            capture_profile="evidence",
            task_id="inspection-task-1",
            camera=True,
            minimum_free_gib=12,
        )

        self.assertIs(result, expected)
        self.client.recording_start.assert_called_once_with(
            "inspection",
            duration=120,
            capture_profile="evidence",
            task_id="inspection-task-1",
            camera=True,
            minimum_free_gib=12,
        )

    async def test_save_map_and_wait_delegates_timeout_contract(self) -> None:
        expected = {"state": "completed"}
        self.client.save_map_and_wait.return_value = expected

        result = await self.robot.save_map_and_wait(
            "factory",
            request_id="request-2",
            timeout=30.0,
            poll_interval=0.25,
        )

        self.assertIs(result, expected)
        self.client.save_map_and_wait.assert_called_once_with(
            "factory",
            request_id="request-2",
            timeout=30.0,
            poll_interval=0.25,
        )

    async def test_download_map_pcd_returns_delegate_path(self) -> None:
        target = Path("map.pcd")
        self.client.download_map_pcd.return_value = target

        result = await self.robot.download_map_pcd("factory", target)

        self.assertEqual(result, target)
        self.client.download_map_pcd.assert_called_once_with("factory", target)

    async def test_seeded_relocalization_delegates_to_domain_facade(self) -> None:
        expected = CommandResult(ok=True)
        pose = Pose2D(x=1.0, y=2.0, yaw=0.5)
        self.client.localization.relocalize.return_value = expected

        result = await self.robot.localization.relocalize("factory", initial_pose=pose)

        self.assertIs(result, expected)
        self.client.localization.relocalize.assert_called_once_with(
            "factory",
            initial_pose=pose,
            request_id=None,
        )

    async def test_global_relocalization_delegates_to_domain_facade(self) -> None:
        expected = CommandResult(ok=True)
        self.client.localization.global_relocalize.return_value = expected

        result = await self.robot.localization.global_relocalize("factory")

        self.assertIs(result, expected)
        self.client.localization.global_relocalize.assert_called_once_with(
            "factory",
            request_id=None,
        )

    async def test_map_tracking_delegates_to_domain_facade(self) -> None:
        expected = CommandResult(ok=True)
        self.client.localization.start_map_tracking.return_value = expected

        result = await self.robot.localization.start_map_tracking("factory")

        self.assertIs(result, expected)
        self.client.localization.start_map_tracking.assert_called_once_with(
            "factory",
            request_id=None,
        )

    async def test_delegate_exception_is_not_rewritten(self) -> None:
        self.client.health.side_effect = RuntimeError("offline")

        with self.assertRaisesRegex(RuntimeError, "offline"):
            await self.robot.health()

    async def test_context_manager_closes_delegate(self) -> None:
        self.client.close.return_value = None

        async with self.robot as opened:
            self.assertIs(opened, self.robot)

        self.client.close.assert_called_once_with()

    def test_removed_aliases_are_not_reintroduced(self) -> None:
        for name in (
            "bag_start",
            "bag_stop",
            "bag_status",
            "swap",
            "use_map",
            "restore_map",
            "start_session",
            "end_session",
        ):
            self.assertFalse(hasattr(AsyncLingTuClient, name), name)


if __name__ == "__main__":
    unittest.main()
