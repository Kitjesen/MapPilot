"""Async SDK failure-mode tests."""

from __future__ import annotations

import unittest
from pathlib import Path
from unittest.mock import AsyncMock, call

import pytest

aiohttp = pytest.importorskip("aiohttp")

from lingtu.sdk import AsyncLingTuClient, CommandResult, NavigationStatus, Position


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


class _BinaryContent:
    def __init__(self, chunks: list[bytes | BaseException]) -> None:
        self._chunks = chunks

    async def iter_chunked(self, _chunk_size: int):
        for chunk in self._chunks:
            if isinstance(chunk, BaseException):
                raise chunk
            yield chunk


class _BinaryResponse:
    status = 200

    def __init__(self, chunks: list[bytes | BaseException]) -> None:
        self.content = _BinaryContent(chunks)

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        return None

    def raise_for_status(self) -> None:
        return None


class _BinarySession:
    def __init__(self, chunks: list[bytes | BaseException]) -> None:
        self.response = _BinaryResponse(chunks)
        self.url: str | None = None
        self.headers: dict[str, str] = {}

    def get(self, url: str, *, headers: dict[str, str]):
        self.url = url
        self.headers = headers
        return self.response


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

    async def test_async_navigation_status_parses_canonical_nested_target(self):
        robot = AsyncLingTuClient()
        robot._get = AsyncMock(
            return_value={
                "state": "EXECUTING",
                "target": {
                    "distance_to_goal_m": 4.5,
                    "goal": {"x": 8.0, "y": 9.0, "z": 0.0, "yaw": 0.2},
                },
                "mission": {"raw": {"time_elapsed": 2.5}},
            }
        )

        status = await robot.navigation_status()

        self.assertEqual(status.state, "EXECUTING")
        self.assertEqual(status.distance_to_goal, 4.5)
        self.assertEqual(status.time_elapsed, 2.5)
        self.assertEqual(status.goal, Position(8.0, 9.0, 0.0, 0.2))

    async def test_async_maps_prefers_current_active_contract(self):
        robot = AsyncLingTuClient()
        robot._get = AsyncMock(
            return_value={
                "active": "warehouse",
                "active_map": "legacy_map",
                "maps": [
                    {"name": "warehouse", "has_pcd": True},
                    {"name": "legacy_map", "has_pcd": True},
                ],
            }
        )

        maps = await robot.maps()

        self.assertEqual(maps.active_map, "warehouse")
        self.assertTrue(maps.maps[0].is_active)
        self.assertFalse(maps.maps[1].is_active)
        robot._get.assert_awaited_once_with("/api/v1/slam/maps")

    async def test_async_recording_uses_canonical_native_api(self):
        robot = AsyncLingTuClient()
        robot._command = AsyncMock(return_value=CommandResult(ok=True, message="ok"))
        robot._get = AsyncMock(return_value={"recording": True})

        await robot.recording_start("inspection", duration=120)
        await robot.recording_stop()
        status = await robot.recording_status()

        self.assertTrue(status["recording"])
        self.assertEqual(
            robot._command.await_args_list,
            [
                call(
                    "/api/v1/recordings/start",
                    {"duration": 120, "prefix": "inspection"},
                ),
                call("/api/v1/recordings/stop"),
            ],
        )
        robot._get.assert_awaited_once_with("/api/v1/recordings/status")

    async def test_async_bag_alias_delegates_to_canonical_recording_api(self):
        robot = AsyncLingTuClient()
        robot.recording_start = AsyncMock(
            return_value=CommandResult(ok=True, message="ok")
        )
        robot.recording_stop = AsyncMock(
            return_value=CommandResult(ok=True, message="ok")
        )
        robot.recording_status = AsyncMock(return_value={"recording": False})

        await robot.bag_start("legacy")
        await robot.bag_stop()
        await robot.bag_status()

        robot.recording_start.assert_awaited_once_with("legacy")
        robot.recording_stop.assert_awaited_once_with()
        robot.recording_status.assert_awaited_once_with()


    async def test_async_wait_ignores_stale_terminal_and_raises_on_new_failure(self):
        robot = AsyncLingTuClient()
        baseline = NavigationStatus(
            state="SUCCESS",
            goal=Position(5.0, 6.0),
            raw={
                "mission": {"raw": {"ts": 1.0}},
                "target": {"goal": {"x": 5.0, "y": 6.0}},
            },
        )
        robot.navigation_status = AsyncMock(
            side_effect=[
                baseline,
                NavigationStatus(
                    state="PLANNING",
                    goal=Position(5.0, 6.0),
                    raw={
                        "mission": {"raw": {"ts": 2.0}},
                        "target": {"goal": {"x": 5.0, "y": 6.0}},
                    },
                ),
                NavigationStatus(
                    state="CANCELLED",
                    goal=Position(5.0, 6.0),
                    raw={
                        "failure_reason": "operator_cancelled",
                        "target": {"goal": {"x": 5.0, "y": 6.0}},
                    },
                ),
            ]
        )

        with self.assertRaisesRegex(RuntimeError, "CANCELLED.*operator_cancelled"):
            await robot.wait_until_arrived(
                timeout=1.0,
                poll_interval=0.0,
                expected_goal=(5.0, 6.0, 0.0),
                baseline=baseline,
            )

        self.assertEqual(robot.navigation_status.await_count, 3)

    async def test_async_batch_go_waits_for_final_waypoint(self):
        robot = AsyncLingTuClient()
        first_baseline = NavigationStatus(state="IDLE", raw={"mission": {"raw": {"ts": 1.0}}})
        second_baseline = NavigationStatus(state="SUCCESS", raw={"mission": {"raw": {"ts": 2.0}}})
        robot.navigation_status = AsyncMock(side_effect=[first_baseline, second_baseline])
        robot.go = AsyncMock(
            side_effect=[
                CommandResult(ok=True, accepted=True, request_id="goal-1"),
                CommandResult(ok=True, accepted=True, request_id="goal-2"),
            ]
        )
        robot.wait_until_arrived = AsyncMock(
            side_effect=[NavigationStatus(state="SUCCESS"), NavigationStatus(state="SUCCESS")]
        )

        results = await robot.batch_go([(1.0, 2.0, 0.1), (3.0, 4.0, 0.2)])

        self.assertEqual(len(results), 2)
        self.assertEqual(
            robot.wait_until_arrived.await_args_list,
            [
                call(
                    request_id="goal-1",
                    expected_goal=(1.0, 2.0, 0.1),
                    baseline=first_baseline,
                ),
                call(
                    request_id="goal-2",
                    expected_goal=(3.0, 4.0, 0.2),
                    baseline=second_baseline,
                ),
            ],
        )

    async def test_async_save_map_sends_idempotency_fields(self):
        robot = AsyncLingTuClient()
        robot._command = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="save-test-001",
            )
        )

        result = await robot.save_map(
            "test_map",
            optimization="auto",
            request_id="save-test-001",
        )

        self.assertEqual(result.request_id, "save-test-001")
        robot._command.assert_awaited_once_with(
            "/api/v1/map/save",
            {
                "name": "test_map",
                "optimization": "auto",
                "request_id": "save-test-001",
            },
        )

    async def test_async_save_map_generates_request_id_when_omitted(self):
        robot = AsyncLingTuClient()
        robot._command = AsyncMock(return_value=CommandResult(ok=True, accepted=True))

        result = await robot.save_map("test_map")

        body = robot._command.await_args.args[1]
        self.assertRegex(body["request_id"], r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")
        self.assertEqual(result.request_id, body["request_id"])

    async def test_async_save_map_and_wait_returns_immediate_success(self):
        robot = AsyncLingTuClient()
        completed = {
            "ok": True,
            "request_id": "request_1",
            "operation_id": "operation_1",
            "operation": {
                "operation_id": "operation_1",
                "state": "SUCCEEDED",
                "map_id": "test_map",
            },
        }
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        robot.get_map_operation = AsyncMock(return_value=completed)

        result = await robot.save_map_and_wait(
            "test_map",
            request_id="request_1",
            poll_interval=0.0,
        )

        self.assertIs(result, completed)
        robot.save_map.assert_awaited_once_with(
            "test_map",
            optimization=None,
            request_id="request_1",
        )
        robot.get_map_operation.assert_awaited_once_with("operation_1")

    async def test_async_save_map_and_wait_uses_server_operation_id(self):
        robot = AsyncLingTuClient()
        receipt = CommandResult(
            ok=True,
            accepted=True,
            request_id="request_1",
            operation_id="operation_9",
            raw={
                "request_id": "request_1",
                "operation_id": "operation_9",
            },
        )
        completed = {
            "ok": True,
            "operation_id": "operation_9",
            "operation": {
                "operation_id": "operation_9",
                "state": "SUCCEEDED",
                "map_id": "test_map",
            },
        }
        robot.save_map = AsyncMock(return_value=receipt)
        robot.get_map_operation = AsyncMock(return_value=completed)

        result = await robot.save_map_and_wait(
            "test_map",
            request_id="request_1",
            poll_interval=0.0,
        )

        assert result is completed
        robot.get_map_operation.assert_awaited_once_with("operation_9")

    async def test_async_legacy_save_job_methods_are_not_public(self):
        robot = AsyncLingTuClient()
        self.assertFalse(hasattr(robot, "get_save_map_job"))
        self.assertFalse(hasattr(robot, "cancel_save_map_job"))
        self.assertFalse(hasattr(robot, "retry_save_map_job"))

    async def test_async_save_map_and_wait_polls_operation_with_generated_request_id(self):
        robot = AsyncLingTuClient()
        completed = {
            "ok": True,
            "operation_id": "operation_generated",
            "operation": {
                "operation_id": "operation_generated",
                "state": "SUCCEEDED",
            },
        }

        async def admit_save(_name, *, optimization, request_id):
            self.assertIsNone(optimization)
            return CommandResult(
                ok=True,
                accepted=True,
                request_id=request_id,
                operation_id="operation_generated",
                raw={
                    "request_id": request_id,
                    "operation_id": "operation_generated",
                },
            )

        robot.save_map = AsyncMock(side_effect=admit_save)
        robot.get_map_operation = AsyncMock(
            side_effect=[
                {
                    "ok": True,
                    "operation_id": "operation_generated",
                    "operation": {
                        "operation_id": "operation_generated",
                        "state": "RUNNING",
                    },
                },
                completed,
            ]
        )

        result = await robot.save_map_and_wait("test_map", poll_interval=0.0)

        resolved_id = robot.save_map.await_args.kwargs["request_id"]
        self.assertRegex(resolved_id, r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")
        self.assertIs(result, completed)
        self.assertEqual(
            robot.get_map_operation.await_args_list,
            [call("operation_generated"), call("operation_generated")],
        )

    async def test_async_save_map_and_wait_raises_with_failed_operation_reason(self):
        robot = AsyncLingTuClient()
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        robot.get_map_operation = AsyncMock(
            return_value={
                "operation_id": "operation_1",
                "operation": {
                    "operation_id": "operation_1",
                    "state": "FAILED",
                    "reason_code": "snapshot_failed",
                },
            }
        )

        with self.assertRaisesRegex(RuntimeError, "FAILED.*snapshot_failed"):
            await robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                timeout=0.01,
                poll_interval=0.0,
            )

    async def test_async_save_map_and_wait_raises_for_cancelled_operation_spellings(self):
        for state in ("CANCELLED", "CANCELED"):
            with self.subTest(state=state):
                robot = AsyncLingTuClient()
                robot.save_map = AsyncMock(
                    return_value=CommandResult(
                        ok=True,
                        accepted=True,
                        request_id="request_1",
                        operation_id="operation_1",
                        raw={
                            "request_id": "request_1",
                            "operation_id": "operation_1",
                        },
                    )
                )
                robot.get_map_operation = AsyncMock(
                    return_value={
                        "operation_id": "operation_1",
                        "operation": {
                            "operation_id": "operation_1",
                            "state": state,
                            "message": "operator cancelled",
                        },
                    }
                )

                with self.assertRaisesRegex(
                    RuntimeError,
                    f"{state}.*operator cancelled",
                ):
                    await robot.save_map_and_wait(
                        "test_map",
                        request_id="request_1",
                        timeout=0.01,
                        poll_interval=0.0,
                    )

    async def test_async_save_map_and_wait_fails_fast_when_submission_is_rejected(self):
        robot = AsyncLingTuClient()
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=False,
                accepted=False,
                message="invalid map name",
                request_id="save-test-001",
                raw={
                    "ok": False,
                    "reason_code": "invalid_map_name",
                    "message": "invalid map name",
                },
            )
        )
        robot.get_map_operation = AsyncMock()

        with self.assertRaisesRegex(
            RuntimeError,
            "request rejected.*invalid_map_name",
        ):
            await robot.save_map_and_wait(
                "bad/name",
                request_id="save-test-001",
                timeout=0.01,
                poll_interval=0.0,
            )

        robot.get_map_operation.assert_not_awaited()

    async def test_async_save_map_and_wait_fails_fast_on_explicit_status_query_error(self):
        robot = AsyncLingTuClient()
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        robot.get_map_operation = AsyncMock(
            return_value={
                "ok": False,
                "success": False,
                "operation_id": "operation_1",
                "operation": {
                    "operation_id": "operation_1",
                    "reason_code": "forbidden",
                    "message": "invalid API key",
                },
            }
        )

        with self.assertRaisesRegex(
            RuntimeError,
            "status query failed.*forbidden",
        ):
            await robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                poll_interval=0.0,
            )

        robot.get_map_operation.assert_awaited_once_with("operation_1")

    async def test_async_save_map_and_wait_rejects_mismatched_request_identity(self):
        robot = AsyncLingTuClient()
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "different-request", "operation_id": "operation_1"},
            )
        )
        robot.get_map_operation = AsyncMock()

        with self.assertRaisesRegex(RuntimeError, "request identity mismatch"):
            await robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                poll_interval=0.0,
            )

        robot.get_map_operation.assert_not_awaited()

    async def test_async_save_map_and_wait_rejects_mismatched_operation_identity(self):
        robot = AsyncLingTuClient()
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        robot.get_map_operation = AsyncMock(
            return_value={
                "operation_id": "different-operation",
                "operation": {
                    "operation_id": "different-operation",
                    "state": "SUCCEEDED",
                },
            }
        )

        with self.assertRaisesRegex(RuntimeError, "identity mismatch"):
            await robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                poll_interval=0.0,
            )

    async def test_async_save_map_and_wait_recovers_from_post_timeout_and_delayed_admission(self):
        robot = AsyncLingTuClient()
        completed = {
            "ok": True,
            "operation_id": "operation_recovered",
            "operation": {
                "operation_id": "operation_recovered",
                "state": "SUCCEEDED",
            },
        }
        robot.save_map = AsyncMock(
            side_effect=[
                CommandResult(
                    ok=False,
                    accepted=False,
                    message="robot not reachable",
                    raw={"ok": False, "error": "robot not reachable"},
                ),
                CommandResult(
                    ok=True,
                    accepted=True,
                    operation_id="operation_recovered",
                    raw={"operation_id": "operation_recovered"},
                ),
            ]
        )
        robot.get_map_operation = AsyncMock(
            side_effect=[
                {
                    "ok": False,
                    "operation_id": "operation_recovered",
                    "operation": {
                        "operation_id": "operation_recovered",
                        "state": "NOT_FOUND",
                        "reason_code": "operation_not_found",
                    },
                },
                {
                    "ok": False,
                    "success": False,
                    "operation_id": "operation_recovered",
                    "operation": {
                        "operation_id": "operation_recovered",
                        "error": "robot not reachable",
                    },
                },
                completed,
            ]
        )

        result = await robot.save_map_and_wait("test_map", poll_interval=0.0)

        first_call, second_call = robot.save_map.await_args_list
        resolved_id = first_call.kwargs["request_id"]
        self.assertRegex(resolved_id, r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$")
        self.assertEqual(second_call.kwargs["request_id"], resolved_id)
        self.assertIs(result, completed)
        self.assertEqual(
            robot.get_map_operation.await_args_list,
            [
                call("operation_recovered"),
                call("operation_recovered"),
                call("operation_recovered"),
            ],
        )

    async def test_async_save_map_and_wait_times_out_with_last_state(self):
        robot = AsyncLingTuClient()
        robot.save_map = AsyncMock(
            return_value=CommandResult(
                ok=True,
                accepted=True,
                request_id="request_1",
                operation_id="operation_1",
                raw={"request_id": "request_1", "operation_id": "operation_1"},
            )
        )
        robot.get_map_operation = AsyncMock(
            return_value={
                "operation_id": "operation_1",
                "operation": {
                    "operation_id": "operation_1",
                    "state": "RUNNING",
                },
            }
        )

        with self.assertRaisesRegex(TimeoutError, "within 0.01s.*RUNNING"):
            await robot.save_map_and_wait(
                "test_map",
                request_id="request_1",
                timeout=0.01,
                poll_interval=0.0,
            )

        robot.get_map_operation.assert_awaited_with("operation_1")

    async def test_async_get_map_operation(self):
        robot = AsyncLingTuClient()
        robot._get = AsyncMock(
            return_value={
                "ok": True,
                "operation_id": "operation-1",
                "operation": {"state": "RUNNING"},
            }
        )

        operation = await robot.get_map_operation("operation-1")

        self.assertEqual(operation["operation"]["state"], "RUNNING")
        robot._get.assert_awaited_once_with("/api/v1/maps/operations/operation-1")

    async def test_async_cancel_map_operation(self):
        robot = AsyncLingTuClient()
        robot._command = AsyncMock(return_value=CommandResult(ok=True, accepted=True))

        result = await robot.cancel_map_operation("operation-1")

        self.assertTrue(result.ok)
        robot._command.assert_awaited_once_with("/api/v1/maps/operations/operation-1/cancel")

    async def test_async_retry_map_operation(self):
        robot = AsyncLingTuClient()
        robot._command = AsyncMock(return_value=CommandResult(ok=True, accepted=True))

        result = await robot.retry_map_operation("operation-1")

        self.assertTrue(result.ok)
        robot._command.assert_awaited_once_with("/api/v1/maps/operations/operation-1/retry")

    async def test_async_download_map_pcd_streams_with_authentication(self):
        session = _BinarySession([b"pcd-", b"bytes"])
        robot = AsyncLingTuClient(api_key="secret")
        robot._session = session

        target = Path.cwd() / ".sdk-test-async-download.pcd"
        self.assertFalse(target.exists())
        try:
            result = await robot.download_map_pcd("factory_01", target)

            self.assertEqual(result, target)
            self.assertEqual(target.read_bytes(), b"pcd-bytes")
        finally:
            target.unlink(missing_ok=True)

        self.assertEqual(
            session.url,
            "http://127.0.0.1:5050/api/v1/maps/factory_01/pcd",
        )
        self.assertEqual(session.headers["X-API-Key"], "secret")

    async def test_async_download_map_pcd_preserves_target_on_interrupted_stream(self):
        session = _BinarySession([b"partial", OSError("connection lost")])
        robot = AsyncLingTuClient()
        robot._session = session
        target = Path.cwd() / ".sdk-test-async-download-failure.pcd"
        target.write_bytes(b"previous-map")

        try:
            with self.assertRaisesRegex(OSError, "connection lost"):
                await robot.download_map_pcd("factory_01", target)

            self.assertEqual(target.read_bytes(), b"previous-map")
            self.assertEqual(
                list(target.parent.glob(f".{target.name}.*.part")),
                [],
            )
        finally:
            target.unlink(missing_ok=True)
            for partial in target.parent.glob(f".{target.name}.*.part"):
                partial.unlink(missing_ok=True)
