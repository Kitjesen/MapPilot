import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest

from gateway import discovery

ROBOT_ID = "7369cda7-9631-4bb4-a4ce-01646b1b56d5"


@pytest.fixture(autouse=True)
def robot_id(monkeypatch):
    monkeypatch.setenv("DOSO_DEVICE_ID", ROBOT_ID)


def bluez(monkeypatch, output):
    stdout = asyncio.StreamReader()
    stdout.feed_data(output)
    process = SimpleNamespace(
        stdin=SimpleNamespace(write=Mock(), drain=AsyncMock()), stdout=stdout,
        returncode=None, terminate=Mock(), kill=Mock(), wait=AsyncMock(return_value=0),
    )
    create = AsyncMock(return_value=process)
    monkeypatch.setattr(discovery.asyncio, "create_subprocess_exec", create)
    return process, create


@pytest.mark.asyncio
async def test_registration_stays_alive_and_cancellation_releases_own_client(monkeypatch):
    process, create = bluez(monkeypatch, b"Waiting to connect to bluetoothd...\nPrimary (yes/no):")
    registered = asyncio.Event()
    messages = []

    def report(message, **_):
        messages.append(message)
        registered.set()

    monkeypatch.setattr(discovery, "print", report, raising=False)
    task = asyncio.create_task(discovery.advertise())
    for _ in range(100):
        if process.stdin.write.call_count >= 2:
            break
        await asyncio.sleep(0.001)
    process.stdout.feed_data(b"Enter value:")
    for _ in range(100):
        if process.stdin.write.call_count >= 3:
            break
        await asyncio.sleep(0.001)
    process.stdout.feed_data(b"Application registered\n")
    for _ in range(100):
        if process.stdin.write.call_count >= 4:
            break
        await asyncio.sleep(0.001)
    process.stdout.feed_data(b"Advertising object registered\n")
    await asyncio.wait_for(registered.wait(), 1)
    assert not task.done()
    assert '"identity_verified": false' in messages[0]
    assert '"connectable": true' in messages[0]
    assert create.call_args.args == ("bluetoothctl",)
    commands = [call.args[0].decode() for call in process.stdin.write.call_args_list]
    assert f"register-service {discovery.SERVICE_UUID}" in commands[0]
    assert commands[1] == f"yes\nregister-characteristic {discovery.ROBOT_ID_UUID} read\n"
    assert commands[2].startswith("0x73 0x69 0xcd 0xa7 ")
    assert commands[2].endswith("register-application\n")
    assert commands[3].endswith("advertise on\n")
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    process.terminate.assert_called_once()
    process.wait.assert_awaited_once()
    process.kill.assert_not_called()


@pytest.mark.asyncio
@pytest.mark.parametrize("output", [
    b"Failed to register advertisement: org.bluez.Error.NotReady\n",
    b"LEAdvertisingManager not found\n",
    b"No default controller available\n",
    b"Invalid command in menu main: name\n",
    b"Failed to register application: org.bluez.Error.Failed\n",
    b"Unable to find GattManager proxy\n",
    b"Advertising object registered\nAdvertising released\n",
])
async def test_bluez_failure_closes_client_without_reporting_success(monkeypatch, output):
    process, _ = bluez(monkeypatch, output)
    with pytest.raises(RuntimeError, match="discovery unavailable"):
        await discovery.advertise()
    process.terminate.assert_called_once()


@pytest.mark.asyncio
async def test_registration_timeout_releases_client(monkeypatch):
    process, _ = bluez(monkeypatch, b"")
    monkeypatch.setattr(discovery, "STARTUP_TIMEOUT", 0.01)
    with pytest.raises(RuntimeError, match="timed out"):
        await discovery.advertise()
    process.terminate.assert_called_once()


@pytest.mark.asyncio
@pytest.mark.parametrize("value", ["", "7369cda7-9631-1bb4-a4ce-01646b1b56d5", ROBOT_ID.upper()])
async def test_invalid_robot_id_never_starts_bluez(monkeypatch, value):
    monkeypatch.setenv("DOSO_DEVICE_ID", value)
    create = AsyncMock()
    monkeypatch.setattr(discovery.asyncio, "create_subprocess_exec", create)
    with pytest.raises(RuntimeError, match="canonical UUID v4"):
        await discovery.advertise()
    create.assert_not_awaited()


@pytest.mark.asyncio
async def test_unexpected_client_exit_is_a_failure(monkeypatch):
    process, _ = bluez(monkeypatch, b"")
    process.stdout.feed_eof()
    process.returncode = 1
    with pytest.raises(RuntimeError, match="bluetoothctl stopped"):
        await discovery.advertise()
    process.terminate.assert_not_called()


@pytest.mark.asyncio
async def test_unresponsive_client_is_killed_after_stop_timeout(monkeypatch):
    process, _ = bluez(monkeypatch, b"LEAdvertisingManager not found\n")
    process.wait.side_effect = [asyncio.TimeoutError, 0]
    with pytest.raises(RuntimeError, match="discovery unavailable"):
        await discovery.advertise()
    process.kill.assert_called_once()
    assert process.wait.await_count == 2
