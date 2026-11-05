"""Advertise an unverified DOSO discovery candidate through Linux BlueZ."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import sys
from uuid import UUID

SERVICE_UUID = "81343a98-47b3-4a0b-af0c-d42d87c7070f"
ROBOT_ID_UUID = "44cecaf8-0fab-4594-96f9-6e864f1dedf2"
LOCAL_NAME = "DOSO"
STARTUP_TIMEOUT = 10.0


async def advertise() -> None:
    value = os.environ.get("DOSO_DEVICE_ID", "")
    try:
        robot_id = UUID(value)
        if robot_id.version != 4 or str(robot_id) != value:
            raise ValueError
    except ValueError as exc:
        raise RuntimeError("DOSO_DEVICE_ID must be a canonical UUID v4") from exc

    process = await asyncio.create_subprocess_exec(
        "bluetoothctl",
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    try:
        assert process.stdin is not None and process.stdout is not None
        # BlueZ prompts for the service type and characteristic value.
        process.stdin.write(f"menu gatt\nregister-service {SERVICE_UUID}\n".encode())
        await process.stdin.drain()
        deadline = asyncio.get_running_loop().time() + STARTUP_TIMEOUT
        registered = False
        stage = "service"
        output = ""
        while True:
            read = process.stdout.read(4096)
            if registered:
                chunk = await read
            else:
                remaining = max(0, deadline - asyncio.get_running_loop().time())
                try:
                    chunk = await asyncio.wait_for(read, remaining)
                except asyncio.TimeoutError as exc:
                    raise RuntimeError("BlueZ advertisement registration timed out") from exc
            if not chunk:
                raise RuntimeError("bluetoothctl stopped before discovery was closed")
            output = (output + chunk.decode(errors="replace"))[-8192:]
            if any(message in output for message in (
                "Failed to register", "No default controller", "LEAdvertisingManager not found",
                "Invalid command", "Invalid UUID", "Invalid uuid", "Advertising released",
                "Failed to register application", "Failed register application", "No service registered",
                "Unable to find GattManager", "Failed to register characteristic", "Invalid value",
                "Failed to register service", "Invalid option",
            )) or (registered and "Waiting to connect to bluetoothd" in output):
                raise RuntimeError("BlueZ discovery unavailable, check controller power, permissions and Bluetooth service")
            if stage == "service" and "Primary (yes/no):" in output:
                process.stdin.write(f"yes\nregister-characteristic {ROBOT_ID_UUID} read\n".encode())
                await process.stdin.drain()
                stage, output = "value", ""
            elif stage == "value" and "Enter value:" in output:
                data = " ".join(f"0x{byte:02x}" for byte in robot_id.bytes)
                process.stdin.write(f"{data}\nregister-application\n".encode())
                await process.stdin.drain()
                stage, output = "application", ""
            elif stage == "application" and "Application registered" in output:
                process.stdin.write((f"back\nmenu advertise\nuuids {SERVICE_UUID}\n"
                                     f"name {LOCAL_NAME}\nback\nadvertise on\n").encode())
                await process.stdin.drain()
                stage, output = "advertisement", ""
            elif stage == "advertisement" and "Advertising object registered" in output:
                registered = True
                print(json.dumps({
                    "status": "advertising", "service_uuid": SERVICE_UUID,
                    "name": LOCAL_NAME, "connectable": True, "identity_verified": False,
                }), flush=True)
                output = ""
    finally:
        # Closing this D-Bus client removes only its own advertisement.
        if process.returncode is None:
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), 3)
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()


async def _run() -> None:
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()
    assert task is not None
    loop.add_signal_handler(signal.SIGTERM, task.cancel)
    try:
        await advertise()
    finally:
        loop.remove_signal_handler(signal.SIGTERM)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    if sys.platform != "linux":
        parser.error("discovery requires Linux BlueZ")
    try:
        asyncio.run(_run())
    except (KeyboardInterrupt, asyncio.CancelledError):
        return 0
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
