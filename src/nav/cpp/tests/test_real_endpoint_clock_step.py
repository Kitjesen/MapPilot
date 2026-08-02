#!/usr/bin/env python3
"""Exercise the real native endpoint across a synthetic realtime step."""

from __future__ import annotations

import ctypes
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path


def _repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / ".git").exists():
            return parent
    raise RuntimeError("repository root not found")


def _configure_client(library: ctypes.CDLL) -> None:
    library.lingtu_nav_client_create.argtypes = [ctypes.c_int]
    library.lingtu_nav_client_create.restype = ctypes.c_void_p
    library.lingtu_nav_client_destroy.argtypes = [ctypes.c_void_p]
    library.lingtu_nav_client_stop_with_id.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_int,
    ]
    library.lingtu_nav_client_stop_with_id.restype = ctypes.c_int
    library.lingtu_nav_client_clear_estop_with_id.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_int,
    ]
    library.lingtu_nav_client_clear_estop_with_id.restype = ctypes.c_int
    library.lingtu_nav_client_last_error.argtypes = [ctypes.c_void_p]
    library.lingtu_nav_client_last_error.restype = ctypes.c_char_p


def main() -> int:
    root = _repo_root()
    build = root / "build" / "mujoco_native_nav"
    endpoint = build / "navd"
    client_library = build / "liblingtu_nav_client.so"
    preload_source = Path(__file__).with_name("test_clock_step_preload.c")
    for required in (endpoint, client_library, preload_source):
        if not required.exists():
            raise RuntimeError(f"required test input is missing: {required}")

    domain_id = 200 + os.getpid() % 20
    os.environ["LINGTU_NAV_CLIENT_DIAGNOSTICS"] = "1"
    with tempfile.TemporaryDirectory(prefix="lingtu-clock-step-") as raw_temp:
        temp = Path(raw_temp)
        preload = temp / "liblingtu_test_clock_step.so"
        offset_file = temp / "endpoint_clock_offset.txt"
        endpoint_log = temp / "endpoint.log"
        status_file = temp / "status.json"
        latch_file = temp / "estop_latch.json"
        offset_file.write_text("0.0\n", encoding="ascii")
        subprocess.run(
            [
                "gcc",
                "-shared",
                "-fPIC",
                "-O2",
                str(preload_source),
                "-o",
                str(preload),
                "-ldl",
            ],
            check=True,
        )

        environment = os.environ.copy()
        environment.update(
            {
                "LD_PRELOAD": str(preload),
                "LINGTU_TEST_CLOCK_OFFSET_FILE": str(offset_file),
                "LINGTU_TELEOP_CMD_MAX_AGE_S": "0.25",
            }
        )
        with endpoint_log.open("wb") as log_stream:
            process = subprocess.Popen(
                [
                    str(endpoint),
                    "--control-mode",
                    "teleop",
                    "--domain-id",
                    str(domain_id),
                    "--tick-hz",
                    "50",
                    "--publish-cmd-vel",
                    "false",
                    "--check-obstacle",
                    "false",
                    "--input-future-tolerance-s",
                    "0.05",
                    "--status-file",
                    str(status_file),
                    "--estop-latch-file",
                    str(latch_file),
                ],
                env=environment,
                stdout=log_stream,
                stderr=subprocess.STDOUT,
            )
            client = None
            try:
                time.sleep(0.4)
                library = ctypes.CDLL(str(client_library))
                _configure_client(library)
                client = library.lingtu_nav_client_create(domain_id)
                if not client:
                    raise RuntimeError("failed to create native navigation client")
                if library.lingtu_nav_client_stop_with_id(client, b"clock-prime", b"clock_prime", 3000) != 0:
                    error = library.lingtu_nav_client_last_error(client).decode()
                    raise RuntimeError(f"failed to prime endpoint clock: {error}")

                os.kill(process.pid, signal.SIGSTOP)
                result: dict[str, object] = {}

                def send_clock_sensitive_command() -> None:
                    result["code"] = library.lingtu_nav_client_clear_estop_with_id(
                        client,
                        b"clock-step-clear-estop",
                        b"clock_step_test",
                        4000,
                    )
                    result["error"] = library.lingtu_nav_client_last_error(client).decode()

                sender = threading.Thread(target=send_clock_sensitive_command, daemon=True)
                sender.start()
                time.sleep(0.2)
                offset_file.write_text("2.6\n", encoding="ascii")
                os.kill(process.pid, signal.SIGCONT)
                sender.join(timeout=6.0)
                if sender.is_alive():
                    raise RuntimeError("clear-estop client did not finish after clock step")
            finally:
                if process.poll() is None:
                    try:
                        os.kill(process.pid, signal.SIGCONT)
                    except ProcessLookupError:
                        pass
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
                if client is not None:
                    library.lingtu_nav_client_destroy(client)

        log_text = endpoint_log.read_text(encoding="utf-8", errors="replace")
        match = re.search(r"clear_estop_source_stamp_stale age_s=([0-9.]+)", log_text)
        if not match or float(match.group(1)) < 2.0:
            raise RuntimeError("synthetic clock step did not reach the real endpoint stale gate")
        if result.get("code") != 0:
            raise RuntimeError("client exposed a recoverable clock-step rejection: " + str(result.get("error", "")))
        print(f"test_real_endpoint_clock_step: PASS initial_rejection_age_s={float(match.group(1)):.6f}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
