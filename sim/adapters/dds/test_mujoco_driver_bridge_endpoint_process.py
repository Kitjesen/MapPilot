from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from test_dds_domain import domain_id_from_environment

_PRODUCT_SESSION_ID = "7" * 32
_CONTROLLER_BOOT_ID = "b" * 32


def _wait_for_file(path: Path, process: subprocess.Popen[str], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file():
            return
        return_code = process.poll()
        if return_code is not None:
            _raise_process_failure(process, f"process exited before publishing {path.name}")
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {path}")


def _raise_process_failure(process: subprocess.Popen[str], message: str) -> None:
    stdout, stderr = process.communicate(timeout=2.0)
    raise AssertionError(
        f"{message}; return_code={process.returncode}; stdout={stdout!r}; stderr={stderr!r}"
    )


def _command_fields(line: str, expected_kind: str) -> list[str]:
    fields = line.split("\t")
    assert len(fields) == 10, fields
    assert fields[0] == "LT_DRIVER_COMMAND_V2", fields
    assert fields[4] == expected_kind, fields
    return fields


def _applied_line(command: list[str], step_seq: int) -> str:
    return "\t".join(
        [
            "LT_DRIVER_APPLIED_V2",
            command[1],
            command[2],
            command[3],
            command[4],
            command[5],
            command[6],
            command[7],
            command[8],
            command[9],
            str(step_seq),
        ]
    )


def _stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def _start_bridge(
    bridge_binary: Path, environment: dict[str, str]
) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [
            str(bridge_binary),
            "--domain-id",
            environment["LINGTU_TEST_DDS_DOMAIN_ID"],
            "--local-endpoint",
            "--heartbeat-timeout-ms",
            "2000",
            "--apply-timeout-ms",
            "2000",
            "--poll-ms",
            "2",
        ],
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def _wait_endpoint_cleanup(readiness: Path, auth: Path) -> None:
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and (readiness.exists() or auth.exists()):
        time.sleep(0.01)
    assert not readiness.exists(), "endpoint readiness was not cleaned"
    assert not auth.exists(), "endpoint auth secret was not cleaned"


def _assert_formal_identity_argument_rejected(
    bridge_binary: Path,
    environment: dict[str, str],
    *arguments: str,
) -> None:
    result = subprocess.run(
        [
            str(bridge_binary),
            "--domain-id",
            environment["LINGTU_TEST_DDS_DOMAIN_ID"],
            "--local-endpoint",
            *arguments,
        ],
        env=environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=3.0,
        check=False,
    )
    assert result.returncode != 0


def main() -> int:
    if len(sys.argv) != 4:
        raise SystemExit(
            "usage: test_mujoco_driver_bridge_endpoint_process.py "
            "BRIDGE WRITER_FIXTURE REPOSITORY_ROOT"
        )

    domain_id = domain_id_from_environment()
    bridge_binary = Path(sys.argv[1]).resolve()
    writer_fixture_binary = Path(sys.argv[2]).resolve()
    repository_root = Path(sys.argv[3]).resolve()
    sys.path.insert(0, str(repository_root / "src"))
    sys.path.insert(0, str(repository_root))

    from sim.scripts.mujoco.native_runtime_endpoint import (
        DriverBridgeClient,
        NativeRuntimeEndpointError,
    )

    with tempfile.TemporaryDirectory(prefix="lingtu-driver-endpoint-") as temporary:
        session_root = Path(temporary).resolve()
        readiness = session_root / "driver.ready.json"
        auth = session_root / "driver.auth"
        writer_ready = session_root / "writer.ready"
        writer_stop = session_root / "writer.stop"
        writer_publish = session_root / "writer.stop.publish"
        plan_path = session_root / f"plan-{_PRODUCT_SESSION_ID}.json"
        plan_path.write_text("{}\n", encoding="utf-8")

        environment = dict(os.environ)
        environment.update(
            {
                "LINGTU_RUN_PLAN": str(plan_path),
                "LINGTU_PRODUCT_SESSION_ID": _PRODUCT_SESSION_ID,
                "LINGTU_PRODUCT": "teleop_avoid",
                "LINGTU_ENV": "sim",
                "LINGTU_ENV_BACKEND": "mujoco",
                "LINGTU_SESSION_ROOT": str(session_root),
                "LINGTU_TEST_DDS_DOMAIN_ID": str(domain_id),
            }
        )

        writer: subprocess.Popen[str] | None = None
        bridge: subprocess.Popen[str] | None = None
        try:
            _assert_formal_identity_argument_rejected(
                bridge_binary, environment, "--ready-file", str(session_root / "other.ready")
            )
            _assert_formal_identity_argument_rejected(
                bridge_binary, environment, "--bridge-boot-id", "a" * 32
            )
            _assert_formal_identity_argument_rejected(
                bridge_binary, environment, "--expected-host-boot-id", "free-host-id"
            )
            _assert_formal_identity_argument_rejected(
                bridge_binary, environment, "--product-session-id", _PRODUCT_SESSION_ID
            )
            assert not readiness.exists() and not auth.exists()
            bridge = _start_bridge(bridge_binary, environment)
            # Transport readiness is a process-start barrier, not proof that
            # navd's DDS writer already exists. ProductControl must be able to
            # start all peers in one stage without this bridge hiding its
            # authenticated endpoint until another process is ready.
            _wait_for_file(readiness, bridge, 8.0)
            writer = subprocess.Popen(
                [
                    str(writer_fixture_binary),
                    "--writer-fixture",
                    str(writer_ready),
                    str(writer_stop),
                ],
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            _wait_for_file(writer_ready, writer, 5.0)
            endpoint_payload = json.loads(readiness.read_text(encoding="utf-8"))
            assert endpoint_payload["schema"] == "lingtu.sim.local_endpoint.v1"
            assert endpoint_payload["ready"] is True
            assert endpoint_payload["role"] == "driver_bridge"
            assert endpoint_payload["protocol"] == "driver-v2"
            assert endpoint_payload["product_session_id"] == _PRODUCT_SESSION_ID
            assert endpoint_payload["auth_file"] == "driver.auth"
            assert "driver_ready" not in endpoint_payload
            assert "nonce" not in endpoint_payload

            client = DriverBridgeClient.connect(
                readiness,
                product_session_id=_PRODUCT_SESSION_ID,
                timeout_s=2.0,
            )
            try:
                hello = client.recv_line().split("\t")
                assert len(hello) == 2 and hello[0] == "LT_DRIVER_HELLO_V2", hello
                bridge_boot_id = hello[1]
                assert len(bridge_boot_id) == 32

                client.send_line(
                    f"LT_DRIVER_ACTIVATE_V2\t{bridge_boot_id}\t{_CONTROLLER_BOOT_ID}\t1"
                )
                activation = _command_fields(client.recv_line(), "activation_zero")
                client.send_line(_applied_line(activation, 1))
                client.send_line(
                    f"LT_DRIVER_HEARTBEAT_V2\t{bridge_boot_id}\t{_CONTROLLER_BOOT_ID}\t2\t2"
                )
                ready = client.recv_line().split("\t")
                assert ready[:3] == [
                    "LT_DRIVER_READY_V2",
                    bridge_boot_id,
                    _CONTROLLER_BOOT_ID,
                ]

                writer_publish.write_text("publish\n", encoding="utf-8")
                host_mismatch_zero = _command_fields(
                    client.recv_line(), "safety_zero"
                )
                client.send_line(_applied_line(host_mismatch_zero, 3))
                assert client.recv_line().startswith("LT_DRIVER_READY_V2\t")

                client.send_line(
                    f"LT_DRIVER_DEACTIVATE_V2\t{bridge_boot_id}\t{_CONTROLLER_BOOT_ID}\t3"
                )
                deactivate = _command_fields(client.recv_line(), "deactivate_zero")
                client.send_line(_applied_line(deactivate, 4))
                stopped = client.recv_line().split("\t")
                assert stopped == [
                    "LT_DRIVER_STOPPED_V2",
                    bridge_boot_id,
                    _CONTROLLER_BOOT_ID,
                    deactivate[3],
                    "4",
                    "deactivate_zero",
                ]
            finally:
                client.close()

            return_code = bridge.wait(timeout=5.0)
            if return_code != 0:
                _raise_process_failure(bridge, "driver bridge endpoint failed")
            _wait_endpoint_cleanup(readiness, auth)

            bridge = _start_bridge(bridge_binary, environment)
            _wait_for_file(readiness, bridge, 8.0)
            protocol_client = DriverBridgeClient.connect(
                readiness,
                product_session_id=_PRODUCT_SESSION_ID,
                timeout_s=2.0,
            )
            try:
                hello = protocol_client.recv_line().split("\t")
                assert hello[0] == "LT_DRIVER_HELLO_V2"
                # Deliberately bypass the public 512-byte client guard to prove that
                # an authenticated raw peer cannot overrun the native parser either.
                protocol_client._connection.sendall(b"x" * 513 + b"\n")
                fault = protocol_client.recv_line().split("\t")
                assert fault == [
                    "LT_DRIVER_FAULT_V2",
                    hello[1],
                    "-",
                    "protocol_violation",
                ]
            finally:
                protocol_client.close()
            assert bridge.wait(timeout=5.0) != 0
            bridge_stdout, _ = bridge.communicate(timeout=2.0)
            assert "LT_DRIVER_STOPPED_V2" not in bridge_stdout
            _wait_endpoint_cleanup(readiness, auth)

            if os.name != "nt":
                bridge = _start_bridge(bridge_binary, environment)
                _wait_for_file(readiness, bridge, 8.0)
                os.kill(bridge.pid, signal.SIGTERM)
                assert bridge.wait(timeout=3.0) == 0
                bridge_stdout, _ = bridge.communicate(timeout=2.0)
                assert "LT_DRIVER_STOPPED_V2" not in bridge_stdout
                _wait_endpoint_cleanup(readiness, auth)

            if os.name != "nt":
                bridge = _start_bridge(bridge_binary, environment)
                _wait_for_file(readiness, bridge, 8.0)
                signal_client = DriverBridgeClient.connect(
                    readiness,
                    product_session_id=_PRODUCT_SESSION_ID,
                    timeout_s=2.0,
                )
                hello_fields = signal_client.recv_line().split("\t")
                bridge_boot_id = hello_fields[1]
                signal_client.send_line(
                    f"LT_DRIVER_ACTIVATE_V2\t{bridge_boot_id}\t{_CONTROLLER_BOOT_ID}\t1"
                )
                activation = _command_fields(
                    signal_client.recv_line(), "activation_zero"
                )
                signal_client.send_line(_applied_line(activation, 1))
                signal_client.send_line(
                    f"LT_DRIVER_HEARTBEAT_V2\t{bridge_boot_id}\t{_CONTROLLER_BOOT_ID}\t2\t2"
                )
                assert signal_client.recv_line().startswith("LT_DRIVER_READY_V2\t")
                os.kill(bridge.pid, signal.SIGTERM)
                safety_zero = _command_fields(
                    signal_client.recv_line(), "safety_zero"
                )
                signal_client.send_line(_applied_line(safety_zero, 3))
                try:
                    unexpected = signal_client.recv_line()
                except NativeRuntimeEndpointError:
                    unexpected = ""
                finally:
                    signal_client.close()
                assert not unexpected.startswith("LT_DRIVER_STOPPED_V2\t")
                assert bridge.wait(timeout=5.0) == 0
                _wait_endpoint_cleanup(readiness, auth)

            bridge = _start_bridge(bridge_binary, environment)
            _wait_for_file(readiness, bridge, 8.0)
            eof_client = DriverBridgeClient.connect(
                readiness,
                product_session_id=_PRODUCT_SESSION_ID,
                timeout_s=2.0,
            )
            hello = eof_client.recv_line()
            assert hello.startswith("LT_DRIVER_HELLO_V2\t")
            eof_client.close()
            assert bridge.wait(timeout=5.0) != 0
            bridge_stdout, _ = bridge.communicate(timeout=2.0)
            assert "LT_DRIVER_STOPPED_V2" not in bridge_stdout
            _wait_endpoint_cleanup(readiness, auth)

            bridge = _start_bridge(bridge_binary, environment)
            _wait_for_file(readiness, bridge, 8.0)
            writer_loss_client = DriverBridgeClient.connect(
                readiness,
                product_session_id=_PRODUCT_SESSION_ID,
                timeout_s=2.0,
            )
            try:
                writer_loss_hello = writer_loss_client.recv_line().split("\t")
                assert writer_loss_hello[0] == "LT_DRIVER_HELLO_V2"
                writer_stop.write_text("stop\n", encoding="utf-8")
                assert writer.wait(timeout=5.0) == 0
                writer_loss_fault = writer_loss_client.recv_line().split("\t")
                assert writer_loss_fault == [
                    "LT_DRIVER_FAULT_V2",
                    writer_loss_hello[1],
                    "-",
                    "writer_missing",
                ]
            finally:
                writer_loss_client.close()
            assert bridge.wait(timeout=5.0) != 0
            _wait_endpoint_cleanup(readiness, auth)
        finally:
            if bridge is not None:
                _stop_process(bridge)
            if writer is not None:
                writer_stop.write_text("stop\n", encoding="utf-8")
                try:
                    writer_return_code = writer.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    _stop_process(writer)
                    raise
                if writer_return_code != 0:
                    _raise_process_failure(writer, "DDS writer fixture failed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
