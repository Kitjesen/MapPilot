from __future__ import annotations

import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from test_dds_domain import domain_id_from_environment, require_unique_domains


def _wait_until(predicate: object, *, timeout_s: float, message: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if callable(predicate) and predicate():
            return
        time.sleep(0.02)
    raise RuntimeError(message)


def _record(record_type: int, timestamp_ns: int, sequence: int, payload: bytes) -> bytes:
    count = 1
    return (
        b"LTU1"
        + bytes((record_type, 0, 0, 0))
        + struct.pack("<QIII", timestamp_ns, sequence, count, len(payload))
        + payload
    )


def _lidar_fixture() -> bytes:
    records = bytearray()
    base_ns = 2_000_000_000
    for index in range(30):
        records.extend(
            _record(
                1,
                base_ns + index * 50_000_000 + 1_000_000,
                index * 2 + 1,
                struct.pack("<4fIBBH", 1.0 + index, 2.0, 3.0, 42.0, 1000, 7, 8, 0),
            )
        )
        observation_ns = base_ns + index * 50_000_000 + 2_000_000
        records.extend(
            _record(
                3,
                observation_ns,
                index * 2,
                struct.pack(
                    "<10dB7x",
                    1.0 + index,
                    2.0,
                    0.4,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.1,
                    0.0,
                    0.0,
                    1,
                ),
            )
        )
        records.extend(
            _record(
                4,
                observation_ns,
                index * 2 + 1,
                struct.pack(
                    "<4fIBBH",
                    2.0,
                    0.0,
                    -0.4,
                    80.0,
                    0,
                    0,
                    0,
                    0,
                ),
            )
        )
    return bytes(records)


def _imu_fixture() -> bytes:
    return _record(2, 2_000_000_000, 1, struct.pack("<6f", 0.25, -0.5, 0.75, 0.15, 0.25, 1.0))


def _camera_fixture() -> bytes:
    header = struct.pack(
        "<4sHHIIIIddddddIddddd",
        b"LTOB",
        2,
        1,
        2,
        2,
        0,
        0,
        2.0,
        100.0,
        101.0,
        1.0,
        1.0,
        0.001,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    if len(header) != 116:
        raise RuntimeError("camera fixture header layout drifted")
    return _record(5, 2_000_000_000, 1, header)


def _process_environment(session_root: Path, product_session_id: str) -> dict[str, str]:
    plan = session_root / f"plan-{product_session_id}.json"
    plan.write_text("{}\n", encoding="utf-8")
    env = os.environ.copy()
    env.update(
        {
            "LINGTU_RUN_PLAN": str(plan),
            "LINGTU_PRODUCT_SESSION_ID": product_session_id,
            "LINGTU_PRODUCT": "teleop_avoid",
            "LINGTU_ENV": "sim",
            "LINGTU_ENV_BACKEND": "mujoco",
            "LINGTU_SESSION_ROOT": str(session_root),
        }
    )
    return env


def _publisher_command(publisher: Path, domain_id: int, stream: str = "lidar") -> list[str]:
    command = [
        str(publisher),
        "--local-endpoint",
        "--dds",
        "--stream",
        stream,
        "--domain-id",
        str(domain_id),
        "--lidar-frame",
        "lidar_test",
        "--imu-frame",
        "imu_test",
        "--scan-window",
        "0",
    ]
    if stream == "lidar":
        command.append("--navigation-fixture")
    return command


def _start_publisher(
    publisher: Path, session_root: Path, product_session_id: str, domain_id: int, stream: str = "lidar"
) -> subprocess.Popen[bytes]:
    options: dict[str, object] = {
        "cwd": str(session_root),
        "env": _process_environment(session_root, product_session_id),
        "stdin": subprocess.DEVNULL,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.PIPE,
    }
    if os.name == "nt":
        options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        options["start_new_session"] = True
    return subprocess.Popen(_publisher_command(publisher, domain_id, stream), **options)


def _stop_publisher(process: subprocess.Popen[bytes]) -> tuple[bytes, bytes]:
    if os.name == "nt":
        process.send_signal(signal.CTRL_BREAK_EVENT)
    else:
        process.send_signal(signal.SIGTERM)
    return process.communicate(timeout=8.0)


def _failure_text(process: subprocess.Popen[bytes]) -> str:
    stdout, stderr = process.communicate(timeout=1.0)
    return f"stdout={stdout.decode(errors='replace')!r} stderr={stderr.decode(errors='replace')!r}"


def _verify_endpoint_dds_flow(
    publisher: Path,
    observer: Path,
    session_root: Path,
    domain_id: int,
    client_type: object,
    load_endpoint: object,
    sensor_role: str,
    sensor_protocol: str,
) -> None:
    product_session_id = "d" * 32
    readiness = session_root / "lidar.ready.json"
    observer_ready = session_root / "observer.ready"
    process = _start_publisher(publisher, session_root, product_session_id, domain_id)
    observer_process: subprocess.Popen[bytes] | None = None
    client = None
    try:
        _wait_until(
            lambda: readiness.exists() or process.poll() is not None,
            timeout_s=8.0,
            message="sensor endpoint readiness timed out",
        )
        if process.poll() is not None:
            raise RuntimeError(f"sensor publisher exited early: {_failure_text(process)}")
        endpoint = load_endpoint(
            readiness,
            role=sensor_role,
            protocol=sensor_protocol,
            product_session_id=product_session_id,
        )
        auth_path = endpoint.auth_file

        # A listener is connection readiness only.  It must remain alive while
        # ProductControl has not yet connected the MuJoCo producer.
        time.sleep(0.55)
        if process.poll() is not None:
            raise RuntimeError("sensor publisher treated pre-connect idle as startup failure")

        observer_process = subprocess.Popen(
            [str(observer), "--observe-endpoint", str(domain_id), str(observer_ready)],
            cwd=str(session_root),
            env=os.environ.copy(),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        _wait_until(
            lambda: observer_ready.exists() or observer_process.poll() is not None,
            timeout_s=10.0,
            message="DDS observer readiness timed out",
        )
        if observer_process.poll() is not None:
            stdout, stderr = observer_process.communicate(timeout=1.0)
            raise RuntimeError(
                "DDS observer exited before sensor input: "
                f"stdout={stdout.decode(errors='replace')!r} "
                f"stderr={stderr.decode(errors='replace')!r}"
            )

        client = client_type.connect(
            readiness,
            product_session_id=product_session_id,
            timeout_s=2.0,
        )
        # One idle read deadline must not terminate the native process.
        time.sleep(0.55)
        if process.poll() is not None:
            raise RuntimeError("sensor publisher treated authenticated idle as EOF")
        payload = _lidar_fixture()
        if client.write(payload) != len(payload):
            raise RuntimeError("SensorPublisherClient reported a short write")

        observer_stdout, observer_stderr = observer_process.communicate(timeout=12.0)
        if observer_process.returncode != 0:
            raise RuntimeError(
                "DDS observer failed: "
                f"stdout={observer_stdout.decode(errors='replace')!r} "
                f"stderr={observer_stderr.decode(errors='replace')!r}"
            )
        publisher_stdout, publisher_stderr = _stop_publisher(process)
        if process.returncode != 0:
            raise RuntimeError(
                "graceful sensor stop failed: "
                f"stdout={publisher_stdout.decode(errors='replace')!r} "
                f"stderr={publisher_stderr.decode(errors='replace')!r}"
            )
        client.close()
        client = None
        _wait_until(
            lambda: not readiness.exists() and not auth_path.exists(),
            timeout_s=2.0,
            message="sensor endpoint readiness/auth survived graceful process exit",
        )
        if process.poll() is None or observer_process.poll() is None:
            raise RuntimeError("sensor endpoint test left a live process owner")
    finally:
        if client is not None:
            client.close()
        if observer_process is not None and observer_process.poll() is None:
            observer_process.kill()
            observer_process.wait(timeout=5.0)
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)


def _verify_unexpected_eof_fails(
    publisher: Path,
    session_root: Path,
    domain_id: int,
    client_type: object,
) -> None:
    product_session_id = "e" * 32
    readiness = session_root / "lidar.ready.json"
    process = _start_publisher(publisher, session_root, product_session_id, domain_id)
    client = None
    try:
        _wait_until(
            lambda: readiness.exists() or process.poll() is not None,
            timeout_s=8.0,
            message="unexpected-EOF sensor endpoint readiness timed out",
        )
        if process.poll() is not None:
            raise RuntimeError(f"unexpected-EOF publisher exited early: {_failure_text(process)}")
        client = client_type.connect(
            readiness,
            product_session_id=product_session_id,
            timeout_s=2.0,
        )
        client.close()
        client = None
        stdout, stderr = process.communicate(timeout=8.0)
        if process.returncode != 2:
            raise RuntimeError(
                "unexpected authenticated socket EOF did not return protocol failure rc=2: "
                f"stdout={stdout.decode(errors='replace')!r} "
                f"stderr={stderr.decode(errors='replace')!r}"
            )
        if readiness.exists() or (session_root / "lidar.auth").exists():
            raise RuntimeError("failed sensor endpoint left readiness/auth artifacts")
    finally:
        if client is not None:
            client.close()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)


def _verify_three_streams(
    publisher: Path,
    observer: Path,
    root: Path,
    domain_id: int,
    connect_authenticated: object,
    client_type: object,
    load_endpoint: object,
) -> None:
    product_session_id = "f" * 32
    specs = {
        "lidar": ("lidar_publisher", "lidar.ready.json", "lidar.auth", _lidar_fixture()),
        "imu": ("imu_publisher", "imu.ready.json", "imu.auth", _imu_fixture()),
        "camera": ("camera_publisher", "camera.ready.json", "camera.auth", _camera_fixture()),
    }
    processes: dict[str, subprocess.Popen[bytes]] = {}
    clients: list[object] = []
    try:
        for stream in specs:
            session = root / stream
            session.mkdir()
            processes[stream] = _start_publisher(
                publisher, session, product_session_id, domain_id, stream
            )
        for stream, (role, ready_name, auth_name, payload) in specs.items():
            process = processes[stream]
            readiness = root / stream / ready_name
            _wait_until(
                lambda readiness=readiness, process=process: readiness.exists()
                or process.poll() is not None,
                timeout_s=8.0,
                message=f"{stream} readiness timed out",
            )
            if process.poll() is not None:
                raise RuntimeError(f"{stream} exited early: {_failure_text(process)}")
            endpoint = load_endpoint(
                readiness, role=role, protocol="ltu1-v1", product_session_id=product_session_id
            )
            if endpoint.auth_file.name != auth_name:
                raise RuntimeError(f"{stream} auth identity mismatch")
            connection, timeout_s = connect_authenticated(
                readiness,
                role=role,
                protocol="ltu1-v1",
                product_session_id=product_session_id,
                timeout_s=2.0,
            )
            client = client_type(connection, timeout_s)
            clients.append(client)
            if client.write(payload) != len(payload):
                raise RuntimeError(f"{stream} endpoint reported a short write")
        writer_observer = subprocess.run(  # noqa: S603
            [str(observer), "--observe-stream-writers", str(domain_id)],
            cwd=str(root),
            check=False,
            capture_output=True,
            timeout=12.0,
        )
        if writer_observer.returncode != 0:
            raise RuntimeError(
                "split DDS writer ownership failed: "
                f"stdout={writer_observer.stdout!r} stderr={writer_observer.stderr!r}"
            )
        time.sleep(0.5)
        for stream, process in processes.items():
            if process.poll() is not None:
                raise RuntimeError(f"{stream} rejected its own stream: {_failure_text(process)}")
        for process in processes.values():
            _stop_publisher(process)
        for client in clients:
            client.close()
        clients.clear()
        for stream, (_, ready_name, auth_name, _) in specs.items():
            if (root / stream / ready_name).exists() or (root / stream / auth_name).exists():
                raise RuntimeError(f"{stream} left endpoint artifacts")
    finally:
        for client in clients:
            client.close()
        for process in processes.values():
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)


def _verify_wrong_streams_fail(
    publisher: Path,
    root: Path,
    domain_id: int,
    connect_authenticated: object,
    client_type: object,
) -> None:
    product_session_id = "a" * 32
    cases = {
        "lidar": ("lidar_publisher", "lidar.ready.json", _imu_fixture()),
        "imu": ("imu_publisher", "imu.ready.json", _camera_fixture()),
        "camera": ("camera_publisher", "camera.ready.json", _lidar_fixture()),
    }
    for stream, (role, ready_name, wrong_payload) in cases.items():
        session = root / stream
        session.mkdir()
        process = _start_publisher(publisher, session, product_session_id, domain_id, stream)
        client = None
        try:
            readiness = session / ready_name
            _wait_until(
                lambda readiness=readiness, process=process: readiness.exists()
                or process.poll() is not None,
                timeout_s=8.0,
                message=f"{stream} wrong-stream readiness timed out",
            )
            connection, timeout_s = connect_authenticated(
                readiness,
                role=role,
                protocol="ltu1-v1",
                product_session_id=product_session_id,
                timeout_s=2.0,
            )
            client = client_type(connection, timeout_s)
            client.write(wrong_payload)
            stdout, stderr = process.communicate(timeout=8.0)
            if process.returncode != 2:
                raise RuntimeError(
                    f"{stream} accepted another stream: stdout={stdout!r} stderr={stderr!r}"
                )
        finally:
            if client is not None:
                client.close()
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5.0)


def _verify_missing_stream_fails(
    publisher: Path, session_root: Path, domain_id: int
) -> None:
    product_session_id = "b" * 32
    command = _publisher_command(publisher, domain_id)
    stream_index = command.index("--stream")
    del command[stream_index : stream_index + 2]
    process = subprocess.run(  # noqa: S603
        command,
        cwd=str(session_root),
        env=_process_environment(session_root, product_session_id),
        check=False,
        capture_output=True,
        timeout=8.0,
    )
    if process.returncode != 2 or b"requires --stream" not in process.stderr:
        raise RuntimeError(
            "legacy monolithic Product endpoint was not rejected: "
            f"stdout={process.stdout!r} stderr={process.stderr!r}"
        )


def _verify_live_endpoint_rejects_replay(
    publisher: Path, session_root: Path, domain_id: int
) -> None:
    product_session_id = "c" * 32
    process = subprocess.run(  # noqa: S603
        [*_publisher_command(publisher, domain_id), "--replay-rate", "1"],
        cwd=str(session_root),
        env=_process_environment(session_root, product_session_id),
        check=False,
        capture_output=True,
        timeout=8.0,
    )
    if process.returncode != 2 or b"live input" not in process.stderr:
        raise RuntimeError(
            "local endpoint accepted a second offline replay scheduler: "
            f"stdout={process.stdout!r} stderr={process.stderr!r}"
        )


def main() -> int:
    if len(sys.argv) != 4:
        raise RuntimeError(
            "usage: test_sensor_publisher_endpoint_process.py PUBLISHER OBSERVER REPOSITORY"
        )
    publisher = Path(sys.argv[1]).resolve(strict=True)
    observer = Path(sys.argv[2]).resolve(strict=True)
    repository = Path(sys.argv[3]).resolve(strict=True)
    sys.path.insert(0, str(repository / "src"))
    sys.path.insert(0, str(repository))
    from sim.scripts.mujoco.native_runtime_endpoint import (
        SensorPublisherClient,
        _connect_authenticated,
        load_local_endpoint,
    )

    class LidarClient:
        @classmethod
        def connect(cls, readiness_path: Path, *, product_session_id: str, timeout_s: float):
            connection, operation_timeout_s = _connect_authenticated(
                readiness_path,
                role="lidar_publisher",
                protocol="ltu1-v1",
                product_session_id=product_session_id,
                timeout_s=timeout_s,
            )
            return SensorPublisherClient(connection, operation_timeout_s)

    domain_id = domain_id_from_environment()
    failure_domain_id = domain_id_from_environment("LINGTU_TEST_DDS_FAILURE_DOMAIN_ID")
    require_unique_domains(domain_id, failure_domain_id)
    with tempfile.TemporaryDirectory(prefix="lingtu-sensor-endpoint-") as temporary:
        root = Path(temporary).resolve()
        successful = root / "successful"
        successful.mkdir()
        _verify_endpoint_dds_flow(
            publisher,
            observer,
            successful,
            domain_id,
            LidarClient,
            load_local_endpoint,
            "lidar_publisher",
            "ltu1-v1",
        )
        unexpected_eof = root / "unexpected-eof"
        unexpected_eof.mkdir()
        _verify_unexpected_eof_fails(
            publisher,
            unexpected_eof,
            failure_domain_id,
            LidarClient,
        )
        three_streams = root / "three-streams"
        three_streams.mkdir()
        _verify_three_streams(
            publisher,
            observer,
            three_streams,
            domain_id,
            _connect_authenticated,
            SensorPublisherClient,
            load_local_endpoint,
        )
        wrong_streams = root / "wrong-streams"
        wrong_streams.mkdir()
        _verify_wrong_streams_fail(
            publisher,
            wrong_streams,
            failure_domain_id,
            _connect_authenticated,
            SensorPublisherClient,
        )
        missing_stream = root / "missing-stream"
        missing_stream.mkdir()
        _verify_missing_stream_fails(publisher, missing_stream, failure_domain_id)
        replay_scheduler = root / "replay-scheduler"
        replay_scheduler.mkdir()
        _verify_live_endpoint_rejects_replay(
            publisher, replay_scheduler, failure_domain_id
        )
    print("sensor publisher authenticated endpoint DDS process: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"sensor publisher authenticated endpoint DDS process: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
