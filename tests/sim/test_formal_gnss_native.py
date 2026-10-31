"""Bounded Python feeder-record -> real native DDS interoperability proof.

Body poses are controlled inputs, not a live Product or a driving test.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import time
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import pytest
from sim.runtime.process_owner import ProcessTreeOwner
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.gnss import GnssConfig
from sim.scripts.mujoco.formal_gnss import FormalGnss
from sim.scripts.mujoco.native_sensor_records import encode_gnss


def test_formal_python_records_reach_real_native_dds(tmp_path):
    root = Path(__file__).resolve().parents[2]
    native = root / "build/windows-native-dds-adapter/Release"
    publisher_exe = native / "lingtu_mujoco_sensor_publisher.exe"
    reader_exe = native / "test_gnss_publisher_dds_process.exe"
    if os.name != "nt" or not publisher_exe.is_file() or not reader_exe.is_file():
        pytest.skip("built Windows native DDS executables required")
    base = Path(os.environ.get("LINGTU_GNSS_EVIDENCE_DIR", str(tmp_path)))
    base.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="formal-gnss-wire-", dir=base))
    reader_ready, pub_ready = run / "reader.ready.json", run / "publisher.ready.json"
    received = run / "received.json"
    stream = SensorStreamPlan(
        "gnss",
        "robot",
        "robot.gnss",
        "robot/antenna",
        "lingtu.dds.GnssFix",
        Fraction(10),
        SensorRoute("physics", "mujoco_gnss", "typed_dds"),
        gnss=GnssConfig(0, 0, 0, "robot/base", antenna_offset_m=(0, 0, 0.14)),
    )
    sampler = FormalGnss(stream, "a" * 32, 2, 0)
    data = SimpleNamespace(get_body_pose=lambda name: ([0, 0, 0.6], [1, 0, 0, 0]))
    producer_owner, reader_owner = ProcessTreeOwner(), ProcessTreeOwner()
    producer = reader = None
    sent = {}

    def wait_ready(path, process):
        deadline = time.monotonic() + 8
        while not path.is_file():
            assert process.poll() is None, f"{path.name}: process exited {process.returncode}"
            assert time.monotonic() < deadline, f"{path.name} timeout"
            time.sleep(0.02)

    with (run / "publisher.stderr.log").open("w") as pub_error, (run / "reader.stderr.log").open("w") as read_error:
        try:
            reader = subprocess.Popen(
                [
                    str(reader_exe),
                    "--observe-json",
                    str(received),
                    "--domain-id",
                    "135",
                    "--count",
                    "5",
                    "--count-mode",
                    "minimum",
                    "--ready-file",
                    str(reader_ready),
                ],
                stdout=subprocess.DEVNULL,
                stderr=read_error,
                **reader_owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW),
            )
            reader_owner.attach(reader)
            wait_ready(reader_ready, reader)
            producer = subprocess.Popen(
                [
                    str(publisher_exe),
                    "--stdin-records",
                    "--dds",
                    "--stream",
                    "imu",
                    "--domain-id",
                    "135",
                    "--scan-window",
                    "0",
                    "--ready-file",
                    str(pub_ready),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=pub_error,
                bufsize=0,
                **producer_owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW),
            )
            producer_owner.attach(producer)
            wait_ready(pub_ready, producer)
            for index in range(30):
                if reader.poll() is not None:
                    break
                sampler.set_fix_available(index % 2 == 0)
                sample = sampler.sample(data, index / 10)
                assert sample is not None
                sent[sample.stamp.sim_time_ns] = int(sample.fix.fix_type)
                wire = encode_gnss(sample).wire
                assert producer.stdin.write(wire) == len(wire)
                producer.stdin.flush()
                time.sleep(0.1)
            reader.wait(timeout=5)
            producer.stdin.close()
            producer.wait(timeout=5)
            assert reader.returncode == producer.returncode == 0
            observed = json.loads(received.read_text())
            fixes, statuses = observed["fixes"], observed["statuses"]
            # A continuous stream can deliver extra records while the other
            # reader catches up. Keep all of them and validate each identity.
            assert len(fixes) >= 5 and len(statuses) >= 5
            for records in (fixes, statuses):
                stamps = [record["time_ns"] for record in records]
                assert stamps == sorted(set(stamps))
                assert all(record["fix_type"] == sent[record["time_ns"]] for record in records)
            assert {fix["fix_type"] for fix in fixes} == {0, 8}
            for fix in fixes:
                assert fix["fix_type"] == sent[fix["time_ns"]]
                assert fix["frame_id"] == "robot/antenna"
                if fix["fix_type"] == 8:
                    assert fix["alt"] == pytest.approx(0.74)
                else:
                    assert fix["alt"] is None and fix["cov"] == [None] * 9
            assert all(status["link_ok"] and not status["rtk"] for status in statuses)
        finally:
            for process, owner in ((producer, producer_owner), (reader, reader_owner)):
                try:
                    if process is not None:
                        if process.stdin is not None and not process.stdin.closed:
                            process.stdin.close()
                        if process.poll() is None:
                            owner.terminate(process, timeout_s=2)
                        else:
                            owner.close_after_exit()
                finally:
                    owner.close()
    (run / "result.json").write_text(
        json.dumps(
            {
                "passed": True,
                "scope": "controlled_pose_formal_python_ltu1_native_dds",
                "publisher_exit_code": producer.returncode,
                "reader_exit_code": reader.returncode,
                "fixes_received": len(fixes),
                "statuses_received": len(statuses),
                "sent_time_to_fix_type": sent,
                "run_dir": str(run),
                "first_packet_delivery_guaranteed": False,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"FORMAL_GNSS_WIRE_EVIDENCE={run}")
