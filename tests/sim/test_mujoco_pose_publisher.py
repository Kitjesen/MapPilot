import threading
from types import SimpleNamespace

from sim.scripts.mujoco.formal_feeder import _RecordPublisher


def test_live_pose_backpressure_keeps_latest_without_accumulating_stale_history():
    entered = threading.Event()
    release = threading.Event()
    written = []

    def write(record):
        written.append(record)
        if len(written) == 1:
            entered.set()
            assert release.wait(3)

    publisher = _RecordPublisher(
        name="odom", client=SimpleNamespace(write=write), stats=None, latest_only=True,
    )
    try:
        publisher.enqueue(b"first")
        assert entered.wait(1)
        for index in range(1024):
            publisher.enqueue(str(index).encode())
        assert publisher._records.qsize() == 1
    finally:
        release.set()
        publisher.close()
    assert written == [b"first", b"1023"]


def test_counted_sensor_records_remain_ordered_and_are_not_coalesced():
    written = []
    publisher = _RecordPublisher(name="imu", client=SimpleNamespace(write=written.append), stats=None)
    for record in (b"first", b"second", b"third"):
        publisher.enqueue(record)
    publisher.close()
    assert written == [b"first", b"second", b"third"]
