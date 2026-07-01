"""Tests for runtime.utils.blackbox_recorder — drift watchdog crash recorder."""

import json
import shutil
import tempfile
import threading
import time
import unittest
from pathlib import Path

from runtime.utils.blackbox_recorder import BlackBoxRecorder, _json_default


class TestBlackBoxRecorderBasics(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="bb_test_"))

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_disabled_is_silent(self):
        bb = BlackBoxRecorder(self.tmp, enabled=False)
        bb.record("odom", {"x": 1.0})
        # Disabled recorder must not allocate buffers nor leave dump artefacts.
        self.assertEqual(bb.channels(), [])
        self.assertIsNone(bb.dump("test"))
        self.assertEqual(list(self.tmp.iterdir()), [])

    def test_record_and_dump_creates_jsonl_per_channel(self):
        bb = BlackBoxRecorder(self.tmp, max_per_channel=10)
        bb.record("odom", {"x": 1.0, "y": 2.0})
        bb.record("odom", {"x": 1.5, "y": 2.5})
        bb.record("slam_diag", {"state": "DEGRADED"})

        out = bb.dump("unit_test", metadata={"reason_code": 42})
        self.assertIsNotNone(out)
        self.assertTrue(out.exists())

        meta = json.loads((out / "metadata.json").read_text())
        self.assertEqual(meta["reason"], "unit_test")
        self.assertEqual(meta["reason_code"], 42)
        self.assertIn("hostname", meta)

        odom_lines = (out / "odom.jsonl").read_text().strip().split("\n")
        self.assertEqual(len(odom_lines), 2)
        first = json.loads(odom_lines[0])
        self.assertIn("t", first)
        self.assertEqual(first["v"]["x"], 1.0)

        slam_lines = (out / "slam_diag.jsonl").read_text().strip().split("\n")
        self.assertEqual(len(slam_lines), 1)
        self.assertEqual(json.loads(slam_lines[0])["v"]["state"], "DEGRADED")

    def test_ring_buffer_respects_max_per_channel(self):
        bb = BlackBoxRecorder(self.tmp, max_per_channel=3)
        for i in range(10):
            bb.record("odom", {"i": i})
        # Only the last 3 entries survive — older ones are evicted by deque(maxlen).
        self.assertEqual(bb.buffer_size("odom"), 3)
        out = bb.dump("ring_test")
        lines = (out / "odom.jsonl").read_text().strip().split("\n")
        kept = [json.loads(line)["v"]["i"] for line in lines]
        self.assertEqual(kept, [7, 8, 9])

    def test_dump_with_no_data_writes_metadata_only(self):
        bb = BlackBoxRecorder(self.tmp)
        out = bb.dump("empty")
        self.assertIsNotNone(out)
        self.assertTrue((out / "metadata.json").exists())
        # No channels recorded → no jsonl files.
        jsonls = list(out.glob("*.jsonl"))
        self.assertEqual(jsonls, [])


class TestBlackBoxRetention(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="bb_retain_"))

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_gc_keeps_only_recent_dumps(self):
        bb = BlackBoxRecorder(self.tmp, retention=2)
        # Pre-create 3 stale dump dirs with progressively older mtimes.
        old_dirs = []
        for i in range(3):
            d = self.tmp / f"drift_{i}"
            d.mkdir()
            (d / "metadata.json").write_text("{}")
            old_dirs.append(d)
        # Force distinct mtimes, oldest first.
        for i, d in enumerate(old_dirs):
            ts = time.time() - (10 - i)
            for f in d.iterdir():
                import os
                os.utime(f, (ts, ts))
            import os
            os.utime(d, (ts, ts))

        # New dump triggers gc; should keep only the 2 newest pre-existing
        # dirs PLUS itself, i.e. the very oldest is removed.
        bb.record("odom", {"x": 0})
        new_out = bb.dump("retention_test")
        remaining = sorted(p.name for p in self.tmp.iterdir() if p.is_dir())
        self.assertIn(new_out.name, remaining)
        # Oldest pre-existing dir gets evicted.
        self.assertNotIn("drift_0", remaining)


class TestJsonDefault(unittest.TestCase):
    def test_numpy_array_serialises_via_tolist(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy not available")
        arr = np.array([1.0, 2.0, 3.0])
        encoded = _json_default(arr)
        self.assertEqual(encoded, [1.0, 2.0, 3.0])

    def test_unknown_object_falls_back_to_repr_dict(self):
        class _Foo:
            def __init__(self):
                self.bar = 1

        encoded = _json_default(_Foo())
        self.assertEqual(encoded["__type__"], "_Foo")
        self.assertIn("_Foo", encoded["repr"])

    def test_dump_with_non_serialisable_does_not_crash(self):
        tmp = Path(tempfile.mkdtemp(prefix="bb_json_"))
        try:
            bb = BlackBoxRecorder(tmp)
            class _NotJSONable:
                pass
            bb.record("weird", _NotJSONable())
            out = bb.dump("non_serialisable")
            self.assertIsNotNone(out)
            line = (out / "weird.jsonl").read_text().strip()
            payload = json.loads(line)
            # Either repr-dict or raw repr — both indicate fallback path engaged.
            self.assertIn("_NotJSONable", json.dumps(payload))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestThreadSafety(unittest.TestCase):
    def test_concurrent_record_and_dump(self):
        tmp = Path(tempfile.mkdtemp(prefix="bb_thread_"))
        try:
            bb = BlackBoxRecorder(tmp, max_per_channel=1000)
            stop = threading.Event()

            def producer():
                i = 0
                while not stop.is_set():
                    bb.record("odom", {"i": i})
                    i += 1

            threads = [threading.Thread(target=producer) for _ in range(4)]
            for t in threads:
                t.start()
            time.sleep(0.05)
            out = bb.dump("threaded")
            stop.set()
            for t in threads:
                t.join()
            self.assertIsNotNone(out)
            # Dump succeeded without exception under contention.
            self.assertTrue((out / "odom.jsonl").exists())
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
