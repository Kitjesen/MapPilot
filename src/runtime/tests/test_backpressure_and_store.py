"""Tests for:
  - In[T] "latest" policy thread-safety (no double-entry under concurrent publish)
  - In[T] "async" policy (publisher never blocks, callback runs off-thread)
  - EpisodicMemory SQLite persistence (records survive close/reopen)
"""
from __future__ import annotations

import threading
import time

import numpy as np

from runtime.stream import In, Out

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_pair(policy: str, **kwargs):
    out: Out[int] = Out(int, "out")
    inp: In[int] = In(int, "inp")
    inp.set_policy(policy, **kwargs)
    out._add_callback(inp._deliver)
    return out, inp


# ---------------------------------------------------------------------------
# "latest" policy — thread-safety
# ---------------------------------------------------------------------------

class TestLatestPolicy:
    def test_no_double_entry_concurrent(self):
        """Two threads publishing simultaneously must not both enter the callback."""
        concurrent_entries = []
        lock = threading.Lock()
        active = [0]

        def slow_cb(msg):
            with lock:
                active[0] += 1
                concurrent_entries.append(active[0])
            time.sleep(0.05)
            with lock:
                active[0] -= 1

        out, inp = _make_pair("latest")
        inp.subscribe(slow_cb)

        threads = [threading.Thread(target=out.publish, args=(i,)) for i in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Maximum concurrent entries must be exactly 1
        assert max(concurrent_entries) == 1, (
            f"concurrent_entries={concurrent_entries} — double-entry detected"
        )

    def test_drops_counted(self):
        """Messages dropped while callback is busy must increment drop_count."""
        barrier = threading.Barrier(2)
        done = threading.Event()

        def blocking_cb(msg):
            barrier.wait(timeout=2)
            done.wait(timeout=2)

        out, inp = _make_pair("latest")
        inp.subscribe(blocking_cb)

        # First publish — enters callback, blocks at barrier
        t = threading.Thread(target=out.publish, args=(1,))
        t.start()
        barrier.wait(timeout=2)  # wait until callback is executing

        # These should all be dropped
        for i in range(5):
            out.publish(i + 10)

        done.set()
        t.join()

        assert inp.drop_count >= 1


# ---------------------------------------------------------------------------
# "async" policy
# ---------------------------------------------------------------------------

class TestAsyncPolicy:
    def test_publisher_does_not_block(self):
        """publish() must return immediately even with a slow callback."""
        called = threading.Event()

        def slow_cb(msg):
            time.sleep(0.1)
            called.set()

        out, inp = _make_pair("async")
        inp.subscribe(slow_cb)

        t0 = time.time()
        out.publish(42)
        elapsed = time.time() - t0

        # publish must return well before the 100ms callback finishes
        assert elapsed < 0.05, f"publish blocked for {elapsed:.3f}s"
        assert called.wait(timeout=2), "async callback never fired"

    def test_callback_receives_correct_value(self):
        received = []
        ev = threading.Event()

        def cb(msg):
            received.append(msg)
            ev.set()

        out, inp = _make_pair("async")
        inp.subscribe(cb)
        out.publish(99)
        ev.wait(timeout=2)
        assert received == [99]


# ---------------------------------------------------------------------------
# EpisodicMemory SQLite persistence
# ---------------------------------------------------------------------------

class TestEpisodicSQLite:
    def test_records_survive_restart(self, tmp_path):
        from memory.spatial.episodic import EpisodicMemory

        db = tmp_path / "ep.db"

        # First session — write 3 records
        mem1 = EpisodicMemory(persist_path=db)
        for i in range(3):
            mem1.add(
                position=np.array([float(i), float(i)]),
                labels=[f"obj_{i}"],
                room_type="room",
            )
        assert len(mem1) == 3
        mem1.close()

        # Second session — records hydrated from DB
        mem2 = EpisodicMemory(persist_path=db)
        assert len(mem2) == 3
        labels_all = [r.labels[0] for r in mem2.recent_n(10)]
        assert "obj_0" in labels_all
        assert "obj_2" in labels_all
        mem2.close()

    def test_fifo_prune_applied_to_db(self, tmp_path):
        from memory.spatial.episodic import EpisodicMemory

        db = tmp_path / "ep_prune.db"
        mem = EpisodicMemory(persist_path=db)
        mem.MAX_RECORDS = 5

        for i in range(8):
            mem.add(
                position=np.array([float(i) * 2, 0.0]),
                labels=[f"item_{i}"],
            )

        mem.close()

        # Reopen — should have at most 5
        mem2 = EpisodicMemory(persist_path=db)
        mem2.MAX_RECORDS = 5
        assert len(mem2) <= 5
        mem2.close()

    def test_no_persist_path_is_pure_memory(self):
        from memory.spatial.episodic import EpisodicMemory

        mem = EpisodicMemory()
        mem.add(position=np.array([0.0, 0.0]), labels=["x"])
        assert len(mem) == 1
        mem.close()  # no-op, should not raise
