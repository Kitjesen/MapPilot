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

    def test_final_pending_message_drains_without_another_publish(self):
        entered = threading.Event()
        release = threading.Event()
        received = []

        def blocking_cb(msg):
            received.append(msg)
            if msg == "A":
                entered.set()
                release.wait(timeout=2.0)

        out, inp = _make_pair("latest")
        inp.subscribe(blocking_cb)
        first = threading.Thread(target=out.publish, args=("A",))
        first.start()
        assert entered.wait(timeout=1.0)

        try:
            out.publish("B")
            out.publish("STOP")
        finally:
            release.set()
            first.join(timeout=2.0)

        assert not first.is_alive()
        assert received == ["A", "STOP"]
        assert inp.drop_count == 1
        assert inp.deliver_count == 2

    def test_clear_subscriber_cancels_pending_latest_message(self):
        entered = threading.Event()
        release = threading.Event()
        received = []

        def blocking_cb(msg):
            received.append(msg)
            if msg == "A":
                entered.set()
                release.wait(timeout=2.0)

        out, inp = _make_pair("latest")
        inp.subscribe(blocking_cb)
        first = threading.Thread(target=out.publish, args=("A",))
        first.start()
        assert entered.wait(timeout=1.0)

        try:
            out.publish("B")
            inp._clear_subscriber()
        finally:
            release.set()
            first.join(timeout=2.0)

        assert not first.is_alive()
        assert received == ["A"]
        assert inp.connected is False

        out.publish("C")
        assert received == ["A"]

    def test_reentrant_publish_is_serialized_and_coalesced(self):
        inp = In(int, "latest")
        inp.set_policy("latest")
        received = []
        active = 0
        max_active = 0

        def callback(msg):
            nonlocal active, max_active
            active += 1
            max_active = max(max_active, active)
            received.append(msg)
            if msg == 1:
                inp._deliver(2)
                inp._deliver(3)
            active -= 1

        inp.subscribe(callback)

        inp._deliver(1)

        assert received == [1, 3]
        assert max_active == 1
        assert inp.drop_count == 1
        assert inp.deliver_count == 2

    def test_callback_error_does_not_strand_pending_message(self):
        entered = threading.Event()
        release = threading.Event()
        received = []

        def failing_cb(msg):
            received.append(msg)
            if msg == "A":
                entered.set()
                release.wait(timeout=2.0)
                raise RuntimeError("expected test failure")

        out, inp = _make_pair("latest")
        inp.subscribe(failing_cb)
        first = threading.Thread(target=out.publish, args=("A",))
        first.start()
        assert entered.wait(timeout=1.0)

        try:
            out.publish("STOP")
        finally:
            release.set()
            first.join(timeout=2.0)

        assert not first.is_alive()
        assert received == ["A", "STOP"]
        assert inp.callback_errors == 1
        assert inp.deliver_count == 2


# ---------------------------------------------------------------------------
# "async" policy
# ---------------------------------------------------------------------------

class TestAsyncPolicy:
    def test_publisher_does_not_block(self):
        """publish() must return immediately even with a slow callback."""
        callback_started = threading.Event()
        release_callback = threading.Event()
        called = threading.Event()
        publish_returned = threading.Event()

        def slow_cb(msg):
            callback_started.set()
            release_callback.wait(timeout=2)
            called.set()

        out, inp = _make_pair("async")
        inp.subscribe(slow_cb)

        publisher = threading.Thread(
            target=lambda: (out.publish(42), publish_returned.set()),
            daemon=True,
        )
        publisher.start()
        assert callback_started.wait(timeout=1), "async callback never started"
        try:
            assert publish_returned.wait(timeout=0.5), "publish waited for the callback to finish"
            assert not called.is_set(), "callback completed before its release signal"
        finally:
            release_callback.set()
        publisher.join(timeout=2)
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
