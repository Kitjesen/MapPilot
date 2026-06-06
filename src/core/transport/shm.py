"""core.transport.shm — SharedMemory Transport (high-speed same-machine channel).

Mechanism:
  - POSIX shared memory double buffer
  - Control block: sequence number + timestamp + data length
  - Polling read, supports bytes / numpy / pickle messages

Performance: 900KB image ~200us (vs DDS ~1300us)
Limitation:  same-machine only
"""

import logging
import multiprocessing.shared_memory as pyshm
import struct
import threading
import time
from collections.abc import Callable
from typing import Any

from core.msgs.numpy_compat import is_numpy_array
from .abc import Publisher, Subscriber, TopicConfig, TransportABC

logger = logging.getLogger(__name__)

# Module-level event registry for cross-thread notification (same-process).
# Both SHMPublisher and SHMSubscriber access the same event via shm_name.
# The publisher sets the event after writing new data; the subscriber waits
# on the event instead of busy-wait polling.  This eliminates CPU-burning
# time.sleep() in the subscriber loop while keeping sub-2ms latency.
# For cross-process scenarios the timeout acts as a safety-net fallback.
_EVENT_REGISTRY: dict[str, threading.Event] = {}


def _get_notify_event(shm_name: str) -> threading.Event:
    if shm_name not in _EVENT_REGISTRY:
        _EVENT_REGISTRY[shm_name] = threading.Event()
    return _EVENT_REGISTRY[shm_name]


# Control block: [seq:8bytes][timestamp:8bytes][data_len:8bytes] = 24 bytes
CTRL_SIZE = 24
DEFAULT_BUF_SIZE = 4 * 1024 * 1024  # 4MB (enough for 1080p RGB)


def _shm_name(topic: str) -> str:
    """Convert topic name to valid shared memory name."""
    return "lingtu_" + topic.replace("/", "_").strip("_")


class SHMPublisher(Publisher):
    """Shared memory publisher — writes to double buffer."""

    def __init__(self, topic: TopicConfig):
        super().__init__(topic)
        buf_size = topic.buffer_size or DEFAULT_BUF_SIZE
        self._total_size = CTRL_SIZE + buf_size
        self._shm_name = _shm_name(topic.name)
        self._seq = 0
        self._event = _get_notify_event(self._shm_name)

        # Create or attach shared memory
        try:
            self._shm = pyshm.SharedMemory(
                name=self._shm_name, create=True, size=self._total_size
            )
            self._owner = True
        except FileExistsError:
            self._shm = pyshm.SharedMemory(name=self._shm_name, create=False)
            self._owner = False

        logger.info(f"[SHM-Pub] {topic.name} -> {self._shm_name} ({buf_size // 1024}KB)")

    def publish(self, msg: Any) -> None:
        """Publish message (bytes or numpy array)."""
        if is_numpy_array(msg):
            data = msg.tobytes()
        elif isinstance(msg, bytes):
            data = msg
        elif isinstance(msg, memoryview):
            data = bytes(msg)
        else:
            import pickle
            data = pickle.dumps(msg)

        data_len = len(data)
        max_data = self._total_size - CTRL_SIZE
        if data_len > max_data:
            raise ValueError(
                f"[SHM-Pub] {self._shm_name}: data size {data_len} exceeds "
                f"buffer capacity {max_data}. Increase buffer_size in TopicConfig."
            )

        complete_seq = self._seq + 2
        busy_seq = complete_seq - 1
        ts = time.time()
        buf = self._shm.buf

        struct.pack_into("<QdQ", buf, 0, busy_seq, ts, data_len)
        buf[CTRL_SIZE : CTRL_SIZE + data_len] = data
        struct.pack_into("<QdQ", buf, 0, complete_seq, ts, data_len)
        self._seq = complete_seq
        self._event.set()  # Notify subscribers: new data available

    def close(self) -> None:
        """Close shared memory handle and unlink if owner."""
        try:
            self._shm.close()
            if self._owner:
                self._shm.unlink()
        except Exception:
            logger.debug("[SHM-Pub] error closing shared memory", exc_info=True)


class SHMSubscriber(Subscriber):
    """Shared memory subscriber — polling read."""

    def __init__(self, topic: TopicConfig, callback: Callable, poll_interval: float = 0.002):
        super().__init__(topic, callback)
        self._shm_name = _shm_name(topic.name)
        self._poll_interval = poll_interval
        self._last_seq = 0
        self._running = False
        self._thread: threading.Thread | None = None
        self._shm: pyshm.SharedMemory | None = None
        self._event = _get_notify_event(self._shm_name)

    def start(self) -> None:
        """Start polling thread. Always starts — retries SHM attach in poll loop."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _try_attach(self) -> bool:
        """Try to attach to shared memory region. Returns True on success."""
        if self._shm is not None:
            return True
        try:
            self._shm = pyshm.SharedMemory(name=self._shm_name, create=False)
            logger.info(f"[SHM-Sub] {self._topic.name} <- {self._shm_name}")
            return True
        except FileNotFoundError:
            return False

    def _poll_loop(self) -> None:
        while self._running:
            if not self._try_attach():
                # Publisher hasn't created the region yet — wait and retry
                self._event.wait(timeout=0.1)
                self._event.clear()
                continue
            buf = self._shm.buf
            seq, ts, data_len = struct.unpack_from("<QdQ", buf, 0)
            if seq > self._last_seq and seq % 2 == 0 and data_len > 0:
                data = bytes(buf[CTRL_SIZE : CTRL_SIZE + data_len])
                seq2, ts2, data_len2 = struct.unpack_from("<QdQ", buf, 0)
                if seq2 != seq or ts2 != ts or data_len2 != data_len or seq2 % 2 != 0:
                    # Inconsistent read — wait and retry
                    self._event.wait(timeout=self._poll_interval)
                    self._event.clear()
                    continue
                self._last_seq = seq2
                try:
                    self._callback(data, ts2)
                except Exception as e:
                    logger.error(f"[SHM-Sub] callback error: {e}")
            # Wait for publisher notification instead of busy-wait polling.
            # The publisher sets self._event after writing new data to SHM.
            # The timeout is a safety net for the narrow lost-wakeup race
            # between event.set() and event.clear() — worst-case ~2ms lag.
            self._event.wait(timeout=self._poll_interval)
            self._event.clear()

    def close(self) -> None:
        """Stop polling thread and close shared memory."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._shm:
            try:
                self._shm.close()
            except Exception:
                logger.debug("[SHM-Sub] error closing shared memory", exc_info=True)


class SHMTransport(TransportABC):
    """SharedMemory transport layer."""

    def __init__(self):
        self._publishers: list = []
        self._subscribers: list = []

    def create_publisher(self, topic: TopicConfig) -> SHMPublisher:
        """Create a shared memory publisher for the given topic."""
        pub = SHMPublisher(topic)
        self._publishers.append(pub)
        return pub

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> SHMSubscriber:
        """Create and start a shared memory subscriber for the given topic."""
        sub = SHMSubscriber(topic, callback)
        sub.start()
        self._subscribers.append(sub)
        return sub

    def close(self) -> None:
        """Close all publishers and subscribers."""
        for s in self._subscribers:
            s.close()
        for p in self._publishers:
            p.close()
        self._subscribers.clear()
        self._publishers.clear()

    @property
    def name(self) -> str:
        """Transport backend name."""
        return "shm"
