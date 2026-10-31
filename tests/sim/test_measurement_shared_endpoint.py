"""RGB and measurements share the camera socket without interleaving records."""

import time
from concurrent.futures import ThreadPoolExecutor

import pytest
from sim.scripts.mujoco.native_runtime_endpoint import NativeRuntimeEndpointError, SensorPublisherClient


def test_concurrent_record_writers_never_interleave_partial_socket_sends():
    class PartialSocket:
        def __init__(self):
            self.bytes = bytearray()
        def settimeout(self, value):
            pass
        def send(self, data):
            self.bytes.extend(data[:1])
            time.sleep(.001)
            return 1
        def close(self):
            pass

    socket = PartialSocket()
    client = SensorPublisherClient(socket, 2)
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(client.write, b"A" * 24)
        second = pool.submit(client.write, b"B" * 24)
        assert first.result(timeout=2) == second.result(timeout=2) == 24
    assert bytes(socket.bytes) in (b"A" * 24 + b"B" * 24, b"B" * 24 + b"A" * 24)
    client.close()


def test_partial_send_failure_closes_stream_and_rejects_waiting_writer():
    class FailingSocket:
        def __init__(self):
            self.bytes = bytearray()
            self.calls = 0
            self.closed = False

        def settimeout(self, value):
            pass

        def send(self, data):
            self.calls += 1
            if self.calls == 2:
                raise TimeoutError("partial send timed out")
            self.bytes.extend(data[:8])
            return min(8, len(data))

        def close(self):
            self.closed = True

    socket = FailingSocket()
    client = SensorPublisherClient(socket, 2)
    with pytest.raises(NativeRuntimeEndpointError):
        client.write(b"A" * 32)
    with ThreadPoolExecutor(max_workers=1) as pool:
        later = pool.submit(client.write, b"B" * 32)
        with pytest.raises(NativeRuntimeEndpointError, match="closed"):
            later.result(timeout=2)
    assert socket.closed
    assert socket.bytes == b"A" * 8
    client.close()
