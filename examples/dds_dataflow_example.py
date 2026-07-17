"""DDS dataflow example — Producer/Consumer via Blueprint with DDS transport.

Demonstrates how to:
    1. Define a producer Module with an Out port that publishes periodically.
    2. Define a consumer Module with an In port that receives and prints data.
    3. Assemble both modules in a Blueprint with DDS delivery.
    4. Gracefully fall back to local transport when DDS is unavailable.

Usage:
    # From the repository root
    python examples/dds_dataflow_example.py

The producer emits a counter every 0.5 s; the consumer prints each message.
Press Ctrl-C to stop.  All resources are cleaned up on exit.

Notes:
    - No ROS 2 or external DDS implementation is required to run this file.
      When the CycloneDDS backend is absent the Blueprint silently downgrades
      to local transport so the example still works.
    - This pattern can be copied to integrate any new sensor data flow into
      the LingTu runtime.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

# ---------------------------------------------------------------------------
# Adjust sys.path so the example can be run directly from the repo root
# without an editable install.
# ---------------------------------------------------------------------------
_REPO_ROOT = __import__("pathlib").Path(__file__).resolve().parent.parent
if str(_REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "src"))

from runtime.blueprint import Blueprint
from runtime.module import Module
from runtime.registry import register
from runtime.stream import In, Out

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Message type
# ---------------------------------------------------------------------------


@dataclass
class SensorReading:
    """Minimal sensor-reading payload for the example."""

    seq: int
    value: float
    timestamp: float


# ---------------------------------------------------------------------------
# Producer Module
# ---------------------------------------------------------------------------


@register("example", "producer", description="Periodic sensor publisher")
class ExampleProducerModule(Module, layer=1):
    """Publishes a monotonically increasing counter as a SensorReading.

    Out ports:
        sensor_data: Out[SensorReading] — emitted every ``interval`` seconds.
    """

    sensor_data: Out[SensorReading]

    def __init__(self, *, interval: float = 0.5, **config: Any) -> None:
        super().__init__(interval=interval, **config)
        self._interval = interval
        self._seq = 0
        self._timer_thread: threading.Thread | None = None

    def start(self) -> None:
        super().start()
        self._timer_thread = threading.Thread(
            target=self._publish_loop,
            daemon=True,
            name="ExampleProducer",
        )
        self._timer_thread.start()

    def stop(self) -> None:
        super().stop()
        if self._timer_thread is not None:
            self._timer_thread.join(timeout=2.0)
            self._timer_thread = None

    def _publish_loop(self) -> None:
        while self.running:
            reading = SensorReading(
                seq=self._seq,
                value=float(self._seq) * 0.1,
                timestamp=time.time(),
            )
            self.sensor_data.publish(reading)
            logger.info("Published: seq=%d value=%.1f", reading.seq, reading.value)
            self._seq += 1
            time.sleep(self._interval)


# ---------------------------------------------------------------------------
# Consumer Module
# ---------------------------------------------------------------------------


@register("example", "consumer", description="Sensor data subscriber")
class ExampleConsumerModule(Module, layer=3):
    """Subscribes to sensor_data and prints each received reading.

    In ports:
        sensor_data: In[SensorReading] — receives readings from the producer.
    """

    sensor_data: In[SensorReading]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self.received: list[SensorReading] = []

    def setup(self) -> None:
        self.sensor_data.subscribe(self._on_reading)

    def _on_reading(self, msg: SensorReading) -> None:
        self.received.append(msg)
        logger.info(
            "Received: seq=%d value=%.1f ts=%.3f",
            msg.seq,
            msg.value,
            msg.timestamp,
        )


# ---------------------------------------------------------------------------
# Assembly helper
# ---------------------------------------------------------------------------


def build_example_blueprint() -> Blueprint:
    """Build a Blueprint with the producer/consumer pair wired via DDS.

    The wire uses ``delivery="dds"`` with an explicit topic so that the DDS
    backend (when available) routes data across processes.  When the DDS
    backend is not installed, the Blueprint build step transparently falls
    back to local transport.
    """
    bp = (
        Blueprint("dds_dataflow_example")
        .add(ExampleProducerModule, interval=0.5)
        .add(ExampleConsumerModule)
        .wire(
            "ExampleProducerModule",
            "sensor_data",
            "ExampleConsumerModule",
            "sensor_data",
            delivery="dds",
            topic="/example/sensor_data",
        )
    )
    return bp


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(duration: float = 3.0) -> None:
    """Run the example for *duration* seconds then shut down."""
    bp = build_example_blueprint()
    system = bp.build()
    system.start()

    logger.info("System running for %.1f seconds ...", duration)
    time.sleep(duration)

    system.stop()
    logger.info("System stopped.")

    # Report how many messages the consumer received.
    try:
        consumer = system.get_module("ExampleConsumerModule")
    except KeyError:
        consumer = None
    if consumer is not None and hasattr(consumer, "received"):
        logger.info("Consumer received %d messages.", len(consumer.received))


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    main(dur)
