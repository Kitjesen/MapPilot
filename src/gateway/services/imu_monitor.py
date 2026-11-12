"""One adapter stream shared by live IMU viewers and durable monitoring records."""

import csv
import io
import json
import math
import re
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import grpc

FIELDS = [f"{group}.{axis}" for group in ("angular_velocity_rad_s", "linear_acceleration_m_s2") for axis in "xyz"]


CSV_FIELDS = [*FIELDS, *(f"orientation_world_to_body.{axis}" for axis in "wxyz")]


def numbers(sample, fields=FIELDS):
    values = sample.get("values", {})
    return [values.get(group, {}).get(axis) for group, axis in (field.split(".") for field in fields)]


class ImuMonitor:
    def __init__(self, source, root, robot=None):
        self.source, self.root, self.robot = source, Path(root), robot
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.samples = deque(maxlen=30000)
        self.sequence = 0
        self.origin = time.monotonic()
        self.requested = -math.inf
        self.cancel = None
        self.worker = None
        self.manager = None
        self.error = None
        self.gap = True
        self.record = None
        self.file = None

    def _available(self):
        return self.source is not None and self.source.capabilities()["sensors"].get("imu", {}).get("stream") is True

    def _start(self):
        if self.closed.is_set():
            raise ValueError("IMU monitor is closed")
        if not self._available():
            raise ValueError("This robot has no configured IMU stream adapter")
        if self.manager is None:
            self.manager = threading.Thread(target=self._manage, name="imu-monitor", daemon=True)
            self.manager.start()

    def _manage(self):
        while not self.closed.is_set():
            with self.lock:
                wanted = self.file is not None or time.monotonic() - self.requested < 3
                if wanted and (self.worker is None or not self.worker.is_alive()):
                    self.worker = threading.Thread(target=self._read, name="imu-source", daemon=True)
                    self.worker.start()
                elif not wanted:
                    if self.cancel is not None:
                        self.cancel()
                    if self.worker is None or not self.worker.is_alive():
                        self.manager = None
                        return
            self.closed.wait(1)

    def _read(self):
        try:
            with self.source.stream("imu") as (samples, cancel):
                with self.lock:
                    self.cancel = cancel
                    self.gap = True
                for values in samples:
                    if self.closed.is_set():
                        break
                    self._append(values)
        except (grpc.RpcError, OSError, ValueError) as error:
            with self.lock:
                self.error = error.details() if isinstance(error, grpc.RpcError) else str(error)
                self.gap = True
        finally:
            with self.lock:
                self.cancel = None

    def _write(self, value):
        self.file.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
        self.file.flush()

    def _append(self, values):
        # Validate source numbers once before sharing them with viewers and the record.
        json.dumps(values, allow_nan=False)
        with self.lock:
            self.sequence += 1
            now = time.monotonic()
            previous = self.samples[-1] if self.samples else None
            elapsed = values.get("source_elapsed")
            old_elapsed = previous["values"].get("source_elapsed") if previous else None
            rollback = elapsed and old_elapsed and (elapsed["seconds"], elapsed.get("nanos", 0)) < (old_elapsed["seconds"], old_elapsed.get("nanos", 0))
            sample = {"seq": self.sequence, "t": now - self.origin, "received_at_s": time.time(),
                      "values": values, "gap_before": bool(self.gap or rollback or (previous and now - self.origin - previous["t"] > 2))}
            self.samples.append(sample)
            while self.samples and sample["t"] - self.samples[0]["t"] > 300:
                self.samples.popleft()
            self.error, self.gap = None, False
            if self.file is not None:
                try:
                    self._write({"type": "sample", **sample})
                    self.record["sample_count"] += 1
                except OSError as error:
                    self._finish("failed", str(error))

    def snapshot(self, after=0):
        with self.lock:
            self.requested = time.monotonic()
            self._start()
            latest = self.samples[-1] if self.samples else None
            age = time.time() - latest["received_at_s"] if latest else None
            if after:
                batch = []
                for row in reversed(self.samples):
                    if row["seq"] <= after or len(batch) == 3000:
                        break
                    batch.append(row)
                batch.reverse()
            else:
                batch = list(self.samples)
            return {"ok": True, "state": "live" if age is not None and age < 2 else "disconnected" if latest or self.error else "connecting",
                    "error": self.error, "provider": self.source.capabilities()["provider"], "robot": self.robot,
                    "source_clock": self.source.capabilities()["sensors"]["imu"].get("clock", "not_reported"),
                    "sequence": self.sequence, "samples": batch, "truncated": bool(batch and after and batch[0]["seq"] > after + 1),
                    "recording": {**self.record, "duration_s": max(0, self.record.get("ended_at_s", time.time()) - self.record["started_at_s"])} if self.record else None}

    def start_record(self):
        with self.lock:
            self._start()
            if self.file:
                return dict(self.record)
            self.root.mkdir(parents=True, exist_ok=True)
            identity = time.strftime("imu-%Y%m%dT%H%M%S", time.gmtime()) + "-" + uuid.uuid4().hex[:8]
            record = {"id": identity, "state": "recording", "started_at_s": time.time(), "sample_count": 0,
                      "start_sequence": self.sequence, "robot": self.robot, "source": self.source.capabilities(), "schema": "lingtu.imu-record.v1"}
            self.file = (self.root / (identity + ".jsonl")).open("x", encoding="utf-8")
            self.record = record
            try:
                self._write({"type": "header", **record})
            except OSError as error:
                self.record.update(state="failed", error=str(error))
                self.file.close()
                self.file = None
                raise
            return dict(record)

    def _finish(self, state, error=None):
        self.record.update(state=state, ended_at_s=time.time(), error=error)
        try:
            self._write({"type": "footer", **self.record})
        finally:
            self.file.close()
            self.file = None

    def stop_record(self, identity):
        with self.lock:
            if self.file:
                if identity != self.record["id"]:
                    raise ValueError("Recording identity does not match the active IMU recording")
                self._finish("completed")
            return self._metadata(self.path(identity))

    def path(self, identity):
        if not re.fullmatch(r"imu-[A-Za-z0-9-]{1,70}", identity):
            raise ValueError("Invalid IMU recording ID")
        path = self.root / (identity + ".jsonl")
        if not path.is_file():
            raise FileNotFoundError("IMU recording not found")
        return path

    def _metadata(self, path):
        try:
            with path.open(encoding="utf-8") as handle:
                header = json.loads(handle.readline())
            with path.open("rb") as handle:
                handle.seek(max(0, path.stat().st_size - 8192))
                lines = handle.read().splitlines()
            footer = {}
            for line in reversed(lines):
                try:
                    footer = json.loads(line)
                    break
                except ValueError:
                    continue
        except (ValueError, OSError):
            return {"id": path.stem, "state": "failed", "sample_count": 0, "started_at_s": 0,
                    "error": "Record metadata is unreadable"}
        if self.file and self.record["id"] == header["id"]:
            return dict(self.record)
        if footer.get("type") == "footer":
            return {**header, **footer}
        return {**header, "state": "interrupted", "ended_at_s": footer.get("received_at_s", header["started_at_s"]),
                "sample_count": max(0, footer.get("seq", header["start_sequence"]) - header["start_sequence"])}

    def records(self):
        with self.lock:
            paths = sorted(self.root.glob("imu-*.jsonl"), reverse=True)[:100]
            return [self._metadata(path) for path in paths]

    def read_record(self, identity, start=0, end=math.inf):
        path = self.path(identity)
        record = self._metadata(path)
        # Keep first/last and each axis extremum per bucket, preserving spikes without loading a whole record.
        buckets, stats, count = {}, {}, 0
        gaps, last_signature = 0, None
        duration = max(.001, record.get("ended_at_s", time.time()) - record["started_at_s"])
        for line in path.open(encoding="utf-8"):
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if row.get("type") != "sample":
                continue
            row["t"] = row["received_at_s"] - record["started_at_s"]
            if not start <= row["t"] <= end:
                continue
            signature = tuple(value is not None for value in numbers(row))
            if row.get("gap_before") or (last_signature is not None and signature != last_signature):
                gaps += 1
            row["_gaps"] = gaps
            last_signature = signature
            count += 1
            bucket = min(119, int(row["t"] / duration * 120))
            selected = buckets.setdefault(bucket, {"first": row})
            selected["last"] = row
            if row.get("gap_before"):
                selected.setdefault("gap", row)
            for index, value in enumerate(numbers(row)):
                if not isinstance(value, (int, float)):
                    continue
                stat = stats.setdefault(FIELDS[index], {"min": value, "max": value, "sum": 0, "square_sum": 0, "count": 0})
                stat.update(min=min(stat["min"], value), max=max(stat["max"], value),
                            sum=stat["sum"] + value, square_sum=stat["square_sum"] + value * value, count=stat["count"] + 1)
                for direction in ("min", "max"):
                    key = f"{direction}{index}"
                    old = numbers(selected[key])[index] if key in selected else None
                    if old is None or (value < old if direction == "min" else value > old):
                        selected[key] = row
        samples = {row["seq"]: row for bucket in buckets.values() for row in bucket.values()}
        ordered, previous_gaps = [], None
        for key in sorted(samples):
            row = samples[key]
            gap_count = row.pop("_gaps")
            row["gap_before"] = bool(row.get("gap_before") or (previous_gaps is not None and gap_count != previous_gaps))
            previous_gaps = gap_count
            ordered.append(row)
        return {"record": record, "samples": ordered, "sampled": True,
                "summary": {"sample_count": count, "axes": {field: {"count": stat["count"], "min": stat["min"], "max": stat["max"],
                    "mean": stat["sum"] / stat["count"], "rms": math.sqrt(stat["square_sum"] / stat["count"])} for field, stat in stats.items()}}}

    def csv(self, identity):
        path = self.path(identity)
        buffer = io.StringIO()
        writer = csv.writer(buffer)
        writer.writerow(["received_at_s", "source_seconds", "source_nanos", "gap_before", *CSV_FIELDS])
        yield buffer.getvalue()
        for line in path.open(encoding="utf-8"):
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if row.get("type") != "sample":
                continue
            buffer.seek(0)
            buffer.truncate(0)
            elapsed = row["values"].get("source_elapsed", {})
            writer.writerow([row["received_at_s"], elapsed.get("seconds", ""), elapsed.get("nanos", ""), row["gap_before"], *numbers(row, CSV_FIELDS)])
            yield buffer.getvalue()

    def close(self):
        self.closed.set()
        with self.lock:
            if self.cancel:
                self.cancel()
            if self.file:
                self._finish("interrupted", "Host stopped")
        for thread in (self.worker, self.manager):
            if thread and thread is not threading.current_thread():
                thread.join(timeout=3)
