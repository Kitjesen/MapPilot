"""Small PCD helpers for Gateway services."""

from __future__ import annotations

import logging

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


def load_xyz(path: str):
    """Load XYZ points from an ASCII or binary little-endian PCD file."""
    try:
        with open(path, "rb") as file:
            header_bytes = b""
            while True:
                line = file.readline()
                if not line:
                    break
                header_bytes += line
                if line.upper().startswith(b"DATA"):
                    break
            header = header_bytes.decode("ascii", errors="ignore")
            fields: list[str] = []
            sizes: list[int] = []
            count = 0
            data_format = "ascii"
            for line in header.splitlines():
                line = line.strip()
                if line.startswith("FIELDS"):
                    fields = line.split()[1:]
                elif line.startswith("SIZE"):
                    sizes = [int(value) for value in line.split()[1:]]
                elif line.startswith("POINTS"):
                    count = int(line.split()[1])
                elif line.startswith("DATA"):
                    data_format = line.split()[1].lower()
            if not fields or "x" not in fields or "y" not in fields:
                return None
            x_index = fields.index("x")
            y_index = fields.index("y")
            z_index = fields.index("z") if "z" in fields else -1
            if data_format == "ascii":
                return _load_ascii_xyz(file, fields, x_index, y_index, z_index)
            if data_format == "binary":
                return _load_binary_xyz(
                    file,
                    fields,
                    sizes,
                    count,
                    x_index,
                    y_index,
                    z_index,
                )
    except Exception as exc:
        logger.debug("PCD load failed %s: %s", path, exc)
        return None
    return None


def _load_ascii_xyz(file, fields: list[str], x_index: int, y_index: int, z_index: int):
    points = []
    for line in file.read().decode(errors="ignore").splitlines():
        parts = line.split()
        if len(parts) < len(fields):
            continue
        try:
            x = float(parts[x_index])
            y = float(parts[y_index])
            z = float(parts[z_index]) if z_index >= 0 else 0.0
            points.append((x, y, z))
        except ValueError:
            pass
    return np.array(points, dtype=np.float32) if points else None


def _load_binary_xyz(
    file,
    fields: list[str],
    sizes: list[int],
    count: int,
    x_index: int,
    y_index: int,
    z_index: int,
):
    if not sizes:
        sizes = [4] * len(fields)
    stride = sum(sizes)
    raw = np.frombuffer(file.read(), dtype=np.uint8)
    rows = count or (len(raw) // stride)
    raw = raw[: rows * stride].reshape(rows, stride)
    offsets = [sum(sizes[:idx]) for idx in range(len(sizes))]
    out = np.zeros((rows, 3), dtype=np.float32)
    out[:, 0] = np.frombuffer(
        raw[:, offsets[x_index] : offsets[x_index] + 4].tobytes(),
        dtype=np.float32,
    )
    out[:, 1] = np.frombuffer(
        raw[:, offsets[y_index] : offsets[y_index] + 4].tobytes(),
        dtype=np.float32,
    )
    if z_index >= 0:
        out[:, 2] = np.frombuffer(
            raw[:, offsets[z_index] : offsets[z_index] + 4].tobytes(),
            dtype=np.float32,
        )
    finite = np.isfinite(out).all(axis=1)
    return out[finite]
