"""Read the existing GNSS owner's snapshot; never open the receiver here."""

import json
import math
import os
from pathlib import Path
import time


def geographic_location() -> dict:
    path = Path(os.environ.get("LINGTU_GNSS_STATUS_FILE", "/dev/shm/lingtu/gnss_status.json"))
    result = {"state": "unavailable", "location": None, "datum": "WGS84", "age_s": None}
    try:
        snapshot = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return result
    except (OSError, ValueError):
        return {**result, "state": "invalid"}
    if not isinstance(snapshot, dict):
        return {**result, "state": "invalid"}
    location = snapshot.get("location")
    if location is None:
        return {**result, "state": "no_fix"}
    if not isinstance(location, dict):
        return {**result, "state": "invalid"}
    for field, limit in (("lat", 90), ("lon", 180), ("ts", None)):
        value = location.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            return {**result, "state": "invalid"}
        if limit is not None and abs(value) > limit:
            return {**result, "state": "invalid"}
    fix = location.get("fix_type")
    if type(fix) is not int or fix not in range(1, 9):
        return {**result, "state": "no_fix"}
    age = time.time() - location["ts"]
    if age < -1:
        return {**result, "state": "invalid"}
    result["age_s"] = max(0, age)
    if age > 3:
        return {**result, "state": "stale"}
    accuracy = location.get("accuracy_m")
    if isinstance(accuracy, bool) or not isinstance(accuracy, (int, float)) or not math.isfinite(accuracy) or accuracy < 0:
        accuracy = None
    return {**result, "state": "ready", "location": {
        "latitude": location["lat"], "longitude": location["lon"], "fix_type": fix,
        "timestamp": location["ts"], "accuracy_m": accuracy,
    }}
