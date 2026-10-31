"""Read-only scene metadata and camera frames from terminal native recordings."""

from __future__ import annotations

import bisect
import math
import re
import shutil
import subprocess
from functools import lru_cache
from pathlib import Path
from typing import Any

from gateway.services.recording import NativeRecordingError, NativeRecordingService
from gateway.services.recording_files import resolve_declared_artifact


def _terminal_manifest(service: NativeRecordingService, session_id: str) -> dict[str, Any]:
    session = service.manifest(session_id=session_id)
    if session.get("state") not in {"completed", "failed"}:
        raise NativeRecordingError("recording_not_terminal", "recording has not stopped", status_code=409)
    return session


def _declared_names(session: dict[str, Any], child_prefix: str) -> list[str]:
    return [
        name for child in session.get("children", [])
        if isinstance(child, dict) and str(child.get("name", "")).startswith(child_prefix)
        for name in child.get("artifacts", []) if isinstance(name, str)
    ]


def dds_scene_artifact(service: NativeRecordingService, session_id: str) -> Path | None:
    session = _terminal_manifest(service, session_id)
    for name in _declared_names(session, "dds"):
        if name.endswith((".mcap", ".mcap.tmp")):
            return resolve_declared_artifact(service, session_id=session_id, artifact_name=name)[0]
    return None


@lru_cache(maxsize=4)
def _scene_metadata(service: NativeRecordingService, session_id: str) -> dict[str, Any]:
    # Terminal recordings are immutable. Cache their native index descriptors;
    # segment access resolves native declarations before reusing a file path.
    session = _terminal_manifest(service, session_id)
    artifact = dds_scene_artifact(service, session_id)
    scene = service.scene_artifact(artifact, info=True) if artifact is not None else None
    segments: list[dict[str, Any]] = []
    camera_names = _declared_names(session, "camera")
    declared = set(camera_names)
    for name in camera_names:
        if not name.endswith((".mcap", ".mcap.tmp")):
            continue
        index = resolve_declared_artifact(service, session_id=session_id, artifact_name=name)[0]
        camera = service.camera_index_artifact(index)
        for item in camera.get("segments", []):
            relative = (Path(name).parent / item["relative_path"]).as_posix()
            if relative in declared:
                segments.append({"camera_id": camera.get("camera_id", "color"), **item, "artifact": relative, "index_artifact": name})
    origin = int(scene["start_time_ns"]) if scene is not None else min(
        (int(item["start_time_ns"]) for item in segments), default=0,
    )
    for item in segments:
        item["start_s"] = (int(item["start_time_ns"]) - origin) / 1e9
        item["end_s"] = (int(item["end_time_ns"]) - origin) / 1e9
        item["exact"] = item.get("timing") == "frame_timestamp_index"
        item["frame_url"] = (
            f"/api/v1/recordings/{session_id}/camera-frame?segment={item['index']}"
            f"&camera_id={item['camera_id']}&t="
        )
    for camera_id in {item["camera_id"] for item in segments}:
        ordered = sorted((item for item in segments if item["camera_id"] == camera_id), key=lambda item: int(item["start_time_ns"]))
        for index, item in enumerate(ordered):
            begin, end = int(item["start_time_ns"]), int(item["end_time_ns"])
            fps = float(item.get("nominal_fps", 0))
            period_ns = round(1e9 / fps) if fps > 0 else round((end - begin) / (item["frame_count"] - 1)) if item["frame_count"] > 1 else 0
            coverage_end = end + period_ns
            if index + 1 < len(ordered):
                next_start = int(ordered[index + 1]["start_time_ns"])
                # One sampled interval can vary around its nominal cadence.
                # Bridge that normal boundary, but leave missing-frame gaps empty.
                coverage_end = next_start if period_ns > 0 and next_start - end <= period_ns * 3 // 2 else min(coverage_end, next_start)
            item["coverage_end_time_ns"] = str(coverage_end)
            item["coverage_end_s"] = (coverage_end - origin) / 1e9
    duration = max([float(scene["duration_s"]) if scene is not None else 0.0, *[item["coverage_end_s"] for item in segments]])
    return {
        "ok": True, "session_id": session_id, "state": session["state"], "scene": scene,
        "start_time_ns": str(origin), "duration_s": duration,
        "camera": {
            "available": bool(segments),
            "exact": bool(segments) and all(item["exact"] for item in segments),
            "timing": "frame_timestamp_index" if segments and all(item["exact"] for item in segments) else "segment_cfr_estimate",
            "segments": segments,
        },
    }


def recording_scene_info(service: NativeRecordingService, session_id: str) -> dict[str, Any]:
    _terminal_manifest(service, session_id)
    return _scene_metadata(service, session_id)


@lru_cache(maxsize=32)
def _camera_segment(service: NativeRecordingService, session_id: str, index_artifact: str, segment: int) -> dict[str, Any]:
    index = resolve_declared_artifact(service, session_id=session_id, artifact_name=index_artifact)[0]
    rows = service.camera_index_artifact(index, segment=segment).get("segments", [])
    if not rows:
        raise NativeRecordingError("recording_camera_segment_missing", "Camera segment is not present in its index", status_code=404)
    row = rows[0]
    artifact = (Path(index_artifact).parent / row["relative_path"]).as_posix()
    path = resolve_declared_artifact(service, session_id=session_id, artifact_name=artifact)[0]
    return {**row, "resolved_path": path}


def clear_recording_media_cache() -> None:
    _scene_metadata.cache_clear()
    _camera_segment.cache_clear()
    _decode_camera_block.cache_clear()


@lru_cache(maxsize=2)
def _decode_camera_block(executable: str, path: Path, first: int, width: int) -> tuple[tuple[bytes, str | None], ...]:
    arguments = [
        executable, "-hide_banner", "-loglevel", "info", "-nostdin", "-threads", "1", "-filter_threads", "1",
        "-i", str(path), "-an", "-vf",
        f"select=between(n\\,{first}\\,{first + 3}),showinfo,scale=w='min({width},iw)':h='min(720,ih)':force_original_aspect_ratio=decrease",
        "-frames:v", "4", "-vsync", "0", "-f", "image2pipe", "-c:v", "mjpeg", "-threads", "1", "pipe:1",
    ]
    try:
        result = subprocess.run(arguments, stdin=subprocess.DEVNULL, capture_output=True, timeout=15, check=False)
    except subprocess.TimeoutExpired as exc:
        raise NativeRecordingError("recording_camera_decode_timeout", "Camera frame decoding timed out", status_code=504) from exc
    except OSError as exc:
        raise NativeRecordingError("recording_camera_decoder_unavailable", "FFmpeg camera decoder could not start", status_code=503) from exc
    if result.returncode != 0 or len(result.stdout) > 8 * 1024 * 1024:
        raise NativeRecordingError("recording_camera_unreadable", "The indexed camera frame could not be decoded", status_code=422)
    # FFmpeg creates these MJPEG images without source EXIF or embedded JPEGs.
    images = re.findall(b"\xff\xd8.*?\xff\xd9", result.stdout, flags=re.DOTALL)
    if not 1 <= len(images) <= 4 or any(len(image) > 2 * 1024 * 1024 for image in images):
        raise NativeRecordingError("recording_camera_unreadable", "The indexed camera frame could not be decoded", status_code=422)
    times = re.findall(r"pts_time:([0-9.eE+\-]+)", result.stderr.decode("utf-8", errors="replace"))
    return tuple((image, times[index] if index < len(times) else None) for index, image in enumerate(images))


def _decode_camera_frame(executable: str, path: Path, frame: int, width: int) -> tuple[bytes, str | None]:
    first = frame // 4 * 4
    block = _decode_camera_block(executable, path, first, width)
    offset = frame - first
    if offset >= len(block):
        raise NativeRecordingError("recording_camera_unreadable", "The indexed camera frame could not be decoded", status_code=422)
    return block[offset]


def recording_camera_frame(
    service: NativeRecordingService, session_id: str, *, t: float, segment: int,
    camera_id: str = "color", width: int = 960,
) -> tuple[bytes, dict[str, str]] | None:
    if not math.isfinite(t) or not 160 <= width <= 1280:
        raise NativeRecordingError("recording_camera_invalid_query", "Use a finite cursor and width from 160 to 1280", status_code=422)
    metadata = _scene_metadata(service, session_id)
    row = next((item for item in metadata["camera"]["segments"] if item["index"] == segment and item["camera_id"] == camera_id), None)
    if row is None:
        raise NativeRecordingError("recording_camera_segment_missing", "Camera segment is not declared by this recording", status_code=404)
    requested_ns = int(metadata["start_time_ns"]) + round(t * 1e9)
    begin, end = int(row["start_time_ns"]), int(row["end_time_ns"])
    if requested_ns < begin or requested_ns >= int(row["coverage_end_time_ns"]):
        return None
    row = {**row, **_camera_segment(service, session_id, row["index_artifact"], segment)}
    stamps = [int(value) for value in row.get("frame_timestamps_ns", [])]
    if stamps:
        frame = bisect.bisect_right(stamps, requested_ns) - 1
        source_ns = stamps[frame]
        timing = "frame_timestamp_index"
    else:
        count = int(row["frame_count"])
        frame = min(count - 1, math.floor((requested_ns - begin) / (end - begin) * (count - 1))) if end > begin else 0
        source_ns = begin + round((end - begin) * frame / (count - 1)) if count > 1 else begin
        timing = "segment_cfr_estimate"
    path = row["resolved_path"]
    if not path.is_file():
        raise NativeRecordingError("recording_camera_segment_missing", "Camera segment no longer exists", status_code=404)
    executable = shutil.which(service._environ.get("LINGTU_RECORDING_FFMPEG_BIN", "ffmpeg"))
    if executable is None:
        raise NativeRecordingError("recording_camera_decoder_unavailable", "FFmpeg camera decoder is not installed", status_code=503)
    payload, pts = _decode_camera_frame(executable, path, frame, width)
    headers = {
        "X-Recording-Time-S": str((source_ns - int(metadata["start_time_ns"])) / 1e9),
        "X-Recording-Source-Time-Ns": str(source_ns), "X-Recording-Frame-Index": str(frame),
        "X-Recording-Timing": timing, "X-Recording-Frame-Id": str(row.get("frame_id", "")),
        "Cache-Control": "no-store",
    }
    if pts:
        headers["X-Recording-Media-Time-S"] = pts
    return payload, headers
