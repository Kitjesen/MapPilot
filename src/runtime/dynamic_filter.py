"""DUFOMap offline dynamic-obstacle filter for saved PGO maps.

Called from map-save flows after the raw SLAM/PGO map-save adapter has produced
map.pcd plus optional patches, and before occupancy-grid / tomogram build.
Takes PGO output (patches + poses) and overwrites map.pcd with a
statically-cleaned version.

Design notes
------------
- Shells out to the C++ ``dufomap_run`` binary (built once at ``~/src/dufomap``).
  We don't use Python bindings because there is no aarch64 wheel on PyPI and
  building the binding in-process is fragile. Subprocess IPC is fast enough
  (0.6s for 105 patches × 1600 pts) and isolates DUFOMap crashes from the
  gateway.
- ``dufomap_run`` expects ``<data_dir>/pcd/*.pcd`` where each PCD's VIEWPOINT
  header encodes the sensor pose. Our PGO dumps patches with identity
  VIEWPOINT + a separate ``poses.txt``. We repack in a tempdir before running.
- Failure is non-fatal: every exception is swallowed, original map.pcd stays
  intact, and the caller sees ``{"success": False, "error": ...}``.
"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)

DUFOMAP_BIN = os.environ.get(
    "LINGTU_DUFOMAP_BIN",
    os.path.expanduser("~/src/dufomap/build/dufomap_run"),
)


def _default_config_path() -> str:
    """Prefer project's config/dufomap.toml (Livox-tuned), fall back to upstream."""
    # Walk up from this file (src/nav/services/) to repo root,
    # then check config/dufomap.toml.
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        cand = os.path.join(here, "config", "dufomap.toml")
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return os.path.expanduser("~/src/dufomap/assets/config.toml")


DUFOMAP_CONFIG = os.environ.get("LINGTU_DUFOMAP_CONFIG", _default_config_path())


def _read_pcd_binary(path: Path) -> np.ndarray:
    """Return (N, 3) float32 xyz, discarding extra fields (intensity etc.)."""
    with open(path, "rb") as f:
        raw = f.read()
    marker = b"DATA binary\n"
    end = raw.find(marker)
    if end < 0:
        raise RuntimeError(f"not a binary PCD: {path}")
    header = raw[:end].decode("ascii", errors="ignore")
    data = raw[end + len(marker):]

    sizes: list[int] = []
    for line in header.splitlines():
        if line.startswith("SIZE"):
            sizes = [int(p) for p in line.split()[1:]]
            break
    stride = sum(sizes) // 4 if sizes else 4
    return np.frombuffer(data, dtype=np.float32).reshape(-1, stride)[:, :3].copy()


def _write_pcd_binary(path: Path, pts: np.ndarray, viewpoint: tuple[float, ...]) -> None:
    """Write minimal (x,y,z) PCD v0.7 binary with given VIEWPOINT."""
    n = len(pts)
    vp_str = " ".join(f"{v:g}" for v in viewpoint)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT {vp_str}\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.astype(np.float32, copy=False).tobytes())


def _parse_poses(poses_path: Path) -> dict[str, tuple[float, ...]]:
    """poses.txt → {patch_name: (tx,ty,tz,qw,qx,qy,qz)}."""
    out: dict[str, tuple[float, ...]] = {}
    with open(poses_path, encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 8:
                out[parts[0]] = tuple(float(p) for p in parts[1:8])
    return out


def _count_pcd_points(path: Path) -> int:
    with open(path, "rb") as f:
        head = f.read(1024).decode("ascii", errors="ignore")
    for line in head.splitlines():
        if line.startswith("POINTS"):
            return int(line.split()[1])
    return -1


def refilter_map(map_dir: str | Path, *, timeout_s: float = 300.0) -> dict:
    """Run DUFOMap on patches+poses, overwrite map.pcd in place.

    Returns a result dict with:
      success:       bool
      clean_count:   int (points in new map.pcd)
      orig_count:    int (points in pre-filter map.pcd)
      dropped:       int
      elapsed_s:     float
      error:         str (only on failure)

    Never raises. On failure the original map.pcd is left untouched.
    """
    t0 = time.time()
    map_dir = Path(map_dir)
    patches = map_dir / "patches"
    poses = map_dir / "poses.txt"
    orig_map = map_dir / "map.pcd"

    if not Path(DUFOMAP_BIN).is_file():
        return {"success": False, "error": f"dufomap binary missing at {DUFOMAP_BIN}",
                "elapsed_s": time.time() - t0}
    if not patches.is_dir() or not poses.is_file() or not orig_map.is_file():
        return {"success": False,
                "error": f"PGO output incomplete in {map_dir}",
                "elapsed_s": time.time() - t0}

    tmp = Path(tempfile.mkdtemp(prefix="dufomap_"))
    try:
        pose_map = _parse_poses(poses)
        pcd_dir = tmp / "pcd"
        pcd_dir.mkdir(parents=True, exist_ok=True)
        repacked = 0
        for patch in sorted(patches.glob("*.pcd")):
            if patch.name not in pose_map:
                continue
            pts = _read_pcd_binary(patch)
            _write_pcd_binary(pcd_dir / patch.name, pts, pose_map[patch.name])
            repacked += 1

        if repacked == 0:
            return {"success": False, "error": "no patches matched poses.txt",
                    "elapsed_s": time.time() - t0}

        # dufomap_run overwrites output.pcd (or dufomap_output.pcd — depends on config)
        proc = subprocess.run(
            [DUFOMAP_BIN, str(tmp), DUFOMAP_CONFIG],
            capture_output=True, text=True, encoding="utf-8",
            errors="replace", timeout=timeout_s,
        )
        if proc.returncode != 0:
            return {"success": False,
                    "error": f"dufomap exit {proc.returncode}: {proc.stderr[-400:]}",
                    "elapsed_s": time.time() - t0}

        # Find output pcd (usually "dufomap_output.pcd" or "output.pcd")
        clean: Path | None = None
        for cand in ("dufomap_output.pcd", "output.pcd"):
            p = tmp / cand
            if p.is_file():
                clean = p
                break
        if clean is None:
            # last resort: any *.pcd directly under tmp (not in pcd/)
            for p in tmp.glob("*.pcd"):
                clean = p
                break
        if clean is None:
            return {"success": False, "error": "dufomap produced no output pcd",
                    "elapsed_s": time.time() - t0}

        orig_count = _count_pcd_points(orig_map)
        clean_count = _count_pcd_points(clean)
        if clean_count <= 0:
            return {"success": False,
                    "error": f"invalid clean pcd (count={clean_count})",
                    "elapsed_s": time.time() - t0}

        # Backup original. If backup fails (disk full / permission), ABORT —
        # we must not overwrite without a restorable backup.
        backup = map_dir / "map.pcd.predufo"
        try:
            shutil.copy(orig_map, backup)
        except Exception as e:
            logger.error("dynamic_filter: backup failed, aborting: %s", e)
            return {"success": False,
                    "error": f"backup failed: {e}",
                    "elapsed_s": time.time() - t0}

        # Atomic overwrite: copy to sibling tmp then rename.
        # shutil.copy is NOT atomic — a crash mid-copy leaves a half-written
        # map.pcd that SLAM will fail to load next time. os.replace on the
        # same filesystem is POSIX-atomic.
        tmp_new = orig_map.with_suffix(".pcd.tmp")
        try:
            shutil.copy(clean, tmp_new)
            os.replace(tmp_new, orig_map)
        except Exception as e:
            logger.error("dynamic_filter: atomic overwrite failed: %s", e)
            try:
                if tmp_new.exists():
                    tmp_new.unlink()
            except Exception:
                pass
            return {"success": False,
                    "error": f"overwrite failed: {e}",
                    "elapsed_s": time.time() - t0}

        elapsed = time.time() - t0
        dropped = orig_count - clean_count if orig_count > 0 else 0
        logger.info(
            "dynamic_filter: %s %d→%d pts (-%d, %.1f%%) in %.1fs",
            map_dir.name, orig_count, clean_count, dropped,
            100 * dropped / orig_count if orig_count > 0 else 0.0,
            elapsed,
        )
        return {
            "success": True,
            "orig_count": orig_count,
            "clean_count": clean_count,
            "dropped": dropped,
            "elapsed_s": elapsed,
            "backup": str(backup),
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": f"dufomap timeout after {timeout_s}s",
                "elapsed_s": time.time() - t0}
    except Exception as e:
        logger.exception("dynamic_filter failed")
        return {"success": False, "error": str(e), "elapsed_s": time.time() - t0}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def apply_dynamic_filter_step1half(save_dir: str | os.PathLike[str]) -> dict | None:
    """Run the optional DUFOMap filter after PGO and before tomogram build."""
    if os.environ.get("LINGTU_SAVE_DYNAMIC_FILTER", "1") in (
        "0",
        "false",
        "False",
        "FALSE",
        "no",
        "off",
        "",
    ):
        return None
    try:
        result = refilter_map(save_dir, timeout_s=300.0)
        if result.get("success"):
            orig = result.get("orig_count", 0)
            clean = result.get("clean_count", 0)
            dropped = result.get("dropped", 0)
            pct = 100 * dropped / max(1, orig)
            logger.info(
                "dynamic_filter: %s %d->%d pts (-%d, %.1f%%) in %.1fs",
                os.path.basename(str(save_dir)),
                orig,
                clean,
                dropped,
                pct,
                result.get("elapsed_s", 0.0),
            )
        else:
            logger.warning("dynamic_filter: skipped: %s", result.get("error"))
        return result
    except Exception as e:
        logger.warning("dynamic_filter: crashed (non-fatal): %s", e)
        return {"success": False, "error": str(e)}
