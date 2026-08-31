#!/usr/bin/env python3
"""DUFOMap offline validation on existing saved-map patch data.

Reads ~/data/nova/maps/<map>/patches/*.pcd + poses.txt, repacks each patch
with VIEWPOINT header = pose (what dufomap_run expects), runs the binary,
and reports static vs original point count.

Usage:
    python3 tools/maps/dufomap_offline_test.py <map_name>
    python3 tools/maps/dufomap_offline_test.py corrected_20260406_224020
"""
from __future__ import annotations

import argparse
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

DUFOMAP_BIN = os.path.expanduser("~/src/dufomap/build/dufomap_run")
DUFOMAP_CONFIG = os.path.expanduser("~/src/dufomap/assets/config.toml")


def read_pcd_binary(path: Path) -> tuple[np.ndarray, tuple]:
    """Return (Nx3 xyz, (viewpoint_tx,ty,tz,qw,qx,qy,qz))."""
    with open(path, "rb") as f:
        raw = f.read()
    # Find DATA binary marker
    end_marker = b"DATA binary\n"
    end = raw.find(end_marker)
    if end < 0:
        raise RuntimeError(f"No binary DATA in {path}")
    header = raw[:end].decode("ascii", errors="ignore")
    data = raw[end + len(end_marker):]

    # Parse VIEWPOINT
    viewpoint = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    for line in header.splitlines():
        if line.startswith("VIEWPOINT"):
            parts = line.split()[1:]
            if len(parts) >= 7:
                viewpoint = tuple(float(p) for p in parts[:7])

    # Parse FIELDS and SIZE to get stride
    fields = []
    sizes: list[int] = []
    for line in header.splitlines():
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("SIZE"):
            sizes = [int(p) for p in line.split()[1:]]
    stride = sum(sizes) if sizes else 16
    n_fields = len(fields)

    pts = np.frombuffer(data, dtype=np.float32).reshape(-1, stride // 4)[:, :3].copy()
    return pts, viewpoint


def write_pcd_binary(path: Path, pts: np.ndarray, viewpoint: tuple) -> None:
    """Write PCD v0.7 binary with given VIEWPOINT header."""
    n = len(pts)
    vp_str = " ".join(f"{v:g}" for v in viewpoint)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        f"VIEWPOINT {vp_str}\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    pts32 = pts.astype(np.float32, copy=False)
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts32.tobytes())


def parse_poses(poses_path: Path) -> dict[str, tuple]:
    """Parse poses.txt → {patch_name: (tx,ty,tz,qw,qx,qy,qz)}."""
    out: dict[str, tuple] = {}
    with open(poses_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            out[parts[0]] = tuple(float(p) for p in parts[1:8])
    return out


def repack_patches(src_dir: Path, poses_path: Path, dst_dir: Path) -> int:
    """Repack each patch with VIEWPOINT = pose from poses.txt.

    Returns count of repacked patches.
    """
    poses = parse_poses(poses_path)
    dst_pcd = dst_dir / "pcd"
    dst_pcd.mkdir(parents=True, exist_ok=True)

    n = 0
    for patch_file in sorted(src_dir.glob("*.pcd")):
        name = patch_file.name
        if name not in poses:
            print(f"  skip {name}: no pose", file=sys.stderr)
            continue
        pts, _ = read_pcd_binary(patch_file)
        # DUFOMap expects VIEWPOINT in the PCD header. poses.txt is
        # (tx,ty,tz,qw,qx,qy,qz) which matches PCL/UFO's pose6 format.
        # DUFOMap also parses PCD stems as numeric ids, so keep the saved map
        # names unchanged and use numeric names only in this temporary input.
        write_pcd_binary(dst_pcd / f"{n:06d}.pcd", pts, poses[name])
        n += 1
    return n


def count_pcd_points(path: Path) -> int:
    """Read POINTS field from PCD header."""
    with open(path, "rb") as f:
        header = f.read(1024).decode("ascii", errors="ignore")
    for line in header.splitlines():
        if line.startswith("POINTS"):
            return int(line.split()[1])
    return -1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("map_name", help="e.g. corrected_20260406_224020")
    ap.add_argument("--maps-dir", default=os.path.expanduser("~/data/nova/maps"))
    ap.add_argument("--keep-tmp", action="store_true",
                    help="don't delete temp dir (for inspection)")
    args = ap.parse_args()

    map_dir = Path(args.maps_dir) / args.map_name
    patches_dir = map_dir / "patches"
    poses_path = map_dir / "poses.txt"
    orig_map = map_dir / "map.pcd"

    if not patches_dir.is_dir():
        print(f"ERROR: no patches dir at {patches_dir}", file=sys.stderr)
        return 1
    if not poses_path.is_file():
        print(f"ERROR: no poses.txt at {poses_path}", file=sys.stderr)
        return 1
    if not Path(DUFOMAP_BIN).is_file():
        print(f"ERROR: dufomap_run not built at {DUFOMAP_BIN}", file=sys.stderr)
        return 1

    tmp = Path(tempfile.mkdtemp(prefix="dufomap_test_"))
    try:
        print(f"[1/4] Repack patches into {tmp}/pcd/")
        t0 = time.time()
        n = repack_patches(patches_dir, poses_path, tmp)
        print(f"      repacked {n} patches in {time.time()-t0:.2f}s")
        if n <= 0:
            print("ERROR: no patches matched poses.txt", file=sys.stderr)
            return 4

        print(f"[2/4] Run dufomap_run {tmp} {DUFOMAP_CONFIG}")
        t0 = time.time()
        result = subprocess.run(
            [DUFOMAP_BIN, str(tmp), DUFOMAP_CONFIG],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=600,
        )
        elapsed = time.time() - t0
        if result.returncode != 0:
            print(f"ERROR: dufomap_run exit {result.returncode}", file=sys.stderr)
            print("stdout:", result.stdout[-2000:], file=sys.stderr)
            print("stderr:", result.stderr[-2000:], file=sys.stderr)
            return 2
        print(f"      finished in {elapsed:.1f}s")
        # Show last lines of dufomap output
        for line in result.stdout.splitlines()[-10:]:
            print(f"      > {line}")

        print(f"[3/4] Read output PCD")
        # Default output filename from config.toml is "output"
        clean_path = tmp / "output.pcd"
        if not clean_path.is_file():
            # Try other common names
            candidates = list(tmp.glob("*.pcd"))
            candidates = [c for c in candidates if c.parent == tmp]
            if not candidates:
                print(f"ERROR: no output PCD in {tmp}", file=sys.stderr)
                return 3
            clean_path = candidates[0]
            print(f"      using {clean_path.name}")

        orig_cnt = count_pcd_points(orig_map) if orig_map.is_file() else -1
        clean_cnt = count_pcd_points(clean_path)

        print(f"[4/4] Results")
        print(f"      original map.pcd: {orig_cnt} points")
        print(f"      DUFOMap clean:    {clean_cnt} points")
        if orig_cnt > 0:
            dropped = orig_cnt - clean_cnt
            print(f"      dropped:          {dropped} ({100*dropped/orig_cnt:.1f}%)")
        print(f"\nclean map saved at: {clean_path}")
        print(f"compare with: pcl_viewer {orig_map} {clean_path}")
        if args.keep_tmp:
            print(f"tmp dir kept: {tmp}")
            return 0
        # Copy clean map to a predictable location for inspection
        dst = orig_map.parent / "map_dufo_clean.pcd"
        shutil.copy(clean_path, dst)
        print(f"clean map copied to: {dst}")
    finally:
        if not args.keep_tmp:
            shutil.rmtree(tmp, ignore_errors=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
