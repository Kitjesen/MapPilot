import os
from pathlib import Path
import subprocess
import tempfile
import unittest

def check_prebuilt_kernel(tmp_path: Path, prebuilt: bool):
    root = Path(__file__).resolve().parents[2]
    shim = tmp_path / "bin"
    shim.mkdir()
    cmake = shim / "cmake"
    cmake.write_text('#!/bin/sh\nprintf "%s\\0" "$@" >> "$BUILD_ARGS_LOG"\n')
    cmake.chmod(0o755)
    log = tmp_path / "args"
    env = dict(os.environ)
    env.pop("LINGTU_POSE_GRAPH_OPT_LIBRARY", None)
    env.update(
        PATH=f"{shim}:{env['PATH']}",
        BUILD_ARGS_LOG=str(log),
        LINGTU_SLAM_CORE_BUILD_DIR=str(tmp_path / "build"),
        LINGTU_SLAM_FASTLIO2="OFF",
        LINGTU_SLAM_BUILD_DDS_RUNTIME="OFF",
        LINGTU_SLAM_BUILD_TESTS="OFF",
    )
    library = str(tmp_path / "prebuilt kernels" / "libpose_graph_opt.a")
    if prebuilt:
        env["LINGTU_POSE_GRAPH_OPT_LIBRARY"] = library
    subprocess.run(["bash", str(root / "scripts/build/build_slam_core.sh")],
                   env=env, check=True, capture_output=True, text=True)
    args = log.read_bytes().decode().split("\0")
    supplied = [arg for arg in args if arg.startswith("-DLINGTU_POSE_GRAPH_OPT_LIBRARY=")]
    assert supplied == []  # Online SLAM must not configure the legacy Rust PGO.


@unittest.skipIf(os.name == "nt", "Native Linux Bash build entry")
class SlamBuildPrebuiltTest(unittest.TestCase):
    def test_optional_prebuilt_kernel(self):
        for prebuilt in (False, True):
            with self.subTest(prebuilt=prebuilt), tempfile.TemporaryDirectory() as directory:
                check_prebuilt_kernel(Path(directory), prebuilt)


if __name__ == "__main__":
    unittest.main()
