# ruff: noqa: S101

from __future__ import annotations

import argparse

from drivers.real.camera.shm import ShmFrameReader, StreamKind, posix_shm_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", required=True)
    parser.add_argument("--depth", required=True)
    parser.add_argument("--info", required=True)
    parser.add_argument("--max-age-s", type=float, default=30.0)
    parser.add_argument("--unlink", action="store_true")
    args = parser.parse_args()
    paths = {
        "color": posix_shm_path(args.color),
        "depth": posix_shm_path(args.depth),
        "info": posix_shm_path(args.info),
    }
    try:
        color = ShmFrameReader(paths["color"], max_age_s=args.max_age_s).read_latest()
        depth = ShmFrameReader(paths["depth"], max_age_s=args.max_age_s).read_latest()
        info = ShmFrameReader(paths["info"], max_age_s=args.max_age_s).read_latest()
        assert color is not None and color.stream_kind is StreamKind.COLOR
        assert color.payload == bytes(range(12))
        assert depth is not None and depth.stream_kind is StreamKind.DEPTH
        assert depth.payload[:2] == (1000).to_bytes(2, "little")
        assert info is not None and info.stream_kind is StreamKind.INFO
        assert info.fx == 500.0 and info.fy == 501.0
        print(f"cross_language_ok color_seq={color.sequence} depth_seq={depth.sequence} info_seq={info.sequence}")
        return 0
    finally:
        if args.unlink:
            for path in paths.values():
                path.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
