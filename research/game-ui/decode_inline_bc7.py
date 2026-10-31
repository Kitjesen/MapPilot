"""Inspect observed inline BC7/DXT1 mips in the local game UI references.

This deliberately supports only the observed local texture shape, not arbitrary
Unreal packages. Outputs are local research references, not product assets.
Platform/mip field ordering follows the upstream CUE4Parse texture readers;
the pixel decoder is the existing Pillow DDS implementation.
"""
from __future__ import annotations

import argparse
import io
import json
import struct
from pathlib import Path

from PIL import Image


def inline_mip(data: bytes, *, pixel_format: str = "PF_BC7",
               has_cooked_flag: bool = True) -> tuple[int, int, int, bytes]:
    block_bytes = {"PF_BC7": 16, "PF_DXT1": 8}[pixel_format]
    marker = pixel_format.encode("ascii") + b"\x00"
    candidates = []
    start = 0
    while (index := data.find(marker, start)) >= 0:
        start = index + len(marker)
        if index < 16 or start + 32 > len(data):
            continue
        width, height, slices, string_length = struct.unpack_from("<4I", data, index - 16)
        if string_length != len(marker) or slices != 1 or not (0 < width <= 4096 and 0 < height <= 4096):
            continue
        first_mip, mip_count = struct.unpack_from("<2I", data, start)
        if first_mip != 0 or not 1 <= mip_count <= 16:
            continue
        cursor = start + 8
        first_payload = None
        for mip in range(mip_count):
            if has_cooked_flag:
                if cursor + 4 > len(data) or struct.unpack_from("<I", data, cursor)[0] != 1:
                    break
                cursor += 4
            if cursor + 20 > len(data):
                break
            flags, elements, size = struct.unpack_from("<3I", data, cursor)
            mip_width, mip_height = max(1, width >> mip), max(1, height >> mip)
            expected = ((mip_width + 3) // 4) * ((mip_height + 3) // 4) * block_bytes
            payload_offset = cursor + 20  # Three uint32 fields and int64 bulk offset.
            payload_end = payload_offset + size
            if flags != 0x48 or elements != expected or size != expected or payload_end + 12 > len(data):
                break
            if struct.unpack_from("<3I", data, payload_end) != (mip_width, mip_height, 1):
                break
            if mip == 0:
                first_payload = (width, height, payload_offset, data[payload_offset:payload_end])
            cursor = payload_end + 12
        else:
            candidates.append(first_payload)
    if len(candidates) != 1:
        raise ValueError(f"expected one supported inline {pixel_format} mip chain, found {len(candidates)}")
    return candidates[0]


def as_dds(width: int, height: int, pixels: bytes, *, pixel_format: str = "PF_BC7") -> bytes:
    header = struct.pack("<7I11I", 124, 0x81007, height, width, len(pixels), 0, 1, *([0] * 11))
    dds_pixel_format = struct.pack("<II4s5I", 32, 4, b"DX10", 0, 0, 0, 0, 0)
    caps = struct.pack("<5I", 0x1000, 0, 0, 0, 0)
    dxgi_format = {"PF_BC7": 98, "PF_DXT1": 71}[pixel_format]
    dx10 = struct.pack("<5I", dxgi_format, 3, 0, 1, 0)  # TEXTURE2D, one image.
    return b"DDS " + header + dds_pixel_format + caps + dx10 + pixels


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--format", choices=("PF_BC7", "PF_DXT1"), default="PF_BC7")
    parser.add_argument("--without-cooked-mip-flag", action="store_true")
    parser.add_argument("--glob", default="UI_T_*.uasset")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    entries = []
    mip_options = {"pixel_format": args.format, "has_cooked_flag": not args.without_cooked_mip_flag}
    for source in sorted(args.source.glob(args.glob)):
        data = source.read_bytes()
        width, height, offset, pixels = inline_mip(data, **mip_options)
        # Truncated pixel payloads must not be accepted as viewable textures.
        try:
            inline_mip(data[:offset + len(pixels) - 1], **mip_options)
        except ValueError:
            pass
        else:
            raise AssertionError("truncated mip was accepted")
        destination = args.output / f"{source.stem}.png"
        with Image.open(io.BytesIO(as_dds(width, height, pixels, pixel_format=args.format))) as image:
            image.load()
            if image.size != (width, height) or image.mode != "RGBA":
                raise ValueError("DDS decoder dimensions/mode differ from the source mip")
            image.save(destination)
            entries.append({
                "source": str(source.resolve()), "source_bytes": len(data),
                "width": width, "height": height, "format": args.format,
                "inline_offset": offset, "pixel_bytes": len(pixels),
                "output": str(destination.resolve()), "rgba_extrema": image.getextrema(),
                "truncated_payload_rejected": True,
            })
    if not entries:
        raise ValueError("no source textures were selected")
    report = {"scope": "local_ui_asset_reference_only", "production_use": False, "entries": entries}
    (args.output / "decode-report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"decoded_textures": len(entries), "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
