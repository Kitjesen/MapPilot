"""Observable decode checks for the locally researched texture shapes."""
import struct
import subprocess
import sys
import runpy
from pathlib import Path

from PIL import Image
import pytest


def test_decode_ue5_dxt1_key_texture_with_inline_mip_chain(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    marker = b"PF_DXT1\0"
    data = struct.pack("<4I", 8, 8, 1, len(marker)) + marker + struct.pack("<2I", 0, 2)
    for side in (8, 4):
        red_block = struct.pack("<HHI", 0xF800, 0, 0)
        payload = red_block * (((side + 3) // 4) ** 2)
        data += struct.pack("<3Iq", 0x48, len(payload), len(payload), 0)
        data += payload + struct.pack("<3I", side, side, 1)
    (source / "W.uexp").write_bytes(data)
    output = tmp_path / "decoded"
    result = subprocess.run([
        sys.executable, str(Path(__file__).with_name("decode_inline_bc7.py")),
        str(source), str(output), "--format", "PF_DXT1",
        "--without-cooked-mip-flag", "--glob", "*.uexp",
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    with Image.open(output / "W.png") as image:
        assert image.size == (8, 8)
        assert image.mode == "RGBA"
        assert image.getpixel((3, 3)) == (255, 0, 0, 255)


def test_decoder_rejects_truncated_lower_mip_in_an_otherwise_valid_chain():
    decoder = runpy.run_path(str(Path(__file__).with_name("decode_inline_bc7.py")))
    marker = b"PF_DXT1\0"
    data = struct.pack("<4I", 8, 8, 1, len(marker)) + marker + struct.pack("<2I", 0, 2)
    for side in (8, 4):
        payload = struct.pack("<HHI", 0xF800, 0, 0) * (((side + 3) // 4) ** 2)
        data += struct.pack("<3Iq", 0x48, len(payload), len(payload), 0)
        data += payload + struct.pack("<3I", side, side, 1)
    with pytest.raises(ValueError, match="supported inline"):
        decoder["inline_mip"](data[:-14], pixel_format="PF_DXT1", has_cooked_flag=False)


def test_original_cooked_bc7_shape_still_decodes_to_rgba():
    decoder = runpy.run_path(str(Path(__file__).with_name("decode_inline_bc7.py")))
    marker = b"PF_BC7\0"
    payload = b"\x40" + b"\0" * 15
    data = struct.pack("<4I", 4, 4, 1, len(marker)) + marker + struct.pack("<3I", 0, 1, 1)
    data += struct.pack("<3Iq", 0x48, 16, 16, 0) + payload + struct.pack("<3I", 4, 4, 1)
    width, height, _, pixels = decoder["inline_mip"](data)
    import io
    with Image.open(io.BytesIO(decoder["as_dds"](width, height, pixels))) as image:
        image.load()
        assert image.size == (4, 4)
        assert image.mode == "RGBA"
