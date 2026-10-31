"""Small, licensed surface palette for the existing FactoryPark geometry."""

from __future__ import annotations

import json
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SURFACES = ROOT / "sim/packages/worlds/factory_park_hf/visual/surfaces"
ASSETS = ("asphalt_02", "concrete_floor_worn_001", "box_profile_metal_sheet", "aerial_grass_rock")


def surface_for(source: str, semantic: str) -> tuple[str, float, tuple[float, float, float]] | None:
    """Do not put scan textures on paint marks, glass, rubber, or safety fixtures."""
    identity = f"{source} {semantic}".lower()
    if any(word in identity for word in ("marking", "paint", "glass", "rubber", "oil", "crack", "stripe", "safety")):
        return None
    if source == "asphalt" or semantic in {"road", "parking_area"}:
        return "asphalt_02", 400.0, (0.75, 0.78, 0.80)
    if any(word in identity for word in ("concrete", "curb", "foundation", "bund")):
        return "concrete_floor_worn_001", 300.0, (0.85, 0.84, 0.81)
    if any(
        word in identity
        for word in ("factory_cladding", "corrugated_panel", "warehouse_wall", "factory_wall", "roof_panel")
    ) or (source == "warehouse_cladding" and "building" in semantic):
        return "box_profile_metal_sheet", 300.0, (2.3, 2.65, 2.9)
    if semantic == "terrain" or source in {"grass", "soil", "gravel"}:
        return "aerial_grass_rock", 1200.0, (0.72, 0.76, 0.65)
    return None


def download_surfaces() -> None:
    """Fetch four specific CC0 assets, not a bulk mirror; cache complete files."""
    SURFACES.mkdir(parents=True, exist_ok=True)
    records = []
    for asset in ASSETS:
        req = urllib.request.Request(
            f"https://api.polyhaven.com/files/{asset}", headers={"User-Agent": "LingTuAssetLookdev/1.0"}
        )
        with urllib.request.urlopen(req, timeout=30) as response:
            files = json.load(response)
        for channel, api_key, fmt in (
            ("base", "Diffuse", "jpg"),
            ("rough", "Rough", "jpg"),
            ("normal", "nor_dx", "png"),
        ):
            item = files[api_key]["2k"][fmt]
            dest = SURFACES / f"{asset}_{channel}.{fmt}"
            if not dest.is_file() or dest.stat().st_size != item["size"]:
                request = urllib.request.Request(item["url"], headers={"User-Agent": "LingTuAssetLookdev/1.0"})
                with urllib.request.urlopen(request, timeout=90) as response:
                    data = response.read()
                if len(data) != item["size"]:
                    raise RuntimeError(f"Incomplete download: {asset}/{channel}")
                dest.write_bytes(data)
            records.append(
                {
                    "asset": asset,
                    "channel": channel,
                    "file": dest.name,
                    "bytes": item["size"],
                    "url": item["url"],
                    "source": f"https://polyhaven.com/a/{asset}",
                    "license": "CC0-1.0",
                }
            )
            print(f"READY {dest.name} ({item['size']} bytes)", flush=True)
    (SURFACES / "sources.json").write_text(
        json.dumps({"license_url": "https://polyhaven.com/license", "textures": records}, indent=2) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    download_surfaces()
