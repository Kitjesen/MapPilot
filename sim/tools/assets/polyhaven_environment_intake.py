"""Download bounded Poly Haven environment assets with provenance."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import Request, urlopen

AGENT = "LingTu-AssetLibrary/0.1 (environment intake)"


def get(url: str) -> bytes:
    with urlopen(Request(url, headers={"User-Agent": AGENT}), timeout=120) as response:
        return response.read()


def save(url: str, size: int, target: Path) -> None:
    if target.exists() and target.stat().st_size == size:
        return
    data = get(url)
    if len(data) != size:
        raise ValueError(f"Incomplete download: {target.name}")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)


def download_model(asset_id: str, output: Path, resolution: str = "2k") -> dict:
    info = json.loads(get(f"https://api.polyhaven.com/info/{asset_id}"))
    files = json.loads(get(f"https://api.polyhaven.com/files/{asset_id}"))
    item = files["gltf"][resolution]["gltf"]
    folder = output / asset_id / resolution
    downloaded = []
    model_name = Path(urlparse(item["url"]).path).name
    for name, spec in [(model_name, item), *item.get("include", {}).items()]:
        target = folder / name
        save(spec["url"], spec["size"], target)
        downloaded.append({"path": name, "url": spec["url"], "bytes": spec["size"]})
    record = {
        "provider": "Poly Haven",
        "asset_id": asset_id,
        "name": info["name"],
        "authors": info.get("authors", {}),
        "license": "CC0-1.0",
        "license_url": "https://polyhaven.com/license",
        "source_url": f"https://polyhaven.com/a/{asset_id}",
        "resolution": resolution,
        "model": model_name,
        "files": downloaded,
    }
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "source.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return record


def download_hdri(asset_id: str, output: Path, resolution: str = "4k") -> dict:
    info = json.loads(get(f"https://api.polyhaven.com/info/{asset_id}"))
    files = json.loads(get(f"https://api.polyhaven.com/files/{asset_id}"))
    item = files["hdri"][resolution]["exr"]
    folder = output / "hdris" / asset_id / resolution
    target = folder / Path(urlparse(item["url"]).path).name
    save(item["url"], item["size"], target)
    record = {
        "provider": "Poly Haven",
        "asset_id": asset_id,
        "name": info["name"],
        "authors": info.get("authors", {}),
        "license": "CC0-1.0",
        "license_url": "https://polyhaven.com/license",
        "source_url": f"https://polyhaven.com/a/{asset_id}",
        "resolution": resolution,
        "file": target.name,
        "bytes": item["size"],
    }
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "source.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return record


def download_texture(asset_id: str, output: Path, resolution: str = "2k") -> dict:
    info = json.loads(get(f"https://api.polyhaven.com/info/{asset_id}"))
    files = json.loads(get(f"https://api.polyhaven.com/files/{asset_id}"))
    folder = output / "textures" / asset_id / resolution
    maps = {}
    for map_name, api_name in (("diff", "Diffuse"), ("nor_gl", "nor_gl"), ("rough", "Rough")):
        spec = files[api_name][resolution]["jpg"]
        target = folder / Path(urlparse(spec["url"]).path).name
        save(spec["url"], spec["size"], target)
        maps[map_name] = {"file": target.name, "url": spec["url"], "bytes": spec["size"]}
    record = {
        "provider": "Poly Haven",
        "asset_id": asset_id,
        "name": info["name"],
        "authors": info.get("authors", {}),
        "license": "CC0-1.0",
        "license_url": "https://polyhaven.com/license",
        "source_url": f"https://polyhaven.com/a/{asset_id}",
        "resolution": resolution,
        "maps": maps,
    }
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "source.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return record


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", action="append", default=[])
    parser.add_argument("--hdri", action="append", default=[])
    parser.add_argument("--texture", action="append", default=[])
    args = parser.parse_args()
    result = {
        "models": [download_model(item, args.output) for item in args.model],
        "hdris": [download_hdri(item, args.output) for item in args.hdri],
        "textures": [download_texture(item, args.output) for item in args.texture],
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
