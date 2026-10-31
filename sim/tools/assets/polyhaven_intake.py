"""Download a selected CC0 Poly Haven glTF and its declared dependencies.

This is asset intake, not a claim of UE import or physics qualification.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import Request, urlopen

AGENT = "LingTu-AssetLibrary/0.1 (local asset intake)"


def fetch(url: str) -> bytes:
    with urlopen(Request(url, headers={"User-Agent": AGENT}), timeout=90) as response:
        return response.read()


def intake(asset_id: str, resolution: str, root: Path) -> dict:
    info = json.loads(fetch(f"https://api.polyhaven.com/info/{asset_id}"))
    files = json.loads(fetch(f"https://api.polyhaven.com/files/{asset_id}"))
    model = files["gltf"][resolution]["gltf"]
    folder = root / asset_id / resolution
    folder.mkdir(parents=True, exist_ok=True)
    model_name = Path(urlparse(model["url"]).path).name
    downloaded = []
    for name, item in [(model_name, model), *model.get("include", {}).items()]:
        target = (folder / name).resolve()
        if not target.is_relative_to(folder.resolve()):
            raise ValueError(f"Dependency escapes asset directory: {name}")
        if not target.exists() or target.stat().st_size != item["size"]:
            data = fetch(item["url"])
            if len(data) != item["size"]:
                raise ValueError(f"Incomplete download: {name}")
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        downloaded.append({"path": name, "url": item["url"], "bytes": target.stat().st_size})
    record = {
        "provider": "Poly Haven",
        "id": asset_id,
        "name": info["name"],
        "authors": info.get("authors", {}),
        "license": "CC0-1.0",
        "license_url": "https://polyhaven.com/license",
        "source_url": f"https://polyhaven.com/a/{asset_id}",
        "resolution": resolution,
        "model": model_name,
        "files": downloaded,
        "ue_imported": False,
        "physics_qualified": False,
    }
    (folder / "source.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return {
        "id": asset_id,
        "folder": str(folder),
        "files": len(downloaded),
        "bytes": sum(x["bytes"] for x in downloaded),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", action="append", required=True)
    parser.add_argument("--resolution", default="2k", choices=["1k", "2k", "4k"])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for identity in args.id:
        print(json.dumps(intake(identity, args.resolution, args.output)), flush=True)
