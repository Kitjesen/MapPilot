"""Bounded local menu-package study. Never launches or changes either game."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path

REPO = Path("D:/inovxio/brain/lingtu")
OUT = REPO / "build/game-ui-research-20261017/frontend"
UNREALPAK = Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealPak.exe")
NODE = Path("C:/Program Files/nodejs/node.exe")
REA = Path("D:/inovxio/tools/rea-cli/node_modules/rea-agents/scripts/rea.mjs")
GAMES = Path("D:/software/steam/steamapps/common")


def run(label: str, argv: list[str], timeout: int = 90) -> dict:
    with (OUT / f"{label}.log").open("wb") as stream:
        result = subprocess.run(argv, stdout=stream, stderr=subprocess.STDOUT,
                                timeout=timeout, creationflags=subprocess.CREATE_NO_WINDOW)
    record = {"label": label, "argv": argv, "exit_code": result.returncode}
    print(json.dumps({"label": label, "exit_code": result.returncode}), flush=True)
    return record


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=False)
    commands = []
    hogwarts = GAMES / "Hogwarts Legacy/Phoenix/Content/Paks/pakchunk0-WindowsNoEditor.utoc"
    filters = [
        "*/UI/Menus/UI_BP_FrontEnd_Menu.uasset",
        "*/UI/Menus/UI_BP_FrontEndDetails.uasset",
        "*/Levels/UI/BP_FrontEndCamera.uasset",
        "*/Levels/UI/Frontend.umap",
        "*/Data/DA_FrontEndLevels.uasset",
        "*/Data/DT_FrontEndCinematics.uasset",
        "*/Cinematics/Misc/FrontEnd/CIV_Frontend_Ambient.uasset",
    ]
    for index, pattern in enumerate(filters):
        result = run(f"hogwarts-{index}", [str(UNREALPAK), "IoStore",
            f"-FindAndExtract={hogwarts}", f"-PackageFilter={pattern}",
            f"-OutPath={OUT / 'hogwarts'}"])
        commands.append(result)
        if result["exit_code"]:
            raise RuntimeError(f"Targeted extraction failed: {pattern}")

    palworld = GAMES / "Palworld/Pal/Content/Paks/Pal-Windows.pak"
    patterns = [
        "*/Pal/Blueprint/UI/UserInterface/Title/WBP_TitleMenu.*",
        "*/Pal/Blueprint/UI/UserInterface/Title/WBP_Title_MenuButton.*",
        "*/Pal/Blueprint/UI/UserInterface/Title/WBP_Title_MenuBG.*",
        "*/Pal/Blueprint/UI/Title/WBP_TItle.*",
    ]
    for index, pattern in enumerate(patterns):
        result = run(f"palworld-{index}", [str(UNREALPAK), str(palworld),
            "-Extract", str(OUT / 'palworld'), f"-Filter={pattern}"])
        commands.append(result)
        if result["exit_code"]:
            raise RuntimeError(f"Targeted extraction failed: {pattern}")

    inventory = []
    for game in ("hogwarts", "palworld"):
        for path in sorted((OUT / game).rglob("*")):
            if not path.is_file():
                continue
            data = path.read_bytes()
            # Printable bytes are names/reference evidence, not decoded Blueprint logic.
            strings = [{"offset": match.start(), "text": match.group().decode("ascii")}
                       for match in re.finditer(rb"[\x20-\x7e]{5,}", data)]
            relative = path.relative_to(OUT).as_posix()
            inventory.append({"path": relative, "size": len(data), "strings": strings})
    (OUT / "package-strings.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
    selected = OUT / "hogwarts/Phoenix/Content/UI/Menus/UI_BP_FrontEnd_Menu.uasset"
    commands.append(run("rea-package", [str(NODE), str(REA), "inspect-artifact", str(selected), "--json"]))
    commands.append(run("rea-inventory", [str(NODE), str(REA), "inspect-artifact", str(OUT / "hogwarts"), "--json"]))
    (OUT / "commands.json").write_text(json.dumps(commands, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(OUT), "files": len(inventory),
                      "bytes": sum(item["size"] for item in inventory)}))


if __name__ == "__main__":
    main()
