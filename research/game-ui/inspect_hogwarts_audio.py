"""Read three selected cooked UI packages without executing or changing the game."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path


def main() -> None:
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=False)
    tool = Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealPak.exe")
    container = Path(
        "D:/software/steam/steamapps/common/Hogwarts Legacy/Phoenix/Content/Paks/"
        "pakchunk0-WindowsNoEditor.utoc"
    )
    report: dict = {
        "scope": "static cooked-package names/references; no recovered runtime behavior",
        "game_executed": False,
        "container": str(container),
        "commands": [],
        "packages": [],
    }
    for index, relative in enumerate((
        "UI/Menus/Settings/UI_BP_Settings_Audio.uasset",
        "UI/HUD/Accessibility/UI_DT_AccessibilityAudioCueIconTable.uasset",
        "UI/HUD/AudioVisualizer/BP_HUD_Audio.uasset",
    )):
        argv = [str(tool), "IoStore", f"-FindAndExtract={container}",
                f"-PackageFilter=*/{relative}", f"-OutPath={output / 'cooked'}"]
        with (output / f"extract-{index}.log").open("wb") as log:
            result = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=90, creationflags=subprocess.CREATE_NO_WINDOW)
        report["commands"].append({"argv": argv, "exit_code": result.returncode})
        if result.returncode:
            raise RuntimeError(f"Targeted extraction failed: {relative}")
    for path in sorted((output / "cooked").rglob("*.uasset")):
        data = path.read_bytes()
        names = [{"offset": match.start(), "text": match.group().decode("ascii")}
                 for match in re.finditer(rb"[\x20-\x7e]{5,}", data)]
        report["packages"].append({
            "path": path.relative_to(output).as_posix(), "bytes": len(data),
            "references": [item for item in names if re.search(
                r"Sound|Audio|Volume|Slider|Mix|Mute|Speaker|Headphone|Accessibility|"
                r"Subtit|Focus|Apply|Cancel|Restore|Default|AkEvent|Wwise", item["text"], re.I)],
        })
    (output / "audio-reference-evidence.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "packages": len(report["packages"]),
                      "bytes": sum(item["bytes"] for item in report["packages"])}))


if __name__ == "__main__":
    main()
