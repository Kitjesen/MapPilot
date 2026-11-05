"""Report local Bluetooth controllers and BlueZ capabilities with read-only probes."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path
from typing import Any

CONTROLLERS_PATH = Path("/sys/class/bluetooth")


def _run(command: list[str]) -> tuple[subprocess.CompletedProcess[str] | None, str | None]:
    try:
        return subprocess.run(command, capture_output=True, text=True, timeout=5, check=False), None
    except FileNotFoundError:
        return None, "command_missing"
    except PermissionError:
        return None, "permission_denied"
    except subprocess.TimeoutExpired:
        return None, "command_timeout"
    except OSError:
        return None, "command_failed"


def collect() -> dict[str, Any]:
    issues = []
    try:
        names = sorted(path.name for path in CONTROLLERS_PATH.iterdir() if re.fullmatch(r"hci\d+", path.name))
        controller_scan = "ok"
    except OSError:
        names = []
        controller_scan = "unknown"
        issues.append({"probe": "controllers", "reason": "sysfs_unavailable"})
    if controller_scan == "ok" and not names:
        issues.append({"probe": "controllers", "reason": "no_controller"})

    result, error = _run(["systemctl", "is-active", "bluetooth"])
    service = "unknown"
    if result is not None:
        state = result.stdout.strip()
        if result.returncode == 0 and state == "active":
            service = "active"
        elif result.returncode == 3 and state in {"inactive", "failed"}:
            service = "inactive"
        else:
            error = "state_unconfirmed"
    if error:
        issues.append({"probe": "bluetooth_service", "reason": error})

    controllers: list[dict[str, Any]] = [
        {"name": name, "le": "unknown", "advertising": "unknown", "powered": "unknown"} for name in names
    ]
    if controllers:
        result, error = _run(["btmgmt", "info"])
        if result is not None and result.returncode != 0:
            detail = f"{result.stdout}\n{result.stderr}".lower()
            error = "permission_denied" if "permission" in detail or "not permitted" in detail else "command_failed"
        if error:
            issues.append({"probe": "controller_capabilities", "reason": error})
        elif result is not None:
            sections = dict(re.findall(r"^\s*(hci\d+):[^\n]*\n(.*?)(?=^\s*hci\d+:|\Z)", result.stdout, re.M | re.S))
            for controller in controllers:
                section = sections.get(controller["name"], "")
                settings = re.search(r"^[ \t]*supported settings:[ \t]*([^\r\n]+)", section, re.M)
                if settings:
                    supported = settings.group(1).split()
                    controller["le"] = "yes" if "le" in supported else "no"
                    controller["advertising"] = "yes" if "advertising" in supported else "no"
                else:
                    issues.append({"probe": controller["name"], "reason": "capabilities_unconfirmed"})
                current = re.search(r"^[ \t]*current settings:[ \t]*([^\r\n]*)", section, re.M)
                if current:
                    controller["powered"] = "powered" in current.group(1).split()
                else:
                    issues.append({"probe": controller["name"], "reason": "power_state_unconfirmed"})

    return {
        "controller_scan": controller_scan,
        "controllers": controllers,
        "bluetooth_service": service,
        "issues": issues,
    }


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()
    print(json.dumps(collect(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
