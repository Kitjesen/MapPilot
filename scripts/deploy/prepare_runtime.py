"""Prepare locked Python dependencies for an extracted Linux field release.

Default: print a read-only plan. --apply creates a NEW environment and verifies
imports; release activation remains owned by install_nav.sh and ProductControl.
"""

import argparse
import json
import os
import platform
import re
import shlex
import shutil
import subprocess
from pathlib import Path

EXTRAS = ("gateway", "llm", "agent", "vision")
IMPORT_CHECK = (
    "import fastapi,uvicorn,grpc,google.protobuf,cv2,agents; "
    "import lingtu.control,decision.tasks.sdk_agent,decision.context.compression; "
    "print('LingTu Host/Gateway/Agent imports verified')"
)


def prepare_plan(package_dir: Path, venv: Path | None = None, python: str = "3.10") -> dict:
    package = package_dir.resolve()
    for name in ("metadata.json", "pyproject.toml", "uv.lock", "src", "install_nav.sh"):
        if not (package / name).exists():
            raise ValueError(f"Not an extracted field release: missing {name}")
    metadata = json.loads((package / "metadata.json").read_text(encoding="utf-8"))
    if metadata.get("schema_version") != "lingtu.native-release.v1":
        raise ValueError("Unsupported release metadata schema")
    version = metadata.get("version", "")
    if not isinstance(version, str) or not re.fullmatch(r"v\d+\.\d+\.\d+(?:[+-][0-9A-Za-z.-]+)?", version):
        raise ValueError("Invalid release version")
    architecture = metadata.get("architecture")
    if architecture not in {"aarch64", "x86_64"}:
        raise ValueError("Unsupported field release architecture")
    destination = (venv or Path("/opt/lingtu/venvs") / version).resolve()
    if destination.exists():
        raise ValueError(f"Environment already exists; leave it untouched and select a new --venv path: {destination}")
    if destination == package or destination.is_relative_to(package):
        raise ValueError("The environment must live outside the extracted/replaced release directory")
    interpreter = destination / "bin/python"
    command = ["uv", "sync", "--locked", "--no-dev", "--no-editable", "--project", str(package), "--python", python]
    for extra in EXTRAS:
        command.extend(["--extra", extra])
    return {"version": version, "architecture": architecture, "package_dir": str(package),
            "environment": str(destination), "python": str(interpreter), "command": command,
            "extras": list(EXTRAS), "requires_network_or_prepared_uv_cache": True,
            "install_command": "sudo env " + shlex.quote(f"LINGTU_PYTHON={interpreter}") + " bash " + shlex.quote(str(package / "install_nav.sh")),
            "service_install_command": "sudo env " + shlex.quote(f"LINGTU_PYTHON={interpreter}") +
                " bash /opt/lingtu/current/scripts/deploy/thunder/install_services.sh field-cpp",
            "activates_product": False, "prepares_native_libraries": False}


def apply_plan(plan: dict) -> dict:
    architecture = {"arm64": "aarch64", "amd64": "x86_64"}.get(platform.machine().lower(), platform.machine().lower())
    if platform.system() != "Linux" or architecture != plan["architecture"]:
        raise ValueError("Apply must run on Linux matching the release architecture; this is not cross-compilation")
    if shutil.which("uv") is None:
        raise ValueError("uv is required on the target; install the approved uv tool before preparation")
    if Path(plan["environment"]).exists():
        raise ValueError("Environment now exists; refusing to modify it")
    env = {**os.environ, "UV_PROJECT_ENVIRONMENT": plan["environment"]}
    # An active shell venv/PYTHONPATH must not make the new runtime appear usable.
    env.pop("VIRTUAL_ENV", None)
    env.pop("PYTHONPATH", None)
    subprocess.run(plan["command"], env=env, check=True)
    subprocess.run([plan["python"], "-I", "-c", IMPORT_CHECK], env=env, check=True)
    return {**plan, "prepared": True, "imports_verified": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--venv", type=Path, help="New stable environment path outside the extracted release")
    parser.add_argument("--python", default="3.10", help="Python version/interpreter for uv; field default is 3.10")
    parser.add_argument("--apply", action="store_true", help="Install locked dependencies; does not activate the release")
    args = parser.parse_args()
    try:
        plan = prepare_plan(args.package_dir, args.venv, args.python)
        result = apply_plan(plan) if args.apply else {**plan, "prepared": False, "mode": "plan_only"}
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f"Runtime preparation failed: {exc}\nNo release was activated. A partially created environment may remain.\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
