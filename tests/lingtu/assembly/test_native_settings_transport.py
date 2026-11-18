"""Every navd setting default must reach the field through the RunPlan.

navd compiles config/system/settings/*.yaml into C++ defaults. A field whose
value only comes from that header needs a native rebuild after a YAML edit,
while a field the RunPlan sends as an environment variable takes effect from
the next compiled RunPlan.
"""

from __future__ import annotations

import re
from pathlib import Path

from lingtu.assembly.compile_plan import compile_run_plan

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "src/nav/cpp/endpoint/nav/app/config/config.hpp"
PARSE = ROOT / "src/nav/cpp/endpoint/nav/app/config/parse.cpp"


def _settings_fields() -> list[str]:
    return re.findall(r"\b([a-z_0-9]+)\{::lingtu::settings::", CONFIG.read_text(encoding="utf-8"))


def _environment_names(field: str, parse: str) -> set[str]:
    return set(re.findall(rf"\bcfg\.{field}\b[^;]*?\"(LINGTU_[A-Z0-9_]+)\"", parse))


def test_navd_setting_defaults_are_sent_by_the_run_plan() -> None:
    parse = PARSE.read_text(encoding="utf-8")
    emitted = set(compile_run_plan("nav", "real", robot="unitree/go2").native_process_environment)
    fields = _settings_fields()
    assert fields

    unread = [field for field in fields if not _environment_names(field, parse)]
    unsent = [
        field for field in fields
        if (names := _environment_names(field, parse)) and not names & emitted
    ]
    assert not unread, f"navd reads no environment variable for: {unread}"
    assert not unsent, f"the nav RunPlan does not send: {unsent}"
