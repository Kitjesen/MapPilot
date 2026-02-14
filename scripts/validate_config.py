#!/usr/bin/env python3
"""validate_config.py — 校验 robot_config.yaml 的结构和值范围。

用法:
    python scripts/validate_config.py            # 从项目根目录执行
    python scripts/validate_config.py --strict   # 将 WARNING 也视为失败

Exit code:
    0  全部通过
    1  存在 ERROR (--strict 模式下 WARNING 也计入)
"""
from __future__ import annotations

import argparse
import os
import sys

import yaml

# ── Schema 定义 ─────────────────────────────────────────────────
# 每个 section/key 的期望类型和可选值范围 (min, max)。
SCHEMA: dict[str, dict[str, dict]] = {
    "robot": {
        "id":               {"type": str,   "required": True},
        "hw_id":            {"type": str,   "required": True},
        "firmware_version": {"type": str,   "required": True},
    },
    "geometry": {
        "vehicle_height":   {"type": (int, float), "required": True,  "min": 0.01, "max": 5.0},
        "vehicle_width":    {"type": (int, float), "required": True,  "min": 0.01, "max": 5.0},
        "vehicle_length":   {"type": (int, float), "required": True,  "min": 0.01, "max": 10.0},
        "sensor_offset_x":  {"type": (int, float), "required": False, "min": -5.0, "max": 5.0},
        "sensor_offset_y":  {"type": (int, float), "required": False, "min": -5.0, "max": 5.0},
    },
    "speed": {
        "max_linear":       {"type": (int, float), "required": True,  "min": 0.0, "max": 10.0},
        "max_angular":      {"type": (int, float), "required": True,  "min": 0.0, "max": 10.0},
        "max_speed":        {"type": (int, float), "required": True,  "min": 0.0, "max": 10.0},
        "autonomy_speed":   {"type": (int, float), "required": True,  "min": 0.0, "max": 10.0},
    },
    "safety": {
        "obstacle_height_thre": {"type": (int, float), "required": True, "min": 0.0, "max": 2.0},
        "ground_height_thre":   {"type": (int, float), "required": True, "min": 0.0, "max": 2.0},
        "stop_distance":        {"type": (int, float), "required": True, "min": 0.1, "max": 10.0},
        "slow_distance":        {"type": (int, float), "required": True, "min": 0.1, "max": 20.0},
        "deadman_timeout_ms":   {"type": (int, float), "required": True, "min": 50,  "max": 5000},
        "cmd_vel_timeout_ms":   {"type": (int, float), "required": True, "min": 50,  "max": 5000},
        "tilt_limit_deg":       {"type": (int, float), "required": False, "min": 1,  "max": 90},
    },
    "driver": {
        "dog_host":             {"type": str,          "required": True},
        "dog_port":             {"type": int,          "required": True, "min": 1, "max": 65535},
        "control_rate":         {"type": (int, float), "required": True, "min": 1, "max": 1000},
        "auto_enable":          {"type": bool,         "required": False},
        "auto_standup":         {"type": bool,         "required": False},
        "reconnect_interval":   {"type": (int, float), "required": False, "min": 0.1, "max": 60},
    },
    "lidar": {
        "frame_id":     {"type": str,          "required": True},
        "publish_freq": {"type": (int, float), "required": True, "min": 1, "max": 100},
    },
    "grpc": {
        "port": {"type": int, "required": True, "min": 1, "max": 65535},
    },
    "control": {
        "yaw_rate_gain":      {"type": (int, float), "required": True, "min": 0, "max": 100},
        "stop_yaw_rate_gain": {"type": (int, float), "required": True, "min": 0, "max": 100},
        "max_yaw_rate":       {"type": (int, float), "required": True, "min": 0, "max": 360},
        "max_accel":          {"type": (int, float), "required": True, "min": 0, "max": 20},
    },
}


def _find_config() -> str:
    """定位 config/robot_config.yaml，向上搜索到项目根目录。"""
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "config", "robot_config.yaml"),
        os.path.join(os.getcwd(), "config", "robot_config.yaml"),
    ]
    for c in candidates:
        full = os.path.normpath(c)
        if os.path.isfile(full):
            return full
    print("ERROR  Cannot find config/robot_config.yaml")
    sys.exit(1)


def validate(cfg: dict, strict: bool = False) -> tuple[int, int]:
    """校验 cfg 并返回 (errors, warnings) 计数。"""
    errors = 0
    warnings = 0

    # 1. 检查未知 section
    for section_name in cfg:
        if section_name not in SCHEMA:
            print(f"  WARNING  unknown section '{section_name}'")
            warnings += 1

    # 2. 按 schema 逐项校验
    for section_name, keys in SCHEMA.items():
        section = cfg.get(section_name)
        if section is None:
            print(f"  ERROR    missing required section '{section_name}'")
            errors += 1
            continue
        if not isinstance(section, dict):
            print(f"  ERROR    section '{section_name}' should be a mapping, got {type(section).__name__}")
            errors += 1
            continue

        for key, spec in keys.items():
            value = section.get(key)

            # required check
            if value is None:
                if spec.get("required"):
                    print(f"  ERROR    {section_name}.{key} is required but missing")
                    errors += 1
                continue

            # type check
            expected_type = spec["type"]
            if not isinstance(value, expected_type):
                # int is acceptable where float is expected
                if expected_type == float and isinstance(value, int):
                    pass
                else:
                    print(f"  ERROR    {section_name}.{key}: expected {expected_type}, got {type(value).__name__} ({value!r})")
                    errors += 1
                    continue

            # range check
            if "min" in spec and isinstance(value, (int, float)):
                if value < spec["min"]:
                    print(f"  ERROR    {section_name}.{key} = {value} < min({spec['min']})")
                    errors += 1
                elif value == spec["min"]:
                    pass  # boundary OK

            if "max" in spec and isinstance(value, (int, float)):
                if value > spec["max"]:
                    print(f"  ERROR    {section_name}.{key} = {value} > max({spec['max']})")
                    errors += 1

        # 检查 section 内未知 key
        for key in section:
            if key not in keys:
                print(f"  WARNING  {section_name}.{key} is not in schema (typo?)")
                warnings += 1

    # 3. 交叉校验
    safety = cfg.get("safety", {})
    stop_d = safety.get("stop_distance", 0)
    slow_d = safety.get("slow_distance", 0)
    if stop_d >= slow_d:
        print(f"  ERROR    safety.stop_distance ({stop_d}) should be < safety.slow_distance ({slow_d})")
        errors += 1

    speed = cfg.get("speed", {})
    if speed.get("max_speed", 0) > speed.get("max_linear", float("inf")):
        print(f"  WARNING  speed.max_speed ({speed['max_speed']}) > speed.max_linear ({speed['max_linear']})")
        warnings += 1

    if speed.get("autonomy_speed", 0) > speed.get("max_speed", float("inf")):
        print(f"  WARNING  speed.autonomy_speed ({speed['autonomy_speed']}) > speed.max_speed ({speed['max_speed']})")
        warnings += 1

    return errors, warnings


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate robot_config.yaml")
    parser.add_argument("--strict", action="store_true", help="Treat warnings as errors")
    parser.add_argument("config", nargs="?", help="Path to robot_config.yaml (auto-detected if omitted)")
    args = parser.parse_args()

    config_path = args.config or _find_config()
    print(f"Validating {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    errors, warnings = validate(cfg, args.strict)

    print(f"\n  Result: {errors} error(s), {warnings} warning(s)")
    if errors > 0 or (args.strict and warnings > 0):
        sys.exit(1)
    print("  OK  robot_config.yaml is valid")


if __name__ == "__main__":
    main()
