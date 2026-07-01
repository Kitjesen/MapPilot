#!/usr/bin/env python3
"""Validate that runtime topic names are defined in topic_contract.yaml.

Usage:
    python tools/validate/validate_topics.py
    python tools/validate/validate_topics.py --verbose

Exit codes:
    0 - all checks passed
    1 - topic(s) found that are not defined in the contract

Validation scope:
    1. config/topic_contract.yaml as source of truth
    2. src/remote_monitoring/config/grpc_gateway.yaml values when present
    3. launch/**/*.launch.py remappings
    4. critical runtime adapter/factory source files
"""

from __future__ import annotations

import os
import re
import sys
from typing import Any

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", ".."))

CONTRACT_PATH = os.path.join(ROOT_DIR, "config", "topic_contract.yaml")
GATEWAY_PATH = os.path.join(
    ROOT_DIR,
    "src",
    "remote_monitoring",
    "config",
    "grpc_gateway.yaml",
)
LAUNCH_DIR = os.path.join(ROOT_DIR, "launch")
CONFIG_DIR = os.path.join(ROOT_DIR, "config")
SOURCE_SCAN_PATHS = (
    os.path.join(ROOT_DIR, "sim", "engine", "bridge"),
    os.path.join(ROOT_DIR, "sim", "scripts"),
    os.path.join(ROOT_DIR, "src", "adapters", "ros2", "rerun_bridge.py"),
    os.path.join(ROOT_DIR, "src", "slam", "native_factories.py"),
    os.path.join(ROOT_DIR, "src", "nav", "local", "legacy_ros"),
    os.path.join(ROOT_DIR, "sim", "scripts", "gazebo_frontier_exploration_smoke.py"),
    os.path.join(ROOT_DIR, "sim", "planning"),
    os.path.join(ROOT_DIR, "tests", "integration", "test_semantic_planner_live.py"),
    os.path.join(ROOT_DIR, "tests", "integration", "test_topic_hz.py"),
)
SOURCE_SCAN_EXTENSIONS = (".py", ".launch.py", ".service", ".sh")
CONFIG_SCAN_EXTENSIONS = (".yaml", ".yml")
CONFIG_SCAN_EXCLUDES = {"topic_contract.yaml"}

VERBOSE = "--verbose" in sys.argv
RUNTIME_TOPIC_VALUE_RE = re.compile(
    r"/(?:nav|slam|lidar|imu|camera|exploration)/[A-Za-z0-9_/\-]+"
)
RUNTIME_TOPIC_LITERAL_RE = re.compile(
    r"['\"](/(?:nav|slam|lidar|imu|camera|exploration)/[A-Za-z0-9_/\-]+)['\"]"
)


def info(msg: str) -> None:
    if VERBOSE:
        print(f"  [INFO] {msg}")


def load_contract(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_contract_topics(data: dict[str, Any]) -> set[str]:
    """Extract runtime topic names from topic_contract.yaml."""

    topics: set[str] = set()

    def _walk(obj: Any) -> None:
        if isinstance(obj, dict):
            for key, value in obj.items():
                if key == "tf":
                    continue
                if isinstance(key, str) and RUNTIME_TOPIC_VALUE_RE.fullmatch(key):
                    topics.add(key)
                _walk(value)
        elif isinstance(obj, list):
            for item in obj:
                _walk(item)
        elif isinstance(obj, str) and RUNTIME_TOPIC_VALUE_RE.fullmatch(obj):
            topics.add(obj)

    _walk(data)
    return topics


def load_required_nav_topics(data: dict[str, Any], contract_topics: set[str]) -> tuple[set[str], list[str]]:
    """Return P0 runtime topics that must be referenced by product paths."""

    required = {
        str(topic)
        for topic in data.get("required_runtime_topics")
        or data.get("required_nav_topics")
        or ()
        if isinstance(topic, str)
    }
    if not required:
        try:
            root_src = os.path.join(ROOT_DIR, "src")
            for path in (ROOT_DIR, root_src):
                if path not in sys.path:
                    sys.path.insert(0, path)
            from runtime.runtime_interface import CORE_REQUIRED_TOPICS

            required = set(CORE_REQUIRED_TOPICS)
        except Exception:
            required = set()

    errors: list[str] = []
    for topic in sorted(required):
        if not RUNTIME_TOPIC_VALUE_RE.fullmatch(topic):
            errors.append(f"  ERROR: required_runtime_topics entry is not a runtime topic: {topic}")
        elif topic not in contract_topics:
            errors.append(f"  ERROR: required_runtime_topics entry {topic} is not in contract")
    return required, errors


def validate_tf_contract(data: dict[str, Any]) -> list[str]:
    """Validate the minimal map->odom->body topology declared in the contract."""

    errors: list[str] = []
    tf = data.get("tf") or {}
    expected_frames = {
        "map_frame": "map",
        "odom_frame": "odom",
        "body_frame": "body",
    }
    for key, expected in expected_frames.items():
        if tf.get(key) != expected:
            errors.append(f"  ERROR: tf.{key} must be {expected!r}, got {tf.get(key)!r}")

    links = tf.get("links") or {}
    expected_links = {
        "map_to_odom": ("map", "odom"),
        "odom_to_body": ("odom", "body"),
    }
    for key, (parent, child) in expected_links.items():
        link = links.get(key) or {}
        if link.get("parent") != parent or link.get("child") != child:
            errors.append(
                f"  ERROR: tf.links.{key} must be parent={parent!r}, child={child!r}"
            )
        if link.get("required") is not True:
            errors.append(f"  ERROR: tf.links.{key}.required must be true")
    return errors


def validate_gazebo_bridge_contract(contract_topics: set[str]) -> tuple[list[str], set[str]]:
    """Validate generated Gazebo bridge LingTu topics against the contract."""

    try:
        root_src = os.path.join(ROOT_DIR, "src")
        for path in (ROOT_DIR, root_src):
            if path not in sys.path:
                sys.path.insert(0, path)
        from sim.engine.bridge.gazebo_bridge import GazeboBridgeConfig
    except Exception as exc:
        return [f"  ERROR: cannot import GazeboBridgeConfig: {exc}"], set()

    errors: list[str] = []
    used: set[str] = set()
    cfg = GazeboBridgeConfig()
    for name, topic in sorted(cfg.required_lingtu_topics().items()):
        if RUNTIME_TOPIC_VALUE_RE.fullmatch(topic):
            used.add(topic)
            if topic not in contract_topics:
                errors.append(f"  ERROR: Gazebo bridge topic {name}={topic} not in contract")
        elif not topic.startswith("/lingtu/gazebo/"):
            errors.append(f"  ERROR: Gazebo bridge topic {name}={topic} has unexpected namespace")

    for name, topic in sorted(cfg.raw_ros_topics().items()):
        if topic.startswith(("/nav/", "/slam/")):
            errors.append(
                f"  ERROR: raw Gazebo topic {name} must not publish directly into "
                f"runtime namespaces: {topic}"
            )
    return errors, used


def _format_is_declared(format_name: str, declared_formats: set[str]) -> bool:
    return (
        format_name in declared_formats
        or format_name == "service"
        or format_name == "application/json"
        or "/msg/" in format_name
    )


def _ros_types_for_formats(
    formats: list[Any],
    data_formats: dict[str, Any],
) -> list[str]:
    resolved: list[str] = []
    for item in formats:
        format_name = str(item)
        spec = data_formats.get(format_name)
        ros_type = str((spec or {}).get("ros_type") or format_name)
        if ros_type not in resolved:
            resolved.append(ros_type)
    return resolved


def validate_topic_format_contract(data: dict[str, Any]) -> list[str]:
    """Validate that contract endpoints also declare payload formats."""

    errors: list[str] = []
    data_formats = data.get("data_formats") or {}
    declared_formats = set(data_formats)
    topic_formats = data.get("topic_formats") or {}
    topic_ros_types = data.get("topic_ros_types") or {}

    if not topic_formats:
        return ["  ERROR: topic_formats must declare topic -> payload format mappings"]
    if not topic_ros_types:
        return ["  ERROR: topic_ros_types must declare topic -> ROS payload mappings"]

    for format_name, spec in data_formats.items():
        topic = (spec or {}).get("topic")
        if isinstance(topic, str) and topic.startswith("/"):
            formats = tuple(topic_formats.get(topic) or ())
            if format_name not in formats:
                errors.append(
                    f"  ERROR: data_formats.{format_name}.topic {topic!r} "
                    "is missing the matching topic_formats entry"
                )

    required_topics: set[str] = set()

    def _add_topic(value: Any) -> None:
        if isinstance(value, str) and value.startswith("/") and not value.startswith("artifact:"):
            required_topics.add(value)

    for spec in (data.get("algorithm_interfaces") or {}).values():
        for topic in tuple((spec or {}).get("inputs") or ()) + tuple(
            (spec or {}).get("outputs") or ()
        ):
            _add_topic(topic)

    for spec in (data.get("data_sources") or {}).values():
        for field in (
            "source_outputs",
            "normalized_outputs",
            "algorithm_entry_outputs",
            "algorithm_context_outputs",
        ):
            for topic in tuple((spec or {}).get(field) or ()):
                _add_topic(topic)
        _add_topic((spec or {}).get("command_sink"))

    for group_name in ("adapter_aliases", "adapter_relays"):
        for aliases in (data.get(group_name) or {}).values():
            for alias in aliases or ():
                _add_topic((alias or {}).get("source"))
                _add_topic((alias or {}).get("target"))

    for topic in sorted(required_topics):
        if topic not in topic_formats:
            errors.append(f"  ERROR: {topic} has no topic_formats payload declaration")

    for topic, formats in sorted(topic_formats.items()):
        if not isinstance(formats, list) or not formats:
            errors.append(f"  ERROR: topic_formats.{topic} must be a non-empty list")
            continue
        for format_name in formats:
            if not _format_is_declared(str(format_name), declared_formats):
                errors.append(
                    f"  ERROR: topic_formats.{topic} references unknown format {format_name!r}"
                )

        expected_ros_types = _ros_types_for_formats(formats, data_formats)
        actual_ros_types = [str(item) for item in topic_ros_types.get(topic) or ()]
        if actual_ros_types != expected_ros_types:
            errors.append(
                f"  ERROR: topic_ros_types.{topic} must be {expected_ros_types}, "
                f"got {actual_ros_types or '<missing>'}"
            )

    for topic in sorted(set(topic_ros_types) - set(topic_formats)):
        errors.append(f"  ERROR: topic_ros_types.{topic} has no topic_formats entry")
    return errors


def extract_nav_topics_from_yaml(path: str) -> dict[str, list[str]]:
    """Extract runtime topic strings from a YAML file."""

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    found: dict[str, list[str]] = {}

    def _walk(obj: Any, prefix: str = "") -> None:
        if isinstance(obj, dict):
            for key, value in obj.items():
                _walk(value, f"{prefix}.{key}" if prefix else str(key))
        elif isinstance(obj, list):
            for index, item in enumerate(obj):
                _walk(item, f"{prefix}[{index}]")
        elif isinstance(obj, str) and RUNTIME_TOPIC_VALUE_RE.fullmatch(obj):
            found.setdefault(obj, []).append(prefix)

    _walk(data)
    return found


def _candidate_source_files(paths: tuple[str, ...]) -> list[str]:
    files: list[str] = []
    for root in paths:
        if os.path.isfile(root):
            files.append(root)
            continue
        if not os.path.isdir(root):
            continue
        for dirpath, _, filenames in os.walk(root):
            for fname in filenames:
                if fname.endswith(SOURCE_SCAN_EXTENSIONS):
                    files.append(os.path.join(dirpath, fname))
    return files


def extract_nav_topics_from_sources(paths: tuple[str, ...]) -> dict[str, list[tuple[str, int]]]:
    """Extract runtime topic string literals from runtime sources."""

    found: dict[str, list[tuple[str, int]]] = {}
    for fpath in _candidate_source_files(paths):
        rel_path = os.path.relpath(fpath, ROOT_DIR)
        with open(fpath, "r", encoding="utf-8", errors="ignore") as f:
            for line_no, line in enumerate(f, 1):
                for match in RUNTIME_TOPIC_LITERAL_RE.finditer(line):
                    found.setdefault(match.group(1), []).append((rel_path, line_no))
    return found


def extract_nav_topics_from_launch(directory: str) -> dict[str, list[tuple[str, int]]]:
    """Extract runtime topic strings from launch files."""

    return extract_nav_topics_from_sources((directory,))


def extract_nav_topics_from_config(directory: str) -> dict[str, list[tuple[str, int]]]:
    """Extract runtime topic strings from non-contract YAML configuration files."""

    found: dict[str, list[tuple[str, int]]] = {}
    if not os.path.isdir(directory):
        return found
    for dirpath, _, filenames in os.walk(directory):
        for fname in filenames:
            if fname in CONFIG_SCAN_EXCLUDES or not fname.endswith(CONFIG_SCAN_EXTENSIONS):
                continue
            fpath = os.path.join(dirpath, fname)
            rel_path = os.path.relpath(fpath, ROOT_DIR)
            with open(fpath, "r", encoding="utf-8", errors="ignore") as f:
                for line_no, line in enumerate(f, 1):
                    for match in RUNTIME_TOPIC_LITERAL_RE.finditer(line):
                        found.setdefault(match.group(1), []).append((rel_path, line_no))
    return found


def _check_topic_locations(
    *,
    label: str,
    found: dict[str, list[Any]],
    contract_topics: set[str],
    used_topics: set[str],
    errors: list[str],
) -> None:
    print(f"\nChecking: {label}")
    for topic, locations in sorted(found.items()):
        used_topics.add(topic)
        if topic not in contract_topics:
            for location in locations:
                errors.append(f"  ERROR: {topic} ({location}) not in contract")
                print(f"  ERROR: {topic} ({location}) NOT in contract")
        else:
            info(f"  OK: {topic} ({len(locations)} references)")


def main() -> int:
    errors: list[str] = []
    warnings: list[str] = []

    if not os.path.exists(CONTRACT_PATH):
        print(f"ERROR: Contract file not found: {CONTRACT_PATH}")
        return 1

    contract = load_contract(CONTRACT_PATH)
    contract_topics = load_contract_topics(contract)
    print(f"Contract: {len(contract_topics)} runtime topics defined")
    for topic in sorted(contract_topics):
        info(f"  {topic}")

    required_topics, required_errors = load_required_nav_topics(contract, contract_topics)
    errors.extend(required_errors)
    if required_topics:
        info(f"  Required P0 runtime topics: {len(required_topics)}")

    used_topics: set[str] = set()

    print("\nChecking: tf frame topology")
    tf_errors = validate_tf_contract(contract)
    if tf_errors:
        errors.extend(tf_errors)
        for err in tf_errors:
            print(err)
    else:
        info("  OK: map -> odom -> body topology declared")

    print("\nChecking: topic payload formats")
    format_errors = validate_topic_format_contract(contract)
    if format_errors:
        errors.extend(format_errors)
        for err in format_errors:
            print(err)
    else:
        info("  OK: topic payload formats declared for contract endpoints")

    print("\nChecking: Gazebo bridge generated topics")
    gazebo_errors, gazebo_topics = validate_gazebo_bridge_contract(contract_topics)
    used_topics.update(gazebo_topics)
    if gazebo_errors:
        errors.extend(gazebo_errors)
        for err in gazebo_errors:
            print(err)
    else:
        info(f"  OK: {len(gazebo_topics)} Gazebo runtime topics are in contract")

    print("\nChecking: grpc_gateway.yaml")
    if os.path.exists(GATEWAY_PATH):
        gateway_topics = extract_nav_topics_from_yaml(GATEWAY_PATH)
        _check_topic_locations(
            label="grpc_gateway.yaml",
            found=gateway_topics,
            contract_topics=contract_topics,
            used_topics=used_topics,
            errors=errors,
        )
    else:
        print(f"  SKIP: {GATEWAY_PATH} not found")

    if os.path.isdir(LAUNCH_DIR):
        launch_topics = extract_nav_topics_from_launch(LAUNCH_DIR)
        _check_topic_locations(
            label="launch/**/*.launch.py",
            found=launch_topics,
            contract_topics=contract_topics,
            used_topics=used_topics,
            errors=errors,
        )
    else:
        print(f"\nChecking: launch/**/*.launch.py")
        print(f"  SKIP: {LAUNCH_DIR} not found")

    config_topics = extract_nav_topics_from_config(CONFIG_DIR)
    _check_topic_locations(
        label="config/**/*.yaml",
        found=config_topics,
        contract_topics=contract_topics,
        used_topics=used_topics,
        errors=errors,
    )

    source_topics = extract_nav_topics_from_sources(SOURCE_SCAN_PATHS)
    _check_topic_locations(
        label="critical runtime source files",
        found=source_topics,
        contract_topics=contract_topics,
        used_topics=used_topics,
        errors=errors,
    )

    unused_required = required_topics - used_topics
    unused_reserved = contract_topics - required_topics - used_topics
    if unused_required:
        print(f"\nWarnings: {len(unused_required)} required contract topics not referenced anywhere")
        for topic in sorted(unused_required):
            warnings.append(f"  WARNING: required {topic} defined in contract but never used")
            print(f"  WARNING: required {topic} (defined but unreferenced)")
    if unused_reserved:
        info(f"  Reserved/optional unreferenced topics: {len(unused_reserved)}")
        for topic in sorted(unused_reserved):
            info(f"  RESERVED: {topic} (defined but unreferenced)")

    print(f"\n{'=' * 50}")
    print(
        f"Summary: {len(contract_topics)} contract topics, "
        f"{len(used_topics)} used, {len(unused_required)} required unused, "
        f"{len(unused_reserved)} reserved unused"
    )

    if errors:
        print(f"\nFAILED: {len(errors)} topic(s) not in contract")
        return 1
    print("\nPASSED: All runtime topics are valid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
