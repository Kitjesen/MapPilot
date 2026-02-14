#!/usr/bin/env python3
"""
validate_topics.py — 校验所有 /nav/* 话题名是否在 topic_contract.yaml 中定义

用法:
    python scripts/validate_topics.py              # 从仓库根目录运行
    python scripts/validate_topics.py --verbose    # 显示详细信息

退出码:
    0 — 全部通过
    1 — 存在未在 contract 中定义的话题 (ERROR)

校验范围:
    1. config/topic_contract.yaml      — source of truth
    2. src/remote_monitoring/config/grpc_gateway.yaml — 所有 /nav/* 值
    3. launch/**/*.launch.py           — remapping 目标中的 /nav/*
"""

import os
import re
import sys
import yaml

# ─── 路径定义 ───
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))

CONTRACT_PATH = os.path.join(ROOT_DIR, 'config', 'topic_contract.yaml')
GATEWAY_PATH = os.path.join(ROOT_DIR, 'src', 'remote_monitoring', 'config', 'grpc_gateway.yaml')
LAUNCH_DIR = os.path.join(ROOT_DIR, 'launch')

VERBOSE = '--verbose' in sys.argv


def info(msg):
    if VERBOSE:
        print(f'  [INFO] {msg}')


def load_contract_topics(path):
    """从 topic_contract.yaml 提取所有 /nav/* 话题名 (忽略 tf 等非话题字段)。"""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    topics = set()
    for section, entries in data.items():
        if section == 'tf':
            # tf 是坐标系名，不是话题
            continue
        if isinstance(entries, dict):
            for key, value in entries.items():
                if isinstance(value, str) and value.startswith('/nav/'):
                    topics.add(value)
    return topics


def extract_nav_topics_from_yaml(path):
    """从 YAML 文件中提取所有值为 /nav/* 的字符串。"""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    found = {}  # topic -> list of param keys

    def _walk(obj, prefix=''):
        if isinstance(obj, dict):
            for k, v in obj.items():
                _walk(v, f'{prefix}.{k}' if prefix else k)
        elif isinstance(obj, list):
            for i, item in enumerate(obj):
                _walk(item, f'{prefix}[{i}]')
        elif isinstance(obj, str) and obj.startswith('/nav/'):
            found.setdefault(obj, []).append(prefix)

    _walk(data)
    return found


def extract_nav_topics_from_launch(directory):
    """从 launch 目录中所有 .launch.py 文件提取 remapping 目标中的 /nav/*。"""
    pattern = re.compile(r'["\'](/nav/[a-z_]+)["\']')
    found = {}  # topic -> list of (file, line_no)

    for dirpath, _, filenames in os.walk(directory):
        for fname in filenames:
            if not fname.endswith('.launch.py'):
                continue
            fpath = os.path.join(dirpath, fname)
            rel_path = os.path.relpath(fpath, ROOT_DIR)
            with open(fpath, 'r', encoding='utf-8') as f:
                for line_no, line in enumerate(f, 1):
                    for match in pattern.finditer(line):
                        topic = match.group(1)
                        found.setdefault(topic, []).append((rel_path, line_no))

    return found


def main():
    errors = []
    warnings = []

    # ─── 1. Load contract ───
    if not os.path.exists(CONTRACT_PATH):
        print(f'ERROR: Contract file not found: {CONTRACT_PATH}')
        return 1

    contract_topics = load_contract_topics(CONTRACT_PATH)
    print(f'Contract: {len(contract_topics)} standard /nav/* topics defined')
    for t in sorted(contract_topics):
        info(f'  {t}')

    used_topics = set()

    # ─── 2. Validate grpc_gateway.yaml ───
    print(f'\nChecking: grpc_gateway.yaml')
    if os.path.exists(GATEWAY_PATH):
        gateway_topics = extract_nav_topics_from_yaml(GATEWAY_PATH)
        for topic, params in sorted(gateway_topics.items()):
            used_topics.add(topic)
            if topic not in contract_topics:
                for p in params:
                    errors.append(f'  ERROR: {topic} (param: {p}) not in contract')
                    print(f'  ERROR: {topic} (param: {p}) NOT in contract')
            else:
                info(f'  OK: {topic}')
    else:
        print(f'  SKIP: {GATEWAY_PATH} not found')

    # ─── 3. Validate launch files ───
    print(f'\nChecking: launch/**/*.launch.py')
    if os.path.isdir(LAUNCH_DIR):
        launch_topics = extract_nav_topics_from_launch(LAUNCH_DIR)
        for topic, locations in sorted(launch_topics.items()):
            used_topics.add(topic)
            if topic not in contract_topics:
                for loc_file, loc_line in locations:
                    errors.append(f'  ERROR: {topic} ({loc_file}:{loc_line}) not in contract')
                    print(f'  ERROR: {topic} ({loc_file}:{loc_line}) NOT in contract')
            else:
                info(f'  OK: {topic} ({len(locations)} references)')
    else:
        print(f'  SKIP: {LAUNCH_DIR} not found')

    # ─── 4. Unused contract topics (warning only) ───
    unused = contract_topics - used_topics
    if unused:
        print(f'\nWarnings: {len(unused)} contract topics not referenced anywhere')
        for t in sorted(unused):
            warnings.append(f'  WARNING: {t} defined in contract but never used')
            print(f'  WARNING: {t} (defined but unreferenced)')

    # ─── Summary ───
    print(f'\n{"=" * 50}')
    print(f'Summary: {len(contract_topics)} contract topics, '
          f'{len(used_topics)} used, {len(unused)} unused')

    if errors:
        print(f'\nFAILED: {len(errors)} topic(s) not in contract')
        return 1
    else:
        print('\nPASSED: All /nav/* topics are valid')
        return 0


if __name__ == '__main__':
    sys.exit(main())
