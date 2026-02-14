"""
机器人参数加载器 — 供 launch 文件使用

用法:
    from _robot_config import robot_cfg

    # 读取参数
    max_speed = robot_cfg('speed', 'max_speed')           # -> 0.875
    vehicle_h = robot_cfg('geometry', 'vehicle_height')    # -> 0.5
    dog_host  = robot_cfg('driver', 'dog_host')            # -> "127.0.0.1"
"""

import os
import yaml

_config = None
_config_path = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'config', 'robot_config.yaml')
)


def _load():
    global _config
    if _config is None:
        with open(_config_path, 'r') as f:
            _config = yaml.safe_load(f)
    return _config


def robot_cfg(section, key, default=None):
    """获取 robot_config.yaml 中的参数值。

    Args:
        section: 一级 key (如 'speed', 'geometry', 'driver')
        key:     二级 key (如 'max_speed', 'vehicle_height')
        default: 找不到时的默认值
    Returns:
        参数值 (原始类型: int, float, str, bool)
    """
    cfg = _load()
    return cfg.get(section, {}).get(key, default)


def robot_cfg_str(section, key, default=''):
    """获取参数并转为字符串 — 供 DeclareLaunchArgument default_value 使用。"""
    return str(robot_cfg(section, key, default))
