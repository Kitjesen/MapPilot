"""Public SDK fields projected from the existing system diagnostic owners."""


def _pick(value, names):
    return {name: value[name] for name in names if name in value}


def session(value):
    return _pick(value, ("mode", "active_map", "product", "product_session_id"))


def state(value):
    navigation = value.get("navigation", {})
    return {
        **_pick(value, ("ts",)),
        "session": session(value.get("session", {})),
        "teleop": _pick(value.get("teleop", {}), ("active", "clients")),
        "safety": _pick(value.get("safety", {}), ("level", "ok", "stop_active", "motion_allowed")),
        "localization": _pick(
            value.get("localization", {}), ("state", "ready", "active_map", "pose_fresh", "reasons"),
        ),
        "navigation": {
            "task": _pick(navigation.get("task", {}), ("state", "task_id", "reason")),
            "goal_admission": _pick(navigation.get("goal_admission", {}), ("state", "reason")),
            "control": _pick(navigation.get("control", {}), ("authority", "resume_required", "reason")),
            "motion": _pick(
                navigation.get("motion", {}), ("permission", "observation", "stop_confirmation", "reason"),
            ),
        },
    }


def health(value):
    return {
        **_pick(value, ("status", "modules_ok", "modules_fail", "has_odom", "slam_hz", "map_points")),
        "modules": {name: status for name, status in value.get("modules", {}).items() if isinstance(status, str)},
        "sensors": {
            name: _pick(sensor, ("status", "ready", "available", "hz", "cloud_hz", "odom_hz"))
            for name, sensor in value.get("sensors", {}).items()
        },
    }


def readiness(value):
    return _pick(value, ("status", "ready", "data_ready", "motion_ready", "non_motion_safe", "reasons", "advisories"))


def capabilities(value):
    features = value.get("features", {})
    return {
        "features": {
            name: features[name]
            for name in (
                "state", "health", "mapping", "map_management", "localization", "navigation",
                "exploration", "teleop", "camera_snapshot", "whep", "scene_graph", "locations",
            )
            if isinstance(features.get(name), bool)
        },
        "products": {
            name: _pick(product, ("available",))
            for name, product in value.get("runtime_products", {}).get("products", {}).items()
        },
    }


def product_status(value):
    result = _pick(value, ("available", "current"))
    if result.get("current") is not None:
        result["current"] = _pick(result["current"], ("ok", "status", "product", "product_session_id"))
    return result
