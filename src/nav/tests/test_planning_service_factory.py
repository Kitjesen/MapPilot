from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from runtime.msgs.numpy_compat import np
from nav.services.plan.contracts import PlannerService
from nav.services.plan.factory import create_planner_service
from nav.services.plan.mapless.direct import MaplessDirectPlannerService
from nav.services.plan.global_planner.service import GlobalPlanner

SRC_DIR = Path(__file__).resolve().parents[2]


def test_create_direct_planner_service_uses_mapless_service() -> None:
    svc = create_planner_service(planner_name="direct", plan_safety_policy="off")

    assert isinstance(svc, MaplessDirectPlannerService)
    assert isinstance(svc, PlannerService)
    assert svc.has_map is False
    assert svc.map_artifact_gate["required"] is False

    svc.setup()
    path, _ = svc.plan(np.asarray([0.0, 0.0, 0.0]), np.asarray([1.0, 1.0, 0.0]))

    assert len(path) == 2
    assert svc.last_plan_report["selected_planner"] == "direct"
    assert svc.last_plan_report["planner_diagnostics"]["map_required"] is False
    assert svc.backend_status()["backend"] == "direct"


def test_create_direct_planner_service_canonicalizes_before_detection() -> None:
    svc = create_planner_service(planner_name=" DIRECT ", plan_safety_policy="off")

    assert isinstance(svc, MaplessDirectPlannerService)
    assert svc.planner_name == "direct"


def test_direct_service_factory_import_does_not_load_global_planner() -> None:
    script = (
        "import sys\n"
        "from nav.services.plan.factory import create_planner_service\n"
        "svc = create_planner_service(planner_name='direct')\n"
        "svc.setup()\n"
        "forbidden = {'nav.services.plan.global_planner.service', 'global_planning', 'nav.services.safety.plan_safety'}\n"
        "print(any(name == f or name.startswith(f + '.') for name in sys.modules for f in forbidden))\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR)
    result = subprocess.run(
        [sys.executable, "-c", script],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )

    assert result.stdout.strip() == "False"


def test_create_map_backed_planner_service_still_uses_global_service() -> None:
    svc = create_planner_service(planner_name="pct")

    assert svc.__class__.__name__ == "GlobalPlanner"
    assert svc.planner_name == "pct"


def test_create_map_backed_planner_service_defaults_to_octoplanner3d() -> None:
    svc = create_planner_service()

    assert svc.__class__.__name__ == "GlobalPlanner"
    assert svc.planner_name == "octoplanner3d"
    assert svc.backend_status()["fallback_backend"] == ""


def test_create_map_backed_planner_service_passes_octoplanner3d_constraints() -> None:
    svc = create_planner_service(
        octoplanner3d_constraints={
            "robot_radius": 0.6,
            "require_ground_support": True,
            "preblocked_costmap_weight": 4.0,
        }
    )

    status = svc.backend_status()
    assert status["octoplanner3d_constraints"]["robot_radius"] == 0.6
    assert status["octoplanner3d_constraints"]["require_ground_support"] is True
    assert status["octoplanner3d_constraints"]["preblocked_costmap_weight"] == 4.0


def test_create_map_backed_planner_service_can_disable_saved_map_gate() -> None:
    svc = create_planner_service(
        planner_name="octoplanner3d",
        map_artifact_gate_required=False,
    )

    gate = svc._validate_map_artifact_gate()

    assert gate["required"] is False
    assert gate["ok"] is True
    assert gate["reason"] == "disabled_by_runtime_profile"


def test_default_map_backed_service_setup_does_not_load_ros2_modules() -> None:
    script = (
        "import sys\n"
        "from nav.services.plan.factory import create_planner_service\n"
        "svc = create_planner_service()\n"
        "svc.setup()\n"
        "roots = (\n"
        "    'rclpy', 'ament_index_python', 'geometry_msgs', 'nav_msgs',\n"
        "    'sensor_msgs', 'std_msgs', 'visualization_msgs', 'octomap_msgs',\n"
        ")\n"
        "loaded = [\n"
        "    name for name in sys.modules\n"
        "    if any(name == root or name.startswith(root + '.') for root in roots)\n"
        "]\n"
        "print(svc.planner_name)\n"
        "print(bool(loaded))\n"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC_DIR)
    result = subprocess.run(
        [sys.executable, "-c", script],
        env=env,
        capture_output=True,
        text=True,
        check=True,
    )

    assert result.stdout.splitlines() == ["octoplanner3d", "False"]


def test_create_map_backed_planner_service_canonicalizes_octplanner_alias() -> None:
    svc = create_planner_service(planner_name="octplanner")

    assert svc.__class__.__name__ == "GlobalPlanner"
    assert svc.planner_name == "octoplanner3d"


def test_create_map_backed_planner_service_preserves_pct() -> None:
    svc = create_planner_service(planner_name="pct")

    assert svc.__class__.__name__ == "GlobalPlanner"
    assert svc.planner_name == "pct"


def test_global_planner_rejects_unknown_planner() -> None:
    svc = GlobalPlanner(planner_name="bad_contract")

    with pytest.raises(ValueError, match="Unknown planner"):
        svc.setup()
