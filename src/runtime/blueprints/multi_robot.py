"""Multi-robot Blueprint composition.

Each robot gets an independent navigation stack under a unique ``robot_N/``
namespace, merged into a single Blueprint so they can be built and started
together as one system.

Typical usage::

    from runtime.blueprint import autoconnect
    from runtime.blueprints.multi_robot import multi_robot_blueprint

    system = autoconnect(
        multi_robot_blueprint(
            ["stub", "stub"],
            profile="stub",
            enable_semantic=True,
            llm="mock",
            enable_gateway=False,
        ),
    ).build()
    system.start()

    nav_0 = system.get_module("robot_0/Navigation")
    nav_1 = system.get_module("robot_1/Navigation")
"""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.blueprints.profile_builder import blueprint_for_resolved_profile


def multi_robot_blueprint(
    robots: list[str],
    port_offset: int = 0,
    **shared_config,
) -> Blueprint:
    """Build a multi-robot system from per-robot namespaced stacks.

    Each robot in ``robots`` receives a LingTu navigation stack under the
    namespace ``robot_<i>``. Namespaces prevent module-name collisions when the
    same module class appears in every robot's stack.

    Args:
        robots:
            Robot preset or driver names. Common values include ``"stub"``,
            ``"thunder"``, ``"sim_mujoco"``, and ``"sim_ros2"``.
        port_offset:
            Offset added to the base gateway port 5050. Robot ``i`` receives
            gateway port ``5050 + i + port_offset``.
        **shared_config:
            Configuration forwarded to the selected profile builder for every
            robot. Pass ``profile="..."`` to choose the base profile; the
            default is ``"stub"``. Do not pass ``robot`` or ``namespace`` here;
            those are set per robot.

    Returns:
        A single :class:`~runtime.blueprint.Blueprint` with every robot's modules
        under distinct namespaces.
    """

    shared_config.pop("namespace", None)
    shared_config.pop("robot", None)
    profile = str(shared_config.pop("profile", "stub"))

    combined = Blueprint()

    for i, robot_name in enumerate(robots):
        config = dict(shared_config)
        config["gateway_port"] = 5050 + i + port_offset
        config["robot"] = robot_name
        config["namespace"] = f"robot_{i}"
        if i > 0:
            config["manage_external_services"] = False

        robot_bp = blueprint_for_resolved_profile(profile, config)
        combined.merge(robot_bp)

    return combined
