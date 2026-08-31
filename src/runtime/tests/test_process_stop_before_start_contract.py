from __future__ import annotations

from copy import deepcopy

import pytest

from runtime.graph import (
    ProcessSpec,
    RuntimeGraph,
    load_runtime_graph,
    resolve_stop_before_start,
    validate_runtime_graph,
)


def _process(
    name: str,
    order: int,
    *,
    lifecycle: str = "mode",
) -> ProcessSpec:
    return ProcessSpec(
        name=name,
        manager="systemd",
        target=f"{name}.service",
        order=order,
        timeout_s=10,
        lifecycle=lifecycle,
    )


def test_explicit_stop_before_start_is_independent_of_start_order() -> None:
    driver = _process("driver", 10)
    nav = _process("nav", 10)
    traversability = _process("traversability", 10)
    host = _process("host", 20)
    implementation = {
        "stop_before_start": [
            host.target,
            nav.target,
            traversability.target,
            driver.target,
        ]
    }

    assert resolve_stop_before_start(
        implementation,
        (driver, nav, traversability, host),
        ("retired.service",),
        owner="test env",
    ) == (
        host.target,
        nav.target,
        traversability.target,
        driver.target,
        "retired.service",
    )


def test_env_stop_before_start_filters_known_processes_not_selected_by_product() -> None:
    driver = _process("driver", 10)
    nav = _process("nav", 20)
    implementation = {
        "processes": {
            "driver": {"target": driver.target},
            "nav": {"target": nav.target},
            "camera": {"target": "camera.service"},
        },
        "stop_before_start": [
            nav.target,
            driver.target,
            "camera.service",
        ],
    }

    assert resolve_stop_before_start(
        implementation,
        (driver, nav),
        (),
        owner="test env",
    ) == (nav.target, driver.target)


def test_env_without_stop_before_start_fails_closed() -> None:
    driver = _process("driver", 10)
    nav = _process("nav", 20)
    persistent = _process("metrics", 30, lifecycle="persistent")

    with pytest.raises(ValueError, match="stop_before_start is required"):
        resolve_stop_before_start(
            {},
            (driver, nav, persistent),
            (),
            owner="test env",
        )


@pytest.mark.parametrize(
    "stop_before_start",
    (
        "nav.service",
        ["nav.service", "nav.service"],
        ["nav.service"],
        ["nav.service", "driver.service", "unknown.service"],
        ["nav.service", "driver.service", 7],
    ),
)
def test_invalid_stop_before_start_fails_closed(stop_before_start: object) -> None:
    processes = (_process("driver", 10), _process("nav", 10))

    with pytest.raises(ValueError, match="stop_before_start"):
        resolve_stop_before_start(
            {"stop_before_start": stop_before_start},
            processes,
            (),
            owner="test env",
        )


def test_runtime_graph_static_validation_rejects_incomplete_stop_before_start() -> None:
    graph = load_runtime_graph()
    envs = deepcopy(graph.envs)
    envs["real"]["stop_before_start"] = [envs["real"]["processes"]["host"]["target"]]
    broken = RuntimeGraph(
        root=graph.root,
        topics=graph.topics,
        products=graph.products,
        envs=envs,
    )

    issues = validate_runtime_graph(broken)

    assert any(
        issue.scope == "env:real" and issue.code == "env_stop_before_start_invalid"
        for issue in issues
    )
