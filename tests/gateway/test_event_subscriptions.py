from __future__ import annotations

import pytest

from gateway.gateway_module import GatewayModule
from gateway.services.subscriptions import setup_subscriptions


@pytest.mark.parametrize('stream,handler', [
    ('navigation_goal_status', '_on_navigation_goal_status'),
    ('exploration_run_event', '_on_exploration_run_event'),
    ('inspection_task_event', '_on_inspection_task_event'),
])
def test_lifecycle_events_are_delivered_individually_without_waiting_for_a_batch(stream, handler):
    gateway = GatewayModule()
    received = []
    setattr(gateway, handler, received.append)
    gateway._build_app = lambda: None
    setup_subscriptions(gateway)
    first, second = object(), object()
    getattr(gateway, stream)._deliver(first)
    assert received == [first]
    getattr(gateway, stream)._deliver(second)
    assert received == [first, second]


def test_one_reached_event_immediately_updates_public_task_status():
    from gateway.navigation.tasks import query_navigation_task_status
    from runtime.msgs.nav import NavigationGoalState, NavigationGoalStatus

    gateway = GatewayModule()
    gateway._build_app = lambda: None
    setup_subscriptions(gateway)
    gateway.navigation_goal_status._deliver(NavigationGoalStatus(
        boot_id='test-boot', sequence=1, task_id='test-task', request_id='test-request',
        state=int(NavigationGoalState.REACHED), reason='goal_reached',
    ))
    result = query_navigation_task_status(gateway, 'test-task')
    assert result['found']
    assert result['status']['state_name'] == 'SUCCESS'
    assert result['source'] == 'live_gateway_cache'
