"""Ordinary instructions create one queryable Agent run without duplicate dispatch."""

import asyncio
import json
import threading

import pytest
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.mcp_server import MCPServerModule
from gateway.schemas import AgentTaskCancelRequest, InstructionRequest
from tests.gateway.test_gateway_commands import _endpoint, _payload


class Agent:
    def __init__(self):
        self.starts = []
        self.cancels = []
        self.run_id = ""

    def run_agent_task(self, instruction):
        self.starts.append(instruction)
        self.run_id = f"agent-{len(self.starts)}"
        return json.dumps({"run_id": self.run_id, "state": "running"})

    def get_agent_status(self):
        return json.dumps({"run_id": self.run_id, "state": "running", "motion_success": False})

    def cancel_agent_task(self, run_id):
        if run_id != self.run_id:
            return json.dumps({"cancel_requested": False, "reason": "not_current"})
        self.cancels.append(run_id)
        return json.dumps({"cancel_requested": True, "stop_confirmed": False})


@pytest.fixture
def gateway():
    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"AgentPlannerModule": Agent()}
    yield gateway
    gateway.stop()


def test_http_instruction_returns_run_id_and_replay_does_not_start_twice(gateway):
    endpoint = _endpoint(gateway, "/api/v1/instruction")
    body = InstructionRequest(text="帮我找一瓶水", request_id="water-1", client_id="test")
    first = asyncio.run(endpoint(body))
    replay = asyncio.run(endpoint(body))
    assert first["run_id"] == replay["run_id"] == "agent-1"
    assert first["executor"] == "agent" and first["execution_confirmed"] is False
    assert gateway._all_modules["AgentPlannerModule"].starts == ["帮我找一瓶水"]
    assert gateway.instruction.msg_count == 0
    query = _endpoint(gateway, "/api/v1/agent/runs/{run_id}")
    view = asyncio.run(query(first["run_id"]))
    assert view["found"] and view["motion_success"] is False


def test_http_serialization_preserves_agent_run_and_cancel_receipts(gateway):
    with TestClient(gateway._app) as client:
        response = client.post("/api/v1/instruction", json={
            "text": "帮我找一瓶水", "request_id": "wire-water", "client_id": "test",
        })
        assert response.status_code == 200
        receipt = response.json()
        assert receipt["run_id"] == "agent-1" and receipt["executor"] == "agent"
        assert receipt["execution_confirmed"] is False
        status = client.get(f"/api/v1/agent/runs/{receipt['run_id']}").json()
        assert status["found"] and status["motion_success"] is False
        cancel = client.post(f"/api/v1/agent/runs/{receipt['run_id']}/cancel", json={
            "request_id": "wire-cancel", "client_id": "test",
        })
        assert cancel.status_code == 200
        assert cancel.json()["cancel_requested"] is True
        assert cancel.json()["stop_confirmed"] is False


def test_old_http_run_cancel_cannot_cancel_the_new_task(gateway):
    submit = _endpoint(gateway, "/api/v1/instruction")
    first = asyncio.run(submit(InstructionRequest(text="帮我找一瓶水", client_id="test")))
    second = asyncio.run(submit(InstructionRequest(text="跟随我", client_id="test")))
    cancel = _endpoint(gateway, "/api/v1/agent/runs/{run_id}/cancel")
    old = _payload(asyncio.run(cancel(first["run_id"], AgentTaskCancelRequest(client_id="test"))))
    assert old["cancel_requested"] is False
    current = _payload(asyncio.run(cancel(second["run_id"], AgentTaskCancelRequest(client_id="test"))))
    assert current["cancel_requested"] is True and current["stop_confirmed"] is False
    assert gateway._all_modules["AgentPlannerModule"].cancels == [second["run_id"]]


@pytest.mark.parametrize("operation", ["start", "instruction_cancel", "explicit_cancel"])
def test_agent_command_and_status_waits_do_not_block_gateway_event_loop(gateway, monkeypatch, operation):
    from decision.modules.agent_planner import AgentPlannerModule

    agent = gateway._all_modules["AgentPlannerModule"]
    agent.run_agent_task("find water")
    agent.classify_instruction_control = AgentPlannerModule.classify_instruction_control
    entered = threading.Event()
    release = threading.Event()
    status_entered = threading.Event()
    run_lock = threading.RLock()
    owner_threads = []
    original_cancel = agent.cancel_agent_task
    original_start = agent.run_agent_task
    original_status = agent.get_agent_status

    def blocking_action(action, argument):
        with run_lock:
            owner_threads.append(threading.get_ident())
            entered.set()
            release.wait(timeout=1.0)
            return action(argument)

    def locked_status():
        owner_threads.append(threading.get_ident())
        status_entered.set()
        with run_lock:
            return original_status()

    monkeypatch.setattr(agent, "run_agent_task", lambda text: blocking_action(original_start, text))
    monkeypatch.setattr(agent, "cancel_agent_task", lambda run_id: blocking_action(original_cancel, run_id))
    monkeypatch.setattr(agent, "get_agent_status", locked_status)
    agent.handle_instruction_control = lambda text, run_id: {
        **json.loads(agent.cancel_agent_task(run_id)), "run_id": run_id, "accepted": True,
    }
    submit = _endpoint(gateway, "/api/v1/instruction")
    cancel = _endpoint(gateway, "/api/v1/agent/runs/{run_id}/cancel")
    query = _endpoint(gateway, "/api/v1/agent/runs/{run_id}")

    async def run_requests():
        loop_thread = threading.get_ident()
        if operation == "explicit_cancel":
            request = cancel("agent-1", AgentTaskCancelRequest(request_id="waiting-cancel"))
        else:
            request = submit(InstructionRequest(
                text="find chair" if operation == "start" else "停下",
                run_id="agent-1", request_id="waiting-instruction",
            ))
        command_task = asyncio.create_task(request)
        query_task = None
        try:
            assert await asyncio.to_thread(entered.wait, 2.0)
            query_task = asyncio.create_task(query("agent-1"))
            assert await asyncio.to_thread(status_entered.wait, 2.0)
            assert not command_task.done()
            assert not query_task.done()
            assert all(thread != loop_thread for thread in owner_threads)
        finally:
            release.set()
            response = await command_task
            if query_task is not None:
                await query_task
        return response

    response = asyncio.run(run_requests())
    assert response["accepted"] is True
    assert response["execution_confirmed"] is False
    if operation == "start":
        assert response["run_id"] == "agent-2"
        assert agent.starts == ["find water", "find chair"]
    else:
        assert response["run_id"] == "agent-1"
        assert response["stop_confirmed"] is False
        assert agent.cancels == ["agent-1"]


def test_mcp_instruction_uses_same_agent_owner_even_when_semantic_is_loaded(gateway):
    from decision.modules.semantic_planner import SemanticPlannerModule

    mcp = MCPServerModule(host="127.0.0.1")
    modules = {**gateway._all_modules, "GatewayModule": gateway, "MCPServerModule": mcp,
               "SemanticPlannerModule": SemanticPlannerModule()}
    mcp.on_system_modules(modules)
    assert mcp._tool_registry["send_instruction"].__self__ is mcp
    receipt = json.loads(mcp._tool_registry["send_instruction"]("帮我找一瓶水"))
    assert receipt["run_id"] == "agent-1" and receipt["executor"] == "agent"
    assert gateway.instruction.msg_count == 0


def test_gateway_keeps_explicit_rejection_with_current_run(gateway, monkeypatch):
    agent = gateway._all_modules['AgentPlannerModule']
    monkeypatch.setattr(agent, 'run_agent_task', lambda text: {
        'accepted': False, 'run_id': 'agent-current', 'state': 'running', 'reason': 'task_active',
    })
    endpoint = _endpoint(gateway, '/api/v1/instruction')
    receipt = _payload(asyncio.run(endpoint(InstructionRequest(text='follow me'))))
    assert receipt['accepted'] is False and receipt['ok'] is False
    assert receipt['run_id'] == 'agent-current' and receipt['reason'] == 'task_active'


@pytest.mark.parametrize('transport', ['http', 'mcp'])
def test_current_task_controls_reach_owner_while_motion_is_blocked(gateway, monkeypatch, transport):
    from fastapi.responses import JSONResponse

    from decision.modules.agent_planner import AgentPlannerModule
    from gateway.services.control_commands import ControlCommandService

    agent = gateway._all_modules['AgentPlannerModule']
    agent.run_agent_task('find water')
    agent.classify_instruction_control = AgentPlannerModule.classify_instruction_control
    controls = []
    def handle(text, expected_run_id=''):
        controls.append((text, expected_run_id))
        operation = agent.classify_instruction_control(text)
        accepted = expected_run_id == agent.run_id and operation not in {'pause', 'resume'}
        return {'accepted': accepted, 'operation': operation, 'run_id': agent.run_id,
                'state': 'running', 'task_status': {'run_id': agent.run_id, 'instruction': 'find water'},
                'reason': '' if accepted else 'not_current'}
    agent.handle_instruction_control = handle
    guarded = []
    def blocked(self, command, body):
        guarded.append(body.text)
        return JSONResponse(status_code=409, content={'ok': False, 'accepted': False, 'error': 'motion_blocked'})
    monkeypatch.setattr(ControlCommandService, 'motion_safety_rejection', blocked)
    mcp = MCPServerModule(host='127.0.0.1')
    mcp.on_system_modules({'AgentPlannerModule': agent, 'GatewayModule': gateway})
    def send(text, run_id='agent-1'):
        if transport == 'http':
            return _payload(asyncio.run(_endpoint(gateway, '/api/v1/instruction')(
                InstructionRequest(text=text, run_id=run_id))))
        return json.loads(mcp.send_instruction(text, run_id))
    for text, operation in [('现在在干什么', 'query'), ('继续跟着他', 'keep'), ('不用跟了', 'cancel')]:
        receipt = send(text)
        assert receipt['accepted'] and receipt['operation'] == operation
        assert receipt['run_id'] == 'agent-1'
        assert receipt['status'] != 'submitted'
        assert receipt['task_status']['instruction'] == 'find water'
    assert send('停下', 'old-run')['accepted'] is False
    assert send('find chair')['error'] == 'motion_blocked'
    assert guarded == ['find chair']
    assert agent.starts == ['find water']
    assert len(controls) == 4


@pytest.mark.parametrize('transport', ['http', 'mcp', 'mcp_without_gateway'])
@pytest.mark.parametrize('active', [False, True])
def test_control_only_preserves_owner_controls_and_never_starts_a_task(gateway, transport, active):
    from decision.modules.agent_planner import AgentPlannerModule, AgentRun

    owner = AgentPlannerModule()
    if active:
        owner._active_run = AgentRun('find water')
    gateway._all_modules = {'AgentPlannerModule': owner}
    mcp = MCPServerModule(host='127.0.0.1')
    modules = {'AgentPlannerModule': owner}
    if transport != 'mcp_without_gateway':
        modules['GatewayModule'] = gateway
    mcp.on_system_modules(modules)
    original = owner._active_run
    with TestClient(gateway._app) as client:
        def send(text):
            if transport == 'http':
                return client.post('/api/v1/instruction', json={
                    'text': text, 'control_only': True,
                    'run_id': original.run_id if original else None,
                }).json()
            return json.loads(mcp.send_instruction(
                text, original.run_id if original else '', control_only=True))
        query = send('当前状态')
        assert query['accepted'] and query['operation'] == 'query'
        assert query['task_status']['found'] is active
        refused = send('find chair')
        assert refused['accepted'] is False and refused['ok'] is False
        assert refused['reason'] == 'motion_start_blocked' and refused['operation'] == 'start'
        assert owner._active_run is original
        assert gateway.instruction.msg_count == 0 and mcp.instruction.msg_count == 0
        paused = send('暂停一下')
        assert paused['accepted'] is False and paused['reason'] == 'unsupported_control'
        if active:
            stopped = send('停下')
            assert stopped['accepted'] and stopped['operation'] == 'cancel'
            assert stopped['task_status']['found'] is True and original.cancelled
        else:
            assert send('停下')['reason'] == 'no_current_task'
    owner.stop()


@pytest.mark.parametrize('transport', ['http', 'mcp_without_gateway'])
def test_control_only_without_agent_does_not_publish_semantic_instruction(gateway, transport):
    gateway._all_modules = {}
    if transport == 'http':
        with TestClient(gateway._app) as client:
            receipt = client.post('/api/v1/instruction', json={
                'text': 'find chair', 'control_only': True,
            }).json()
        assert gateway.instruction.msg_count == 0
    else:
        mcp = MCPServerModule(host='127.0.0.1')
        mcp.on_system_modules({})
        receipt = json.loads(mcp.send_instruction('find chair', control_only=True))
        assert mcp.instruction.msg_count == 0
    assert receipt['accepted'] is False and receipt['reason'] == 'motion_start_blocked'
