"""Draft plans use the existing durable message, search and share paths without execution."""

import pytest

from decision.tasks.sdk_agent import TurnResult
from decision.tasks.sdk_agent import run_turn as run_sdk_turn
from tests.decision.test_agent_conversations import agent as agent
from tests.decision.test_agent_conversations import submit
from tests.decision.test_sdk_agent import FakeModel, call, message


def test_real_sdk_plan_finalizes_the_same_stream_message_and_remains_shareable(agent, monkeypatch):
    plan = {"title": "玻璃通道排查方案", "summary": "先观察，再决定是否调整。", "steps": [
        {"title": "读取定位与导航状态", "detail": "保留本次观测，不写入地图。"},
        {"title": "检查缺失区域", "detail": "区分显示采样与真实碰撞数据。"}]}
    model = FakeModel([[message("我先整理一份方案。"), call("propose_plan", plan)]])
    events, admissions = [], []
    agent.agent_message.subscribe(events.append)

    async def runner(**kwargs):
        assert "generating a plan is not task completion" in kwargs["instructions"]
        return await run_sdk_turn(**kwargs, model=model)

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    turn = submit(agent, cid, "plan-request", "只给方案，先不要执行。", permission_mode="read_only",
                  motion_guard=lambda *args: admissions.append(args))
    snapshot = agent.get_conversation(cid)
    answers = [row for row in snapshot["messages"] if row["role"] == "assistant"]
    assert turn["state"] == "completed" and turn["kind"] == "plan"
    assert len(answers) == 1 and answers[0]["kind"] == "plan"
    assert answers[0]["metadata"] == {"status": "completed", "plan": plan}
    assert answers[0]["id"] == events[0]["id"] == events[-1]["id"]
    assert events[0]["metadata"]["status"] == "streaming"
    assert events[-1]["metadata"]["plan"] == plan and events[-1]["revision"] > events[0]["revision"]
    assert answers[0]["text"] == turn["text"] and "# 玻璃通道排查方案" in turn["text"]
    receipt = agent.share_conversation(cid)
    shared = agent.get_shared_conversation(receipt["token"])
    assert shared["messages"][-1]["text"] == answers[0]["text"]
    assert "方案草稿" in shared["messages"][-1]["text"]
    assert "2. **检查缺失区域**" in shared["messages"][-1]["text"]
    assert agent.search_conversations("区分显示采样")["results"][0]["message_id"] == answers[0]["id"]
    assert not admissions and agent._active_run is None and snapshot["approvals"] == []


@pytest.mark.parametrize(("kind", "failure"), [("reply", ""), ("failed", "error"), ("plan", "error")])
def test_plan_metadata_is_not_attached_to_other_or_failed_results(agent, monkeypatch, kind, failure):
    async def runner(**kwargs):
        return TurnResult(kind, "普通正文", [], failure, plan={"title": "不应展示", "steps": [{"title": "观察"}]})

    monkeypatch.setattr("decision.tasks.sdk_agent.run_turn", runner)
    cid = agent.create_conversation()["conversation_id"]
    submit(agent, cid, "not-plan", "问一句")
    answer = agent.get_conversation(cid)["messages"][-1]
    assert "plan" not in answer["metadata"]
