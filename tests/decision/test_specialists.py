"""Read-only delegation scope and cancellation, without external model calls."""
import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock
import pytest
from decision.tasks.specialists import review_specialists

TOOLS = [{"function": {"name": name, "parameters": {}}} for name in
         ("list_saved_maps", "get_agent_status", "run_python", "save_map_copy", "navigate_to")]

@pytest.mark.asyncio
async def test_parallel_specialists_are_isolated_and_writes_are_rejected():
    invoke = AsyncMock(return_value={"ok": True})
    entered = set()
    both_started = asyncio.Event()
    async def runner(task, **kwargs):
        names = {tool['function']['name'] for tool in kwargs['tools']}
        assert not names & {'run_python', 'save_map_copy', 'navigate_to'}
        assert kwargs['allow_specialists'] is False
        assert kwargs['history'] == []
        entered.update(names)
        if len(entered) == 2: both_started.set()
        await asyncio.wait_for(both_started.wait(), 1)
        with pytest.raises(ValueError, match='read-only'):
            await kwargs['invoke_tool']('navigate_to', {}, 'bad')
        await kwargs['invoke_tool'](next(iter(names)), {}, 'same-call-id')
        return SimpleNamespace(kind='reply', text='检查完毕')
    events = []
    result = await review_specialists('检查', parent_id='parent', tools=TOOLS, invoke_tool=invoke,
        on_event=events.append, runner=runner, runner_options={})
    assert len(result['reviews']) == 2
    assert invoke.await_count == 2
    assert len({call.args[2] for call in invoke.await_args_list}) == 2
    assert all(len(item['evidence']) == 1 for item in result['reviews'])
    assert len([e for e in events if e['status'] == 'completed']) == 2

@pytest.mark.asyncio
async def test_parent_cancel_joins_all_children_and_reports_cancelled():
    entered = 0
    settled = 0
    started = asyncio.Event()
    events = []
    async def runner(*args, **kwargs):
        nonlocal entered, settled
        entered += 1
        if entered == 2: started.set()
        try: await asyncio.Event().wait()
        finally: settled += 1
    work = asyncio.create_task(review_specialists('检查', parent_id='parent', tools=TOOLS,
        invoke_tool=AsyncMock(), on_event=events.append, runner=runner, runner_options={}))
    await asyncio.wait_for(started.wait(), 1)
    work.cancel()
    with pytest.raises(asyncio.CancelledError): await work
    assert settled == 2
    assert len([e for e in events if e['status'] == 'cancelled']) == 2

@pytest.mark.asyncio
async def test_partial_failure_preserves_other_report():
    async def runner(*args, **kwargs):
        if kwargs['tools'][0]['function']['name'] == 'list_saved_maps': raise RuntimeError('map unavailable')
        return SimpleNamespace(kind='reply', text='观测已返回')
    result = await review_specialists('检查', parent_id='parent', tools=TOOLS,
        invoke_tool=AsyncMock(), on_event=lambda e: None, runner=runner, runner_options={})
    assert [r['status'] for r in result['reviews']] == ['failed', 'completed']
