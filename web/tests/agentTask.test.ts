import assert from 'node:assert/strict'
import test from 'node:test'

import {
  isCurrentAgentResponse,
  presentAgentTask,
  reconcileCancelRequested,
  shouldDisplayAgentMessage,
  shouldPollAgentTask,
} from '../src/services/agentTask.ts'
import { cancelAgentTask, getAgentTask } from '../src/services/api.ts'
import type { AgentTaskStatusResponse } from '../src/types/index.ts'

function status(overrides: Partial<AgentTaskStatusResponse> = {}): AgentTaskStatusResponse {
  return {
    found: true,
    run_id: 'agent-current',
    state: 'running',
    motion_attempted: false,
    motion_success: false,
    ...overrides,
  }
}

test('task presentation follows domain evidence instead of inventing fixed steps', () => {
  const searching = presentAgentTask(status({
    navigation: { state: 'SEARCHING', terminal: false, success: false, search_moves: 2 },
  }))
  const planning = presentAgentTask(status({
    navigation: { state: 'CHECKING_OBSERVATION_PATH', terminal: false, success: false },
  }))
  const verifying = presentAgentTask(status({
    navigation: { state: 'VERIFYING_TARGET', terminal: false, success: false, confirmations: 1 },
  }))
  const following = presentAgentTask(status({
    navigation: { state: 'following', terminal: false, success: false },
  }))

  assert.equal(searching.label, '正在搜索目标')
  assert.equal(searching.detail, '正在执行第 2 次观察任务。')
  assert.equal(planning.label, '正在规划观察路线')
  assert.equal(verifying.label, '正在确认目标')
  assert.match(verifying.detail, /1 次视觉确认/)
  assert.equal(following.label, '正在持续跟随')
  assert.equal(following.terminal, false)
})

test('unknown, superseded and unconfirmed cancellation never present success or physical stop', () => {
  const unknown = presentAgentTask(status({ found: false, reason: 'not_current' }))
  const superseded = presentAgentTask(status({
    motion_attempted: true,
    navigation: { state: 'SUPERSEDED', terminal: true, success: false },
  }))
  const cancelled = presentAgentTask(status({ state: 'cancelled', stop_confirmed: false }))

  assert.equal(unknown.successful, false)
  assert.equal(unknown.terminal, false)
  assert.equal(superseded.successful, false)
  assert.equal(superseded.label, '任务未完成')
  assert.equal(cancelled.successful, false)
  assert.equal(cancelled.detail, '任务已取消，未派发运动。')
  assert.doesNotMatch(cancelled.detail, /停稳/)
})

test('Follow phases report the domain state even when its generic state is running', () => {
  const phases = [
    ['selecting', '正在选择跟随目标'],
    ['following', '正在持续跟随'],
    ['reacquiring', '正在重新识别跟随目标'],
    ['lost', '跟随目标暂时丢失'],
  ] as const

  for (const [phase, label] of phases) {
    const view = presentAgentTask(status({
      motion_attempted: true,
      navigation: {
        state: 'running', phase, follow_task_id: 'follow-1',
        terminal: false, success: false,
      },
    }))
    assert.equal(view.label, label)
    assert.equal(view.successful, false)
  }
})

test('failed or cancelled motion keeps polling until the native motion task ends', () => {
  assert.equal(shouldPollAgentTask(status({
    state: 'cancelled', motion_attempted: true, stop_confirmed: false,
  })), true)
  assert.equal(shouldPollAgentTask(status({
    state: 'failed', motion_attempted: true, stop_confirmed: false,
  })), true)
  assert.equal(shouldPollAgentTask(status({
    state: 'failed', motion_attempted: true, stop_confirmed: true,
  })), false)
  assert.equal(shouldPollAgentTask(status({
    state: 'cancelled', motion_attempted: false, stop_confirmed: false,
  })), false)
})

test('agent failure reason is not hidden by a generic navigation reason', () => {
  const view = presentAgentTask(status({
    state: 'failed', failure_reason: 'llm_timeout', motion_attempted: true,
    navigation: { state: 'FAILED', terminal: true, success: false, reason: 'agent_failed' },
  }))
  assert.equal(view.detail, '模型响应超时，任务没有继续执行。')
})

test('search exhaustion explains the bounded search without claiming the object does not exist', () => {
  const states = [
    'SEARCH_COVERAGE_EXHAUSTED',
    'SEARCH_NO_REACHABLE_VIEW',
    'SEARCH_BUDGET_EXHAUSTED',
  ]
  for (const state of states) {
    const view = presentAgentTask(status({
      motion_attempted: true,
      navigation: { state, terminal: true, success: false },
    }))
    assert.equal(view.successful, false)
    assert.doesNotMatch(view.detail, /不存在|没有水/)
  }
})

test('common Agent service failures have actionable user copy', () => {
  const expected: Record<string, string> = {
    agent_error: '任务执行器发生内部错误，任务未完成。',
    llm_unavailable: '模型服务尚未就绪，任务没有开始执行。',
  }
  for (const [failure_reason, detail] of Object.entries(expected)) {
    const view = presentAgentTask(status({ state: 'failed', failure_reason }))
    assert.equal(view.detail, detail)
  }
})

test('active navigation shows native holds without claiming physical motion', () => {
  const held = presentAgentTask(status({
    motion_attempted: true,
    navigation: { state: 'EXECUTING', terminal: false, success: false, reason: 'local_blocked' },
  }))
  const active = presentAgentTask(status({
    motion_attempted: true,
    navigation: { state: 'EXECUTING', terminal: false, success: false },
  }))

  assert.match(held.detail, /局部路线受阻/)
  assert.equal(active.detail, '正在执行路线，等待到达反馈。')
})

test('a pre-dispatch domain placeholder cannot fail a running Agent task', () => {
  const view = presentAgentTask(status({
    motion_attempted: false,
    navigation: { state: 'SUPERSEDED', terminal: true, success: false },
  }))

  assert.equal(view.label, '正在理解指令')
  assert.equal(view.terminal, false)
  assert.equal(view.successful, false)
})

test('only the current run and generation can apply a polling response', () => {
  const current = { runId: 'agent-new', generation: 8 }

  assert.equal(isCurrentAgentResponse(current, { run_id: 'agent-new' }, 8), true)
  assert.equal(isCurrentAgentResponse(current, { run_id: 'agent-old' }, 8), false)
  assert.equal(isCurrentAgentResponse(current, { run_id: 'agent-new' }, 7), false)
})

test('agent done is the only successful terminal presentation', () => {
  const done = presentAgentTask(status({ state: 'done', motion_attempted: true, motion_success: true }))
  const failed = presentAgentTask(status({ state: 'failed', failure_reason: 'llm_timeout' }))

  assert.equal(done.successful, true)
  assert.equal(done.terminal, true)
  assert.equal(failed.successful, false)
  assert.equal(failed.terminal, true)
  assert.equal(failed.detail, '模型响应超时，任务没有继续执行。')
})

test('confirmed domain result remains a non-terminal Agent finishing state', () => {
  const view = presentAgentTask(status({
    state: 'running', motion_attempted: true, motion_success: true,
    navigation: {
      state: 'COMPLETED', terminal: true, success: true, confirmations: 2,
    },
  }))

  assert.equal(view.label, '执行结果已确认，正在结束任务')
  assert.match(view.detail, /目标已通过视觉核验/)
  assert.equal(view.terminal, false)
  assert.equal(view.successful, false)
  assert.equal(view.canCancel, true)
  assert.equal(shouldPollAgentTask(status({ state: 'running' })), true)
})

test('cancel-pending UI clears only when the scoped task reaches a queryable end', () => {
  assert.equal(reconcileCancelRequested(true, status({
    state: 'cancelled', motion_attempted: true, stop_confirmed: true,
  })), false)
  assert.equal(reconcileCancelRequested(true, status({
    state: 'cancelled', motion_attempted: false, stop_confirmed: false,
  })), false)
  assert.equal(reconcileCancelRequested(true, status({
    state: 'cancelled', motion_attempted: true, stop_confirmed: false,
  })), true)
  assert.equal(reconcileCancelRequested(true, status({
    found: false, reason: 'not_current',
  })), false)
})

test('top-level failure cannot be hidden by an earlier successful domain result', () => {
  const view = presentAgentTask(status({
    state: 'failed', failure_reason: 'llm_error', motion_attempted: true,
    navigation: { state: 'COMPLETED', terminal: true, success: true, confirmations: 2 },
  }))
  assert.equal(view.label, '任务未完成')
  assert.equal(view.successful, false)
})

test('Agent API lookup and cancellation remain scoped to the supplied run id', async () => {
  const originalFetch = globalThis.fetch
  const originalTimeout = Object.getOwnPropertyDescriptor(AbortSignal, 'timeout')
  const timeoutCalls: number[] = []
  const requests: Array<{
    url: string
    body?: Record<string, unknown>
    signal?: AbortSignal | null
  }> = []
  Object.defineProperty(AbortSignal, 'timeout', {
    configurable: true,
    value: (milliseconds: number) => {
      timeoutCalls.push(milliseconds)
      return new AbortController().signal
    },
  })
  globalThis.fetch = async (input, init) => {
    const body = init?.body ? JSON.parse(String(init.body)) as Record<string, unknown> : undefined
    requests.push({ url: String(input), body, signal: init?.signal })
    if (init?.method === 'POST') {
      return Response.json({
        schema_version: 1,
        ok: true,
        status: 'cancelling',
        command: { name: 'agent_cancel', client_id: 'web-dashboard', accepted: true, replay: false, ts: 1 },
        run_id: 'agent/a',
        cancel_requested: true,
        stop_confirmed: false,
      })
    }
    return Response.json({ found: true, run_id: 'agent/a', state: 'running' })
  }

  try {
    await getAgentTask('agent/a')
    await cancelAgentTask('agent/a')
  } finally {
    globalThis.fetch = originalFetch
    if (originalTimeout) Object.defineProperty(AbortSignal, 'timeout', originalTimeout)
  }

  assert.deepEqual(timeoutCalls, [5_000, 5_000])
  assert.equal(requests[0]?.url, '/api/v1/agent/runs/agent%2Fa')
  assert.ok(requests[0]?.signal instanceof AbortSignal)
  assert.equal(requests[1]?.url, '/api/v1/agent/runs/agent%2Fa/cancel')
  assert.ok(requests[1]?.signal instanceof AbortSignal)
  assert.equal(requests[1]?.body?.client_id, 'web-dashboard')
  assert.match(String(requests[1]?.body?.request_id), /^agent-cancel-/)
})

test('active tasks hide internal planner chat and accept only scoped Agent summaries', () => {
  const base = { type: 'agent_message' as const, ts: 1, run_id: 'agent-current' }
  assert.equal(shouldDisplayAgentMessage({
    ...base, role: 'thinking', phase: 'planner_search', text: 'Fast path failed; checking vector memory',
  }, 'agent-current'), false)
  assert.equal(shouldDisplayAgentMessage({
    ...base, role: 'assistant', phase: 'agent_say', text: '我正在处理。',
  }, 'agent-current'), true)
  assert.equal(shouldDisplayAgentMessage({
    ...base, run_id: 'agent-old', role: 'assistant', phase: 'agent_done', text: 'done',
  }, 'agent-current'), false)
})
