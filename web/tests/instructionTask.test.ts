import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import { createRequire } from 'node:module'
import ts from 'typescript'
import type { InstructionTaskController } from '../src/hooks/useInstructionTask.ts'
import type { AgentTaskStatusResponse } from '../src/types/index.ts'
import * as agentTask from '../src/services/agentTask.ts'
import * as instructionTask from '../src/services/instructionTask.ts'
import * as navigationStatus from '../src/services/navigationStatus.ts'

const errorExports: Record<string, unknown> = {}
const apiSource = readFileSync(new URL('../src/services/api.ts', import.meta.url), 'utf8')
runInNewContext(ts.transpileModule(apiSource.slice(apiSource.indexOf('export class GatewayApiError'), apiSource.indexOf('export function formatCommandAck')), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
}).outputText, { exports: errorExports, Error })
const GatewayApiError = errorExports.GatewayApiError as new (code: number, body: Record<string, unknown>) => Error

function status(changes: Partial<AgentTaskStatusResponse> = {}): AgentTaskStatusResponse {
  return { found: true, run_id: 'run-1', state: 'running', motion_attempted: false, ...changes }
}

function harness(apiOverrides: Record<string, unknown> = {}, allowed = true) {
  const slots: Array<{ value?: unknown; deps?: unknown[]; cleanup?: () => void }> = []
  const timers = new Map<number, () => Promise<void>>()
  let cursor = 0
  let timerId = 0
  let effects: Array<() => void> = []
  const requests: string[] = []
  const api = {
    sendInstruction: async (text: string, runId?: string) => {
      requests.push(text)
      return runId ? { ok: false, accepted: false, operation: 'start', reason: 'task_active' }
        : { ok: true, accepted: true, operation: 'start', run_id: 'run-1' }
    },
    getAgentTask: async () => status(),
    cancelAgentTask: async () => ({ ok: true, accepted: true, operation: 'start', run_id: 'run-1', cancel_requested: true, stop_confirmed: false }),
    formatCommandAck: (response: { reason?: string }) => response.reason ?? 'Rejected',
    formatCommandError: (error: unknown) => error instanceof Error ? error.message : String(error),
    isGatewayApiError: errorExports.isGatewayApiError,
    ...apiOverrides,
  }
  const react = {
    useState(initial: unknown) {
      const slot = cursor++
      slots[slot] ??= { value: initial }
      return [slots[slot].value, (value: unknown) => { slots[slot].value = value }]
    },
    useRef(initial: unknown) {
      const slot = cursor++
      slots[slot] ??= { value: { current: initial } }
      return slots[slot].value
    },
    useCallback(callback: unknown, deps: unknown[]) {
      const slot = cursor++
      const previous = slots[slot]
      if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) {
        slots[slot] = { value: callback, deps }
      }
      return slots[slot].value
    },
    useEffect(effect: () => (() => void) | undefined, deps: unknown[]) {
      const slot = cursor++
      const previous = slots[slot]
      if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) {
        effects.push(() => {
          previous?.cleanup?.()
          slots[slot] = { deps, cleanup: effect() }
        })
      }
    },
  }
  const exports: Record<string, unknown> = {}
  const source = readFileSync(new URL('../src/hooks/useInstructionTask.ts', import.meta.url), 'utf8')
  const code = ts.transpileModule(source, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  runInNewContext(code, {
    exports,
    require: (name: string) => name === 'react' ? react
      : name.endsWith('/api') ? api : name.endsWith('/agentTask') ? agentTask : instructionTask,
    setTimeout: (callback: () => Promise<void>) => { timers.set(++timerId, callback); return timerId },
    clearTimeout: (id: number) => { timers.delete(id) },
    Error,
  })
  const runHook = exports.useInstructionTask as (options: {
    motionStartAllowed: boolean; motionStartBlockedReason: string; enabled: boolean
  }) => InstructionTaskController
  function render(motionAllowed = allowed, enabled = true) {
    cursor = 0
    effects = []
    const controller = runHook({ motionStartAllowed: motionAllowed, motionStartBlockedReason: 'Telemetry stale', enabled })
    for (const effect of effects) effect()
    return controller
  }
  async function tick() {
    const timer = timers.entries().next().value
    assert.ok(timer, 'expected a task status poll')
    timers.delete(timer[0])
    await timer[1]()
  }
  return { render, tick, timers, requests }
}

test('standby pauses task polling and start restores it', () => {
  const h = harness()
  h.render().adopt('run-1', '会话任务')
  h.render()
  assert.equal(h.timers.size, 1)
  h.render(true, false)
  assert.equal(h.timers.size, 0)
  h.render()
  assert.equal(h.timers.size, 1)
})

test('conversation adoption polls the real run without overwriting newer updates for the same id', async () => {
  const h = harness({ getAgentTask: async () => status({ state: 'done', summary: '真实结果' }) })
  h.render().adopt('run-1', '会话任务')
  h.render()
  await h.tick()
  const polled = h.render().task
  assert.equal(polled?.status.state, 'done')
  h.render().adopt('run-1', '重复快照')
  assert.equal(h.render().task, polled)
  h.render().adopt('run-2', '下一项任务')
  assert.equal(h.render().task?.runId, 'run-2')
  assert.equal(h.render().task?.instruction, '下一项任务')
  assert.equal(h.render().task?.statusSource, 'ack')
})

test('idle dialogue reaches the owner when motion is blocked and reports idle without an execution ACK', async () => {
  const requests: Array<{ text: string; controlOnly?: boolean }> = []
  const h = harness({ sendInstruction: async (text: string, _runId?: string, controlOnly?: boolean) => {
    requests.push({ text, controlOnly })
    return { ok: true, accepted: true, operation: 'query', state: 'idle', run_id: '', task_status: { found: false, state: 'idle' } }
  } }, false)
  await h.render().submit('现在在干什么')
  assert.deepEqual(requests, [{ text: '现在在干什么', controlOnly: true }])
  assert.equal(h.render().task, null)
  assert.match(h.render().receipt?.message ?? '', /没有正在执行/)
  assert.doesNotMatch(h.render().receipt?.message ?? '', /已受理/)
  assert.equal(h.timers.size, 0)
})

test('terminal task queries preserve run identity instruction and generation', async () => {
  let operation = 'start'
  const requests: Array<string | undefined> = []
  const h = harness({ sendInstruction: async (_text: string, runId?: string) => {
    requests.push(runId)
    return { ok: true, accepted: true, operation, run_id: 'run-1', task_status: status({ state: 'done' }) }
  }, getAgentTask: async () => status({ state: 'done' }) })
  await h.render().submit('帮我找水')
  h.render()
  await h.tick()
  const original = h.render().task!
  operation = 'query'
  await h.render(false).submit('找到没有')
  assert.equal(requests[1], original.runId)
  assert.equal(h.render().task?.generation, original.generation)
  assert.equal(h.render().task?.instruction, original.instruction)
  assert.equal(h.render().task?.statusSource, 'poll')
  assert.equal(h.timers.size, 0)
})

test('only an explicitly accepted start creates a task and increments its generation', async () => {
  let reply: Record<string, unknown> = { ok: false, accepted: false, operation: 'pause', reason: 'unsupported_control' }
  const h = harness({ sendInstruction: async () => reply }, false)
  await assert.rejects(h.render().submit('暂停'), /unsupported_control/)
  assert.equal(h.render().task, null)
  reply = { ok: true, operation: 'start', run_id: 'unconfirmed-run' }
  await assert.rejects(h.render().submit('跟随我'), /未明确接受/)
  assert.equal(h.render().task, null)
  reply = { ok: true, accepted: true, operation: 'start', run_id: 'run-1' }
  await h.render().submit('跟随我')
  assert.equal(h.render().task?.generation, 1)
})

test('idle cancellation restores the real owner snapshot and waits for its stop confirmation', async () => {
  const h = harness({ sendInstruction: async () => ({ ok: true, accepted: true, operation: 'cancel', run_id: 'run-1',
    task_status: status({ instruction: '跟随红衣服的人', state: 'cancelled', motion_attempted: true, stop_confirmed: false }) }) }, false)
  await h.render().submit('停止')
  const current = h.render().task!
  assert.equal(current.instruction, '跟随红衣服的人')
  assert.equal(current.statusSource, 'poll')
  assert.equal(current.cancelRequested, true)
  assert.equal(instructionTask.instructionTaskCanDismiss(current), false)
  assert.equal(h.timers.size, 1)
})

test('an owner rejected ordinary start is sent for classification but creates no task', async () => {
  const requests: Array<{ text: string; controlOnly?: boolean }> = []
  const h = harness({ sendInstruction: async (text: string, _runId?: string, controlOnly?: boolean) => {
    requests.push({ text, controlOnly })
    return { ok: false, accepted: false, operation: 'start', reason: 'motion_not_ready' }
  } }, false)
  await assert.rejects(h.render().submit('去门口'), /motion_not_ready/)
  assert.deepEqual(requests, [{ text: '去门口', controlOnly: true }])
  assert.equal(h.render().task, null)
})

test('an active local run sends control_only and still tracks an actually accepted replacement', async () => {
  let acceptReplacement = false
  const requests: Array<{ runId?: string; controlOnly?: boolean }> = []
  const h = harness({ sendInstruction: async (_text: string, runId?: string, controlOnly?: boolean) => {
    requests.push({ runId, controlOnly })
    if (runId && !acceptReplacement) return { ok: false, accepted: false, operation: 'start', reason: 'task_active' }
    return { ok: true, accepted: true, operation: 'start', run_id: runId ? 'run-2' : 'run-1' }
  } })
  await h.render().submit('跟随我')
  const original = h.render().task!
  await assert.rejects(h.render().submit('去门口'), /请先结束当前任务/)
  assert.deepEqual(requests[1], { runId: 'run-1', controlOnly: true })
  assert.equal(h.render().task, original)
  acceptReplacement = true
  await h.render().submit('去门口')
  assert.equal(h.render().task?.runId, 'run-2')
  assert.equal(h.render().task?.instruction, '去门口')
  assert.equal(h.render().task?.generation, original.generation + 1)
})

test('active queries bind the current run without changing instruction or generation when motion is blocked', async () => {
  const requests: Array<{ text: string; runId?: string }> = []
  const h = harness({ sendInstruction: async (text: string, runId?: string) => {
    requests.push({ text, runId })
    return runId ? { ok: true, accepted: true, operation: 'query', run_id: runId, task_status: status({ summary: 'Following' }) }
      : { ok: true, accepted: true, operation: 'start', run_id: 'run-1' }
  } })
  await h.render().submit('跟随我')
  const initial = h.render().task!
  await h.render(false).submit('现在做什么')
  const current = h.render(false).task!
  assert.deepEqual(requests[1], { text: '现在做什么', runId: 'run-1' })
  assert.equal(current.runId, initial.runId)
  assert.equal(current.generation, initial.generation)
  assert.equal(current.instruction, initial.instruction)
  assert.equal(current.status.summary, 'Following')
  assert.equal(h.timers.size, 1)
})

test('HTTP 409 instruction rejection preserves the draft-facing reason and the current run', async () => {
  let reason = ''
  const h = harness({ sendInstruction: async () => {
    if (reason) throw new GatewayApiError(409, { ok: false, error: 'command_rejected', reason })
    return { ok: true, accepted: true, operation: 'start', run_id: 'run-1' }
  } })
  await h.render().submit('跟随我')
  const current = h.render().task
  for (const [rejection, expected] of [['task_active', /请先结束当前任务/],
    ['motion_start_blocked', /不允许启动新运动任务/], ['unsupported_control', /不支持这项控制/]] as const) {
    reason = rejection
    await assert.rejects(h.render(false).submit('控制请求'), expected)
    assert.equal(h.render().task, current)
    assert.match(h.render().error, expected)
  }
  reason = 'unrelated_gateway_error'
  await assert.rejects(h.render().submit('控制请求'), /command_rejected/)
})

test('a late running query cannot downgrade a newer terminal poll', async () => {
  for (const terminal of [status({ state: 'done' }), status({ state: 'cancelled', motion_attempted: true, stop_confirmed: true })]) {
    let release!: (response: unknown) => void
    const h = harness({ sendInstruction: async (_text: string, runId?: string) => runId
      ? new Promise(resolve => { release = resolve })
      : { ok: true, accepted: true, operation: 'start', run_id: 'run-1' }, getAgentTask: async () => terminal })
    await h.render().submit('跟随我')
    const generation = h.render().task?.generation
    const pending = h.render().submit('现在在干什么')
    await h.tick()
    const polled = h.render().task
    release({ ok: true, accepted: true, operation: 'query', run_id: 'run-1', task_status: status() })
    await pending
    assert.equal(h.render().task, polled)
    assert.equal(h.render().task?.generation, generation)
    assert.equal(h.render().task?.status.state, terminal.state)
    assert.equal(h.timers.size, 0)
  }
})

test('dialogue cancel acknowledgement retains polling until true stop confirmation', async () => {
  const h = harness({ sendInstruction: async (_text: string, runId?: string) => runId
    ? { ok: true, accepted: true, operation: 'cancel', run_id: runId,
      task_status: status({ state: 'cancelled', motion_attempted: true, stop_confirmed: false }) }
    : { ok: true, accepted: true, operation: 'start', run_id: 'run-1' } })
  await h.render().submit('跟随我')
  await h.render(false).submit('停止')
  const current = h.render().task!
  assert.equal(current.cancelRequested, true)
  assert.equal(instructionTask.instructionTaskCanDismiss(current), false)
  assert.equal(h.timers.size, 1)
  h.render().dismiss()
  assert.ok(h.render().task)
})

test('unsupported control and stale run responses cannot clear or replace the active task', async () => {
  let reply: Record<string, unknown> = { ok: false, accepted: false, operation: 'pause', reason: 'unsupported_control' }
  const h = harness({ sendInstruction: async (_text: string, runId?: string) => runId ? reply : { ok: true, accepted: true, operation: 'start', run_id: 'run-1' } })
  await h.render().submit('跟随我')
  const original = h.render().task!
  await assert.rejects(h.render(false).submit('暂停'), /unsupported_control/)
  assert.equal(h.render().task, original)
  reply = { ok: true, accepted: true, operation: 'query', run_id: 'old-run', task_status: status({ run_id: 'old-run' }) }
  await assert.rejects(h.render().submit('状态'), /不匹配/)
  assert.equal(h.render().task, original)
})

test('request ACK never claims execution or completion, even with a done ACK state', async () => {
  const h = harness({ sendInstruction: async () => ({ ok: true, accepted: true, operation: 'start', run_id: 'run-1', state: 'done' }) })
  await h.render().submit('帮我找一瓶水')
  const current = h.render().task!
  assert.equal(current.instruction, '帮我找一瓶水')
  assert.equal(instructionTask.presentInstructionTask(current).terminal, false)
  assert.equal(instructionTask.presentInstructionTask(current).label, '指令已受理')
  assert.equal(h.timers.size, 1)
  h.render().dismiss()
  assert.ok(h.render().task)
})

test('duplicate submissions and status responses for another run do not replace the task', async () => {
  let finishSend!: (value: { ok: boolean; accepted: boolean; operation: string; run_id: string }) => void
  const h = harness({
    sendInstruction: (_text: string, runId?: string) => runId
      ? Promise.resolve({ ok: false, accepted: false, reason: 'task_active' })
      : new Promise(resolve => { finishSend = resolve }),
    getAgentTask: async () => status({ run_id: 'old-run', state: 'done' }),
  })
  const pending = h.render().submit('跟随我')
  await assert.rejects(h.render().submit('第二条指令'), /正在发送/)
  finishSend({ ok: true, accepted: true, operation: 'start', run_id: 'run-1' })
  await pending
  h.render()
  await h.tick()
  assert.equal(h.render().task?.statusSource, 'ack')
  assert.equal(h.render().task?.runId, 'run-1')
  await assert.rejects(h.render().submit('第二条指令'), /请先结束当前任务/)
})

test('polling never overlaps and recovers from a failed status lookup', async () => {
  let finishPoll!: (value: AgentTaskStatusResponse) => void
  let calls = 0
  const h = harness({ getAgentTask: () => {
    calls++
    if (calls === 1) return Promise.reject(new Error('Offline'))
    return new Promise(resolve => { finishPoll = resolve })
  } })
  await h.render().submit('跟随我')
  h.render()
  await h.tick()
  assert.equal(h.render().task?.pollError, 'Offline')
  const pending = h.tick()
  h.render()
  assert.equal(calls, 2)
  assert.equal(h.timers.size, 0)
  finishPoll(status())
  await pending
  assert.equal(h.render().task?.pollError, '')
  assert.equal(h.timers.size, 1)
})

test('cancel acknowledgement keeps the task visible until polled stop confirmation', async () => {
  let confirmed = false
  const h = harness({ getAgentTask: async () => status({
    state: 'cancelled', motion_attempted: true, stop_confirmed: confirmed,
  }) })
  await h.render().submit('跟随我')
  await h.render().cancel()
  assert.equal(h.render().task?.cancelRequested, true)
  h.render().dismiss()
  assert.ok(h.render().task)
  await h.tick()
  let current = h.render().task!
  assert.equal(instructionTask.instructionTaskCanDismiss(current), false)
  assert.equal(instructionTask.presentInstructionTask(current).label, '正在等待任务停止确认')
  assert.equal(h.timers.size, 1)
  confirmed = true
  await h.tick()
  current = h.render().task!
  assert.equal(current.cancelRequested, false)
  assert.equal(instructionTask.instructionTaskCanDismiss(current), true)
  assert.equal(h.timers.size, 0)
  h.render().dismiss()
  assert.equal(h.render().task, null)
})

test('a late cancel ACK cannot overwrite a polled completed task', async () => {
  let finishCancel!: (value: unknown) => void
  const h = harness({
    getAgentTask: async () => status({ state: 'done' }),
    cancelAgentTask: () => new Promise(resolve => { finishCancel = resolve }),
  })
  await h.render().submit('帮我找一瓶水')
  const pending = h.render().cancel()
  await h.tick()
  assert.equal(instructionTask.instructionTaskCanDismiss(h.render().task!), true)
  finishCancel({ ok: true, accepted: true, operation: 'start', run_id: 'run-1', cancel_requested: true, stop_confirmed: false })
  await pending
  assert.equal(instructionTask.instructionTaskCanDismiss(h.render().task!), true)
})

test('rejected cancellation reports the control error without making status feedback stale', async () => {
  const h = harness({ cancelAgentTask: async () => ({ ok: false, cancel_requested: false, reason: 'not_current' }) })
  await h.render().submit('跟随我')
  h.render()
  await h.tick()
  await h.render().cancel()
  const current = h.render().task!
  assert.equal(current.cancelRequested, false)
  assert.equal(current.cancelError, 'not_current')
  assert.equal(current.pollError, '')
  assert.equal(instructionTask.presentInstructionTask(current).label, '正在理解指令')
})

test('stop confirmation arriving in the cancel response releases a polled cancelled task', async () => {
  let finishCancel!: (value: unknown) => void
  const h = harness({
    getAgentTask: async () => status({ state: 'cancelled', motion_attempted: true, stop_confirmed: false }),
    cancelAgentTask: () => new Promise(resolve => { finishCancel = resolve }),
  })
  await h.render().submit('跟随我')
  const pending = h.render().cancel()
  await h.tick()
  h.render()
  finishCancel({ ok: true, accepted: true, operation: 'start', run_id: 'run-1', cancel_requested: true, stop_confirmed: true })
  await pending
  const current = h.render().task!
  assert.equal(current.cancelRequested, false)
  assert.equal(instructionTask.instructionTaskCanDismiss(current), true)
  assert.equal(h.timers.size, 0)
})

test('semantic fallback is an admission receipt without an invented Agent run', async () => {
  const h = harness({ sendInstruction: async () => ({ ok: true, accepted: true, operation: 'start', executor: 'semantic' }) })
  await h.render().submit('帮我找一瓶水')
  const current = h.render()
  assert.equal(current.task, null)
  assert.equal(current.receipt?.executor, 'semantic')
  assert.match(current.receipt?.message ?? '', /执行结果尚未确认/)
  assert.equal(h.timers.size, 0)
})

test('external replacement releases this UI run without claiming stopped or cancelling its replacement', async () => {
  let cancelCalls = 0
  const h = harness({
    getAgentTask: async () => status({ found: false, reason: 'not_current' }),
    cancelAgentTask: async () => { cancelCalls++; return { ok: true, cancel_requested: true } },
  })
  await h.render().submit('跟随我')
  h.render()
  await h.tick()
  const current = h.render().task!
  assert.equal(instructionTask.instructionTaskCanDismiss(current), true)
  assert.equal(instructionTask.presentInstructionTask(current).successful, false)
  assert.match(instructionTask.presentInstructionTask(current).detail, /停止状态未获得确认/)
  assert.equal(h.timers.size, 0)
  await h.render().cancel()
  assert.equal(cancelCalls, 0)
  h.render().dismiss()
  assert.equal(h.render().task, null)
  await h.render().submit('帮我找一瓶水')
  assert.equal(h.render().task?.instruction, '帮我找一瓶水')
})

test('temporary Agent unavailability keeps retrying without releasing the current task', async () => {
  let available = false
  const h = harness({ getAgentTask: async () => available ? status() : status({ found: false, reason: 'agent_unavailable' }) })
  await h.render().submit('跟随我')
  h.render()
  await h.tick()
  assert.equal(instructionTask.instructionTaskCanDismiss(h.render().task!), false)
  assert.equal(h.timers.size, 1)
  available = true
  await h.tick()
  assert.equal(h.render().task?.status.found, true)
  assert.equal(h.timers.size, 1)
})

test('the active task card retains an enabled dialogue composer when motion is blocked', async () => {
  const requests: string[] = []
  const h = harness()
  await h.render().submit('跟随我')
  const controller = { ...h.render(), submit: async (text: string) => { requests.push(text) } }
  const require = createRequire(import.meta.url)
  const exports: Record<string, unknown> = {}
  const source = readFileSync(new URL('../src/components/InstructionTaskBar.tsx', import.meta.url), 'utf8')
  const code = ts.transpileModule(source, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
  interface Element { type: string; props: Record<string, unknown> & { children?: unknown } }
  const jsx = (type: string, props: Element['props']) => ({ type, props })
  let stateSlot = 0
  runInNewContext(code, { exports, require: (name: string) => {
    if (name === 'react') return { useRef: () => ({ current: null }), useState: (initial: unknown) => [stateSlot++ === 0 ? '现在做什么' : initial, () => {}] }
    if (name === 'react/jsx-runtime') return { jsx, jsxs: jsx }
    if (name.endsWith('/instructionTask')) return instructionTask
    if (name.endsWith('/navigationStatus')) return navigationStatus
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  const component = exports.InstructionTaskBar as (props: Record<string, unknown>) => Element
  const root = component({ controller, locale: 'zh', motionStartAllowed: false, motionStartBlockedReason: 'Telemetry stale', hasNativeTask: true })
  const elements: Element[] = []
  function visit(value: unknown) {
    if (Array.isArray(value)) { value.forEach(visit); return }
    if (!value || typeof value !== 'object' || !('props' in value)) return
    const element = value as Element
    elements.push(element)
    visit(element.props.children)
  }
  visit(root)
  assert.ok(elements.some(element => element.type === 'input'))
  const send = elements.find(element => element.type === 'button' && element.props['aria-label'] === '发送任务')!
  assert.equal(send.props.disabled, false)
  await (send.props.onClick as () => Promise<void>)()
  assert.deepEqual(requests, ['现在做什么'])
})

test('chat shows the current idle feedback once and drops it when dismissed or a run is installed', async () => {
  const h = harness({ sendInstruction: async () => ({ ok: true, accepted: true, operation: 'query', state: 'idle', run_id: '' }) })
  await h.render().submit('现在在做什么')
  const require = createRequire(import.meta.url)
  const react = require('react')
  const { renderToStaticMarkup } = require('react-dom/server')
  const exports: Record<string, unknown> = {}
  const code = ts.transpileModule(readFileSync(new URL('../src/components/ChatPanel.tsx', import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
  runInNewContext(code, { exports, require: (name: string) => {
    if (name.endsWith('/instructionTask')) return instructionTask
    if (name.endsWith('/agentTask')) return agentTask
    if (name.endsWith('/navigationStatus')) return { liveNavigationStatus: () => null }
    if (name.endsWith('/slashCommands')) return { isSlashCommand: () => false, matchSlashCommands: () => [] }
    if (name.endsWith('/ThinkingBubble')) return { ThinkingBubble: () => null }
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  }, Date })
  const render = (controller: InstructionTaskController) => renderToStaticMarkup(react.createElement(exports.ChatPanel, {
    controller, sseState: { connected: false }, motionStartAllowed: false, motionStartBlockedReason: 'Telemetry stale',
  })) as string
  const controller = h.render()
  for (let i = 0; i < 2; i++) assert.equal(render(controller).split('当前没有正在执行的任务。').length - 1, 1)
  assert.doesNotMatch(render({ ...controller, receipt: null }), /当前没有正在执行的任务/)
  const task: instructionTask.InstructionTask = { runId: 'run-2', instruction: '去门口', generation: 1, executor: 'agent',
    status: status({ run_id: 'run-2' }), statusSource: 'poll', cancelRequested: false, cancelError: '', pollError: '' }
  assert.doesNotMatch(render({ ...controller, task }), /当前没有正在执行的任务/)
})

test('non-scene workspaces hide the idle composer but retain active task controls and receipts', () => {
  const require = createRequire(import.meta.url)
  const react = require('react')
  const { renderToStaticMarkup } = require('react-dom/server')
  const exports: Record<string, unknown> = {}
  const code = ts.transpileModule(readFileSync(new URL('../src/components/InstructionTaskBar.tsx', import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
  runInNewContext(code, { exports, require: (name: string) => {
    if (name.endsWith('/instructionTask')) return instructionTask
    if (name.endsWith('/navigationStatus')) return navigationStatus
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  const idle = { task: null, receipt: null, sending: false, error: '', submit() {}, dismiss() {}, cancel() {} }
  const render = (controller: object, showComposer = false): string => renderToStaticMarkup(react.createElement(exports.InstructionTaskBar, {
    controller, locale: 'zh', showComposer, hasNativeTask: false, motionStartAllowed: true, motionStartBlockedReason: '',
  }))
  assert.equal(render(idle), '')
  assert.match(render(idle, true), /想让机器人做什么/)
  const task: instructionTask.InstructionTask = { runId: 'run-2', instruction: '去门口', generation: 1, executor: 'agent',
    status: status({ run_id: 'run-2' }), statusSource: 'poll', cancelRequested: false, cancelError: '', pollError: '' }
  const active = render({ ...idle, task })
  assert.match(active, /去门口/)
  assert.match(active, /结束任务/)
  assert.doesNotMatch(active, /<input/)
  const visible = (changes: Partial<AgentTaskStatusResponse>) => render({ ...idle, task: { ...task, status: status(changes) } }).split('<details')[0]
  for (const state of ['FAILED', 'RECOVERING', 'PAUSED']) {
    assert.match(visible({ navigation: { state, reason: 'local_blocked' } }), /局部路线受阻。/)
  }
  assert.match(visible({ state: 'failed', failure_reason: 'llm_timeout' }), /模型响应超时，任务没有继续执行。/)
  assert.doesNotMatch(visible({ navigation: { state: 'EXECUTING', reason: 'path_active' } }), /Navigation status|导航状态/)
  const unconfirmed = render({ ...idle, task: { ...task, statusSource: 'ack', status: status({ navigation: { state: 'FAILED', reason: 'local_blocked' } }) } }).split('<details')[0]
  assert.doesNotMatch(unconfirmed, /局部路线受阻。/)
  assert.match(render({ ...idle, sending: true }), /正在发送指令/)
  assert.match(render({ ...idle, error: '服务未连接' }), /服务未连接/)
  const receipt = render({ ...idle, receipt: { instruction: '现在什么状态', operation: 'query', message: '当前空闲' } })
  assert.match(receipt, /当前空闲/)
  assert.doesNotMatch(receipt, /<input/)
})
