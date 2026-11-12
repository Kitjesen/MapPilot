import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import * as lifecycle from '../src/services/navigationTaskLifecycle.ts'
import * as navigationStatus from '../src/services/navigationStatus.ts'

const require = createRequire(import.meta.url)
const source = readFileSync(new URL('../src/components/CurrentTaskCard.tsx', import.meta.url), 'utf8')
const code = ts.transpileModule(source, { compilerOptions: {
  target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX,
} }).outputText
interface Element { type: unknown; props: Record<string, unknown> }

function card(initialState = 'PAUSED', cancelResponse?: () => Promise<{ ok: boolean; status: string }>) {
  const state: unknown[] = []
  let cursor = 0
  const feedback = { identity: { task_id: 'task-a', submitted_at: 1 }, stale: false,
    snapshot: { found: true, status: { state_name: initialState, reason: '', ts: Date.now() / 1000 } }, dismiss() {} }
  const calls: string[] = []
  const request = (action: string) => async (id: string) => {
    calls.push(`${action}:${id}`)
    return action === 'cancel' && cancelResponse ? cancelResponse() : { ok: true, status: `${action}_requested` }
  }
  const ConfirmModal = () => null
  const exports: Record<string, unknown> = {}
  const mocks: Record<string, unknown> = {
    react: { useEffect() {}, useState(initial: unknown) {
      const index = cursor++
      if (!(index in state)) state[index] = typeof initial === 'function' ? initial() : initial
      return [state[index], (next: unknown) => { state[index] = typeof next === 'function' ? next(state[index]) : next }]
    } },
    'react-dom': { createPortal: (node: Element) => node },
    '../hooks/useCurrentNavigationTask': { useCurrentNavigationTask: () => feedback },
    '../i18n': { text: (_locale: string, _en: string, zh: string) => zh },
    '../services/navigationTaskLifecycle': lifecycle,
    '../services/navigationStatus': navigationStatus,
    '../services/api': { resumeNavigationTask: request('resume'), cancelNavigationTask: request('cancel'),
      pauseNavigationTask: request('pause'), formatCommandAck: () => '取消请求被拒绝',
      formatCommandError: (error: Error) => error.message },
    './Modal': { ConfirmModal }, './CurrentTaskCard.module.css': { default: {} },
  }
  runInNewContext(code, { exports, document: { body: {} }, require: (name: string) => mocks[name] ?? require(name) })
  const Component = exports.CurrentTaskCard as (props: Record<string, unknown>) => Element
  let tree: Element
  function render(props: Record<string, unknown> = {}) {
    cursor = 0
    tree = Component({ locale: 'zh', showToast() {}, ...props })
  }
  function find(predicate: (node: Element) => boolean, node: unknown): Element | undefined {
    if (!node || typeof node !== 'object') return
    if (Array.isArray(node)) {
      for (const child of node) { const result = find(predicate, child); if (result) return result }
      return
    }
    const element = node as Element
    if (!element.props || element.type === 'details') return
    return predicate(element) ? element : find(predicate, element.props.children)
  }
  function button(label: string) {
    const result = find(node => node.type === 'button' && (node.props['aria-label'] === label
      || (Array.isArray(node.props.children) && node.props.children.some(child => child?.props?.children === label))), tree)
    assert.ok(result, label)
    return result
  }
  function modal() { const result = find(node => node.type === ConfirmModal, tree); assert.ok(result); return result.props }
  function click(element: Element) { assert.ok(!element.props.disabled); (element.props.onClick as () => void)() }
  render()
  function visibleText(node: unknown): string {
    if (typeof node === 'string') return node
    if (Array.isArray(node)) return node.map(visibleText).join(' ')
    if (!node || typeof node !== 'object') return ''
    const element = node as Element
    return element.type === 'details' || element.type === ConfirmModal ? '' : visibleText(element.props?.children)
  }
  return { feedback, calls, render, button, modal, click, visibleText: () => visibleText(tree) }
}

test('opening or dismissing continuation confirmation sends nothing; confirming dispatches once', async () => {
  const ui = card()
  ui.click(ui.button('继续导航')); ui.render()
  assert.equal(ui.modal().open, true)
  assert.deepEqual(ui.calls, [])
  ;(ui.modal().onCancel as () => void)(); ui.render()
  assert.equal(ui.modal().open, false)
  assert.deepEqual(ui.calls, [])
  ui.click(ui.button('继续导航')); ui.render()
  ;(ui.modal().onConfirm as () => void)(); await Promise.resolve(); ui.render()
  assert.deepEqual(ui.calls, ['resume:task-a'])
  assert.equal(ui.modal().open, false)
  assert.equal(ui.button('继续导航').props.disabled, true)
})

test('a confirmation cannot transfer to a replacement task or bypass updated motion gates', () => {
  const switched = card()
  switched.click(switched.button('继续导航')); switched.render()
  switched.feedback.identity.task_id = 'task-b'; switched.render()
  assert.equal(switched.modal().open, false)
  ;(switched.modal().onConfirm as () => void)()
  assert.deepEqual(switched.calls, [])
  for (const change of ['stale', 'gate', 'state']) {
    const ui = card()
    ui.click(ui.button('继续导航')); ui.render()
    if (change === 'stale') ui.feedback.stale = true
    if (change === 'state') ui.feedback.snapshot.status.state_name = 'EXECUTING'
    ui.render(change === 'gate' ? { resumeAllowed: false } : {})
    assert.equal(ui.modal().confirmDisabled, true)
    ;(ui.modal().onConfirm as () => void)()
    assert.deepEqual(ui.calls, [])
  }
})

test('task cancellation also requires confirmation and waits for the robot result', async () => {
  const ui = card('EXECUTING')
  ui.click(ui.button('取消任务')); ui.render()
  assert.equal(ui.modal().open, true)
  assert.deepEqual(ui.calls, [])
  ;(ui.modal().onCancel as () => void)(); ui.render()
  assert.deepEqual(ui.calls, [])
  ui.click(ui.button('取消任务')); ui.render()
  ;(ui.modal().onConfirm as () => void)(); await Promise.resolve(); ui.render()
  assert.deepEqual(ui.calls, ['cancel:task-a'])
  assert.equal(ui.button('取消任务').props.disabled, true)
  assert.equal(ui.feedback.snapshot.status.state_name, 'EXECUTING')
})


test('cancel feedback moves from sending through stop confirmation to the confirmed result', async () => {
  let finish: (response: { ok: boolean; status: string }) => void = () => {}
  const ui = card('EXECUTING', () => new Promise(resolve => { finish = resolve }))
  ui.click(ui.button('取消任务')); ui.render()
  ;(ui.modal().onConfirm as () => void)(); ui.render()
  assert.match(ui.visibleText(), /正在请求取消/)
  assert.doesNotMatch(ui.visibleText(), /已取消/)
  assert.equal(ui.button('取消任务').props.disabled, true)
  finish({ ok: true, status: 'cancel_requested' })
  await new Promise(resolve => setImmediate(resolve)); ui.render()
  assert.match(ui.visibleText(), /等待停车确认/)
  assert.doesNotMatch(ui.visibleText(), /已取消/)
  ui.feedback.snapshot.status.state_name = 'CANCELLED'; ui.render()
  assert.match(ui.visibleText(), /已取消/)
  assert.doesNotMatch(ui.visibleText(), /等待停车确认/)
})

test('rejected and failed cancel requests leave the task available for retry', async () => {
  for (const response of [async () => ({ ok: false, status: 'rejected' }), async () => { throw new Error('network unavailable') }]) {
    const ui = card('EXECUTING', response)
    ui.click(ui.button('取消任务')); ui.render()
    ;(ui.modal().onConfirm as () => void)()
    await new Promise(resolve => setImmediate(resolve)); ui.render()
    assert.equal(ui.button('取消任务').props.disabled, false)
    assert.match(ui.visibleText(), /执行中/)
    assert.doesNotMatch(ui.visibleText(), /等待停车确认|已取消/)
  }
})

test('abnormal task reasons are visible before details while normal execution stays compact', () => {
  for (const state of ['FAILED', 'RECOVERING', 'PAUSED']) {
    const ui = card(state)
    ui.feedback.snapshot.status.reason = 'local_blocked'
    ui.render({ resumeAllowed: false, resumeBlockedReason: state === 'PAUSED' ? '定位尚未恢复' : '' })
    assert.match(ui.visibleText(), state === 'PAUSED' ? /定位尚未恢复/ : /局部路线受阻。/)
    ui.feedback.stale = true; ui.render({ resumeAllowed: false })
    assert.doesNotMatch(ui.visibleText(), /局部路线受阻。|定位尚未恢复/)
  }
  for (const [state, reason] of [['EXECUTING', 'path_active'], ['RECOVERING', 'replan_waiting'], ['PAUSED', 'operator_pause']]) {
    const ui = card(state)
    ui.feedback.snapshot.status.reason = reason; ui.render()
    assert.doesNotMatch(ui.visibleText(), /任务已由操作者暂停。|Navigation status|导航状态/)
  }
})
