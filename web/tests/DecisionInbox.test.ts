import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/DecisionInbox.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText

test('decision inbox renders pending actions and the clear state', () => {
  const exports: Record<string, unknown> = {}
  runInNewContext(source, { exports, require: (name: string) => name.endsWith('.css')
    ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(name) })
  const react = require('react')
  const render = require('react-dom/server').renderToStaticMarkup
  const inbox = exports.DecisionInbox as (props: Record<string, unknown>) => unknown
  const pending = render(react.createElement(inbox, { decisions: [{ id: 'a', title: '允许分析？', primary: '允许执行', secondary: '拒绝' }] }))
  assert.match(pending, /决策收件箱/)
  assert.match(pending, /允许分析？/)
  assert.match(pending, /允许执行/)
})

test('a decision added after mount stays open until its answer is shown', () => {
  const exports: Record<string, unknown> = {}
  const react = require('react')
  const render = require('react-dom/server').renderToStaticMarkup
  let state: Record<string, unknown> | undefined
  const timers: Array<() => void> = []
  runInNewContext(source, { exports, window: { setTimeout: (callback: () => void) => timers.push(callback) },
    require: (name: string) => name === 'react' ? { ...react, useState: (initial: () => Record<string, unknown>) => {
      state ??= initial()
      return [state, (update: (current: Record<string, unknown>) => Record<string, unknown>) => { state = update(state!) }]
    } } : name.endsWith('.css') ? { default: {} } : require(name) })
  const inbox = exports.DecisionInbox as (props: Record<string, unknown>) => unknown
  inbox({ decisions: [{ id: 'old', title: '旧决策', primary: '允许执行', secondary: '拒绝' }] })
  const decisions = [{ id: 'new', title: '新决策', primary: '允许执行', secondary: '拒绝' }]
  const answers: string[] = []
  const draw = () => inbox({ decisions, onDecide: (decision: { id: string }, answer: string) => answers.push(`${decision.id}:${answer}`) })
  type Element = { type?: string; props?: { children?: unknown; onClick?: () => void } }
  function elements(node: unknown): Element[] {
    if (Array.isArray(node)) return node.flatMap(elements)
    if (!node || typeof node !== 'object') return []
    const element = node as Element
    return [element, ...elements(element.props?.children)]
  }
  const action = elements(draw()).find(element => element.type === 'button' && element.props?.children === '允许执行')
  assert.ok(action)
  action.props?.onClick?.()
  assert.deepEqual(answers, ['new:primary'])
  assert.match(render(draw()), /role="status"[^>]*>[\s\S]*允许执行/)
  timers[0]()
  assert.match(render(draw()), /data-gone="true"/)
})
