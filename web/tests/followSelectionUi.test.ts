import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import type { FollowSelection } from '../src/services/api.ts'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/FollowSelectionCard.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
type Element = { type: string; props: Record<string, any> }
function elements(node: unknown): Element[] {
  if (Array.isArray(node)) return node.flatMap(elements)
  if (!node || typeof node !== 'object') return []
  const element = node as Element
  return [element, ...elements(element.props?.children)]
}
function setup(view: FollowSelection) {
  const state: unknown[] = [view]
  let cursor = 0
  const exports: Record<string, any> = {}
  runInNewContext(source, { exports, require: (name: string) => {
    if (name === 'react') return { ...require('react'), useEffect() {}, useRef: () => ({ current: null }),
      useState(initial: unknown) {
        const index = cursor++
        if (!(index in state)) state[index] = initial
        return [state[index], (value: unknown) => { state[index] = typeof value === 'function' ? value(state[index]) : value }]
      },
    }
    if (name.endsWith('/services/api')) return {}
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  return () => { cursor = 0; return elements(exports.FollowSelectionCard({ task: { follow_task_id: 'follow', product_session_id: 'session' } })) }
}
const view: FollowSelection = { found: true, confirmation_required: true, state: 'running', review_state: 'ready',
  image: '/frame.jpg', width: 100, height: 100, observation_sequence: 8,
  candidates: [
    { target_id: 'poster', bbox: [10, 10, 20, 20], kind: 'non_person', reason: '广告人物' },
    { target_id: 'person', bbox: [30, 20, 50, 80], kind: 'person' },
    { target_id: 'uncertain', bbox: [60, 20, 70, 60], kind: 'unknown' },
  ],
}
test('comparison preserves frame and numbering, and only verified retained people can be confirmed', () => {
  const render = setup(view)
  let nodes = render()
  const find = (label: string) => nodes.find(node => node.props?.['aria-label'] === label)!
  const confirm = () => nodes.find(node => node.type === 'button' && node.props.children === '确认跟踪')!
  assert.equal(find('选择人员 1'), undefined)
  assert.equal(find('选择人员 3').props.disabled, true)
  assert.equal(confirm().props.disabled, true)
  find('选择人员 2').props.onClick()
  nodes = render()
  assert.equal(confirm().props.disabled, false)
  const selectedBox = find('选择人员 2').props.style
  nodes.find(node => node.type === 'button' && node.props.children?.[0] === '原始候选 ')!.props.onClick()
  nodes = render()
  assert.equal(find('原始候选 1').props.disabled, true)
  assert.equal(find('原始候选 2').props.disabled, true)
  assert.equal(confirm().props.disabled, true)
  assert.deepEqual(find('原始候选 2').props.style, selectedBox)
  assert.equal(nodes.find(node => node.type === 'img')!.props.src, '/frame.jpg')
  nodes.find(node => node.type === 'button' && node.props.children?.[0] === '过滤结果 ')!.props.onClick()
  nodes = render()
  assert.equal(find('选择人员 1'), undefined)
  assert.equal(find('选择人员 2').props['aria-pressed'], true)
  assert.equal(confirm().props.disabled, false)
})
test('unavailable review and a terminal task cannot authorize a candidate', () => {
  for (const changed of [{ review_state: 'unavailable' as const }, { terminal: true }]) {
    const nodes = setup({ ...view, ...changed })()
    const choices = nodes.filter(node => node.type === 'button' && String(node.props['aria-label']).startsWith('选择人员'))
    assert.ok(choices.every(node => node.props.disabled))
    const confirm = nodes.find(node => node.type === 'button' && node.props.children === '确认跟踪')
    assert.ok(!confirm || confirm.props.disabled)
  }
})
