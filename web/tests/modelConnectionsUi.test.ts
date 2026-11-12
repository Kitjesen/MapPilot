import assert from 'node:assert/strict'
import test from 'node:test'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/ModelConnections.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX, target: ts.ScriptTarget.ES2022 },
}).outputText
type View = { type: unknown; props: Record<string, any> }
const walk = (node: any): View[] => Array.isArray(node) ? node.flatMap(walk) : node?.props ? [node, ...walk(node.props.children)] : []
function render(states: unknown[], api: Record<string, unknown> = {}, onClose = () => {}, clipboard?: { writeText(value: string): Promise<void> }, updates: Array<[number, unknown]> = [], options?: unknown) {
  let index = 0
  const exports: Record<string, (props: unknown) => View> = {}
  runInNewContext(source, { exports, navigator: { clipboard }, require: (name: string) => {
    if (name === 'react') return { useState: (initial: unknown) => { const slot = index++; return [states[slot] ?? initial, (value: unknown) => updates.push([slot, value])] }, useEffect() {}, useRef: (current: unknown) => ({ current }) }
    if (name.endsWith('/Modal')) return { ModalShell: (props: Record<string, unknown>) => require('react').createElement('dialog', { open: true }, props.children) }
    if (name.endsWith('modelConnections.ts')) return api
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  return exports.ModelConnections({ onClose, onChanged() {}, options })
}
test('opening model settings does not initiate authorization', () => {
  let starts = 0
  const tree = render([{ connected: false }], { startCodexLogin: async () => { starts++ } })
  assert.equal(starts, 0)
  assert.ok(walk(tree).some(node => node.type === 'button' && node.props.children === '登录'))
})
test('closing a pending login cancels that exact authorization before closing', async () => {
  const events: string[] = []
  const login = { id: 'login-id', state: 'pending', user_code: 'TEST-CODE', expires_at: 1000, interval: 5, verification_url: 'https://auth.openai.com/codex/device' }
  const tree = render([{ connected: false }, login], {
    cancelCodexLogin: async (id: string) => { events.push(id); return { ...login, state: 'cancelled' } },
  }, () => events.push('closed'))
  tree.props.onClose()
  await new Promise(resolve => setImmediate(resolve))
  assert.deepEqual(events, ['login-id', 'closed'])
  assert.equal(walk(tree).find(node => node.type === 'a')?.props.href, login.verification_url)
})
test('connected account displays disconnect without claiming inference success', () => {
  const tree = render([{ connected: true }])
  const markup = require('react-dom/server').renderToStaticMarkup(tree)
  assert.match(markup, /模型可用性以实际请求为准/)
  assert.match(markup, /断开/)
  assert.doesNotMatch(markup, /连接正常|额度充足/)
})

test('manual login copies the official URL and device code separately without starting another login', async () => {
  const copied: string[] = []
  const login = { id: 'active', state: 'pending', user_code: 'TEST-CODE', expires_at: 1000, interval: 5, verification_url: 'https://auth.openai.com/codex/device' }
  const tree = render([{ connected: false }, login], {}, () => {}, { writeText: async value => { copied.push(value) } })
  const nodes = walk(tree)
  nodes.find(node => node.props['aria-label'] === '复制登录链接')!.props.onClick()
  nodes.find(node => node.props['aria-label'] === '复制验证码')!.props.onClick()
  await new Promise(resolve => setImmediate(resolve))
  assert.deepEqual(copied, [login.verification_url, login.user_code])
  const markup = require('react-dom/server').renderToStaticMarkup(tree)
  assert.match(markup, /常用浏览器/)
  assert.match(markup, /https:\/\/auth.openai.com\/codex\/device/)
})

test('clipboard denial leaves authorization polling error state untouched', async () => {
  const updates: Array<[number, unknown]> = []
  const login = { id: 'active', state: 'pending', user_code: 'TEST-CODE', expires_at: 1000, interval: 5, verification_url: 'https://auth.openai.com/codex/device' }
  const tree = render([{ connected: false }, login], {}, () => {}, { writeText: async () => { throw new Error('denied') } }, updates)
  walk(tree).find(node => node.props['aria-label'] === '复制登录链接')!.props.onClick()
  await new Promise(resolve => setImmediate(resolve))
  assert.ok(updates.some(([slot, value]) => slot === 5 && String(value).includes('手动复制')))
  assert.ok(updates.every(([slot]) => slot !== 3))
})

test('pending authorization is not presented as saved or connected', () => {
  const login = { id: 'active', state: 'pending', user_code: 'TEST-CODE', expires_at: 1000, interval: 5, verification_url: 'https://auth.openai.com/codex/device' }
  const markup = require('react-dom/server').renderToStaticMarkup(render([{ connected: false }, login]))
  assert.match(markup, /等待官方授权/)
  assert.doesNotMatch(markup, /授权已保存|已授权/)
})

const options = { models: [
  { id: 'kimi', provider: 'moonshot', label: 'Kimi', configured: true },
  { id: 'claude', provider: 'claude', label: 'Claude', key_env: 'ANTHROPIC_API_KEY', configured: false },
  { id: 'gpt', provider: 'openai', label: 'GPT', key_env: 'OPENAI_API_KEY', configured: false },
  { id: 'qwen', provider: 'qwen', label: 'Qwen', key_env: 'DASHSCOPE_API_KEY', configured: false },
] }

test('provider tabs show only the selected connection panel', () => {
  const tree = render([{ connected: false }, null, false, '', '', '', 'claude'], {}, () => {}, undefined, [], options)
  const nodes = walk(tree)
  assert.equal(nodes.filter(node => node.props.role === 'tab').length, 5)
  assert.equal(nodes.filter(node => node.props.role === 'tabpanel').length, 1)
  const panel = require('react-dom/server').renderToStaticMarkup(nodes.find(node => node.props.role === 'tabpanel'))
  assert.match(panel, /Claude API/)
  assert.match(panel, /ANTHROPIC_API_KEY/)
  assert.doesNotMatch(panel, /ChatGPT 账号|OPENAI_API_KEY|DASHSCOPE_API_KEY/)
})

test('switching the panel preserves pending authorization and supports arrow navigation', () => {
  let cancelled = false
  const updates: Array<[number, unknown]> = []
  const login = { id: 'active', state: 'pending', user_code: 'TEST-CODE', expires_at: 1000, interval: 5, verification_url: 'https://auth.openai.com/codex/device' }
  const tree = render([{ connected: false }, login], { cancelCodexLogin: () => { cancelled = true } }, () => {}, undefined, updates, options)
  const nodes = walk(tree)
  nodes.find(node => node.props.id === 'connection-tab-claude')!.props.onClick()
  assert.deepEqual(updates.pop(), [6, 'claude'])
  assert.equal(cancelled, false)
  let focused = -1
  const buttons = Array.from({ length: 5 }, (_, i) => ({ focus: () => { focused = i } }))
  nodes.find(node => node.props.role === 'tablist')!.props.onKeyDown({ key: 'ArrowRight', preventDefault() {}, currentTarget: { querySelectorAll: () => buttons } })
  assert.deepEqual(updates.pop(), [6, 'moonshot'])
  assert.equal(focused, 1)
})
