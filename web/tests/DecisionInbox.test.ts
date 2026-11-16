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
