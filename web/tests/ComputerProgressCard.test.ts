import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/ComputerProgressCard.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText

function exportsObject() {
  const exports: Record<string, unknown> = {}
  runInNewContext(source, { exports, require: (name: string) => name.endsWith('.css')
    ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(name) })
  return exports
}

test('computer progress formatting and state controls are explicit', () => {
  const module = exportsObject() as { ComputerProgressCard: (props: Record<string, unknown>) => unknown }
  const element = module.ComputerProgressCard({ state: 'awaiting_permission', action: '等待浏览器确认', elapsedMs: 72_000, onRequestPermission() {} }) as { props: { children: unknown } }
  const html = require('react-dom/server').renderToStaticMarkup(element)
  assert.match(html, /等待电脑权限/)
  assert.match(html, /等待浏览器确认/)
  assert.match(html, /请求权限/)
  assert.match(html, /1 分 12 秒/)
})
