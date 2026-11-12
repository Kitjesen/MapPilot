import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'

const source = ts.createSourceFile('App.tsx', readFileSync(new URL('../src/App.tsx', import.meta.url), 'utf8'), ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
const callbacks: Record<string, string> = {}
function visit(node: ts.Node) {
  if (ts.isVariableDeclaration(node) && ts.isIdentifier(node.name) && node.initializer
    && ts.isCallExpression(node.initializer) && node.initializer.expression.getText(source) === 'useCallback') {
    callbacks[node.name.text] = node.initializer.arguments[0].getText(source)
  }
  ts.forEachChild(node, visit)
}
visit(source)

test('standby requires a confirmed stop and ends as soon as startup begins', () => {
  let expression = ''
  function find(node: ts.Node) {
    if (ts.isVariableDeclaration(node) && node.name.getText(source) === 'standby') {
      expression = node.initializer!.getText(source)
    }
    ts.forEachChild(node, find)
  }
  find(source)
  assert.ok(expression)
  for (const [available, status, busy, expected] of [
    [true, 'stopped', false, true],
    [true, 'stopped', true, false],
    [false, 'stopped', false, false],
    [true, 'active', false, false],
    [false, undefined, false, false],
  ]) {
    const result = runInNewContext(expression, {
      productControl: { available, current: status ? { status } : null, busy },
    })
    assert.equal(result, expected)
  }
})

function callback(name: string, context: Record<string, unknown>) {
  return runInNewContext(ts.transpileModule(`(${callbacks[name]})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022 },
  }).outputText, context) as (value: string | null, restoring?: boolean) => void
}

test('business switching only selects a page and closes manual input', () => {
  for (const tab of ['mapping', 'scene']) {
    const changes: unknown[][] = []
    callback('handleTabChange', {
      activeTab: 'scene', leaveMapEditor: (next: () => void) => next(),
      setTeleopOpen: (value: boolean) => changes.push(['teleop', value]),
      setSelectedSavedMap: (value: string | null) => changes.push(['map', value]),
      setActiveTab: (value: string) => changes.push(['page', value]),
    })(tab)
    assert.deepEqual(changes, [['map', null], ['page', tab]])
  }
})

test('map management remembers its business parent and returns after a confirmed Product switch', () => {
  const changes: unknown[][] = []
  callback('handleOpenSavedMap', {
    activeTab: 'mapping', leaveMapEditor: (next: () => void) => next(),
    setMapReturnTab: (value: string) => changes.push(['return', value]),
    setSelectedSavedMap: (value: string | null) => changes.push(['map', value]),
    setActiveTab: (value: string) => changes.push(['page', value]),
  })(null)
  assert.deepEqual(changes, [['return', 'mapping'], ['map', null], ['page', 'map']])
  for (const product of ['map', 'nav']) {
    changes.length = 0
    callback('handleProductChanged', {
      setTeleopOpen: (value: boolean) => changes.push(['teleop', value]),
      setActiveTab: (value: string | ((current: string) => string)) => changes.push([
        'page', typeof value === 'function' ? value('map') : value,
      ]),
    })(product, false)
    assert.deepEqual(changes, [['page', product === 'map' ? 'mapping' : 'scene']])
  }
})
