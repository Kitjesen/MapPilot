import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const source = readFileSync(new URL('../src/components/MapEditor.tsx', import.meta.url), 'utf8')
const file = ts.createSourceFile('MapEditor.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
const expressions = new Map<string, ts.Expression>()
const visit = (node: ts.Node) => {
  if (ts.isVariableDeclaration(node) && node.initializer) expressions.set(node.name.getText(file), node.initializer)
  ts.forEachChild(node, visit)
}
visit(file)

function evaluate(name: string, context: Record<string, unknown>) {
  const expression = expressions.get(name)
  assert.ok(expression, name)
  return runInNewContext(ts.transpileModule(`(${expression.getText(file)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, context)
}

test('selection panel stays inside its editor and follows a smaller viewport', () => {
  const context = { viewport: { width: 1000, bottom: 688 }, selectionSize: { width: 280, height: 300 } }
  const clamp = evaluate('clampSelectionPosition', context) as (left: number, top: number) => { left: number; top: number }
  assert.deepEqual({ ...clamp(9000, 9000) }, { left: 708, top: 388 })
  assert.deepEqual({ ...clamp(-100, -100) }, { left: 12, top: 12 })
  context.viewport = { width: 320, bottom: 316 }
  assert.deepEqual({ ...clamp(708, 388) }, { left: 28, top: 16 })
  context.selectionSize = { width: 280, height: 500 }
  assert.deepEqual({ ...clamp(708, 388) }, { left: 28, top: 12 })
})

test('header dragging captures the pointer, ignores action buttons and releases on cancellation', () => {
  type Position = { left: number; top: number }
  const positions: Position[] = []
  const dragging: boolean[] = []
  const captures: number[] = []
  const context: Record<string, unknown> = {
    viewport: { width: 1000, bottom: 688 }, selectionSize: { width: 280, height: 300 },
    position: { left: 200, top: 100 }, selectionDrag: { current: null },
    setSelectionPosition: (value: Position) => positions.push(value),
    setDraggingSelection: (value: boolean) => dragging.push(value),
  }
  context.clampSelectionPosition = evaluate('clampSelectionPosition', context)
  const begin = evaluate('beginSelectionDrag', context) as (event: unknown) => void
  const move = evaluate('moveSelectionDrag', context) as (event: unknown) => void
  const end = evaluate('endSelectionDrag', context) as () => void
  let prevented = 0
  const pointer = { button: 0, pointerId: 7, clientX: 220, clientY: 120,
    target: { closest: () => null }, currentTarget: { setPointerCapture: (id: number) => captures.push(id) },
    preventDefault: () => prevented++, stopPropagation() {} }
  begin({ ...pointer, target: { closest: () => ({ tagName: 'BUTTON' }) } })
  begin({ ...pointer, button: 2 })
  move({ clientX: 400, clientY: 300 })
  assert.equal(positions.length, 0)
  assert.equal(captures.length, 0)
  begin(pointer)
  move({ clientX: 320, clientY: 220 })
  assert.deepEqual({ ...positions.at(-1)! }, { left: 300, top: 200 })
  move({ clientX: 9000, clientY: 9000 })
  assert.deepEqual({ ...positions.at(-1)! }, { left: 708, top: 388 })
  assert.deepEqual(captures, [7])
  assert.equal(prevented, 1)
  end()
  move({ clientX: 0, clientY: 0 })
  assert.equal(positions.length, 2)
  assert.deepEqual(dragging, [true, false])
})
