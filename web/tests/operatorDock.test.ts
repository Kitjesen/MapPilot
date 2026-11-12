import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const source = readFileSync(new URL('../src/components/Topbar.tsx', import.meta.url), 'utf8')
const css = readFileSync(new URL('../src/components/Topbar.module.css', import.meta.url), 'utf8')

test('operator dock opens explicitly from a draggable capsule and retains an independent stop', () => {
  assert.match(source, /data-dock-drag/)
  assert.match(source, /onPointerMove=\{moveDrag\}/)
  assert.match(source, /setCollapsed\(true\)/)
  assert.match(source, /const \[collapsed, setCollapsed\] = useState\(true\)/)
  assert.match(source, /className=\{styles\.emergencyStop\}/)
  assert.match(source, /loadLayouts\(\)/)
  assert.match(source, /saveLayouts\(/)
  assert.match(source, /setCollapsed\(false\)/)
  assert.doesNotMatch(source, /onPointerEnter|setHoverPreview/)
  assert.match(source, /handle\.setPointerCapture\?\.\(event\.pointerId\)/)
  assert.doesNotMatch(source, /event\.currentTarget\.setPointerCapture/)
  assert.match(source, /id="scene-actions-slot"/)
  assert.match(css, /\.collapsed \{[^}]*border-radius: var\(--r-full\)/)
  assert.match(css, /background: var\(--bg-layer-1\)/)
})

test('operator dock keeps the mark and collapses without switching the business workspace', () => {
  assert.match(source, /<Navigation size=\{24\} strokeWidth=\{1\.8\} aria-hidden="true" \/>/)
  assert.match(source, /className=\{styles.logo\} onClick=\{\(\) => setCollapsed\(value => !value\)\}/)
})

test('dock hover waits, cancels a pass, closes on exit and releases timers', t => {
  t.mock.timers.enable({ apis: ['setTimeout'] })
  const ast = ts.createSourceFile('Topbar.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
  let effect = ''
  function visit(node: ts.Node) {
    if (ts.isCallExpression(node) && node.expression.getText(ast) === 'useEffect'
      && node.arguments[0]?.getText(ast).includes("dock.addEventListener('pointerenter'")) effect = node.arguments[0].getText(ast)
    ts.forEachChild(node, visit)
  }
  visit(ast)
  assert.ok(effect)
  const handlers = new Map<string, (event: object) => void>()
  let collapsed = true
  const dock = {
    addEventListener: (key: string, handler: (event: object) => void) => handlers.set(key, handler),
    removeEventListener: (key: string) => handlers.delete(key),
    contains: () => false,
  }
  const exports: { cleanup?: () => void } = {}
  runInNewContext(ts.transpileModule(`exports.cleanup = (${effect})();`, { compilerOptions: { target: ts.ScriptTarget.ES2022 } }).outputText,
    { exports, dockRef: { current: dock }, dragRef: { current: null }, document: { activeElement: null }, setTimeout, clearTimeout,
      setCollapsed: (value: boolean) => { collapsed = value } })
  handlers.get('pointerenter')!({ pointerType: 'mouse' })
  t.mock.timers.tick(100)
  assert.equal(collapsed, true)
  handlers.get('pointerleave')!({ pointerType: 'mouse' })
  t.mock.timers.tick(250)
  assert.equal(collapsed, true)
  handlers.get('pointerenter')!({ pointerType: 'mouse' })
  t.mock.timers.tick(220)
  assert.equal(collapsed, false)
  handlers.get('pointerleave')!({ pointerType: 'mouse' })
  t.mock.timers.tick(100)
  handlers.get('pointerenter')!({ pointerType: 'mouse' })
  t.mock.timers.tick(180)
  assert.equal(collapsed, false)
  handlers.get('pointerleave')!({ pointerType: 'mouse' })
  t.mock.timers.tick(180)
  assert.equal(collapsed, true)
  handlers.get('pointerenter')!({ pointerType: 'touch' })
  t.mock.timers.tick(500)
  assert.equal(collapsed, true)
  handlers.get('pointerenter')!({ pointerType: 'mouse' })
  exports.cleanup!()
  t.mock.timers.tick(500)
  assert.equal(collapsed, true)
  assert.equal(handlers.size, 0)
})
