import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import { projectPcdPoint, type PointCloudPick } from '../src/services/pcdPicking.ts'

const source = readFileSync(new URL('../src/components/PointCloudViewer.tsx', import.meta.url), 'utf8')
const file = ts.createSourceFile('PointCloudViewer.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
const matrixNames = ['perspective', 'orthographic', 'lookAt', 'mulM', 'mvpFor']
const matrices = file.statements.filter(node => ts.isFunctionDeclaration(node) && matrixNames.includes(node.name?.text ?? ''))
assert.equal(matrices.length, matrixNames.length)
let onMove: ts.Expression | undefined
function visit(node: ts.Node) {
  if (ts.isVariableDeclaration(node) && node.name.getText(file) === 'onMove') onMove = node.initializer
  ts.forEachChild(node, visit)
}
visit(file)
assert.ok(onMove)
const code = ts.transpileModule(`${matrices.map(node => node.getText(file)).join('\n')}
({ mvpFor, move: ${onMove.getText(file)} })`, {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
}).outputText

function interaction(mode: '2d' | '3d' = '3d', theta = .4, phi = 1.05) {
  const scene = { mode, theta, phi, center: [1, .5, -2], radius: 2, dist: 10, fitted: true,
    gl: { canvas: { width: 800, height: 600, clientHeight: 600 } } }
  const drag = { x: 100, y: 100, startX: 100, startY: 100, moved: false }
  let draws = 0
  const context = { glRef: { current: scene as typeof scene | null }, dragRef: { current: drag as typeof drag | null }, draw: () => draws++ }
  const actual = runInNewContext(code, context) as {
    mvpFor: (state: typeof scene) => Float32Array
    move: (event: { clientX: number; clientY: number }) => void
  }
  return { scene, drag, context, actual, draws: () => draws,
    project(point: PointCloudPick) {
      const projected = projectPcdPoint(actual.mvpFor(scene), point, 800, 600)
      assert.ok(projected, 'the test point must stay visible')
      return projected
    } }
}

test('3D dragging moves near-side geometry with the pointer in all four directions', () => {
  for (const theta of [-2.2, .4, 2.1]) for (const phi of [.35, 1.05, 2.5]) {
    for (const distance of [.6, 1.2]) for (const [dx, dy] of [[20, 0], [-20, 0], [0, 20], [0, -20]]) {
      const view = interaction('3d', theta, phi)
      // Choose geometry on the camera-facing hemisphere in the renderer's (x,z,-y) frame.
      const point = { x: 1 + distance * Math.sin(phi) * Math.cos(theta),
        y: -(-2 + distance * Math.sin(phi) * Math.sin(theta)), z: .5 + distance * Math.cos(phi) }
      const before = view.project(point)
      view.actual.move({ clientX: 100 + dx, clientY: 100 + dy })
      const after = view.project(point)
      assert.ok((dx || dy) * (dx ? after.x - before.x : after.y - before.y) > 0,
        `projected drag must follow pointer at theta=${theta}, phi=${phi}, delta=${dx},${dy}`)
      assert.equal(view.drag.moved, true)
      assert.equal(view.draws(), 1)
    }
  }
})

test('vertical orbit stays inside both polar limits', () => {
  const view = interaction()
  view.actual.move({ clientX: 100, clientY: 10100 })
  assert.equal(view.scene.phi, .05)
  view.actual.move({ clientX: 100, clientY: -9900 })
  assert.equal(view.scene.phi, Math.PI - .05)
  assert.equal(view.draws(), 2)
})

test('2D panning still moves visible geometry by the pointer displacement', () => {
  for (const [dx, dy] of [[20, 0], [-20, 0], [0, 20], [0, -20]]) {
    const view = interaction('2d', -Math.PI / 2, 0)
    const point = { x: 1, y: 2, z: .5 }
    const before = view.project(point)
    view.actual.move({ clientX: 100 + dx, clientY: 100 + dy })
    const after = view.project(point)
    assert.ok(Math.abs(after.x - before.x - dx) < .001)
    assert.ok(Math.abs(after.y - before.y - dy) < .001)
    assert.equal(view.scene.fitted, false)
    assert.equal(view.scene.theta, -Math.PI / 2)
    assert.equal(view.scene.phi, 0)
    assert.equal(view.draws(), 1)
  }
})

test('the 3px click threshold uses total displacement and stays moved after returning', () => {
  const view = interaction()
  for (const [x, y, moved] of [[103, 100, false], [102, 102, false], [103, 101, true], [100, 100, true]] as const) {
    view.actual.move({ clientX: x, clientY: y })
    assert.equal(view.drag.moved, moved)
    assert.equal(view.drag.x, x)
    assert.equal(view.drag.y, y)
  }
  assert.equal(view.draws(), 4)
})

test('moving without an active scene or drag does not draw', () => {
  const view = interaction()
  view.context.dragRef.current = null
  view.actual.move({ clientX: 120, clientY: 100 })
  view.context.dragRef.current = view.drag
  view.context.glRef.current = null
  view.actual.move({ clientX: 120, clientY: 100 })
  assert.equal(view.draws(), 0)
})
