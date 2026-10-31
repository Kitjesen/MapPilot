import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as THREE from 'three'
import ts from 'typescript'
import { alignMapEditTopView, createMapEditBox, frameMapEditView, mapEditDragBounds, mapEditSelectionAnchor } from '../src/components/scene3d/mapEditing.ts'
import { applyMapEditOperations, pointInMapEditBounds } from '../src/services/mapEdit.ts'

const source = { min: { x: -5, y: -5, z: -1 }, max: { x: 5, y: 5, z: 8 } }
const viewport = { width: 100, height: 100 }
const rectangle = { x0: 40, y0: 40, x1: 60, y1: 60 }

function topCamera() {
  const camera = new THREE.OrthographicCamera(-5, 5, 5, -5, 0.1, 100)
  camera.position.set(0, 20, 0)
  camera.up.set(0, 0, -1)
  camera.lookAt(0, 0, 0)
  camera.updateMatrixWorld()
  return camera
}

test('top drag creates a volume with full original height, including hidden ground and points behind a wall', () => {
  const bounds = mapEditDragBounds(topCamera(), viewport, rectangle, source)!
  assert.ok(bounds)
  assert.equal(bounds.min.z, -1)
  assert.equal(bounds.max.z, 8)
  for (const z of [-1, 0, 3, 8]) assert.equal(pointInMapEditBounds(0, 0, z, bounds), true)
  assert.equal(pointInMapEditBounds(3, 0, 3, bounds), false)
  const cloud = [0, 0, -1, 0, 0, 3, 0, 0, 8, 3, 0, 3]
  assert.deepEqual(applyMapEditOperations(cloud, [{ operation: 'delete_inside', bounds }]), [3, 0, 3])
  assert.deepEqual(mapEditDragBounds(topCamera(), viewport, { x0: 60, y0: 60, x1: 40, y1: 40 }, source), bounds)
})

test('selection volume does not depend on perspective projection of objects at different heights', () => {
  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100)
  frameMapEditView(camera, new THREE.Vector3(), source, 'top')
  const bounds = mapEditDragBounds(camera, viewport, rectangle, source)!
  assert.ok(bounds.min.x < 0 && bounds.max.x > 0)
  assert.ok(bounds.min.y < 0 && bounds.max.y > 0)
  assert.equal(bounds.min.z, source.min.z)
  assert.equal(bounds.max.z, source.max.z)
  assert.equal(mapEditDragBounds(camera, viewport, { x0: 40, y0: 40, x1: 42, y1: 42 }, source), null)
})

test('selection box uses LingTu Z-up coordinates and view actions frame the selection', () => {
  const bounds = { min: { x: 1, y: 2, z: 3 }, max: { x: 4, y: 6, z: 8 } }
  const helper = createMapEditBox(bounds)
  assert.deepEqual(helper.box.min.toArray(), [1, 3, -6])
  assert.deepEqual(helper.box.max.toArray(), [4, 8, -2])
  const camera = new THREE.PerspectiveCamera(48, 2, 0.1, 500)
  const target = new THREE.Vector3()
  for (const view of ['top', 'front', 'side', 'three'] as const) {
    frameMapEditView(camera, target, bounds, view)
    assert.deepEqual(target.toArray(), [2.5, 5.5, -4])
    assert.ok(camera.position.distanceTo(target) > 1)
  }
  helper.geometry.dispose(); (helper.material as THREE.Material).dispose()
})

test('contextual tools follow camera projection and disappear for selections behind the camera', () => {
  const camera = topCamera()
  assert.deepEqual(mapEditSelectionAnchor(camera, viewport, source), { x: 50, y: 50 })
  camera.position.x += 2
  camera.updateMatrixWorld()
  assert.deepEqual(mapEditSelectionAnchor(camera, viewport, source), { x: 30, y: 50 })
  camera.lookAt(2, 100, 0)
  camera.updateMatrixWorld()
  assert.equal(mapEditSelectionAnchor(camera, viewport, source), null)
})

const sceneSource = readFileSync(new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8')
const sceneFile = ts.createSourceFile('Scene3D.tsx', sceneSource, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
function handler(name: string, context: Record<string, unknown>): (event: unknown) => void {
  let expression: ts.Expression | undefined
  const visit = (node: ts.Node) => {
    if (ts.isVariableDeclaration(node) && node.name.getText(sceneFile) === name) expression = node.initializer
    ts.forEachChild(node, visit)
  }
  visit(sceneFile)
  assert.ok(expression)
  const code = ts.transpileModule(`(${expression.getText(sceneFile)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  return runInNewContext(code, context)
}

function effectContaining(fragment: string, context: Record<string, unknown>): () => void {
  let expression: ts.Node | undefined
  const visit = (node: ts.Node) => {
    if (ts.isCallExpression(node) && node.expression.getText(sceneFile) === 'useEffect'
      && node.arguments[0]?.getText(sceneFile).includes(fragment)) expression = node.arguments[0]
    ts.forEachChild(node, visit)
  }
  visit(sceneFile)
  assert.ok(expression)
  const code = ts.transpileModule(`(${expression.getText(sceneFile)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  return runInNewContext(code, context)
}

function interaction() {
  const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 500)
  const target = new THREE.Vector3()
  frameMapEditView(camera, target, source, 'top')
  let captured = false
  const selected: unknown[] = []
  const context = {
    THREE, alignMapEditTopView, mapEditDragBounds, mapEdit: { selecting: true }, mapEditSourceBounds: source, stairTool: null,
    editingMap: true,
    mapEditPointerRef: { current: null }, mouseDownPos: { current: null },
    stairSelectionRef: { current: { style: {} } },
    rendererRef: { current: { domElement: { getBoundingClientRect: () => ({ ...viewport, left: 0, top: 0 }) } } },
    cameraRef: { current: camera }, controlsRef: { current: {
      target, enabled: true, enableZoom: true, enableRotate: true, enablePan: true, enableDamping: true,
    } },
    onMapEditBounds: (bounds: unknown) => selected.push(bounds),
    onPendingGoal: () => assert.fail('editing must not propose navigation'),
    onRelocalize: () => assert.fail('editing must not relocalize'),
    clearMapEditPointer: (_event: unknown) => { void _event }, handleMapEditPointerMove: (_event: unknown) => { void _event },
  }
  context.clearMapEditPointer = handler('clearMapEditPointer', context)
  context.handleMapEditPointerMove = handler('handleMapEditPointerMove', context)
  const event = (x: number, y: number) => ({
    clientX: x, clientY: y, pointerId: 1, button: 0, shiftKey: true, altKey: true,
    stopPropagation() {}, preventDefault() {}, currentTarget: {
      setPointerCapture() { captured = true }, hasPointerCapture() { return captured },
      releasePointerCapture() { captured = false },
    },
  })
  return { context, event, captured: () => captured, selected }
}

test('drag captures and releases orbit; cancel creates neither an edit nor a navigation request', () => {
  const { context, event, captured, selected } = interaction()
  handler('handleMapEditPointerDown', context)(event(40, 40))
  assert.equal(captured(), true)
  assert.equal(context.controlsRef.current.enabled, false)
  context.handleMapEditPointerMove(event(60, 60))
  context.clearMapEditPointer(event(60, 60))
  assert.equal(captured(), false)
  assert.equal(context.controlsRef.current.enabled, true)
  handler('handleMapEditPointerUp', context)(event(60, 60))
  handler('handleMouseDown', context)(event(60, 60))
  handler('handleMouseUp', context)(event(60, 60))
  assert.equal(selected.length, 0)
  context.mapEdit.selecting = false
  handler('handleMouseDown', context)(event(60, 60))
  handler('handleMouseUp', context)(event(60, 60))
})

test('selection mode keeps wheel zoom enabled between drags, blocks orbit, and restores orbit on exit', () => {
  const { context, event, captured } = interaction()
  const configure = effectContaining('controls.enableRotate', context)
  configure()
  const controls = context.controlsRef.current
  assert.equal(controls.enabled, true)
  assert.equal(controls.enableZoom, true)
  assert.equal(controls.enableRotate, false)
  assert.equal(controls.enablePan, false)
  let stopped = false
  const rightClick = { ...event(40, 40), button: 2, stopPropagation() { stopped = true } }
  handler('handleMapEditPointerDown', context)(rightClick)
  assert.equal(stopped, true)
  assert.equal(captured(), false)
  assert.equal(controls.enabled, true)
  handler('handleMapEditPointerDown', context)(event(40, 40))
  assert.equal(controls.enabled, false)
  handler('handleMapEditPointerUp', context)(event(60, 60))
  assert.equal(controls.enabled, true)
  assert.equal(controls.enableZoom, true)
  context.mapEdit.selecting = false
  configure()
  assert.equal(controls.enableRotate, true)
  assert.equal(controls.enablePan, true)
  assert.equal(controls.enableDamping, true)
})

test('entering selection preserves current target and zoom, including after zooming between selections', () => {
  const camera = new THREE.PerspectiveCamera(48, 1, 0.1, 500)
  const target = new THREE.Vector3(12, 3, -7)
  camera.position.copy(target).add(new THREE.Vector3(3, 4, 0))
  let flush = true
  const controls = { target, update() { if (flush) { target.addScalar(1); flush = false } } }
  const context = {
    alignMapEditTopView, mapEditRef: { current: { selecting: true } },
    cameraRef: { current: camera }, controlsRef: { current: controls },
  }
  const enter = effectContaining('Flush pending orbit damping', context)
  enter()
  assert.deepEqual(target.toArray(), [12, 3, -7])
  assert.ok(Math.abs(camera.position.distanceTo(target) - 5) < 1e-9)
  assert.ok(Math.abs(camera.position.x - target.x) < 1e-9)
  camera.position.copy(target).add(new THREE.Vector3(0, 2, 0))
  enter()
  assert.deepEqual(target.toArray(), [12, 3, -7])
  assert.ok(Math.abs(camera.position.distanceTo(target) - 2) < 1e-9)
})

test('completed drag returns full-height bounds and restores orbit without touching camera or source cloud', () => {
  const { context, event, selected } = interaction()
  handler('handleMapEditPointerDown', context)(event(40, 40))
  const before = context.cameraRef.current.position.clone()
  handler('handleMapEditPointerUp', context)(event(60, 60))
  assert.equal(selected.length, 1)
  assert.equal((selected[0] as typeof source).min.z, source.min.z)
  assert.equal((selected[0] as typeof source).max.z, source.max.z)
  assert.equal(context.controlsRef.current.enabled, true)
  assert.ok(context.cameraRef.current.position.equals(before))
  assert.deepEqual(source, { min: { x: -5, y: -5, z: -1 }, max: { x: 5, y: 5, z: 8 } })
})
