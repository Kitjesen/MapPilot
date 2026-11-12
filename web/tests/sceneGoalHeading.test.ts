import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { pickGoalHeading, pickNavigationPoint } from '../src/components/scene3d/goalPicking.ts'
import { lingtuToThree, threeToLingtu } from '../src/services/coordinateFrame.ts'

const source = readFileSync(new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8')
const file = ts.createSourceFile('Scene3D.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
function expression(predicate: (node: ts.Node) => ts.Node | undefined, context: Record<string, unknown>) {
  let found: ts.Node | undefined
  const visit = (node: ts.Node) => { found ??= predicate(node); ts.forEachChild(node, visit) }
  visit(file)
  assert.ok(found)
  return runInNewContext(ts.transpileModule(`(${found.getText(file)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, context)
}
function handler(name: string, context: Record<string, unknown>) {
  return expression(node => {
    if (!ts.isVariableDeclaration(node) || node.name.getText(file) !== name) return
    return node.initializer && ts.isCallExpression(node.initializer) && node.initializer.expression.getText(file) === 'useCallback'
      ? node.initializer.arguments[0] : node.initializer
  }, context)
}

test('navigation keeps rotation and pan available while left drag selects a goal', () => {
  const start = source.indexOf('const controls = new OrbitControls(')
  const end = source.indexOf('controlsRef.current = controls', start)
  const controlsRef = { current: null as OrbitControls | null }
  runInNewContext(source.slice(start, end) + 'controlsRef.current = controls', {
    THREE, OrbitControls, controlsRef, camera: new THREE.PerspectiveCamera(), renderer: { domElement: null },
  })
  assert.equal(controlsRef.current!.mouseButtons.RIGHT, THREE.MOUSE.ROTATE)
  assert.equal(controlsRef.current!.mouseButtons.MIDDLE, THREE.MOUSE.PAN)
  assert.equal(controlsRef.current!.enableZoom, true)
  for (const button of [1, 2]) {
    const { context, event, captured } = interaction()
    handler('handleGoalHeadingPointerDown', context)({ ...event(), button })
    assert.equal(captured(), false)
    assert.equal(context.controlsRef.current.enabled, true)
  }
})
function interaction() {
  const camera = new THREE.PerspectiveCamera(48, 1, .1, 100)
  camera.position.set(0, 10, 0)
  camera.up.set(0, 0, -1)
  camera.lookAt(0, 0, 0)
  camera.updateMatrixWorld()
  const headings: Array<number | undefined> = []
  const selected: Array<{ x: number; y: number; z?: number }> = []
  let captured = false, completed = 0
  const context = {
    THREE, pickGoalHeading, pickNavigationPoint, threeToLingtu, goalGestureEnabled: true, goalHeadingEditing: true,
    pendingGoal: { x: 0, y: 0, z: .4 } as { x: number; y: number; z?: number } | null, pendingGoalYaw: .7 as number | undefined,
    goalPickMode: 'plane', mappingObservationVisible: false, robotZ: .4,
    savedMapRef: { current: null }, liveCloudRef: { current: null }, scanCloudRef: { current: null }, octomapRef: { current: null },
    mapEdit: null as unknown, stairTool: null as unknown,
    goalHeadingPointerRef: { current: null }, mouseDownPos: { current: null },
    raycaster: { current: new THREE.Raycaster() }, cameraRef: { current: camera },
    rendererRef: { current: { domElement: { getBoundingClientRect: () => ({ left: 0, top: 0, width: 100, height: 100 }) } } },
    controlsRef: { current: { enabled: true } },
    onGoalHeadingChange: (yaw: number | undefined) => headings.push(yaw),
    onGoalHeadingComplete: () => completed++,
    onPendingGoal: (x: number, y: number, z?: number) => ({ x, y, z }),
    onRelocalize: () => assert.fail('heading must not relocalize'),
    handleGoalHeadingPointerMove: (_event: unknown) => { void _event },
    clearGoalHeadingPointer() {}, navigationPointAt: () => null as THREE.Vector3 | null,
  }
  context.onPendingGoal = (x, y, z) => { const goal = { x, y, z }; selected.push(goal); context.pendingGoal = goal; return goal }
  context.handleGoalHeadingPointerMove = handler('handleGoalHeadingPointerMove', context)
  context.clearGoalHeadingPointer = handler('clearGoalHeadingPointer', context)
  context.navigationPointAt = handler('navigationPointAt', context)
  const element = {
    setPointerCapture() { captured = true }, hasPointerCapture() { return captured },
    releasePointerCapture() { captured = false },
  }
  const event = (x = 75, y = 50) => ({ clientX: x, clientY: y, pointerId: 1, button: 0,
    shiftKey: false, altKey: false, currentTarget: element, preventDefault() {}, stopPropagation() {} })
  return { context, event, headings, selected, captured: () => captured, completed: () => completed }
}

test('ordinary point clicks select the clicked position without capturing a heading gesture', () => {
  const { context, event, captured, headings, selected } = interaction()
  context.goalGestureEnabled = false
  context.goalHeadingEditing = false
  context.pendingGoal = null
  context.pendingGoalYaw = undefined
  handler('handleGoalHeadingPointerDown', context)(event(50, 50))
  assert.equal(captured(), false)
  handler('handleMouseDown', context)(event(50, 50))
  handler('handleMouseUp', context)(event(50, 50))
  assert.equal(selected.length, 1)
  assert.ok(Math.hypot(selected[0].x, selected[0].y, selected[0].z! - .4) < 1e-10)
  assert.equal(headings.length, 0)
  handler('handleMouseDown', context)(event(50, 50))
  handler('handleMouseUp', context)(event(75, 50))
  assert.equal(selected.length, 1, 'camera dragging must not create another goal')
})

test('an optional first heading edit keeps the existing point even before a yaw exists', () => {
  const { context, event, selected, headings, completed } = interaction()
  context.pendingGoalYaw = undefined
  const original = context.pendingGoal
  handler('handleGoalHeadingPointerDown', context)(event(75, 50))
  handler('finishGoalHeading', context)(event(50, 25), false)
  assert.equal(selected.length, 0)
  assert.equal(context.pendingGoal, original)
  assert.ok(Math.abs(headings.at(-1)! - Math.PI / 2) < 1e-6)
  assert.equal(completed(), 1)
})

test('heading drag updates yaw only; release commits direction and restores map controls', () => {
  const { context, event, headings, selected, captured, completed } = interaction()
  const original = { ...context.pendingGoal }
  handler('handleGoalHeadingPointerDown', context)(event())
  assert.equal(captured(), true)
  assert.equal(context.controlsRef.current.enabled, false)
  assert.equal(headings.length, 0, 'press does not invent a direction')
  context.handleGoalHeadingPointerMove(event(50, 25))
  assert.ok(Math.abs(headings.at(-1)! - Math.PI / 2) < 1e-6)
  handler('finishGoalHeading', context)(event(50, 25), false)
  assert.equal(captured(), false)
  assert.equal(context.controlsRef.current.enabled, true)
  assert.equal(completed(), 1)
  assert.deepEqual(context.pendingGoal, original)
  assert.equal(selected.length, 0)
  handler('handleMouseUp', context)(event())
})

test('cancel restores the original yaw and the pre-drag orbit state', () => {
  for (const enabled of [true, false]) {
    const { context, event, headings, captured, completed } = interaction()
    context.controlsRef.current.enabled = enabled
    handler('handleGoalHeadingPointerDown', context)(event())
    handler('finishGoalHeading', context)(event(50, 25), true)
    assert.equal(headings.at(-1), .7)
    assert.equal(captured(), false)
    assert.equal(context.controlsRef.current.enabled, enabled)
    assert.equal(completed(), 1)
  }
})

test('browse/readonly and modifier gestures do not enter heading drag', () => {
  const passive = interaction()
  passive.context.goalGestureEnabled = false
  handler('handleGoalHeadingPointerDown', passive.context)(passive.event())
  assert.equal(passive.captured(), false)
  for (const patch of [{ shiftKey: true }, { altKey: true }, { button: 2 }]) {
    const { context, event, captured } = interaction()
    handler('handleGoalHeadingPointerDown', context)({ ...event(), ...patch })
    assert.equal(captured(), false)
  }
})

test('another pointer cannot overwrite or end the active drag', () => {
  const { context, event, headings, captured, completed } = interaction()
  handler('handleGoalHeadingPointerDown', context)(event())
  const before = headings.length
  const second = { ...event(50, 25), pointerId: 2 }
  handler('handleGoalHeadingPointerDown', context)(second)
  context.handleGoalHeadingPointerMove(second)
  handler('finishGoalHeading', context)(second, false)
  assert.equal(headings.length, before)
  assert.equal(captured(), true)
  assert.equal(completed(), 0)
})

test('mode/target changes and unmount release an unfinished drag', () => {
  const { context, event, captured } = interaction()
  const effect = expression(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file) === '() => clearGoalHeadingPointer' ? node.arguments[0] : undefined, context)
  const cleanup = effect()
  handler('handleGoalHeadingPointerDown', context)(event())
  cleanup()
  assert.equal(captured(), false)
  assert.equal(context.controlsRef.current.enabled, true)
  assert.equal(context.goalHeadingPointerRef.current, null)
})

test('a single press-drag-release selects the position and heading without a second press', () => {
  const { context, event, headings, selected, captured, completed } = interaction()
  context.pendingGoal = null; context.pendingGoalYaw = undefined; context.goalHeadingEditing = false
  handler('handleGoalHeadingPointerDown', context)(event(50, 50))
  assert.equal(selected.length, 1)
  assert.equal(headings.length, 0)
  assert.equal(captured(), true)
  const echoEffect = expression(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('pointer.goal !== pendingGoal') ? node.arguments[0] : undefined, context)
  context.goalHeadingEditing = true
  echoEffect()
  assert.equal(captured(), true, 'new target echo must not cancel its own drag')
  context.handleGoalHeadingPointerMove(event(75, 50))
  handler('finishGoalHeading', context)(event(75, 50), false)
  assert.equal(completed(), 1)
  assert.equal(captured(), false)
  assert.ok(Math.abs(headings.at(-1)!) < 1e-6)
  assert.equal(selected.length, 1)
  assert.ok(Math.hypot(selected[0].x, selected[0].y, selected[0].z! - .4) < 1e-10)
})

test('tap and cancelled first drag leave no fabricated direction or ready preview', () => {
  for (const cancelled of [false, true]) {
    const { context, event, headings, completed } = interaction()
    context.pendingGoal = null; context.pendingGoalYaw = undefined; context.goalHeadingEditing = false
    handler('handleGoalHeadingPointerDown', context)(event(50, 50))
    if (cancelled) context.handleGoalHeadingPointerMove(event(75, 50))
    handler('finishGoalHeading', context)(event(50, 50), cancelled)
    assert.equal(headings.at(-1), undefined)
    assert.equal(completed(), 0)
  }
})

test('a goal renders as a small point with no acceptance ring or default heading arrow', () => {
  const scene = new THREE.Scene()
  const pendingGoalArrowRef = { current: null as THREE.ArrowHelper | null }
  const cleanup = expression(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('const marker = new THREE.Group()') ? node.arguments[0] : undefined, {
    THREE, lingtuToThree, sceneRef: { current: scene }, pendingGoal: { x: 2, y: 3, z: .4 },
    underlayDisplayZ: undefined, plannedGoal: null, goalGestureEnabled: false, pendingGoalLabel: '点击目标',
    pendingGoalArrowRef, makeGoalLabel: () => assert.fail('an ordinary point needs no oversized label'),
    removeFrom: (parent: THREE.Scene, object: THREE.Object3D) => parent.remove(object),
  })()
  const marker = scene.children[0]
  assert.deepEqual(marker.position.toArray(), lingtuToThree([2, 3, .45]))
  const meshes = marker.children.filter((child): child is THREE.Mesh => child instanceof THREE.Mesh)
  assert.equal(meshes.length, 1)
  assert.ok(meshes[0].geometry instanceof THREE.CircleGeometry)
  assert.equal(meshes[0].geometry.parameters.radius, .03)
  assert.equal(pendingGoalArrowRef.current!.visible, false)
  cleanup()
})

test('direction updates reuse the marker and update the existing arrow', () => {
  let marker: ts.CallExpression | undefined, direction: ts.CallExpression | undefined
  const visit = (node: ts.Node) => {
    if (ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect') {
      if (node.arguments[0].getText(file).includes('const marker = new THREE.Group()')) marker = node
      if (node.arguments[0].getText(file).includes('arrow.setDirection')) direction = node
    }
    ts.forEachChild(node, visit)
  }
  visit(file)
  assert.ok(marker && direction)
  assert.doesNotMatch(marker.arguments[1].getText(file), /pendingGoalYaw/)
  const arrow = new THREE.ArrowHelper()
  expression(node => node === direction ? direction.arguments[0] : undefined, {
    THREE, pendingGoalArrowRef: { current: arrow }, pendingGoalYaw: undefined,
  })()
  assert.equal(arrow.visible, false)
  expression(node => node === direction ? direction.arguments[0] : undefined, {
    THREE, pendingGoalArrowRef: { current: arrow }, pendingGoalYaw: Math.PI / 2,
  })()
  assert.equal(arrow.visible, true)
  assert.ok(new THREE.Vector3(0, 1, 0).applyQuaternion(arrow.quaternion).distanceTo(new THREE.Vector3(0, 0, -1)) < 1e-6)
  arrow.dispose()
})
