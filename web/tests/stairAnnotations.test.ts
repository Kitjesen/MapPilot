import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as THREE from 'three'
import ts from 'typescript'
import { createStairAnnotations, selectSavedMapBounds } from '../src/components/scene3d/stairAnnotations.ts'
import { updateCloudHeightRange } from '../src/components/scene3d/layers/layerUtils.ts'
import { pickNavigationPoint } from '../src/components/scene3d/goalPicking.ts'
import { threeToLingtu } from '../src/services/coordinateFrame.ts'
import type { StairAnnotation } from '../src/types/stairs.ts'

function setup() {
  const camera = new THREE.OrthographicCamera(-5, 5, 5, -5, 0.1, 100)
  camera.position.set(0, 10, 0)
  camera.up.set(0, 0, -1)
  camera.lookAt(0, 0, 0)
  camera.updateMatrixWorld()
  const points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial())
  const viewport = { width: 100, height: 100 }
  const rectangle = { x0: 40, y0: 40, x1: 60, y1: 60 }
  return { camera, points, viewport, rectangle }
}

test('rectangle projection returns observed map XYZ bounds and skips outside or behind-camera points', () => {
  const { camera, points, viewport, rectangle } = setup()
  const saved = [-0.5, -0.5, 0, 0.5, 0.5, 2, 2, 2, 4, 0, 0, 11, Number.NaN, 0, 0]
  assert.deepEqual(selectSavedMapBounds(saved, points, camera, viewport, rectangle), {
    min: { x: -0.5, y: -0.5, z: 0 }, max: { x: 0.5, y: 0.5, z: 2 },
  })
  assert.deepEqual(selectSavedMapBounds(saved, points, camera, viewport, {
    x0: rectangle.x1, y0: rectangle.y1, x1: rectangle.x0, y1: rectangle.y0,
  }), selectSavedMapBounds(saved, points, camera, viewport, rectangle))
})

test('height selection includes visible fade transitions and full height restores hidden floors', () => {
  const { camera, points, viewport, rectangle } = setup()
  const saved = [-0.5, -0.5, 0, -0.4, -0.4, 1.5, 0.5, 0.5, 3, 0.4, 0.4, 4.5, 0, 0, 6]
  updateCloudHeightRange(points, 2, 4, 'clip')
  const clipped = selectSavedMapBounds(saved, points, camera, viewport, rectangle)
  assert.ok(clipped)
  assert.equal(clipped.min.z, 2.975)
  assert.equal(clipped.max.z, 3.025)
  updateCloudHeightRange(points, 2, 4, 'fade')
  assert.deepEqual(selectSavedMapBounds(saved, points, camera, viewport, rectangle), {
    min: { x: -0.4, y: -0.4, z: 1.5 }, max: { x: 0.5, y: 0.5, z: 4.5 },
  })
  updateCloudHeightRange(points, null, null)
  assert.deepEqual(selectSavedMapBounds(saved, points, camera, viewport, rectangle), {
    min: { x: -0.5, y: -0.5, z: 0 }, max: { x: 0.5, y: 0.5, z: 6 },
  })
  assert.deepEqual(saved, [-0.5, -0.5, 0, -0.4, -0.4, 1.5, 0.5, 0.5, 3, 0.4, 0.4, 4.5, 0, 0, 6], 'display selection does not alter saved map data')
})

test('empty or hidden selections have no bounds and planar observations get only box thickness', () => {
  const { camera, points, viewport, rectangle } = setup()
  assert.equal(selectSavedMapBounds([3, 3, 0], points, camera, viewport, rectangle), null)
  const selected = selectSavedMapBounds([0, 0, 1], points, camera, viewport, rectangle)
  assert.deepEqual(selected, { min: { x: -0.025, y: -0.025, z: 0.975 }, max: { x: 0.025, y: 0.025, z: 1.025 } })
  assert.ok(selected && !('lower' in selected) && !('upper' in selected))
  points.visible = false
  assert.equal(selectSavedMapBounds([0, 0, 1], points, camera, viewport, rectangle), null)
})

test('annotation render preserves map coordinates and makes the draft more visible than saved boxes', () => {
  const annotation = {
    id: 'stairs-1', name: 'North stair', frame_id: 'map', content_epoch: 1,
    bounds: { min: { x: 1, y: 2, z: 0 }, max: { x: 3, y: 4, z: 3 } },
    lower: { x: 1, y: 3, z: 0 }, upper: { x: 3, y: 3, z: 3 },
  } as StairAnnotation
  const rendered = createStairAnnotations([annotation], { ...annotation })
  const saved = rendered.getObjectByName('stair-stairs-1')!
  const draft = rendered.getObjectByName('stair-draft')!
  assert.deepEqual(saved.getObjectByName('stair-bounds')?.position.toArray(), [2, 1.5, -3])
  assert.deepEqual(saved.getObjectByName('stair-lower')?.position.toArray(), [1, 0, -3])
  assert.deepEqual(saved.getObjectByName('stair-upper')?.position.toArray(), [3, 3, -3])
  assert.ok(saved.getObjectByName('stair-direction'))
  const savedBox = saved.getObjectByName('stair-bounds') as THREE.LineSegments
  const draftBox = draft.getObjectByName('stair-bounds') as THREE.LineSegments
  assert.ok((draftBox.material as THREE.Material).opacity > (savedBox.material as THREE.Material).opacity)
})

const sceneSource = readFileSync(new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8')
const sceneFile = ts.createSourceFile('Scene3D.tsx', sceneSource, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
function sceneHandler(name: string, context: Record<string, unknown>): (event: unknown) => void {
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

function interaction(tool: 'bounds' | 'lower' | 'upper') {
  const { camera, points } = setup()
  points.geometry.setAttribute('position', new THREE.Float32BufferAttribute([0, 1, 0], 3))
  points.updateMatrixWorld()
  let captured = false
  let stopped = 0
  const selectedBounds: unknown[] = []
  const selectedPoints: unknown[] = []
  const context = {
    THREE, pickNavigationPoint, selectSavedMapBounds, threeToLingtu,
    stairTool: tool, robotZ: 0.42, savedMapFlat: [0, 0, 1],
    controlsRef: { current: { enabled: true } },
    rendererRef: { current: { domElement: { getBoundingClientRect: () => ({ left: 0, top: 0, width: 100, height: 100 }) } } },
    cameraRef: { current: camera }, savedMapRef: { current: points },
    stairSelectionRef: { current: { style: {} } },
    stairPointerRef: { current: null }, raycaster: { current: new THREE.Raycaster() },
    mouseDownPos: { current: null },
    onStairBounds: (value: unknown) => selectedBounds.push(JSON.parse(JSON.stringify(value))),
    onStairPoint: (value: unknown) => selectedPoints.push(JSON.parse(JSON.stringify(value))),
    onPendingGoal: () => assert.fail('annotation must not propose a navigation target'),
    onRelocalize: () => assert.fail('annotation must not relocalize'),
    clearStairPointer: (event: unknown) => { void event },
  }
  context.clearStairPointer = sceneHandler('clearStairPointer', context)
  const event = (x: number, y: number) => ({
    clientX: x, clientY: y, pointerId: 1, button: 0,
    shiftKey: true, altKey: true,
    stopPropagation: () => { stopped++ }, preventDefault() {},
    currentTarget: {
      setPointerCapture: () => { captured = true }, hasPointerCapture: () => captured,
      releasePointerCapture: () => { captured = false },
    },
  })
  return { context, event, selectedBounds, selectedPoints, captured: () => captured, stopped: () => stopped }
}

test('annotation dragging captures the pointer, disables orbit, returns bounds, and never invokes navigation', () => {
  const { context, event, selectedBounds, selectedPoints, captured, stopped } = interaction('bounds')
  sceneHandler('handleStairPointerDown', context)(event(40, 40))
  assert.equal(context.controlsRef.current.enabled, false)
  assert.equal(captured(), true)
  sceneHandler('handleStairPointerMove', context)(event(60, 60))
  sceneHandler('handleStairPointerUp', context)(event(60, 60))
  sceneHandler('handleMouseDown', context)(event(50, 50))
  sceneHandler('handleMouseUp', context)(event(50, 50))
  assert.equal(captured(), false)
  assert.ok(stopped() >= 3)
  assert.deepEqual(selectedBounds, [{ min: { x: -0.025, y: -0.025, z: 0.975 }, max: { x: 0.025, y: 0.025, z: 1.025 } }])
  assert.deepEqual(selectedPoints, [])
})

test('entrance tools select saved-map surfaces only and leave blank or clipped clicks empty', () => {
  for (const tool of ['lower', 'upper'] as const) {
    const { context, event, selectedBounds, selectedPoints } = interaction(tool)
    const down = sceneHandler('handleStairPointerDown', context)
    const up = sceneHandler('handleStairPointerUp', context)
    down(event(50, 50)); up(event(50, 50))
    assert.deepEqual(selectedPoints, [{ x: 0, y: 0, z: 1 }])
    down(event(10, 10)); up(event(10, 10))
    assert.equal(selectedPoints.length, 1)
    updateCloudHeightRange(context.savedMapRef.current, null, 0.5)
    down(event(50, 50)); up(event(50, 50))
    assert.equal(selectedPoints.length, 1)
    assert.deepEqual(selectedBounds, [])
  }
})
