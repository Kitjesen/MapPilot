import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import * as THREE from 'three'
import * as coordinateFrame from '../src/services/coordinateFrame.ts'
import * as mapEdit from '../src/services/mapEdit.ts'
import type { PcdCloud } from '../src/services/pcdPreview.ts'
import type { MapEditOperation, MapEditPreview } from '../src/types/mapEdit.ts'
import * as mapEditing from '../src/components/scene3d/mapEditing.ts'
import * as stairAnnotations from '../src/components/scene3d/stairAnnotations.ts'
import { createPcdLayer } from '../src/components/scene3d/layers/pcdLayer.ts'

const exports = {} as {
  previewDistance: (bounds: THREE.Box3, aspect: number, direction: THREE.Vector3) => number
  projectAxes: (rotation: THREE.Quaternion) => { name: string; x: number; y: number; depth: number }[]
  editedCloud: (cloud: PcdCloud, view?: string, operations?: readonly MapEditOperation[], preview?: MapEditPreview | null) => PcdCloud
}
const source = readFileSync(new URL('../src/components/MapGalleryScene.tsx', import.meta.url), 'utf8')
const compiledSource = ts.transpileModule(`${source}\nexport { previewDistance, projectAxes, editedCloud }`, { compilerOptions: {
  module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022, jsx: ts.JsxEmit.ReactJSX,
} }).outputText
runInNewContext(compiledSource, { exports, require: (name: string) => {
  if (name === 'three') return THREE
  if (name.includes('coordinateFrame')) return coordinateFrame
  if (name.includes('/services/mapEdit')) return mapEdit
  return {}
} })

test('preview fits the map bounds in both orbit and top views at narrow and wide aspect ratios', () => {
  for (const size of [[60, 4, 35], [2, 20, 2], [.1, .1, .1]]) for (const aspect of [.6, 1, 2]) for (const top of [false, true]) {
    const center = new THREE.Vector3(15, 2, -8)
    const bounds = new THREE.Box3().setFromCenterAndSize(center, new THREE.Vector3(...size))
    const direction = new THREE.Vector3(...(top ? [0, 1, .001] : [1, 1.5, 1.4])).normalize()
    const distance = exports.previewDistance(bounds, aspect, direction)
    const camera = new THREE.PerspectiveCamera(45, aspect, .01, 10000)
    camera.position.copy(center).addScaledVector(direction, distance)
    camera.lookAt(center)
    camera.updateMatrixWorld()
    for (const x of [bounds.min.x, bounds.max.x]) for (const y of [bounds.min.y, bounds.max.y]) for (const z of [bounds.min.z, bounds.max.z]) {
      const point = new THREE.Vector3(x, y, z).project(camera)
      assert.ok(Math.abs(point.x) < 1 && Math.abs(point.y) < 1 && point.z < 1, `${size}, ${aspect}, top=${top}`)
    }
  }
})

test('orientation follows the camera in map X/Y/Z coordinates, including Z up', () => {
  const cases = [
    { rotation: new THREE.Quaternion(), expected: [[71, 44, 0], [44, 44, -1], [44, 17, 0]] },
    { rotation: new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2),
      expected: [[44, 44, 1], [71, 44, 0], [44, 17, 0]] },
  ]
  for (const { rotation, expected } of cases) {
    const axes = exports.projectAxes(rotation).sort((a, b) => a.name.localeCompare(b.name))
    axes.forEach((axis, i) => {
      assert.equal(axis.name, ['X', 'Y', 'Z'][i])
      ;[axis.x, axis.y, axis.depth].forEach((value, j) => {
        assert.ok(Math.abs(value - expected[i][j]) < 1e-9, `${axis.name} coordinate ${j}: ${value}`)
      })
    })
  }
})

function mountScene(reducedMotion = false, pointCount = 0, initialProps: Record<string, unknown> = {}) {
  const effects: { deps: unknown[]; cleanup?: void | (() => void) }[] = []
  const refs: { current: unknown }[] = []
  const memos: { value: unknown; deps: unknown[] }[] = []
  let pendingEffects: (() => void)[] = []
  let refIndex = 0, effectIndex = 0, memoIndex = 0
  const sameDeps = (a: unknown[], b: unknown[]) => a.length === b.length && a.every((value, i) => Object.is(value, b[i]))
  const memo = (callback: () => unknown, deps: unknown[]) => {
    const index = memoIndex++
    if (!memos[index] || !sameDeps(memos[index].deps, deps)) memos[index] = { value: callback(), deps }
    return memos[index].value
  }
  const events: string[] = []
  const labels: string[] = []
  const frames = new Map<number, () => void>()
  const cancelled: number[] = []
  let frameId = 0
  const instances: Controls[] = []
  let camera: THREE.PerspectiveCamera
  let renderedPoints: THREE.Points | undefined
  class Controls extends THREE.EventDispatcher<{ change: { type: 'change' }; start: { type: 'start' }; end: { type: 'end' } }> {
    enableDamping = false
    dampingFactor = .05
    changesRemaining = 0
    target = new THREE.Vector3()
    constructor() { super(); instances.push(this) }
    update() {
      events.push('update')
      if (this.changesRemaining > 0) {
        this.changesRemaining--
        this.dispatchEvent({ type: 'change' })
        return true
      }
      return false
    }
    listenToKeyEvents() {}
    dispose() {}
  }
  class Renderer {
    domElement = { setAttribute() {}, remove() {} }
    setPixelRatio() {}
    getPixelRatio() { return 1 }
    setSize() {}
    render(scene: THREE.Scene, renderedCamera: THREE.PerspectiveCamera) {
      camera = renderedCamera
      renderedPoints = scene.children.find((object): object is THREE.Points => object instanceof THREE.Points)
      events.push('map')
    }
    dispose() {}
  }
  const axesContext = {
    clearRect() { events.push('axes'); labels.length = 0 }, scale() {}, beginPath() {},
    moveTo() {}, lineTo() {}, stroke() {}, fillText(text: string) { labels.push(text) },
  }
  const canvas = { getContext: () => axesContext }
  const element = { clientWidth: 640, clientHeight: 480, appendChild() {}, hidden: true, style: {},
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 640, height: 480 }),
    setPointerCapture() {}, hasPointerCapture: () => true, releasePointerCapture() {} }
  const pointLayer = pointCount ? new THREE.Points(new THREE.BufferGeometry()
    .setAttribute('position', new THREE.BufferAttribute(new Float32Array(pointCount * 3), 3)), new THREE.PointsMaterial()) : null
  if (pointLayer) pointLayer.geometry.boundingBox = new THREE.Box3(new THREE.Vector3(-1, -1, -1), new THREE.Vector3(1, 1, 1))
  const jsx = (type: string, props: { ref?: { current: unknown } }) => {
    if (props.ref) props.ref.current = type === 'canvas' ? canvas : element
    return { type, props }
  }
  const sceneExports = {} as { MapGalleryScene: (props: object) => unknown }
  runInNewContext(compiledSource, {
    exports: sceneExports,
    require: (name: string) => {
      if (name === 'react') return {
        useRef: (current: unknown) => refs[refIndex++] ?? (refs[refIndex - 1] = { current }),
        useCallback: (callback: unknown, deps: unknown[]) => memo(() => callback, deps), useMemo: memo,
        useEffect: (effect: () => void | (() => void), deps: unknown[]) => {
          const index = effectIndex++
          if (effects[index] && sameDeps(effects[index].deps, deps)) return
          pendingEffects.push(() => {
            effects[index]?.cleanup?.()
            effects[index] = { deps, cleanup: effect() }
          })
        },
        useState: (initial: unknown) => [initial, () => assert.fail('scene initialization failed')],
      }
      if (name === 'react/jsx-runtime') return { jsx, jsxs: jsx }
      if (name === 'three') return { ...THREE, WebGLRenderer: Renderer }
      if (name.includes('OrbitControls')) return { OrbitControls: Controls }
      if (name.includes('coordinateFrame')) return coordinateFrame
      if (name.includes('/services/mapEdit')) return mapEdit
      if (name.includes('/scene3d/mapEditing')) return mapEditing
      if (name.includes('/scene3d/stairAnnotations')) return stairAnnotations
      if (name.includes('/layers/pcdLayer')) return { createPcdLayer }
      if (name.includes('savedMapLayer')) return { createSavedMapLayer: () => pointLayer }
      return { default: {} }
    },
    devicePixelRatio: 1,
    matchMedia: (query: string) => {
      assert.equal(query, '(prefers-reduced-motion: reduce)')
      return { matches: reducedMotion }
    },
    requestAnimationFrame: (callback: () => void) => { frames.set(++frameId, callback); return frameId },
    cancelAnimationFrame: (id: number) => { cancelled.push(id); frames.delete(id) },
    ResizeObserver: class { observe() {} disconnect() {} },
  })
  let props = { name: 'garage', voxelScale: 1, pointSize: 2, opacity: 1, ceiling: null, grid: false, gridStep: 1, ...initialProps }
  let tree: { props: Record<string, (event: unknown) => void> }
  const render = (changes: Record<string, unknown> = {}) => {
    props = { ...props, ...changes }
    refIndex = effectIndex = memoIndex = 0
    pendingEffects = []
    tree = sceneExports.MapGalleryScene(props) as typeof tree
    pendingEffects.forEach(effect => effect())
  }
  render()
  const nextFrame = () => {
    const frame = frames.entries().next().value
    assert.ok(frame, 'a rendering frame is queued')
    frames.delete(frame[0])
    frame[1]()
  }
  nextFrame()
  events.length = 0
  return { controls: instances[0], pointLayer, events, labels, frames, cancelled, nextFrame, render,
    getCamera: () => camera, getPoints: () => renderedPoints, getTree: () => tree, element,
    cleanup: () => effects.forEach(effect => effect.cleanup?.()) }
}

test('draft rendering preserves source metadata and restores its buffer on undo or original view', () => {
  const cloud: PcdCloud = { positions: new Float32Array([-5, 0, 0, 5, 0, 1]),
    bounds: { min: [-5, 0, 0], max: [5, 0, 1] }, colorRange: { min: 0, max: 1 }, sourceCount: 2 }
  const operations: MapEditOperation[] = [{ operation: 'delete_inside', bounds: {
    min: { x: -6, y: -1, z: -1 }, max: { x: -4, y: 1, z: 1 },
  } }]
  const draft = exports.editedCloud(cloud, 'draft', operations)
  assert.deepEqual([...draft.positions], [5, 0, 1])
  assert.equal(draft.bounds, cloud.bounds)
  assert.equal(draft.colorRange, cloud.colorRange)
  assert.equal(exports.editedCloud(cloud, 'draft', []), cloud)
  assert.equal(exports.editedCloud(cloud, 'original', operations), cloud)
  const preview = { preview_points: [1, 2, 3], preview_removed: [], preview_sample: true, point_count: 1, removed_points: 1 }
  assert.deepEqual([...exports.editedCloud(cloud, 'draft', [{ operation: 'voxel', voxel_size: .1 }], preview).positions], [1, 2, 3])
})

test('manual patches expand the display bounds without mutating source bounds', () => {
  const cloud: PcdCloud = { positions: new Float32Array([0, 0, 0, 1, 1, 1]),
    bounds: { min: [0, 0, 0], max: [1, 1, 1] }, colorRange: { min: 0, max: 1 }, sourceCount: 2 }
  const plane: MapEditOperation = { operation: 'add_plane', axis: 'z', offset: 2, spacing: 1,
    bounds: { min: { x: 2, y: 2, z: 1 }, max: { x: 3, y: 3, z: 3 } } }
  const draft = exports.editedCloud(cloud, 'draft', [plane])
  assert.deepEqual([...draft.bounds.min], [0, 0, 0])
  assert.deepEqual([...draft.bounds.max], [3, 3, 2])
  assert.deepEqual(cloud.bounds.max, [1, 1, 1])
  assert.equal(draft.positions.length, 18)
})

test('entering crop retains camera and selects the displayed draft, with undo restoring source selection', t => {
  const cloud: PcdCloud = { positions: new Float32Array([-5, 0, 0, 5, 0, 1]),
    bounds: { min: [-5, 0, 0], max: [5, 0, 1] }, colorRange: { min: 0, max: 1 }, sourceCount: 2 }
  let selected: { min: { x: number }; max: { x: number } } | undefined
  const scene = mountScene(false, 0, { cloud, onMapEditBounds: (bounds: typeof selected) => { selected = bounds } })
  t.after(scene.cleanup)
  const camera = scene.getCamera()
  camera.position.set(0, 10, 20)
  scene.controls.target.set(0, 0, 0)
  camera.lookAt(scene.controls.target)
  const position = camera.position.clone(), rotation = camera.quaternion.clone(), target = scene.controls.target.clone()
  const operations: MapEditOperation[] = [{ operation: 'delete_inside', bounds: {
    min: { x: -6, y: -1, z: -1 }, max: { x: -4, y: 1, z: 1 },
  } }]
  const edit = { selecting: true, selection: null, view: 'draft', operations, showDeleted: false }
  scene.render({ mapEdit: edit })
  assert.equal(scene.getCamera(), camera)
  assert.ok(camera.position.equals(position))
  assert.ok(camera.quaternion.equals(rotation))
  assert.ok(scene.controls.target.equals(target))
  assert.equal(camera.fov, 45)
  const drag = () => {
    const base = { pointerId: 1, button: 0, currentTarget: scene.element, preventDefault() {}, stopPropagation() {} }
    scene.getTree().props.onPointerDownCapture({ ...base, clientX: 0, clientY: 0 })
    scene.getTree().props.onPointerUpCapture({ ...base, clientX: 640, clientY: 480 })
  }
  drag()
  assert.ok(selected && selected.min.x > 4)
  assert.ok(camera.position.equals(position))
  scene.render({ mapEdit: { ...edit, operations: [] } })
  drag()
  assert.equal(selected?.min.x, -5)
  assert.equal(selected?.max.x, 5)
})

test('multiple camera changes share one frame for the map and XYZ axes', t => {
  const scene = mountScene()
  t.after(scene.cleanup)
  for (let i = 0; i < 3; i++) scene.controls.dispatchEvent({ type: 'change' })
  assert.equal(scene.frames.size, 1)
  assert.deepEqual(scene.events, [])
  scene.nextFrame()
  assert.deepEqual(scene.events, ['update', 'map', 'axes'])
  assert.equal(scene.frames.size, 0)
})

test('damping continues rendering until controls stop changing', t => {
  const scene = mountScene()
  t.after(scene.cleanup)
  assert.equal(scene.controls.enableDamping, true)
  assert.equal(scene.controls.dampingFactor, .1)
  scene.controls.changesRemaining = 2
  scene.controls.dispatchEvent({ type: 'change' })
  for (let i = 0; i < 2; i++) {
    scene.nextFrame()
    assert.equal(scene.frames.size, 1)
  }
  scene.nextFrame()
  assert.deepEqual(scene.events, Array(3).fill(['update', 'map', 'axes']).flat())
  assert.equal(scene.frames.size, 0)
})

test('unmount cancels a pending rendering frame', () => {
  const scene = mountScene()
  scene.controls.dispatchEvent({ type: 'change' })
  const pendingFrame = [...scene.frames.keys()][0]
  scene.cleanup()
  assert.deepEqual(scene.cancelled, [pendingFrame])
  assert.equal(scene.frames.size, 0)
  assert.deepEqual(scene.events, [])
})

test('unmount removes the camera change listener', () => {
  const scene = mountScene()
  scene.cleanup()
  scene.controls.dispatchEvent({ type: 'change' })
  assert.equal(scene.frames.size, 0)
  assert.deepEqual(scene.events, [])
})

test('reduced motion disables orbit damping', t => {
  const scene = mountScene(true)
  t.after(scene.cleanup)
  assert.equal(scene.controls.enableDamping, false)
})

test('large clouds retain all vertices and the same appearance through wheel, drag and idle', async t => {
  const count = 1_000_001
  const cloud: PcdCloud = { positions: new Float32Array(count * 3),
    bounds: { min: [-1, -1, -1], max: [1, 1, 1] }, colorRange: { min: -1, max: 1 }, sourceCount: count }
  const scene = mountScene(false, 0, { cloud, pointSize: 2.5, opacity: .65 })
  t.after(scene.cleanup)
  const points = scene.getPoints()!
  const geometry = points.geometry
  const material = points.material as THREE.ShaderMaterial
  const assertFullCloud = () => {
    assert.equal(scene.getPoints(), points)
    assert.equal(points.geometry, geometry)
    assert.equal(geometry.index, null)
    assert.equal(geometry.getAttribute('position').count, count)
    assert.equal(geometry.getAttribute('position').array, cloud.positions)
    assert.equal(points.material, material)
    assert.equal(material.uniforms.pointSize.value, 2.5)
    assert.equal(material.uniforms.opacity.value, .65)
    assert.equal(material.opacity, .65)
  }
  assertFullCloud()
  for (let i = 0; i < 5; i++) {
    // Each OrbitControls wheel event emits this complete gesture sequence.
    scene.controls.dispatchEvent({ type: 'start' })
    scene.controls.dispatchEvent({ type: 'change' })
    scene.controls.dispatchEvent({ type: 'end' })
    scene.nextFrame()
    assertFullCloud()
    assert.equal(scene.frames.size, 0)
  }
  scene.controls.dispatchEvent({ type: 'start' })
  scene.controls.changesRemaining = 2
  scene.controls.dispatchEvent({ type: 'change' })
  while (scene.frames.size) { scene.nextFrame(); assertFullCloud() }
  scene.controls.dispatchEvent({ type: 'end' })
  await new Promise(resolve => setTimeout(resolve, 350))
  assertFullCloud()
  assert.equal(scene.frames.size, 0, 'stopping does not schedule another density state')
  scene.render({ onMapEditAnchor: () => {} })
  assertFullCloud()
  assert.equal(scene.frames.size, 0, 'unrelated callbacks do not redraw or rebuild the cloud')
})

test('unmount disposes the full point geometry and material once', () => {
  const scene = mountScene(false, 1_000_001)
  const geometry = scene.pointLayer!.geometry
  const material = scene.pointLayer!.material
  let geometryDisposals = 0
  let materialDisposals = 0
  geometry.addEventListener('dispose', () => { geometryDisposals++ })
  material.addEventListener('dispose', () => { materialDisposals++ })
  scene.cleanup()
  assert.equal(geometryDisposals, 1)
  assert.equal(materialDisposals, 1)
  assert.equal(geometry.index, null)
  assert.equal(scene.frames.size, 0)
})
