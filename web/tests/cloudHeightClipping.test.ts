import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'
import type { BinaryCloud } from '../src/hooks/useBinaryCloud.ts'
import { createLiveCloudLayer, upsertLiveCloudLayer } from '../src/components/scene3d/layers/liveCloudLayer.ts'
import { pickNavigationPoint } from '../src/components/scene3d/goalPicking.ts'
import { cloudPointIsVisible, interpolateCloudHeightRange, updateCloudHeightRange, visibleCloudBounds } from '../src/components/scene3d/layers/layerUtils.ts'
import { createSavedMapLayer, SAVED_MAP_Z_CEIL, SAVED_MAP_Z_FLOOR } from '../src/components/scene3d/layers/savedMapLayer.ts'

function liveCloud(positions: number[]): BinaryCloud {
  return {
    positions: new Float32Array(positions), colors: new Float32Array(positions.length), count: positions.length / 3,
    seq: 1, protocolVersion: null, frameId: null, epoch: null, stampS: null, sequence: null,
    streamKind: null, connected: true, transport: 'ws', lastFrameAt: 1, error: null,
  }
}

test('saved-map GPU geometry preserves all source points above the former 80000 point cap', () => {
  const count = 1266153
  const source = Array.from({ length: count * 3 }, (_, i) => i % 3 === 0 ? i / 3 : 0)
  const points = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  const positions = points.geometry.getAttribute('position')
  assert.equal(positions.count, count)
  assert.equal(positions.getX(count - 1), count - 1)
  points.geometry.dispose()
  ;(points.material as THREE.Material).dispose()
})

test('display height follows upstairs and downstairs continuously without overshoot', () => {
  const lower = { minZ: -1, maxZ: 1 }
  const upper = { minZ: 3, maxZ: 5 }
  assert.deepEqual(interpolateCloudHeightRange(lower, upper, 0), lower)
  assert.deepEqual(interpolateCloudHeightRange(lower, upper, .25), { minZ: 0, maxZ: 2 })
  assert.deepEqual(interpolateCloudHeightRange(lower, upper, .5), { minZ: 1, maxZ: 3 })
  assert.deepEqual(interpolateCloudHeightRange(upper, lower, .75), { minZ: 0, maxZ: 2 })
  assert.deepEqual(interpolateCloudHeightRange(lower, upper, 2), upper)
  assert.deepEqual(interpolateCloudHeightRange(lower, { minZ: null, maxZ: null }, .25), { minZ: null, maxZ: null })
})

test('fading excludes distant context and updates GPU uniforms without rebuilding geometry or recompiling', () => {
  const source = [1, 2, -1, 3, 4, 0, 100, 200, 8]
  const live = liveCloud([1, -1, -2, 3, 0, -4, 100, 8, -200])
  for (const points of [
    createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL),
    createLiveCloudLayer(live, .2),
    createLiveCloudLayer(live, .2, { vertexColors: false }),
  ]) {
    assert.ok(points)
    const positions = points.geometry.getAttribute('position')
    const material = points.material as THREE.PointsMaterial
    updateCloudHeightRange(points, -1, 1, 'fade')
    const shader = {
      uniforms: {},
      vertexShader: THREE.ShaderLib.points.vertexShader,
      fragmentShader: THREE.ShaderLib.points.fragmentShader,
    } as Parameters<THREE.PointsMaterial['onBeforeCompile']>[0]
    material.onBeforeCompile(shader, {} as THREE.WebGLRenderer)
    const range = shader.uniforms.cloudHeightRange.value as THREE.Vector4
    assert.deepEqual(range.toArray(), [-1, 1, 1, 1])
    assert.match(shader.vertexShader, /\(modelMatrix \* vec4\(transformed, 1\.0\)\)\.y/)
    assert.match(shader.fragmentShader, /if \(outsideRange >= 0\.6\) discard;/)
    assert.match(shader.fragmentShader, /diffuseColor\.a \*= 1\.0 - smoothstep\(0\.0, 0\.6, outsideRange\);/)
    assert.ok(shader.fragmentShader.indexOf('diffuseColor.a *=') < shader.fragmentShader.indexOf('#include <alphatest_fragment>'))
    assert.equal(material.clippingPlanes, null)
    assert.equal(material.transparent, true)
    assert.equal(material.depthWrite, false)
    assert.deepEqual(visibleCloudBounds(points).min.toArray(), [1, -1, -4])
    assert.deepEqual(visibleCloudBounds(points).max.toArray(), [3, 0, -2])
    const version = material.version
    const programKey = material.customProgramCacheKey()
    updateCloudHeightRange(points, 2, 4, 'fade')
    assert.equal(shader.uniforms.cloudHeightRange.value, range)
    assert.deepEqual(range.toArray(), [2, 4, 1, 1])
    assert.equal(material.version, version)
    assert.equal(material.customProgramCacheKey(), programKey)
    assert.equal(points.geometry.getAttribute('position'), positions)
  }
})

test('follow height retains its clear band and transition, then hides both far limits', () => {
  const points = createSavedMapLayer([0, 0, 0], SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  updateCloudHeightRange(points, -1, 1, 'fade')
  for (const z of [-1, 0, 1, -1.3, 1.3, -1.59, 1.59]) {
    assert.equal(cloudPointIsVisible(points, new THREE.Vector3(0, z, 0)), true, `Z ${z} remains visible`)
  }
  for (const z of [-1.6, 1.6, -2, 2, 8]) {
    assert.equal(cloudPointIsVisible(points, new THREE.Vector3(0, z, 0)), false, `Z ${z} is fully hidden`)
  }
  updateCloudHeightRange(points, null, 1, 'fade')
  assert.equal(cloudPointIsVisible(points, new THREE.Vector3(0, -8, 0)), true)
  updateCloudHeightRange(points, -1, null, 'fade')
  assert.equal(cloudPointIsVisible(points, new THREE.Vector3(0, 8, 0)), true)
})

test('upstairs and downstairs following moves the visible band while retaining stair transitions', () => {
  const heights = [0, .5, 1, 1.5, 2, 2.5, 3, 3.5, 4]
  const source = heights.flatMap(z => [0, 0, z])
  const original = source.slice()
  const points = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  const positions = points.geometry.getAttribute('position')
  const lower = { minZ: -1, maxZ: 1 }
  const upper = { minZ: 2, maxZ: 4 }
  const shown = (from: typeof lower, to: typeof lower, progress: number) => {
    const range = interpolateCloudHeightRange(from, to, progress)
    updateCloudHeightRange(points, range.minZ, range.maxZ, 'fade')
    return heights.filter(z => cloudPointIsVisible(points, new THREE.Vector3(0, z, 0)))
  }
  assert.deepEqual(shown(lower, upper, 0), [0, .5, 1, 1.5])
  assert.deepEqual(shown(lower, upper, .25), [0, .5, 1, 1.5, 2])
  assert.deepEqual(shown(lower, upper, .5), [0, .5, 1, 1.5, 2, 2.5, 3])
  assert.deepEqual(shown(lower, upper, 1), [1.5, 2, 2.5, 3, 3.5, 4])
  assert.deepEqual(shown(upper, lower, .5), [0, .5, 1, 1.5, 2, 2.5, 3])
  assert.deepEqual(shown(upper, lower, 1), [0, .5, 1, 1.5])
  assert.equal(points.geometry.getAttribute('position'), positions)
  assert.deepEqual(source, original)
})

test('fully hidden ceiling points cannot be selected and full height restores them', () => {
  const source = [0, 0, 0, 0, 0, 1.3, 0, 0, 3]
  const original = source.slice()
  const points = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  const positions = points.geometry.getAttribute('position')
  const ray = new THREE.Raycaster(new THREE.Vector3(0, 5, 0), new THREE.Vector3(0, -1, 0))
  updateCloudHeightRange(points, -1, 1, 'fade')
  const transitionPoint = pickNavigationPoint(ray, [points], .42, 'surface')
  assert.ok(transitionPoint)
  assert.ok(Math.abs(transitionPoint.y - 1.3) < 1e-6, 'the visible transition remains surface-pickable')
  assert.ok(Math.abs(visibleCloudBounds(points).max.y - 1.3) < 1e-6)
  updateCloudHeightRange(points, -1, 1, 'clip')
  assert.equal(pickNavigationPoint(ray, [points], .42, 'surface')?.y, 0, 'fixed mode excludes everything beyond the clear band')
  updateCloudHeightRange(points, null, null, 'clip')
  assert.equal(pickNavigationPoint(ray, [points], .42, 'surface')?.y, 3)
  assert.equal(visibleCloudBounds(points).max.y, 3)
  updateCloudHeightRange(points, -1, 1, 'fade')
  updateCloudHeightRange(points, null, null, 'fade')
  assert.equal(pickNavigationPoint(ray, [points], .42, 'surface')?.y, 3)
  assert.equal(points.geometry.getAttribute('position'), positions)
  assert.deepEqual(source, original)
})

test('leaving fade restores material depth, transparency and shader behavior for fixed and full views', () => {
  const points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ transparent: false, depthWrite: true }))
  const material = points.material
  updateCloudHeightRange(points, null, 1, 'fade')
  const compile = material.onBeforeCompile
  const programKey = material.customProgramCacheKey()
  const shader = {
    uniforms: {},
    vertexShader: THREE.ShaderLib.points.vertexShader,
    fragmentShader: THREE.ShaderLib.points.fragmentShader,
  } as Parameters<THREE.PointsMaterial['onBeforeCompile']>[0]
  compile(shader, {} as THREE.WebGLRenderer)
  const range = shader.uniforms.cloudHeightRange.value as THREE.Vector4
  updateCloudHeightRange(points, 0, 2, 'clip')
  assert.equal(material.transparent, false)
  assert.equal(material.depthWrite, true)
  assert.equal(material.onBeforeCompile, compile)
  assert.equal(material.customProgramCacheKey(), programKey)
  assert.deepEqual(range.toArray(), [0, 2, 0, 0], 'fixed mode disables fading while retaining stable uniforms')
  assert.equal(material.clippingPlanes?.length, 2)
  updateCloudHeightRange(points, 0, 2, 'fade')
  assert.equal(material.clippingPlanes, null)
  assert.deepEqual(range.toArray(), [0, 2, 1, 1], 'the existing cached shader receives the new fade range')
  updateCloudHeightRange(points, null, null, 'fade')
  assert.equal(material.onBeforeCompile, compile)
  assert.equal(material.customProgramCacheKey(), programKey)
  assert.deepEqual(range.toArray(), [0, 0, 0, 0])
  assert.equal(material.transparent, false)
  assert.equal(material.depthWrite, true)
  assert.equal(material.clippingPlanes, null)
})

test('saved, accumulated and scan layers share a reversible world-Z display ceiling', () => {
  const source = [1, 2, -1, 3, 4, 0, 100, 200, 8]
  const live = liveCloud([1, -1, -2, 3, 0, -4, 100, 8, -200])
  const sourceCopy = source.slice()
  const liveCopy = live.positions.slice()
  const layers = [
    createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL),
    createLiveCloudLayer(live, .2),
    createLiveCloudLayer(live, .2, { vertexColors: false }),
  ]
  for (const points of layers) {
    assert.ok(points)
    const positions = points.geometry.getAttribute('position')
    updateCloudHeightRange(points, null, 0)
    const material = points.material as THREE.PointsMaterial
    assert.equal(material.clippingPlanes?.length, 1, 'the GPU material receives the display ceiling')
    assert.deepEqual(visibleCloudBounds(points).min.toArray(), [1, -1, -4])
    assert.deepEqual(visibleCloudBounds(points).max.toArray(), [3, 0, -2])
    updateCloudHeightRange(points, null, -1)
    assert.deepEqual(visibleCloudBounds(points).min.toArray(), [1, -1, -2])
    assert.deepEqual(visibleCloudBounds(points).max.toArray(), [1, -1, -2])
    updateCloudHeightRange(points, null, -2)
    assert.ok(visibleCloudBounds(points).isEmpty())
    for (const limit of [null, undefined]) {
      updateCloudHeightRange(points, null, limit)
      assert.equal(material.clippingPlanes, null)
      assert.deepEqual(visibleCloudBounds(points).min.toArray(), [1, -1, -200])
      assert.deepEqual(visibleCloudBounds(points).max.toArray(), [100, 8, -2])
    }
    assert.equal(points.geometry.getAttribute('position'), positions, 'adjusting height retains the original geometry')
  }
  assert.deepEqual(source, sourceCopy)
  assert.deepEqual(live.positions, liveCopy)
})

test('fit bounds exclude unused buffer capacity and a completely hidden layer', () => {
  const scene = new THREE.Scene()
  const points = upsertLiveCloudLayer(scene, null, liveCloud([100, 10, 100, 200, 20, 200]), .2)
  assert.ok(points)
  updateCloudHeightRange(points, null, 2)
  upsertLiveCloudLayer(scene, points, liveCloud([4, 1, 6]), .2)
  assert.equal(points.geometry.getAttribute('position').count, 2, 'the live buffer retains spare capacity')
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [4, 1, 6])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [4, 1, 6])
  points.visible = false
  assert.ok(visibleCloudBounds(points).isEmpty())
})

test('a moving height band excludes floors above and below without rebuilding cloud geometry or material', () => {
  const points = createSavedMapLayer([0, 0, -2, 1, 0, 0, 2, 0, 2, 3, 0, 4], SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  const material = points.material as THREE.PointsMaterial
  const positions = points.geometry.getAttribute('position')
  updateCloudHeightRange(points, 0, 2)
  const planes = material.clippingPlanes
  const version = material.version
  assert.equal(planes?.length, 2)
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [1, 0, 0])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [2, 2, 0])

  updateCloudHeightRange(points, -2, 0)
  assert.equal(material.clippingPlanes, planes, 'following height updates the existing clipping planes')
  assert.equal(material.version, version, 'moving a band does not recompile the material')
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [0, -2, 0])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [1, 0, 0])

  updateCloudHeightRange(points, 2, null)
  assert.equal(material.clippingPlanes?.length, 1)
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [2, 2, 0])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [3, 4, 0])
  updateCloudHeightRange(points, null, 0)
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [0, -2, 0])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [1, 0, 0])
  updateCloudHeightRange(points, undefined, undefined)
  assert.equal(material.clippingPlanes, null)
  assert.equal(points.geometry.getAttribute('position'), positions)
})

test('height band fit applies both limits in world coordinates', () => {
  const points = createSavedMapLayer([0, 0, -2, 1, 0, 0, 2, 0, 2], SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  points.position.y = 3
  updateCloudHeightRange(points, 2, 4)
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [1, 3, 0])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [1, 3, 0])
})

test('fit bounds apply the ceiling after object transforms into world coordinates', () => {
  const points = createSavedMapLayer([1, 2, -1, 3, 4, 0], SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
  assert.ok(points)
  points.position.set(10, 2, 20)
  updateCloudHeightRange(points, null, 1)
  assert.deepEqual(visibleCloudBounds(points).min.toArray(), [11, 1, 18])
  assert.deepEqual(visibleCloudBounds(points).max.toArray(), [11, 1, 18])
})

test('a newer cloud frame remains surface-pickable outside the previous frame bounds', () => {
  const scene = new THREE.Scene()
  const points = upsertLiveCloudLayer(scene, null, liveCloud([0, 0, 0]), .2)
  assert.ok(points)
  updateCloudHeightRange(points, null, 2)
  const ray = new THREE.Raycaster(new THREE.Vector3(0, 5, 0), new THREE.Vector3(0, -1, 0))
  assert.equal(pickNavigationPoint(ray, [points], .42, 'surface')?.y, 0)
  upsertLiveCloudLayer(scene, points, liveCloud([10, 1, 0]), .2)
  ray.ray.origin.x = 10
  assert.deepEqual(pickNavigationPoint(ray, [points], .42, 'surface')?.toArray(), [10, 1, 0])
})
