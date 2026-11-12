import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'
import { CLOUD_HEIGHT_GLSL, HEIGHT_COLORS, cloudHeightRange, cloudColorRange, writeCloudHeightColor } from '../src/services/cloudHeightColor.ts'
import { setHeightColorRange } from '../src/components/scene3d/layers/heightColor.ts'
import { updateCloudHeightRange } from '../src/components/scene3d/layers/layerUtils.ts'
import { createOctomapLayer } from '../src/components/scene3d/layers/octomapLayer.ts'
import { upsertLiveCloudLayer } from '../src/components/scene3d/layers/liveCloudLayer.ts'
import { createSavedMapLayer, SAVED_MAP_Z_CEIL, SAVED_MAP_Z_FLOOR } from '../src/components/scene3d/layers/savedMapLayer.ts'
import type { BinaryCloud } from '../src/hooks/useBinaryCloud.ts'
import type { MapSceneEvent } from '../src/types/index.ts'

function colorAt(z: number, min?: number, max?: number): number[] {
  const out = new Float32Array(3)
  writeCloudHeightColor(z, out, 0, min, max)
  return Array.from(out)
}

test('height colors stay fixed between frames, clamp outliers, and handle a flat map', () => {
  assert.deepEqual(colorAt(-100), colorAt(-1))
  assert.deepEqual(colorAt(100), colorAt(2.5))
  assert.ok(colorAt(-1)[2] > colorAt(-1)[0], 'low points are blue')
  assert.ok(colorAt(2.5)[0] > colorAt(2.5)[2], 'high points are red')
  assert.deepEqual(colorAt(8, 8, 8), colorAt(0.75))
  assert.deepEqual(cloudHeightRange([1, 2, -10, 4, 5, 30, NaN, 0, 999]), { min: -10, max: 30 })
  assert.equal(cloudHeightRange([]), null)
})

test('live height display uses map Z / Three Y, preserves sensor RGB, and reuses colors', () => {
  const sourceColors = new Float32Array([1, 0, 1, 1, 0, 1])
  const cloud = {
    positions: new Float32Array([100, -1, -900, -100, 2.5, 900]),
    colors: sourceColors, count: 2,
  } as BinaryCloud
  const scene = new THREE.Scene()
  const points = upsertLiveCloudLayer(scene, null, cloud, 0.1, { heightColors: true, color: 0xffffff })!
  const colors = points.geometry.getAttribute('color') as THREE.BufferAttribute
  assert.deepEqual(Array.from(colors.array), [...colorAt(-1), ...colorAt(2.5)])
  assert.deepEqual(Array.from(sourceColors), [1, 0, 1, 1, 0, 1])
  cloud.positions[1] = 0.75
  const updated = upsertLiveCloudLayer(scene, points, cloud, 0.1, { heightColors: true })!
  assert.equal(updated.geometry.getAttribute('color'), colors)
  assert.deepEqual(Array.from(colors.array.slice(0, 3)), colorAt(0.75))
  assert.deepEqual(Array.from(sourceColors), [1, 0, 1, 1, 0, 1])
})

test('saved-map color scale covers all floors and stays unchanged when height clipping changes', () => {
  const source = [0, 0, -10, 1, 0, 10, 2, 0, 30]
  const all = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)!
  const clipped = createSavedMapLayer(source, 5, 15)!
  const allColors = all.geometry.getAttribute('color').array
  assert.deepEqual(Array.from(allColors.slice(0, 3)), colorAt(-10, -10, 30))
  assert.deepEqual(Array.from(allColors.slice(6, 9)), colorAt(30, -10, 30))
  assert.deepEqual(Array.from(clipped.geometry.getAttribute('color').array), Array.from(allColors.slice(3, 6)))
  const flat = createSavedMapLayer([0, 0, 8, 2, 4, 8], SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)!
  const flatColors = flat.geometry.getAttribute('color').array
  assert.deepEqual(Array.from(flatColors.slice(0, 3)), Array.from(flatColors.slice(3, 6)))
  assert.ok(Array.from(flatColors).every(Number.isFinite))
})

test('height mode takes precedence over supplied semantic labels; semantics remain an explicit option', () => {
  const source = [0, 0, -10, 1, 0, 30]
  const semantic = { layers: [{ type: 'pointcloud', labels: [1, 1], palette: { '1': { color: '#ff00ff' } } }] } as MapSceneEvent
  const height = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL, semantic)!
  assert.deepEqual(Array.from(height.geometry.getAttribute('color').array), [...colorAt(-10, -10, 30), ...colorAt(30, -10, 30)])
  const labels = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL, semantic, 0.1, 'semantic')!
  assert.deepEqual(Array.from(labels.geometry.getAttribute('color').array), [1, 0, 1, 1, 0, 1])
})

test('switching an existing RGB layer to height colors does not mutate source RGB', () => {
  const rgb = new Float32Array([1, 0, 1])
  const cloud = { positions: new Float32Array([0, -1, 0]), colors: rgb, count: 1 } as BinaryCloud
  const scene = new THREE.Scene()
  const points = upsertLiveCloudLayer(scene, null, cloud, 0.1)!
  upsertLiveCloudLayer(scene, points, cloud, 0.1, { heightColors: true })
  assert.deepEqual(Array.from(points.geometry.getAttribute('color').array), colorAt(-1))
  assert.deepEqual(Array.from(rgb), [1, 0, 1])
})

test('display percentiles ignore isolated extreme heights without changing map bounds', () => {
  const points = [-100, ...Array.from({ length: 100 }, (_, i) => i / 100), 100].flatMap(z => [0, 0, z])
  const range = cloudColorRange(points)!
  assert.ok(range.min >= 0 && range.max <= 1)
  assert.deepEqual(cloudHeightRange(points), { min: -100, max: 100 })
  assert.deepEqual(cloudColorRange([0, 0, 8, 0, 0, 8]), { min: 8, max: 8 })
  assert.deepEqual(cloudColorRange([0, 0, 1, .05, 0, 0, 2, .05], 4), { min: 1, max: 2 })
  assert.equal(cloudColorRange([]), null)
})

test('saved point and voxel colors span the main heights while retaining isolated extremes', () => {
  const heights = [-100, ...Array.from({ length: 101 }, (_, i) => i / 25), 100]
  const source = heights.flatMap((z, i) => [i / 5, 0, z])
  const original = [...source]
  const range = cloudColorRange(source)!
  const points = createSavedMapLayer(source, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)!
  const cells = new Float32Array(heights.flatMap((z, i) => [i / 5, 0, z, .1]))
  const mesh = createOctomapLayer({ resolution: .1, cells,
    bounds: { min: [0, 0, -100], max: [11, 1, 100] } }, 0, 0, 'height', 1, Infinity)!
  assert.equal(points.geometry.getAttribute('position').count, heights.length)
  assert.equal(mesh.count, heights.length)
  assert.deepEqual(source, original)
  const colors = points.geometry.getAttribute('color').array
  assert.deepEqual(Array.from(colors.slice(0, 3)), colorAt(range.min, range.min, range.max))
  const unique = new Set(heights.filter(z => z >= 0 && z <= 4)
    .map(z => colorAt(z, range.min, range.max).map(v => v.toFixed(2)).join(',')))
  assert.ok(unique.size > 90, 'main structure uses a continuous multicolor ramp')
  for (let i = 0; i < mesh.count; i++) {
    const color = new THREE.Color()
    mesh.getColorAt(i, color)
    const expected = colorAt(cells[i * 4 + 2], range.min, range.max)
    assert.ok(color.toArray().every((v, channel) => Math.abs(v - expected[channel]) < .15))
  }
  points.geometry.dispose(); (points.material as THREE.Material).dispose()
  mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose(); mesh.dispose()
})

test('CPU and GPU use the same height palette', () => {
  for (let i = 0; i < HEIGHT_COLORS.length; i++) {
    const cpu = colorAt(i, 0, HEIGHT_COLORS.length - 1)
    assert.ok(cpu.every((value, channel) => Math.abs(value - HEIGHT_COLORS[i][channel]) < 1e-6))
    assert.ok(CLOUD_HEIGHT_GLSL.includes(`vec3 h${i} = vec3(${HEIGHT_COLORS[i].map(v => v.toFixed(2)).join(', ')});`))
  }
})

test('GPU height colors compose with fading and cube edges; range changes reuse geometry and shaders', () => {
  const points = createSavedMapLayer([0, 0, 0, 0, 0, 2], SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)!
  updateCloudHeightRange(points, -1, 1, 'fade')
  const mesh = createOctomapLayer({ resolution: .05, cells: new Float32Array([.025, .025, .025, .05]),
    bounds: { min: [0, 0, 0], max: [.05, .05, .05] } }, 0, 0, 'height')!
  for (const object of [points, mesh]) {
    const material = object.material as THREE.Material
    const geometry = object.geometry
    setHeightColorRange(material, -1, 1, true)
    const source = object === points ? THREE.ShaderLib.points : THREE.ShaderLib.basic
    const shader = { uniforms: {}, vertexShader: source.vertexShader, fragmentShader: source.fragmentShader } as Parameters<THREE.Material['onBeforeCompile']>[0]
    material.onBeforeCompile(shader, {} as THREE.WebGLRenderer)
    const uniform = shader.uniforms.heightColorRange.value as THREE.Vector3
    assert.deepEqual(uniform.toArray(), [-1, 1, 1])
    assert.match(shader.vertexShader, /colorPosition = instanceMatrix \* colorPosition/)
    assert.match(shader.vertexShader, /colorWorldHeight = \(modelMatrix \* colorPosition\)\.y/)
    assert.match(shader.fragmentShader, object === points ? /diffuseColor\.a \*=/ : /diffuseColor\.rgb \*= 0\.55/)
    const version = material.version, key = material.customProgramCacheKey()
    setHeightColorRange(material, 2, 4, false)
    assert.equal(shader.uniforms.heightColorRange.value, uniform)
    assert.deepEqual(uniform.toArray(), [2, 4, 0])
    assert.equal(material.version, version)
    assert.equal(material.customProgramCacheKey(), key)
    assert.equal(object.geometry, geometry)
    geometry.dispose(); material.dispose()
  }
  mesh.dispose()
})
