import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import * as THREE from 'three'
import { parseOctomap } from '../src/services/octomap.ts'
import { createOctomapLayer, clipOctomapLayer } from '../src/components/scene3d/layers/octomapLayer.ts'
import { pickNavigationPoint } from '../src/components/scene3d/goalPicking.ts'
import type { Octomap } from '../src/services/octomap.ts'

function fixture(): ArrayBuffer {
  const bytes = readFileSync(new URL('./fixtures/native-octomap.ot', import.meta.url))
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength)
}

test('native occupied leaves retain signs, child axes, coarse sizes and omit free leaves', () => {
  const map = parseOctomap(fixture())
  const expected: number[][] = JSON.parse(readFileSync(new URL('./fixtures/native-octomap.json', import.meta.url), 'utf8'))
  assert.equal(map.resolution, 0.1)
  assert.equal(map.cells.length, expected.length * 4)
  expected.flat().forEach((value, i) => assert.ok(Math.abs(map.cells[i] - value) < 1e-7))
  assert.ok(expected.some(cell => cell[3] === 0.2), 'fixture must contain a pruned coarse leaf')
  assert.ok(expected.some(cell => cell[0] < 0 && cell[1] < 0 && cell[2] < 0))
})

test('truncated and binary-thresholded maps fail instead of becoming plausible geometry', () => {
  const bytes = fixture()
  assert.throws(() => parseOctomap(bytes.slice(0, bytes.byteLength - 1)), /长度/)
  assert.throws(() => parseOctomap(new TextEncoder().encode('# Octomap OcTree binary file\n').buffer), /完整概率/)
})

test('voxel drawing keeps native scale, height axis and local bounds; surface selection respects clipping', () => {
  const map = parseOctomap(fixture())
  const mesh = createOctomapLayer(map, 0, 0, 'height')!
  assert.equal(mesh.count, map.cells.length / 4)
  const matrix = new THREE.Matrix4()
  mesh.getMatrixAt(0, matrix)
  assert.ok(Math.abs(matrix.elements[12] - map.cells[0]) < 1e-7)
  assert.ok(Math.abs(matrix.elements[13] - map.cells[2]) < 1e-7)
  assert.ok(Math.abs(matrix.elements[14] + map.cells[1]) < 1e-7)
  assert.ok(Math.abs(matrix.elements[0] - map.cells[3]) < 1e-7)
  assert.equal(createOctomapLayer(map, 100, 100, 'height'), null)
  mesh.updateMatrixWorld()
  const ray = new THREE.Raycaster(new THREE.Vector3(.1, 2, -.1), new THREE.Vector3(0, -1, 0))
  const hit = pickNavigationPoint(ray, [], 0, 'surface', mesh)
  assert.ok(hit && Math.abs(hit.y - .2) < 1e-7)
  clipOctomapLayer(mesh, null, 0.05)
  assert.equal(pickNavigationPoint(ray, [], 0, 'surface', mesh), null)
  mesh.dispose(); mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose()
})

test('display aggregation aligns negative coordinates, keeps coarse leaves and never invents picking surfaces', () => {
  const map: Octomap = { resolution: .05,
    cells: new Float32Array([.025, .025, .025, .05, .075, .025, .025, .05, -.025, .025, .025, .05, .3, .3, .3, .2]),
    bounds: { min: [-.05, 0, 0], max: [.4, .4, .4] } }
  const original = map.cells.slice()
  const mesh = createOctomapLayer(map, 0, 0, 'height', 2)!
  assert.equal(mesh.count, 3, 'two fine leaves share their parent; negative X stays separate')
  const matrix = new THREE.Matrix4()
  mesh.getMatrixAt(0, matrix)
  assert.ok(Math.abs(matrix.elements[12] - .05) < 1e-7)
  assert.ok(Math.abs(matrix.elements[0] - .1) < 1e-7)
  mesh.getMatrixAt(1, matrix)
  assert.ok(Math.abs(matrix.elements[12] + .05) < 1e-7)
  mesh.getMatrixAt(2, matrix)
  assert.ok(Math.abs(matrix.elements[0] - .2) < 1e-7, 'native coarse leaves never shrink')
  mesh.updateMatrixWorld()
  const ray = new THREE.Raycaster(new THREE.Vector3(.025, 2, -.025), new THREE.Vector3(0, -1, 0))
  const hit = pickNavigationPoint(ray, [], 0, 'surface', mesh)
  assert.ok(hit && Math.abs(hit.y - .05) < 1e-7, 'pick native 5 cm top, not display 10 cm top')
  ray.ray.origin.set(.025, 2, -.075)
  assert.equal(pickNavigationPoint(ray, [], 0, 'surface', mesh), null, 'empty part of parent cube is not measured ground')
  assert.deepEqual(map.cells, original)
  mesh.dispose(); mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose()
})

test('saved-map viewing includes occupied leaves beyond the live local window', () => {
  const map: Octomap = { resolution: .1, cells: new Float32Array([.05, .05, .05, .1, 30.05, .05, .05, .1]),
    bounds: { min: [0, 0, 0], max: [30.1, .1, .1] } }
  const local = createOctomapLayer(map, 0, 0, 'height')!
  const full = createOctomapLayer(map, 0, 0, 'height', 1, Infinity)!
  assert.equal(local.count, 1)
  assert.equal(full.count, 2)
  assert.equal((full.userData.nativeIndices as number[]).length, map.cells.length / 4)
  for (const mesh of [local, full]) {
    mesh.dispose(); mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose()
  }
})
