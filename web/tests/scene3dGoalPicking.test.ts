import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import * as THREE from 'three'
import { pickNavigationPoint } from '../src/components/scene3d/goalPicking.ts'
import { updateCloudHeightRange } from '../src/components/scene3d/layers/layerUtils.ts'

function cloud(...positions: number[]) {
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
  const points = new THREE.Points(geometry, new THREE.PointsMaterial())
  points.updateMatrixWorld()
  return points
}

function ray() {
  return new THREE.Raycaster(new THREE.Vector3(0, 5, 5), new THREE.Vector3(0, -1, -1).normalize())
}

test('ordinary clicks ignore the nearer ceiling/wall return and keep continuous body height', () => {
  const points = cloud(0, 3, 3, 0, .4, .4)
  assert.ok(ray().intersectObject(points)[0].point.y > 2.9, 'reproduce the old first-hit selection')
  for (const height of [.42, .0184659, -.12]) {
    const picked = pickNavigationPoint(ray(), [points], height)
    assert.ok(picked)
    assert.ok(Math.abs(picked.y - height) < 1e-12)
    assert.ok(Math.abs(picked.z - height) < 1e-12)
  }
})

test('explicit 3D selection preserves an observed raised surface', () => {
  const picked = pickNavigationPoint(ray(), [cloud(0, 1.25, 1.25)], .42, 'surface')
  assert.ok(picked)
  assert.deepEqual(picked.toArray(), [0, 1.25, 1.25])
})

test('3D selection returns the transformed vertex rather than a nearby point on the ray', () => {
  const points = cloud(0, 1.25, 1.25)
  points.position.x = .08
  points.updateMatrixWorld()
  const picked = pickNavigationPoint(ray(), [points], .42, 'surface')
  assert.ok(picked)
  assert.ok(Math.abs(picked.x - .08) < 1e-12)
  assert.equal(picked.y, 1.25)
})

test('surface picking skips a clipped ceiling and can select the visible floor behind it', () => {
  const points = cloud(0, 3, 3, 0, 1, 1)
  updateCloudHeightRange(points, null, 1)
  assert.deepEqual(pickNavigationPoint(ray(), [points], .42, 'surface')?.toArray(), [0, 1, 1])

  updateCloudHeightRange(points, null, null)
  assert.deepEqual(pickNavigationPoint(ray(), [points], .42, 'surface')?.toArray(), [0, 3, 3])
})

test('surface clipping uses the observed world vertex rather than ray height or local height', () => {
  const points = cloud(0, .5, .5)
  points.position.set(0, .56, .5)
  points.updateMatrixWorld()
  // The closest point on the diagonal ray is below 1.04, but the vertex is above it.
  updateCloudHeightRange(points, null, 1.04)
  assert.equal(pickNavigationPoint(ray(), [points], .42, 'surface'), null)
})

test('surface picking excludes the lower floor as the displayed height band moves upstairs', () => {
  const points = cloud(0, 0, 0, 0, 2, 2, 0, 4, 4)
  const fromBelow = new THREE.Raycaster(new THREE.Vector3(0, -5, -5), new THREE.Vector3(0, 1, 1).normalize())
  updateCloudHeightRange(points, 1, 3)
  assert.deepEqual(pickNavigationPoint(fromBelow, [points], .42, 'surface')?.toArray(), [0, 2, 2])
  updateCloudHeightRange(points, 3, 5)
  assert.deepEqual(pickNavigationPoint(fromBelow, [points], .42, 'surface')?.toArray(), [0, 4, 4])
  updateCloudHeightRange(points, null, null)
  assert.deepEqual(pickNavigationPoint(fromBelow, [points], .42, 'surface')?.toArray(), [0, 0, 0])
})

test('surface picking skips fully faded context and keeps the visible transition until fixed clipping', () => {
  const points = cloud(0, 3, 3, 0, 1.5, 1.5, 0, 1, 1)
  updateCloudHeightRange(points, 0, 1, 'fade')
  assert.deepEqual(pickNavigationPoint(ray(), [points], .42, 'surface')?.toArray(), [0, 1.5, 1.5])
  updateCloudHeightRange(points, 0, 1, 'clip')
  assert.deepEqual(pickNavigationPoint(ray(), [points], .42, 'surface')?.toArray(), [0, 1, 1])
  updateCloudHeightRange(points, null, null, 'fade')
  assert.deepEqual(pickNavigationPoint(ray(), [points], .42, 'surface')?.toArray(), [0, 3, 3])
})

test('an entirely clipped cloud allows plane picking and proposes no surface target', () => {
  const points = cloud(0, 3, 3)
  updateCloudHeightRange(points, null, -.5)
  const picked = pickNavigationPoint(ray(), [points], .42, 'plane')
  assert.ok(picked)
  assert.ok(Math.abs(picked.y - .42) < 1e-12)
  assert.equal(pickNavigationPoint(ray(), [points], .42, 'surface'), null)
})

test('surface misses propose no target while plane picking retains the current body height', () => {
  assert.equal(pickNavigationPoint(ray(), [], .42, 'surface'), null)
  assert.equal(pickNavigationPoint(ray(), [cloud(1, 1, 1)], .42, 'surface'), null)
  const picked = pickNavigationPoint(ray(), [], .42, 'plane')
  assert.ok(picked)
  assert.ok(Math.abs(picked.y - .42) < 1e-12)
  const upward = new THREE.Raycaster(new THREE.Vector3(0, 5, 5), new THREE.Vector3(0, 1, 0))
  assert.equal(pickNavigationPoint(upward, [], .42), null)
  assert.equal(pickNavigationPoint(upward, [], .42, 'surface'), null)
})

test('scene uses explicit surface mode, keeps mapping/Shift semantics, and forwards all three coordinates', () => {
  const source = readFileSync(new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8')
  assert.match(source, /goalPickMode = 'plane'/)
  assert.match(source, /mappingObservationVisible \? observationDisplayZ \?\? robotZ : robotZ/)
  assert.match(source, /shiftDown \|\| down\.alt \|\| e\.altKey \? 'surface' : goalPickMode/)
  assert.match(source, /pickNavigationPoint\(raycaster\.current, pickableClouds, pickingZ, mode, octomapRef\.current\)/)
  assert.match(source, /onRelocalize\(wx, wy\)/)
  assert.match(source, /onPendingGoal\?\.\(wx, wy, wz, mode\)/)
})
