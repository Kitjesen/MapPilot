import assert from 'node:assert/strict'
import test from 'node:test'

import { PCD_POINT_SIZE_CSS, pickVisiblePcdPoint, projectPcdPoint } from '../src/services/pcdPicking.ts'

// Orthographic view from above: map X/Y stay on screen, higher Z is nearer.
const topView = new Float32Array([
  1, 0, 0, 0,
  0, 0, -1, 0,
  0, 1, 0, 0,
  0, 0, 0, 1,
])

function pick(points: number[], dpr = 1, x = 50, y = 50) {
  return pickVisiblePcdPoint(new Float32Array(points), topView, 100 * dpr, 100 * dpr,
    x * dpr, y * dpr, 18 * dpr, PCD_POINT_SIZE_CSS * dpr)
}

test('overlapping floors select the visible upper floor regardless of PCD order or DPR', () => {
  for (const dpr of [1, 1.5, 2, 3]) {
    for (const heights of [[-0.5, 0.5], [0.5, -0.5]]) {
      assert.deepEqual(pick(heights.flatMap(z => [0, 0, z]), dpr), { x: 0, y: 0, z: 0.5 })
    }
  }
})

test('a nearer point away from the cursor cannot steal the clicked visible point', () => {
  for (const dpr of [1, 2]) {
    for (const points of [[0, 0, -0.5, 0.2, 0, 0.5], [0.2, 0, 0.5, 0, 0, -0.5]]) {
      assert.deepEqual(pick(points, dpr), { x: 0, y: 0, z: -0.5 })
    }
  }
})

test('a rear point remains selectable on its visible edge but not through a front point', () => {
  const points = [0, 0, -0.5, 0.02, 0, 0.5]
  assert.equal(pick(points, 1, 49.5, 50.5)?.z, -0.5)
  assert.equal(pick(points, 1, 50.5, 50.5)?.z, 0.5)
})

test('the circular shader cutout does not occlude a rear point through an empty square corner', () => {
  // Front projects to (49.5, 49.5); (50.5, 50.5) is outside its radius 1.1 disc.
  const points = [-0.01, -0.01, 0.5, 0.01, 0.01, -0.5]
  assert.equal(pick(points, 1, 50.5, 50.5)?.z, -0.5)
})

test('point coverage uses the rendered device-pixel size', () => {
  const points = new Float32Array([0, 0, 0])
  const large = pickVisiblePcdPoint(points, topView, 200, 200, 98.5, 99.5, 0.1, PCD_POINT_SIZE_CSS * 2)
  const small = pickVisiblePcdPoint(points, topView, 200, 200, 98.5, 99.5, 0.1, PCD_POINT_SIZE_CSS)
  assert.deepEqual(large, { x: 0, y: 0, z: 0 })
  assert.equal(small, null)
})

const near = 0.1, far = 10
const perspective = new Float32Array([
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, -(far + near) / (far - near), -1,
  0, 0, -2 * near * far / (far - near), 0,
])

test('perspective overlap uses projected depth rather than map height or file order', () => {
  for (const points of [[0.125, 1, 0.25, 0.25, 2, 0.5], [0.25, 2, 0.5, 0.125, 1, 0.25]]) {
    const result = pickVisiblePcdPoint(new Float32Array(points), perspective, 100, 100,
      56.25, 37.5, 18, PCD_POINT_SIZE_CSS)
    assert.deepEqual(result, { x: 0.125, y: 1, z: 0.25 })
  }
})

test('points outside the camera clip volume cannot become pick targets', () => {
  for (const point of [
    { x: 0, y: -1, z: 0 },
    { x: 0, y: 0.05, z: 0 },
    { x: 0, y: 11, z: 0 },
    { x: 2, y: 1, z: 0 },
    { x: 0, y: 1, z: 2 },
    { x: 0, y: 1, z: Number.NaN },
  ]) {
    assert.equal(projectPcdPoint(perspective, point, 100, 100), null)
    assert.equal(pickVisiblePcdPoint(new Float32Array([point.x, point.y, point.z]), perspective,
      100, 100, 50, 50, 18, PCD_POINT_SIZE_CSS), null)
  }
})

test('empty regions and far-plane fragments do not produce picks', () => {
  assert.equal(pick([]), null)
  assert.equal(pick([0.6, 0, 0]), null)
  assert.equal(pick([0, 0, -1]), null)
})
