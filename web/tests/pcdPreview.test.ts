import assert from 'node:assert/strict'
import test from 'node:test'

import { parsePcd, pcdCameraDistance } from '../src/services/pcdPreview.ts'

test('whole-map camera fits all bounding-box corners in wide and narrow viewports', () => {
  const halfSize = [30, 5, 22]
  const radius = Math.hypot(...halfSize)
  const verticalLimit = Math.tan(Math.PI / 8)
  for (const aspect of [2, 1, 0.3, 0.1]) {
    const distance = pcdCameraDistance(radius, aspect)
    for (const [theta, phi] of [[0.4, 1.05], [1.8, 0.7], [-Math.PI / 2, 0]]) {
      const forward = [Math.sin(phi) * Math.cos(theta), Math.cos(phi), Math.sin(phi) * Math.sin(theta)]
      const right = [Math.sin(theta), 0, -Math.cos(theta)]
      const up = [-Math.cos(phi) * Math.cos(theta), Math.sin(phi), -Math.cos(phi) * Math.sin(theta)]
      for (const xSign of [-1, 1]) for (const ySign of [-1, 1]) for (const zSign of [-1, 1]) {
        const corner = [xSign * halfSize[0], ySign * halfSize[1], zSign * halfSize[2]]
        const dot = (axis: number[]) => corner.reduce((sum, value, index) => sum + value * axis[index], 0)
        const depth = distance - dot(forward)
        assert.ok(depth > 0)
        assert.ok(Math.abs(dot(right)) / depth < verticalLimit * aspect)
        assert.ok(Math.abs(dot(up)) / depth < verticalLimit)
      }
    }
  }
})

test('single-point maps retain a visible camera distance and valid clip range', () => {
  for (const aspect of [2, 0.3]) {
    const distance = pcdCameraDistance(0, aspect)
    assert.ok(Number.isFinite(distance))
    assert.ok(distance > 0.05)
    assert.ok(distance * 20 > distance)
  }
})

function pcdFixture(format: 'ascii' | 'binary', count: number): ArrayBuffer {
  const header = new TextEncoder().encode([
    'VERSION .7',
    'FIELDS x y z intensity',
    'SIZE 4 4 4 4',
    'TYPE F F F F',
    'COUNT 1 1 1 1',
    `WIDTH ${count}`,
    'HEIGHT 1',
    `POINTS ${count}`,
    `DATA ${format}\n`,
  ].join('\n'))
  const regionY = (index: number) => index < count / 2 ? -1000 : 1000
  if (format === 'ascii') {
    const lines = Array.from({ length: count }, (_, i) => `${i} ${regionY(i)} 3 42`)
    const body = new TextEncoder().encode(`${lines.join('\n')}\n`)
    const buffer = new ArrayBuffer(header.length + body.length)
    new Uint8Array(buffer).set(header)
    new Uint8Array(buffer, header.length).set(body)
    return buffer
  }

  const buffer = new ArrayBuffer(header.length + count * 16)
  new Uint8Array(buffer).set(header)
  const view = new DataView(buffer, header.length)
  for (let i = 0; i < count; i++) {
    view.setFloat32(i * 16, i, true)
    view.setFloat32(i * 16 + 4, regionY(i), true)
    view.setFloat32(i * 16 + 8, 3, true)
    view.setFloat32(i * 16 + 12, 42, true)
  }
  return buffer
}

for (const [format, budget] of [['ascii', 300_000], ['binary', 500_000]] as const) {
  test(`${format} preview samples both map regions within its point budget`, () => {
    const total = budget * 2 + 7
    const points = parsePcd(pcdFixture(format, total))
    assert.ok(points)
    assert.equal(points.length / 3, budget)
    assert.deepEqual(Array.from(points.slice(0, 3)), [0, -1000, 3])
    assert.deepEqual(Array.from(points.slice(-3)), [total - 1, 1000, 3])

    let tailRegionCount = 0
    let minGap = Infinity
    let maxGap = 0
    for (let i = 0; i < points.length; i += 3) {
      if (points[i + 1] === 1000) tailRegionCount++
      if (i > 0) {
        const gap = points[i] - points[i - 3]
        minGap = Math.min(minGap, gap)
        maxGap = Math.max(maxGap, gap)
      }
    }
    assert.ok(Math.abs(tailRegionCount - budget / 2) <= 1)
    assert.ok(minGap >= 1)
    assert.ok(maxGap <= Math.ceil(total / budget))
  })

  test(`${format} preview preserves all points below its budget`, () => {
    assert.deepEqual(Array.from(parsePcd(pcdFixture(format, 3))!), [
      0, -1000, 3,
      1, -1000, 3,
      2, 1000, 3,
    ])
    assert.deepEqual(Array.from(parsePcd(pcdFixture(format, 1))!), [0, -1000, 3])
  })
}
