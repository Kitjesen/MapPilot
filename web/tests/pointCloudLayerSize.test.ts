import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import * as THREE from 'three'

import type { BinaryCloud } from '../src/hooks/useBinaryCloud.ts'
import { upsertLiveCloudLayer } from '../src/components/scene3d/layers/liveCloudLayer.ts'
import {
  createSavedMapLayer,
  SAVED_MAP_Z_CEIL,
  SAVED_MAP_Z_FLOOR,
  updateSavedMapPointSize,
} from '../src/components/scene3d/layers/savedMapLayer.ts'

const scene3dSource = readFileSync(
  new URL('../src/components/Scene3D.tsx', import.meta.url),
  'utf8',
)

const cloud: BinaryCloud = {
  positions: new Float32Array([0, 0, 0]),
  colors: new Float32Array([0.2, 0.8, 1]),
  count: 1,
  seq: 1,
  protocolVersion: null,
  frameId: null,
  epoch: null,
  stampS: null,
  sequence: null,
  streamKind: null,
  connected: true,
  transport: 'ws',
  lastFrameAt: 1,
  error: null,
}

function materialSize(points: THREE.Points | null): number {
  assert.ok(points)
  assert.ok(!Array.isArray(points.material))
  return (points.material as THREE.PointsMaterial).size
}

function assertClearlyAdjustable(small: number, large: number, layer: string): void {
  assert.ok(
    large / small >= 8,
    `${layer} point size should have a clearly visible range; got ${small} -> ${large}`,
  )
}

test('one point-size control visibly resizes accumulated, current-scan, and saved-map clouds', () => {
  const liveScene = new THREE.Scene()
  const live = upsertLiveCloudLayer(liveScene, null, cloud, 0.02, { pointSizeScale: 0.9 })
  const liveSmall = materialSize(live)
  const resizedLive = upsertLiveCloudLayer(liveScene, live, cloud, 0.4, { pointSizeScale: 0.9 })
  assert.equal(resizedLive, live, 'the accumulated cloud should resize in place')
  assertClearlyAdjustable(liveSmall, materialSize(resizedLive), 'accumulated cloud')

  const scanScene = new THREE.Scene()
  const scan = upsertLiveCloudLayer(scanScene, null, cloud, 0.02, { pointSizeScale: 1.18 })
  const scanSmall = materialSize(scan)
  const resizedScan = upsertLiveCloudLayer(scanScene, scan, cloud, 0.4, { pointSizeScale: 1.18 })
  assert.equal(resizedScan, scan, 'the current scan should resize in place')
  assertClearlyAdjustable(scanSmall, materialSize(resizedScan), 'current scan')

  const saved = createSavedMapLayer([0, 0, 0], -1, 1, undefined, 0.02)
  const savedSmall = materialSize(saved)
  updateSavedMapPointSize(saved, 0.4)
  assertClearlyAdjustable(savedSmall, materialSize(saved), 'saved map')
})

test('saved-map layer keeps outdoor height and applies height colors without semantics', () => {
  const saved = createSavedMapLayer(
    [0, 0, -2, 1, 0, 4, 2, 0, 12],
    SAVED_MAP_Z_FLOOR,
    SAVED_MAP_Z_CEIL,
  )

  assert.ok(saved)
  assert.equal(saved.geometry.getAttribute('position').count, 3)
  const colors = saved.geometry.getAttribute('color')
  assert.equal(colors.count, 3)
  assert.notDeepEqual(
    Array.from((colors as THREE.BufferAttribute).array.slice(0, 3)),
    Array.from((colors as THREE.BufferAttribute).array.slice(6, 9)),
  )
})

test('periodic map-scene events do not rebuild the saved-map GPU layer', () => {
  assert.doesNotMatch(scene3dSource, /\[savedMapFlat,\s*mapScene\]/)
  assert.match(
    scene3dSource,
    /createSavedMapLayer\([\s\S]*?SAVED_MAP_Z_CEIL,\s*undefined,\s*pointSizeRef\.current/,
  )
})
