import assert from 'node:assert/strict'
import test from 'node:test'

import { resolveNativeTraversabilityLayer, createNativeTraversabilityLayer } from '../src/components/scene3d/layers/traversabilityLayer.ts'
import * as THREE from 'three'
import { estimateSceneTime } from '../src/services/sceneTelemetry.ts'
import type { NativeTraversabilityEvent } from '../src/types/index.ts'

const event: NativeTraversabilityEvent = {
  type: 'native_traversability',
  grid_b64: Buffer.from([0, 25, 75, 100]).toString('base64'),
  rows: 2,
  cols: 2,
  resolution: 0.2,
  origin: [1, -2, 0.4],
  yaw: 0,
  frame_id: 'map',
  stamp_s: 100,
  reset_epoch: 2,
  sequence: 8,
  source: 'native_nav_client',
  control_authority: true,
  value_semantics: 'control_risk_0_100',
  identity_verified: true,
}

test('risk cells retain discrete boundaries and an unlit, transparent-zero palette', () => {
  const state = resolveNativeTraversabilityLayer(event, { nowS: 101, allowedFrameIds: ['map'] })
  const mesh = createNativeTraversabilityLayer(state)!
  const material = mesh.material as THREE.MeshBasicMaterial
  const texture = material.map as THREE.DataTexture
  assert.equal(texture.minFilter, THREE.NearestFilter)
  assert.equal(texture.magFilter, THREE.NearestFilter)
  assert.equal(texture.generateMipmaps, false)
  assert.equal(texture.colorSpace, THREE.SRGBColorSpace)
  assert.equal(material.toneMapped, false)
  assert.equal(material.depthWrite, false)
  const pixels = texture.image.data as Uint8Array
  assert.equal(pixels[3], 0)
  assert.equal(pixels[15], 250)
  assert.equal(pixels[12], 250)
  texture.dispose(); material.dispose(); mesh.geometry.dispose()
})

test('native traversability accepts only fresh map-frame control risk', () => {
  const state = resolveNativeTraversabilityLayer(event, {
    nowS: 101,
    allowedFrameIds: ['map'],
  })
  assert.equal(state.status, 'ready')
  if (state.status === 'ready') assert.deepEqual(Array.from(state.values), [0, 25, 75, 100])
})

test('native traversability fails closed for rotated grids until the nav contract supports them', () => {
  const state = resolveNativeTraversabilityLayer(
    { ...event, yaw: 0.1 },
    { nowS: 101, allowedFrameIds: ['map'] },
  )
  assert.equal(state.status, 'error')
})

test('native traversability hides stale data instead of leaving an old risk mesh', () => {
  const state = resolveNativeTraversabilityLayer(event, {
    nowS: 104,
    allowedFrameIds: ['map'],
  })
  assert.equal(state.status, 'stale')
})

test('Gateway clock offset does not reject fresh risk, and disconnected risk still expires', () => {
  const receivedAtMs = 97_800
  assert.equal(resolveNativeTraversabilityLayer(event, {
    nowS: 97.8, allowedFrameIds: ['map'],
  }).status, 'error')
  assert.equal(resolveNativeTraversabilityLayer(event, {
    nowS: estimateSceneTime(97.8, 100, receivedAtMs), allowedFrameIds: ['map'],
  }).status, 'ready')
  assert.equal(resolveNativeTraversabilityLayer(event, {
    nowS: estimateSceneTime(101.8, 100, receivedAtMs), allowedFrameIds: ['map'],
  }).status, 'stale')
})

test('scene time falls back to local time until a paired snapshot is received', () => {
  assert.equal(estimateSceneTime(100, undefined, null), 100)
  assert.equal(estimateSceneTime(100, 102.2, undefined), 100)
  assert.equal(estimateSceneTime(100, NaN, 100_000), 100)
})
