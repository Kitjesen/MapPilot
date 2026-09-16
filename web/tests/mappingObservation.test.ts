import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'

import {
  MAPPING_OBSERVATION_COLORS,
  mappingObservationPointAt,
  resolveMappingObservation,
} from '../src/services/mappingObservation.ts'
import { createMappingObservationLayer } from '../src/components/scene3d/layers/mappingObservationLayer.ts'
import type { MapSceneEvent } from '../src/types/index.ts'

function observationScene(
  values = [-1, 0, 100, 100, -1, 0],
  overrides: Record<string, unknown> = {},
): MapSceneEvent {
  return {
    type: 'map_scene',
    frame_id: 'map',
    ts: 100,
    sequence: 8,
    metadata: {
      producer_boot_id: 'mapd-a', reset_epoch: '2', generation: 4, observation_sequence: 8, live: true,
    },
    layers: [{
      id: 'maps.surface_projection', type: 'grid', frame_id: 'map',
      producer_boot_id: 'mapd-a', reset_epoch: '2', generation: 4, observation_sequence: 8, live: true,
      stamp_s: 100, rows: 2, cols: 3, resolution: 0.5, origin: [10, 20, 0.2], yaw: 0,
      encoding: 'int8', value_semantics: 'ground_relative_surface_not_traversability', scope: 'rolling_window',
      downsample_factor: 1, grid_b64: Buffer.from(values).toString('base64'),
      unknown_count: values.filter(value => value === -1).length,
      free_count: values.filter(value => value === 0).length,
      occupied_count: values.filter(value => value === 100).length,
      ...overrides,
    }],
  }
}

test('observation decoding preserves unknown, free and occupied with exact counts and reuses decoded cells', () => {
  const scene = observationScene()
  const state = resolveMappingObservation(scene, { nowS: 101 })
  assert.equal(state.status, 'ready')
  if (state.status !== 'ready') return
  assert.deepEqual(Array.from(state.cells), [-1, 0, 100, 100, -1, 0])
  assert.deepEqual([state.unknownCount, state.freeCount, state.occupiedCount], [2, 2, 2])
  assert.match(state.message, /1.5 × 1.0 m/)
  assert.doesNotMatch(state.message, /%|完成率|可通行/)
  const refreshed = resolveMappingObservation(scene, { nowS: 102 })
  assert.equal(refreshed.status, 'ready')
  if (refreshed.status === 'ready') assert.equal(refreshed.cells, state.cells)
})

test('an entirely unknown window stays unknown instead of becoming free or disappearing', () => {
  const state = resolveMappingObservation(observationScene(Array(6).fill(-1)), { nowS: 101 })
  assert.equal(state.status, 'ready')
  if (state.status !== 'ready') return
  assert.equal(state.unknownCount, 6)
  assert.equal(state.freeCount, 0)
  assert.equal(mappingObservationPointAt(state, 10.25, 20.25)?.value, -1)
  assert.match(mappingObservationPointAt(state, 10.25, 20.25)?.label ?? '', /未确认/)
})

test('matched local-surface diagnostics appear only for the current projection cell', () => {
  const scene = observationScene()
  const diagnostics = [
    ['maps.ground_height', 'local_surface_fit_height_m', [0.1, 0.2, Number.NaN, 0.4, 0.5, 0.6]],
    ['maps.ground_roughness', 'local_surface_fit_residual_rms_m', [0.01, 0.02, Number.NaN, 0.04, 0.05, 0.06]],
    ['maps.ground_support', 'distinct_fine_xy_support_count', [3, 4, 0, 6, 7, 8]],
  ] as const
  scene.layers.push(...diagnostics.map(([id, value_semantics, values]) => ({
    id, type: 'grid', frame_id: 'map', producer_boot_id: 'mapd-a',
    reset_epoch: '2', generation: 4, observation_sequence: 8, live: true,
    stamp_s: 100, rows: 2, cols: 3, resolution: 0.5, origin: [10, 20, 0.2], yaw: 0,
    encoding: 'float32_le', value_semantics, scope: 'rolling_window', downsample_factor: 1,
    valid_count: values.filter(Number.isFinite).length,
    grid_b64: Buffer.from(new Float32Array(values).buffer).toString('base64'),
  })))
  const state = resolveMappingObservation(scene, { nowS: 101 })
  assert.equal(state.status, 'ready')
  if (state.status !== 'ready') return
  assert.deepEqual(mappingObservationPointAt(state, 10.75, 20.25)?.ground, {
    heightM: 0.20000000298023224, roughnessM: 0.019999999552965164, supportCount: 4,
  })
  assert.equal(mappingObservationPointAt(state, 11.25, 20.25)?.ground, undefined)
})

test('missing, non-live and stale observations clear the raster rather than retaining an older grid', () => {
  const scene = observationScene()
  assert.equal(resolveMappingObservation(scene, { nowS: 101 }).status, 'ready')
  assert.equal(resolveMappingObservation(null, { nowS: 101 }).status, 'unavailable')
  assert.equal(resolveMappingObservation({ ...scene, layers: [] }, { nowS: 101 }).status, 'unavailable')
  assert.equal(resolveMappingObservation(observationScene(undefined, { grid_b64: undefined }), { nowS: 101 }).status, 'unavailable')
  assert.equal(resolveMappingObservation(observationScene(undefined, { live: false }), { nowS: 101 }).status, 'unavailable')
  const stale = resolveMappingObservation(scene, { nowS: 106 })
  assert.equal(stale.status, 'stale')
  assert.equal(createMappingObservationLayer(stale), null)
  assert.equal(mappingObservationPointAt(stale, 10.25, 20.25), null)
  assert.equal(resolveMappingObservation(scene, { nowS: 103, maxAgeS: 2 }).status, 'stale')
})

test('producer, reset, generation and observation identities must belong to the current scene', () => {
  for (const override of [
    { producer_boot_id: 'mapd-old' }, { producer_boot_id: '' }, { reset_epoch: '1' },
    { generation: 3 }, { observation_sequence: 7 }, { reset_epoch: -1 },
  ]) {
    assert.equal(resolveMappingObservation(observationScene(undefined, override), { nowS: 101 }).status, 'error')
  }
  const resetScene = observationScene(undefined, {
    producer_boot_id: 'mapd-b', reset_epoch: '3', generation: 1, observation_sequence: 1,
  })
  resetScene.metadata = { producer_boot_id: 'mapd-b', reset_epoch: '3', generation: 1, observation_sequence: 1, live: true }
  assert.equal(resolveMappingObservation(resetScene, { nowS: 101 }).status, 'ready')
})

test('uint64 reset epochs remain exact opaque identities across scene and occupancy metadata', () => {
  const liveEpoch = '117269136217079808'
  const scene = observationScene(undefined, { reset_epoch: liveEpoch })
  scene.metadata = { ...scene.metadata, reset_epoch: liveEpoch }
  const state = resolveMappingObservation(scene, { nowS: 101 })
  assert.equal(state.status, 'ready')
  if (state.status === 'ready') assert.equal(state.layer.reset_epoch, liveEpoch)

  scene.metadata = { ...scene.metadata, reset_epoch: '117269136217079809' }
  assert.equal(resolveMappingObservation(scene, { nowS: 101 }).status, 'error')
  scene.metadata = { ...scene.metadata, reset_epoch: 117269136217079808 }
  assert.equal(resolveMappingObservation(scene, { nowS: 101 }).status, 'error')
})

test('legacy reset epochs are accepted only as safe integers and normalized to opaque strings', () => {
  const scene = observationScene(undefined, { reset_epoch: 2 })
  scene.metadata = { ...scene.metadata, reset_epoch: 2 }
  const state = resolveMappingObservation(scene, { nowS: 101 })
  assert.equal(state.status, 'ready')
  if (state.status === 'ready') assert.equal(state.layer.reset_epoch, '2')

  for (const resetEpoch of [-1, 1.5, Number.MAX_SAFE_INTEGER + 1]) {
    assert.equal(resolveMappingObservation(observationScene(undefined, { reset_epoch: resetEpoch }), { nowS: 101 }).status, 'error')
  }
})

test('an explicit omitted grid clears prior data without reporting missing source identity as an error', () => {
  const scene = observationScene()
  assert.equal(resolveMappingObservation(scene, { nowS: 101 }).status, 'ready')
  const omitted = {
    ...scene,
    layers: [{ id: 'maps.surface_projection', payload: 'omitted', retain_previous: false }],
  }
  const state = resolveMappingObservation(omitted, { nowS: 101 })
  assert.deepEqual(state, { status: 'unavailable', message: '观测图暂不可用，等待新数据' })
  assert.equal(createMappingObservationLayer(state), null)
  const invalidInline = observationScene(undefined, {
    payload: 'omitted', retain_previous: false, producer_boot_id: '',
  })
  assert.equal(resolveMappingObservation(invalidInline, { nowS: 101 }).status, 'error')
})

test('coordinate frames and source timestamps must align before displaying occupancy', () => {
  const scene = observationScene()
  assert.equal(resolveMappingObservation(observationScene(undefined, { frame_id: 'odom' }), { nowS: 101 }).status, 'error')
  assert.equal(resolveMappingObservation(scene, { nowS: 101, savedMapFrameId: 'odom' }).status, 'error')
  assert.equal(resolveMappingObservation(scene, { nowS: 101, savedMapFrameId: 'map' }).status, 'ready')
  assert.equal(resolveMappingObservation(scene, { nowS: Number.NaN }).status, 'error')
  assert.equal(resolveMappingObservation(observationScene(undefined, { stamp_s: 103 }), { nowS: 101 }).status, 'error')
})

test('malformed payloads, non-contract values, incorrect counts and invalid geometry are rejected', () => {
  for (const override of [
    { grid_b64: '%%%bad' }, { grid_b64: Buffer.from([0, 100]).toString('base64') },
    { free_count: 3 }, { rows: 0 }, { rows: 131_073, cols: 1 },
    { origin: [0, 0, Number.NaN] }, { resolution: 0 }, { yaw: Number.NaN },
    { encoding: 'uint8' }, { downsample_factor: 0 }, { scope: 'global' },
    { value_semantics: 'traversable' },
  ]) {
    assert.equal(resolveMappingObservation(observationScene(undefined, override), { nowS: 101 }).status, 'error')
  }
  assert.equal(resolveMappingObservation(observationScene([-1, 0, 100, 1, -1, 0]), { nowS: 101 }).status, 'error')
})

test('world lookup applies inverse yaw and never infers cells outside the rolling window', () => {
  const state = resolveMappingObservation(observationScene(undefined, { yaw: Math.PI / 2 }), { nowS: 101 })
  assert.deepEqual(mappingObservationPointAt(state, 9.75, 20.25), {
    row: 0, col: 0, value: -1, label: '未确认 · 缺少地面依据',
  })
  assert.equal(mappingObservationPointAt(state, 9.75, 21.25)?.value, 100)
  assert.equal(mappingObservationPointAt(state, 9.25, 21.25)?.value, 0)
  assert.equal(mappingObservationPointAt(state, 10.25, 20.25), null)
  assert.equal(mappingObservationPointAt(state, 9.25, 21.75), null)
  assert.equal(mappingObservationPointAt(state, Number.NaN, 20), null)
})

test('Three raster preserves each source cell without a subcell pattern and keeps the planning-map XY convention', () => {
  const state = resolveMappingObservation(observationScene(undefined, { yaw: Math.PI / 2 }), { nowS: 101 })
  const mesh = createMappingObservationLayer(state, 0.5)
  assert.ok(mesh)
  assert.equal(mesh.name, 'mapping-observation')
  assert.deepEqual(mesh._group?.position.toArray(), [9.5, 0.5, -20.75])
  assert.equal(mesh.rotation.x, -Math.PI / 2)
  const texture = (mesh.material as THREE.MeshBasicMaterial).map as THREE.DataTexture
  assert.equal(texture.minFilter, THREE.NearestFilter)
  assert.equal(texture.magFilter, THREE.NearestFilter)
  assert.equal(texture.flipY, false)
  assert.equal(texture.generateMipmaps, false)
  const pixels = texture.image.data as Uint8Array
  assert.equal(texture.image.width, 3)
  assert.equal(texture.image.height, 2)
  assert.deepEqual(Array.from(pixels), [-1, 0, 100, 100, -1, 0].flatMap(value =>
    [...MAPPING_OBSERVATION_COLORS[value as -1 | 0 | 100]]))
  texture.dispose()
  mesh.geometry.dispose()
  ;(mesh.material as THREE.Material).dispose()
})


test('raw height-band occupancy must never be displayed as ground-relative obstacles', () => {
  const scene = observationScene(undefined, {
    id: 'maps.occupancy', value_semantics: 'height_band_occupancy_not_traversability',
  })
  assert.equal(resolveMappingObservation(scene, { nowS: 101 }).status, 'unavailable')
})
