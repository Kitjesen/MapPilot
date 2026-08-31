import assert from 'node:assert/strict'
import test from 'node:test'

import * as THREE from 'three'

import {
  createElevationLayer,
  resolveElevationLayer,
} from '../src/components/scene3d/layers/elevationLayer.ts'
import { mergeMapSceneElevation } from '../src/services/mapSceneState.ts'
import type { MapSceneEvent } from '../src/types/index.ts'

function float32Base64(values: number[]): string {
  const bytes = Buffer.alloc(values.length * 4)
  values.forEach((value, index) => bytes.writeFloatLE(value, index * 4))
  return bytes.toString('base64')
}

function elevationScene(
  values: number[],
  overrides: Record<string, unknown> = {},
): MapSceneEvent {
  const finite = values.filter(Number.isFinite)
  return {
    type: 'map_scene',
    schema_version: 'map.scene_frame',
    source: 'mapd',
    frame_id: 'map',
    ts: 100,
    sequence: 8,
    metadata: {
      producer_boot_id: 'mapd-boot-a',
      generation: 4,
      reset_epoch: 2,
      observation_sequence: 8,
      live: true,
    },
    layers: [{
      id: 'maps.elevation',
      type: 'grid',
      topic: '/maps/elevation',
      source: 'mapd',
      frame_id: 'map',
      producer_boot_id: 'mapd-boot-a',
      stamp_s: 100,
      generation: 4,
      reset_epoch: 2,
      observation_sequence: 8,
      live: true,
      grid_b64: float32Base64(values),
      rows: 2,
      cols: 2,
      resolution: 0.5,
      origin: [10, 20, 42],
      yaw: 0,
      encoding: 'float32_le',
      valid_count: finite.length,
      min_z: Math.min(...finite),
      max_z: Math.max(...finite),
      downsample_factor: 1,
      value_semantics: 'min_observed_z_not_ground',
      ...overrides,
    }],
  }
}

test('elevation grid becomes a height-displaced Three.js surface in the map frame', () => {
  const state = resolveElevationLayer(elevationScene([-0.5, 0, 0.5, 1]), {
    nowS: 101,
    savedMapFrameId: 'map',
  })
  assert.equal(state.status, 'ready')

  const mesh = createElevationLayer(state)
  assert.ok(mesh)
  assert.equal(mesh.name, 'elevation-surface')
  assert.deepEqual(
    Array.from(mesh.geometry.getAttribute('position').array),
    [-0.25, -0.5, 0.25, 0.25, 0, 0.25, -0.25, 0.5, -0.25, 0.25, 1, -0.25],
  )
  assert.equal(mesh.geometry.getIndex()?.count, 6)

  const group = mesh._group as THREE.Group
  assert.deepEqual(group.position.toArray(), [10.5, 0, -20.5])
})

test('elevation surface fails closed when its frame does not match the scene', () => {
  const state = resolveElevationLayer(
    elevationScene([0, 0, 0, 0], { frame_id: 'odom' }),
    { nowS: 101, savedMapFrameId: 'map' },
  )

  assert.equal(state.status, 'error')
  assert.match(state.message, /frame/i)
  assert.equal(createElevationLayer(state), null)
})

test('metadata-only map scene updates retain the latest aligned elevation payload', () => {
  const previous = elevationScene([0, 0.25, 0.5, 0.75])
  const incoming: MapSceneEvent = {
    ...previous,
    ts: 101,
    sequence: 9,
    metadata: {
      producer_boot_id: 'mapd-boot-a',
      generation: 5,
      reset_epoch: 2,
      observation_sequence: 9,
      live: true,
    },
    layers: [{
      id: 'maps.elevation',
      type: 'grid',
      frame_id: 'map',
      producer_boot_id: 'mapd-boot-a',
      reset_epoch: 2,
      generation: 5,
      rows: 2,
      cols: 2,
      resolution: 0.5,
      origin: [10, 20, 42],
      yaw: 0,
      downsample_factor: 1,
      payload: 'metadata',
      reason: 'raster_rate_limited',
      retain_previous: true,
      retention_scope: 'same_elevation_cohort',
    }],
  }
  previous.metadata = {
    producer_boot_id: 'mapd-boot-a',
    generation: 4,
    reset_epoch: 2,
    observation_sequence: 8,
    live: true,
  }

  const merged = mergeMapSceneElevation(previous, incoming)
  assert.equal(merged.metadata?.generation, 5)
  const elevation = merged.layers.find(layer => layer.id === 'maps.elevation')
  assert.equal(elevation?.grid_b64, previous.layers[0].grid_b64)
  assert.equal(elevation?.generation, 4)
  assert.equal(elevation?.stamp_s, 100)
  assert.equal(elevation?.payload_retained, true)
  assert.equal(elevation?.retained_for_generation, 5)
  assert.equal(resolveElevationLayer(merged, { nowS: 102, savedMapFrameId: 'map' }).status, 'ready')
  assert.match(resolveElevationLayer(merged, { nowS: 102, savedMapFrameId: 'map' }).message, /保留上一帧/)
})

test('gateway-shaped sparse scene metadata uses the elevation layer identity atomically', () => {
  const previous = elevationScene([-0.5, 0, 0.5, 1])
  previous.metadata = { producer_boot_id: 'mapd-boot-a' }
  const direct = resolveElevationLayer(previous, { nowS: 102, savedMapFrameId: 'map' })
  assert.equal(direct.status, 'ready')

  const incoming: MapSceneEvent = {
    ...previous,
    ts: 102,
    sequence: 9,
    metadata: { producer_boot_id: 'mapd-boot-a' },
    layers: [{
      ...previous.layers[0],
      grid_b64: undefined,
      generation: 5,
      observation_sequence: 9,
      stamp_s: 102,
      payload: 'omitted',
      reason: 'rate_limited',
      retain_previous: true,
      retention_scope: 'same_elevation_cohort',
    }],
  }
  const merged = mergeMapSceneElevation(previous, incoming)
  const elevation = merged.layers[0]
  assert.equal(elevation.payload_retained, true)
  assert.equal(elevation.payload_generation, 4)
  assert.equal(elevation.retained_for_generation, 5)
  assert.equal(resolveElevationLayer(merged, { nowS: 102, savedMapFrameId: 'map' }).status, 'ready')
})

test('metadata without an explicit retention grant clears the previous elevation payload', () => {
  const previous = elevationScene([0, 0.25, 0.5, 0.75])
  previous.metadata = { generation: 4, live: true, reset_epoch: 2 }
  const incoming: MapSceneEvent = {
    ...previous,
    sequence: 9,
    metadata: { generation: 5, live: true, reset_epoch: 2 },
    layers: [{ id: 'maps.elevation', type: 'grid', frame_id: 'map', payload: 'omitted' }],
  }

  assert.equal(mergeMapSceneElevation(previous, incoming).layers[0].grid_b64, undefined)
})

test('producer restart and geometry changes clear old elevation even when counters regress', () => {
  const previous = elevationScene([0, 0.25, 0.5, 0.75])
  const producerRestart: MapSceneEvent = {
    ...previous,
    sequence: 1,
    metadata: {
      producer_boot_id: 'mapd-boot-b',
      generation: 1,
      reset_epoch: 2,
      observation_sequence: 1,
      live: true,
    },
    layers: [{
      id: 'maps.elevation',
      type: 'grid',
      frame_id: 'map',
      producer_boot_id: 'mapd-boot-b',
      reset_epoch: 2,
      generation: 1,
      rows: 2,
      cols: 2,
      resolution: 0.5,
      origin: [10, 20, 42],
      yaw: 0,
      downsample_factor: 1,
      payload: 'omitted',
      retain_previous: true,
      retention_scope: 'same_elevation_cohort',
    }],
  }
  assert.equal(mergeMapSceneElevation(previous, producerRestart).layers[0].grid_b64, undefined)

  const geometryChanged: MapSceneEvent = {
    ...previous,
    sequence: 9,
    metadata: { ...previous.metadata, generation: 5, observation_sequence: 9 },
    layers: [{
      id: 'maps.elevation',
      type: 'grid',
      frame_id: 'map',
      producer_boot_id: 'mapd-boot-a',
      reset_epoch: 2,
      generation: 5,
      rows: 2,
      cols: 2,
      resolution: 0.25,
      origin: [10, 20, 42],
      yaw: 0,
      downsample_factor: 1,
      payload: 'omitted',
      retain_previous: true,
      retention_scope: 'same_elevation_cohort',
    }],
  }
  assert.equal(mergeMapSceneElevation(previous, geometryChanged).layers[0].grid_b64, undefined)
})

test('invalid cells cut holes in the elevation surface instead of creating NaN triangles', () => {
  const state = resolveElevationLayer(elevationScene([Number.NaN, 0, 0.5, 1]), {
    nowS: 101,
    savedMapFrameId: 'map',
  })
  const mesh = createElevationLayer(state)
  assert.ok(mesh)
  assert.deepEqual(Array.from(mesh.geometry.getIndex()?.array ?? []), [1, 2, 3])
  assert.ok(Array.from(mesh.geometry.getAttribute('position').array).every(Number.isFinite))
})

test('reset epochs and regressing scene identities never inherit an older elevation payload', () => {
  const previous = elevationScene([0, 0.25, 0.5, 0.75])
  previous.map_id = 'map-a'
  previous.metadata = { generation: 4, live: true, reset_epoch: 2 }
  const reset: MapSceneEvent = {
    ...previous,
    ts: 101,
    sequence: 1,
    metadata: { generation: 1, live: true, reset_epoch: 3 },
    layers: [{ id: 'maps.elevation', type: 'grid', frame_id: 'map', payload: 'omitted' }],
  }
  assert.equal(mergeMapSceneElevation(previous, reset).layers[0].grid_b64, undefined)

  const switchedMap: MapSceneEvent = {
    ...reset,
    map_id: 'map-b',
    metadata: { generation: 1, live: true, reset_epoch: 2 },
  }
  assert.equal(mergeMapSceneElevation(previous, switchedMap).layers[0].grid_b64, undefined)

  const unsafe: MapSceneEvent = {
    ...reset,
    sequence: 9,
    map_id: 'map-a',
    metadata: { generation: 5, live: true, reset_epoch: 2 },
    layers: [{
      id: 'maps.elevation',
      type: 'grid',
      frame_id: 'map',
      payload: 'omitted',
      reason: 'unsafe_elevation_payload',
      retain_previous: true,
    }],
  }
  assert.equal(mergeMapSceneElevation(previous, unsafe).layers[0].grid_b64, undefined)

  const regressing: MapSceneEvent = {
    ...reset,
    sequence: 7,
    map_id: 'map-a',
    metadata: {
      producer_boot_id: 'mapd-boot-a',
      generation: 3,
      reset_epoch: 2,
      observation_sequence: 7,
      live: true,
    },
    layers: [{
      id: 'maps.elevation',
      type: 'grid',
      frame_id: 'map',
      producer_boot_id: 'mapd-boot-a',
      reset_epoch: 2,
      generation: 3,
      rows: 2,
      cols: 2,
      resolution: 0.5,
      origin: [10, 20, 42],
      yaw: 0,
      downsample_factor: 1,
      payload: 'omitted',
      retain_previous: true,
      retention_scope: 'same_elevation_cohort',
    }],
  }
  assert.equal(mergeMapSceneElevation(previous, regressing), previous)
})

test('stale elevation payloads are hidden with a visible stale state', () => {
  const state = resolveElevationLayer(elevationScene([0, 0.25, 0.5, 0.75]), {
    nowS: 106,
    savedMapFrameId: 'map',
  })
  assert.equal(state.status, 'stale')
  assert.match(state.message, /过期/)
})

test('zero valid cells clear the surface as unavailable instead of becoming an error mesh', () => {
  const state = resolveElevationLayer(
    elevationScene(
      [Number.NaN, Number.NaN, Number.NaN, Number.NaN],
      { valid_count: 0, min_z: null, max_z: null },
    ),
    { nowS: 101, savedMapFrameId: 'map' },
  )
  assert.equal(state.status, 'unavailable')
  assert.match(state.message, /无有效观测/)
})
