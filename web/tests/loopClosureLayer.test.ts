import assert from 'node:assert/strict'
import test from 'node:test'

import * as THREE from 'three'

import {
  createLoopClosureLayer,
  LOOP_CLOSURE_LAYER_COLORS,
  resolveLoopClosureLayer,
} from '../src/components/scene3d/layers/loopClosureLayer.ts'
import type { MapSceneEvent } from '../src/types/index.ts'

const scene: MapSceneEvent = {
  type: 'map_scene',
  frame_id: 'map',
  ts: 100,
  sequence: 42,
  metadata: {
    producer_boot_id: 'slam-boot-a',
    reset_epoch: 7,
    observation_sequence: 42,
    generation: 9,
  },
  layers: [
    {
      id: 'localization.loop_constraints',
      type: 'loop_constraints',
      frame_id: 'map',
      stamp_s: 100,
      producer_boot_id: 'slam-boot-a',
      reset_epoch: 7,
      observation_sequence: 42,
      generation: 9,
      online: true,
      identity_verified: true,
      constraint_semantics: 'loop_closure_validation_v1',
      constraints: [
        {
          from_index: 1,
          to_index: 12,
          from: [1, 2, 0.3],
          to: [4, -1, 0.5],
          state: 'accepted',
          geometrically_verified: true,
          rmse_m: 0.12,
        },
        {
          from_index: 3,
          to_index: 15,
          from: [-2, 1, 0.1],
          to: [0, 5, 0.2],
          state: 'rejected',
          geometrically_verified: false,
          reason: 'point_to_plane_rank_gate',
        },
      ],
    },
  ],
}

function colorOf(object: THREE.Object3D | undefined): number | null {
  const material = (object as THREE.LineSegments | undefined)?.material
  if (!material || Array.isArray(material) || !('color' in material)) return null
  return (material.color as THREE.Color).getHex()
}

function withLoopLayer(patch: Record<string, unknown>): MapSceneEvent {
  return {
    ...scene,
    layers: [{ ...scene.layers[0], ...patch }],
  }
}

test('online loop validation renders accepted and rejected constraints as a read-only scene layer', () => {
  const state = resolveLoopClosureLayer(scene, { nowS: 101, allowedFrameIds: ['map'] })
  assert.equal(state.status, 'ready')
  if (state.status !== 'ready') return
  assert.equal(state.accepted.length, 1)
  assert.equal(state.rejected.length, 1)

  const layer = createLoopClosureLayer(state)
  assert.ok(layer)
  assert.equal(layer.name, 'online-loop-closure-validation')
  assert.equal(colorOf(layer.getObjectByName('accepted-loop-constraints')), LOOP_CLOSURE_LAYER_COLORS.accepted)
  assert.equal(colorOf(layer.getObjectByName('rejected-loop-candidates')), LOOP_CLOSURE_LAYER_COLORS.rejected)
  assert.equal(layer.userData.acceptedCount, 1)
  assert.equal(layer.userData.rejectedCount, 1)
})

test('loop validation hides stale, cross-frame, and unverified producers', () => {
  assert.equal(
    resolveLoopClosureLayer(scene, { nowS: 106, allowedFrameIds: ['map'] }).status,
    'stale',
  )
  assert.equal(
    resolveLoopClosureLayer(
      { ...scene, frame_id: 'odom' },
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  const unverified: MapSceneEvent = {
    ...scene,
    layers: scene.layers.map(layer => ({ ...layer, identity_verified: false })),
  }
  assert.equal(
    resolveLoopClosureLayer(unverified, { nowS: 101, allowedFrameIds: ['map'] }).status,
    'error',
  )
})

test('loop validation requires the exact current scene identity', () => {
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ observation_sequence: 41 }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ generation: 8 }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
})

test('loop validation rejects incomplete or malformed producer contracts', () => {
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ frame_id: undefined }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ stamp_s: undefined }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({
        constraints: [{
          from_index: 1,
          to_index: 2,
          from: [0, 0, 0],
          to: [1, 1, 0],
          state: 'candidate',
          geometrically_verified: false,
        }],
      }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ constraints: [{ from_index: 1 }] }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  const accepted = (scene.layers[0].constraints as Array<Record<string, unknown>>)[0]
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ constraints: [{ ...accepted, reason: 42 }] }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
  assert.equal(
    resolveLoopClosureLayer(
      withLoopLayer({ constraints: [{ ...accepted, rmse_m: -0.1 }] }),
      { nowS: 101, allowedFrameIds: ['map'] },
    ).status,
    'error',
  )
})
