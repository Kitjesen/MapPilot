import assert from 'node:assert/strict'
import test from 'node:test'

import * as THREE from 'three'

import {
  createLocalPlannerDiagnosticLayer,
  LOCAL_PLANNER_LAYER_COLORS,
} from '../src/components/scene3d/layers/localPlannerLayer.ts'
import type { NavigationDdsSnapshotResponse } from '../src/types/index.ts'

function meshColor(object: THREE.Object3D): number | null {
  const material = (object as THREE.Mesh | THREE.Points | THREE.Line).material
  if (!material || Array.isArray(material) || !('color' in material)) return null
  return (material.color as THREE.Color).getHex()
}

const snapshot: NavigationDdsSnapshotResponse = {
  schema_version: 'lingtu.navigation.dds_snapshot.v1',
  global_path: { schema_version: 1, path: [], robot: null, count: 0, frame_id: 'map', source: 'test' },
  local_path: { schema_version: 1, path: [], robot: null, count: 0, frame_id: 'map', source: 'test' },
  cmd_vel: null,
  nav_endpoint: {
    stamp_s: 10,
    local_map: {
      enabled: true,
      frame_id: 'map',
      obstacle_points_fresh: true,
      obstacle_points: [[1, 2, 0.3, 4]],
      traversability: {
        rows: 2,
        cols: 2,
        resolution_m: 0.5,
        origin_xy: [0, 0],
        fresh: true,
        risk_cells: [[1, 0, 0.8]],
      },
    },
    local_candidates: {
      valid: true,
      frame_id: 'map',
      candidates: [
        { selected: false, state: 'feasible', path: [[0, 0, 0], [1, 0, 0]] },
        { selected: false, state: 'collision_blocked', path: [[0, 0, 0], [0, 1, 0]] },
        { selected: true, state: 'feasible', path: [[0, 0, 0], [1, 1, 0]] },
      ],
    },
  },
  traversability_endpoint: null,
  navigation: {},
  ts: 10,
  source: 'test',
}

test('native local-planner diagnostics render as distinct read-only scene layers', () => {
  const layer = createLocalPlannerDiagnosticLayer(snapshot)
  assert.ok(layer)

  const obstacle = layer.getObjectByName('planner-obstacles')
  const traversability = layer.getObjectByName('traversability-risk')
  const candidates = layer.getObjectByName('candidate-paths')
  const selected = layer.getObjectByName('selected-path')

  assert.ok(obstacle)
  assert.ok(traversability)
  assert.ok(candidates)
  assert.ok(selected)
  assert.equal(meshColor(obstacle), LOCAL_PLANNER_LAYER_COLORS.obstacle)
  assert.equal(meshColor(traversability), LOCAL_PLANNER_LAYER_COLORS.traversability)
  assert.equal(meshColor(selected), LOCAL_PLANNER_LAYER_COLORS.selected)
  assert.equal(candidates.children.length, 2)
  assert.ok(candidates.children.some(item => meshColor(item) === LOCAL_PLANNER_LAYER_COLORS.blocked))

  const obstaclePosition = (obstacle as THREE.Points).geometry.getAttribute('position')
  assert.deepEqual(Array.from(obstaclePosition.array), [1, 0.30000001192092896, -2])
  const terrainMatrix = new THREE.Matrix4()
  ;(traversability as THREE.InstancedMesh).getMatrixAt(0, terrainMatrix)
  const terrainPosition = new THREE.Vector3().setFromMatrixPosition(terrainMatrix)
  assert.deepEqual(terrainPosition.toArray(), [0.25, 0.03999999910593033, -0.75])
})

test('invalid or stale endpoint layers are hidden instead of presented as live planning data', () => {
  const stale: NavigationDdsSnapshotResponse = {
    ...snapshot,
    nav_endpoint: {
      ...snapshot.nav_endpoint,
      local_map: {
        ...snapshot.nav_endpoint?.local_map,
        obstacle_points_fresh: false,
        traversability: {
          ...snapshot.nav_endpoint?.local_map?.traversability,
          fresh: false,
        },
      },
      local_candidates: {
        ...snapshot.nav_endpoint?.local_candidates,
        valid: false,
      },
    },
  }

  assert.equal(createLocalPlannerDiagnosticLayer(stale), null)
  assert.equal(createLocalPlannerDiagnosticLayer({
    ...snapshot,
    ts: 20,
    nav_endpoint: { ...snapshot.nav_endpoint, stamp_s: 10 },
  }), null)
})
