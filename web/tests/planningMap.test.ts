import assert from 'node:assert/strict'
import test from 'node:test'
import { currentPlanningMap, navigationPreviewIsCurrent, planningCellAt, planningCellLabel, type PlanningMap } from '../src/services/planningMap.ts'
import type { PlanPreviewResponse } from '../src/types/index.ts'
import * as THREE from 'three'
import { createPlanningMapLayer } from '../src/components/scene3d/layers/planningMapLayer.ts'

const fixture: PlanningMap = {
  available: true, reason: 'ready', schema_version: 1, frame_id: 'map', product_session_id: 'run',
  map_id: 'hall', map_content_epoch: 2, stamp_s: 1, resolution: .2, rows: 2, cols: 3,
  origin: [-1, -2, .1], reference_z: .12, cells: [0, 1, 2, 2, 1, 0],
}

test('rendered grid pixels and world-space picking agree for every cell', () => {
  const map = currentPlanningMap(fixture, true, 'hall', 'run')!
  const mesh = createPlanningMapLayer(map)!
  mesh._group!.updateMatrixWorld(true)
  const material = mesh.material as THREE.MeshBasicMaterial
  const texture = material.map as THREE.DataTexture
  assert.equal(texture.flipY, false)
  assert.equal(texture.generateMipmaps, false)
  assert.equal(texture.minFilter, THREE.NearestFilter)
  assert.equal(texture.magFilter, THREE.NearestFilter)
  assert.equal(texture.colorSpace, THREE.SRGBColorSpace)
  assert.equal(material.toneMapped, false)
  for (let row = 0; row < map.rows; row++) for (let col = 0; col < map.cols; col++) {
    const x = map.origin[0] + (col + .5) * map.resolution
    const y = map.origin[1] + (row + .5) * map.resolution
    const hit = new THREE.Raycaster(new THREE.Vector3(x, 10, -y), new THREE.Vector3(0, -1, 0)).intersectObject(mesh)[0]
    assert.ok(hit?.uv)
    const pixel = Math.floor(hit.uv.y * map.rows) * map.cols + Math.floor(hit.uv.x * map.cols)
    assert.equal(pixel, row * map.cols + col)
    assert.equal(map.cells[pixel], planningCellAt(map, x, y))
  }
  texture.dispose(); material.dispose(); mesh.geometry.dispose()
})

test('a preview cannot authorize a path from a robot pose that has since moved', () => {
  const preview = { feasible: true, frame_id: 'map', start: { x: 1, y: 2, z: .2 } } as PlanPreviewResponse
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1.1, y: 2, z: .2 }), true)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 2, y: 2, z: .2 }), false)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1, y: 2, z: 1.2 }), false)
  assert.equal(navigationPreviewIsCurrent(preview, null), false)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1, y: 2, z: null }), false)
  assert.equal(navigationPreviewIsCurrent({ ...preview, frame_id: 'odom' }, preview.start), false)
})

test('planning map keeps unknown, traversable and blocked distinct in map coordinates', () => {
  const map = currentPlanningMap(fixture, true, 'hall', 'run')
  assert.ok(map)
  assert.equal(planningCellAt(map, -.9, -1.9), 0)
  assert.equal(planningCellAt(map, -.7, -1.9), 1)
  assert.equal(planningCellAt(map, -.5, -1.9), 2)
  assert.equal(planningCellAt(map, -.9, -1.7), 2)
  assert.equal(planningCellAt(map, -1.001, -1.9), null)
  assert.equal(planningCellAt(map, -.39, -1.9), null)
  assert.equal(planningCellAt(map, 0, Number.NaN), null)
  assert.match(planningCellLabel(0), /缺少.*支撑/)
  assert.match(planningCellLabel(1), /仍需路径预检/)
  assert.match(planningCellLabel(2), /净空/)
})

test('disconnection, replaced session, other map and partial grids cannot remain as a usable layer', () => {
  assert.equal(currentPlanningMap(fixture, false, 'hall', 'run'), null)
  assert.equal(currentPlanningMap(fixture, true, 'other', 'run'), null)
  assert.equal(currentPlanningMap(fixture, true, 'hall', 'new-run'), null)
  assert.equal(currentPlanningMap({ ...fixture, cells: [0, 1] }, true, 'hall', 'run'), null)
  assert.equal(currentPlanningMap({ ...fixture, available: false }, true, 'hall', 'run'), null)
  assert.equal(currentPlanningMap({ ...fixture, origin: [0, 0, NaN] }, true, 'hall', 'run'), null)
})
