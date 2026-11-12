import test from 'node:test'
import assert from 'node:assert/strict'
import { applyMapEditOperations, isMapCrop, validMapEditOperation, mapEditCloudBounds, mapEditNameError, mapEditSelectionImpact, mapEditWireOperations, suggestedEditedMapName, validMapEditBounds } from '../src/services/mapEdit.ts'
import type { MapEditOperation } from '../src/types/mapEdit.ts'

const box = { min: { x: -1, y: -1, z: 0 }, max: { x: 1, y: 1, z: 2 } }

test('manual planes preserve source points, map axes, ordered crop and undo', () => {
  for (const axis of ['x', 'y', 'z'] as const) {
    const plane = { operation: 'add_plane', bounds: box, axis, offset: .5, spacing: 1 } as const
    assert.equal(validMapEditOperation(plane), true)
    const source = [9, 8, 7]
    const result = applyMapEditOperations(source, [plane])
    assert.equal(result.length, 30)
    assert.deepEqual(result.slice(0, 3), source)
    for (let i = 3 + ['x', 'y', 'z'].indexOf(axis); i < result.length; i += 3) assert.equal(result[i], .5)
    assert.deepEqual(applyMapEditOperations(source, [plane, { operation: 'delete_inside', bounds: box }]), source)
    assert.deepEqual(applyMapEditOperations(source, []), source)
    assert.equal(validMapEditOperation({ ...plane, spacing: .00001 }), false)
    assert.equal(validMapEditOperation({ ...plane, offset: 20 }), false)
    assert.deepEqual(mapEditWireOperations([plane])[0], { ...plane,
      bounds: { min_x: -1, min_y: -1, min_z: 0, max_x: 1, max_y: 1, max_z: 2 } })
  }
})

test('source subset edits include box faces, preserve coordinates and never mutate the source', () => {
  const original = [-2, 0, 1, -1, 0, 0, 0, 0, 2, 0, 0, 3, 2, 0, 0]
  const snapshot = [...original]
  assert.deepEqual(applyMapEditOperations(original, [{ operation: 'delete_inside', bounds: box }]), [-2, 0, 1, 0, 0, 3, 2, 0, 0])
  assert.deepEqual(applyMapEditOperations(original, [{ operation: 'keep_inside', bounds: box }]), [-1, 0, 0, 0, 0, 2])
  assert.deepEqual(original, snapshot)
})

test('undoing the operation list restores removed points and planes remain distinct at the same XY', () => {
  const original = [0, 0, 0, 0, 0, 1, 0, 0, 4, 2, 0, 1]
  const operations: MapEditOperation[] = [{ operation: 'delete_inside', bounds: box }, {
    operation: 'keep_inside', bounds: { min: { x: -1, y: -1, z: -1 }, max: { x: 1, y: 1, z: 5 } },
  }]
  assert.deepEqual(applyMapEditOperations(original, operations), [0, 0, 4])
  assert.deepEqual(applyMapEditOperations(original, operations.slice(0, 1)), [0, 0, 4, 2, 0, 1])
  assert.deepEqual(applyMapEditOperations(original, []), original)
})

test('full-height source bounds do not inherit a display clip or flatten stacked surfaces', () => {
  assert.deepEqual(mapEditCloudBounds([1, 2, -2, 3, 4, 8, NaN, 0, 20]), {
    min: { x: 1, y: 2, z: -2 }, max: { x: 3, y: 4, z: 8 },
  })
  assert.equal(mapEditCloudBounds([]), null)
  assert.equal(validMapEditBounds(box), true)
  assert.equal(validMapEditBounds({ ...box, max: { ...box.max, z: NaN } }), false)
  assert.equal(validMapEditBounds({ min: box.max, max: box.min }), false)
})

test('native edit request expresses physical XYZ bounds without Three.js axis substitution', () => {
  assert.deepEqual(mapEditWireOperations([{ operation: 'delete_inside', bounds: box }]), [{
    operation: 'delete_inside', bounds: { min_x: -1, min_y: -1, min_z: 0, max_x: 1, max_y: 1, max_z: 2 },
  }])
})

test('selection impact includes box faces and keeps stacked Z surfaces distinct', () => {
  const preview = [-1, -1, 0, 1, 1, 2, 0, 0, 3, 2, 0, 1]
  const snapshot = [...preview]
  assert.deepEqual(mapEditSelectionImpact(preview, box), { selected: 2, total: 4 })
  assert.deepEqual(preview, snapshot)
})

test('selection impact counts the current draft without counting previously deleted points again', () => {
  const source = [0, 0, 0, 0, 0, 4, 2, 0, 1]
  const draft = applyMapEditOperations(source, [{ operation: 'delete_inside', bounds: box }])
  assert.deepEqual(mapEditSelectionImpact(draft, box), { selected: 0, total: 2 })
  assert.deepEqual(mapEditSelectionImpact(draft, {
    min: { x: -1, y: -1, z: 3 }, max: { x: 1, y: 1, z: 5 },
  }), { selected: 1, total: 2 })
})

test('selection impact excludes invalid XYZ and incomplete points from the denominator', () => {
  const preview = [0, 0, 0, NaN, 0, 1, 0, Infinity, 1, 0, 0, NaN, 3, 3, 3, 44, 55]
  assert.deepEqual(mapEditSelectionImpact(preview, box), { selected: 1, total: 2 })
  assert.deepEqual(mapEditSelectionImpact([], box), { selected: 0, total: 0 })
  assert.deepEqual(mapEditSelectionImpact(preview, {
    min: { x: 10, y: 10, z: 10 }, max: { x: 11, y: 11, z: 11 },
  }), { selected: 0, total: 2 })
})

test('save-as names follow public map-service limits before submitting the draft', () => {
  assert.equal(mapEditNameError('office-edited', 'office'), null)
  assert.equal(mapEditNameError('   ', 'office'), '填写新地图名称。')
  assert.equal(mapEditNameError(' office ', 'office'), '使用新名称，原地图会保留。')
  assert.equal(mapEditNameError('a'.repeat(100), 'office'), null)
  assert.equal(mapEditNameError('a'.repeat(101), 'office'), '地图名称最多 100 个字符。')
  assert.equal(mapEditNameError('operations', 'office'), '这个名称已由地图服务使用，请换一个名称。')
  assert.equal(mapEditNameError('office..edited', 'office'), '名称不能包含连续的两个点。')
  assert.ok(mapEditNameError('office/edited', 'office'))
  assert.ok(mapEditNameError('-office', 'office'))
})

test('the suggested name fits even when a valid source map already uses all 100 characters', () => {
  assert.equal(suggestedEditedMapName('office'), 'office-edited')
  for (const length of [93, 94, 100]) {
    const source = 'a'.repeat(length)
    const suggested = suggestedEditedMapName(source)
    assert.equal(suggested.length, 100)
    assert.equal(mapEditNameError(suggested, source), null)
    assert.notEqual(suggested, source)
  }
  const editedSource = `${'a'.repeat(93)}-edited`
  assert.equal(mapEditNameError(suggestedEditedMapName(editedSource), editedSource), null)
})

const filterOperations: MapEditOperation[] = [
  { operation: 'voxel', voxel_size: .1 },
  { operation: 'statistical', neighbors: 20, std_ratio: 1.5 },
  { operation: 'radius', radius: .2, min_points: 12 },
  { operation: 'clusters', radius: .2, neighbors: 20, min_cluster_points: 100 },
  { operation: 'plane_crop', distance_threshold: .05, margin: .5 },
]

test('native cleaning parameters keep their ordered values alongside physical crop bounds', () => {
  const operations = [filterOperations[0], { operation: 'keep_inside', bounds: box } as MapEditOperation, ...filterOperations.slice(1)]
  const wire = mapEditWireOperations(operations)
  assert.deepEqual(wire[0], filterOperations[0])
  assert.deepEqual(wire[1], { operation: 'keep_inside', bounds: { min_x: -1, min_y: -1, min_z: 0, max_x: 1, max_y: 1, max_z: 2 } })
  assert.deepEqual(wire.slice(2), filterOperations.slice(1))
  assert.equal(operations.every(validMapEditOperation), true)
  assert.equal(operations.filter(isMapCrop).length, 1)
})

test('cleaning cannot silently run through the browser crop preview', () => {
  assert.throws(() => applyMapEditOperations([0, 0, 0], filterOperations), /原生地图预览/)
  for (const edit of [
    { operation: 'voxel', voxel_size: 0 },
    { operation: 'statistical', neighbors: 2.5, std_ratio: 1.5 },
    { operation: 'radius', radius: .2, min_points: NaN },
    { operation: 'clusters', radius: .2, neighbors: 20, min_cluster_points: -1 },
    { operation: 'plane_crop', distance_threshold: .05, margin: -1 },
  ]) assert.equal(validMapEditOperation(edit), false)
  assert.equal(validMapEditOperation({ operation: 'plane_crop', distance_threshold: .05, margin: 0 }), true)
})
