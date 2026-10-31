import test from 'node:test'
import assert from 'node:assert/strict'
import { applyMapEditOperations, mapEditCloudBounds, mapEditNameError, mapEditSelectionImpact, mapEditWireOperations, suggestedEditedMapName, validMapEditBounds } from '../src/services/mapEdit.ts'
import type { MapEditOperation } from '../src/types/mapEdit.ts'

const box = { min: { x: -1, y: -1, z: 0 }, max: { x: 1, y: 1, z: 2 } }

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
