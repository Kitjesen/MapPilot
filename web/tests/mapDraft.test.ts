import test from 'node:test'
import assert from 'node:assert/strict'
import { readMapDraft, removeMapDraft, writeMapDraft } from '../src/services/mapDraft.ts'
import type { MapDraftStorage, MapEditDraft } from '../src/services/mapDraft.ts'

function memoryStorage(): MapDraftStorage {
  const values = new Map<string, string>()
  return {
    getItem: key => values.get(key) ?? null,
    setItem: (key, value) => { values.set(key, value) },
    removeItem: key => { values.delete(key) },
  }
}

function draft(): MapEditDraft {
  const bounds = { min: { x: 0, y: 1, z: 2 }, max: { x: 3, y: 4, z: 5 } }
  return {
    source: { name: '903', contentEpoch: 4, frameId: 'map' },
    name: '903-edited',
    history: { edits: [
      { operation: 'delete_inside', bounds },
      { operation: 'keep_inside', bounds },
    ], cursor: 1 },
    status: 'editing',
  }
}

test('roundtrip preserves source, operation order, target name and redo history', () => {
  const storage = memoryStorage(), original = draft()
  assert.equal(writeMapDraft(original, storage), true)
  const result = readMapDraft(original.source, storage)
  assert.deepEqual(result, { kind: 'ready', draft: original })
  result!.draft.history.edits[0].bounds.min.x = -10
  assert.deepEqual(readMapDraft(original.source, storage)?.draft, original)
  original.history.cursor = 0
  assert.equal(writeMapDraft(original, storage), true)
  assert.deepEqual(readMapDraft(original.source, storage)?.draft, original)
})

test('epoch or frame changes mark a draft stale without changing stored data', () => {
  const storage = memoryStorage(), original = draft()
  writeMapDraft(original, storage)
  for (const source of [
    { ...original.source, contentEpoch: 5 },
    { ...original.source, frameId: 'other-map' },
  ]) assert.deepEqual(readMapDraft(source, storage), { kind: 'stale', draft: original })
  assert.equal(readMapDraft(original.source, storage)?.kind, 'ready')
})

test('unconfirmed saves preserve the original request identity even after map changes', () => {
  const storage = memoryStorage(), original = draft()
  original.status = 'unconfirmed'
  writeMapDraft(original, storage)
  assert.deepEqual(readMapDraft({ ...original.source, contentEpoch: 6, frameId: 'new' }, storage), {
    kind: 'unconfirmed', draft: original,
  })
})

test('map names isolate records and removal', () => {
  const storage = memoryStorage(), first = draft(), second = draft()
  second.source.name = 'building'
  writeMapDraft(first, storage)
  writeMapDraft(second, storage)
  assert.deepEqual(readMapDraft(second.source, storage)?.draft, second)
  assert.equal(removeMapDraft(first.source, storage), true)
  assert.equal(readMapDraft(first.source, storage), null)
  assert.deepEqual(readMapDraft(second.source, storage)?.draft, second)
})

test('empty histories are not written and do not remove an existing draft', () => {
  const storage = memoryStorage(), original = draft()
  writeMapDraft(original, storage)
  assert.equal(writeMapDraft({ ...original, history: { edits: [], cursor: 0 } }, storage), false)
  assert.deepEqual(readMapDraft(original.source, storage)?.draft, original)
})

test('only editing metadata is persisted, never extra point cloud data', () => {
  const storage = memoryStorage(), original = draft()
  writeMapDraft({ ...original, points: [1, 2, 3] } as MapEditDraft, storage)
  assert.deepEqual(readMapDraft(original.source, storage)?.draft, original)
})

test('malformed records are ignored without deleting or rewriting them', () => {
  const original = draft()
  const malformed = [
    '{', JSON.stringify(null), JSON.stringify({}),
    JSON.stringify({ ...original, source: { ...original.source, name: 'other' } }),
    JSON.stringify({ ...original, history: { edits: original.history.edits, cursor: -1 } }),
    JSON.stringify({ ...original, history: { edits: original.history.edits, cursor: 3 } }),
    JSON.stringify({ ...original, history: { edits: original.history.edits, cursor: 0.5 } }),
    JSON.stringify({ ...original, history: { edits: [], cursor: 0 } }),
    JSON.stringify({ ...original, history: { edits: [{ operation: 'unknown', bounds: original.history.edits[0].bounds }], cursor: 1 } }),
    JSON.stringify({ ...original, history: { edits: [{ operation: 'delete_inside', bounds: {} }], cursor: 1 } }),
    JSON.stringify({ ...original, history: { edits: [{ operation: 'delete_inside', bounds: { min: { x: 0, y: 0, z: 0 }, max: { x: 0, y: 1, z: 1 } } }], cursor: 1 } }),
  ]
  for (const raw of malformed) {
    const storage = memoryStorage()
    storage.setItem('lingtu.map-edit-draft.v1:903', raw)
    assert.equal(readMapDraft(original.source, storage), null)
    assert.equal(storage.getItem('lingtu.map-edit-draft.v1:903'), raw)
  }
})

test('unavailable browser storage and failed storage operations do not block editing', () => {
  const original = draft()
  const storage: MapDraftStorage = {
    getItem: () => { throw new Error('unavailable') },
    setItem: () => { throw new Error('full') },
    removeItem: () => { throw new Error('unavailable') },
  }
  assert.equal(readMapDraft(original.source, storage), null)
  assert.equal(writeMapDraft(original, storage), false)
  assert.equal(removeMapDraft(original.source, storage), false)
  assert.equal(readMapDraft(original.source), null)
  assert.equal(writeMapDraft(original), false)
  assert.equal(removeMapDraft(original.source), false)
})
