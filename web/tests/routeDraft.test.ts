import assert from 'node:assert/strict'
import test from 'node:test'

import {
  EMPTY_ROUTE_DRAFT,
  appendRouteDraftPoint,
  buildInspectionRouteRequest,
  ensureRouteStartIntent,
  moveRouteDraftPoint,
  removeRouteDraftPoint,
  resolveRouteDraftIdentity,
  routeDraftFromSaved,
  routeDraftSignature,
  saveAndStartRouteDraft,
  selectUniqueOpenRouteTask,
  sameRouteDraftIdentity,
} from '../src/services/routeDraft.ts'
import type { InspectionRoute, InspectionTaskStatusResponse } from '../src/types/index.ts'

const mapA = { mapId: 'hall', contentEpoch: 4 }
const mapB = { mapId: 'hall', contentEpoch: 5 }
const savedRoute: InspectionRoute = {
  id: 'evening-inspection',
  name: 'Evening inspection',
  map_id: mapA.mapId,
  map_content_epoch: mapA.contentEpoch,
  revision: 7,
  loop_count: 3,
  failure_policy: 'retry',
  max_retries: 2,
  points: [
    { id: 'point-1', x: 1, y: 2, z: 0.2, yaw: 0, tolerance: 0.3, dwell: 4, action: 'capture:overview', enabled: true },
    { id: 'point-3', x: 3, y: 4, z: 0.8, yaw: 1.5, tolerance: 0.2, dwell: 0, action: '', enabled: false },
  ],
}

test('saved route editing retains point metadata and policies while assigning unique IDs to new points', () => {
  const imported = routeDraftFromSaved(savedRoute)
  const reordered = moveRouteDraftPoint(imported, 1, -1)
  const extended = appendRouteDraftPoint(reordered, mapA, { x: 5, y: 6, z: 1.2 })
  const draft = appendRouteDraftPoint(extended, mapA, { x: 7, y: 8, z: 1.6 })
  const request = buildInspectionRouteRequest(draft, {
    id: 'unused-new-id',
    name: 'Edited route',
    tolerance: 0.45,
  })

  assert.equal(draft.source, savedRoute)
  assert.deepEqual(request, {
    id: savedRoute.id,
    name: 'Edited route',
    map_id: savedRoute.map_id,
    map_content_epoch: savedRoute.map_content_epoch,
    revision: 8,
    loop_count: 3,
    failure_policy: 'retry',
    max_retries: 2,
    points: [
      savedRoute.points[1],
      savedRoute.points[0],
      { id: 'point-2', x: 5, y: 6, z: 1.2, yaw: null, tolerance: 0.45, dwell: 0, action: '', enabled: true },
      { id: 'point-4', x: 7, y: 8, z: 1.6, yaw: null, tolerance: 0.45, dwell: 0, action: '', enabled: true },
    ],
  })
  assert.notEqual(imported.points[0], savedRoute.points[0])
  assert.deepEqual(savedRoute.points.map(point => point.id), ['point-1', 'point-3'])
  assert.equal(new Set(request.points.map(point => point.id)).size, request.points.length)
})

test('appending on a different map or content epoch drops the saved route source', () => {
  for (const identity of [mapB, { mapId: 'courtyard', contentEpoch: 4 }]) {
    const draft = appendRouteDraftPoint(routeDraftFromSaved(savedRoute), identity, { x: 4, y: 5, z: 6 })
    assert.deepEqual(draft, { identity, points: [{ x: 4, y: 5, z: 6 }] })
  }
})

test('draft signatures include route names and policies but ignore persistence revisions', () => {
  const draft = routeDraftFromSaved(savedRoute)
  const signature = routeDraftSignature(draft, 0.45, 'Evening inspection')

  assert.equal(routeDraftSignature({
    ...draft,
    source: { ...savedRoute, revision: 8 },
  }, 0.45, 'Evening inspection'), signature)
  assert.notEqual(routeDraftSignature(draft, 0.45, 'Morning inspection'), signature)
  assert.notEqual(routeDraftSignature({
    ...draft,
    source: { ...savedRoute, loop_count: 4 },
  }, 0.45, 'Evening inspection'), signature)
})

test('route draft preserves click order and three-dimensional coordinates', () => {
  const first = appendRouteDraftPoint(EMPTY_ROUTE_DRAFT, mapA, { x: 1, y: 2, z: 0.15 })
  const second = appendRouteDraftPoint(first, mapA, { x: 3, y: 4, z: -0.05 })

  assert.deepEqual(second, {
    identity: mapA,
    points: [
      { x: 1, y: 2, z: 0.15 },
      { x: 3, y: 4, z: -0.05 },
    ],
  })
})

test('route draft supports ordered removal and movement', () => {
  const draft = {
    identity: mapA,
    points: [
      { x: 1, y: 0, z: 0 },
      { x: 2, y: 0, z: 0 },
      { x: 3, y: 0, z: 0 },
    ],
  }

  const moved = moveRouteDraftPoint(draft, 2, -1)
  assert.deepEqual(moved.points.map(point => point.x), [1, 3, 2])
  assert.deepEqual(removeRouteDraftPoint(moved, 0).points.map(point => point.x), [3, 2])
})

test('a new map content epoch starts a new route draft', () => {
  const draft = appendRouteDraftPoint(EMPTY_ROUTE_DRAFT, mapA, { x: 1, y: 2, z: 3 })
  const replaced = appendRouteDraftPoint(draft, mapB, { x: 4, y: 5, z: 6 })

  assert.equal(sameRouteDraftIdentity(draft.identity, mapB), false)
  assert.deepEqual(replaced, { identity: mapB, points: [{ x: 4, y: 5, z: 6 }] })
})

test('loaded cloud epoch supersedes a stale map-list epoch after map reactivation', () => {
  assert.deepEqual(resolveRouteDraftIdentity('hall', 4, undefined), mapA)
  assert.deepEqual(resolveRouteDraftIdentity('hall', 4, 5), mapB)
  assert.deepEqual(resolveRouteDraftIdentity('hall', 0, undefined), {
    mapId: 'hall',
    contentEpoch: 0,
  })
  assert.equal(resolveRouteDraftIdentity('hall', undefined, undefined), null)
})

test('non-finite coordinates never enter a route draft', () => {
  const draft = appendRouteDraftPoint(EMPTY_ROUTE_DRAFT, mapA, { x: 1, y: 2, z: Number.NaN })
  assert.equal(draft, EMPTY_ROUTE_DRAFT)
})

test('a route draft saves once and starts one native task with the saved revision', async () => {
  const draft = {
    identity: mapA,
    points: [
      { x: 1, y: 2, z: 0.1 },
      { x: 3, y: 4, z: 0.2 },
    ],
  }
  const request = buildInspectionRouteRequest(draft, {
    id: 'scene-route',
    name: 'Route',
    tolerance: 0.45,
  })
  const calls: string[] = []
  const receipt = await saveAndStartRouteDraft(request, 'start-fixed', {
    save: async value => {
      calls.push(`save:${value.points.map(point => point.z).join(',')}`)
      return {
        schema_version: 'lingtu.inspection.v1',
        ok: true,
        route: { ...value, revision: 3 },
        ts: 1,
      }
    },
    start: async (routeId, options) => {
      calls.push(`start:${routeId}:${options.map_id}:${options.revision}:${options.request_id}`)
      return {
        schema_version: 'lingtu.inspection.task.v1',
        ok: true,
        accepted: true,
        action: 'start',
        task_id: 'task-1',
        request_id: 'request-1',
        lifecycle: 'submission_accepted',
        terminal: false,
        ts: 2,
      }
    },
  })

  assert.deepEqual(calls, ['save:0.1,0.2', 'start:scene-route:hall:3:start-fixed'])
  assert.equal(receipt.task_id, 'task-1')
})

test('a retry keeps the same start request and route identity until the draft changes', () => {
  let sequence = 0
  const createRequestId = () => `request-${++sequence}`
  const createRouteId = () => `route-${++sequence}`
  const signature = routeDraftSignature({
    identity: mapA,
    points: [{ x: 1, y: 2, z: 3 }, { x: 4, y: 5, z: 6 }],
  }, 0.45)

  const first = ensureRouteStartIntent(null, signature, createRequestId, createRouteId)
  const retry = ensureRouteStartIntent(first, signature, createRequestId, createRouteId)
  const changed = ensureRouteStartIntent(first, `${signature}:changed`, createRequestId, createRouteId)

  assert.equal(retry, first)
  assert.equal(retry.requestId, first.requestId)
  assert.equal(retry.routeId, first.routeId)
  assert.notEqual(changed.requestId, first.requestId)
  assert.notEqual(changed.routeId, first.routeId)
})

test('task recovery resumes only one unambiguous open task', () => {
  const task = (taskId: string, terminal: boolean): InspectionTaskStatusResponse => ({
    task_id: taskId,
    terminal,
  } as InspectionTaskStatusResponse)

  assert.deepEqual(selectUniqueOpenRouteTask([]), { kind: 'none', task: null })
  assert.deepEqual(
    selectUniqueOpenRouteTask([task('done', true), task('active', false)]),
    { kind: 'one', task: task('active', false) },
  )
  assert.deepEqual(
    selectUniqueOpenRouteTask([task('first', false), task('second', false)]),
    { kind: 'ambiguous', task: null },
  )
})
