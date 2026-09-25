import assert from 'node:assert/strict'
import test from 'node:test'
import { navigationPreviewIsCurrent, sceneGoalHeight } from '../src/services/navigationPreview.ts'
import type { PlanPreviewResponse } from '../src/types/index.ts'

test('a preview cannot authorize a path from a robot pose that has since moved', () => {
  const preview = { feasible: true, frame_id: 'map', start: { x: 1, y: 2, z: .2 } } as PlanPreviewResponse
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1.1, y: 2, z: .2 }), true)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 2, y: 2, z: .2 }), false)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1, y: 2, z: 1.2 }), false)
  assert.equal(navigationPreviewIsCurrent(preview, null), false)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1, y: 2, z: null }), false)
  assert.equal(navigationPreviewIsCurrent({ ...preview, frame_id: 'odom' }, preview.start), false)
  assert.equal(navigationPreviewIsCurrent({ ...preview, feasible: false }, preview.start), false)
})

test('planar map clicks preserve continuous body height across the old 20 cm slice boundary', () => {
  assert.equal(sceneGoalHeight(-.01), -.01)
  assert.equal(sceneGoalHeight(.01), .01)
  assert.equal(sceneGoalHeight(.0184659), .0184659)
})

test('an explicit 3D target keeps its elevation while missing height cannot become z=0', () => {
  assert.equal(sceneGoalHeight(.02, 1.25), 1.25)
  assert.equal(sceneGoalHeight(.02, 0), 0)
  for (const z of [null, undefined, NaN, Infinity]) assert.equal(sceneGoalHeight(z), null)
  assert.equal(sceneGoalHeight(.02, NaN), null)
})
