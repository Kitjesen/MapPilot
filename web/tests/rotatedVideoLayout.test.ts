import assert from 'node:assert/strict'
import test from 'node:test'

import { fitRotatedVideo } from '../src/hooks/useRotatedVideoLayout.ts'

test('standard camera rotations remain fully inside a landscape preview', () => {
  assert.deepEqual(fitRotatedVideo(400, 300, 400, 300, 0), {
    rotation: 0, width: 400, height: 300,
  })
  assert.deepEqual(fitRotatedVideo(400, 300, 400, 300, 180), {
    rotation: 180, width: 400, height: 300,
  })
  assert.deepEqual(fitRotatedVideo(400, 300, 400, 300, 90), {
    rotation: 90, width: 300, height: 225,
  })
  assert.deepEqual(fitRotatedVideo(400, 300, 400, 300, 270), {
    rotation: 270, width: 300, height: 225,
  })
})

test('quarter-turn video also fits a portrait preview without clipping', () => {
  assert.deepEqual(fitRotatedVideo(300, 400, 400, 300, 90), {
    rotation: 90, width: 400, height: 300,
  })
})
