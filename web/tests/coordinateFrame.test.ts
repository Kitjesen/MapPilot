import assert from 'node:assert/strict'
import test from 'node:test'

import {
  lingtuToThree,
  lingtuYawToThree,
  threeToLingtu,
} from '../src/services/coordinateFrame.ts'

test('LingTu coordinate probe maps X-forward, Y-left, Z-up into Three Y-up', () => {
  assert.deepEqual(lingtuToThree([1, 0, 0]), [1, 0, 0])
  assert.deepEqual(lingtuToThree([0, 1, 0]), [0, 0, -1])
  assert.deepEqual(lingtuToThree([0, 0, 1]), [0, 1, 0])
  assert.deepEqual(threeToLingtu([1, 0, 0]), [1, 0, 0])
  assert.deepEqual(threeToLingtu([0, 0, -1]), [0, 1, 0])
  assert.deepEqual(threeToLingtu([0, 1, 0]), [0, 0, 1])
})

test('positive LingTu yaw uses the same right-handed heading convention', () => {
  assert.equal(lingtuYawToThree(Math.PI / 2), Math.PI / 2)
})
