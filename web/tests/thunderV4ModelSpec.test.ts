import assert from 'node:assert/strict'
import test from 'node:test'

import {
  THUNDER_V4_DISPLAY_SCALE,
  THUNDER_V4_MESH_FILES,
  THUNDER_V4_PHYSICAL_SIZE_M,
} from '../src/components/scene3d/robot/thunderV4ModelSpec.ts'

test('Thunder V4 viewer uses the real mesh set at the requested smaller display scale', () => {
  assert.equal(THUNDER_V4_DISPLAY_SCALE, 0.75)
  assert.deepEqual(THUNDER_V4_PHYSICAL_SIZE_M, {
    length: 0.86,
    width: 0.58,
    standingHeight: 0.68,
  })
  assert.equal(THUNDER_V4_MESH_FILES[0], 'base_link.STL')
  assert.equal(THUNDER_V4_MESH_FILES.length, 21)
  assert.equal(new Set(THUNDER_V4_MESH_FILES).size, THUNDER_V4_MESH_FILES.length)
})
