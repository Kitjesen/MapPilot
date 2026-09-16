import assert from 'node:assert/strict'
import test from 'node:test'

import {
  mapProjectionDisplayZ,
  riskProjectionDisplayZ,
  NOMINAL_GO2_FOOT_OFFSET_M,
} from '../src/services/mapProjectionHeight.ts'

const closeTo = (actual: number, expected: number) => {
  assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`)
}

test('risk follows the displayed projection instead of cutting through the robot body', () => {
  const underlay = mapProjectionDisplayZ(0, 0, 'go2', true)
  closeTo(riskProjectionDisplayZ(.4, 0, 'go2', true, underlay), underlay + .006)
  closeTo(riskProjectionDisplayZ(.4, 0, 'go2', true), underlay + .006)
  closeTo(riskProjectionDisplayZ(.4, 0, undefined, false), .418)
})

test('derives the nominal Go2 foot-center offset from the checked-in URDF leg geometry', () => {
  closeTo(NOMINAL_GO2_FOOT_OFFSET_M, 2 * 0.213 * Math.cos(0.8))
})

test('places a body-origin Go2 projection near the nominal feet', () => {
  closeTo(
    mapProjectionDisplayZ(0, 0, 'go2', true),
    -NOMINAL_GO2_FOOT_OFFSET_M + 0.012,
  )
})

test('keeps an already-lower source projection for a Go2 pose above it', () => {
  closeTo(mapProjectionDisplayZ(0, 0.34, 'go2', true), 0.012)
})

test('keeps source display height when the Go2 pose is unavailable or invalid', () => {
  closeTo(mapProjectionDisplayZ(0.2, 0, 'go2', false), 0.212)
  closeTo(mapProjectionDisplayZ(0.2, Number.NaN, 'go2', true), 0.212)
  closeTo(mapProjectionDisplayZ(0.2, Number.POSITIVE_INFINITY, 'go2', true), 0.212)
})

test('does not infer a foot offset for other or unidentified robot models', () => {
  closeTo(mapProjectionDisplayZ(-0.1, 0, 'thunder_v4', true), -0.088)
  closeTo(mapProjectionDisplayZ(-0.1, 0, undefined, true), -0.088)
})
