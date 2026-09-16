import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'
import { safetyEnvelopeFromRunPlan, createSafetyEnvelope } from '../src/components/scene3d/layers/safetyEnvelope.ts'

test('envelope follows the active RunPlan rather than a hardcoded robot size', () => {
  assert.equal(safetyEnvelopeFromRunPlan(null), null)
  assert.equal(safetyEnvelopeFromRunPlan({ host: { config: {} } }), null)
  const shape = safetyEnvelopeFromRunPlan({ host: { config: {
    collision_cylinder_radius_m: 0.25, collision_cylinder_offset_m: 0.18,
    collision_clearance_above_m: 0.1, collision_clearance_below_m: 0.1,
  } } })!
  const envelope = createSafetyEnvelope(shape)
  const bounds = new THREE.Box3().setFromObject(envelope)
  assert.ok(Math.abs(bounds.max.x - 0.43) < 1e-6)
  assert.ok(Math.abs(bounds.min.x + 0.43) < 1e-6)
  assert.ok(Math.abs(bounds.max.z - 0.25) < 1e-6)
  assert.ok(Math.abs(bounds.max.y - 0.1) < 1e-6)
  const robot = new THREE.Group()
  robot.position.set(2, 0.4, -3)
  robot.rotation.y = Math.PI / 2
  robot.add(envelope)
  const front = envelope.getObjectByName('front-query-center')!.getWorldPosition(new THREE.Vector3())
  assert.ok(front.distanceTo(new THREE.Vector3(2, 0.4, -3.18)) < 1e-6)
})
