import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'

import { createGo2Model } from '../src/components/scene3d/robot/go2Model.ts'
import { GO2_URDF } from '../src/components/scene3d/robot/go2UrdfSpec.ts'
import {
  GO2_JOINT_NAMES,
  go2JointSampleStatus,
  parseGo2JointState,
  type Go2JointName,
  type Go2JointSample,
} from '../src/services/robotJointState.ts'

const near = (actual: number, expected: number) => assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`)

function event(position = GO2_JOINT_NAMES.map((_, index) => (index - 5.5) * 0.07)) {
  return {
    type: 'joint_state', stamp: 100, source_age_s: 0.02, robot_model: 'go2',
    names: [...GO2_JOINT_NAMES], position, velocity: [], effort: [],
  }
}

function sample(stamp: number, receivedAtMs: number, scale = 1): Go2JointSample {
  return parseGo2JointState({
    ...event(GO2_JOINT_NAMES.map((_, index) => scale * (index - 5.5) * 0.07)), stamp,
  }, receivedAtMs)!
}

test('Go2 joint payload maps all 12 unique canonical names and rejects malformed or stale input', () => {
  const reversedNames = [...GO2_JOINT_NAMES].reverse()
  const reversedPositions = reversedNames.map((_, index) => index * 0.1)
  const parsed = parseGo2JointState({
    ...event(), names: reversedNames, position: reversedPositions,
  }, 250)
  assert.ok(parsed)
  reversedNames.forEach((name, index) => near(parsed.positions[name], reversedPositions[index]))

  assert.equal(parseGo2JointState({ ...event(), robot_model: 'thunder_v4' }, 0), null)
  assert.equal(parseGo2JointState({ ...event(), names: GO2_JOINT_NAMES.slice(0, 11), position: Array(11).fill(0) }, 0), null)
  assert.equal(parseGo2JointState({ ...event(), names: GO2_JOINT_NAMES.map((name, index) => index === 1 ? GO2_JOINT_NAMES[0] : name) }, 0), null)
  assert.equal(parseGo2JointState({ ...event(), position: GO2_JOINT_NAMES.map((_, index) => index === 4 ? NaN : 0) }, 0), null)
  assert.equal(parseGo2JointState({ ...event(), source_age_s: 0.501 }, 0), null)
})

test('joint sample status advances from backend source age using only monotonic receipt time', () => {
  const parsed = parseGo2JointState(event(), 1000)!
  assert.equal(go2JointSampleStatus(null, 1000), 'fallback')
  assert.equal(go2JointSampleStatus(parsed, 1479), 'live')
  assert.equal(go2JointSampleStatus(parsed, 1481), 'stale')
})

test('measured joint rotations preserve every URDF origin and axis sign without accumulating', () => {
  const model = createGo2Model(async () => new THREE.Group())
  const first = sample(1, 100, 1)
  assert.equal(model.ingestJointState(first, 100), true)
  assert.equal(model.updateJointPose(100), true)

  const expectedByName = new Map(GO2_URDF.joints.map(joint => [joint.name, joint]))
  for (const name of GO2_JOINT_NAMES) {
    const spec = expectedByName.get(name)!
    const frame = model.getObjectByName(`go2:joint:${name}`)!
    assert.deepEqual(frame.position.toArray(), spec.origin.xyz)
    const origin = new THREE.Quaternion().setFromEuler(new THREE.Euler(
      spec.origin.rpy[0], spec.origin.rpy[1], spec.origin.rpy[2], 'ZYX',
    ))
    const expected = origin.clone().multiply(new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(...spec.axis).normalize(), first.positions[name],
    ))
    near(1 - Math.abs(frame.quaternion.dot(expected)), 0)
    near(frame.userData.jointAngle, first.positions[name])
  }

  const second = sample(2, 300, -0.4)
  assert.equal(model.ingestJointState(second, 300), true)
  assert.equal(model.updateJointPose(420), true)
  for (const name of GO2_JOINT_NAMES) {
    const spec = expectedByName.get(name)!
    const frame = model.getObjectByName(`go2:joint:${name}`)!
    const expected = new THREE.Quaternion().setFromEuler(new THREE.Euler(
      spec.origin.rpy[0], spec.origin.rpy[1], spec.origin.rpy[2], 'ZYX',
    )).multiply(new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(...spec.axis).normalize(), second.positions[name],
    ))
    near(1 - Math.abs(frame.quaternion.dot(expected)), 0)
  }
  const before = model.getObjectByName('go2:joint:FR_thigh_joint')!.quaternion.clone()
  assert.equal(model.updateJointPose(440), true)
  near(1 - Math.abs(model.getObjectByName('go2:joint:FR_thigh_joint')!.quaternion.dot(before)), 0)
})

test('continuous 30 Hz samples stay within one measured interval without extrapolation', () => {
  const model = createGo2Model(async () => new THREE.Group())
  const joint = model.getObjectByName('go2:joint:FR_hip_joint')!
  let previousTarget = 0
  for (let index = 0; index < 40; index++) {
    const nowMs = 100 + index * 33
    const positions = Object.fromEntries(GO2_JOINT_NAMES.map(name => [name, 0])) as Record<Go2JointName, number>
    const target = Math.sin(index * 0.18) * 0.45
    positions.FR_hip_joint = target
    const next: Go2JointSample = { stamp: 10 + index * 0.033, sourceAgeS: 0.01, receivedAtMs: nowMs, positions }
    assert.equal(model.ingestJointState(next, nowMs), true)
    const rendered = joint.userData.jointAngle as number
    if (index === 0) near(rendered, target)
    else {
      near(rendered, previousTarget)
      assert.ok(Math.abs(target - rendered) <= 0.082, 'display lag must stay within one 30 Hz sample step')
    }
    model.updateJointPose(nowMs + 16.5)
    near(joint.userData.jointAngle, previousTarget + (target - previousTarget) * 0.5)
    model.updateJointPose(nowMs + 33)
    near(joint.userData.jointAngle, target)
    previousTarget = target
  }
})

test('stale and out-of-order samples freeze the last measured pose', () => {
  const model = createGo2Model(async () => new THREE.Group())
  assert.equal(model.jointStateStatus(0), 'fallback')
  assert.equal(model.ingestJointState(sample(2, 100), 100), true)
  model.updateJointPose(220)
  const before = model.getObjectByName('go2:joint:FR_calf_joint')!.quaternion.clone()
  assert.equal(model.ingestJointState(sample(1, 250, -1), 250), false)
  assert.equal(model.updateJointPose(700), false)
  assert.equal(model.jointStateStatus(700), 'stale')
  near(1 - Math.abs(model.getObjectByName('go2:joint:FR_calf_joint')!.quaternion.dot(before)), 0)
  const almostStale = { ...sample(3, 800), sourceAgeS: 0.49 }
  assert.equal(model.ingestJointState(almostStale, 811), false)
})
