import test from 'node:test'
import assert from 'node:assert/strict'
import { createJointTelemetryStream } from '../src/services/jointTelemetryStream.ts'
import { GO2_JOINT_NAMES } from '../src/services/robotJointState.ts'

const event = (stamp: number) => ({ type: 'joint_state', stamp, robot_model: 'go2', source_age_s: .01,
  names: [...GO2_JOINT_NAMES], position: Array(12).fill(.2), velocity: Array(12).fill(0), effort: Array(12).fill(0) })

test('joint stream notifies only its subscribers for new complete frames and preserves snapshot identity otherwise', () => {
  const stream = createJointTelemetryStream()
  let notifications = 0
  const unsubscribe = stream.subscribe(() => notifications++)
  stream.setConnected(true)
  assert.equal(stream.ingest(event(100), 1000), true)
  const snapshot = stream.getSnapshot()
  assert.equal(stream.ingest(event(100), 1100), false)
  assert.equal(stream.ingest(event(99), 1200), false)
  assert.equal(stream.ingest({ ...event(101), names: [] }, 1300), false)
  assert.equal(stream.ingest({ ...event(101), source_age_s: 1 }, 1400), false)
  assert.equal(stream.getSnapshot(), snapshot)
  assert.equal(notifications, 2)
  unsubscribe()
  stream.ingest(event(102), 1500)
  assert.equal(notifications, 2)
})

test('disconnect and reconnect preserve original sample age without replaying old measurements', () => {
  const stream = createJointTelemetryStream()
  stream.setConnected(true)
  stream.ingest(event(100), 1000)
  const sample = stream.getSnapshot().sample
  stream.setConnected(false)
  assert.equal(stream.getSnapshot().connected, false)
  assert.equal(stream.getSnapshot().sample, sample)
  stream.setConnected(true)
  assert.equal(stream.getSnapshot().sample?.receivedAtMs, 1000)
  assert.equal(stream.ingest(event(100), 5000), false)
  assert.equal(stream.ingest(event(101), 5000), true)
  assert.equal(stream.getSnapshot().sample?.receivedAtMs, 5000)
})
