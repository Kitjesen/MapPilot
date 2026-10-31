import assert from 'node:assert/strict'
import test from 'node:test'
import type { AgentTaskStatusResponse, VisualServoStatus } from '../src/types/index.ts'
import { deriveSceneTarget, type SceneTargetInput } from '../src/services/sceneTarget.ts'

function input(): SceneTargetInput {
  return {
    task: {
      found: true, run_id: 'agent-1', state: 'running',
      navigation: { owned: true, follow_task_id: 'follow-1', product_session_id: 'product-1' },
    },
    sseState: {
      connected: true,
      session: { product_session_id: 'product-1' } as SceneTargetInput['sseState']['session'],
      stateSnapshot: { schema_version: 4, ts: 100 },
      stateSnapshotReceivedAt: 200_000,
      visualServoStatus: {
        mode: 'follow', follow_task_id: 'follow-1', product_session_id: 'product-1',
        state: 'following', target: '穿蓝衣服的人', target_visible: true, ts: 100,
        person: { position: [4, 5, 1], last_seen: 100 },
      } as VisualServoStatus & { ts: number },
    },
    nowS: 201,
    cameraLive: true,
  }
}

test('fresh Agent-owned follow target presents state without fabricating spatial evidence', () => {
  const target = deriveSceneTarget(input())!
  assert.equal(target.state, 'following')
  assert.equal(target.targetLabel, '穿蓝衣服的人')
  assert.equal(target.cameraVisible, true)
  assert.equal(target.marker, null, 'raw person position has no map/frame epoch')
  assert.equal('bbox' in target, false, 'transport does not align bbox to the displayed camera frame')
})

test('a different follow task or Product session never appears as the selected target', () => {
  for (const patch of [
    { follow_task_id: 'follow-2' }, { product_session_id: 'product-2' },
  ]) {
    const value = input()
    Object.assign(value.sseState.visualServoStatus!, patch)
    const target = deriveSceneTarget(value)!
    assert.equal(target.state, 'unavailable')
    assert.equal(target.targetLabel, null)
    assert.equal(target.marker, null)
  }
  const restarted = input()
  restarted.sseState.session!.product_session_id = 'product-2'
  assert.equal(deriveSceneTarget(restarted)?.state, 'unavailable')
})

test('stale, disconnected, or unreported timestamp never keeps a following label', () => {
  const stale = input()
  stale.nowS = 204
  assert.equal(deriveSceneTarget(stale)?.state, 'unavailable')
  const disconnected = input()
  disconnected.sseState.connected = false
  assert.equal(deriveSceneTarget(disconnected)?.state, 'unavailable')
  const missing = input()
  delete (missing.sseState.visualServoStatus as VisualServoStatus & { ts?: number }).ts
  assert.equal(deriveSceneTarget(missing)?.state, 'unavailable')
  const stalePerception = input()
  stalePerception.sseState.visualServoStatus!.reason = 'visual_servo_perception_stale'
  assert.equal(deriveSceneTarget(stalePerception)?.state, 'unavailable')
})

test('lost and reacquiring clear any implication of visible target lock', () => {
  const lost = input()
  lost.sseState.visualServoStatus!.target_visible = false
  assert.equal(deriveSceneTarget(lost)?.state, 'lost')
  for (const state of ['selecting', 'reacquiring', 'lost'] as const) {
    const value = input()
    value.sseState.visualServoStatus!.state = state
    assert.equal(deriveSceneTarget(value)?.state, state)
    assert.equal(deriveSceneTarget(value)?.marker, null)
  }
})

test('completed, superseded, unowned, and absent Agent tasks show no current follow target', () => {
  for (const patch of [
    { found: false }, { state: 'done' }, { state: 'cancelled' }, { state: 'failed' },
    { navigation: { owned: false, follow_task_id: 'follow-1' } },
    { navigation: { owned: true, follow_task_id: 'follow-1', terminal: true } },
  ] satisfies Partial<AgentTaskStatusResponse>[]) {
    const value = input()
    Object.assign(value.task!, patch)
    assert.equal(deriveSceneTarget(value), null)
  }
  const value = input()
  value.task = null
  assert.equal(deriveSceneTarget(value), null)
})

test('a fresh tracking status does not overlay a stale or hidden camera', () => {
  const value = input()
  value.cameraLive = false
  const target = deriveSceneTarget(value)!
  assert.equal(target.state, 'following')
  assert.equal(target.cameraVisible, false)
})
