import assert from 'node:assert/strict'
import test from 'node:test'

import { presentVisualFollowStatus } from '../src/services/visualFollowStatus.ts'
import type { NavigationStatusResponse, VisualServoStatus } from '../src/types/index.ts'

function visual(overrides: Partial<VisualServoStatus> = {}): VisualServoStatus {
  return {
    mode: 'follow',
    target: 'person in red',
    target_id: 'person-1',
    select: 'clip',
    follow_available: true,
    target_visible: true,
    state: 'following',
    reason: '',
    navigation_state: 'executing',
    navigation_task_id: 'follow-task',
    navigation_reason: '',
    goal_rate_hz: 2,
    robot_position: [0, 0, 0],
    goal_position: [1, 0, 0],
    distance_m: 2,
    desired_distance_m: 1.5,
    person: null,
    ...overrides,
  }
}

function navigation(overrides: Partial<NavigationStatusResponse> = {}): NavigationStatusResponse {
  return {
    schema_version: 3,
    task: { state: 'EXECUTING', task_id: 'follow-task', reason: '' },
    goal_admission: { state: 'ACCEPTING', reason: '' },
    control: { authority: 'AUTONOMY', resume_required: false, reason: '' },
    motion: {
      permission: 'CLEAR',
      observation: 'QUIET',
      stop_confirmation: 'NOT_REQUESTED',
      reason: '',
    },
    ts: 1,
    ...overrides,
  }
}

test('a locked visual target does not claim that the robot is moving', () => {
  const view = presentVisualFollowStatus(visual(), navigation(), 'zh')

  assert.equal(view.visual.label, '目标已锁定')
  assert.equal(view.motion.observation, '静止观测')
  assert.equal(view.motion.stopConfirmation, '未请求停稳')
  assert.equal(view.motion.stopConfirmed, false)
  assert.doesNotMatch(`${view.visual.label}${view.motion.observation}`, /正在跟人|已停稳/)
})

test('normal native motion and visual cancellation receipts do not become failures', () => {
  for (const reason of ['motion_clear', 'nonzero_output_published', 'stop_confirmed']) {
    const view = presentVisualFollowStatus(visual({
      navigation_state: 'cancel_requested', navigation_reason: 'visual_servo_stop',
    }), navigation({
      motion: {
        permission: 'CLEAR', observation: 'QUIET', stop_confirmation: 'NOT_REQUESTED', reason,
      },
    }), 'zh')
    assert.equal(view.motion.reasonMessage, '')
    assert.equal(view.navigation?.message, '')
    assert.equal(view.motion.stopConfirmed, false)
  }
  const lost = presentVisualFollowStatus(visual({
    state: 'reacquiring', navigation_state: 'cancel_requested',
    navigation_reason: 'visual_servo_target_lost',
  }), null, 'zh')
  assert.equal(lost.navigation?.message, '目标人员已丢失，已请求取消跟随导航。')
})

test('reacquisition and verification reasons explain the visual stage', () => {
  const reacquiring = presentVisualFollowStatus(visual({
    state: 'reacquiring',
    reason: 'identity_verification_running',
  }), navigation(), 'zh')
  const position = presentVisualFollowStatus(visual({
    state: 'lost',
    reason: 'position_rejected',
  }), navigation(), 'en')

  assert.equal(reacquiring.visual.label, '正在重新识别人员')
  assert.equal(reacquiring.visual.message, '正在核验人员身份')
  assert.equal(position.visual.message, 'The observed position was rejected')
})

test('only confirmed stop confirmation reports a confirmed physical stop', () => {
  for (const stop_confirmation of ['NOT_REQUESTED', 'PENDING', 'FAILED', 'UNKNOWN'] as const) {
    const view = presentVisualFollowStatus(visual({ state: 'stopping' }), navigation({
      motion: {
        permission: 'HELD',
        observation: 'QUIET',
        stop_confirmation,
        reason: '',
      },
    }), 'zh')
    assert.equal(view.motion.stopConfirmed, false)
    assert.doesNotMatch(view.motion.stopConfirmation, /已确认停稳/)
  }

  const confirmed = presentVisualFollowStatus(visual({ state: 'idle' }), navigation({
    motion: {
      permission: 'HELD',
      observation: 'QUIET',
      stop_confirmation: 'CONFIRMED',
      reason: '',
    },
  }), 'zh')
  assert.equal(confirmed.motion.stopConfirmed, true)
  assert.equal(confirmed.motion.stopConfirmation, '已确认停稳')
})

test('native task reasons are attached to follow navigation only for the same task', () => {
  const matching = presentVisualFollowStatus(visual(), navigation({
    task: { state: 'RECOVERING', task_id: 'follow-task', reason: 'local_blocked' },
  }), 'zh')
  const unrelated = presentVisualFollowStatus(visual(), navigation({
    task: { state: 'FAILED', task_id: 'other-task', reason: 'local_blocked' },
  }), 'zh')

  assert.equal(matching.navigation?.message, '局部路线受阻。')
  assert.equal(unrelated.navigation?.message, '')
  assert.equal(unrelated.motion.observation, '静止观测')
})

test('navigation hold remains separate from visual target visibility', () => {
  const view = presentVisualFollowStatus(visual({ target_visible: true }), navigation({
    motion: {
      permission: 'HELD',
      observation: 'MOVING',
      stop_confirmation: 'PENDING',
      reason: 'local_near_field_stop',
    },
  }), 'zh')

  assert.equal(view.visual.label, '目标已锁定')
  assert.equal(view.navigation?.message, '检测到近场障碍，已请求停车。')
  assert.equal(view.motion.permission, '运动保持')
  assert.equal(view.motion.observation, '运动中')
  assert.equal(view.motion.stopConfirmation, '等待停稳确认')
})

test('accepted acknowledgement yields to the current matching native hold', () => {
  const view = presentVisualFollowStatus(visual({ navigation_reason: 'accepted' }), navigation({
    motion: {
      permission: 'HELD',
      observation: 'QUIET',
      stop_confirmation: 'NOT_REQUESTED',
      reason: 'localization_not_tracking',
    },
  }), 'zh')

  assert.equal(view.navigation?.message, '定位尚未进入跟踪状态，运动指令已保持。')
  assert.equal(view.motion.reasonMessage, '定位尚未进入跟踪状态，运动指令已保持。')
  assert.doesNotMatch(view.navigation?.message ?? '', /条件未满足|失败/)
})

test('normal navigation lifecycle acknowledgements have no error copy', () => {
  for (const navigation_reason of ['accepted', 'path_active', 'cancel_request', 'cancel_requested']) {
    const view = presentVisualFollowStatus(visual({ navigation_reason }), navigation(), 'zh')
    assert.equal(view.navigation?.message, '')
  }
})

test('first goal rejection explains localization and trajectory-start collision', () => {
  const localization = presentVisualFollowStatus(visual({
    navigation_state: 'rejected',
    navigation_reason: 'localization_not_tracking',
  }), null, 'zh')
  const collision = presentVisualFollowStatus(visual({
    navigation_state: 'rejected',
    navigation_reason: 'collision_at_trajectory_start',
  }), null, 'en')

  assert.equal(localization.navigation?.message, '定位尚未进入跟踪状态，运动指令已保持。')
  assert.doesNotMatch(localization.navigation?.message ?? '', /已停止|已停稳/)
  assert.equal(collision.navigation?.message, 'A collision exists at the trajectory start; motion was not authorized.')
})

test('global motion hold reason stays visible without becoming a follow failure', () => {
  const view = presentVisualFollowStatus(visual({ navigation_reason: 'accepted' }), navigation({
    task: { state: 'EXECUTING', task_id: 'other-task', reason: 'local_blocked' },
    motion: {
      permission: 'HELD',
      observation: 'QUIET',
      stop_confirmation: 'PENDING',
      reason: 'collision_at_trajectory_start',
    },
  }), 'zh')

  assert.equal(view.navigation?.message, '')
  assert.equal(view.motion.reasonMessage, '轨迹起点存在碰撞，运动指令未获放行。')
})

test('reached is a native visual navigation lifecycle state', () => {
  const view = presentVisualFollowStatus(visual({
    navigation_state: 'reached',
    navigation_reason: '',
  }), navigation(), 'zh')

  assert.equal(view.navigation?.label, '已到达跟随目标')
})
