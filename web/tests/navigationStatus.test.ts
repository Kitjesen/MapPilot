import assert from 'node:assert/strict'
import test from 'node:test'

import { liveNavigationStatus, navigationTaskReasonMessage, operatorReasonMessage, presentNavigationStatus } from '../src/services/navigationStatus.ts'
import type { NavigationStatusResponse } from '../src/types/index.ts'

type StatusInput = {
  task?: NavigationStatusResponse['task']['state']
  admission?: NavigationStatusResponse['goal_admission']['state']
  authority?: NavigationStatusResponse['control']['authority']
  permission?: NavigationStatusResponse['motion']['permission']
  observation?: NavigationStatusResponse['motion']['observation']
  stop?: NavigationStatusResponse['motion']['stop_confirmation']
  taskReason?: string
  admissionReason?: string
  motionReason?: string
}

function status(input: StatusInput = {}): NavigationStatusResponse {
  return {
    schema_version: 3,
    task: { state: input.task ?? 'IDLE', task_id: 'task-1', reason: input.taskReason ?? '' },
    goal_admission: { state: input.admission ?? 'ACCEPTING', reason: input.admissionReason ?? '' },
    control: { authority: input.authority ?? 'AUTONOMY', resume_required: false, reason: '' },
    motion: {
      permission: input.permission ?? 'CLEAR',
      observation: input.observation ?? 'QUIET',
      stop_confirmation: input.stop ?? 'NOT_REQUESTED',
      reason: input.motionReason ?? '',
    },
    ts: 1,
  }
}

test('v3 navigation status uses the complete task vocabulary', () => {
  const labels = new Map([
    ['IDLE', '空闲'],
    ['PLANNING', '规划中'],
    ['EXECUTING', '执行中'],
    ['RECOVERING', '恢复中'],
    ['PAUSED', '已暂停'],
    ['SUCCESS', '已到达'],
    ['FAILED', '失败'],
    ['CANCELLED', '已取消'],
    ['UNKNOWN', '未知'],
  ] as const)

  for (const [state, expected] of labels) {
    assert.equal(presentNavigationStatus(status({ task: state }), 'zh').task.label, expected)
  }
})

test('motion hold does not rewrite the task lifecycle', () => {
  const view = presentNavigationStatus(status({
    task: 'EXECUTING',
    admission: 'BLOCKED',
    authority: 'OPERATOR',
    permission: 'HELD',
  }), 'zh')

  assert.equal(view.task.label, '执行中')
  assert.equal(view.goalAdmission.label, '目标受阻')
  assert.equal(view.control.label, '操作者控制')
  assert.equal(view.motion.permission.label, '运动保持')
})

test('automatic route waiting is visible on an executing task without claiming stop confirmation', () => {
  for (const task of ['EXECUTING', 'RECOVERING'] as const) {
    const sample = status({
      task,
      taskReason: 'replan_waiting',
      observation: 'MOVING',
      stop: 'PENDING',
    })
    const view = presentNavigationStatus(sample, 'zh')

    assert.equal(view.task.state, task)
    assert.equal(view.task.label, '等待通路，自动继续')
    assert.equal(view.motion.observation.state, 'MOVING')
    assert.equal(view.motion.stopConfirmation.state, 'PENDING')
    assert.equal(view.motion.permission.state, 'CLEAR')
    assert.equal(
      presentNavigationStatus(sample, 'en').task.label,
      'Waiting for a clear route; navigation will continue automatically',
    )
  }
})

test('legacy exhausted retries remain failed and explain why automatic retry ended', () => {
  for (const reason of ['replan_budget_consumed', 'replan_failed_budget_consumed']) {
    const view = presentNavigationStatus(status({ task: 'FAILED', taskReason: reason }), 'zh')

    assert.equal(view.task.state, 'FAILED')
    assert.equal(view.task.label, '失败')
    assert.match(operatorReasonMessage(reason, 'zh'), /自动重试机会已用完/)
    assert.match(operatorReasonMessage(reason, 'en'), /retry attempts have been exhausted/)
    assert.doesNotMatch(operatorReasonMessage(reason, 'zh'), /自动继续|停稳|已暂停/)
  }
})

test('initial route retry remains planning and does not imply an active path or confirmed stop', () => {
  const sample = status({ task: 'PLANNING', taskReason: 'planning_waiting', stop: 'NOT_REQUESTED' })
  const view = presentNavigationStatus(sample, 'zh')
  assert.equal(view.task.state, 'PLANNING')
  assert.equal(view.task.label, '等待通路，自动重试规划')
  assert.equal(view.motion.stopConfirmation.state, 'NOT_REQUESTED')
  assert.equal(presentNavigationStatus(sample, 'en').task.label,
    'Waiting for a clear route; planning will retry automatically')
})

test('localization hold explains recovery without claiming an unconfirmed physical stop', () => {
  const view = presentNavigationStatus(status({
    task: 'EXECUTING',
    admission: 'BLOCKED',
    permission: 'HELD',
    stop: 'NOT_REQUESTED',
    admissionReason: 'localization_not_tracking',
    motionReason: 'localization_not_tracking',
  }), 'zh')

  assert.equal(view.task.state, 'EXECUTING')
  assert.equal(view.motion.reason, 'localization_not_tracking')
  assert.equal(view.motion.message, '定位恢复中，运动指令已保持')
  assert.doesNotMatch(view.motion.message, /机器人已停止|机器人停稳/)
  assert.equal(view.goalAdmission.message, '定位恢复中，新目标会排队等待输入恢复')
})

test('localization hold reports stop command confirmation separately from motion', () => {
  const view = presentNavigationStatus(status({
    permission: 'HELD',
    stop: 'CONFIRMED',
    motionReason: 'localization_not_tracking',
  }), 'zh')

  assert.equal(view.motion.message, '定位恢复中，停止命令已确认')
})

test('legacy localization timeout stays a generic native reason', () => {
  assert.equal(
    navigationTaskReasonMessage('localization_recovery_timeout:localization_not_tracking'),
    'localization_recovery_timeout:localization_not_tracking',
  )
  assert.equal(
    navigationTaskReasonMessage('localization_recovery_timeout:localization_health_stale'),
    'localization_recovery_timeout:localization_health_stale',
  )
  assert.equal(navigationTaskReasonMessage('local_recovery_exhausted'), 'local_recovery_exhausted')
})

test('operator reason copy distinguishes blockers and preserves an unknown reason', () => {
  assert.equal(
    operatorReasonMessage('local_near_field_stop', 'zh'),
    '检测到近场障碍，已请求停车。',
  )
  assert.equal(
    operatorReasonMessage('start_body_occupied:voxel_17', 'en'),
    'The robot body intersects a mapped obstacle at the start.',
  )
  assert.equal(
    operatorReasonMessage('planner_internal_code', 'zh'),
    '导航状态：planner_internal_code',
  )
  assert.equal(operatorReasonMessage('support_disconnected', 'zh'), '起点与目标之间的地面支撑尚未连通。')
  assert.equal(operatorReasonMessage('input_gate_collision_stale', 'zh'), '等待实时雷达地图更新。')
  assert.equal(operatorReasonMessage('driver_control_stale', 'zh'), '等待驱动连接恢复。')
  assert.equal(operatorReasonMessage('takeover_active', 'zh'), '人工接管尚未退出，请恢复自主导航。')
  assert.equal(operatorReasonMessage('operator_takeover_resume_required', 'zh'), '人工接管尚未退出，请恢复自主导航。')
  assert.equal(operatorReasonMessage('control_hold', 'zh'), '运动暂时保持，等待控制恢复。')
})

test('short motion reasons', () => {
  const reasons = [
    ['brake_blocked', '制动空间受阻，已请求停车。', 'Braking space is blocked. A stop was requested.'],
    ['cmd_blocked', '速度指令没有可用制动空间，已请求停车。', 'The command has no braking space. A stop was requested.'],
    ['cmd_limited', '速度指令已按制动空间降低。', 'Command speed was reduced to fit braking space.'],
    ['loop_hold', '控制循环暂停运动，等待恢复。', 'Motion is held while the control loop recovers.'],
    ['takeover_active', '人工接管尚未退出，请恢复自主导航。', 'Manual takeover is still active; resume autonomous navigation.'],
  ] as const
  for (const [reason, zh, en] of reasons) {
    assert.equal(operatorReasonMessage(reason, 'zh'), zh)
    assert.equal(operatorReasonMessage(reason, 'en'), en)
    assert.equal(operatorReasonMessage(`input_gate_${reason}:detail`, 'zh'), zh)
  }
})

test('follow admission failures have truthful operator copy without claiming a stop', () => {
  assert.equal(
    operatorReasonMessage('localization_not_tracking', 'zh'),
    '定位尚未进入跟踪状态，运动指令已保持。',
  )
  assert.equal(
    operatorReasonMessage('collision_at_trajectory_start', 'en'),
    'A collision exists at the trajectory start; motion was not authorized.',
  )
  assert.doesNotMatch(operatorReasonMessage('localization_not_tracking', 'zh'), /已停止|已停稳/)
})

const stairReasons = [
  ['stair_approach', '正在接近楼梯入口。', 'Approaching the stair entrance.'],
  ['stair_aligning', '正在对齐楼梯入口朝向。', 'Aligning with the stair entrance.'],
  ['stair_switching', '正在切换楼梯模式。', 'Switching to the stair motion profile.'],
  ['stair_ascending', '正在上楼。', 'Ascending the stairs.'],
  ['stair_descending', '正在下楼。', 'Descending the stairs.'],
  ['stair_settling', '等待机器人停稳后恢复行走模式。', 'Waiting for the robot to settle before restoring the walking profile.'],
  ['stair_restoring', '正在恢复进入楼梯前的行走模式。', 'Restoring the motion profile used before the stairs.'],
  ['stair_profile_wait', '等待楼梯模式切换确认。', 'Waiting for confirmation of the stair motion profile.'],
  ['stair_profile_unsupported', '当前机器人不支持所需楼梯模式。', 'The robot does not support the required stair motion profile.'],
  ['stair_landing_space', '楼梯入口平台空间不足，无法容纳整机。', 'The stair entrance landing has insufficient space for the whole robot.'],
  ['stair_entry_required', '需要先到楼梯入口平台。', 'The robot must first reach the stair entrance landing.'],
  ['stair_hold_on_flight', '仍处于楼梯段，保留当前楼梯模式。', 'The robot is still on the stair flight; the current stair motion profile is retained.'],
  ['stair_route_wait', '等待满足楼梯约束的安全路线。', 'Waiting for a safe route that meets the stair constraints.'],
  ['stair_exit_target', '等待出口平台满足整机退出条件。', 'Waiting for the exit landing to allow the whole robot to leave the stairs.'],
  ['profile_transition', '正在切换行走模式，运动指令暂时保持。', 'The motion profile is changing; motion commands are temporarily held.'],
  ['driver_control_profile_transition', '正在切换行走模式，运动指令暂时保持。', 'The motion profile is changing; motion commands are temporarily held.'],
  ['input_gate_driver_control_profile_transition', '正在切换行走模式，运动指令暂时保持。', 'The motion profile is changing; motion commands are temporarily held.'],
] as const

test('stair phases and profile handoff have operator copy in both languages', () => {
  for (const [reason, zh, en] of stairReasons) {
    assert.equal(operatorReasonMessage(reason, 'zh'), zh)
    assert.equal(operatorReasonMessage(reason, 'en'), en)
    assert.equal(operatorReasonMessage(`${reason}:native_detail`, 'zh'), zh)
    assert.equal(operatorReasonMessage(`${reason}:native_detail`, 'en'), en)
  }
})

test('stair reasons do not infer a paused task or a confirmed physical stop', () => {
  for (const [reason] of stairReasons) {
    for (const task of ['EXECUTING', 'RECOVERING'] as const) {
      const view = presentNavigationStatus(status({
        task, taskReason: reason, motionReason: reason,
        permission: 'HELD', observation: 'MOVING', stop: 'PENDING',
      }), 'zh')
      assert.equal(view.task.state, task)
      assert.equal(view.task.label, task === 'EXECUTING' ? '执行中' : '恢复中')
      assert.equal(view.motion.permission.state, 'HELD')
      assert.equal(view.motion.observation.state, 'MOVING')
      assert.equal(view.motion.stopConfirmation.state, 'PENDING')
      assert.doesNotMatch(operatorReasonMessage(reason, 'zh'), /已停车|已停止|已停稳|已暂停/)
      assert.doesNotMatch(operatorReasonMessage(reason, 'en'), /has stopped|stop confirmed|paused/i)
    }
  }
})

test('ordinary driver faults are not presented as a planned profile transition', () => {
  for (const reason of ['driver_control_stale', 'driver_control_motor_fault', 'input_gate_driver_control_rejected']) {
    assert.doesNotMatch(operatorReasonMessage(reason, 'zh'), /切换行走模式/)
    assert.doesNotMatch(operatorReasonMessage(reason, 'en'), /profile is changing/)
  }
})

test('stop and SDK timeouts do not claim takeover or a confirmed stop', () => {
  for (const reason of [
    'stop_confirmation_timeout_stop_remains_latched',
    'driver_control_transport_error:stop_unconfirmed:stop_timeout:code=3104',
    'input_gate_driver_control_rpc_timeout:code=3104',
  ]) {
    assert.match(operatorReasonMessage(reason, 'zh'), /超时/)
    assert.match(operatorReasonMessage(reason, 'en'), /timed out/)
    assert.doesNotMatch(operatorReasonMessage(reason, 'zh'), /人工接管|已停稳|已停车|拒绝/)
  }
})

test('quiet observation is distinct from stop confirmation', () => {
  const pending = presentNavigationStatus(status({ stop: 'PENDING' }), 'zh')
  const confirmed = presentNavigationStatus(status({ stop: 'CONFIRMED' }), 'zh')
  const failed = presentNavigationStatus(status({ stop: 'FAILED' }), 'zh')

  assert.equal(pending.motion.observation.label, '静止观测')
  assert.equal(pending.motion.stopConfirmation.label, '等待停止确认')
  assert.equal(confirmed.motion.stopConfirmation.label, '停止命令已确认')
  assert.equal(failed.motion.stopConfirmation.label, '停止确认失败')
  const stillMoving = presentNavigationStatus(status({ stop: 'CONFIRMED', observation: 'MOVING' }), 'zh')
  assert.equal(stillMoving.motion.observation.label, '运动中')
  assert.doesNotMatch(stillMoving.motion.stopConfirmation.label, /停稳/)
})

test('obstacle and stalled reasons do not assert a stopped or paused robot', () => {
  for (const reason of ['local_near_field_stop', 'execution_stalled']) {
    const sample = status({
      task: 'RECOVERING', permission: 'HELD', stop: 'PENDING',
      observation: 'MOVING', motionReason: reason,
    })
    sample.task.reason = reason
    const view = presentNavigationStatus(sample, 'zh')
    assert.equal(view.task.state, 'RECOVERING')
    assert.equal(view.motion.stopConfirmation.state, 'PENDING')
    assert.equal(view.motion.observation.state, 'MOVING')
    assert.doesNotMatch(operatorReasonMessage(reason, 'zh'), /已停车|已停稳|已暂停/)
    assert.doesNotMatch(operatorReasonMessage(reason, 'en'), /has stopped|paused/i)
  }
})

test('missing status reports every axis as unknown', () => {
  const view = presentNavigationStatus(null, 'zh')

  assert.equal(view.task.state, 'UNKNOWN')
  assert.equal(view.goalAdmission.state, 'UNKNOWN')
  assert.equal(view.control.state, 'UNKNOWN')
  assert.equal(view.motion.permission.label, '运动权限未知')
  assert.equal(view.motion.observation.label, '运动状态未知')
  assert.equal(view.motion.stopConfirmation.label, '停止确认状态未知')
})

test('nominal and emergency axes have stable labels', () => {
  const nominal = presentNavigationStatus(status({ observation: 'MOVING' }), 'zh')
  assert.equal(nominal.goalAdmission.label, '可接收目标')
  assert.equal(nominal.control.label, '自主控制')
  assert.equal(nominal.motion.permission.label, '允许运动')
  assert.equal(nominal.motion.observation.label, '运动中')

  const emergency = presentNavigationStatus(status({
    admission: 'UNKNOWN',
    authority: 'NONE',
    permission: 'ESTOPPED',
    observation: 'UNKNOWN',
    stop: 'UNKNOWN',
  }), 'zh')
  assert.equal(emergency.control.label, '无控制者')
  assert.equal(emergency.motion.permission.label, '急停保持')
})

test('stale or disconnected status is never presented as motion clear', () => {
  const fresh = { ...status(), ts: 100 }
  const stream = { connected: true, navigationStatus: fresh, stateSnapshot: null, stateSnapshotReceivedAt: null }

  assert.equal(liveNavigationStatus(stream, 103), fresh)
  assert.equal(liveNavigationStatus(stream, 108), null)
  assert.equal(liveNavigationStatus({ ...stream, connected: false }, 103), null)
  assert.equal(presentNavigationStatus(liveNavigationStatus(stream, 108), 'zh').motion.permission.state, 'UNKNOWN')
})
