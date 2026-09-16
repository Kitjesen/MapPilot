import assert from 'node:assert/strict'
import test from 'node:test'

import { presentNavigationStatus } from '../src/services/navigationStatus.ts'
import type { NavigationStatusResponse } from '../src/types/index.ts'

type StatusInput = {
  task?: NavigationStatusResponse['task']['state']
  admission?: NavigationStatusResponse['goal_admission']['state']
  authority?: NavigationStatusResponse['control']['authority']
  permission?: NavigationStatusResponse['motion']['permission']
  observation?: NavigationStatusResponse['motion']['observation']
  stop?: NavigationStatusResponse['motion']['stop_confirmation']
}

function status(input: StatusInput = {}): NavigationStatusResponse {
  return {
    schema_version: 3,
    task: { state: input.task ?? 'IDLE', task_id: 'task-1', reason: '' },
    goal_admission: { state: input.admission ?? 'ACCEPTING' },
    control: { authority: input.authority ?? 'AUTONOMY', resume_required: false, reason: '' },
    motion: {
      permission: input.permission ?? 'CLEAR',
      observation: input.observation ?? 'QUIET',
      stop_confirmation: input.stop ?? 'NOT_REQUESTED',
      reason: '',
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

test('quiet observation is distinct from stop confirmation', () => {
  const pending = presentNavigationStatus(status({ stop: 'PENDING' }), 'zh')
  const confirmed = presentNavigationStatus(status({ stop: 'CONFIRMED' }), 'zh')
  const failed = presentNavigationStatus(status({ stop: 'FAILED' }), 'zh')

  assert.equal(pending.motion.observation.label, '静止观测')
  assert.equal(pending.motion.stopConfirmation.label, '等待停稳确认')
  assert.equal(confirmed.motion.stopConfirmation.label, '已确认停稳')
  assert.equal(failed.motion.stopConfirmation.label, '停稳确认失败')
})

test('missing status reports every axis as unknown', () => {
  const view = presentNavigationStatus(null, 'zh')

  assert.equal(view.task.state, 'UNKNOWN')
  assert.equal(view.goalAdmission.state, 'UNKNOWN')
  assert.equal(view.control.state, 'UNKNOWN')
  assert.equal(view.motion.permission.label, '运动权限未知')
  assert.equal(view.motion.observation.label, '运动状态未知')
  assert.equal(view.motion.stopConfirmation.label, '停稳状态未知')
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
