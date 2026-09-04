import assert from 'node:assert/strict'
import test from 'node:test'

import { presentNavigationOperatorStatus } from '../src/services/navigationOperatorState.ts'
import type { NavigationStatusResponse } from '../src/types/index.ts'

test('an SSE navigation status immediately replaces stale mission state with recovering', () => {
  const navigationStatus = {
    operator_state: {
      schema_version: 1,
      task: {
        state: 'RECOVERING',
        task_id: 'task-1',
        request_id: 'request-1',
        terminal: false,
        progress: 0.4,
        reason: 'local_recovery',
      },
      goal_admission: {
        state: 'ACCEPTING',
        blockers: [],
        advisories: [],
      },
      control: {
        authority: 'AUTONOMY',
        resume_required: false,
        reason: '',
      },
      motion: {
        permission: 'CLEAR',
        observation: 'MOVING',
        stop_confirmation: 'NOT_REQUESTED',
        linear_speed_mps: 0.2,
        angular_speed_radps: 0,
        reason: '',
      },
      summary: {
        severity: 'INFO',
        code: 'TASK_RECOVERING',
        next_action: 'WAIT_FOR_RECOVERY',
      },
    },
  } as NavigationStatusResponse

  const view = presentNavigationOperatorStatus(navigationStatus, {
    locale: 'zh',
    legacyTaskState: 'IDLE',
  })

  assert.equal(view.source, 'operator_state')
  assert.equal(view.task.state, 'RECOVERING')
  assert.equal(view.task.label, '恢复中')
})

test('paused and successful tasks use the operator vocabulary', () => {
  const status = (state: 'PAUSED' | 'SUCCESS') => ({
    operator_state: {
      task: { state },
    },
  }) as NavigationStatusResponse

  assert.equal(
    presentNavigationOperatorStatus(status('PAUSED'), { locale: 'zh' }).task.label,
    '已暂停',
  )
  assert.equal(
    presentNavigationOperatorStatus(status('SUCCESS'), { locale: 'zh' }).task.label,
    '已到达',
  )
})

test('a motion hold does not rewrite an executing task as paused', () => {
  const navigationStatus = {
    operator_state: {
      task: { state: 'EXECUTING' },
      motion: {
        permission: 'HELD',
        observation: 'QUIET',
        stop_confirmation: 'NOT_REQUESTED',
      },
    },
  } as NavigationStatusResponse

  const view = presentNavigationOperatorStatus(navigationStatus, { locale: 'zh' })

  assert.equal(view.task.state, 'EXECUTING')
  assert.equal(view.task.label, '执行中')
  assert.equal(view.motion.permission.state, 'HELD')
  assert.equal(view.motion.permission.label, '运动保持')
})

test('quiet motion with pending stop evidence is not presented as confirmed stopped', () => {
  const navigationStatus = {
    operator_state: {
      task: { state: 'CANCELLED' },
      motion: {
        permission: 'HELD',
        observation: 'QUIET',
        stop_confirmation: 'PENDING',
      },
    },
  } as NavigationStatusResponse

  const view = presentNavigationOperatorStatus(navigationStatus, { locale: 'zh' })

  assert.equal(view.motion.observation.label, '静止观测')
  assert.equal(view.motion.stopConfirmation.label, '等待停稳确认')
  assert.notEqual(view.motion.stopConfirmation.label, '已确认停稳')
})

test('a legacy response without stop evidence reports stop confirmation as unknown', () => {
  const view = presentNavigationOperatorStatus(null, {
    locale: 'zh',
    legacyTaskState: 'EXECUTING',
  })

  assert.equal(view.source, 'legacy')
  assert.equal(view.motion.permission.label, '运动权限未知')
  assert.equal(view.motion.observation.label, '运动状态未知')
  assert.equal(view.motion.stopConfirmation.label, '停稳状态未知')
  assert.notEqual(view.motion.stopConfirmation.label, '已确认停稳')
})

test('explicit confirmed and failed stop evidence have distinct labels', () => {
  const status = (stopConfirmation: 'CONFIRMED' | 'FAILED') => ({
    operator_state: {
      task: { state: 'CANCELLED' },
      motion: {
        permission: 'HELD',
        observation: 'QUIET',
        stop_confirmation: stopConfirmation,
      },
    },
  }) as NavigationStatusResponse

  assert.equal(
    presentNavigationOperatorStatus(status('CONFIRMED'), { locale: 'zh' })
      .motion.stopConfirmation.label,
    '已确认停稳',
  )
  assert.equal(
    presentNavigationOperatorStatus(status('FAILED'), { locale: 'zh' })
      .motion.stopConfirmation.label,
    '停稳确认失败',
  )
})

test('goal admission, control authority, motion permission, and next action stay independent', () => {
  const navigationStatus = {
    operator_state: {
      task: { state: 'EXECUTING', reason: 'input_gate_stale' },
      goal_admission: {
        state: 'BLOCKED',
        blockers: ['input_gate_stale'],
        advisories: [],
      },
      control: {
        authority: 'OPERATOR',
        resume_required: true,
        reason: 'manual_override',
      },
      motion: {
        permission: 'HELD',
        observation: 'QUIET',
        stop_confirmation: 'NOT_REQUESTED',
      },
      summary: {
        severity: 'WARNING',
        code: 'GOAL_ADMISSION_BLOCKED',
        next_action: 'resolve_goal_blockers',
      },
    },
  } as NavigationStatusResponse

  const view = presentNavigationOperatorStatus(navigationStatus, { locale: 'zh' })

  assert.equal(view.task.state, 'EXECUTING')
  assert.equal(view.goalAdmission.label, '目标受阻')
  assert.equal(view.control.label, '操作者控制')
  assert.equal(view.motion.permission.label, '运动保持')
  assert.equal(view.summary.nextAction, '处理目标准入阻塞')
  assert.notEqual(view.summary.nextAction, 'input_gate_stale')
})

test('the complete task vocabulary has stable Chinese labels', () => {
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
    const navigationStatus = {
      operator_state: { task: { state } },
    } as NavigationStatusResponse
    assert.equal(
      presentNavigationOperatorStatus(navigationStatus, { locale: 'zh' }).task.label,
      expected,
    )
  }
})

test('nominal and emergency operator axes have stable user-facing labels', () => {
  const status = (input: {
    admission: 'ACCEPTING' | 'UNKNOWN'
    authority: 'AUTONOMY' | 'NONE'
    permission: 'CLEAR' | 'ESTOPPED'
    observation: 'MOVING' | 'UNKNOWN'
    stop: 'NOT_REQUESTED' | 'UNKNOWN'
  }) => ({
    operator_state: {
      task: { state: 'IDLE' },
      goal_admission: { state: input.admission, blockers: [], advisories: [] },
      control: { authority: input.authority, resume_required: false },
      motion: {
        permission: input.permission,
        observation: input.observation,
        stop_confirmation: input.stop,
      },
    },
  }) as NavigationStatusResponse

  const nominal = presentNavigationOperatorStatus(status({
    admission: 'ACCEPTING',
    authority: 'AUTONOMY',
    permission: 'CLEAR',
    observation: 'MOVING',
    stop: 'NOT_REQUESTED',
  }), { locale: 'zh' })
  assert.equal(nominal.goalAdmission.label, '可接收目标')
  assert.equal(nominal.control.label, '自主控制')
  assert.equal(nominal.motion.permission.label, '允许运动')
  assert.equal(nominal.motion.observation.label, '运动中')
  assert.equal(nominal.motion.stopConfirmation.label, '未请求停稳')

  const emergency = presentNavigationOperatorStatus(status({
    admission: 'UNKNOWN',
    authority: 'NONE',
    permission: 'ESTOPPED',
    observation: 'UNKNOWN',
    stop: 'UNKNOWN',
  }), { locale: 'zh' })
  assert.equal(emergency.control.label, '无控制者')
  assert.equal(emergency.motion.permission.label, '急停保持')
})

test('legacy navigation fields remain usable without inventing stop confirmation', () => {
  const navigationStatus = {
    can_accept_goal: true,
    readiness: {
      can_accept_goal: true,
      blockers: [],
      advisories: ['speed_limited'],
    },
    control: {
      manual_override: true,
      preempting_autonomy: true,
      active_cmd_source: 'teleop',
    },
    motion: {
      speed_policy: { mode: 'hold' },
    },
  } as NavigationStatusResponse

  const view = presentNavigationOperatorStatus(navigationStatus, {
    locale: 'zh',
    legacyTaskState: 'EXECUTING',
  })

  assert.equal(view.source, 'legacy')
  assert.equal(view.goalAdmission.state, 'ACCEPTING')
  assert.deepEqual(view.goalAdmission.advisories, ['speed_limited'])
  assert.equal(view.control.state, 'OPERATOR')
  assert.equal(view.motion.permission.state, 'HELD')
  assert.equal(view.motion.stopConfirmation.state, 'UNKNOWN')
})
