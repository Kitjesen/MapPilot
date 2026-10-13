import type { Locale } from '../i18n'
import type {
  NavigationControlAuthority,
  NavigationGoalAdmission,
  NavigationMotionObservation,
  NavigationMotionPermission,
  NavigationStopConfirmation,
  NavigationTaskState,
  NavigationStatusResponse,
  SSEState,
} from '../types/index.ts'
import { estimateSceneTime, freshSource } from './sceneTelemetry.ts'

/** Older navigation status is presented as unknown, never as the last good state. */
export const NAVIGATION_STATUS_MAX_AGE_S = 7

/** Return navigation status only while the stream is connected and the status is fresh. */
export function liveNavigationStatus(
  sseState: Pick<SSEState, 'connected' | 'navigationStatus' | 'stateSnapshot' | 'stateSnapshotReceivedAt'>,
  localNowS: number,
): NavigationStatusResponse | null {
  const status = sseState.navigationStatus
  const nowS = estimateSceneTime(localNowS, sseState.stateSnapshot?.ts, sseState.stateSnapshotReceivedAt)
  return sseState.connected && freshSource(status?.ts, nowS, NAVIGATION_STATUS_MAX_AGE_S) ? status : null
}

interface LabeledState<T extends string> {
  state: T
  label: string
}

export interface NavigationStatusPresentation {
  task: {
    state: NavigationTaskState
    label: string
  }
  goalAdmission: LabeledState<NavigationGoalAdmission> & {
    reason: string
  }
  control: LabeledState<NavigationControlAuthority> & {
    resumeRequired: boolean
  }
  motion: {
    permission: LabeledState<NavigationMotionPermission>
    observation: LabeledState<NavigationMotionObservation>
    stopConfirmation: LabeledState<NavigationStopConfirmation>
  }
}

const TASK_LABELS: Record<NavigationTaskState, { en: string; zh: string }> = {
  IDLE: { en: 'Idle', zh: '空闲' },
  PLANNING: { en: 'Planning', zh: '规划中' },
  EXECUTING: { en: 'Executing', zh: '执行中' },
  RECOVERING: { en: 'Recovering', zh: '恢复中' },
  PAUSED: { en: 'Paused', zh: '已暂停' },
  SUCCESS: { en: 'Reached', zh: '已到达' },
  FAILED: { en: 'Failed', zh: '失败' },
  CANCELLED: { en: 'Cancelled', zh: '已取消' },
  UNKNOWN: { en: 'Unknown', zh: '未知' },
}

const MOTION_PERMISSION_LABELS: Partial<Record<NavigationMotionPermission, { en: string; zh: string }>> = {
  CLEAR: { en: 'Motion clear', zh: '允许运动' },
  HELD: { en: 'Motion held', zh: '运动保持' },
  ESTOPPED: { en: 'Emergency stop held', zh: '急停保持' },
  UNKNOWN: { en: 'Motion permission unknown', zh: '运动权限未知' },
}

const MOTION_OBSERVATION_LABELS: Partial<Record<NavigationMotionObservation, { en: string; zh: string }>> = {
  MOVING: { en: 'Moving', zh: '运动中' },
  QUIET: { en: 'Observed quiet', zh: '静止观测' },
  UNKNOWN: { en: 'Motion state unknown', zh: '运动状态未知' },
}

const STOP_CONFIRMATION_LABELS: Partial<Record<NavigationStopConfirmation, { en: string; zh: string }>> = {
  NOT_REQUESTED: { en: 'Stop not requested', zh: '未请求停稳' },
  PENDING: { en: 'Awaiting stop confirmation', zh: '等待停稳确认' },
  CONFIRMED: { en: 'Stop confirmed', zh: '已确认停稳' },
  FAILED: { en: 'Stop confirmation failed', zh: '停稳确认失败' },
  UNKNOWN: { en: 'Stop confirmation unknown', zh: '停稳状态未知' },
}

const GOAL_ADMISSION_LABELS: Partial<Record<NavigationGoalAdmission, { en: string; zh: string }>> = {
  ACCEPTING: { en: 'Accepting goals', zh: '可接收目标' },
  BLOCKED: { en: 'Goal blocked', zh: '目标受阻' },
  UNKNOWN: { en: 'Goal admission unknown', zh: '目标准入未知' },
}

const CONTROL_AUTHORITY_LABELS: Partial<Record<NavigationControlAuthority, { en: string; zh: string }>> = {
  AUTONOMY: { en: 'Autonomy control', zh: '自主控制' },
  OPERATOR: { en: 'Operator control', zh: '操作者控制' },
  NONE: { en: 'No controller', zh: '无控制者' },
  UNKNOWN: { en: 'Control authority unknown', zh: '控制权未知' },
}

function labelState<T extends string>(
  state: T,
  labels: Partial<Record<T, { en: string; zh: string }>>,
  locale: Locale,
): LabeledState<T> {
  const label = labels[state]
  return { state, label: label ? label[locale] : state }
}

function normalizeTaskState(value: unknown): NavigationTaskState {
  if (typeof value !== 'string' || !value.trim()) return 'UNKNOWN'
  const candidate = value.trim().toUpperCase()
  return candidate in TASK_LABELS ? candidate as NavigationTaskState : 'UNKNOWN'
}

export function navigationTaskLabel(
  state: NavigationTaskState,
  locale: Locale,
): string {
  return TASK_LABELS[state][locale]
}

export function presentNavigationStatus(
  status: NavigationStatusResponse | null | undefined,
  locale: Locale,
): NavigationStatusPresentation {
  const state = normalizeTaskState(status?.task?.state)
  const motion = status?.motion
  const goalAdmission = status?.goal_admission
  const control = status?.control

  return {
    task: { state, label: navigationTaskLabel(state, locale) },
    goalAdmission: {
      ...labelState(goalAdmission?.state ?? 'UNKNOWN', GOAL_ADMISSION_LABELS, locale),
      reason: goalAdmission?.reason ?? '',
    },
    control: {
      ...labelState(control?.authority ?? 'UNKNOWN', CONTROL_AUTHORITY_LABELS, locale),
      resumeRequired: control?.resume_required === true,
    },
    motion: {
      permission: labelState(motion?.permission ?? 'UNKNOWN', MOTION_PERMISSION_LABELS, locale),
      observation: labelState(motion?.observation ?? 'UNKNOWN', MOTION_OBSERVATION_LABELS, locale),
      stopConfirmation: labelState(motion?.stop_confirmation ?? 'UNKNOWN', STOP_CONFIRMATION_LABELS, locale),
    },
  }
}

export function operatorReasonMessage(reason: string, locale: Locale): string {
  const value = reason.trim()
  if (!value) return ''

  const messages: Array<[string, string, string]> = [
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
    ['replan_waiting', '等待通路，自动继续', 'Waiting for a clear route; navigation will continue automatically'],
    ['replan_budget_consumed', '自动重试机会已用完。', 'Automatic retry attempts have been exhausted.'],
    ['replan_failed_budget_consumed', '重规划失败，自动重试机会已用完。', 'Replanning failed; automatic retry attempts have been exhausted.'],
    ['local_no_path', '暂未生成局部路线。', 'No local route is available yet.'],
    ['local_blocked', '局部路线受阻。', 'The local route is blocked.'],
    ['local_near_field_stop', '检测到近场障碍，已请求停车。', 'A near-field obstacle was detected; a stop was requested.'],
    ['execution_stalled', '运动进展不足，请查看当前任务状态。', 'Motion progress is insufficient. Check the current task state.'],
    ['local_recovery_exhausted', '当前路线无法脱困，请重新预览。', 'The current route could not recover. Preview it again.'],
    ['empty_path', '没有生成可执行路线，请换一个目标。', 'No executable route was generated. Choose another goal.'],
    ['localization_not_tracking', '定位尚未进入跟踪状态，运动指令已保持。', 'Localization is not tracking; motion commands are held.'],
    ['collision_at_trajectory_start', '轨迹起点存在碰撞，运动指令未获放行。', 'A collision exists at the trajectory start; motion was not authorized.'],
    ['visual_servo_target_lost', '目标人员已丢失，已请求取消跟随导航。', 'The target person was lost; cancellation of follow navigation was requested.'],
    ['visual_servo_perception_stale', '相机感知已过期，已请求取消视觉导航。', 'Camera perception is stale; cancellation of visual navigation was requested.'],
  ]
  const match = messages.find(([code]) => value === code || value.startsWith(`${code}:`))
  if (match) return locale === 'zh' ? match[1] : match[2]

  // Bare snake-case values are implementation details, not operator copy.
  if (/^[a-z][a-z0-9_:-]*$/i.test(value)) {
    return locale === 'zh' ? '当前导航条件未满足，请检查现场后重新预览。' : 'The navigation conditions are not met. Check the scene and preview again.'
  }
  return value
}
