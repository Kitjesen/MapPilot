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
