import type { Locale } from '../i18n.ts'
import type {
  NavigationOperatorControlAuthority,
  NavigationOperatorGoalAdmission,
  NavigationOperatorMotionObservation,
  NavigationOperatorMotionPermission,
  NavigationOperatorSummarySeverity,
  NavigationOperatorStopConfirmation,
  NavigationOperatorTaskState,
  NavigationStatusResponse,
} from '../types/index.ts'

interface LabeledState<T extends string> {
  state: T
  label: string
}

export interface NavigationOperatorPresentation {
  source: 'operator_state' | 'legacy'
  task: {
    state: NavigationOperatorTaskState
    label: string
  }
  goalAdmission: LabeledState<NavigationOperatorGoalAdmission> & {
    blockers: string[]
    advisories: string[]
  }
  control: LabeledState<NavigationOperatorControlAuthority> & {
    resumeRequired: boolean
  }
  motion: {
    permission: LabeledState<NavigationOperatorMotionPermission>
    observation: LabeledState<NavigationOperatorMotionObservation>
    stopConfirmation: LabeledState<NavigationOperatorStopConfirmation>
  }
  summary: {
    severity: NavigationOperatorSummarySeverity
    code: string
    nextAction: string
  }
}

interface PresentationOptions {
  locale: Locale
  legacyTaskState?: unknown
}

const TASK_LABELS: Record<NavigationOperatorTaskState, { en: string; zh: string }> = {
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

const MOTION_PERMISSION_LABELS: Partial<Record<NavigationOperatorMotionPermission, { en: string; zh: string }>> = {
  CLEAR: { en: 'Motion clear', zh: '允许运动' },
  HELD: { en: 'Motion held', zh: '运动保持' },
  ESTOPPED: { en: 'Emergency stop held', zh: '急停保持' },
  UNKNOWN: { en: 'Motion permission unknown', zh: '运动权限未知' },
}

const MOTION_OBSERVATION_LABELS: Partial<Record<NavigationOperatorMotionObservation, { en: string; zh: string }>> = {
  MOVING: { en: 'Moving', zh: '运动中' },
  QUIET: { en: 'Observed quiet', zh: '静止观测' },
  UNKNOWN: { en: 'Motion state unknown', zh: '运动状态未知' },
}

const STOP_CONFIRMATION_LABELS: Partial<Record<NavigationOperatorStopConfirmation, { en: string; zh: string }>> = {
  NOT_REQUESTED: { en: 'Stop not requested', zh: '未请求停稳' },
  PENDING: { en: 'Awaiting stop confirmation', zh: '等待停稳确认' },
  CONFIRMED: { en: 'Stop confirmed', zh: '已确认停稳' },
  FAILED: { en: 'Stop confirmation failed', zh: '停稳确认失败' },
  UNKNOWN: { en: 'Stop confirmation unknown', zh: '停稳状态未知' },
}

const GOAL_ADMISSION_LABELS: Partial<Record<NavigationOperatorGoalAdmission, { en: string; zh: string }>> = {
  ACCEPTING: { en: 'Accepting goals', zh: '可接收目标' },
  BLOCKED: { en: 'Goal blocked', zh: '目标受阻' },
  UNKNOWN: { en: 'Goal admission unknown', zh: '目标准入未知' },
}

const CONTROL_AUTHORITY_LABELS: Partial<Record<NavigationOperatorControlAuthority, { en: string; zh: string }>> = {
  AUTONOMY: { en: 'Autonomy control', zh: '自主控制' },
  OPERATOR: { en: 'Operator control', zh: '操作者控制' },
  NONE: { en: 'No controller', zh: '无控制者' },
  UNKNOWN: { en: 'Control authority unknown', zh: '控制权未知' },
}

const NEXT_ACTION_LABELS: Record<string, { en: string; zh: string }> = {
  inspect_stop_failure: { en: 'Inspect the stop failure', zh: '检查停稳失败原因' },
  wait_for_stop_confirmation: { en: 'Wait for stop confirmation', zh: '等待停稳确认' },
  clear_estop: { en: 'Clear the emergency stop', zh: '解除急停后再继续' },
  check_status_sources: { en: 'Check status sources', zh: '检查状态数据源' },
  resolve_motion_hold: { en: 'Resolve the motion hold', zh: '解除运动保持条件' },
  resolve_goal_blockers: { en: 'Resolve goal blockers', zh: '处理目标准入阻塞' },
  inspect_task_failure: { en: 'Inspect the task failure', zh: '检查任务失败原因' },
  monitor_recovery: { en: 'Monitor recovery', zh: '等待恢复完成' },
  wait_for_plan: { en: 'Wait for a plan', zh: '等待路径规划' },
  resume_or_cancel: { en: 'Resume or cancel the task', zh: '恢复或取消任务' },
  review_advisories: { en: 'Review navigation advisories', zh: '查看导航提示' },
  monitor_progress: { en: 'Monitor task progress', zh: '监控任务进度' },
  choose_goal: { en: 'Choose a navigation goal', zh: '选择导航目标' },
}

function labelState<T extends string>(
  state: T,
  labels: Partial<Record<T, { en: string; zh: string }>>,
  locale: Locale,
): LabeledState<T> {
  const label = labels[state]
  return { state, label: label ? label[locale] : state }
}

function legacyGoalAdmission(
  navigation: NavigationStatusResponse | null | undefined,
): NavigationOperatorGoalAdmission {
  const value = navigation?.readiness?.can_accept_goal ?? navigation?.can_accept_goal
  return typeof value === 'boolean' ? (value ? 'ACCEPTING' : 'BLOCKED') : 'UNKNOWN'
}

function legacyControlAuthority(
  navigation: NavigationStatusResponse | null | undefined,
): NavigationOperatorControlAuthority {
  const control = navigation?.control
  if (!control) return 'UNKNOWN'
  if (control.manual_override === true || control.preempting_autonomy === true) return 'OPERATOR'
  if (control.autonomy_requested === true || control.source_category === 'autonomy') return 'AUTONOMY'
  const source = control.active_cmd_source?.trim().toLowerCase()
  return source === '' || source === 'none' ? 'NONE' : 'UNKNOWN'
}

function legacyMotionPermission(
  navigation: NavigationStatusResponse | null | undefined,
): NavigationOperatorMotionPermission {
  const mode = navigation?.motion?.speed_policy?.mode
  if (mode === 'hold') return 'HELD'
  if (mode === 'normal' || mode === 'cautious' || mode === 'restricted') return 'CLEAR'
  return 'UNKNOWN'
}

function normalizeTaskState(value: unknown, emptyState: NavigationOperatorTaskState): NavigationOperatorTaskState {
  if (typeof value !== 'string' || !value.trim()) return emptyState
  const candidate = value.trim().toUpperCase()
  if (candidate === 'ARRIVED' || candidate === 'REACHED') return 'SUCCESS'
  if (candidate === 'PATH_ACTIVE') return 'EXECUTING'
  return candidate in TASK_LABELS ? candidate as NavigationOperatorTaskState : 'UNKNOWN'
}

export function navigationOperatorTaskLabel(
  state: NavigationOperatorTaskState,
  locale: Locale,
): string {
  return TASK_LABELS[state][locale]
}

export function presentNavigationOperatorStatus(
  navigation: NavigationStatusResponse | null | undefined,
  options: PresentationOptions,
): NavigationOperatorPresentation {
  const operatorState = navigation?.operator_state
  const state = operatorState
    ? normalizeTaskState(operatorState.task?.state, 'UNKNOWN')
    : normalizeTaskState(options.legacyTaskState, 'IDLE')
  const motion = operatorState?.motion
  const goalAdmission = operatorState?.goal_admission
  const control = operatorState?.control
  const summary = operatorState?.summary
  const nextActionLabels = summary ? NEXT_ACTION_LABELS[summary.next_action] : undefined
  const legacyBlockers = [
    ...(navigation?.readiness?.blockers ?? []),
    ...(navigation?.feedback?.blockers ?? []),
  ]

  return {
    source: operatorState ? 'operator_state' : 'legacy',
    task: { state, label: navigationOperatorTaskLabel(state, options.locale) },
    goalAdmission: {
      ...labelState(
        goalAdmission?.state ?? legacyGoalAdmission(navigation),
        GOAL_ADMISSION_LABELS,
        options.locale,
      ),
      blockers: goalAdmission?.blockers ?? legacyBlockers,
      advisories: goalAdmission?.advisories ?? navigation?.readiness?.advisories ?? [],
    },
    control: {
      ...labelState(
        control?.authority ?? legacyControlAuthority(navigation),
        CONTROL_AUTHORITY_LABELS,
        options.locale,
      ),
      resumeRequired: control?.resume_required === true,
    },
    motion: {
      permission: labelState(
        motion?.permission ?? legacyMotionPermission(navigation),
        MOTION_PERMISSION_LABELS,
        options.locale,
      ),
      observation: labelState(motion?.observation ?? 'UNKNOWN', MOTION_OBSERVATION_LABELS, options.locale),
      stopConfirmation: labelState(motion?.stop_confirmation ?? 'UNKNOWN', STOP_CONFIRMATION_LABELS, options.locale),
    },
    summary: {
      severity: summary?.severity ?? 'WARNING',
      code: summary?.code ?? 'STATUS_SOURCE_UNKNOWN',
      nextAction: nextActionLabels
        ? nextActionLabels[options.locale]
        : (options.locale === 'zh' ? '查看导航详情' : 'Review navigation details'),
    },
  }
}
