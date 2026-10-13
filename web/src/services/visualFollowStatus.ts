import type { Locale } from '../i18n'
import type { NavigationStatusResponse, VisualServoStatus } from '../types/index.ts'
import { operatorReasonMessage, presentNavigationStatus } from './navigationStatus.ts'

export interface VisualFollowPresentation {
  visual: {
    label: string
    message: string
  }
  navigation: {
    label: string
    message: string
  } | null
  motion: {
    permission: string
    observation: string
    stopConfirmation: string
    stopConfirmed: boolean
    reasonMessage: string
  }
}

const VISUAL_REASON_MESSAGES: Record<string, { en: string; zh: string }> = {
  waiting_perception: { en: 'Waiting for camera perception', zh: '等待相机感知' },
  visual_servo_perception_stale: { en: 'Camera perception is stale', zh: '相机感知已过期' },
  target_selection_running: { en: 'Selecting the target person', zh: '正在选择目标人员' },
  target_selection_unavailable: { en: 'Person selection is unavailable', zh: '人员选择不可用' },
  target_not_visible: { en: 'The target is not visible', zh: '当前未看到目标' },
  waiting_selection_image: { en: 'Waiting for an image to select from', zh: '等待可用于选人的图像' },
  selecting_target: { en: 'Selecting the target person', zh: '正在选择目标人员' },
  identity_verification_running: { en: 'Verifying the person identity', zh: '正在核验人员身份' },
  identity_verification_unavailable: { en: 'Identity verification is unavailable', zh: '人员身份核验不可用' },
  confirming: { en: 'Confirming the person across consecutive frames', zh: '正在通过连续画面确认人员' },
  awaiting_identity_verification: { en: 'Waiting for identity verification', zh: '等待人员身份核验' },
  identity_rejected: { en: 'The candidate did not match the target person', zh: '候选人员与目标不符' },
  identity_unknown: { en: 'The candidate identity could not be confirmed', zh: '暂时无法确认候选人员身份' },
  verification_backoff: { en: 'Identity verification will retry shortly', zh: '身份核验稍后重试' },
  low_similarity: { en: 'No sufficiently similar person was found', zh: '未找到足够相似的人员' },
  ambiguous: { en: 'Several people look like the target', zh: '存在多个相似人员，暂无法确认' },
  missing_image: { en: 'The candidate image is unavailable', zh: '候选人员图像不可用' },
  encode_failed: { en: 'The candidate image could not be processed', zh: '候选人员图像处理失败' },
  no_candidate: { en: 'No matching person candidate was found', zh: '未找到匹配的候选人员' },
  target_absent: { en: 'The target person is absent from the current view', zh: '当前画面中没有目标人员' },
  position_rejected: { en: 'The observed position was rejected', zh: '人员位置观测未通过检查' },
  stale_observation: { en: 'The observed position is stale', zh: '人员位置观测已过期' },
}

const VISUAL_STATE_LABELS: Record<string, { en: string; zh: string }> = {
  idle: { en: 'Ready', zh: '就绪' },
  searching: { en: 'Searching for target', zh: '正在寻找目标' },
  approaching: { en: 'Target located', zh: '已找到目标' },
  selecting: { en: 'Selecting target', zh: '正在选择目标' },
  reacquiring: { en: 'Reacquiring person', zh: '正在重新识别人员' },
  following: { en: 'Target locked', zh: '目标已锁定' },
  lost: { en: 'Target lost', zh: '目标丢失' },
  stopping: { en: 'Stopping follow task', zh: '正在停止跟随任务' },
  failed: { en: 'Visual task failed', zh: '视觉任务失败' },
}

const NAVIGATION_STATE_LABELS: Record<string, { en: string; zh: string }> = {
  accepted: { en: 'Follow goal accepted', zh: '跟随目标已接受' },
  planning: { en: 'Follow route planning', zh: '正在规划跟随路线' },
  executing: { en: 'Follow navigation active', zh: '跟随导航任务执行中' },
  path_active: { en: 'Follow navigation active', zh: '跟随导航任务执行中' },
  recovering: { en: 'Follow navigation recovering', zh: '跟随导航正在恢复' },
  cancel_requested: { en: 'Follow goal cancellation requested', zh: '已请求取消跟随目标' },
  cancelled: { en: 'Follow goal cancelled', zh: '跟随目标已取消' },
  rejected: { en: 'Follow goal rejected', zh: '跟随目标被拒绝' },
  failed: { en: 'Follow navigation failed', zh: '跟随导航失败' },
  reached: { en: 'Follow goal reached', zh: '已到达跟随目标' },
}

const ROUTINE_NAVIGATION_REASONS = new Set([
  'accepted',
  'path_active',
  'cancel_request',
  'cancel_requested',
  'visual_servo_stop',
  'visual_servo_target_changed',
  'motion_clear',
  'nonzero_output_published',
  'stop_confirmed',
])

function actionableReason(reason: string): string {
  const value = reason.trim()
  return ROUTINE_NAVIGATION_REASONS.has(value) ? '' : value
}

function localized(
  values: Record<string, { en: string; zh: string }>,
  key: string,
  locale: Locale,
  fallback: { en: string; zh: string },
): string {
  return (values[key] ?? fallback)[locale]
}

/** Present visual tracking and native robot motion as separate facts. */
export function presentVisualFollowStatus(
  visual: VisualServoStatus,
  navigation: NavigationStatusResponse | null | undefined,
  locale: Locale,
): VisualFollowPresentation {
  const visualReason = visual.reason?.trim() ?? ''
  const visualLabel = localized(
    VISUAL_STATE_LABELS,
    visual.state.trim().toLowerCase(),
    locale,
    { en: 'Visual task active', zh: '视觉任务进行中' },
  )
  const reasonCopy = VISUAL_REASON_MESSAGES[visualReason]?.[locale] ?? ''

  const navigationState = visual.navigation_state.trim().toLowerCase()
  const taskMatches = Boolean(
    visual.navigation_task_id
      && navigation?.task.task_id
      && visual.navigation_task_id === navigation.task.task_id,
  )
  const matchingNativeReason = taskMatches
    ? navigation?.motion.permission !== 'CLEAR' && navigation?.motion.reason
      ? navigation.motion.reason
      : navigation?.goal_admission.state === 'BLOCKED' && navigation?.goal_admission.reason
        ? navigation.goal_admission.reason
        : navigation?.task.reason || ''
    : ''
  const navigationReason = actionableReason(matchingNativeReason)
    || actionableReason(visual.navigation_reason)
  const showNavigation = !['', 'idle'].includes(navigationState) || Boolean(navigationReason)
  const navigationLabel = localized(
    NAVIGATION_STATE_LABELS,
    navigationState,
    locale,
    { en: 'Follow navigation status', zh: '跟随导航状态' },
  )

  const native = presentNavigationStatus(navigation, locale)
  const motionReason = actionableReason(navigation?.motion.reason ?? '')
  return {
    visual: {
      label: visualLabel,
      message: reasonCopy,
    },
    navigation: showNavigation
      ? {
          label: navigationLabel,
          message: navigationReason ? operatorReasonMessage(navigationReason, locale) : '',
        }
      : null,
    motion: {
      permission: native.motion.permission.label,
      observation: native.motion.observation.label,
      stopConfirmation: native.motion.stopConfirmation.label,
      stopConfirmed: native.motion.stopConfirmation.state === 'CONFIRMED',
      reasonMessage: motionReason
        ? operatorReasonMessage(motionReason, locale)
        : '',
    },
  }
}
