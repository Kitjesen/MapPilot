import type { Locale } from '../i18n.ts'
import type { PlanPreviewResponse, OdometryEvent } from '../types/index.ts'
import { formatCommandError, isGatewayApiError } from './api.ts'
import { operatorReasonMessage } from './navigationStatus.ts'

export function navigationPreviewErrorMessage(error: string): string {
  if (error.startsWith('规划失败：')
    && /Failed to fetch|NetworkError|Load failed|local_review_preview_service_unavailable/i.test(error)) {
    return '路线预览服务不可用'
  }
  for (const prefix of ['规划失败：', '无法生成路线：']) {
    if (error.startsWith(prefix)) return prefix + operatorReasonMessage(error.slice(prefix.length), 'zh')
  }
  return error
}

export function navigationGoalErrorMessage(error: unknown, locale: Locale): string {
  if (isGatewayApiError(error)) {
    if (error.errorCode === 'native_command_unconfirmed') {
      return locale === 'zh' ? '未收到导航服务确认，请先查看当前任务，再重新操作。'
        : 'Navigation did not confirm the request. Check the current task before trying again.'
    }
    if (error.errorCode === 'native_command_rejected' && error.detail && typeof error.detail === 'object'
      && 'reason_code' in error.detail && typeof error.detail.reason_code === 'string') {
      const reason = operatorReasonMessage(error.detail.reason_code, locale)
      return locale === 'zh' ? `目标未接受：${reason}` : `Goal not accepted: ${reason}`
    }
  }
  return formatCommandError(error, locale === 'zh' ? '发送目标失败' : 'Goal submission failed')
}

export function navigationPreviewIsCurrent(preview: PlanPreviewResponse | null, pose: Pick<OdometryEvent, 'x' | 'y' | 'z'> | null): boolean {
  const start = preview?.start
  return preview?.feasible === true && preview.frame_id === 'map' && !!start && !!pose
    && Math.hypot(start.x - pose.x, start.y - pose.y) <= 0.25
    && typeof pose.z === 'number' && Math.abs(start.z - pose.z) <= 0.15
}

export function navigationSupportHeightFromRunPlan(plan: unknown): number | null {
  const config = (plan as { host?: { config?: Record<string, unknown> } } | null)?.host?.config
  const value = config?.octoplanner3d_support_height_m
  return typeof value === 'number' && Number.isFinite(value) && value >= 0 ? value : null
}

/** Convert a picked surface once; previews and manual height edits already use body coordinates. */
export function sceneGoalHeight(bodyZ: number | null | undefined, selectedZ?: number,
  mode: 'plane' | 'surface' = 'plane', supportHeight?: number | null): number | null {
  const z = selectedZ ?? bodyZ
  if (typeof z !== 'number' || !Number.isFinite(z)) return null
  if (mode === 'surface') {
    return selectedZ !== undefined && typeof supportHeight === 'number' && Number.isFinite(supportHeight)
      && supportHeight >= 0 ? z + supportHeight : null
  }
  return z
}
