import type { AgentMessageEvent, AgentTaskStatusResponse } from '../types/index.ts'

export type AgentTaskTone = 'active' | 'success' | 'warning' | 'error'

export interface AgentTaskView {
  label: string
  detail: string
  tone: AgentTaskTone
  terminal: boolean
  successful: boolean
  canCancel: boolean
}

const FAILURE_LABELS: Record<string, string> = {
  TARGET_MISMATCH: '候选目标不符合要求',
  TARGET_UNCONFIRMED: '无法确认目标',
  TARGET_VERIFICATION_TIMEOUT: '目标确认超时',
  TARGET_VERIFICATION_UNAVAILABLE: '视觉确认不可用',
  FOLLOW_UNAVAILABLE: '跟随能力不可用',
  FOLLOW_UNKNOWN: '无法确认跟随任务',
  SEARCH_COVERAGE_EXHAUSTED: '当前地图范围内没有更多可用观察位置。',
  SEARCH_NO_REACHABLE_VIEW: '当前没有可安全到达的观察位置。',
  SEARCH_BUDGET_EXHAUSTED: '本次任务的观察次数已用完，尚未确认目标。',
  SUPERSEDED: '任务已被新指令替换',
}

const AGENT_FAILURES: Record<string, string> = {
  llm_timeout: '模型响应超时，任务没有继续执行。',
  timeout: '任务等待超时，未获得完成证据。',
  llm_error: '模型服务调用失败，任务没有继续执行。',
  llm_unavailable: '模型服务尚未就绪，任务没有开始执行。',
  agent_error: '任务执行器发生内部错误，任务未完成。',
  step_limit: '任务达到允许的最大决策步数，仍未完成。',
}

function failureDetail(state: string, reason: string): string {
  if (reason === 'localization_not_tracking') return '定位尚未进入跟踪状态，任务没有继续运动。'
  if (reason === 'collision_at_trajectory_start') return '轨迹起点存在碰撞，运动未获放行。'
  if (reason === 'not_current') return '该任务已不是当前任务。'
  return FAILURE_LABELS[state] ?? (reason ? '任务返回了未识别的失败原因，请查看详情。' : '任务未能完成。')
}

function activeDetail(reason: string, fallback: string): string {
  if (!reason || ['accepted', 'path_active', 'running'].includes(reason)) return fallback
  if (reason === 'localization_not_tracking') return '定位尚未进入跟踪状态，运动指令已保持。'
  if (reason === 'collision_at_trajectory_start') return '轨迹起点存在碰撞，运动指令未获放行。'
  if (reason === 'local_blocked') return '局部路线受阻，任务仍在等待后续处理。'
  return '任务仍在进行，等待新的执行反馈。'
}

export function presentAgentTask(status: AgentTaskStatusResponse): AgentTaskView {
  if (!status.found) {
    return {
      label: '无法确认任务状态',
      detail: status.reason === 'not_current'
        ? '该任务可能已被新指令替换。'
        : '与任务服务的状态尚未对齐，请稍后重试。',
      tone: 'warning', terminal: false, successful: false, canCancel: false,
    }
  }

  const agentState = String(status.state ?? '').toLowerCase()
  const navigation = status.navigation
  const navState = String(navigation?.state ?? '').toUpperCase()
  const agentReason = String(status.failure_reason ?? status.reason ?? '')
  const navigationReason = String(navigation?.reason ?? '')

  if (agentState === 'done') {
    return {
      label: '任务已完成', detail: '执行结果已经由任务链确认。',
      tone: 'success', terminal: true, successful: true, canCancel: false,
    }
  }
  if (agentState === 'cancelled') {
    return {
      label: '任务已取消',
      detail: status.motion_attempted !== true
        ? '任务已取消，未派发运动。'
        : status.stop_confirmed === true
          ? '任务已取消，任务停止已确认。'
          : '任务执行已取消，正在等待运动任务结束。',
      tone: 'warning', terminal: true, successful: false, canCancel: false,
    }
  }
  if (agentState === 'running' && navigation?.terminal === true && navigation.success === true) {
    return {
      label: '执行结果已确认，正在结束任务',
      detail: (navigation.confirmations ?? 0) >= 2
        ? '目标已通过视觉核验，正在等待 Agent 完成本次任务。'
        : '领域执行已经确认成功，正在等待 Agent 完成本次任务。',
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }
  const domainFailure = status.motion_attempted === true
    && navigation?.terminal === true
    && navigation.success !== true
  if (agentState === 'failed' || domainFailure) {
    const reason = agentState === 'failed' && agentReason ? agentReason : navigationReason
    return {
      label: '任务未完成', detail: AGENT_FAILURES[reason] ?? failureDetail(navState, reason),
      tone: 'error', terminal: true, successful: false, canCancel: false,
    }
  }

  if (navState === 'SEARCHING') {
    const moves = navigation?.search_moves ?? 0
    return {
      label: navigation?.pending_query ? '正在分析搜索位置' : '正在搜索目标',
      detail: moves > 0 ? `正在执行第 ${moves} 次观察任务。` : '正在从地图中选择下一处观察位置。',
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }
  if (navState === 'CHECKING_OBSERVATION_PATH') {
    return {
      label: '正在规划观察路线', detail: '正在检查可到达的观察位置。',
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }
  if (navState === 'VERIFYING_TARGET' || navState === 'REOBSERVING_TARGET') {
    const confirmations = navigation?.confirmations ?? 0
    return {
      label: navState === 'REOBSERVING_TARGET' ? '正在重新观察目标' : '正在确认目标',
      detail: confirmations > 0 ? `已获得 ${confirmations} 次视觉确认，仍需继续核验。` : '正在使用新的相机画面核验目标。',
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }
  if (navigation?.follow_task_id) {
    const phase = String(navigation.phase ?? '').toLowerCase()
    const followView = {
      selecting: ['正在选择跟随目标', '正在从当前观测中选择符合指令的人员。'],
      following: ['正在持续跟随', '跟随会持续进行，可随时取消。'],
      reacquiring: ['正在重新识别跟随目标', '目标离开视野后，正在核验重新出现的人员身份。'],
      lost: ['跟随目标暂时丢失', '系统仍在尝试重新识别目标；当前没有完成证据。'],
    }[phase]
    if (followView) {
      return {
        label: followView[0], detail: activeDetail(navigationReason, followView[1]),
        tone: phase === 'lost' ? 'warning' : 'active',
        terminal: false, successful: false, canCancel: true,
      }
    }
  }
  if (navState === 'FOLLOW' || navState.includes('FOLLOW') || navState === 'REACQUIRING') {
    return {
      label: navState === 'REACQUIRING' ? '正在重新识别跟随目标' : '正在持续跟随',
      detail: activeDetail(navigationReason, '跟随会持续进行，可随时取消。'),
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }
  if (['EXECUTING', 'ACTIVE', 'NAVIGATING', 'APPROACHING'].includes(navState)) {
    return {
      label: '正在前往目标',
      detail: activeDetail(navigationReason, '正在执行路线，等待到达反馈。'),
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }
  if (['PLANNING', 'QUEUED', 'ACCEPTED', 'PENDING'].includes(navState)) {
    return {
      label: '正在规划路线', detail: '任务已受理，正在等待原生导航执行。',
      tone: 'active', terminal: false, successful: false, canCancel: true,
    }
  }

  return {
    label: '正在理解指令', detail: '任务已受理，正在选择需要调用的能力。',
    tone: 'active', terminal: false, successful: false, canCancel: true,
  }
}

export function shouldPollAgentTask(status: AgentTaskStatusResponse): boolean {
  if (!status.found) return false
  const state = String(status.state ?? '').toLowerCase()
  if (state === 'done') return false
  if ((state === 'cancelled' || state === 'failed') && status.motion_attempted === true) {
    return status.stop_confirmed !== true
  }
  return state !== 'cancelled' && state !== 'failed'
}

export function shouldDisplayAgentMessage(
  message: AgentMessageEvent,
  activeRunId: string,
): boolean {
  if (!activeRunId) return true
  if (message.run_id !== activeRunId || message.role !== 'assistant') return false
  return ['agent_done', 'agent_error', 'agent_say'].includes(message.phase ?? '')
}

export function reconcileCancelRequested(
  requested: boolean,
  status: AgentTaskStatusResponse,
): boolean {
  if (!requested || !status.found) return false
  const state = String(status.state ?? '').toLowerCase()
  if (state !== 'cancelled' && state !== 'failed') return true
  if (status.motion_attempted !== true) return false
  return status.stop_confirmed !== true
}

export function isCurrentAgentResponse(
  current: { runId: string; generation: number },
  response: Pick<AgentTaskStatusResponse, 'run_id'>,
  responseGeneration: number,
): boolean {
  return Boolean(current.runId)
    && response.run_id === current.runId
    && responseGeneration === current.generation
}
