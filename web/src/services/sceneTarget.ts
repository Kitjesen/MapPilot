import type { AgentTaskStatusResponse, SSEState, VisualServoStatus } from '../types/index.ts'
import { estimateSceneTime, freshSource } from './sceneTelemetry.ts'

export type SceneTargetState = 'selecting' | 'following' | 'reacquiring' | 'lost' | 'unavailable'

export interface SceneTargetPresentation {
  state: SceneTargetState
  label: string
  targetLabel: string | null
  detail: string
  cameraVisible: boolean
  // The current transport has neither a camera-frame-aligned bbox nor a
  // map/frame epoch for the person position. Neither authorizes a live marker.
  marker: null
}

export interface SceneTargetInput {
  task: AgentTaskStatusResponse | null | undefined
  sseState: Pick<SSEState, 'visualServoStatus' | 'session' | 'connected' | 'stateSnapshot' | 'stateSnapshotReceivedAt'>
  /** Browser wall-clock seconds; translated into the Gateway clock below. */
  nowS: number
  /** True only after a fresh camera frame has actually decoded. */
  cameraLive?: boolean
}

const LABELS: Record<SceneTargetState, string> = {
  selecting: '正在选择目标',
  following: '正在跟随目标',
  reacquiring: '正在重新识别目标',
  lost: '目标暂时丢失',
  unavailable: '目标状态待确认',
}

function presentation(state: SceneTargetState, cameraLive: boolean, detail: string,
  targetLabel: string | null = null): SceneTargetPresentation {
  return { state, label: LABELS[state], targetLabel, detail, cameraVisible: cameraLive, marker: null }
}

/** Follow evidence belongs to the current Agent task, never a nearby scene object. */
export function deriveSceneTarget({ task, sseState, nowS, cameraLive = false }: SceneTargetInput): SceneTargetPresentation | null {
  const owned = task?.navigation
  if (!task?.found || !owned?.follow_task_id || owned.owned !== true
    || owned.terminal === true || ['done', 'cancelled', 'failed'].includes(task.state ?? '')) return null

  const visual = sseState.visualServoStatus
  const sessionId = sseState.session?.product_session_id
  const taskSessionId = owned.product_session_id
  const unavailable = (detail: string) => presentation('unavailable', cameraLive, detail)
  if (!visual || visual.follow_task_id !== owned.follow_task_id
    || (taskSessionId && visual.product_session_id !== taskSessionId)
    || (sessionId && (taskSessionId !== sessionId || visual.product_session_id !== sessionId))) {
    return unavailable('等待与当前任务和运行会话一致的目标反馈。')
  }

  // VisualServo publishes ts in its status payload; it is not a camera timestamp.
  const source = visual as VisualServoStatus & { ts?: number }
  const gatewayNow = estimateSceneTime(nowS, sseState.stateSnapshot?.ts, sseState.stateSnapshotReceivedAt)
  if (!sseState.connected || !freshSource(source.ts, gatewayNow)) {
    return unavailable(sseState.connected ? '目标反馈已过期，等待新的观测。' : '连接已断开，等待新的目标反馈。')
  }
  if (visual.mode !== 'follow' || visual.task_terminal === true) return null

  const targetLabel = visual.target?.trim() || null
  const state = visual.state.trim().toLowerCase()
  if (state === 'selecting') return presentation('selecting', cameraLive, '正在从当前观测中选择符合指令的人员。', targetLabel)
  if (state === 'reacquiring') return presentation('reacquiring', cameraLive, '正在核验重新出现的人员身份。', targetLabel)
  if (state === 'lost' || (state === 'following' && !visual.target_visible)) {
    return presentation('lost', cameraLive, '当前观测未确认目标，正在等待重新识别。', targetLabel)
  }
  if (state === 'following' && visual.target_visible) {
    if (['visual_servo_perception_stale', 'stale_observation', 'waiting_perception'].includes(visual.reason ?? '')) {
      return unavailable('目标观测尚未更新，等待新的确认。')
    }
    return presentation('following', cameraLive, '当前跟随任务已确认目标可见。', targetLabel)
  }
  return unavailable('等待新的目标跟踪反馈。')
}
