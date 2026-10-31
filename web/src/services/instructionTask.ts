import type { AgentTaskStatusResponse } from '../types/index.ts'
import { presentAgentTask, shouldPollAgentTask, type AgentTaskView } from './agentTask.ts'

export interface InstructionTask {
  runId: string
  instruction: string
  executor: string
  generation: number
  status: AgentTaskStatusResponse
  statusSource: 'ack' | 'poll'
  cancelRequested: boolean
  cancelError: string
  pollError: string
}

export interface InstructionReceipt {
  operation: 'start' | 'query' | 'keep' | 'cancel'
  instruction: string
  executor: string
  message: string
}

export function instructionRejectionMessage(reason: string | null | undefined): string | undefined {
  if (reason === 'task_active') return '请先结束当前任务，并等待停止确认；新指令已保留在输入框。'
  if (reason === 'motion_start_blocked') return '当前状态不允许启动新运动任务；可继续查询或结束当前任务（motion_start_blocked）。'
  if (reason === 'unsupported_control') return '当前任务不支持这项控制（unsupported_control）；可查询进展或结束任务。'
  return undefined
}

export function instructionTaskNeedsPolling(task: InstructionTask): boolean {
  return task.statusSource === 'ack'
    || (!task.status.found && task.status.reason !== 'not_current')
    || shouldPollAgentTask(task.status)
}

export function instructionTaskCanDismiss(task: InstructionTask): boolean {
  return task.statusSource === 'poll'
    && ((!task.status.found && task.status.reason === 'not_current')
      || (presentAgentTask(task.status).terminal
        && !shouldPollAgentTask(task.status)
        && !task.cancelRequested))
}

export function presentInstructionTask(task: InstructionTask): AgentTaskView {
  const view = presentAgentTask(task.status)
  if (task.statusSource === 'poll' && !task.status.found && task.status.reason === 'not_current') {
    return { ...view, label: '该指令已不是当前任务',
      detail: '任务服务已不再持有此运行。旧任务的停止状态未获得确认。', canCancel: false }
  }
  if (task.pollError) {
    return { ...view, label: '任务状态暂时无法刷新', detail: '等待任务反馈恢复。', tone: 'warning' }
  }
  if (task.statusSource === 'ack') {
    return { ...view, label: task.cancelRequested ? '正在结束任务' : '指令已受理',
      detail: task.cancelRequested ? '正在等待任务结束确认。' : '正在等待任务执行反馈。',
      terminal: false, successful: false, canCancel: !task.cancelRequested }
  }
  if (task.cancelRequested || (view.terminal && instructionTaskNeedsPolling(task))) {
    return { ...view, label: '正在等待任务停止确认',
      detail: task.status.motion_attempted === true
        ? '运动任务停止尚未确认，正在等待执行反馈。'
        : '正在等待任务结束确认；当前尚未派发运动。',
      terminal: false, successful: false, canCancel: false, tone: 'warning' }
  }
  return view
}
