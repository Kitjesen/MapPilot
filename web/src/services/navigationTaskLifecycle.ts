export type NavigationTaskLifecycleState =
  | 'PLANNING'
  | 'EXECUTING'
  | 'PAUSED'
  | 'RECOVERING'
  | 'SUCCESS'
  | 'FAILED'
  | 'CANCELLED'
  | 'UNKNOWN'

interface NavigationTaskLifecycleInput {
  lifecycle_state_name?: unknown
  state_name?: unknown
  state?: unknown
  terminal?: unknown
}

export interface NavigationTaskDescription {
  state: NavigationTaskLifecycleState
  terminal: boolean
  canPause: boolean
  canResume: boolean
  canCancel: boolean
}

export type NavigationTaskControlAction = 'pause' | 'resume'
export type NavigationTaskControlRequestState = 'requesting' | 'requested'

export interface NavigationTaskControlRequest {
  taskId: string
  action: NavigationTaskControlAction
  state: NavigationTaskControlRequestState
}

const STATE_ALIASES: Record<string, NavigationTaskLifecycleState> = {
  PATH_ACTIVE: 'EXECUTING',
  REACHED: 'SUCCESS',
}

function normalizedState(input: NavigationTaskLifecycleInput): NavigationTaskLifecycleState {
  const named = [input.lifecycle_state_name, input.state_name]
    .find(value => typeof value === 'string' && value.trim())
  if (typeof named === 'string') {
    const candidate = named.trim().toUpperCase()
    if (candidate in STATE_ALIASES) return STATE_ALIASES[candidate]
    if (['PLANNING', 'EXECUTING', 'PAUSED', 'RECOVERING', 'SUCCESS', 'FAILED', 'CANCELLED'].includes(candidate)) {
      return candidate as NavigationTaskLifecycleState
    }
  }
  return 'UNKNOWN'
}

export function describeNavigationTask(input: NavigationTaskLifecycleInput): NavigationTaskDescription {
  const state = normalizedState(input)
  const terminal = input.terminal === true || ['SUCCESS', 'FAILED', 'CANCELLED'].includes(state)
  return {
    state,
    terminal,
    canPause: !terminal && ['EXECUTING', 'RECOVERING'].includes(state),
    canResume: !terminal && state === 'PAUSED',
    canCancel: !terminal,
  }
}

export function isTaskControlAwaitingConfirmation(
  request: NavigationTaskControlRequest,
  currentTaskId: string,
  task: NavigationTaskDescription,
): boolean {
  if (request.taskId !== currentTaskId || task.terminal) return false
  if (request.action === 'pause') return task.state !== 'PAUSED'
  return !['EXECUTING', 'RECOVERING'].includes(task.state)
}
