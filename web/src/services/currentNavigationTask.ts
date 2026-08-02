import type { ControlCommandResponse } from '../types/index.ts'

export const CURRENT_NAVIGATION_TASK_STORAGE_KEY = 'lingtu.current-navigation-task'

export interface CurrentNavigationTaskStorage {
  getItem(key: string): string | null
  setItem(key: string, value: string): void
  removeItem(key: string): void
}

export interface CurrentNavigationTaskIdentity {
  schema_version: 'lingtu.web.current_navigation_task.v1'
  task_id: string
  request_id: string
  submitted_at: number
}

type NavigationTaskCommandResponse = ControlCommandResponse & {
  task_id?: unknown
  request_id?: unknown
  command: ControlCommandResponse['command'] & {
    task_id?: unknown
  }
}

export interface AuthoritativeNavigationTaskIdentity {
  task_id?: unknown
  request_id?: unknown
}

function cleanId(value: unknown): string {
  return typeof value === 'string' ? value.trim() : ''
}
function browserStorage(): CurrentNavigationTaskStorage | null {
  if (typeof window === 'undefined') return null
  try {
    return window.localStorage
  } catch {
    return null
  }
}

function readStoredIdentity(
  storage: CurrentNavigationTaskStorage | null,
): CurrentNavigationTaskIdentity | null {
  if (!storage) return null
  try {
    const raw = storage.getItem(CURRENT_NAVIGATION_TASK_STORAGE_KEY)
    if (!raw) return null
    const parsed = JSON.parse(raw) as Partial<CurrentNavigationTaskIdentity>
    const taskId = cleanId(parsed.task_id)
    const requestId = cleanId(parsed.request_id)
    const submittedAt = Number(parsed.submitted_at)
    if (
      parsed.schema_version !== 'lingtu.web.current_navigation_task.v1'
      || !taskId
      || !Number.isFinite(submittedAt)
      || submittedAt < 0
    ) {
      storage.removeItem(CURRENT_NAVIGATION_TASK_STORAGE_KEY)
      return null
    }
    return {
      schema_version: 'lingtu.web.current_navigation_task.v1',
      task_id: taskId,
      request_id: requestId,
      submitted_at: submittedAt,
    }
  } catch {
    try {
      storage.removeItem(CURRENT_NAVIGATION_TASK_STORAGE_KEY)
    } catch {
      // The in-memory identity remains usable when browser storage is unavailable.
    }
    return null
  }
}

export class CurrentNavigationTaskStore {
  private readonly storage: CurrentNavigationTaskStorage | null
  private readonly now: () => number
  private readonly listeners = new Set<() => void>()
  private identity: CurrentNavigationTaskIdentity | null

  constructor(
    storage: CurrentNavigationTaskStorage | null = browserStorage(),
    now: () => number = () => Date.now() / 1000,
  ) {
    this.storage = storage
    this.now = now
    this.identity = readStoredIdentity(storage)
  }

  getSnapshot = (): CurrentNavigationTaskIdentity | null => {
    return this.identity
  }

  subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener)
    return () => {
      this.listeners.delete(listener)
    }
  }

  clear = (): void => {
    if (this.identity === null) return
    this.identity = null
    try {
      this.storage?.removeItem(CURRENT_NAVIGATION_TASK_STORAGE_KEY)
    } catch {
      // Dismissal still applies in memory when browser storage is unavailable.
    }
    this.notify()
  }

  adoptAuthoritative(
    active: AuthoritativeNavigationTaskIdentity,
  ): CurrentNavigationTaskIdentity | null {
    const taskId = cleanId(active.task_id)
    if (!taskId) return this.identity
    if (this.identity?.task_id === taskId) return this.identity

    const identity: CurrentNavigationTaskIdentity = {
      schema_version: 'lingtu.web.current_navigation_task.v1',
      task_id: taskId,
      request_id: cleanId(active.request_id),
      submitted_at: this.now(),
    }
    this.identity = identity
    try {
      this.storage?.setItem(
        CURRENT_NAVIGATION_TASK_STORAGE_KEY,
        JSON.stringify(identity),
      )
    } catch {
      // Native task adoption remains available in memory when persistence is unavailable.
    }
    this.notify()
    return identity
  }

  trackAccepted(
    response: NavigationTaskCommandResponse,
  ): CurrentNavigationTaskIdentity | null {
    if (!response.ok || response.command?.accepted !== true) return null

    const taskId = cleanId(response.task_id) || cleanId(response.command.task_id)
    if (!taskId) {
      throw new Error('navigation_task_identity_missing')
    }

    const identity: CurrentNavigationTaskIdentity = {
      schema_version: 'lingtu.web.current_navigation_task.v1',
      task_id: taskId,
      request_id: cleanId(response.request_id) || cleanId(response.command.request_id),
      submitted_at: this.now(),
    }
    this.identity = identity
    try {
      this.storage?.setItem(CURRENT_NAVIGATION_TASK_STORAGE_KEY, JSON.stringify(identity))
    } catch {
      // Tracking remains available in memory when persistence is unavailable.
    }
    this.notify()
    return identity
  }

  private notify(): void {
    for (const listener of this.listeners) listener()
  }
}

export const currentNavigationTaskStore = new CurrentNavigationTaskStore()
