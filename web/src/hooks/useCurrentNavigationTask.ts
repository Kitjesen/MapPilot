import { useEffect, useState, useSyncExternalStore } from 'react'
import { fetchNavigationTaskStatus } from '../services/api'
import {
  currentNavigationTaskStore,
  type CurrentNavigationTaskIdentity,
} from '../services/currentNavigationTask'
import type { NavigationTaskStatusQueryResponse } from '../types'

export const CURRENT_NAVIGATION_TASK_POLL_INTERVAL_MS = 2_000

export type NavigationTaskQuerySnapshot = NavigationTaskStatusQueryResponse

interface PollingState {
  identityKey: string | null
  snapshot: NavigationTaskQuerySnapshot | null
  loading: boolean
  stale: boolean
  error: string | null
}

export interface CurrentNavigationTaskState {
  identity: CurrentNavigationTaskIdentity | null
  snapshot: NavigationTaskQuerySnapshot | null
  loading: boolean
  stale: boolean
  error: string | null
  dismiss: () => void
}

function taskIdentityKey(identity: CurrentNavigationTaskIdentity | null): string | null {
  if (!identity) return null
  return `${identity.task_id}\u0000${identity.request_id}\u0000${identity.submitted_at}`
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : String(error)
}

function isTerminal(snapshot: NavigationTaskQuerySnapshot): boolean {
  if (!snapshot.found || !snapshot.status) return false
  if (snapshot.status.terminal === true) return true

  const stateName = String(snapshot.status.state_name ?? '').toUpperCase()
  return ['FAILED', 'REACHED', 'CANCELLED', 'SUCCESS'].includes(stateName)
}

export function useCurrentNavigationTask(): CurrentNavigationTaskState {
  const identity = useSyncExternalStore(
    currentNavigationTaskStore.subscribe,
    currentNavigationTaskStore.getSnapshot,
    currentNavigationTaskStore.getSnapshot,
  )
  const identityKey = taskIdentityKey(identity)
  const [polling, setPolling] = useState<PollingState>({
    identityKey: null,
    snapshot: null,
    loading: false,
    stale: false,
    error: null,
  })

  useEffect(() => {
    let cancelled = false
    let timer: number | undefined

    if (!identity || !identityKey) {
      setPolling({
        identityKey: null,
        snapshot: null,
        loading: false,
        stale: false,
        error: null,
      })
      return undefined
    }

    setPolling({
      identityKey,
      snapshot: null,
      loading: true,
      stale: false,
      error: null,
    })

    const scheduleNext = (load: () => Promise<void>) => {
      timer = window.setTimeout(() => void load(), CURRENT_NAVIGATION_TASK_POLL_INTERVAL_MS)
    }

    const load = async (): Promise<void> => {
      try {
        const next = await fetchNavigationTaskStatus(identity.task_id)
        if (cancelled) return

        setPolling({
          identityKey,
          snapshot: next,
          loading: false,
          stale: false,
          error: null,
        })
        if (!isTerminal(next)) scheduleNext(load)
      } catch (error: unknown) {
        if (cancelled) return

        setPolling(previous => previous.identityKey === identityKey
          ? {
              ...previous,
              loading: false,
              stale: true,
              error: errorMessage(error),
            }
          : {
              identityKey,
              snapshot: null,
              loading: false,
              stale: true,
              error: errorMessage(error),
            })
        scheduleNext(load)
      }
    }

    void load()
    return () => {
      cancelled = true
      if (timer !== undefined) window.clearTimeout(timer)
    }
  }, [identity, identityKey])

  const isCurrent = identityKey !== null && polling.identityKey === identityKey
  return {
    identity,
    snapshot: isCurrent ? polling.snapshot : null,
    loading: isCurrent ? polling.loading : identity !== null,
    stale: isCurrent ? polling.stale : false,
    error: isCurrent ? polling.error : null,
    dismiss: currentNavigationTaskStore.clear,
  }
}
