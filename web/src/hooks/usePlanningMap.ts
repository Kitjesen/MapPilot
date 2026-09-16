import { useEffect, useMemo, useState } from 'react'
import { fetchPlanningMap } from '../services/api.ts'
import { currentPlanningMap, type PlanningMap } from '../services/planningMap.ts'

export function usePlanningMap(connected: boolean, mapId: string | null, sessionId: string | null | undefined, nowS: number) {
  const [snapshot, setSnapshot] = useState<{ map: PlanningMap; receivedAt: number } | null>(null)
  useEffect(() => {
    setSnapshot(null)
    if (!connected || !mapId || !sessionId) return
    let cancelled = false
    let timer: ReturnType<typeof setTimeout> | undefined
    let controller: AbortController | undefined
    async function poll() {
      controller = new AbortController()
      const timeout = setTimeout(() => controller?.abort(), 4000)
      try {
        const map = await fetchPlanningMap(controller.signal)
        if (!cancelled) setSnapshot(previous => ({
          map: previous?.map.available && map.available && previous.map.stamp_s === map.stamp_s
            && previous.map.map_id === map.map_id && previous.map.map_content_epoch === map.map_content_epoch
            && previous.map.product_session_id === map.product_session_id ? previous.map : map,
          receivedAt: Date.now() / 1000,
        }))
      } catch {
        // Keep the last map until its freshness deadline. A failed poll must
        // not alternate the scene between the map and the raw point cloud.
      } finally {
        clearTimeout(timeout)
        if (!cancelled) timer = setTimeout(poll, 5000)
      }
    }
    void poll()
    return () => { cancelled = true; clearTimeout(timer); controller?.abort() }
  }, [connected, mapId, sessionId])
  const fresh = !!snapshot && nowS - snapshot.receivedAt < 10
  const map = useMemo(() => currentPlanningMap(snapshot?.map ?? null, connected && fresh, mapId, sessionId),
    [snapshot?.map, connected, fresh, mapId, sessionId])
  return { map, reason: connected && fresh ? snapshot?.map.reason : 'navigation_status_stale' }
}
