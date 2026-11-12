import { useEffect, useState } from 'react'
import { fetchSurfaceMap, type SurfaceMap, type SurfaceOptions } from '../services/surfaceMap.ts'

type State = { map?: SurfaceMap; error?: string; loading?: boolean }
export function useSurfaceMap(name: string, epoch: number | null | undefined, options: SurfaceOptions, enabled: boolean): State {
  const [result, setResult] = useState<{ name: string; epoch: typeof epoch; options: SurfaceOptions; state: State }>()
  useEffect(() => {
    if (!enabled) return
    const controller = new AbortController()
    const update = (state: State) => {
      if (!controller.signal.aborted) setResult({ name, epoch, options, state })
    }
    void fetchSurfaceMap(name, options, controller.signal).then(map => {
      update(epoch != null && map.content_epoch !== epoch
        ? { error: '地图已更新，请刷新后查看补面。' } : { map })
    }, cause => update({ error: cause instanceof Error ? cause.message : String(cause) }))
    return () => controller.abort()
  }, [name, epoch, options, enabled])
  if (!enabled) return {}
  return result?.name === name && result.epoch === epoch && result.options === options ? result.state : { loading: true }
}
