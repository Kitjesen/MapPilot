import { useEffect, useState } from 'react'
import { fetchNoGo, type NoGoMap } from '../services/noGo.ts'

export function useNoGo(name: string | undefined, epoch?: number | null) {
  const key = `${name}:${epoch ?? ''}`
  const [state, setState] = useState<{ key?: string; map?: NoGoMap; error?: string }>({})
  useEffect(() => {
    if (!name) return
    const controller = new AbortController()
    let active = true
    void fetchNoGo(name, controller.signal).then(map => {
      if (active) setState(epoch != null && map.content_epoch !== epoch
        ? { key, error: '地图内容已更新，请刷新地图。' } : { key, map })
    }, cause => { if (active) setState({ key, error: cause instanceof Error ? cause.message : String(cause) }) })
    return () => { active = false; controller.abort() }
  }, [name, epoch, key])
  return name && state.key === key ? state : {}
}
