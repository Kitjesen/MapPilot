import { useEffect, useState } from 'react'
import { fetchMapPreview } from '../services/mapPreview.ts'
import type { PcdCloud } from '../services/pcdPreview.ts'

type LoadedCloud = { cloud: PcdCloud; frameId: string }
type State = { status: 'loading' | 'idle' } | { status: 'error'; message: string }
  | ({ status: 'ready' } & LoadedCloud)

export function useSavedCloud(name: string | undefined, epoch: number | null | undefined, enabled: boolean): State {
  const [cache, setCache] = useState(() => new Map<string, LoadedCloud>())
  const [result, setResult] = useState<{ key: string; state: State }>()
  const key = `${name}:${epoch ?? ''}`
  useEffect(() => {
    if (!enabled || !name || cache.has(key)) return
    const abort = new AbortController()
    let worker: Worker | undefined
    const update = (state: State) => {
      if (!abort.signal.aborted) setResult({ key, state })
    }
    // Hovering across rows should not download a full cloud for every crossed row.
    const timer = setTimeout(() => {
      update({ status: 'loading' })
      void fetchMapPreview(name, epoch, abort.signal).then(preview => {
        if (abort.signal.aborted) return
        worker = new Worker(new URL('../workers/pcd.worker.ts', import.meta.url), { type: 'module' })
        worker.onmessage = (event: MessageEvent<{ cloud?: PcdCloud; error?: string }>) => {
          if (abort.signal.aborted) return
          const { cloud, error } = event.data
          if (cloud) {
            const loaded = { cloud, frameId: preview.frameId }
            // Two full clouds cover back-and-forth selection without retaining the whole library.
            setCache(current => {
              const next = new Map(current)
              if (next.size === 2) next.delete(next.keys().next().value!)
              next.set(key, loaded)
              return next
            })
            update({ status: 'ready', ...loaded })
          } else update({ status: 'error', message: error || '点云解析失败' })
          worker?.terminate()
        }
        worker.onerror = () => { update({ status: 'error', message: '点云解析失败' }); worker?.terminate() }
        worker.postMessage(preview.buffer, [preview.buffer])
      }).catch(error => update({ status: 'error', message: error instanceof Error ? error.message : String(error) }))
    }, 160)
    return () => { clearTimeout(timer); abort.abort(); worker?.terminate() }
  }, [name, epoch, enabled, key, cache])
  if (!enabled || !name) return { status: 'idle' }
  const loaded = cache.get(key)
  if (loaded) return { status: 'ready', ...loaded }
  return result?.key === key ? result.state : { status: 'loading' }
}
