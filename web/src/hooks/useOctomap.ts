import { useEffect, useState } from 'react'
import { fetchMapPreview } from '../services/mapPreview.ts'
import type { Octomap } from '../services/octomap.ts'

type State = { status: 'idle' | 'loading' | 'error'; message?: string }
  | { status: 'ready'; map: Octomap }

export function useOctomap(name: string | undefined, epoch: number | null | undefined, frame: string | undefined, enabled: boolean): State {
  const [result, setResult] = useState<{ name: string; epoch: number | null | undefined; state: State }>()
  useEffect(() => {
    if (!enabled || !name) return
    const abort = new AbortController()
    let worker: Worker | undefined
    const update = (state: State) => {
      if (!abort.signal.aborted) setResult({ name, epoch, state })
    }
    update({ status: 'loading' })
    void fetchMapPreview(name, epoch, abort.signal, 'octomap').then(preview => {
      if (abort.signal.aborted) return
      if (frame && preview.frameId.replace(/^\/+/, '') !== frame.replace(/^\/+/, '')) {
        throw new Error('导航体素与场景坐标系不一致')
      }
      worker = new Worker(new URL('../workers/octomap.worker.ts', import.meta.url), { type: 'module' })
      worker.onmessage = (event: MessageEvent<{ map?: Octomap; error?: string }>) => {
        const { map, error } = event.data
        update(map ? { status: 'ready', map } : { status: 'error', message: error })
        worker?.terminate()
      }
      worker.onerror = () => { update({ status: 'error', message: '导航体素解析失败' }); worker?.terminate() }
      worker.postMessage(preview.buffer, [preview.buffer])
    }).catch(error => update({ status: 'error', message: error instanceof Error ? error.message : String(error) }))
    return () => { abort.abort(); worker?.terminate() }
  }, [name, epoch, frame, enabled])
  if (!enabled) return { status: 'idle' }
  return result && result.name === name && result.epoch === epoch ? result.state : { status: 'loading' }
}
