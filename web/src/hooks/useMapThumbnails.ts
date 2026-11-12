import { useCallback, useEffect, useRef, useState } from 'react'
import type { MapLibraryEntry } from '../services/mapSavePresentation.ts'
import { drawMapThumbnail, fetchMapThumbnail, type MapThumbnailData } from '../services/mapThumbnail.ts'

export type Thumbnail = { state: 'loading' } | { state: 'error'; message: string }
  | { state: 'ready'; image: string; data: MapThumbnailData }

export function useMapThumbnails() {
  const [thumbnails, setThumbnails] = useState<Record<string, Thumbnail>>({})
  const requests = useRef(new Map<string, Promise<void>>())
  const queue = useRef(Promise.resolve())
  const controller = useRef<AbortController | null>(null)
  useEffect(() => {
    const current = new AbortController()
    const pending = requests.current
    controller.current = current
    return () => { current.abort(); pending.clear() }
  }, [])

  const load = useCallback((entry: MapLibraryEntry) => {
    if (!entry.map?.has_pcd) return
    const key = `${entry.name}:${entry.map.content_epoch ?? ''}`
    if (requests.current.has(key)) return
    const signal = controller.current!.signal
    setThumbnails(current => ({ ...current, [key]: { state: 'loading' } }))
    // Serialize native PCD reads; all tiles and the large preview share the result.
    const request = queue.current.then(async () => {
      if (signal.aborted) return
      try {
        const data = await fetchMapThumbnail(entry.name, entry.map?.content_epoch, signal)
        if (data.points.length === 0) throw new Error('地图暂无可显示的点云')
        const image = drawMapThumbnail(data.points)
        if (!signal.aborted) setThumbnails(current => ({ ...current, [key]: { state: 'ready', image, data } }))
      } catch (cause) {
        if (!signal.aborted) {
          requests.current.delete(key)
          setThumbnails(current => ({ ...current, [key]: { state: 'error',
            message: cause instanceof Error ? cause.message : String(cause) } }))
        }
      }
    })
    requests.current.set(key, request)
    queue.current = request
  }, [])
  return { thumbnails, load }
}
