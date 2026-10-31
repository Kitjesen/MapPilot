import { useEffect, useRef, useState } from 'react'
import { fetchRecordingScene, fetchRecordingSceneInfo } from '../services/api'
import { SCENE_WINDOW_SECONDS, sceneWindowIndex, sceneWindowRange } from '../services/recordingScene'
import type { RecordingScene, RecordingSceneInfoResponse, RecordingSceneQuery } from '../types/recordingScene'

export function useRecordingSceneInfo(sessionId: string) {
  const [result, setResult] = useState<{ id: string; info?: RecordingSceneInfoResponse; error?: string }>({ id: '' })
  useEffect(() => {
    if (!sessionId) return
    const controller = new AbortController()
    void fetchRecordingSceneInfo(sessionId, controller.signal).then(info => {
      if (!controller.signal.aborted) setResult({ id: sessionId, info })
    }).catch((cause: unknown) => {
      if (!controller.signal.aborted) setResult({ id: sessionId, error: cause instanceof Error ? cause.message : String(cause) })
    })
    return () => controller.abort()
  }, [sessionId])
  return result.id === sessionId ? result : { id: sessionId }
}

export function useRecordingSceneWindow(sessionId: string, time: number, info: RecordingSceneInfoResponse | undefined, selection: Pick<RecordingSceneQuery, 'cloud_topic' | 'pose_topic' | 'path_topic'>) {
  const cache = useRef(new Map<string, RecordingScene | null>())
  const [result, setResult] = useState<{ key: string; sessionId?: string; roles?: string; scene?: RecordingScene | null; error?: string }>({ key: '' })
  const index = sceneWindowIndex(time)
  const duration = info?.scene?.duration_s ?? 0
  const roles = JSON.stringify(selection)
  const key = JSON.stringify([sessionId, index, roles])
  const hasData = !!info?.scene?.topics.length && time >= 0 && time <= duration
  useEffect(() => {
    if (!hasData || !sessionId) return
    const controller = new AbortController()
    const read = async () => {
      const readWindow = async (windowIndex: number) => {
        const cacheKey = JSON.stringify([sessionId, windowIndex, roles])
        if (cache.current.has(cacheKey)) return cache.current.get(cacheKey)!
        const [start, end] = sceneWindowRange(windowIndex, duration)
        const response = await fetchRecordingScene(sessionId, { ...JSON.parse(roles), start, end }, controller.signal)
        if (controller.signal.aborted) return null
        cache.current.set(cacheKey, response.scene)
        while (cache.current.size > 4) cache.current.delete(cache.current.keys().next().value!)
        return response.scene
      }
      try {
        const scene = await readWindow(index)
        if (controller.signal.aborted) return
        setResult({ key, scene, sessionId, roles })
        if ((index + 1) * SCENE_WINDOW_SECONDS < duration) await readWindow(index + 1)
      } catch (cause: unknown) {
        if (!controller.signal.aborted) setResult({ key, sessionId, roles, error: cause instanceof Error ? cause.message : String(cause) })
      }
    }
    void read()
    return () => controller.abort()
  }, [sessionId, index, duration, roles, key, hasData])
  return result.key === key ? result : { key, scene: result.sessionId === sessionId && result.roles === roles ? result.scene : undefined }
}
