import { useEffect, useState, type RefObject } from 'react'

/** Return the browser's observed decoded-frame rate, averaged over one second. */
export function useVideoDecodeRate(
  videoRef: RefObject<HTMLVideoElement | null>,
  stream: MediaStream | null,
  enabled = true,
): number | null {
  const [sample, setSample] = useState<{ stream: MediaStream; fps: number } | null>(null)

  useEffect(() => {
    const video = videoRef.current
    if (!video || !stream || !enabled) {
      return
    }

    let active = true
    let callbackId: number | null = null
    let windowStart = performance.now()
    let frameCount = 0

    const observe = (now: number) => {
      if (!active) return
      frameCount += 1
      const elapsed = now - windowStart
      if (elapsed >= 1000) {
        setSample({ stream, fps: (frameCount * 1000) / elapsed })
        frameCount = 0
        windowStart = now
      }
      callbackId = video.requestVideoFrameCallback(observe)
    }

    video.srcObject = stream
    if (typeof video.requestVideoFrameCallback !== 'function') {
      return () => {
        active = false
        if (video.srcObject === stream) video.srcObject = null
      }
    }

    callbackId = video.requestVideoFrameCallback(observe)
    return () => {
      active = false
      if (callbackId !== null) video.cancelVideoFrameCallback(callbackId)
      if (video.srcObject === stream) video.srcObject = null
    }
  }, [enabled, stream, videoRef])

  return enabled && sample?.stream === stream ? sample.fps : null
}
