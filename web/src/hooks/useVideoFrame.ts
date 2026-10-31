import { useEffect, useState, type RefObject } from 'react'

export interface VideoFrameState {
  hasFrame: boolean
  lastFrameAt: number | null
  stalled: boolean
}

export function videoFrameFreshness(
  attachedAt: number,
  lastFrameAt: number | null,
  now: number,
  staleAfterMs: number,
): VideoFrameState {
  if (lastFrameAt == null) {
    return { hasFrame: false, lastFrameAt: null, stalled: now - attachedAt > staleAfterMs }
  }
  const fresh = isFreshVideoFrame(lastFrameAt, now, staleAfterMs)
  return { hasFrame: fresh, lastFrameAt: fresh ? lastFrameAt : null, stalled: !fresh }
}

/** Keeps per-frame evidence local and only publishes meaningful UI changes. */
export class VideoFrameClock {
  private readonly attachedAt: number
  private lastDecodedAt: number | null = null
  private lastPublishedAt: number | null = null
  private publishedHasFrame = false
  private publishedStalled = false

  constructor(attachedAt: number) {
    this.attachedAt = attachedAt
  }

  observe(decodedAt: number): VideoFrameState | null {
    this.lastDecodedAt = decodedAt
    if (this.publishedHasFrame && !this.publishedStalled) return null
    this.publishedHasFrame = true
    this.publishedStalled = false
    this.lastPublishedAt = decodedAt
    return { hasFrame: true, lastFrameAt: decodedAt, stalled: false }
  }

  tick(now: number, staleAfterMs: number): VideoFrameState | null {
    const next = videoFrameFreshness(this.attachedAt, this.lastDecodedAt, now, staleAfterMs)
    if (next.stalled) {
      if (this.publishedStalled) return null
      this.publishedHasFrame = false
      this.publishedStalled = true
      this.lastPublishedAt = null
      return next
    }
    if (!next.hasFrame || next.lastFrameAt === this.lastPublishedAt) return null
    this.publishedHasFrame = true
    this.publishedStalled = false
    this.lastPublishedAt = next.lastFrameAt
    return next
  }
}

export function isFreshVideoFrame(
  lastFrameAt: number | null,
  now: number,
  staleAfterMs: number,
): boolean {
  return lastFrameAt != null && now - lastFrameAt <= staleAfterMs
}

/** Tracks decoded frames; enable after transport connects to start the first-frame deadline. */
export function useVideoFrame(
  videoRef: RefObject<HTMLVideoElement | null>,
  stream: MediaStream | null,
  enabled = true,
  staleAfterMs = 3000,
): VideoFrameState {
  const [frameState, setFrameState] = useState<{
    stream: MediaStream | null
    lastFrameAt: number | null
    stalled: boolean
  }>({ stream: null, lastFrameAt: null, stalled: false })

  useEffect(() => {
    const video = videoRef.current
    if (!video || !stream || !enabled) return

    let active = true
    let callbackId: number | null = null
    const attachedAt = Date.now()
    const clock = new VideoFrameClock(attachedAt)
    const markFrame = () => {
      if (!active) return
      const update = clock.observe(Date.now())
      if (update) setFrameState({ stream, lastFrameAt: update.lastFrameAt, stalled: update.stalled })
    }
    const scheduleFrame = () => {
      callbackId = video.requestVideoFrameCallback(() => {
        markFrame()
        if (active) scheduleFrame()
      })
    }

    video.srcObject = stream
    const hasVideoFrameCallback = typeof video.requestVideoFrameCallback === 'function'
    if (hasVideoFrameCallback) scheduleFrame()
    else {
      video.addEventListener('loadeddata', markFrame)
      video.addEventListener('timeupdate', markFrame)
    }
    void video.play().catch(() => undefined)

    const freshnessTimer = setInterval(() => {
      const update = clock.tick(Date.now(), staleAfterMs)
      if (update) setFrameState({ stream, lastFrameAt: update.lastFrameAt, stalled: update.stalled })
    }, Math.min(1000, Math.max(250, staleAfterMs / 3)))

    return () => {
      active = false
      clearInterval(freshnessTimer)
      if (callbackId != null) video.cancelVideoFrameCallback(callbackId)
      if (!hasVideoFrameCallback) {
        video.removeEventListener('loadeddata', markFrame)
        video.removeEventListener('timeupdate', markFrame)
      }
      if (video.srcObject === stream) video.srcObject = null
    }
  }, [enabled, staleAfterMs, stream, videoRef])

  if (frameState.stream !== stream || !enabled) {
    return { hasFrame: false, lastFrameAt: null, stalled: false }
  }
  return {
    hasFrame: frameState.lastFrameAt != null,
    lastFrameAt: frameState.lastFrameAt,
    stalled: frameState.stalled,
  }
}
