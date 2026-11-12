import { useEffect, useState, type CSSProperties, type RefObject } from 'react'

export interface RotatedVideoBox {
  rotation: number
  width: number
  height: number
}

export function normalizeVideoRotation(rotationDegrees: number): number {
  if (!Number.isFinite(rotationDegrees)) return 0
  return ((Math.round(rotationDegrees) % 360) + 360) % 360
}

/** Fits the rotated video bounds inside its container without cropping. */
export function fitRotatedVideo(
  containerWidth: number,
  containerHeight: number,
  videoWidth: number,
  videoHeight: number,
  rotationDegrees: number,
  fit: 'contain' | 'cover' = 'contain',
): RotatedVideoBox {
  const rotation = normalizeVideoRotation(rotationDegrees)
  const quarterTurn = rotation === 90 || rotation === 270
  const sourceAspect = videoWidth > 0 && videoHeight > 0 ? videoWidth / videoHeight : 4 / 3
  const displayAspect = quarterTurn ? 1 / sourceAspect : sourceAspect
  const containerAspect = containerWidth > 0 && containerHeight > 0
    ? containerWidth / containerHeight
    : displayAspect
  const constrainHeight = fit === 'cover' ? containerAspect < displayAspect : containerAspect > displayAspect
  const displayWidth = constrainHeight
    ? containerHeight * displayAspect
    : containerWidth
  const displayHeight = constrainHeight
    ? containerHeight
    : containerWidth / displayAspect
  return {
    rotation,
    width: quarterTurn ? displayHeight : displayWidth,
    height: quarterTurn ? displayWidth : displayHeight,
  }
}

export function useRotatedVideoLayout(
  containerRef: RefObject<HTMLElement | null>,
  videoRef: RefObject<HTMLVideoElement | null>,
  rotationDegrees: number,
  enabled: boolean,
  fit: 'contain' | 'cover' = 'contain',
): { ready: boolean; style: CSSProperties } {
  const rotation = normalizeVideoRotation(rotationDegrees)
  const [box, setBox] = useState<RotatedVideoBox | null>(null)

  useEffect(() => {
    const container = containerRef.current
    const video = videoRef.current
    if (!enabled || !container || !video) return
    let active = true
    const update = () => {
      if (!active) return
      const rect = container.getBoundingClientRect()
      if (rect.width <= 0 || rect.height <= 0) return
      setBox(fitRotatedVideo(
        rect.width,
        rect.height,
        video.videoWidth,
        video.videoHeight,
        rotation,
        fit,
      ))
    }
    const observer = typeof ResizeObserver === 'function' ? new ResizeObserver(update) : null
    observer?.observe(container)
    video.addEventListener('loadedmetadata', update)
    video.addEventListener('resize', update)
    if (!observer) window.addEventListener('resize', update)
    queueMicrotask(update)
    return () => {
      active = false
      observer?.disconnect()
      video.removeEventListener('loadedmetadata', update)
      video.removeEventListener('resize', update)
      if (!observer) window.removeEventListener('resize', update)
    }
  }, [containerRef, enabled, rotation, videoRef, fit])

  const current = box?.rotation === rotation ? box : null
  return {
    ready: current != null,
    style: current
      ? {
          position: 'absolute',
          left: '50%',
          top: '50%',
          width: `${current.width}px`,
          height: `${current.height}px`,
          transform: `translate(-50%, -50%) rotate(${rotation}deg)`,
        }
      : { opacity: 0 },
  }
}
