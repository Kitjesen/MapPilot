import type { RecordingCameraSegment, RecordingCloudView, RecordingPoseSample, RecordingScene, RecordingSceneStamp } from '../types/recordingScene.ts'

export const SCENE_WINDOW_SECONDS = 5
export const CAMERA_PREVIEW_HZ = 5

export function cameraCursorTime(sharedTime: number, offset: number, playing: boolean): number {
  return (playing ? Math.floor(sharedTime * CAMERA_PREVIEW_HZ) / CAMERA_PREVIEW_HZ : sharedTime) - offset
}

export function sceneWindowIndex(time: number): number { return Math.floor(Math.max(0, time) / SCENE_WINDOW_SECONDS) }

export function sceneWindowRange(index: number, duration: number): [number, number] {
  const start = Math.min(Math.max(0, index) * SCENE_WINDOW_SECONDS, duration)
  return [Math.max(0, start - 1), Math.max(0, Math.min(duration, start + SCENE_WINDOW_SECONDS))]
}

export function sceneFrameAt<T extends RecordingSceneStamp>(frames: T[], time: number): T | undefined {
  let low = 0; let high = frames.length
  while (low < high) { const middle = (low + high) >>> 1; if (frames[middle].t <= time) low = middle + 1; else high = middle }
  return frames[low - 1]
}

export function cameraSegmentAt(segments: RecordingCameraSegment[], time: number, cameraId?: string): RecordingCameraSegment | undefined {
  let selected: RecordingCameraSegment | undefined
  for (const segment of segments) {
    if ((!cameraId || segment.camera_id === cameraId) && segment.start_s <= time && (segment.coverage_end_s === undefined ? time <= segment.end_s : time < segment.coverage_end_s)
      && (!selected || segment.start_s > selected.start_s)) selected = segment
  }
  return selected
}

function linePairs(points: number[], dashed = false): Float32Array {
  const pairs: number[] = []
  for (let index = 3; index < points.length; index += 3) {
    if (dashed && (index / 3) % 2 === 0) continue
    pairs.push(...points.slice(index - 3, index + 3))
  }
  return new Float32Array(pairs)
}

export function pastPoseSegments(samples: RecordingPoseSample[], time: number, frameId: string): Float32Array {
  const pairs: number[] = []
  let previous: RecordingPoseSample | undefined
  for (const sample of samples) {
    if (sample.t > time) break
    if (sample.frame_id !== frameId || sample.gap_before) previous = undefined
    if (sample.frame_id === frameId) {
      if (previous) pairs.push(...previous.position, ...sample.position)
      previous = sample
    }
  }
  return new Float32Array(pairs)
}

export function recordingCloudView(scene: RecordingScene | null | undefined, time: number, frameId: string): RecordingCloudView {
  const cloud = sceneFrameAt(scene?.cloud?.frames ?? [], time)
  const lines: RecordingCloudView['lines'] = []
  if (scene?.trajectory && frameId) {
    const points = pastPoseSegments(scene.trajectory.samples, time, frameId)
    if (points.length) lines.push({ points, kind: 'trajectory' })
  }
  for (const path of scene?.paths ?? []) {
    const latest = sceneFrameAt(path.frames, time)
    if (frameId && latest?.frame_id === frameId && latest.points.length) lines.push({ points: linePairs(latest.points, true), kind: 'path' })
  }
  return { points: new Float32Array(cloud?.frame_id === frameId ? cloud.points : []), frameId, originalCount: cloud?.frame_id === frameId ? cloud.original_points : null, lines }
}

export function sceneFrames(scene: RecordingScene | null | undefined, time: number): string[] {
  const frames = new Set<string>()
  const cloud = sceneFrameAt(scene?.cloud?.frames ?? [], time)
  if (cloud) frames.add(cloud.frame_id)
  const pose = sceneFrameAt(scene?.trajectory?.samples ?? [], time)
  if (pose) frames.add(pose.frame_id)
  for (const path of scene?.paths ?? []) { const frame = sceneFrameAt(path.frames, time); if (frame) frames.add(frame.frame_id) }
  return [...frames]
}
