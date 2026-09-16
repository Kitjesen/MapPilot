import { Quaternion } from 'three'

export interface DisplayPose {
  x: number
  y: number
  z: number
  orientation?: [number, number, number, number] | null
  yaw: number
  stampS?: number | null
  epoch?: string | number | null
}

/** Interpolate only between received poses; never integrate a velocity command. */
export class PoseInterpolation {
  private from: DisplayPose | null = null
  private latest: DisplayPose | null = null
  private startedMs = 0
  private durationMs = 0

  update(pose: DisplayPose | null, nowMs: number): void {
    if (!pose || ![pose.x, pose.y, pose.z, pose.yaw].every(Number.isFinite)
      || (pose.orientation != null && (!pose.orientation.every(Number.isFinite) || Math.hypot(...pose.orientation) < 1e-9))) {
      this.from = this.latest = null
      this.durationMs = 0
      return
    }
    const previous = this.latest
    if (previous && pose.epoch === previous.epoch && pose.stampS != null
      && pose.stampS === previous.stampS) return
    const timeStepS = previous && typeof previous.stampS === 'number' && typeof pose.stampS === 'number'
      ? pose.stampS - previous.stampS : 0
    const distance = previous ? Math.hypot(pose.x - previous.x, pose.y - previous.y, pose.z - previous.z) : 0
    const headingChange = previous ? Math.abs(shortestAngle(previous.yaw, pose.yaw)) : 0
    const reset = !previous || pose.epoch !== previous.epoch || timeStepS <= 0 || timeStepS > 0.3
      || distance > 0.5 || headingChange > Math.PI / 2
    this.from = reset ? pose : this.sample(nowMs)
    this.latest = { ...pose }
    this.startedMs = nowMs
    this.durationMs = reset ? 0 : Math.min(120, Math.max(20, timeStepS * 1000))
  }

  sample(nowMs: number): DisplayPose | null {
    if (!this.latest || !this.from) return null
    const t = this.durationMs > 0 ? Math.max(0, Math.min(1, (nowMs - this.startedMs) / this.durationMs)) : 1
    if (t >= 1) return this.latest
    const yaw = this.from.yaw + shortestAngle(this.from.yaw, this.latest.yaw) * t
    return {
      ...this.latest,
      ...(this.from.orientation && this.latest.orientation ? {
        orientation: new Quaternion().fromArray(this.from.orientation).normalize()
          .slerp(new Quaternion().fromArray(this.latest.orientation).normalize(), t).toArray(),
      } : {}),
      x: this.from.x + (this.latest.x - this.from.x) * t,
      y: this.from.y + (this.latest.y - this.from.y) * t,
      z: this.from.z + (this.latest.z - this.from.z) * t,
      yaw: Math.atan2(Math.sin(yaw), Math.cos(yaw)),
    }
  }
}

function shortestAngle(from: number, to: number): number {
  return Math.atan2(Math.sin(to - from), Math.cos(to - from))
}
