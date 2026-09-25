export interface LocalizationInitialPose {
  x: number
  y: number
  z: number
  yaw: number
}

export type InitialPoseInput = Record<keyof LocalizationInitialPose, string>

export function parseInitialPose(input: InitialPoseInput): LocalizationInitialPose {
  const pose = { x: Number(input.x), y: Number(input.y), z: Number(input.z), yaw: Number(input.yaw) }
  if (Object.values(input).some(value => !value.trim()) || !Object.values(pose).every(Number.isFinite)) {
    throw new Error('请填写有效的 X、Y、Z（米）和航向（弧度）')
  }
  return pose
}
