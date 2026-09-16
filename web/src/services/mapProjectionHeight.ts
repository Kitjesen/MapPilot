import { GO2_URDF } from '../components/scene3d/robot/go2UrdfSpec.ts'
import { GO2_STANDING_JOINTS } from '../components/scene3d/robot/go2Model.ts'

const MAP_PROJECTION_SURFACE_OFFSET_M = 0.012

function jointOriginZ(name: string): number {
  return Math.abs(GO2_URDF.joints.find(joint => joint.name === name)!.origin.xyz[2])
}

const GO2_THIGH_LENGTH_M = jointOriginZ('FL_calf_joint')
const GO2_CALF_LENGTH_M = jointOriginZ('FL_foot_joint')

/** Nominal body-origin-to-foot-center distance; a display reference, not measured ground. */
export const NOMINAL_GO2_FOOT_OFFSET_M =
  GO2_THIGH_LENGTH_M * Math.cos(GO2_STANDING_JOINTS.thigh)
  + GO2_CALF_LENGTH_M * Math.cos(GO2_STANDING_JOINTS.thigh + GO2_STANDING_JOINTS.calf)

/**
 * Place the non-physical 2D observation projection beneath a valid Go2 pose.
 * This changes display depth only; the source grid origin remains authoritative.
 */
export function mapProjectionDisplayZ(
  originZ: number,
  robotZ: number,
  robotModel: 'go2' | 'thunder_v4' | undefined,
  robotValid: boolean,
): number {
  const sourceDisplayZ = originZ + MAP_PROJECTION_SURFACE_OFFSET_M
  if (robotModel !== 'go2' || !robotValid || !Number.isFinite(robotZ)) return sourceDisplayZ
  return Math.min(
    sourceDisplayZ,
    robotZ - NOMINAL_GO2_FOOT_OFFSET_M + MAP_PROJECTION_SURFACE_OFFSET_M,
  )
}
