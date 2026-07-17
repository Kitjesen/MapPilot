export const THUNDER_V4_DISPLAY_SCALE = 0.75

// Physical envelope at the nominal standing pose.  The browser applies the
// separate display scale above; navigation collision dimensions stay metric.
export const THUNDER_V4_PHYSICAL_SIZE_M = {
  length: 0.86,
  width: 0.58,
  standingHeight: 0.68,
} as const

export const THUNDER_V4_MESH_FILES = [
  'base_link.STL',
  'fr_hip_link.STL', 'fr_thigh_Link.STL', 'fr_calf_Link.STL', 'fr_foot_Link.STL',
  'fl_hip_Link.STL', 'fl_thigh_Link.STL', 'fl_calf_Link.STL', 'fl_foot_Link.STL',
  'rr_hip_Link.STL', 'rr_thigh_Link.STL', 'rr_calf_Link.STL', 'rr_foot_Link.STL',
  'rl_hip_Link.STL', 'rl_thigh_Link.STL', 'rl_calf_Link.STL', 'rl_foot_Link.STL',
  'lidar1_Link.STL', 'camera1_Link.STL', 'lidar2_Link.STL', 'camera2_Link.STL',
] as const
