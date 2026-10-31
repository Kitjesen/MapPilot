export interface XYZ { x: number; y: number; z: number }
export interface Bounds { min: XYZ; max: XYZ }

/** Entrance coordinates describe observed landing surfaces, not body goals. */
export interface StairAnnotation {
  id: string
  name: string
  frame_id: string
  content_epoch: number
  bounds: Bounds
  lower: XYZ
  upper: XYZ
  geometry?: {
    recognized: boolean
    reason: string
    width_m?: number
    mean_rise_m?: number
    mean_run_m?: number
    surfaces?: { s_min: number; s_max: number; lateral_min: number; lateral_max: number; surface_z: number; observed_points: number }[]
  } | null
}

export interface StairDraft {
  bounds?: Bounds
  lower?: XYZ
  upper?: XYZ
}
export type StairTool = 'bounds' | 'lower' | 'upper' | null
export interface StairScene {
  annotations: StairAnnotation[]
  draft?: StairDraft
  tool: StairTool
}
export interface StairListResponse {
  success: boolean
  map_id: string
  content_epoch: number
  stairs: StairAnnotation[]
}
