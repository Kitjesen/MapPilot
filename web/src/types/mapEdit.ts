import type { Bounds } from './stairs.ts'

export type MapCropKind = 'delete_inside' | 'keep_inside'
export interface MapCropOperation { operation: MapCropKind; bounds: Bounds }
export interface MapPlaneOperation { operation: 'add_plane'; bounds: Bounds; axis: 'x' | 'y' | 'z'; offset: number; spacing: number }
export type MapFilterOperation =
  | { operation: 'voxel'; voxel_size: number }
  | { operation: 'statistical'; neighbors: number; std_ratio: number }
  | { operation: 'radius'; radius: number; min_points: number }
  | { operation: 'clusters'; radius: number; neighbors: number; min_cluster_points: number }
  | { operation: 'plane_crop'; distance_threshold: number; margin: number }
export type MapEditOperation = MapCropOperation | MapFilterOperation | MapPlaneOperation
export type MapEditOperationKind = MapEditOperation['operation']

export interface MapEditPreview {
  preview_points: number[]
  preview_removed: number[]
  point_count: number
  removed_points: number
  added_points?: number
  preview_sample: boolean
}

export interface MapEditScene {
  selecting: boolean
  selection: Bounds | null
  view: 'original' | 'draft'
  operations: readonly MapEditOperation[]
  showDeleted: boolean
  preview?: MapEditPreview | null
  surface?: MapPlaneOperation | null
}

export interface MapEditSource {
  name: string
  contentEpoch: number
  frameId: string
}

export type MapEditView = 'top' | 'front' | 'side' | 'three'
