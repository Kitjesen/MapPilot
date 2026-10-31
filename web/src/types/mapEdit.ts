import type { Bounds } from './stairs.ts'

export type MapEditOperationKind = 'delete_inside' | 'keep_inside'

export interface MapEditOperation {
  operation: MapEditOperationKind
  bounds: Bounds
}

export interface MapEditScene {
  selecting: boolean
  selection: Bounds | null
  view: 'original' | 'draft'
  operations: readonly MapEditOperation[]
  showDeleted: boolean
}

export interface MapEditSource {
  name: string
  contentEpoch: number
  frameId: string
}

export type MapEditView = 'top' | 'front' | 'side' | 'three'
