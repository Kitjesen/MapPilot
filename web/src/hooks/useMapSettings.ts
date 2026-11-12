import { createContext, useContext } from 'react'
import type { Bounds } from '../types/stairs.ts'
import type { MapEditScene } from '../types/mapEdit.ts'
import type { SurfaceOptions } from '../services/surfaceMap.ts'

export interface SettingsMap {
  name: string
  contentEpoch: number
  active: boolean
  readOnly: boolean
  onChanged(): void
}

export type MapSettingsFocus = 'cloud' | 'no-go'

export interface AgentMapPreview {
  mapId: string
  contentEpoch: number
  scene: MapEditScene
}

export const MapSettingsContext = createContext<{
  surfaceOptions: SurfaceOptions
  setSurfaceOptions(value: SurfaceOptions): void
  target: HTMLDivElement | null
  setTarget(value: HTMLDivElement | null): void
  selection: Bounds | null
  setSelection(value: Bounds | null): void
  agentPreview: AgentMapPreview | null
  setAgentPreview(value: AgentMapPreview | null): void
  editing: boolean
  setEditing(value: boolean): void
  map: SettingsMap | null
  setMap(value: SettingsMap | null): void
  request: number
  focus: MapSettingsFocus
  open(focus?: MapSettingsFocus): void
} | null>(null)

export function useMapSettings() { return useContext(MapSettingsContext) }
