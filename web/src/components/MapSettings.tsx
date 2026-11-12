import { useCallback, useMemo, useState, type ReactNode } from 'react'
import type { Bounds } from '../types/stairs.ts'
import { defaultSurfaceOptions } from '../services/surfaceMap.ts'
import { MapSettingsContext, type MapSettingsFocus, type SettingsMap, type AgentMapPreview } from '../hooks/useMapSettings.ts'

export function MapSettingsProvider({ children }: { children: ReactNode }) {
  const [surfaceOptions, setSurfaceOptions] = useState(defaultSurfaceOptions)
  const [target, setTarget] = useState<HTMLDivElement | null>(null)
  const [selection, setSelection] = useState<Bounds | null>(null)
  const [agentPreview, setAgentPreview] = useState<AgentMapPreview | null>(null)
  const [editing, setEditing] = useState(false)
  const [map, setMap] = useState<SettingsMap | null>(null)
  const [request, setRequest] = useState(0)
  const [focus, setFocus] = useState<MapSettingsFocus>('cloud')
  const open = useCallback((next: MapSettingsFocus = 'cloud') => {
    setFocus(next)
    setRequest(value => value + 1)
  }, [])
  const value = useMemo(() => ({ selection, setSelection, agentPreview, setAgentPreview, surfaceOptions, setSurfaceOptions, target, setTarget, editing, setEditing, map, setMap, request, focus, open }), [selection, agentPreview, surfaceOptions, target, editing, map, request, focus, open])
  return <MapSettingsContext.Provider value={value}>{children}</MapSettingsContext.Provider>
}
