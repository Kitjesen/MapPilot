import { createContext, useContext } from 'react'
import type { SensorView } from '../services/sensors.ts'

export const SensorMonitorContext = createContext<((view: SensorView, anchor?: HTMLElement) => void) | null>(null)
export function useSensorMonitor() { return useContext(SensorMonitorContext) }
