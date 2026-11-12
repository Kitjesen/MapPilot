import { createContext, useContext } from 'react'
export const ImuMonitorContext = createContext<(() => void) | null>(null)
export function useImuMonitor() { return useContext(ImuMonitorContext) }
