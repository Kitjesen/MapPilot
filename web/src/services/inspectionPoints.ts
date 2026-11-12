import type { InspectionRoutePoint, LocationEntry } from '../types/index.ts'

export const inspectionActions = [
  ['', '仅停留'], ['capture:overview', '拍摄环境总览'], ['capture:parking', '停车区域巡检'],
  ['capture:bin_full', '检查料箱'], ['capture:plate_ocr', '识别标识'],
  ['capture:indicator_on', '检查指示灯亮'], ['capture:indicator_off', '检查指示灯灭'],
  ['capture:indicator_green', '检查绿灯'], ['capture:indicator_red', '检查红灯'],
  ['capture:indicator_yellow', '检查黄灯'], ['capture:door_open', '检查柜门打开'],
  ['capture:door_closed', '检查柜门关闭'],
] as const

export function routePointFromLocation(location: LocationEntry): InspectionRoutePoint {
  const settings = location.metadata?.inspection as { dwell?: number; action?: string } | undefined
  return { id: location.name, x: location.x, y: location.y, z: location.z ?? 0, yaw: location.yaw ?? null,
    tolerance: .35, dwell: settings?.dwell ?? 3, action: settings?.action ?? 'capture:overview', enabled: true }
}

export function findInspectionPoints(points: LocationEntry[], query: string): LocationEntry[] {
  const text = query.trim().toLocaleLowerCase()
  return points.filter(point => `${point.name} ${point.metadata?.region ?? ''} ${point.tags.join(' ')}`.toLocaleLowerCase().includes(text))
}
