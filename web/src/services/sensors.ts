import { dashboardFetch } from './observationMode.ts'

export const sensorCards = [
  { id: 'imu', name: 'IMU', detail: '角速度、加速度 · 实时曲线与记录', en: 'Angular velocity, acceleration and records', color: '#5584c7' },
  { id: 'joints', name: '关节', detail: '位置、速度、力矩 · 按实际关节排列', en: 'Joint position, velocity and torque', color: '#8270b8' },
  { id: 'motors', name: '电机与温度', detail: '在线状态、温度、设备错误', en: 'Motor status, temperature and errors', color: '#bd8155' },
  { id: 'voltages', name: '供电电压', detail: '各供电通道与电压趋势', en: 'Channel voltages', color: '#b09a45' },
  { id: 'camera', name: '相机', detail: '实时画面 · 保留最后接收时间', en: 'Live camera and receipt time', color: '#599786' },
  { id: 'lidar', name: 'LiDAR', detail: '三维扫描、坐标系与来源时间', en: '3D scans, reference frame and source time', color: '#548fac' },
  { id: 'gnss', name: 'GNSS 与定位', detail: '经纬度、定位精度与融合状态', en: 'Position, accuracy and fusion', color: '#758e50' },
  { id: 'thermal', name: '热成像', detail: '温度图、像素读数与原始数组', en: 'Temperature image and raw arrays', color: '#bf716c' },
  { id: 'acoustic', name: '声学', detail: '声压波形、采样率与原始数组', en: 'Pressure waveform and sample rate', color: '#9b71a4' },
  { id: 'semantic', name: '语义真值', detail: '来源声明的类别与实例 · 用于仿真检查', en: 'Declared semantic ground truth', color: '#74889c' },
] as const
export type SensorId = typeof sensorCards[number]['id']
export type SensorView = SensorId | 'catalog'
export type NumericSensor = 'joints' | 'motors' | 'voltages'
export interface DisplaySettings { interval: number; window: number; pointSize: number }
export const defaultDisplay: DisplaySettings = { interval: 1000, window: 60, pointSize: 2 }
export function readDisplay(): DisplaySettings {
  try {
    const value = JSON.parse(localStorage.getItem('lingtu.sensor-display') ?? '{}')
    return { interval: [500,1000,2000].includes(value.interval) ? value.interval : 1000,
      window: [15,60,300].includes(value.window) ? value.window : 60,
      pointSize: [1,2,3].includes(value.pointSize) ? value.pointSize : 2 }
  } catch { return { ...defaultDisplay } }
}
export async function sensorRequest(path: string, signal: AbortSignal): Promise<Record<string, unknown>> {
  const response = await dashboardFetch(path, { signal })
  const body = await response.json()
  if (!response.ok || body.success === false) throw new Error(typeof body.detail === 'string' ? body.detail : JSON.stringify(body.detail ?? body.error ?? body))
  return body
}
export function numericFields(snapshot: Record<string, unknown>): Record<string, number> {
  const values = snapshot.values as Record<string, unknown> | undefined
  const result: Record<string, number> = {}
  const finite = (key: string, value: unknown) => { if (typeof value === 'number' && Number.isFinite(value)) result[key] = value }
  if (Array.isArray(values?.volts)) values.volts.forEach((value, index) => finite(`通道 ${index} · V`, value))
  for (const [list, metrics] of [['joints', [['position_rad', '位置', 'rad'], ['velocity_rad_s', '速度', 'rad/s'], ['torque_nm', '力矩', 'N·m']]], ['motors', [['temperature_c', '温度', '°C']]]] as const) {
    if (!Array.isArray(values?.[list])) continue
    for (const row of values[list] as Record<string, unknown>[]) {
      for (const [key, name, unit] of metrics) finite(`${row.name ?? row.id} · ${name} (${unit})`, row[key])
    }
  }
  return result
}
export interface SensorTrace { t: number; receivedAt: number; source: unknown; values: Record<string, number> }
export function appendTrace(rows: SensorTrace[], row: SensorTrace): SensorTrace[] {
  return [...rows.filter(item => row.t - item.t <= 300), row].slice(-1500)
}
export function traceCsv(rows: SensorTrace[]): string {
  const fields = [...new Set(rows.flatMap(row => Object.keys(row.values)))]
  const quote = (value: unknown) => `"${String(value ?? '').replaceAll('"', '""')}"`
  return [['elapsed_s', 'received_at_s', 'source_time', ...fields], ...rows.map(row => [row.t, row.receivedAt, JSON.stringify(row.source), ...fields.map(key => row.values[key])])].map(row => row.map(quote).join(',')).join('\r\n')
}
