type LngLat = [number, number]
interface MapInstance {
  on(event: 'click', callback: (event: { lnglat: { getLng(): number; getLat(): number } }) => void): void
  add(markers: unknown[]): void
  clearMap(): void
  setFitView(): void
  destroy(): void
}
export interface AMapAPI {
  Map: new (element: HTMLElement, options: { zoom: number; viewMode: '2D' }) => MapInstance
  Marker: new (options: { position: LngLat; title: string; label: { content: string; direction: 'top' } }) => unknown
  convertFrom(points: LngLat[], source: 'gps', callback: (status: string, result: {
    locations?: { getLng(): number; getLat(): number }[]
  }) => void): void
}

declare global {
  interface Window {
    AMap?: AMapAPI
    _AMapSecurityConfig?: { serviceHost: string }
  }
}

export interface AMapConfig { state: string; key: string | null; service_host: string }
export async function fetchAMapConfig(signal: AbortSignal): Promise<AMapConfig> {
  if (import.meta.env.VITE_AMAP_KEY && import.meta.env.VITE_AMAP_SERVICE_HOST) {
    return { state: 'configured', key: import.meta.env.VITE_AMAP_KEY, service_host: import.meta.env.VITE_AMAP_SERVICE_HOST }
  }
  const response = await fetch('/api/v1/geography/map-config', { signal, cache: 'no-store' })
  if (!response.ok) throw new Error('地图配置服务不可用')
  return response.json()
}
let loading: Promise<AMapAPI> | undefined

export function loadAMap(config: AMapConfig): Promise<AMapAPI> {
  if (config.state !== 'configured' || !config.key) return Promise.reject(new Error('高德底图未配置'))
  if (window.AMap) return Promise.resolve(window.AMap)
  if (loading) return loading
  loading = new Promise<AMapAPI>((resolve, reject) => {
    window._AMapSecurityConfig = { serviceHost: new URL(config.service_host, window.location.origin).href }
    const script = document.createElement('script')
    const timeout = window.setTimeout(() => fail(), 12_000)
    function fail() {
      window.clearTimeout(timeout)
      script.remove()
      reject(new Error('高德地图加载失败，请检查网络与部署配置'))
    }
    script.src = `https://webapi.amap.com/maps?v=2.0&key=${encodeURIComponent(config.key!)}`
    script.onerror = fail
    script.onload = () => {
      window.clearTimeout(timeout)
      if (window.AMap) resolve(window.AMap)
      else fail()
    }
    document.head.append(script)
  }).catch(error => { loading = undefined; throw error })
  return loading
}

export async function amapPositions(api: AMapAPI, points: { longitude: number; latitude: number; datum: string }[]): Promise<LngLat[]> {
  const gps = points.filter(point => point.datum === 'WGS84')
  const converted = gps.length ? await new Promise<LngLat[]>((resolve, reject) => {
    const timeout = window.setTimeout(() => reject(new Error('GPS 显示坐标转换超时')), 10_000)
    api.convertFrom(gps.map(point => [point.longitude, point.latitude]), 'gps', (status, result) => {
      window.clearTimeout(timeout)
      if (status !== 'complete' || result.locations?.length !== gps.length) reject(new Error('GPS 显示坐标转换失败'))
      else resolve(result.locations.map(point => [point.getLng(), point.getLat()]))
    })
  }) : []
  let index = 0
  return points.map(point => point.datum === 'WGS84' ? converted[index++] : [point.longitude, point.latitude])
}
