export type GeographicDatum = 'WGS84' | 'GCJ02'
export interface GeographicPoint {
  name: string
  longitude: number
  latitude: number
  datum: GeographicDatum
}

export const GEOGRAPHIC_DRAFT_KEY = 'lingtu.geographic-draft.v1'
export const MAX_GEOGRAPHIC_POINTS = 500

export function geographicPoint(value: unknown): GeographicPoint {
  const p = value as Partial<GeographicPoint> | null
  if (!p || typeof p.name !== 'string' || !p.name.trim() || p.name.length > 80) {
    throw new Error('点位名称需为 1–80 个字符')
  }
  if (typeof p.longitude !== 'number' || !Number.isFinite(p.longitude) || Math.abs(p.longitude) > 180
    || typeof p.latitude !== 'number' || !Number.isFinite(p.latitude) || Math.abs(p.latitude) > 90) {
    throw new Error('经纬度无效：经度 −180～180，纬度 −90～90')
  }
  if (p.datum !== 'WGS84' && p.datum !== 'GCJ02') throw new Error('请明确坐标系：WGS84 或 GCJ02')
  return { name: p.name.trim(), longitude: p.longitude, latitude: p.latitude, datum: p.datum }
}

export function parseGeographicPoints(source: string): GeographicPoint[] {
  if (source.length > 1_000_000) throw new Error('点位文件不能超过 1 MB')
  let data
  try { data = JSON.parse(source) } catch { throw new Error('文件不是有效的 JSON') }
  if (!data || typeof data !== 'object') throw new Error('请选择 GeoJSON 点集或 LingTu 地理点文件')
  let points: unknown[]
  if (data.type === 'FeatureCollection') {
    if (data.crs != null) throw new Error('GeoJSON 只接受标准 WGS84 点集，请先转换坐标系')
    if (!Array.isArray(data.features)) throw new Error('GeoJSON 缺少 features 点集')
    points = data.features.map((feature: { type?: string; geometry?: { type?: string; coordinates?: unknown[] }; properties?: { name?: string }; crs?: unknown }, index: number) => {
      if (!feature || feature.type !== 'Feature' || feature.crs != null || feature.geometry?.type !== 'Point'
        || !Array.isArray(feature.geometry.coordinates)) throw new Error(`第 ${index + 1} 项不是标准 GeoJSON Point`)
      return { name: feature.properties?.name ?? `点位 ${index + 1}`, longitude: feature.geometry.coordinates[0],
        latitude: feature.geometry.coordinates[1], datum: 'WGS84' }
    })
  } else if (data.version === 1 && Array.isArray(data.points)) points = data.points
  else throw new Error('请选择 GeoJSON 点集或 LingTu 地理点文件')
  if (points.length > MAX_GEOGRAPHIC_POINTS) throw new Error('最多导入 500 个点位')
  return points.map(geographicPoint)
}

export function serializeGeographicPoints(points: GeographicPoint[]): string {
  return JSON.stringify({ version: 1, points }, null, 2)
}
