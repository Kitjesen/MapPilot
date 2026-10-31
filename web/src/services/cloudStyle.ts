import { PCD_POINT_SIZE_CSS } from './pcdPicking.ts'

export const CLOUD_PALETTES = [
  { id: 'blue', label: '青蓝', low: '#2477b8', high: '#54bdad' },
  { id: 'warm', label: '冷暖', low: '#426ab3', high: '#e1a14b' },
  { id: 'mono', label: '单色', low: '#6e95ab', high: '#6e95ab' },
  { id: 'theme', label: '柔和 · 随主题', low: '', high: '' },
  { id: 'custom', label: '自定义', low: '', high: '' },
] as const

export interface CloudStyle {
  palette: typeof CLOUD_PALETTES[number]['id']
  lowColor: string
  highColor: string
  monoColor: string
  pointSize: number
  opacity: number
  background: 'theme' | 'light' | 'dark' | 'custom'
  backgroundColor: string
  grid: boolean
}

export const DEFAULT_CLOUD_STYLE: CloudStyle = {
  palette: 'blue', lowColor: '#2477b8', highColor: '#54bdad', monoColor: '#6e95ab',
  pointSize: PCD_POINT_SIZE_CSS, opacity: 1,
  background: 'theme', backgroundColor: '#101820', grid: true,
}

const STORAGE_KEY = 'lingtu.map.cloudStyle'

export function readCloudStyle(): CloudStyle {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? 'null')
    return { ...DEFAULT_CLOUD_STYLE, ...stored }
  } catch { return { ...DEFAULT_CLOUD_STYLE } }
}

export function saveCloudStyle(style: CloudStyle) {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(style)) } catch { /* Keep display controls usable without browser storage. */ }
}

export function cloudStyleColors(style: CloudStyle) {
  if (style.palette === 'theme') return null
  const palette = CLOUD_PALETTES.find(option => option.id === style.palette)!
  const rgb = (hex: string) => [1, 3, 5].map(offset => parseInt(hex.slice(offset, offset + 2), 16))
  if (style.palette === 'mono') {
    const color = rgb(style.monoColor)
    return { lo: color, hi: color }
  }
  return {
    lo: rgb(style.palette === 'custom' ? style.lowColor : palette.low),
    hi: rgb(style.palette === 'custom' ? style.highColor : palette.high),
  }
}
