export const CLOUD_PALETTES = [
  { id: 'height', label: '多色高度', low: '#0529ff', high: '#ff2905' },
  { id: 'blue', label: '青蓝', low: '#2477b8', high: '#54bdad' },
  { id: 'warm', label: '冷暖', low: '#426ab3', high: '#e1a14b' },
  { id: 'theme', label: '柔和 · 随主题', low: '', high: '' },
  { id: 'custom', label: '自定义', low: '', high: '' },
] as const

export interface CloudStyle {
  palette: typeof CLOUD_PALETTES[number]['id']
  lowColor: string
  highColor: string
  pointSize: number
  opacity: number
  background: 'theme' | 'light' | 'dark' | 'custom'
  backgroundColor: string
  grid: boolean
}

export const DEFAULT_CLOUD_STYLE: CloudStyle = {
  palette: 'height', lowColor: '#2477b8', highColor: '#54bdad',
  pointSize: 1.1, opacity: .29,
  background: 'theme', backgroundColor: '#101820', grid: true,
}

const STORAGE_KEY = 'lingtu.map.cloudStyle'

export function readCloudStyle(): CloudStyle {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? 'null')
    const style = { ...DEFAULT_CLOUD_STYLE, ...stored }
    if (style.palette === 'mono') style.palette = 'height'
    delete style.monoColor
    return style
  } catch { return { ...DEFAULT_CLOUD_STYLE } }
}

export function saveCloudStyle(style: CloudStyle) {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(style)) } catch { /* Keep display controls usable without browser storage. */ }
}

export function cloudStyleColors(style: CloudStyle) {
  if (style.palette === 'theme') return null
  const palette = CLOUD_PALETTES.find(option => option.id === style.palette)!
  const rgb = (hex: string) => [1, 3, 5].map(offset => parseInt(hex.slice(offset, offset + 2), 16))
  return {
    lo: rgb(style.palette === 'custom' ? style.lowColor : palette.low),
    hi: rgb(style.palette === 'custom' ? style.highColor : palette.high),
  }
}
