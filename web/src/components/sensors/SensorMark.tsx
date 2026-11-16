import type { CSSProperties } from 'react'
import type { SensorId } from '../../services/sensors.ts'
import { IconGlyph } from '../icons/IconGlyph.tsx'

export function SensorMark({ sensor, size = 30, style }: { sensor: SensorId; size?: number; style?: CSSProperties }) {
  return <IconGlyph name={sensor} size={size} style={style} />
}
