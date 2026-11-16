import type { CSSProperties } from 'react'
import { IconGlyph } from './icons/IconGlyph.tsx'

/** Small field-console mark; keep it local so the top bar does not depend on an icon catalogue. */
export function LingTuMark({ size = 24, className, style }: { size?: number; className?: string; style?: CSSProperties }) {
  return <IconGlyph name="brand" size={size} className={className} style={style} />
}
