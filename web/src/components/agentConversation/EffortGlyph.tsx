// Adapted from T3 Code e89abadaee70e0d73fb19a5ce40a686b1a92310e.
// apps/web/src/components/chat/EffortGlyph.tsx. See LICENSE.t3code.txt.

function effortBarHeights(count: number, maxBar: number): number[] {
  const step = Math.max(1, Math.round(maxBar / 2 / Math.max(count - 1, 1)))
  return Array.from({ length: count }, (_, index) => maxBar - (count - 1 - index) * step)
}

export function EffortGlyph({ count, filled }: { count: number; filled: number }) {
  const height = 14
  const step = 4
  const width = step * count - 2
  const maxBar = height - 2
  return <svg aria-hidden="true" width={width} height={height} viewBox={`0 0 ${width} ${height}`}
    style={{ width, height, flexShrink: 0 }} data-composer-effort-glyph>
    {effortBarHeights(count, maxBar).map((barHeight, index) => <rect key={index * step}
      x={index * step} y={maxBar - barHeight} width={2} height={barHeight}
      fill="currentColor" opacity={index < filled ? 1 : 0.4} />)}
  </svg>
}
