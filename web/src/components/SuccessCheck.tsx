import type { CSSProperties } from 'react'
import styles from './SuccessCheck.module.css'

/*!
 * Pixel Icon "check" by HackerNoon — https://github.com/hackernoon/pixel-icon-library
 * SVG source: https://api.iconify.design/pixel/check.svg
 * Licensed under CC BY 4.0 — https://creativecommons.org/licenses/by/4.0/
 * Original path retained. Display color and size adapted for LingTu.
 * Attribution is also distributed in /pixel-icon-NOTICE.txt.
 */
export function PixelCheck({ size = 24, className }: { size?: number; className?: string }) {
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="currentColor" className={className}
    shapeRendering="crispEdges" aria-hidden="true" data-icon="pixel:check" data-pixel-check="">
    <path d="M22 4v2h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1h-1v1H8v-1H7v-1H6v-1H5v-1H4v-1H3v-1H2v-2h2v1h1v1h1v1h1v1h1v1h2v-1h1v-1h1v-1h1v-1h1v-1h1V9h1V8h1V7h1V6h1V5h1V4z" />
  </svg>
}

const bursts = [
  { x: 150, y: 92, delay: 180 },
  { x: 370, y: 88, delay: 240 },
  { x: 260, y: 122, delay: 300 },
]
const colors = ['#22c55e', '#eab308', '#fb7185', '#86efac']

export function SuccessCheck() {
  return <svg className={styles.artwork} viewBox="0 0 520 230" fill="none" aria-hidden="true" data-success-check="">
    {bursts.map(burst => <g key={burst.x} transform={`translate(${burst.x} ${burst.y})`} data-firework="">
      <g className={styles.burst}>
        {Array.from({ length: 16 }, (_, index) => {
          const angle = index * Math.PI / 8
          const distance = 42 + index % 3 * 10
          return <g key={index} className={styles.spark} style={{
            '--x': `${Math.round(Math.sin(angle) * distance / 4) * 4}px`,
            '--y': `${Math.round(-Math.cos(angle) * distance / 4) * 4}px`,
            '--delay': `${burst.delay + index % 3 * 25}ms`,
          } as CSSProperties}>
            <rect x="-2" y="-2" width="4" height="4" fill={colors[index % colors.length]} shapeRendering="crispEdges" />
          </g>
        })}
      </g>
    </g>)}
    <g transform="translate(176 38)" color="#22c55e">
      <g className={styles.check}><PixelCheck size={168} /></g>
    </g>
  </svg>
}
