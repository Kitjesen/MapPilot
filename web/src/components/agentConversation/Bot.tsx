import { useEffect, useRef, type CSSProperties } from 'react'
import { BOT_COLORS, type BotProfile } from '../../services/botProfile.ts'
import { GROK_SHAPES, LINGTU_HEADS } from '../../services/botShapes.ts'
import { startBotBlink } from './botMotion.ts'
import { GrokAvatar } from './GrokAvatar.tsx'
import type { BotActivity, BotExpression } from './botPresentation.ts'
import styles from './Bot.module.css'

export type BotState = 'idle' | 'processing' | 'working' | 'writing' | 'waiting' | 'done' | 'error' | 'unavailable'

interface BotProps {
  state: BotState; shape?: BotProfile['shape']; color?: BotProfile['color']; size?: 32 | 48 | 64; preview?: boolean
  look?: { x: number; y: number }; greeting?: boolean; animated?: boolean
  expression?: BotExpression; activity?: BotActivity
}

export function Bot(props: BotProps) {
  const shape = props.shape ?? 'orb'
  const grok = GROK_SHAPES[shape as keyof typeof GROK_SHAPES]
  if (grok) {
    const palette = BOT_COLORS[props.color ?? 'graphite']
    return <GrokAvatar {...props} shape={grok.shape} fill={palette.fill} eyes={palette.eyes}
      size={(props.size ?? 48) * (props.preview ? 2 : 1)} animated={props.animated ?? true}
      look={props.look ?? { x: 0, y: 0 }} greeting={props.greeting ?? false} className={styles.bot} />
  }
  return <FlatBot {...props} shape={shape as keyof typeof LINGTU_HEADS} />
}

function FlatBot({ state, shape, color = 'graphite', size = 48, preview = false,
  look = { x: 0, y: 0 }, greeting = false, animated = true, expression, activity }: Omit<BotProps, 'shape'> & { shape: keyof typeof LINGTU_HEADS }) {
  const root = useRef<SVGSVGElement>(null)
  const lids = useRef<SVGGElement>(null)
  useEffect(() => {
    if (animated && root.current && lids.current) return startBotBlink(root.current, lids.current)
  }, [animated])
  const palette = BOT_COLORS[color]
  const head: { path: string; eyeX?: number; eyeY?: number } = LINGTU_HEADS[shape]
  const displaySize = size * (preview ? 2 : 1)
  const style = { '--look-x': `${look.x * .6}px`, '--look-y': `${look.y * .5}px`,
    '--look-tilt': `${look.x * 4}deg`,
    '--bot-fill': palette.fill, '--bot-eyes': palette.eyes, width: displaySize, height: displaySize } as CSSProperties
  const face = expression ?? (greeting || state === 'done' ? 'happy' : state === 'error' ? 'sad' : state === 'unavailable' ? 'sleepy' : 'neutral')
  return <svg ref={root} className={`${styles.bot} ${preview ? styles.preview : ''}`} data-state={state} data-color={color}
    data-animated={animated} data-family="lingtu" data-shape={shape} data-expression={face} data-activity={activity}
    data-size={size} data-greeting={greeting || undefined} style={style} viewBox="0 0 16 16" shapeRendering="geometricPrecision"
    aria-hidden="true" focusable="false">
    {activity && ['orbit', 'radar', 'progress', 'uploading'].includes(activity) && <g className={styles.effect}>
      <circle cx="8" cy="8" r="7.5" fill="none" stroke="var(--bot-fill)" strokeWidth=".35" strokeDasharray={activity === 'radar' ? '2 4' : '12 35'} />
    </g>}
    <g className={styles.head}>
      <g className={styles.orientation}><g className={styles.motion}>
      <path className={styles.silhouette} d={head.path} fillRule="evenodd" />
      <g transform={`translate(${head.eyeX ?? 0} ${head.eyeY ?? 0})`}><g className={styles.gaze}><g className={styles.attention}><g ref={lids} className={styles.lids}><g className={styles.eyes}>
        <Eyes expression={face} />
      </g></g></g></g></g>
      </g></g>
    </g>
  </svg>
}

function Eyes({ expression }: { expression: BotExpression }) {
  if (expression === 'happy') return <path d="M4.9 8Q5.9 5.8 6.9 8M9.1 8Q10.1 5.8 11.1 8" fill="none" stroke="var(--bot-eyes)" strokeWidth="1.1" strokeLinecap="round" />
  if (expression === 'sad') return <path d="M5 7Q5.9 8.4 6.8 7M9.2 7Q10.1 8.4 11 7" fill="none" stroke="var(--bot-eyes)" strokeWidth="1.1" strokeLinecap="round" />
  if (expression === 'sleepy' || expression === 'squinted' || expression === 'calm') return <path d={expression === 'squinted' ? 'M5 7.4L6.8 8M9.2 8L11 7.4' : 'M5 8H6.8M9.2 8H11'} stroke="var(--bot-eyes)" strokeWidth={expression === 'calm' ? '.8' : '1.1'} strokeLinecap="round" />
  if (expression === 'surprised') return <><ellipse cx="5.9" cy="7.6" rx="1.1" ry="1.7" /><ellipse cx="10.1" cy="7.6" rx="1.1" ry="1.7" /></>
  return <><rect x="5" y={expression === 'inspecting' ? '5.6' : '6'} width="1.8" height={expression === 'confused' ? '2.2' : '3.2'} rx=".9" /><rect x="9.2" y={expression === 'confused' ? '6.7' : '6'} width="1.8" height="3.2" rx=".9" /></>
}
