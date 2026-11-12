import { useEffect, useEffectEvent, useId, useRef } from 'react'
import { bodyFor, createAvatar, paintOf, renderAvatarSvg, type AvatarController } from '../../vendor/grok-avatar/src/index.mjs'
import type { BotState } from './Bot.tsx'

const nativeStates: Record<BotState, string> = {
  idle: 'idle', processing: 'thinking', working: 'working', writing: 'streaming',
  waiting: 'waiting', done: 'idle', error: 'failed', unavailable: 'sleeping',
}

export interface GrokAvatarProps {
  shape: string
  fill: string
  eyes: string
  state: BotState
  size: number
  animated: boolean
  look: { x: number; y: number }
  greeting: boolean
  className?: string
  expression?: string
  activity?: string
}

export function GrokAvatar({ shape, fill, eyes, state, size, animated, look,
  greeting, className, expression, activity }: GrokAvatarProps) {
  const host = useRef<HTMLSpanElement>(null)
  const controller = useRef<AvatarController | null>(null)
  const previousFace = useRef<string | null>(null)
  const previousState = useRef<string | null>(null)
  const previousGreeting = useRef(false)
  const id = useId()
  const refresh = useEffectEvent(() => {
    if (!host.current) return
    const reduced = animated && window.matchMedia('(prefers-reduced-motion: reduce)').matches
    const playing = animated && !reduced && !document.hidden
    const face = expression ?? (state === 'done' || greeting ? 'happy'
      : state === 'unavailable' ? 'sleepy' : state === 'error' ? 'sad' : 'neutral')
    const heldFace = expression ?? (state === 'done' || greeting ? 'happy' : state === 'error' ? 'sad' : null)
    const options = {
      shape, id, body: bodyFor(shape), appearance: paintOf(fill, eyes),
      state: activity ?? nativeStates[state], size, fill: .8,
      isFollowingPointer: playing, isPlaying: playing, isStatic: !playing,
      maxFrameRate: 30,
    }
    if (!playing) {
      controller.current?.destroy()
      controller.current = null
      previousFace.current = null
      host.current.innerHTML = renderAvatarSvg({ ...options, expression: face })
    } else {
      const created = controller.current === null
      const avatar = controller.current ?? createAvatar(host.current, options)
      controller.current = avatar
      if (!created) avatar.update(options)
      if (created || previousFace.current !== heldFace || previousState.current !== options.state) avatar.holdExpression(heldFace)
      avatar.gaze(look.x === 0 && look.y === 0 ? null : [look.x, look.y])
      if (greeting && !previousGreeting.current) avatar.poke('nod')
      previousFace.current = heldFace
      previousState.current = options.state
    }
    previousGreeting.current = greeting
  })

  useEffect(() => {
    if (!animated) return
    const media = window.matchMedia('(prefers-reduced-motion: reduce)')
    const onChange = () => refresh()
    media.addEventListener('change', onChange)
    document.addEventListener('visibilitychange', onChange)
    return () => {
      media.removeEventListener('change', onChange)
      document.removeEventListener('visibilitychange', onChange)
      controller.current?.destroy()
      controller.current = null
      previousFace.current = null
    }
  }, [animated])

  useEffect(() => {
    refresh()
  }, [shape, fill, eyes, state, size, animated, look.x, look.y, greeting, expression, activity])

  return <span ref={host} className={className} style={{ display: 'inline-block', width: size, height: size }}
    data-state={state} data-size={size} data-animated={animated} data-family="grok" data-shape={shape}
    data-expression={expression} data-activity={activity} aria-hidden="true" />
}
