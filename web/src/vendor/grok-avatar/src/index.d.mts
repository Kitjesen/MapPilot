export interface Paint {
  ink: [number, number, number]
  spot: [number, number, number]
  cutout: boolean
  inkPaint: string
  spotPaint?: string
}

export interface AvatarOptions {
  shape?: string
  color?: string
  agentId?: string
  avatarShape?: string | null
  avatarColor?: string | null
  id?: string
  body?: string
  appearance?: Paint
  state?: string
  size?: number
  fill?: number
  resolution?: number
  maxFrameRate?: number
  isPlaying?: boolean
  isStatic?: boolean
  isFollowingPointer?: boolean
  isEmphasized?: boolean
  isProof?: boolean
  expression?: string
  clip?: unknown
  morph?: unknown
  seed?: number | string
  time?: number | null
}

export interface AvatarController {
  update(options: AvatarOptions): void
  setState(state: string): void
  setBody(body: string): void
  setPlaying(playing: boolean): void
  setExpression(name: string, options?: { isThroughBlink?: boolean }): void
  holdExpression(name: string | null): void
  blink(): void
  look(preset: string): void
  restart(): void
  poke(name: 'spin' | 'nod' | 'bounce', options?: { turns?: number }): void
  gaze(point: [number, number] | null): void
  destroy(): void
}

export const SHAPES: readonly string[]
export const BODIES: readonly string[]
export const COLORS: readonly { id: string; label: string; value: string }[]
export const STATES: readonly string[]
export const COMPACT_SIZE: number
export function bodyFor(shape: string): string
export function colorValue(id: string): string | null
export function pinnedColor(shape: string): string | null
export function paintOf(ink: string, spot?: string): Paint
export function resolveAvatar(input: string | { agentId?: string; avatarShape?: string | null; avatarColor?: string | null }): { shape: string; color: string }
export function renderAvatarSvg(options: AvatarOptions): string
export function createAvatar(element: Element, options?: AvatarOptions): AvatarController
export const client: {
  ni: readonly string[]
  lo: Record<string, { face?: string; [key: string]: unknown }>
  [key: string]: unknown
}
